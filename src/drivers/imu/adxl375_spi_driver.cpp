/**
 * @file  adxl375_spi_driver.cpp
 * @brief ADXL375 SPI high-g accelerometer driver implementation.
 */

#include "drivers/imu/adxl375_spi_driver.h"
#include "config.h"
#include "debug/ares_log.h"
#include "rtos_guard.h"  // ScopedLock — CERT-18.1

#include <Arduino.h>
#include <cmath>
#include <freertos/task.h>

// ── ADXL375 register map ────────────────────────────────────
// (shared definitions — identical to the I2C driver's namespace)
namespace adxl375spi
{
    constexpr uint8_t REG_DEVID        = 0x00U;
    constexpr uint8_t REG_BW_RATE      = 0x2CU;
    constexpr uint8_t REG_POWER_CTL    = 0x2DU;
    constexpr uint8_t REG_DATA_FORMAT  = 0x31U;
    constexpr uint8_t REG_DATAX0       = 0x32U;
    constexpr uint8_t REG_FIFO_CTL     = 0x38U;

    constexpr uint8_t DEVID_VALUE          = 0xE5U;
    constexpr uint8_t DATA_FORMAT_FULLRES  = 0x0BU; ///< FULL_RES=1, RANGE=11 (±200 g).
    constexpr uint8_t BW_RATE_100HZ        = 0x0AU; ///< 100 Hz ODR, normal power.
    constexpr uint8_t POWER_CTL_STANDBY    = 0x00U; ///< Standby mode (safe for config writes).
    constexpr uint8_t POWER_CTL_MEASURE    = 0x08U; ///< Start continuous measurement.
    constexpr uint8_t FIFO_BYPASS          = 0x00U; ///< No FIFO buffering.
    constexpr uint8_t BURST_LEN            = 6U;    ///< Bytes in one XYZ burst read.

    // SPI command byte masks (ADXL375 datasheet §SPI Protocol).
    constexpr uint8_t SPI_READ            = 0x80U;  ///< bit7=1 → read.
    constexpr uint8_t SPI_WRITE           = 0x00U;  ///< bit7=0 → write.
    constexpr uint8_t SPI_MULTI_BYTE      = 0x40U;  ///< bit6=1 → multi-byte transfer.
    constexpr uint8_t SPI_ADDR_MASK       = 0x3FU;  ///< bits[5:0] = register address.

    // Scale factor: 49 mg/LSB (fixed, ADXL375 full-resolution mode).
    constexpr float ACCEL_MG_PER_LSB = 49.0e-3f;
    constexpr float GRAVITY_MS2      = 9.80665f;
}

static constexpr const char* TAG = "IMU";

// ── Constructor ─────────────────────────────────────────────

Adxl375SpiDriver::Adxl375SpiDriver(SPIClass& spi, uint8_t csPin, uint32_t freqHz)
    : spi_(spi)
    , csPin_(csPin)
    , freqHz_(freqHz)
{
}

// ── Public ──────────────────────────────────────────────────

bool Adxl375SpiDriver::begin() // NOLINT(readability-function-size)
{
    // CERT-13: create the driver mutex on first begin() call.
    if (imuMutex_ == nullptr)
    {
        imuMutex_ = xSemaphoreCreateMutex();
        if (imuMutex_ == nullptr)
        {
            LOG_E(TAG, "ADXL375_SPI: mutex create failed");
            return false;
        }
    }

    ready_ = false;

    // Configure chip-select: start idle-high (deasserted).
    pinMode(csPin_, OUTPUT);
    digitalWrite(csPin_, HIGH);

    // Step 1: Verify device identity.
    uint8_t idBuf[1] = {};
    if (!readRegs(adxl375spi::REG_DEVID, idBuf, 1U))
    {
        LOG_E(TAG, "ADXL375_SPI: DEVID read failed (CS=%u)", static_cast<unsigned>(csPin_));
        return false;
    }
    if (idBuf[0] != adxl375spi::DEVID_VALUE)
    {
        LOG_E(TAG, "ADXL375_SPI: unexpected DEVID 0x%02X (expected 0x%02X)",
              idBuf[0], adxl375spi::DEVID_VALUE);
        return false;
    }
    LOG_I(TAG, "ADXL375_SPI: DEVID OK (0xE5) — CS=%u, %lu Hz",
          static_cast<unsigned>(csPin_), static_cast<unsigned long>(freqHz_));

    // Step 2: Standby mode before changing configuration.
    // cppcheck-suppress knownConditionTrueFalse
    if (!writeReg(adxl375spi::REG_POWER_CTL, adxl375spi::POWER_CTL_STANDBY))
    {
        LOG_E(TAG, "ADXL375_SPI: POWER_CTL standby write failed");
        return false;
    }

    // Step 3: Full-resolution, ±200 g format.
    // cppcheck-suppress knownConditionTrueFalse
    if (!writeReg(adxl375spi::REG_DATA_FORMAT, adxl375spi::DATA_FORMAT_FULLRES))
    {
        LOG_E(TAG, "ADXL375_SPI: DATA_FORMAT write failed");
        return false;
    }

    // Step 4: 100 Hz ODR, normal power.
    // cppcheck-suppress knownConditionTrueFalse
    if (!writeReg(adxl375spi::REG_BW_RATE, adxl375spi::BW_RATE_100HZ))
    {
        LOG_E(TAG, "ADXL375_SPI: BW_RATE write failed");
        return false;
    }

    // Step 5: FIFO bypass — read latest sample directly from output registers.
    // cppcheck-suppress knownConditionTrueFalse
    if (!writeReg(adxl375spi::REG_FIFO_CTL, adxl375spi::FIFO_BYPASS))
    {
        LOG_E(TAG, "ADXL375_SPI: FIFO_CTL write failed");
        return false;
    }

    // Step 6: Enter measurement mode.
    // cppcheck-suppress knownConditionTrueFalse
    if (!writeReg(adxl375spi::REG_POWER_CTL, adxl375spi::POWER_CTL_MEASURE))
    {
        LOG_E(TAG, "ADXL375_SPI: POWER_CTL measure write failed");
        return false;
    }

    // Brief settle after measurement mode enable.
    // RTOS-1: deviation — vTaskDelay required after POWER_CTL transition.
    vTaskDelay(pdMS_TO_TICKS(ares::ADXL375_SETTLE_MS));

    ready_ = true;
    LOG_I(TAG, "ADXL375_SPI: init OK — 100 Hz, ±200g, SPI Mode3, CS=%u",
          static_cast<unsigned>(csPin_));
    return true;
}

ImuStatus Adxl375SpiDriver::read(ImuReading& out)
{
    // CERT-13/CERT-18.1: serialise concurrent calls using RAII guard.
    if (imuMutex_ == nullptr) { return ImuStatus::NOT_READY; }
    ScopedLock lk(imuMutex_, pdMS_TO_TICKS(ares::IMU_LOCK_TIMEOUT_MS));
    if (!lk.acquired()) { return ImuStatus::NOT_READY; }
    const ImuStatus result = readLocked(out);
    return result;
}

// ── Private ─────────────────────────────────────────────────

ImuStatus Adxl375SpiDriver::readLocked(ImuReading& out)
{
    if (!ready_)
    {
        // Lazy re-init: retry begin() at most once every IMU_REINIT_INTERVAL_MS.
        const uint32_t now = millis();
        if ((now - lastReinitAttemptMs_) >= ares::IMU_REINIT_INTERVAL_MS)
        {
            lastReinitAttemptMs_ = now;
            LOG_W(TAG, "ADXL375_SPI: not ready — attempting lazy re-init");
            if (!begin())
            {
                return ImuStatus::NOT_READY;
            }
        }
        else
        {
            return ImuStatus::NOT_READY;
        }
    }

    // Burst read: DATAX0..DATAZ1 (6 bytes, multi-byte SPI frame).
    uint8_t buf[adxl375spi::BURST_LEN] = {};  // MISRA-4.1: zero-init
    if (!readRegs(adxl375spi::REG_DATAX0, buf, adxl375spi::BURST_LEN))
    {
        consecutiveErrors_++;
        if (consecutiveErrors_ >= ares::IMU_MAX_CONSECUTIVE_ERRORS)
        {
            LOG_W(TAG, "ADXL375_SPI: read failed %u times — marking not-ready, retry in %lu ms",
                  static_cast<unsigned>(consecutiveErrors_),
                  static_cast<unsigned long>(ares::IMU_REINIT_INTERVAL_MS));
            ready_             = false;
            consecutiveErrors_ = 0U;
        }
        return ImuStatus::ERROR;
    }

    // ADXL375 data format: little-endian (LSB first), signed 16-bit per axis.
    // Burst layout: X0, X1, Y0, Y1, Z0, Z1  (datasheet §Output Data Registers).
    auto toInt16 = [](uint8_t lo, uint8_t hi) -> int16_t
    {
        return static_cast<int16_t>(
            (static_cast<uint16_t>(hi) << 8U) | static_cast<uint16_t>(lo));
    };

    const int16_t rawAx = toInt16(buf[0], buf[1]);
    const int16_t rawAy = toInt16(buf[2], buf[3]);
    const int16_t rawAz = toInt16(buf[4], buf[5]);

    const float scale = adxl375spi::ACCEL_MG_PER_LSB * adxl375spi::GRAVITY_MS2;
    out.accelX = static_cast<float>(rawAx) * scale;
    out.accelY = static_cast<float>(rawAy) * scale;
    out.accelZ = static_cast<float>(rawAz) * scale;

    // No gyroscope or temperature sensor on ADXL375.
    out.gyroX = 0.0f;
    out.gyroY = 0.0f;
    out.gyroZ = 0.0f;
    out.tempC = 0.0f;

    // Guard against NaN/Inf (CERT-16.2).
    if (!isfinite(out.accelX) || !isfinite(out.accelY) || !isfinite(out.accelZ))
    {
        return ImuStatus::ERROR;
    }

    consecutiveErrors_ = 0U;
    return ImuStatus::OK;
}

bool Adxl375SpiDriver::writeReg(uint8_t reg, uint8_t value)
{
    // Write frame: R/W̄=0, MB=0, addr[5:0]. SPI_WRITE=0x00 is intentional documentation.
    // cppcheck-suppress badBitmaskCheck
    const uint8_t cmd = adxl375spi::SPI_WRITE | (reg & adxl375spi::SPI_ADDR_MASK);

    spi_.beginTransaction(SPISettings(freqHz_, MSBFIRST, SPI_MODE3));
    digitalWrite(csPin_, LOW);
    spi_.transfer(cmd);
    spi_.transfer(value);
    digitalWrite(csPin_, HIGH);
    spi_.endTransaction();
    return true;
}

bool Adxl375SpiDriver::readRegs(uint8_t reg, uint8_t* buf, uint8_t len)
{
    if (buf == nullptr || len == 0U)
    {
        return false;
    }

    // Read frame: R/W̄=1, MB=1 for multi-byte, addr[5:0].
    const uint8_t mbFlag = (len > 1U) ? adxl375spi::SPI_MULTI_BYTE : 0x00U;
    const uint8_t cmd    = adxl375spi::SPI_READ
                         | mbFlag
                         | (reg & adxl375spi::SPI_ADDR_MASK);

    spi_.beginTransaction(SPISettings(freqHz_, MSBFIRST, SPI_MODE3));
    digitalWrite(csPin_, LOW);
    spi_.transfer(cmd);
    for (uint8_t i = 0U; i < len; i++)
    {
        buf[i] = static_cast<uint8_t>(spi_.transfer(0x00U));
    }
    digitalWrite(csPin_, HIGH);
    spi_.endTransaction();
    return true;
}

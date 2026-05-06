/**
 * @file  adxl375_driver.cpp
 * @brief ADXL375 I2C high-g accelerometer driver implementation.
 */

#include "drivers/imu/adxl375_driver.h"
#include "config.h"
#include "debug/ares_log.h"
#include "rtos_guard.h"  // ScopedLock — CERT-18.1

#include <cmath>
#include <freertos/task.h>

// ── ADXL375 register map (Analog Devices Rev E datasheet) ───
namespace adxl375
{
    constexpr uint8_t REG_DEVID        = 0x00;  ///< Device ID (read-only, always 0xE5).
    constexpr uint8_t REG_BW_RATE      = 0x2C;  ///< Output data rate and power mode.
    constexpr uint8_t REG_POWER_CTL    = 0x2D;  ///< Power control.
    constexpr uint8_t REG_DATA_FORMAT  = 0x31;  ///< Data format / range bits.
    constexpr uint8_t REG_DATAX0       = 0x32;  ///< Burst origin: X0,X1,Y0,Y1,Z0,Z1.
    constexpr uint8_t REG_FIFO_CTL     = 0x38;  ///< FIFO control.

    constexpr uint8_t DEVID_VALUE      = 0xE5;  ///< Expected DEVID value.

    constexpr uint8_t ADDR_LOW         = 0x53;  ///< ALT_ADDRESS/CS = GND (default).
    constexpr uint8_t ADDR_HIGH        = 0x1D;  ///< ALT_ADDRESS/CS = VCC.

    // DATA_FORMAT: FULL_RES=1 (bit 3), RANGE=11 (bits[1:0]) → ±200 g full resolution.
    constexpr uint8_t DATA_FORMAT_FULLRES = 0x0BU;

    // BW_RATE: D3:D0 = 0xA → 100 Hz ODR, normal power (datasheet Table 6).
    constexpr uint8_t BW_RATE_100HZ    = 0x0AU;

    // POWER_CTL: bit 3 (Measure) = 1 → start continuous measurement.
    constexpr uint8_t POWER_CTL_MEASURE = 0x08U;

    // FIFO_CTL: bypass mode (no FIFO) → 0x00.
    constexpr uint8_t FIFO_BYPASS      = 0x00U;

    constexpr uint8_t BURST_LEN        = 6U;    ///< Bytes in one XYZ burst read.

    // Scale factor: 49 mg/LSB (fixed, independent of range bits on ADXL375).
    // Converted to m/s²: 49e-3 g/LSB * 9.80665 m/s²/g = 0.48053... m/s²/LSB.
    constexpr float ACCEL_MG_PER_LSB   = 49.0e-3f;  ///< g per LSB.
    constexpr float GRAVITY_MS2        = 9.80665f;   ///< Standard gravity (MISRA-7).
}

// ── Module-local constants ──────────────────────────────────
static constexpr const char* TAG = "IMU";  // MISRA-7: named constant

// ── Constructor ─────────────────────────────────────────────

Adxl375Driver::Adxl375Driver(TwoWire& wire, uint8_t addr)
    : wire_(wire)
    , addr_(addr)
{
}

// ── Public ──────────────────────────────────────────────────

bool Adxl375Driver::begin()
{
    // CERT-13: create the driver mutex on first begin() call.
    if (imuMutex_ == nullptr)
    {
        imuMutex_ = xSemaphoreCreateMutex();
        if (imuMutex_ == nullptr)
        {
            LOG_E(TAG, "ADXL375: mutex create failed");
            return false;
        }
    }

    ready_ = false;

    auto tryInitAtAddress = [&](uint8_t addr) -> bool
    {
        addr_ = addr;

        // Step 1: Verify device identity.
        uint8_t idBuf[1] = {};
        if (!readRegs(adxl375::REG_DEVID, idBuf, 1U))
        {
            LOG_E(TAG, "ADXL375: DEVID read failed at 0x%02X", addr);
            return false;
        }
        if (idBuf[0] != adxl375::DEVID_VALUE)
        {
            LOG_E(TAG, "ADXL375: unexpected DEVID 0x%02X at 0x%02X (expected 0x%02X)",
                  idBuf[0], addr, adxl375::DEVID_VALUE);
            return false;
        }
        LOG_I(TAG, "ADXL375: DEVID OK (0xE5) at addr 0x%02X", addr);

        // Step 2: Standby mode before changing configuration.
        // Writing 0x00 to POWER_CTL ensures the device is in standby so
        // DATA_FORMAT and BW_RATE writes take effect cleanly (datasheet §Serial Comm.).
        if (!writeReg(adxl375::REG_POWER_CTL, 0x00U))
        {
            LOG_E(TAG, "ADXL375: POWER_CTL standby write failed");
            return false;
        }

        // Step 3: Set full-resolution, ±200 g format.
        if (!writeReg(adxl375::REG_DATA_FORMAT, adxl375::DATA_FORMAT_FULLRES))
        {
            LOG_E(TAG, "ADXL375: DATA_FORMAT write failed");
            return false;
        }

        // Step 4: 100 Hz output data rate, normal power.
        if (!writeReg(adxl375::REG_BW_RATE, adxl375::BW_RATE_100HZ))
        {
            LOG_E(TAG, "ADXL375: BW_RATE write failed");
            return false;
        }

        // Step 5: FIFO bypass (no buffering — read latest sample directly).
        if (!writeReg(adxl375::REG_FIFO_CTL, adxl375::FIFO_BYPASS))
        {
            LOG_E(TAG, "ADXL375: FIFO_CTL write failed");
            return false;
        }

        // Step 6: Enter measurement mode.
        if (!writeReg(adxl375::REG_POWER_CTL, adxl375::POWER_CTL_MEASURE))
        {
            LOG_E(TAG, "ADXL375: POWER_CTL measure write failed");
            return false;
        }

        // Brief settle after measurement mode enable (I2C clock stabilisation).
        // RTOS-1: deviation — vTaskDelay required after POWER_CTL transition.
        vTaskDelay(pdMS_TO_TICKS(ares::ADXL375_SETTLE_MS));

        return true;
    };

    // Try configured address first, then the alternate.
    if (!tryInitAtAddress(addr_))
    {
        const uint8_t altAddr = (addr_ == adxl375::ADDR_LOW)
                              ? adxl375::ADDR_HIGH
                              : adxl375::ADDR_LOW;
        LOG_W(TAG, "ADXL375: retrying at alternate address 0x%02X", altAddr);
        if (!tryInitAtAddress(altAddr))
        {
            LOG_E(TAG, "ADXL375: init FAILED — not found on I2C1 (SDA=%d SCL=%d)",
                  ares::PIN_IMU_SDA, ares::PIN_IMU_SCL);
            return false;
        }
    }

    ready_ = true;
    LOG_I(TAG, "ADXL375: init OK — addr 0x%02X, 100 Hz, ±200g, I2C1 SDA=%d SCL=%d",
          addr_, ares::PIN_IMU_SDA, ares::PIN_IMU_SCL);
    return true;
}

ImuStatus Adxl375Driver::read(ImuReading& out)
{
    // CERT-13/CERT-18.1: serialise concurrent calls using RAII guard.
    if (imuMutex_ == nullptr) { return ImuStatus::NOT_READY; }
    ScopedLock lk(imuMutex_, pdMS_TO_TICKS(ares::IMU_LOCK_TIMEOUT_MS));
    if (!lk.acquired()) { return ImuStatus::NOT_READY; }
    const ImuStatus result = readLocked(out);
    return result;
}

// ── Private ─────────────────────────────────────────────────

ImuStatus Adxl375Driver::readLocked(ImuReading& out)
{
    if (!ready_)
    {
        // Lazy re-init: retry begin() at most once every IMU_REINIT_INTERVAL_MS.
        const uint32_t now = millis();
        if ((now - lastReinitAttemptMs_) >= ares::IMU_REINIT_INTERVAL_MS)
        {
            lastReinitAttemptMs_ = now;
            LOG_W(TAG, "ADXL375: not ready — attempting lazy re-init");
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

    // Burst read: DATAX0, DATAX1, DATAY0, DATAY1, DATAZ0, DATAZ1 (6 bytes).
    uint8_t buf[adxl375::BURST_LEN] = {};  // MISRA-4.1: zero-init
    if (!readRegs(adxl375::REG_DATAX0, buf, adxl375::BURST_LEN))
    {
        consecutiveErrors_++;
        if (consecutiveErrors_ >= ares::IMU_MAX_CONSECUTIVE_ERRORS)
        {
            LOG_W(TAG, "ADXL375: read failed %u times — marking not-ready, retry in %lu ms",
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

    // Convert raw counts to m/s² (49 mg/LSB × g).
    const float scale = adxl375::ACCEL_MG_PER_LSB * adxl375::GRAVITY_MS2;
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

bool Adxl375Driver::writeReg(uint8_t reg, uint8_t value)
{
    wire_.beginTransmission(addr_);
    wire_.write(reg);
    wire_.write(value);
    return wire_.endTransmission() == 0;
}

bool Adxl375Driver::readRegs(uint8_t reg, uint8_t* buf, uint8_t len)
{
    if (buf == nullptr || len == 0U)
    {
        return false;
    }

    wire_.beginTransmission(addr_);
    wire_.write(reg);
    // Use STOP+START (endTransmission(true)) for bus robustness (same rationale as Mpu6050Driver).
    if (wire_.endTransmission(true) != 0)
    {
        return false;
    }

    const uint8_t received = wire_.requestFrom(
        static_cast<uint8_t>(addr_), static_cast<uint8_t>(len));
    if (received != len)
    {
        return false;
    }

    for (uint8_t i = 0U; i < len; i++)
    {
        buf[i] = static_cast<uint8_t>(wire_.read());
    }
    return true;
}

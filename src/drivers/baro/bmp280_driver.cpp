/**
 * @file  bmp280_driver.cpp
 * @brief BMP280 I2C barometer driver implementation.
 */

#include "drivers/baro/bmp280_driver.h"
#include "config.h"

#include <cmath>

// ── BMP280 register map (BST-BMP280-DS001, Table 18) ────────
namespace bmp280
{
    constexpr uint8_t REG_CALIB      = 0x88;
    constexpr uint8_t REG_CHIP_ID    = 0xD0;
    constexpr uint8_t REG_RESET      = 0xE0;
    constexpr uint8_t REG_STATUS     = 0xF3;
    constexpr uint8_t REG_CTRL_MEAS  = 0xF4;
    constexpr uint8_t REG_CONFIG     = 0xF5;
    constexpr uint8_t REG_PRESS_MSB  = 0xF7;

    constexpr uint8_t CHIP_ID_BMP280 = 0x58;
    constexpr uint8_t CHIP_ID_BME280 = 0x60;
    constexpr uint8_t RESET_CMD      = 0xB6;

    // ctrl_meas register (0xF4):
    //   bits [7:5] osrs_t  = 001  → temperature oversampling ×1
    //   bits [4:2] osrs_p  = 011  → pressure oversampling ×4
    //   bits [1:0] mode    = 11   → normal (continuous) mode
    constexpr uint8_t CTRL_MEAS_VAL  = 0x2F;

    // config register (0xF5) — must be written BEFORE ctrl_meas
    // enters normal mode, or settings are ignored (datasheet §5.4.6).
    //   bits [7:5] t_sb    = 100  → 500 ms standby between samples
    //   bits [4:2] filter  = 010  → IIR filter coefficient ×4
    //   bit  [0]   spi3w_en= 0    → SPI 3-wire disabled (I2C)
    constexpr uint8_t CONFIG_VAL     = 0x88;

    constexpr uint8_t CALIB_LEN      = 24;
    constexpr uint8_t DATA_LEN       = 6;
    constexpr uint8_t STATUS_IM_UPD  = 0x01;
    constexpr uint8_t MAX_INIT_POLLS = 10;

    // Hypsometric formula constants (MISRA-7)
    // altitude = 44330 × (1 − (P / P₀) ^ 0.190295)
    constexpr float PA_PER_HPA         = 100.0f;    ///< Pa per hPa conversion factor.
    constexpr float HYPSOMETRIC_COEFF  = 44330.0f;   ///< Hypsometric formula constant (m).
    constexpr float HYPSOMETRIC_EXP    = 0.190295f;  ///< Hypsometric pressure exponent.
}

// ── Public ──────────────────────────────────────────────────

Bmp280Driver::Bmp280Driver(TwoWire& wire, uint8_t addr)
    : wire_(wire)
    , addr_(addr)
{
}

bool Bmp280Driver::begin()
{
    ready_ = false;

    uint8_t id = 0;
    if (!readReg(bmp280::REG_CHIP_ID, id))
    {
        return false;
    }
    if (id != bmp280::CHIP_ID_BMP280 && id != bmp280::CHIP_ID_BME280)
    {
        return false;
    }

    if (!writeReg(bmp280::REG_RESET, bmp280::RESET_CMD))
    {
        return false;
    }
    delay(ares::BMP280_RESET_DELAY_MS);  // init-only wait for reset (RTOS-1.1: acceptable in setup)

    for (uint8_t i = 0; i < bmp280::MAX_INIT_POLLS; i++)
    {
        uint8_t status = 0;
        // PO10-7: readReg failure yields status=0 → loop exits safely
        (void)readReg(bmp280::REG_STATUS, status);
        if ((status & bmp280::STATUS_IM_UPD) == 0)
        {
            break;
        }
        delay(ares::BMP280_POLL_DELAY_MS);
    }

    if (!readCalibration())
    {
        return false;
    }

    if (!writeReg(bmp280::REG_CONFIG, bmp280::CONFIG_VAL))
    {
        return false;
    }

    if (!writeReg(bmp280::REG_CTRL_MEAS, bmp280::CTRL_MEAS_VAL))
    {
        return false;
    }

    ready_ = true;
    return true;
}

BaroStatus Bmp280Driver::read(BaroReading& out)
{
    if (!ready_)
    {
        return BaroStatus::NOT_READY;
    }

    uint8_t buf[bmp280::DATA_LEN] = {};  // MISRA-4.1
    if (!readRegs(bmp280::REG_PRESS_MSB, buf, bmp280::DATA_LEN))
    {
        return BaroStatus::ERROR;
    }

    // 20-bit raw values packed as [MSB:8 | LSB:8 | XLSB:4+pad:4].
    // Burst layout: press[0..2], temp[3..5] (datasheet Table 29).
    int32_t rawPress = (static_cast<int32_t>(buf[0]) << 12)
                     | (static_cast<int32_t>(buf[1]) << 4)
                     | (static_cast<int32_t>(buf[2]) >> 4);

    int32_t rawTemp  = (static_cast<int32_t>(buf[3]) << 12)
                     | (static_cast<int32_t>(buf[4]) << 4)
                     | (static_cast<int32_t>(buf[5]) >> 4);

    // Temperature must be compensated first: it computes tFine_,
    // which is then used as an input to the pressure formula.
    out.temperatureC = compensateTemperature(rawTemp);
    out.pressurePa   = compensatePressure(rawPress);

    // CERT-16.2: guard against NaN/Inf from compensation
    if (!isfinite(out.temperatureC) || !isfinite(out.pressurePa))
    {
        return BaroStatus::ERROR;
    }

    float pressHPa = out.pressurePa / bmp280::PA_PER_HPA;
    out.altitudeM  = bmp280::HYPSOMETRIC_COEFF
                   * (1.0f - powf(pressHPa / seaLevelHPa_,
                                  bmp280::HYPSOMETRIC_EXP));

    if (!isfinite(out.altitudeM))
    {
        return BaroStatus::ERROR;
    }

    return BaroStatus::OK;
}

void Bmp280Driver::setSeaLevelPressure(float hPa)
{
    // PO10-7.2: validate parameter (interface @pre: finite, positive)
    if (isfinite(hPa) && hPa > 0.0f)
    {
        seaLevelHPa_ = hPa;
    }
}

// ── Calibration (datasheet Table 16: registers 0x88..0x9F) ──
//
// 24 bytes encode 12 coefficients in little-endian pairs:
//   T1(u16) T2(s16) T3(s16) P1(u16) P2..P9(s16)

bool Bmp280Driver::readCalibration()
{
    uint8_t buf[bmp280::CALIB_LEN] = {};  // MISRA-4.1
    if (!readRegs(bmp280::REG_CALIB, buf, bmp280::CALIB_LEN))
    {
        return false;
    }

    // Lambdas assemble little-endian byte pairs into 16-bit values.
    auto u16 = [&](uint8_t i) -> uint16_t
    {
        return static_cast<uint16_t>(buf[i])
             | (static_cast<uint16_t>(buf[i + 1]) << 8);
    };

    auto s16 = [&](uint8_t i) -> int16_t
    {
        return static_cast<int16_t>(u16(i));
    };

    cal_.digT1 = u16(0);
    cal_.digT2 = s16(2);
    cal_.digT3 = s16(4);
    cal_.digP1 = u16(6);
    cal_.digP2 = s16(8);
    cal_.digP3 = s16(10);
    cal_.digP4 = s16(12);
    cal_.digP5 = s16(14);
    cal_.digP6 = s16(16);
    cal_.digP7 = s16(18);
    cal_.digP8 = s16(20);
    cal_.digP9 = s16(22);

    return true;
}

// ── Compensation (Bosch datasheet Section 8.1, float) ───────
// DEVIATION: MISRA-7 — numeric literals are direct transcription of the
// Bosch BMP280 datasheet compensation formulas (BST-BMP280-DS001,
// Section 8.1).  Named constants would hinder verification against the
// datasheet.  Approved: SL 2026-04-20.

float Bmp280Driver::compensateTemperature(int32_t rawTemp)
{
    float var1 = (static_cast<float>(rawTemp) / 16384.0f
              -   static_cast<float>(cal_.digT1) / 1024.0f)
              *   static_cast<float>(cal_.digT2);

    float diff = static_cast<float>(rawTemp) / 131072.0f
               - static_cast<float>(cal_.digT1) / 8192.0f;

    float var2 = diff * diff * static_cast<float>(cal_.digT3);

    tFine_ = static_cast<int32_t>(var1 + var2);
    return (var1 + var2) / 5120.0f;
}

float Bmp280Driver::compensatePressure(int32_t rawPress)
{
    float var1 = static_cast<float>(tFine_) / 2.0f - 64000.0f;
    float var2 = var1 * var1 * static_cast<float>(cal_.digP6) / 32768.0f;
    var2 = var2 + var1 * static_cast<float>(cal_.digP5) * 2.0f;
    var2 = var2 / 4.0f + static_cast<float>(cal_.digP4) * 65536.0f;

    var1 = (static_cast<float>(cal_.digP3) * var1 * var1 / 524288.0f
          + static_cast<float>(cal_.digP2) * var1) / 524288.0f;
    var1 = (1.0f + var1 / 32768.0f) * static_cast<float>(cal_.digP1);

    // Guard: if calibration yields P1 ≈ 0 the divisor collapses
    // and the result is meaningless.  Return 0 Pa instead of Inf.
    if (var1 < 1.0f)
    {
        return 0.0f;
    }

    float p = 1048576.0f - static_cast<float>(rawPress);
    p = (p - var2 / 4096.0f) * 6250.0f / var1;

    var1 = static_cast<float>(cal_.digP9) * p * p / 2147483648.0f;
    var2 = p * static_cast<float>(cal_.digP8) / 32768.0f;
    p = p + (var1 + var2 + static_cast<float>(cal_.digP7)) / 16.0f;

    return p;  // Pascals
}

// ── I2C helpers ─────────────────────────────────────────────
//
// Wire transactions use the repeated-start pattern for reads:
//   START → addr+W → reg → REPEATED START → addr+R → data → STOP
// endTransmission(false) sends REPEATED START instead of STOP,
// keeping the bus held so no other master can interleave.

bool Bmp280Driver::writeReg(uint8_t reg, uint8_t value)
{
    wire_.beginTransmission(addr_);
    wire_.write(reg);
    wire_.write(value);
    return wire_.endTransmission() == 0;
}

bool Bmp280Driver::readRegs(uint8_t reg, uint8_t* buf, uint8_t len)
{
    wire_.beginTransmission(addr_);
    wire_.write(reg);
    if (wire_.endTransmission(false) != 0)
    {
        return false;
    }

    if (wire_.requestFrom(addr_, len) != len)
    {
        return false;
    }

    for (uint8_t i = 0; i < len; i++)
    {
        buf[i] = static_cast<uint8_t>(wire_.read());
    }
    return true;
}

bool Bmp280Driver::readReg(uint8_t reg, uint8_t& out)
{
    return readRegs(reg, &out, 1);  // PO10-7: propagate error
}

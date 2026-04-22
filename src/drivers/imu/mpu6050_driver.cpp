/**
 * @file  mpu6050_driver.cpp
 * @brief MPU-6050 I2C IMU driver implementation.
 */

#include "drivers/imu/mpu6050_driver.h"
#include "config.h"

#include <cmath>

// ── MPU-6050 register map (PS-MPU-6000A-00, Table 3) ────────
namespace mpu6050
{
    constexpr uint8_t REG_SMPLRT_DIV   = 0x19;  ///< Sample-rate divider.
    constexpr uint8_t REG_CONFIG       = 0x1A;  ///< DLPF / FSYNC config.
    constexpr uint8_t REG_GYRO_CONFIG  = 0x1B;  ///< Gyro full-scale range.
    constexpr uint8_t REG_ACCEL_CONFIG = 0x1C;  ///< Accel full-scale range.
    constexpr uint8_t REG_ACCEL_XOUT_H = 0x3B;  ///< Burst origin: accel+temp+gyro.
    constexpr uint8_t REG_PWR_MGMT_1   = 0x6B;  ///< Power management 1.
    constexpr uint8_t REG_WHO_AM_I     = 0x75;  ///< Device ID register.

    constexpr uint8_t WHO_AM_I_ID      = 0x68;  ///< Expected WHO_AM_I value.
    constexpr uint8_t PWR_WAKE         = 0x00;  ///< Clear sleep bit → normal mode.

    // GYRO_CONFIG bits [4:3]: FS_SEL = 00 → ±250 deg/s (datasheet §4.4)
    constexpr uint8_t GYRO_FS_250DPS   = 0x00;
    // ACCEL_CONFIG bits [4:3]: AFS_SEL = 00 → ±2 g (datasheet §4.5)
    constexpr uint8_t ACCEL_FS_2G      = 0x00;
    // CONFIG: DLPF_CFG = 2 → accel 94 Hz / gyro 98 Hz bandwidth (datasheet §4.3)
    constexpr uint8_t DLPF_BW_94HZ     = 0x02;
    // SMPLRT_DIV: sample rate = 1 kHz / (1 + DIV) → DIV=9 → 100 Hz
    constexpr uint8_t SAMPLE_RATE_100HZ = 0x09;

    constexpr uint8_t BURST_LEN         = 14;  ///< Bytes in accel+temp+gyro burst.

    // Sensitivity scale factors (datasheet §6.2, §6.1)
    constexpr float ACCEL_SCALE        = 16384.0f;  ///< LSB per g at ±2 g.
    constexpr float GYRO_SCALE         = 131.0f;    ///< LSB per deg/s at ±250 deg/s.
    // Temperature formula: Temp(°C) = raw / 340.0 + 36.53 (datasheet §4.18)
    constexpr float TEMP_SCALE         = 340.0f;
    constexpr float TEMP_OFFSET        = 36.53f;
    // Standard gravity constant for g → m/s² conversion (MISRA-7)
    constexpr float GRAVITY_MS2        = 9.80665f;
}

// ── Public ──────────────────────────────────────────────────

Mpu6050Driver::Mpu6050Driver(TwoWire& wire, uint8_t addr)
    : wire_(wire)
    , addr_(addr)
{
}

bool Mpu6050Driver::begin()
{
    ready_ = false;

    // Wake the device (clears SLEEP bit in PWR_MGMT_1).
    if (!writeReg(mpu6050::REG_PWR_MGMT_1, mpu6050::PWR_WAKE))
    {
        return false;
    }
    delay(ares::MPU6050_WAKE_DELAY_MS);  // wait for clocks to settle (init-only)

    // Verify identity.
    uint8_t id = 0;
    uint8_t idBuf[1] = {};
    if (!readRegs(mpu6050::REG_WHO_AM_I, idBuf, 1U))
    {
        return false;
    }
    id = idBuf[0];
    if (id != mpu6050::WHO_AM_I_ID)
    {
        return false;
    }

    // Configure sample rate: 100 Hz.
    if (!writeReg(mpu6050::REG_SMPLRT_DIV, mpu6050::SAMPLE_RATE_100HZ))
    {
        return false;
    }

    // Digital low-pass filter: ~94 Hz accel / ~98 Hz gyro bandwidth.
    if (!writeReg(mpu6050::REG_CONFIG, mpu6050::DLPF_BW_94HZ))
    {
        return false;
    }

    // Gyro full-scale: ±250 deg/s.
    if (!writeReg(mpu6050::REG_GYRO_CONFIG, mpu6050::GYRO_FS_250DPS))
    {
        return false;
    }

    // Accel full-scale: ±2 g.
    if (!writeReg(mpu6050::REG_ACCEL_CONFIG, mpu6050::ACCEL_FS_2G))
    {
        return false;
    }

    ready_ = true;
    return true;
}

ImuStatus Mpu6050Driver::read(ImuReading& out)
{
    if (!ready_)
    {
        return ImuStatus::NOT_READY;
    }

    uint8_t buf[mpu6050::BURST_LEN] = {};  // MISRA-4.1: zero-init
    if (!readRegs(mpu6050::REG_ACCEL_XOUT_H, buf, mpu6050::BURST_LEN))
    {
        return ImuStatus::ERROR;
    }

    // Burst layout (14 bytes, big-endian pairs, datasheet §4.17):
    //   [0..1]  ACCEL_X  [2..3]  ACCEL_Y  [4..5]  ACCEL_Z
    //   [6..7]  TEMP
    //   [8..9]  GYRO_X   [10..11] GYRO_Y  [12..13] GYRO_Z
    auto toInt16 = [&](uint8_t hi, uint8_t lo) -> int16_t
    {
        return static_cast<int16_t>(
            (static_cast<uint16_t>(hi) << 8U) | static_cast<uint16_t>(lo));
    };

    const int16_t rawAx = toInt16(buf[0],  buf[1]);
    const int16_t rawAy = toInt16(buf[2],  buf[3]);
    const int16_t rawAz = toInt16(buf[4],  buf[5]);
    const int16_t rawT  = toInt16(buf[6],  buf[7]);
    const int16_t rawGx = toInt16(buf[8],  buf[9]);
    const int16_t rawGy = toInt16(buf[10], buf[11]);
    const int16_t rawGz = toInt16(buf[12], buf[13]);

    out.accelX = (static_cast<float>(rawAx) / mpu6050::ACCEL_SCALE) * mpu6050::GRAVITY_MS2;
    out.accelY = (static_cast<float>(rawAy) / mpu6050::ACCEL_SCALE) * mpu6050::GRAVITY_MS2;
    out.accelZ = (static_cast<float>(rawAz) / mpu6050::ACCEL_SCALE) * mpu6050::GRAVITY_MS2;
    out.gyroX  = static_cast<float>(rawGx) / mpu6050::GYRO_SCALE;
    out.gyroY  = static_cast<float>(rawGy) / mpu6050::GYRO_SCALE;
    out.gyroZ  = static_cast<float>(rawGz) / mpu6050::GYRO_SCALE;
    out.tempC  = static_cast<float>(rawT) / mpu6050::TEMP_SCALE + mpu6050::TEMP_OFFSET;

    // Guard against NaN/Inf from raw-to-float conversion (CERT-16.2).
    if (!isfinite(out.accelX) || !isfinite(out.accelY) || !isfinite(out.accelZ)
     || !isfinite(out.gyroX)  || !isfinite(out.gyroY)  || !isfinite(out.gyroZ)
     || !isfinite(out.tempC))
    {
        return ImuStatus::ERROR;
    }

    return ImuStatus::OK;
}

// ── Private helpers ─────────────────────────────────────────

bool Mpu6050Driver::writeReg(uint8_t reg, uint8_t value)
{
    wire_.beginTransmission(addr_);
    wire_.write(reg);
    wire_.write(value);
    return wire_.endTransmission() == 0;
}

bool Mpu6050Driver::readRegs(uint8_t reg, uint8_t* buf, uint8_t len)
{
    if (buf == nullptr || len == 0U)
    {
        return false;
    }

    wire_.beginTransmission(addr_);
    wire_.write(reg);
    if (wire_.endTransmission(false) != 0)   // repeated-start
    {
        return false;
    }

    const uint8_t received = wire_.requestFrom(
        static_cast<uint8_t>(addr_), static_cast<uint8_t>(len));
    if (received != len)
    {
        return false;
    }

    for (uint8_t i = 0; i < len; i++)
    {
        buf[i] = static_cast<uint8_t>(wire_.read());
    }
    return true;
}

/**
 * @file  mpu6050_driver.h
 * @brief MPU-6050 I2C IMU driver (InvenSense PS-MPU-6000A-00).
 *
 * Implements ImuInterface for the MPU-6050 6-axis IMU.
 * Reads accelerometer (±2 g, full scale), gyroscope (±250 deg/s),
 * and die temperature via I2C using a single 14-byte burst read.
 *
 * Thread safety: NOT thread-safe.  Must be accessed from a single
 *                task or protected externally (CERT-13).
 */
#pragma once

#include <Wire.h>

#include "hal/imu/imu_interface.h"

/**
 * Concrete ImuInterface for the InvenSense MPU-6050.
 *
 * Init sequence (begin()):
 *   1. Wake device by writing 0x00 to PWR_MGMT_1 (0x6B).
 *   2. Verify WHO_AM_I register (0x75) returns 0x68.
 *   3. Set sample-rate divider for 100 Hz output (SMPLRT_DIV = 9).
 *   4. Set DLPF for 94 Hz bandwidth (CONFIG = 0x02).
 *
 * Reading (read()):
 *   1. Burst-read 14 bytes starting at ACCEL_XOUT_H (0x3B).
 *   2. Convert raw values to SI units:
 *      - Accel: raw / 16384.0 * 9.80665 (±2 g full scale, m/s²).
 *      - Gyro : raw / 131.0             (±250 deg/s full scale).
 *      - Temp : raw / 340.0 + 36.53     (datasheet formula, °C).
 */
class Mpu6050Driver : public ImuInterface
{
public:
    /**
     * Construct an MPU-6050 driver instance.
     * @param[in] wire  I2C bus instance (must be initialised before begin()).
     * @param[in] addr  7-bit I2C address (0x68 if AD0→GND, 0x69 if AD0→VCC).
     */
    Mpu6050Driver(TwoWire& wire, uint8_t addr);

    // Non-copyable, non-movable (CERT-18.3)
    Mpu6050Driver(const Mpu6050Driver&)            = delete;
    Mpu6050Driver& operator=(const Mpu6050Driver&) = delete;

    /**
     * Initialise the MPU-6050 sensor over I2C.
     * @return true on success; false if WHO_AM_I check fails.
     * @post Sensor is in normal measurement mode on success.
     */
    bool begin() override;

    /**
     * Read the latest IMU measurement.
     * @param[out] out  Populated with accel, gyro, and die temp on ImuStatus::OK.
     * @return Status code (OK, ERROR, or NOT_READY).
     */
    ImuStatus read(ImuReading& out) override;
    const char* driverModel() const override { return "MPU6050"; }

private:
    /// Write a single register over I2C.
    bool writeReg(uint8_t reg, uint8_t value);
    /// Burst-read @p len consecutive registers starting at @p reg.
    bool readRegs(uint8_t reg, uint8_t* buf, uint8_t len);

    TwoWire& wire_;            ///< I2C bus (injected, not owned).
    uint8_t  addr_;            ///< 7-bit I2C slave address.
    bool     ready_ = false;   ///< true after successful begin().
};

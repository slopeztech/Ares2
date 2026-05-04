/**
 * @file  adxl375_driver.h
 * @brief ADXL375 I2C high-g accelerometer driver (Analog Devices ADXL375).
 *
 * Implements ImuInterface for the ADXL375 ±200 g shock accelerometer.
 * The sensor provides three-axis acceleration only; gyroscope and
 * temperature fields of ImuReading are set to 0.0f.
 *
 * Measurement configuration (set by begin()):
 *   - Output data rate: 100 Hz (BW_RATE = 0x0A, normal power mode).
 *   - Resolution: fixed 49 mg/LSB (independent of DATA_FORMAT range bits).
 *   - FIFO: bypass mode (streaming, no FIFO buffering).
 *
 * Thread safety: thread-safe.  An internal FreeRTOS mutex serialises
 *                concurrent read() calls across tasks (CERT-13).
 *
 * @note The ADXL375 shares the I2C1 bus with the MPU-6050 — addresses
 *       are distinct (0x53 vs 0x68), no bus conflict arises.
 */
#pragma once

#include <Wire.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#include "hal/imu/imu_interface.h"

/**
 * Concrete ImuInterface for the Analog Devices ADXL375.
 *
 * Init sequence (begin()):
 *   1. Read DEVID register (0x00) — must return 0xE5.
 *   2. Write DATA_FORMAT (0x31) = 0x0B: FULL_RES=1, RANGE=11 (±200 g).
 *   3. Write BW_RATE (0x2C) = 0x0A: normal power, 100 Hz ODR.
 *   4. Write POWER_CTL (0x2D) = 0x08: enter Measure mode.
 *
 * Reading (read()):
 *   1. Burst-read 6 bytes from DATAX0 (0x32).
 *   2. Each axis is little-endian signed 16-bit.
 *   3. Scale: raw * 49 mg/LSB * 9.80665 m/s²/g → m/s².
 */
class Adxl375Driver : public ImuInterface
{
public:
    /**
     * Construct an ADXL375 driver instance.
     * @param[in] wire  I2C bus instance (must be initialised before begin()).
     * @param[in] addr  7-bit I2C address (0x53 if ALT_ADDRESS=GND, 0x1D if VCC).
     */
    Adxl375Driver(TwoWire& wire, uint8_t addr);

    // Non-copyable, non-movable (CERT-18.3).
    Adxl375Driver(const Adxl375Driver&)            = delete;
    Adxl375Driver& operator=(const Adxl375Driver&) = delete;

    /**
     * Initialise the ADXL375 sensor over I2C.
     * @return true on success; false if DEVID check fails or write errors.
     * @post Sensor is in Measure mode, 100 Hz ODR on success.
     */
    bool begin() override;

    /**
     * Read the latest acceleration measurement.
     * @param[out] out  Populated on ImuStatus::OK.  gyroX/Y/Z and tempC are 0.0f.
     * @return Status code (OK, ERROR, or NOT_READY).
     */
    ImuStatus read(ImuReading& out) override;

    const char* driverModel() const override { return "ADXL375"; }

private:
    /// Write a single register over I2C.
    bool writeReg(uint8_t reg, uint8_t value);
    /// Burst-read @p len consecutive registers starting at @p reg.
    bool readRegs(uint8_t reg, uint8_t* buf, uint8_t len);
    /// Inner read implementation — called while imuMutex_ is held.
    ImuStatus readLocked(ImuReading& out);

    TwoWire& wire_;                         ///< I2C bus (injected, not owned).
    uint8_t  addr_;                         ///< 7-bit I2C slave address.
    bool     ready_                = false; ///< true after successful begin().
    uint32_t lastReinitAttemptMs_  = 0U;    ///< millis() of last lazy re-init attempt.
    uint8_t  consecutiveErrors_    = 0U;    ///< Consecutive readRegs() failures.
    SemaphoreHandle_t imuMutex_    = nullptr; ///< Serialises concurrent calls (CERT-13).
};

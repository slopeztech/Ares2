/**
 * @file  bmp280_driver.h
 * @brief BMP280 I2C barometer driver (Bosch BST-BMP280-DS001).
 *
 * Thread safety: NOT thread-safe.  Must be accessed from a single
 *                task or protected externally (CERT-13).
 */
#pragma once

#include <Wire.h>

#include "hal/baro/barometer_interface.h"

/**
 * Concrete BarometerInterface for the Bosch BMP280 (and BME280).
 *
 * Reads temperature and pressure via I2C using the Bosch float
 * compensation formulae (datasheet BST-BMP280-DS001, Section 8.1).
 *
 * Init sequence (begin()):
 *   1. Verify chip ID (0x58 BMP280 / 0x60 BME280).
 *   2. Soft-reset and wait for NVM copy.
 *   3. Read factory calibration coefficients.
 *   4. Write CONFIG then CTRL_MEAS to enter normal mode.
 *
 * Reading (read()):
 *   1. Burst-read 6 bytes (press[3] + temp[3]).
 *   2. Compensate temperature first (sets tFine_).
 *   3. Compensate pressure (uses tFine_).
 *   4. Compute altitude via the hypsometric formula.
 */
class Bmp280Driver : public BarometerInterface
{
public:
    /**
     * Construct a BMP280 driver instance.
     * @param[in] wire  I2C bus instance (must be initialised before begin()).
     * @param[in] addr  7-bit I2C address (0x76 if SDO→GND, 0x77 if SDO→VCC).
     */
    Bmp280Driver(TwoWire& wire, uint8_t addr);

    // Non-copyable, non-movable (CERT-18.3)
    Bmp280Driver(const Bmp280Driver&)            = delete;
    Bmp280Driver& operator=(const Bmp280Driver&) = delete;

    /**
     * Initialise the BMP280 sensor over I2C.
     * @return true on success; false if chip ID is invalid.
     * @post Sensor is in normal measurement mode on success.
     */
    bool begin() override;

    /**
     * Read the latest barometer measurement.
     * @param[out] out  Populated with temperature, pressure, and altitude on BaroStatus::OK.
     * @return Status code (OK, ERROR, or NOT_READY).
     */
    BaroStatus read(BaroReading& out) override;

    /**
     * Set the reference sea-level pressure for altitude calculation.
     * @param[in] hPa  Sea-level pressure in hectopascals (default 1013.25).
     */
    void setSeaLevelPressure(float hPa) override;
    const char* driverModel() const override { return "BMP280"; }

private:
    /// Write a single register over I2C.
    bool writeReg(uint8_t reg, uint8_t value);
    /// Burst-read @p len consecutive registers starting at @p reg.
    bool readRegs(uint8_t reg, uint8_t* buf, uint8_t len);
    /// Read a single register (convenience wrapper around readRegs).
    bool readReg(uint8_t reg, uint8_t& out);

    /// Read the 12 factory calibration coefficients from NVM.
    bool readCalibration();
    /// Bosch float compensation for temperature; updates tFine_.
    float compensateTemperature(int32_t rawTemp);
    /// Bosch float compensation for pressure; requires valid tFine_.
    float compensatePressure(int32_t rawPress);

    TwoWire& wire_;           ///< I2C bus (injected, not owned).
    uint8_t addr_;             ///< 7-bit I2C slave address.
    bool ready_ = false;       ///< true after successful begin().
    float seaLevelHPa_ = 1013.25f;  ///< Reference pressure for altitude.

    /// Intermediate temperature value shared between compensateTemperature()
    /// and compensatePressure().  Must call compensateTemperature() first
    /// on each read cycle so this value is valid for the pressure formula.
    int32_t tFine_ = 0;

    /// Factory calibration coefficients read from NVM (registers 0x88–0x9F).
    /// Layout matches the Bosch datasheet table 16 (3 temperature + 9 pressure).
    struct CalibData
    {
        uint16_t digT1 = 0;
        int16_t  digT2 = 0;
        int16_t  digT3 = 0;
        uint16_t digP1 = 0;
        int16_t  digP2 = 0;
        int16_t  digP3 = 0;
        int16_t  digP4 = 0;
        int16_t  digP5 = 0;
        int16_t  digP6 = 0;
        int16_t  digP7 = 0;
        int16_t  digP8 = 0;
        int16_t  digP9 = 0;
    };

    CalibData cal_ = {};
};

/**
 * @file  barometer_interface.h
 * @brief Hardware-agnostic barometer interface (pure virtual).
 *
 * Thread safety: Implementations are NOT thread-safe.
 *                Callers must ensure single-task access or
 *                external synchronisation (CERT-13).
 */
#pragma once

#include <cstdint>

/**
 * Status codes returned by BarometerInterface::read().
 */
enum class BaroStatus : uint8_t
{
    OK        = 0,   ///< Valid reading available.
    ERROR     = 1,   ///< Communication or hardware error.
    NOT_READY = 2,   ///< Sensor not yet initialised.

    FIRST = OK,           // CERT-6.1 — range validation sentinels
    LAST  = NOT_READY
};

/**
 * A single barometer measurement.
 * All fields default-initialised to zero (MISRA-4).
 */
struct BaroReading
{
    float temperatureC = 0.0f;   ///< Temperature in degrees Celsius.
    float pressurePa   = 0.0f;   ///< Absolute pressure in Pascals.
    float altitudeM    = 0.0f;   ///< Estimated MSL altitude in metres.
};

/**
 * Abstract barometer interface.
 *
 * Concrete drivers (e.g. Bmp280Driver) implement this interface.
 * Application code accesses barometer data through a reference
 * to this interface, never through the concrete driver directly.
 */
class BarometerInterface
{
public:
    virtual ~BarometerInterface() = default;

    /**
     * Initialise the sensor hardware.
     * @pre  I2C/SPI bus must be initialised before calling.
     * @post Sensor is in normal measurement mode on success.
     * @return true on success, false on hardware failure.
     */
    virtual bool begin() = 0;

    /**
     * Read the latest measurement from the sensor.
     * @param[out] out  Populated with the reading on BaroStatus::OK.
     * @return Status code indicating result (see BaroStatus).
     */
    virtual BaroStatus read(BaroReading& out) = 0;

    /**
     * Set the reference sea-level pressure for altitude calculation.
     * @param[in] hPa  Sea-level pressure in hectopascals (default 1013.25).
     * @pre   hPa must be a finite, positive value.
     */
    virtual void setSeaLevelPressure(float hPa) = 0;

    /**
     * Return the driver model identifier (e.g. "BMP280", "BMP390").
     * Used by the AMS engine to validate 'include <MODEL> as BARO'.
     * @return Null-terminated model name string (static storage).
     */
    virtual const char* driverModel() const = 0;
};

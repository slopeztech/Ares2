/**
 * @file  imu_interface.h
 * @brief Hardware-agnostic IMU interface (pure virtual).
 *
 * Provides accelerometer, gyroscope, and die-temperature readings.
 * Concrete drivers (e.g. Mpu6050Driver) implement this interface.
 * Application code accesses IMU data through a reference to this
 * interface, never through the concrete driver directly.
 *
 * Thread safety: Implementations are NOT thread-safe.
 *                Callers must ensure single-task access or
 *                external synchronisation (CERT-13).
 */
#pragma once

#include <cstdint>

/**
 * Status codes returned by ImuInterface::read().
 */
enum class ImuStatus : uint8_t
{
    OK        = 0,   ///< Valid reading available.
    ERROR     = 1,   ///< Communication or hardware error.
    NOT_READY = 2,   ///< Sensor not yet initialised.

    FIRST = OK,           // CERT-6.1 — range validation sentinels
    LAST  = NOT_READY
};

/**
 * A single IMU measurement.
 * All fields default-initialised to zero (MISRA-4).
 *
 * Axes follow the right-hand NED body-frame convention
 * used across ARES avionics (DOX-4, hardware_registry.md).
 */
struct ImuReading
{
    float accelX = 0.0f;   ///< Body-frame X-axis acceleration in m/s².
    float accelY = 0.0f;   ///< Body-frame Y-axis acceleration in m/s².
    float accelZ = 0.0f;   ///< Body-frame Z-axis acceleration in m/s².
    float gyroX  = 0.0f;   ///< Body-frame X-axis angular rate in deg/s.
    float gyroY  = 0.0f;   ///< Body-frame Y-axis angular rate in deg/s.
    float gyroZ  = 0.0f;   ///< Body-frame Z-axis angular rate in deg/s.
    float tempC  = 0.0f;   ///< Die temperature in degrees Celsius.
};

/**
 * Abstract IMU interface.
 *
 * Concrete drivers implement this interface.  Application code
 * uses a reference to ImuInterface, enabling hardware substitution
 * without changing higher-level code.
 */
class ImuInterface
{
public:
    virtual ~ImuInterface() = default;

    /**
     * Initialise the sensor hardware.
     * @pre  I2C bus must be initialised before calling.
     * @post Sensor is in normal measurement mode on success.
     * @return true on success, false on hardware failure.
     */
    virtual bool begin() = 0;

    /**
     * Read the latest measurement from the sensor.
     * @param[out] out  Populated with the reading on ImuStatus::OK.
     * @return Status code indicating result (see ImuStatus).
     */
    virtual ImuStatus read(ImuReading& out) = 0;

    /**
     * Return the driver model identifier (e.g. "MPU6050", "ICM42688").
    * Used by the AMS engine to validate 'include <MODEL> as IMU'.
     */
    virtual const char* driverModel() const = 0;
};

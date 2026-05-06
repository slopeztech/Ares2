/**
 * @file  gps_interface.h
 * @brief Hardware-agnostic GNSS receiver interface (pure virtual).
 *
 * Thread safety: Implementations are NOT thread-safe.
 *                Callers must ensure single-task access or
 *                external synchronisation (CERT-13).
 */
#pragma once

#include <cstdint>

/**
 * GNSS fix type.
 */
enum class GpsFixType : uint8_t
{
    NONE   = 0,   ///< No fix.
    FIX_2D = 1,   ///< 2D fix (lat/lon only).
    FIX_3D = 2,   ///< 3D fix (lat/lon/alt).

    FIRST = NONE,      // CERT-6.1 — range validation sentinels
    LAST  = FIX_3D
};

/**
 * Status codes returned by GpsInterface::read().
 */
enum class GpsStatus : uint8_t
{
    OK        = 0,   ///< Valid fix available.
    NO_FIX    = 1,   ///< Receiver active, no fix yet.
    ERROR     = 2,   ///< Communication or hardware error.
    NOT_READY = 3,   ///< Receiver not yet initialised.
    TIMEOUT   = 4,   ///< No data received within timeout.

    FIRST = OK,           // CERT-6.1 — range validation sentinels
    LAST  = TIMEOUT
};

/**
 * A single GNSS measurement snapshot.
 * Fields ordered by decreasing alignment to minimise padding (MISRA-20.3).
 * All fields default-initialised to zero (MISRA-4).
 */
struct GpsReading
{
    float latitude     = 0.0f;   ///< Decimal degrees (+ N, - S).
    float longitude    = 0.0f;   ///< Decimal degrees (+ E, - W).
    float altitudeM    = 0.0f;   ///< MSL altitude in metres.
    float speedKmh     = 0.0f;   ///< Ground speed in km/h.
    float courseDeg    = 0.0f;   ///< Course over ground in degrees.
    float hdop         = 0.0f;   ///< Horizontal dilution of precision.
    uint32_t timestampMs = 0;    ///< millis() at last valid parse.
    uint8_t    satellites = 0;   ///< Satellites in use.
    GpsFixType fixType    = GpsFixType::NONE;  ///< Fix quality.
};

/**
 * Abstract GNSS receiver interface.
 *
 * Concrete drivers (e.g. Bn220Driver) implement this interface.
 * Application code accesses GPS data through a reference to this
 * interface, never through the concrete driver directly.
 */
class GpsInterface
{
public:
    virtual ~GpsInterface() = default;

    /**
     * Initialise the receiver (open UART, configure baud rate).
     * @pre  UART pins must be configured in config.h.
     * @post Receiver is listening for NMEA sentences on success.
     * @return true on success, false on hardware failure.
     */
    virtual bool begin() = 0;

    /**
     * Process available serial bytes from the receiver.
     * Must be called frequently (at least once per main-loop
     * iteration) to consume UART data and keep the internal
     * parser up to date.
     * @pre  begin() returned true.
     */
    virtual void update() = 0;

    /**
     * Read the latest GNSS fix.
     * @param[out] out  Populated with the reading on GpsStatus::OK.
     * @return Status code indicating result (see GpsStatus).
     */
    virtual GpsStatus read(GpsReading& out) = 0;

    /**
     * Check whether a valid fix is currently available.
     * @return true if the receiver has a 2D or 3D fix.
     */
    virtual bool hasFix() const = 0;

    /**
     * Return the driver model identifier (e.g. "BN220", "BN880").
    * Used by the AMS engine to validate 'include <MODEL> as GPS'.
     * @return Null-terminated model name string (static storage).
     */
    virtual const char* driverModel() const = 0;
};

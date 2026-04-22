/**
 * @file  bn220_driver.h
 * @brief BN-220 GPS driver with NMEA 0183 sentence parser.
 *
 * Thread safety: NOT thread-safe.  Must be accessed from a single
 *                task or protected externally (CERT-13).
 */
#pragma once

#include <Arduino.h>

#include "hal/gps/gps_interface.h"

/**
 * Concrete GpsInterface for the BN-220 (u-blox UBX-M8030).
 *
 * Parses NMEA 0183 sentences from a UART-connected GNSS receiver.
 * Only two sentence types are consumed:
 *   - **GGA** — position, altitude, fix quality, HDOP, satellites.
 *   - **RMC** — ground speed and course over ground.
 *
 * Other talker IDs (GP, GN, GL, GA) are accepted transparently;
 * only the last 3 characters of the sentence type are matched.
 *
 * The driver is poll-based: call update() each loop iteration to
 * drain the hardware UART FIFO, then read() to get the latest fix.
 * If no valid GGA arrives within GPS_TIMEOUT_MS the fix is
 * automatically invalidated.
 */
class Bn220Driver : public GpsInterface
{
public:
    /**
     * Construct a BN-220 GPS driver instance.
     * @param[in] serial  HardwareSerial port connected to the receiver.
     * @param[in] rxPin   ESP32 GPIO pin for UART RX (← GPS TX).
     * @param[in] txPin   ESP32 GPIO pin for UART TX (→ GPS RX).
     * @param[in] baud    UART baud rate (typically 9600).
     */
    Bn220Driver(HardwareSerial& serial, int8_t rxPin, int8_t txPin, uint32_t baud);

    // Non-copyable, non-movable (CERT-18.3)
    Bn220Driver(const Bn220Driver&)            = delete;
    Bn220Driver& operator=(const Bn220Driver&) = delete;

    /**
     * Initialise the UART connection to the GPS receiver.
     * @return true on success; false on UART failure.
     * @post Receiver is listening for NMEA sentences on success.
     */
    bool begin() override;

    /**
     * Process available UART bytes from the receiver.
     * Must be called frequently (every loop iteration) to keep the NMEA parser current.
     */
    void update() override;

    /**
     * Read the latest GPS fix snapshot.
     * @param[out] out  Populated with the latest fix on GpsStatus::OK.
     * @return Status code (OK, NO_FIX, ERROR, NOT_READY, or TIMEOUT).
     */
    GpsStatus read(GpsReading& out) override;

    /**
     * Check whether a valid GPS fix is currently available.
     * @return true if the receiver has a 2D or 3D fix.
     */
    bool hasFix() const override;

private:
    /// Feed one byte into the NMEA state machine.
    void processByte(char c);
    /// Parse a complete sentence (after '\r'/'\n').
    void processSentence();
    /// Verify the NMEA XOR checksum between '$' and '*'.
    bool validateChecksum();
    /// Extract position data from a GGA sentence.
    void parseGGA(char* fields[], uint8_t count);
    /// Extract speed/course data from an RMC sentence.
    void parseRMC(char* fields[], uint8_t count);

    /// Convert NMEA ddmm.mmmm / dddmm.mmmm + hemisphere to decimal degrees.
        static float parseCoordinate(const char* raw, const char* hem);
    /// Safe strtof wrapper: returns 0.0f on null, empty, NaN, or Inf.
    static float  parseFloat(const char* str);
    /// Safe strtol wrapper: returns 0 on null or empty.
    static int32_t parseInt(const char* str);

    HardwareSerial& serial_;   ///< UART port (injected, not owned).
    int8_t rxPin_;              ///< ESP32 GPIO for UART RX.
    int8_t txPin_;              ///< ESP32 GPIO for UART TX.
    uint32_t baud_;             ///< Configured baud rate.
    bool ready_ = false;        ///< true after begin().

    // NMEA state machine ——————————————————————————————
    // Bytes accumulate in sentence_[] between a '$' start marker
    // and a '\r'/'\n' end marker.  The '$' itself is excluded so
    // the checksum XOR can be computed directly over the buffer.
    static constexpr uint8_t NMEA_MAX_LEN = 83;  ///< 82 chars + null (NMEA spec).
    static constexpr uint8_t MAX_FIELDS   = 20;  ///< Max comma-separated fields.
    char     sentence_[NMEA_MAX_LEN] = {};
    uint8_t  sentenceLen_ = 0;
    bool     inSentence_  = false;

    // Parsed data
    GpsReading reading_ = {};     ///< Latest parsed fix snapshot.
    bool       hasFix_  = false;  ///< true while fix is current.
    uint32_t   lastFixMs_ = 0;    ///< millis() of last valid GGA parse.

    /// If no valid GGA arrives within this interval the fix is
    /// automatically invalidated and read() returns TIMEOUT.
    static constexpr uint32_t GPS_TIMEOUT_MS = 5000;
};

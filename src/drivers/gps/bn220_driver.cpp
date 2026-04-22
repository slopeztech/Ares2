/**
 * @file  bn220_driver.cpp
 * @brief BN-220 GPS driver implementation (NMEA 0183 parser).
 */

#include "drivers/gps/bn220_driver.h"
#include "config.h"

#include <climits>
#include <cmath>
#include <cstring>
#include <cstdlib>

// ── NMEA / conversion constants (MISRA-7) ───────────────────
//
// NMEA 0183 encodes coordinates as ddmm.mmmm (lat) or dddmm.mmmm
// (lon).  To convert to decimal degrees:
//   degrees = int(raw / 100)
//   minutes = raw - degrees * 100
//   decimal = degrees + minutes / 60
//
namespace nmea
{
    constexpr float   KNOTS_TO_KMH    = 1.852f;   ///< NMEA speed unit conversion.
    constexpr float   DEG_DIVISOR     = 100.0f;   ///< NMEA ddmm.mmmm encoding.
    constexpr float   MINUTES_PER_DEG = 60.0f;    ///< Arc-minutes per degree.
    constexpr uint8_t MIN_GGA_FIELDS  = 15;       ///< NMEA 0183 GGA sentence.
    constexpr uint8_t MIN_RMC_FIELDS  = 12;       ///< NMEA 0183 RMC sentence.
    constexpr uint8_t MIN_TYPE_LEN    = 3;        ///< e.g. "GGA", "RMC".
    constexpr uint8_t MIN_SATS_3D     = 4;        ///< Satellites required for 3D fix.
}

// ── Public ──────────────────────────────────────────────────

Bn220Driver::Bn220Driver(HardwareSerial& serial,
                         int8_t rxPin, int8_t txPin, uint32_t baud)
    : serial_(serial)
    , rxPin_(rxPin)
    , txPin_(txPin)
    , baud_(baud)
{
}

bool Bn220Driver::begin()
{
    serial_.begin(baud_, SERIAL_8N1, rxPin_, txPin_);
    ready_ = true;
    return true;
}

void Bn220Driver::update()
{
    if (!ready_)
    {
        return;
    }

    // Drain the UART FIFO into the NMEA state machine.
    // Bounded to MAX_SERIAL_READ bytes per call to prevent a
    // flood of NMEA data from starving other processing.
    uint16_t bytesRead = 0;
    while (serial_.available() > 0 && bytesRead < ares::MAX_SERIAL_READ)
    {
        char c = static_cast<char>(serial_.read());
        processByte(c);
        bytesRead++;
    }
}

GpsStatus Bn220Driver::read(GpsReading& out)
{
    if (!ready_)
    {
        return GpsStatus::NOT_READY;  // MISRA-5.1: guard
    }

    uint32_t now = millis();

    if (hasFix_ && (now - lastFixMs_ > GPS_TIMEOUT_MS))
    {
        hasFix_ = false;
        reading_.fixType = GpsFixType::NONE;
    }

    out = reading_;

    GpsStatus result = GpsStatus::OK;

    if (!hasFix_)
    {
        result = GpsStatus::NO_FIX;
        if (lastFixMs_ != 0 && (now - lastFixMs_ > GPS_TIMEOUT_MS))
        {
            result = GpsStatus::TIMEOUT;
        }
    }

    return result;
}

bool Bn220Driver::hasFix() const
{
    return hasFix_;
}

// ── NMEA state machine ───────────────────────────────────
//
// NMEA sentence format:  $TALKER,F1,F2,...,Fn*HH\r\n
//   - '$' marks the start (not stored — excluded from checksum).
//   - '*' separates payload from the 2-digit hex checksum.
//   - '\r\n' marks the end and triggers parsing.
//   - Checksum = XOR of all bytes between '$' and '*'.
//
// State transitions:
//   IDLE  ──'$'──→  COLLECTING  ──'\r'/'\n'──→  PROCESS  →  IDLE
//                      │
//                  overflow → discard → IDLE──

void Bn220Driver::processByte(char c)
{
    if (c == '$')
    {
        sentenceLen_ = 0;
        inSentence_  = true;
        return;
    }

    if (!inSentence_)
    {
        return;
    }

    if (c == '\r' || c == '\n')
    {
        if (sentenceLen_ > 0)
        {
            sentence_[sentenceLen_] = '\0';
            processSentence();
        }
        inSentence_ = false;
        return;
    }

    if (sentenceLen_ < NMEA_MAX_LEN - 1)
    {
        sentence_[sentenceLen_++] = c;
    }
    else
    {
        inSentence_ = false;  // overflow — discard
    }
}

void Bn220Driver::processSentence()
{
    if (!validateChecksum())
    {
        return;
    }

    // Truncate at '*' (remove checksum from working buffer)
    char* star = strchr(sentence_, '*');
    if (star != nullptr)
    {
        *star = '\0';
    }

    // Split into fields by comma
    char* fields[MAX_FIELDS] = {};  // MISRA-4.1
    uint8_t fieldCount = 0;
    char* ptr = sentence_;

    while (fieldCount < MAX_FIELDS)
    {
        fields[fieldCount++] = ptr;
        ptr = strchr(ptr, ',');
        if (ptr == nullptr)
        {
            break;
        }
        *ptr = '\0';
        ptr++;
    }

    if (fieldCount < 2)
    {
        return;
    }

    // Match sentence type — skip talker ID (GP, GN, GL, GA, …)
    const char* type = fields[0];
    size_t len = strnlen(type, NMEA_MAX_LEN);  // CERT-3: bounded length
    if (len < nmea::MIN_TYPE_LEN)
    {
        return;
    }

    const char* suffix = type + len - nmea::MIN_TYPE_LEN;

    if (strcmp(suffix, "GGA") == 0 && fieldCount >= nmea::MIN_GGA_FIELDS)
    {
        parseGGA(fields, fieldCount);
    }
    else if (strcmp(suffix, "RMC") == 0 && fieldCount >= nmea::MIN_RMC_FIELDS)
    {
        parseRMC(fields, fieldCount);
    }
}

bool Bn220Driver::validateChecksum()
{
    // NMEA checksum: XOR of every byte between '$' (exclusive,
    // not in our buffer) and '*' (exclusive).  The two hex digits
    // after '*' are the expected result.
    const char* star = strchr(sentence_, '*');
    if (star == nullptr)
    {
        return false;
    }

    // Need at least 2 hex digits after '*'
    ptrdiff_t starIdx = star - sentence_;
    if (starIdx + 2 >= static_cast<ptrdiff_t>(sentenceLen_))
    {
        return false;
    }

    // XOR all bytes between '$' (excluded, not in buffer) and '*'
    uint8_t computed = 0;
    for (const char* p = sentence_; p < star; p++)
    {
        computed ^= static_cast<uint8_t>(*p);
    }

    const char hexStr[3] = { star[1], star[2], '\0' };
    uint8_t expected = static_cast<uint8_t>(strtoul(hexStr, nullptr, 16));

    return computed == expected;
}

// ── Sentence parsers ──────────────────────────────────────
//
// GGA fields used (NMEA 0183 §6.3):
//   [2] latitude    [3] N/S    [4] longitude  [5] E/W
//   [6] fix quality [7] sats   [8] HDOP       [9] altitude
//
// RMC fields used (NMEA 0183 §6.5):
//   [2] status (A/V)  [7] speed (knots)  [8] course (degrees)──

void Bn220Driver::parseGGA(char* fields[], uint8_t count)
{
    (void)count;

    // Field 6: fix quality (0 = invalid)
    int32_t fixQual = parseInt(fields[6]);
    if (fixQual == 0)
    {
        reading_.fixType = GpsFixType::NONE;
        hasFix_ = false;
        return;
    }

    reading_.latitude   = parseCoordinate(fields[2], fields[3]);
    reading_.longitude  = parseCoordinate(fields[4], fields[5]);

    // DO-6.1 / CERT-1: validate coordinate ranges
    if (reading_.latitude < -90.0f || reading_.latitude > 90.0f ||
        reading_.longitude < -180.0f || reading_.longitude > 180.0f)
    {
        reading_.fixType = GpsFixType::NONE;
        hasFix_ = false;
        return;
    }

    int32_t satRaw = parseInt(fields[7]);
    // CERT-1 / MISRA-2: clamp to uint8_t range before narrowing
    if (satRaw < 0)     { satRaw = 0; }
    if (satRaw > UINT8_MAX) { satRaw = UINT8_MAX; }
    reading_.satellites = static_cast<uint8_t>(satRaw);
    reading_.hdop       = parseFloat(fields[8]);
    reading_.altitudeM  = parseFloat(fields[9]);

    reading_.fixType = (reading_.satellites >= nmea::MIN_SATS_3D)
                     ? GpsFixType::FIX_3D
                     : GpsFixType::FIX_2D;

    reading_.timestampMs = millis();
    lastFixMs_ = reading_.timestampMs;
    hasFix_ = true;
}

void Bn220Driver::parseRMC(char* fields[], uint8_t count)
{
    (void)count;

    // Field 2: status — 'A' = active, 'V' = void
    if (fields[2][0] != 'A')
    {
        return;
    }

    reading_.speedKmh  = parseFloat(fields[7]) * nmea::KNOTS_TO_KMH;
    reading_.courseDeg = parseFloat(fields[8]);
    reading_.timestampMs = millis();
}

// ── Helpers ─────────────────────────────────────────────
//
// All parse helpers return a safe default (0 / 0.0) on invalid
// input so that callers never operate on uninitialised data.────

float Bn220Driver::parseCoordinate(const char* raw, const char* hem)
{
    if (raw == nullptr || raw[0] == '\0')
    {
        return 0.0f;
    }

    // NMEA: ddmm.mmmm (lat) or dddmm.mmmm (lon)
    float rawVal = strtof(raw, nullptr);
    if (!isfinite(rawVal))
    {
        return 0.0f;
    }
    int32_t degrees = static_cast<int32_t>(rawVal / nmea::DEG_DIVISOR);
    float minutes = rawVal - static_cast<float>(degrees) * nmea::DEG_DIVISOR;
    float decimal = static_cast<float>(degrees) +
                    (minutes / nmea::MINUTES_PER_DEG);

    if (hem != nullptr && (hem[0] == 'S' || hem[0] == 'W'))
    {
        decimal = -decimal;
    }

    return decimal;
}

float Bn220Driver::parseFloat(const char* str)
{
    if (str == nullptr || str[0] == '\0')
    {
        return 0.0f;
    }
    float val = strtof(str, nullptr);
    // CERT-16.2: reject NaN/Inf from malformed NMEA fields
    if (!isfinite(val))
    {
        return 0.0f;
    }
    return val;
}

int32_t Bn220Driver::parseInt(const char* str)
{
    if (str == nullptr || str[0] == '\0')
    {
        return 0;
    }
    return static_cast<int32_t>(strtol(str, nullptr, 10));
}

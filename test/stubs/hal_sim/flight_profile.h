/**
 * @file  flight_profile.h
 * @brief Deterministic flight-data profile for SITL replay drivers.
 *
 * A FlightProfile is a compile-time-sized array of FlightSamples.  Each
 * sample captures a snapshot of every sensor channel at a specific elapsed
 * time.  SITL drivers interpolate linearly between consecutive samples so
 * that the engine sees smooth, physically plausible data at any tick rate
 * without requiring a large table.
 *
 * Design constraints:
 *   - No dynamic allocation (PO10-3).
 *   - Samples must be stored in strictly ascending timeMs order.
 *   - Maximum profile length: SIM_MAX_PROFILE_SAMPLES (compile-time).
 *
 * Thread safety: profiles are read-only after construction; no locking
 *                required (CERT-13).
 */
#pragma once

#include <cstdint>
#include <cmath>

namespace ares
{
namespace sim
{

/// Maximum number of samples per flight profile (PO10-2 bounded array).
static constexpr uint16_t SIM_MAX_PROFILE_SAMPLES = 64U;

/**
 * One sensor snapshot at a point in simulated time.
 *
 * All sensor channels are present in every sample; unused channels
 * should be set to their zero/quiescent values.
 */
struct FlightSample
{
    // ── Time ─────────────────────────────────────────────────────────────────
    uint32_t timeMs = 0U;         ///< Elapsed sim time for this sample (ms).

    // ── GPS ──────────────────────────────────────────────────────────────────
    float gpsLatDeg   = 0.0f;     ///< Latitude  in decimal degrees.
    float gpsLonDeg   = 0.0f;     ///< Longitude in decimal degrees.
    float gpsAltM     = 0.0f;     ///< MSL altitude in metres.
    float gpsSpeedKmh = 0.0f;     ///< Ground speed in km/h.
    uint8_t gpsSats   = 0U;       ///< Satellites in use.
    bool    gpsFix    = false;    ///< True if a valid 3-D fix is available.

    // ── Barometer ─────────────────────────────────────────────────────────────
    float baroAltM    = 0.0f;     ///< Pressure altitude in metres.
    float baroPressurePa = 101325.0f; ///< Absolute pressure in Pascals.
    float baroTempC   = 25.0f;    ///< Air temperature in °C.

    // ── IMU ───────────────────────────────────────────────────────────────────
    float accelX = 0.0f;          ///< Body-frame X acceleration (m/s²).
    float accelY = 0.0f;          ///< Body-frame Y acceleration (m/s²).
    float accelZ = 9.81f;         ///< Body-frame Z acceleration (m/s², ≈g).
    float gyroX  = 0.0f;          ///< Body-frame X angular rate (deg/s).
    float gyroY  = 0.0f;          ///< Body-frame Y angular rate (deg/s).
    float gyroZ  = 0.0f;          ///< Body-frame Z angular rate (deg/s).
    float imuTempC = 25.0f;       ///< Die temperature in °C.
};

/**
 * Complete flight profile: a fixed-length array of FlightSamples and its
 * actual populated count.
 *
 * Typical usage (static initialization):
 * @code
 * static const ares::sim::FlightProfile kLaunchProfile = {
 *     .count = 3,
 *     .samples = {
 *         { .timeMs = 0,    .gpsAltM = 0.0f,   .accelZ = 9.81f },
 *         { .timeMs = 5000, .gpsAltM = 100.0f, .accelZ = 15.0f },
 *         { .timeMs = 10000,.gpsAltM = 300.0f, .accelZ = 9.81f },
 *     }
 * };
 * @endcode
 */
struct FlightProfile
{
    uint16_t    count = 0U;                          ///< Valid entries in samples[].
    FlightSample samples[SIM_MAX_PROFILE_SAMPLES] = {}; ///< Ordered sample array.
};

/**
 * @brief Linearly interpolate a float value between two samples.
 *
 * @param[in] a     Value at the earlier sample.
 * @param[in] b     Value at the later sample.
 * @param[in] t     Interpolation factor in [0.0, 1.0].
 * @return          Linearly blended value.
 */
inline float lerpf(float a, float b, float t)
{
    return a + (b - a) * t;
}

/**
 * @brief Find the FlightSample that corresponds to @p elapsedMs by binary
 *        search and populate @p out with linearly interpolated values.
 *
 * If @p elapsedMs is before the first sample the first sample is returned.
 * If @p elapsedMs is after the last sample the last sample is returned.
 *
 * @param[in]  profile    Profile to query.
 * @param[in]  elapsedMs  Query time in milliseconds.
 * @param[out] out        Interpolated sample.
 */
inline void sampleProfile(const FlightProfile& profile,
                           uint32_t             elapsedMs,
                           FlightSample&        out)
{
    if (profile.count == 0U)
    {
        out = FlightSample{};
        out.timeMs = elapsedMs;
        return;
    }

    // Clamp below the first sample
    if (elapsedMs <= profile.samples[0].timeMs)
    {
        out = profile.samples[0];
        return;
    }

    // Clamp above the last sample
    const uint16_t last = static_cast<uint16_t>(profile.count - 1U);
    if (elapsedMs >= profile.samples[last].timeMs)
    {
        out = profile.samples[last];
        return;
    }

    // Binary search for the surrounding interval [lo, hi]
    uint16_t lo = 0U;
    uint16_t hi = last;

    while ((hi - lo) > 1U)
    {
        const uint16_t mid = static_cast<uint16_t>((lo + hi) / 2U);
        if (profile.samples[mid].timeMs <= elapsedMs)
        {
            lo = mid;
        }
        else
        {
            hi = mid;
        }
    }

    const FlightSample& s0 = profile.samples[lo];
    const FlightSample& s1 = profile.samples[hi];

    const float span = static_cast<float>(s1.timeMs - s0.timeMs);
    const float t    = (span > 0.0f)
                       ? static_cast<float>(elapsedMs - s0.timeMs) / span
                       : 0.0f;

    out.timeMs        = elapsedMs;
    out.gpsLatDeg     = lerpf(s0.gpsLatDeg,     s1.gpsLatDeg,     t);
    out.gpsLonDeg     = lerpf(s0.gpsLonDeg,     s1.gpsLonDeg,     t);
    out.gpsAltM       = lerpf(s0.gpsAltM,       s1.gpsAltM,       t);
    out.gpsSpeedKmh   = lerpf(s0.gpsSpeedKmh,   s1.gpsSpeedKmh,   t);
    out.gpsSats       = (t < 0.5f) ? s0.gpsSats : s1.gpsSats;
    out.gpsFix        = (t < 0.5f) ? s0.gpsFix  : s1.gpsFix;
    out.baroAltM      = lerpf(s0.baroAltM,      s1.baroAltM,      t);
    out.baroPressurePa= lerpf(s0.baroPressurePa, s1.baroPressurePa, t);
    out.baroTempC     = lerpf(s0.baroTempC,     s1.baroTempC,     t);
    out.accelX        = lerpf(s0.accelX,        s1.accelX,        t);
    out.accelY        = lerpf(s0.accelY,        s1.accelY,        t);
    out.accelZ        = lerpf(s0.accelZ,        s1.accelZ,        t);
    out.gyroX         = lerpf(s0.gyroX,         s1.gyroX,         t);
    out.gyroY         = lerpf(s0.gyroY,         s1.gyroY,         t);
    out.gyroZ         = lerpf(s0.gyroZ,         s1.gyroZ,         t);
    out.imuTempC      = lerpf(s0.imuTempC,      s1.imuTempC,      t);
}

} // namespace sim
} // namespace ares

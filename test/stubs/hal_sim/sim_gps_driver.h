/**
 * @file  sim_gps_driver.h
 * @brief Simulation GPS driver — deterministic profile replay (GpsInterface).
 *
 * SimGpsDriver replays a FlightProfile, returning linearly interpolated
 * readings at the current sim-clock time.  Model identifier "SIM_GPS"
 * must be declared in the AMS script as:
 *
 *   include SIM_GPS as GPS
 *
 * Thread safety: NOT thread-safe.  Access from a single task only (CERT-13).
 */
#pragma once

#include "hal/gps/gps_interface.h"
#include "flight_profile.h"
#include "sim_clock.h"

namespace ares
{
namespace sim
{

/**
 * GPS simulator driver.
 *
 * Backed by a FlightProfile, which it samples at each read() call using
 * the current value of ares::sim::clock::nowMs() as the query time.
 */
class SimGpsDriver final : public GpsInterface
{
public:
    /**
     * Construct a SimGpsDriver bound to @p profile.
     * @param[in] profile  Read-only flight profile (must outlive this object).
     */
    explicit SimGpsDriver(const FlightProfile& profile)
        : profile_(profile), ready_(false)
    {}

    // ── GpsInterface ─────────────────────────────────────────────────────────

    bool begin() override
    {
        ready_ = true;
        return true;
    }

    void update() override
    {
        // Nothing to do — readings are generated on demand from the profile.
    }

    GpsStatus read(GpsReading& out) override
    {
        if (!ready_) { return GpsStatus::NOT_READY; }

        FlightSample sample{};
        sampleProfile(profile_, clock::nowMs(), sample);

        out.latitude    = sample.gpsLatDeg;
        out.longitude   = sample.gpsLonDeg;
        out.altitudeM   = sample.gpsAltM;
        out.speedKmh    = sample.gpsSpeedKmh;
        out.courseDeg   = 0.0f;
        out.hdop        = sample.gpsHdop;
        out.timestampMs = clock::nowMs();
        out.satellites  = sample.gpsSats;
        out.fixType     = sample.gpsFix ? GpsFixType::FIX_3D : GpsFixType::NONE;

        return sample.gpsFix ? GpsStatus::OK : GpsStatus::NO_FIX;
    }

    bool hasFix() const override
    {
        FlightSample sample{};
        sampleProfile(profile_, clock::nowMs(), sample);
        return sample.gpsFix;
    }

    const char* driverModel() const override
    {
        return "SIM_GPS";
    }

private:
    const FlightProfile& profile_;
    bool                 ready_;
};

} // namespace sim
} // namespace ares

/**
 * @file  sim_baro_driver.h
 * @brief Simulation barometer driver — deterministic profile replay
 *        (BarometerInterface).
 *
 * Returns linearly interpolated altitude, pressure, and temperature from a
 * FlightProfile at the current sim-clock time.  Declare in AMS scripts as:
 *
 *   include SIM_BARO as BARO
 *
 * Thread safety: NOT thread-safe.  Access from a single task only (CERT-13).
 */
#pragma once

#include "hal/baro/barometer_interface.h"
#include "flight_profile.h"
#include "sim_clock.h"

namespace ares
{
namespace sim
{

/**
 * Barometer simulator driver.
 */
class SimBaroDriver final : public BarometerInterface
{
public:
    /**
     * Construct a SimBaroDriver bound to @p profile.
     * @param[in] profile  Read-only flight profile (must outlive this object).
     */
    explicit SimBaroDriver(const FlightProfile& profile)
        : profile_(profile), ready_(false)
    {}

    // ── BarometerInterface ────────────────────────────────────────────────────

    bool begin() override
    {
        ready_ = true;
        return true;
    }

    BaroStatus read(BaroReading& out) override
    {
        if (!ready_) { return BaroStatus::NOT_READY; }

        FlightSample sample{};
        sampleProfile(profile_, clock::nowMs(), sample);

        out.altitudeM    = sample.baroAltM;
        out.pressurePa   = sample.baroPressurePa;
        out.temperatureC = sample.baroTempC;

        return BaroStatus::OK;
    }

    void setSeaLevelPressure(float /*hPa*/) override
    {
        // No-op in sim — altitude is driven by the profile directly.
    }

    const char* driverModel() const override
    {
        return "SIM_BARO";
    }

private:
    const FlightProfile& profile_;
    bool                 ready_;
};

} // namespace sim
} // namespace ares

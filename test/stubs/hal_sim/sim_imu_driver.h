/**
 * @file  sim_imu_driver.h
 * @brief Simulation IMU driver — deterministic profile replay (ImuInterface).
 *
 * Returns linearly interpolated accelerometer, gyroscope, and temperature
 * data from a FlightProfile at the current sim-clock time.  Declare in AMS
 * scripts as:
 *
 *   include SIM_IMU as IMU
 *
 * Thread safety: NOT thread-safe.  Access from a single task only (CERT-13).
 */
#pragma once

#include "hal/imu/imu_interface.h"
#include "flight_profile.h"
#include "sim_clock.h"

namespace ares
{
namespace sim
{

/**
 * IMU simulator driver.
 */
class SimImuDriver final : public ImuInterface
{
public:
    /**
     * Construct a SimImuDriver bound to @p profile.
     * @param[in] profile  Read-only flight profile (must outlive this object).
     */
    explicit SimImuDriver(const FlightProfile& profile)
        : profile_(profile), ready_(false)
    {}

    // ── ImuInterface ─────────────────────────────────────────────────────────

    bool begin() override
    {
        ready_ = true;
        return true;
    }

    ImuStatus read(ImuReading& out) override
    {
        if (!ready_) { return ImuStatus::NOT_READY; }

        FlightSample sample{};
        sampleProfile(profile_, clock::nowMs(), sample);

        out.accelX = sample.accelX;
        out.accelY = sample.accelY;
        out.accelZ = sample.accelZ;
        out.gyroX  = sample.gyroX;
        out.gyroY  = sample.gyroY;
        out.gyroZ  = sample.gyroZ;
        out.tempC  = sample.imuTempC;

        return ImuStatus::OK;
    }

    const char* driverModel() const override
    {
        return "SIM_IMU";
    }

private:
    const FlightProfile& profile_;
    bool                 ready_;
};

} // namespace sim
} // namespace ares

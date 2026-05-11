/**
 * @file  sim_pulse_driver.h
 * @brief SITL simulation electric-pulse channel driver (no GPIO / no FreeRTOS).
 *
 * Implements PulseInterface for native-platform integration tests.
 * All GPIO and timer operations are replaced with in-memory state so the
 * tests run without hardware dependencies.
 *
 * Test helpers (reset(), setContinuity(), getFireCount(), etc.) allow tests
 * to interrogate and precondition driver state without going through the
 * public PulseInterface.
 */
#pragma once

#include <cstdint>
#include <cstring>

#include "hal/pulse/pulse_interface.h"

namespace ares {
namespace sim {

/**
 * Software-only PulseInterface for SITL integration tests.
 */
class SimPulseDriver : public PulseInterface
{
public:
    SimPulseDriver()  = default;
    ~SimPulseDriver() override = default;

    // ── PulseInterface ─────────────────────────────────────────────────────

    bool begin() override
    {
        reset();
        return true;
    }

    bool fire(uint8_t channel, uint32_t durationMs) override
    {
        if (channel >= PulseChannel::COUNT) { return false; }
        if (fired_[channel]) { return false; }   // safety interlock
        if (durationMs == 0U) { return false; }

        fired_[channel]          = true;
        fireCount_[channel]++;
        lastDurationMs_[channel] = durationMs;
        return true;
    }

    bool readContinuity(uint8_t channel) const override
    {
        if (channel >= PulseChannel::COUNT) { return false; }
        return continuity_[channel];
    }

    bool isFired(uint8_t channel) const override
    {
        if (channel >= PulseChannel::COUNT) { return false; }
        return fired_[channel];
    }

    // ── Test helpers ───────────────────────────────────────────────────────

    /** Reset all channel state to initial values (unfired, continuous). */
    void reset()
    {
        for (uint8_t i = 0U; i < PulseChannel::COUNT; i++)
        {
            fired_[i]          = false;
            fireCount_[i]      = 0U;
            lastDurationMs_[i] = 0U;
            continuity_[i]     = true;
        }
    }

    /**
     * Manually set the continuity state of a channel.
     * @param ch     PulseChannel::CH_A (0) or CH_B (1).
     * @param state  true = circuit intact; false = open circuit.
     */
    void setContinuity(uint8_t ch, bool state)
    {
        if (ch < PulseChannel::COUNT) { continuity_[ch] = state; }
    }

    /**
     * @return Number of successful fire() calls on @p ch since last reset().
     */
    uint8_t getFireCount(uint8_t ch) const
    {
        return (ch < PulseChannel::COUNT) ? fireCount_[ch] : 0U;
    }

    /**
     * @return Duration passed to the last successful fire() on @p ch.
     *         Returns 0 if the channel has never been fired.
     */
    uint32_t getLastDurationMs(uint8_t ch) const
    {
        return (ch < PulseChannel::COUNT) ? lastDurationMs_[ch] : 0U;
    }

private:
    bool     fired_[PulseChannel::COUNT]          = {false, false};
    uint8_t  fireCount_[PulseChannel::COUNT]      = {0U, 0U};
    uint32_t lastDurationMs_[PulseChannel::COUNT] = {0U, 0U};
    bool     continuity_[PulseChannel::COUNT]     = {true, true};
};

} // namespace sim
} // namespace ares

/**
 * @file  sim_buzzer_driver.h
 * @brief SITL simulation buzzer driver (no GPIO / no FreeRTOS).
 *
 * Implements BuzzerInterface for native-platform integration tests.
 * All hardware operations are replaced with in-memory state so the tests run
 * without dependencies on the ESP32 LEDC peripheral or FreeRTOS timers.
 *
 * Test helpers (reset(), getBeepCount(), getLastDurationMs(), getLastFreqHz(),
 * isBeeping()) let tests interrogate driver state without going through the
 * public BuzzerInterface.
 */
#pragma once

#include <cstdint>

#include "hal/buzzer/buzzer_interface.h"

namespace ares {
namespace sim {

/**
 * @brief Software-only BuzzerInterface for SITL integration tests (AMS-4.20).
 */
class SimBuzzerDriver : public BuzzerInterface
{
public:
    SimBuzzerDriver()  = default;
    ~SimBuzzerDriver() override = default;

    // ── BuzzerInterface ────────────────────────────────────────────────────

    bool begin() override
    {
        reset();
        return true;
    }

    /**
     * @brief Record a beep request and mark the driver as busy.
     *
     * Simulates hardware acceptance without actually driving a pin.
     * A second beep() while already busy is accepted and overrides the
     * previous one (mirrors real driver behaviour where the timer is
     * restarted).
     *
     * @param durationMs Beep duration in milliseconds.
     * @param freqHz     Tone frequency in Hz; 0 = driver default.
     * @return Always @c true (driver never rejects in simulation).
     */
    bool beep(uint32_t durationMs, uint32_t freqHz = 0U, uint8_t count = 1U) override
    {
        if (durationMs == 0U) { return false; }

        beepCount_++;
        lastDurationMs_  = durationMs;
        lastFreqHz_      = freqHz;
        lastRepeatCount_ = (count == 0U) ? 1U : count;
        busy_            = true;
        return true;
    }

    void stop() override
    {
        busy_ = false;
    }

    bool isBusy() const override
    {
        return busy_;
    }

    // ── Test helpers ───────────────────────────────────────────────────────

    /** Reset all state to initial values. */
    void reset()
    {
        beepCount_       = 0U;
        lastDurationMs_  = 0U;
        lastFreqHz_      = 0U;
        lastRepeatCount_ = 0U;
        busy_            = false;
    }

    /** @return Total number of successful beep() calls since last reset(). */
    uint32_t getBeepCount() const { return beepCount_; }

    /** @return Duration passed to the most recent beep(); 0 if never called. */
    uint32_t getLastDurationMs() const { return lastDurationMs_; }

    /** @return Frequency passed to the most recent beep(); 0 if default or never called. */
    uint32_t getLastFreqHz() const { return lastFreqHz_; }

    /** @return Repeat count passed to the most recent beep(); 0 if never called. */
    uint8_t getLastRepeatCount() const { return lastRepeatCount_; }

    /** @return true if a beep is conceptually in progress (not yet stop()'d). */
    bool isBeeping() const { return busy_; }

private:
    uint32_t beepCount_       = 0U;
    uint32_t lastDurationMs_  = 0U;
    uint32_t lastFreqHz_      = 0U;
    uint8_t  lastRepeatCount_ = 0U;
    bool     busy_            = false;
};

} // namespace sim
} // namespace ares

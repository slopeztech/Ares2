/**
 * @file  pulse_driver.cpp
 * @brief ESP32 GPIO timed electric-pulse channel driver implementation.
 *
 * Pulse timing uses FreeRTOS one-shot software timers (xTimerCreateStatic).
 * fire() sets the GPIO HIGH synchronously; the timer callback lowers it
 * after the pulse expires, without blocking the calling task (RTOS-2).
 *
 * Safety interlocks:
 *   - Each channel may only be fired once per power cycle.
 *   - Pulse duration is capped at kMaxDurationMs (30 s).
 *   - The driver rejects fire() calls before begin() succeeds.
 */

#include "drivers/pulse/pulse_driver.h"
#include "debug/ares_log.h"

#include <Arduino.h>

static constexpr const char* TAG = "PULSE";

// ── Constructor ──────────────────────────────────────────────────────────────

PulseDriver::PulseDriver(uint8_t pinA,
                         uint8_t pinB,
                         uint8_t contPinA,
                         uint8_t contPinB)
{
    channels_[PulseChannel::CH_A] = { pinA, contPinA, false };
    channels_[PulseChannel::CH_B] = { pinB, contPinB, false };
}

// ── begin ────────────────────────────────────────────────────────────────────

bool PulseDriver::begin()
{
    // Configure fire GPIOs as OUTPUT LOW (safe / unarmed state).
    for (uint8_t ch = 0U; ch < PulseChannel::COUNT; ch++)
    {
        pinMode(channels_[ch].firePin, OUTPUT);
        digitalWrite(channels_[ch].firePin, LOW);

        if (channels_[ch].contPin != kNoPinAssigned)
        {
            pinMode(channels_[ch].contPin, INPUT_PULLUP);
        }
    }

    // Create one-shot FreeRTOS timers (static memory — PO10-3).
    // Period is a placeholder (1 tick); xTimerChangePeriod() sets the
    // real duration before each fire.
    timers_[PulseChannel::CH_A] = xTimerCreateStatic(
        "pulse_ch_a",
        1U,
        pdFALSE,
        &channels_[PulseChannel::CH_A],
        timerCallback,
        &timerBufs_[PulseChannel::CH_A]);

    timers_[PulseChannel::CH_B] = xTimerCreateStatic(
        "pulse_ch_b",
        1U,
        pdFALSE,
        &channels_[PulseChannel::CH_B],
        timerCallback,
        &timerBufs_[PulseChannel::CH_B]);

    if (timers_[PulseChannel::CH_A] == nullptr
        || timers_[PulseChannel::CH_B] == nullptr)
    {
        LOG_E(TAG, "timer creation failed");
        return false;
    }

    ready_ = true;
    LOG_I(TAG, "ready (ch_a=GPIO%u ch_b=GPIO%u)",
          static_cast<uint32_t>(channels_[PulseChannel::CH_A].firePin),
          static_cast<uint32_t>(channels_[PulseChannel::CH_B].firePin));
    return true;
}

// ── fire ─────────────────────────────────────────────────────────────────────

bool PulseDriver::fire(uint8_t channel, uint32_t durationMs)
{
    if (!ready_)
    {
        LOG_E(TAG, "fire rejected: driver not initialised");
        return false;
    }
    if (channel >= PulseChannel::COUNT)
    {
        LOG_E(TAG, "fire rejected: invalid channel %u", static_cast<uint32_t>(channel));
        return false;
    }
    if (durationMs == 0U || durationMs > kMaxDurationMs)
    {
        LOG_E(TAG, "fire rejected: invalid duration %" PRIu32 " ms", durationMs);
        return false;
    }

    ChannelData& ch = channels_[channel];
    if (ch.fired)
    {
        LOG_W(TAG, "fire rejected: channel %u already fired", static_cast<uint32_t>(channel));
        return false;
    }

    // Mark fired before GPIO to guarantee the flag is set even if the
    // timer fires back-to-back (one-shot timer cannot re-arm itself).
    ch.fired = true;

    // Set GPIO HIGH to initiate the pulse.
    digitalWrite(ch.firePin, HIGH);

    // Programme the one-shot timer to pull GPIO LOW after durationMs.
    // xTimerChangePeriod() also starts the timer if it is stopped (RTOS API).
    const BaseType_t ok = xTimerChangePeriod(
        timers_[channel],
        pdMS_TO_TICKS(durationMs),
        0U);  // do not block — timer daemon must be running

    if (ok != pdPASS)
    {
        // Timer queue full; manually lower GPIO to avoid stuck-HIGH.
        digitalWrite(ch.firePin, LOW);
        LOG_E(TAG, "timer start failed for channel %u — GPIO forced LOW",
              static_cast<uint32_t>(channel));
        return false;
    }

    LOG_I(TAG, "channel %u fired (pin=%u dur=%" PRIu32 "ms)",
          static_cast<uint32_t>(channel),
          static_cast<uint32_t>(ch.firePin),
          durationMs);
    return true;
}

// ── readContinuity ───────────────────────────────────────────────────────────

bool PulseDriver::readContinuity(uint8_t channel) const
{
    if (channel >= PulseChannel::COUNT) { return false; }
    const ChannelData& ch = channels_[channel];

    // If a dedicated continuity-sense pin is wired, read it directly.
    if (ch.contPin != kNoPinAssigned)
    {
        return (digitalRead(ch.contPin) == HIGH);
    }

    // Fallback: assume continuity is intact until the channel is fired.
    return !ch.fired;
}

// ── isFired ──────────────────────────────────────────────────────────────────

bool PulseDriver::isFired(uint8_t channel) const
{
    if (channel >= PulseChannel::COUNT) { return false; }
    return channels_[channel].fired;
}

// ── timerCallback ────────────────────────────────────────────────────────────

void PulseDriver::timerCallback(TimerHandle_t xTimer)
{
    // pvTimerGetTimerID returns the ChannelData pointer set at timer creation.
    const ChannelData* ch = static_cast<const ChannelData*>(pvTimerGetTimerID(xTimer));
    if (ch != nullptr)
    {
        // Drive GPIO LOW to end the fire pulse.
        // This is the only operation in the callback — no mutex, no logging.
        digitalWrite(ch->firePin, LOW);
    }
}

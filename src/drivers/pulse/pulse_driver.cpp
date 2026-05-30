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

PulseDriver::PulseDriver(const ChannelConfig (&cfg)[PulseChannel::COUNT])
{
    for (uint8_t i = 0U; i < PulseChannel::COUNT; i++)
    {
        channels_[i] = { cfg[i].firePin, cfg[i].contPin, false };
    }
}

// ── begin ────────────────────────────────────────────────────────────────────

bool PulseDriver::begin()
{
    // Timer name prefix — indexed below per channel.
    static const char* const kTimerNames[PulseChannel::COUNT] = {
        "pulse_ch_a", "pulse_ch_b", "pulse_ch_c", "pulse_ch_d"
    };

    // Configure fire GPIOs as OUTPUT LOW (safe / unarmed state).
    // Skip channels whose firePin is kNoPinAssigned (not wired).
    for (uint8_t ch = 0U; ch < PulseChannel::COUNT; ch++)
    {
        if (channels_[ch].firePin == kNoPinAssigned) { continue; }

        // Pre-load the output latch LOW *before* switching direction to OUTPUT.
        // This eliminates the theoretical glitch window where the pin could
        // momentarily be HIGH when transitioning from INPUT to OUTPUT.
        // ESP32 output register defaults to LOW after reset, but this makes
        // the safe state explicit and independent of reset-state assumptions.
        digitalWrite(channels_[ch].firePin, LOW);
        pinMode(channels_[ch].firePin, OUTPUT);

        if (channels_[ch].contPin != kNoPinAssigned)
        {
            pinMode(channels_[ch].contPin, INPUT_PULLUP);
        }
    }

    // Create one-shot FreeRTOS timers (static memory — PO10-3).
    // Period is a placeholder (1 tick); xTimerChangePeriod() sets the
    // real duration before each fire.  Skip unwired channels.
    for (uint8_t ch = 0U; ch < PulseChannel::COUNT; ch++)
    {
        if (channels_[ch].firePin == kNoPinAssigned)
        {
            timers_[ch] = nullptr;
            continue;
        }
        timers_[ch] = xTimerCreateStatic(
            kTimerNames[ch],
            1U,
            pdFALSE,
            &channels_[ch],
            timerCallback,
            &timerBufs_[ch]);

        if (timers_[ch] == nullptr)
        {
            LOG_E(TAG, "timer creation failed for channel %u", static_cast<uint32_t>(ch));
            // Rollback: remove from the timer service all timers already
            // created during this begin() call so the driver is left in a
            // fully defined invalid state (no orphaned timer handles).
            // Only clear the handle if xTimerDelete succeeds; if it fails
            // the handle remains reachable so it is not permanently orphaned.
            for (uint8_t i = 0U; i < ch; i++)
            {
                if (timers_[i] != nullptr)
                {
                    if (xTimerDelete(timers_[i], pdMS_TO_TICKS(100U)) == pdPASS)
                    {
                        timers_[i] = nullptr;
                    }
                    else
                    {
                        LOG_E(TAG, "timer delete failed for channel %u", static_cast<uint32_t>(i));
                    }
                }
            }
            return false;
        }
    }

    ready_ = true;
    {
        // Build per-channel pin labels; print "--" for unassigned channels so
        // "GPIO255" never appears in the ready message.
        char la[8]; char lb[8]; char lc[8]; char ld[8];
        const auto pinStr = [](char* buf, size_t sz, uint8_t pin) -> const char* {
            if (pin == kNoPinAssigned) { return "--"; }
            (void)snprintf(buf, sz, "GPIO%u", static_cast<uint32_t>(pin));
            return buf;
        };
        LOG_I(TAG, "ready (a=%s b=%s c=%s d=%s)",
              pinStr(la, sizeof(la), channels_[PulseChannel::CH_A].firePin),
              pinStr(lb, sizeof(lb), channels_[PulseChannel::CH_B].firePin),
              pinStr(lc, sizeof(lc), channels_[PulseChannel::CH_C].firePin),
              pinStr(ld, sizeof(ld), channels_[PulseChannel::CH_D].firePin));
    }
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
    if (ch.firePin == kNoPinAssigned)
    {
        LOG_W(TAG, "fire rejected: channel %u not wired", static_cast<uint32_t>(channel));
        return false;
    }
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

// ── hasContPin ───────────────────────────────────────────────────────────────

bool PulseDriver::hasContPin(uint8_t channel) const
{
    if (channel >= PulseChannel::COUNT) { return false; }
    return channels_[channel].contPin != kNoPinAssigned;
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

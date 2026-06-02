/**
 * @file  buzzer_interface.h
 * @brief Hardware-agnostic buzzer output interface (pure virtual).
 *
 * Abstracts a single buzzer output that can produce timed audio beeps.
 * Two concrete drivers exist:
 *   - ActiveBuzzerDriver  — digital GPIO on/off; the buzzer has an internal
 *                           oscillator so no frequency control is needed.
 *   - PassiveBuzzerDriver — ESP32 LEDC PWM tone; frequency is programmable.
 *
 * Callers that target an active buzzer should pass @p freqHz = 0 (the
 * default), which instructs the driver to use its built-in default tone.
 * Callers that target a passive buzzer can supply an explicit frequency.
 *
 * Thread safety: Implementations are NOT thread-safe.
 *                The AMS engine calls beep() and stop() inside a
 *                mutex-protected section (CERT-13).
 *
 * @note  AMS feature reference: AMS-4.20 (BUZZER.beep action).
 */
#pragma once

#include <cstdint>

namespace ares
{

/**
 * @brief Abstract buzzer output interface.
 *
 * @par Safety invariants
 *   - beep() with durationMs == 0 must return false (no-op).
 *   - stop() must always be safe to call even if no beep is in progress.
 *   - All implementations must silently ignore out-of-range parameters
 *     rather than invoking undefined behaviour.
 */
class BuzzerInterface
{
public:
    virtual ~BuzzerInterface() = default;

    // Non-copyable, non-movable (CERT-18.3)
    BuzzerInterface(const BuzzerInterface&)            = delete;
    BuzzerInterface& operator=(const BuzzerInterface&) = delete;
    BuzzerInterface(BuzzerInterface&&)                 = delete;
    BuzzerInterface& operator=(BuzzerInterface&&)      = delete;

    /**
     * @brief Initialise the buzzer hardware.
     *
     * Configures GPIO and/or LEDC peripheral.  Must be called before any
     * other method.
     *
     * @return true on success; false on hardware initialisation failure.
     */
    virtual bool begin() = 0;

    /**
     * @brief Start a non-blocking timed beep (with optional repeat).
     *
     * The driver starts the output immediately and stops it automatically
     * after @p durationMs milliseconds.  If @p count > 1 the driver
     * repeats the beep @p count times, inserting a silent gap of
     * @p durationMs between each repetition (equal on/off duty cycle).
     *
     * @param[in] durationMs  Beep duration in milliseconds (> 0).
     * @param[in] freqHz      Tone frequency in Hz.  Pass 0 to use the
     *                        driver's built-in default (active buzzers and
     *                        passive buzzers with a compile-time default).
     *                        Ignored by active buzzer drivers.
     * @param[in] count       Number of beeps to produce.  1 = single beep
     *                        (default); 2–BUZZER_MAX_REPEAT_COUNT = repeat
     *                        with equal-length gaps.  Values outside the
     *                        valid range are clamped silently.
     *
     * @pre   begin() returned true.
     * @pre   durationMs > 0.
     * @return true if the beep sequence was started; false if durationMs == 0,
     *         hardware not initialised, or the driver declined the call.
     */
    virtual bool beep(uint32_t durationMs, uint32_t freqHz = 0U, uint8_t count = 1U) = 0;

    /**
     * @brief Stop any beep in progress immediately.
     *
     * Safe to call at any time, including when no beep is active.
     *
     * @pre   begin() returned true.
     */
    virtual void stop() = 0;

    /**
     * @brief Query whether a beep sequence is currently in progress.
     *
     * For multi-beep sequences this includes the silent gap phase between tones.
     *
     * @return true if the driver is busy with a beep sequence; false otherwise.
     */
    virtual bool isBusy() const = 0;

protected:
    BuzzerInterface() = default;  ///< Protected: only subclasses may construct.
};

} // namespace ares

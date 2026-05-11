/**
 * @file  pulse_driver.h
 * @brief ESP32 GPIO timed electric-pulse channel driver (FreeRTOS timer-based).
 *
 * Implements PulseInterface for two GPIO-driven channels (A and B).
 * Each channel drives a deployment actuator via a digital output.
 *
 * Pulse timing is handled by FreeRTOS one-shot software timers so
 * fire() returns immediately without blocking the calling task (RTOS-2).
 *
 * Optional separate continuity-sense input pins may be wired to a
 * resistive bridge circuit to detect an open circuit.  Pass
 * @c kNoPinAssigned to the constructor for channels without dedicated
 * continuity hardware; in that case readContinuity() returns @c false
 * when the channel has been fired and @c true otherwise (optimistic
 * pre-fire assumption).
 *
 * Thread safety: NOT thread-safe.  Must be accessed from a single task
 *                or protected externally (CERT-13).  The FreeRTOS timer
 *                callback runs from the timer daemon task; it only drives
 *                GPIO LOW and does not touch shared driver state.
 */
#pragma once

#include <cstdint>
#include <freertos/FreeRTOS.h>
#include <freertos/timers.h>

#include "hal/pulse/pulse_interface.h"

/**
 * Concrete PulseInterface for two GPIO-controlled output channels.
 *
 * Instantiated statically in main.cpp with the pin numbers from config.h.
 * No heap allocation is used (PO10-3): FreeRTOS timers are created with
 * xTimerCreateStatic using per-instance static buffers.
 */
class PulseDriver : public PulseInterface
{
public:
    /// Sentinel value for an unconnected continuity-sense pin.
    static constexpr uint8_t kNoPinAssigned = 0xFFU;

    /**
     * Construct the driver.
     *
     * @param[in] pinA      GPIO output for channel A.
     * @param[in] pinB      GPIO output for channel B.
     * @param[in] contPinA  GPIO input for channel A continuity sense
     *                      (kNoPinAssigned = not wired).
     * @param[in] contPinB  GPIO input for channel B continuity sense
     *                      (kNoPinAssigned = not wired).
     */
    explicit PulseDriver(uint8_t pinA,
                         uint8_t pinB,
                         uint8_t contPinA = kNoPinAssigned,
                         uint8_t contPinB = kNoPinAssigned);

    // Non-copyable, non-movable (CERT-18.3)
    PulseDriver(const PulseDriver&)            = delete;
    PulseDriver& operator=(const PulseDriver&) = delete;
    PulseDriver(PulseDriver&&)                 = delete;
    PulseDriver& operator=(PulseDriver&&)      = delete;

    /**
     * Initialise GPIO outputs and create FreeRTOS one-shot timers.
     *
     * @return true on success; false if timer creation fails.
     * @post Both fire GPIOs are OUTPUT LOW (safe / unarmed state).
     */
    bool begin() override;

    /**
     * Fire a channel.
     *
     * Sets @p channel HIGH immediately then starts a one-shot FreeRTOS
     * timer that drives it LOW after @p durationMs milliseconds.
     *
     * @param[in] channel     PulseChannel::CH_A (0) or CH_B (1).
     * @param[in] durationMs  Pulse duration in milliseconds (> 0, ≤ 30 000).
     * @return true on success; false if channel is already fired, invalid,
     *         or the driver is not initialised.
     */
    bool fire(uint8_t channel, uint32_t durationMs) override;

    /**
     * Read circuit continuity for a channel.
     *
     * If a continuity-sense pin was provided at construction, reads that
     * GPIO (HIGH = circuit intact).  Otherwise returns @c !isFired(channel).
     *
     * @param[in] channel  PulseChannel::CH_A (0) or CH_B (1).
     * @return true if the circuit appears continuous.
     */
    bool readContinuity(uint8_t channel) const override;

    /**
     * Query whether a channel has been fired this session.
     *
     * @param[in] channel  PulseChannel::CH_A (0) or CH_B (1).
     * @return true after a successful fire() call for this channel.
     */
    bool isFired(uint8_t channel) const override;

private:
    /// Maximum permitted pulse duration (safety cap).
    static constexpr uint32_t kMaxDurationMs = 30000U;

    /**
     * Per-channel runtime data passed as timer ID to the callback.
     * Allocated in-place as a driver member (PO10-3 — no heap).
     */
    struct ChannelData
    {
        uint8_t firePin;    ///< GPIO driven HIGH to initiate the pulse.
        uint8_t contPin;    ///< GPIO read for continuity (kNoPinAssigned = none).
        bool    fired;      ///< true after a successful fire().
    };

    /**
     * Timer daemon callback — drives the fire pin LOW to end the pulse.
     *
     * Called from the FreeRTOS timer task; performs only a single GPIO write.
     * @param[in] xTimer  Handle of the expiring timer (ID = ChannelData*).
     */
    static void timerCallback(TimerHandle_t xTimer);

    ChannelData   channels_[PulseChannel::COUNT]  = {};               ///< Per-channel state.
    StaticTimer_t timerBufs_[PulseChannel::COUNT] = {};               ///< Static timer memory.
    TimerHandle_t timers_[PulseChannel::COUNT]    = {nullptr, nullptr};  ///< Timer handles.
    bool          ready_ = false;                                     ///< true after successful begin().
};

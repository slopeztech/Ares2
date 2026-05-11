/**
 * @file  pulse_interface.h
 * @brief Hardware-agnostic timed electric-pulse channel interface (pure virtual).
 *
 * Abstracts two generic electric-pulse output channels (A and B),
 * each driven by a timed electric pulse.  Concrete drivers (e.g. PulseDriver) implement this
 * interface; application code (AMS engine, REST API) accesses channel state
 * only through this interface.
 *
 * Thread safety: Implementations are NOT thread-safe.
 *                The AMS engine calls fire() inside a mutex-protected
 *                section; the API task calls readContinuity() / isFired()
 *                as read-only, best-effort queries (CERT-13).
 */
#pragma once

#include <cstdint>

/**
 * Pulse channel identifiers.
 *
 * Used as the @p channel argument to PulseInterface methods.
 * Values are guaranteed to be contiguous [0, COUNT).
 */
struct PulseChannel
{
    static constexpr uint8_t CH_A  = 0U;  ///< Pulse channel A.
    static constexpr uint8_t CH_B  = 1U;  ///< Pulse channel B.
    static constexpr uint8_t COUNT = 2U;  ///< Total number of channels.

    PulseChannel()                               = delete;  // utility struct — no instances
};

/**
 * Abstract timed electric-pulse channel interface.
 *
 * Provides a minimal surface for firing channels and querying their
 * continuity / fired status.  The AMS engine uses fire() to actuate
 * channels during @c on_enter: execution; the REST API uses
 * readContinuity() and isFired() to populate @c GET /api/pulse/status.
 *
 * Safety invariants enforced at the interface boundary:
 *   - fire() is a one-way operation per channel: once a channel has been
 *     fired (@c isFired() == true), subsequent fire() calls are rejected
 *     and return @c false.
 *   - readContinuity() is always passive (never triggers a fire).
 *   - Channel indices outside [0, PulseChannel::COUNT) always return false /
 *     have no side effects.
 */
class PulseInterface
{
public:
    virtual ~PulseInterface() = default;

    // Non-copyable, non-movable (CERT-18.3)
    PulseInterface(const PulseInterface&)            = delete;
    PulseInterface& operator=(const PulseInterface&) = delete;
    PulseInterface(PulseInterface&&)                 = delete;
    PulseInterface& operator=(PulseInterface&&)      = delete;

    /**
     * Initialise the channel hardware.
     *
     * Configures GPIO pins as OUTPUT LOW (safe default) and prepares
     * any hardware timers used for timed pulse generation.
     *
     * @pre  GPIO subsystem is initialised.
     * @post All channels are in the safe (unarmed) state on success.
     * @return true on success, false on hardware initialisation failure.
     */
    virtual bool begin() = 0;

    /**
     * Fire a channel with the specified pulse duration.
     *
     * Sets the channel output HIGH for @p durationMs milliseconds then
     * returns LOW.  The pulse is non-blocking — the output goes HIGH
     * immediately; a hardware timer lowers it after the pulse expires.
     *
     * @param[in] channel     Channel index (PulseChannel::CH_A or CH_B).
     * @param[in] durationMs  Pulse duration in milliseconds (> 0).
     * @pre   begin() returned true.
     * @pre   channel < PulseChannel::COUNT.
     * @pre   durationMs > 0.
     * @post  isFired(channel) == true on success.
     * @return true on success; false if channel is invalid, already fired,
     *         durationMs == 0, or hardware not initialised.
     * @note  This operation is irreversible.  See safety invariants above.
     */
    virtual bool fire(uint8_t channel, uint32_t durationMs) = 0;

    /**
     * Read the continuity state of a channel.
     *
     * A @c true return indicates that the circuit is closed
     * (the firing circuit is intact and connected).
     * A @c false return indicates an open circuit or that the channel
     * has already been fired.
     *
     * This is always a passive, read-only operation.
     *
     * @param[in] channel  Channel index (PulseChannel::CH_A or CH_B).
     * @return true if the circuit appears continuous; false otherwise.
     * @note  Not guaranteed to detect partial bridgewire resistance changes.
     */
    virtual bool readContinuity(uint8_t channel) const = 0;

    /**
     * Query whether a channel has been fired in the current session.
     *
     * @param[in] channel  Channel index (PulseChannel::CH_A or CH_B).
     * @return true if fire() has been called and accepted for this channel.
     * @note  State persists for the lifetime of the driver object;
     *        it is not cleared on deactivate() or reboot.
     */
    virtual bool isFired(uint8_t channel) const = 0;

protected:
    PulseInterface() = default;  ///< Protected: only subclasses may construct.
};

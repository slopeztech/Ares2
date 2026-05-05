/**
 * @file  radio_dispatcher.h
 * @brief Incoming APUS frame dispatcher — receive path (APUS-4.4, APUS-9, APUS-14).
 *
 * Polls the primary radio interface every loop iteration, accumulates raw bytes
 * in a static ring buffer, and extracts complete APUS frames using the
 * synchronisation-marker framing scheme (APUS-4.3).
 *
 * For every valid COMMAND frame the dispatcher:
 *   1. Detects duplicate frames by SEQ number (APUS-4.7).
 *   2. Sends an acceptance ACK unconditionally (APUS-9.1).
 *   3. Executes the command via MissionScriptEngine.
 *   4. Sends a completion ACK or NACK if FLAG_ACK_REQ was set (APUS-9.3).
 *
 * HEARTBEAT frames are echoed back to maintain the ST[17] connection-test
 * handshake.  TELEMETRY, EVENT, and ACK frames received from ground are logged
 * at DEBUG level and discarded (APUS-5.1, APUS-14.2).
 *
 * Standards compliance:
 *   APUS-4.4  — receive path polled every main-loop iteration.
 *   APUS-4.7  — duplicate SEQ detection on the rocket side.
 *   APUS-9.1  — acceptance ACK sent for every valid COMMAND.
 *   APUS-9.2  — NACK sent with FailureCode on every rejection.
 *   APUS-9.3  — completion ACK/NACK gated by FLAG_ACK_REQ.
 *   APUS-9.5  — originalSeq in ACK/NACK matches incoming SEQ.
 *   APUS-14.1 — O(1) dispatch via switch (no linear search).
 *   APUS-14.2 — every routing decision logged at DEBUG level.
 *   APUS-14.3 — routing failures generate NACK when source known.
 *
 * Coding standards: MISRA C++ 2023 Rule 6.4 (bounded loops), PO10-3 (no heap).
 */
#pragma once

#include "comms/ares_radio_protocol.h"
#include "ams/mission_script_engine.h"
#include "hal/radio/radio_interface.h"

#include <cstdint>

namespace ares
{

/**
 * @brief Transport-level APUS frame dispatcher for the rocket receive path.
 *
 * Statically allocated — no heap usage (PO10-3).
 * Must be polled from a single RTOS task (not thread-safe internally;
 * all MissionScriptEngine calls are individually mutex-guarded).
 */
class RadioDispatcher
{
public:
    /**
     * @param[in] radio   Primary radio interface used for both RX and TX.
     * @param[in] engine  Mission script engine used for command injection.
     */
    RadioDispatcher(RadioInterface&              radio,
                    ares::ams::MissionScriptEngine& engine);

    /**
     * @brief Drain available radio bytes, decode complete APUS frames,
     *        and dispatch each frame to its handler.
     *
     * Non-blocking — returns immediately if no bytes are available.
     * Must be called once per main-loop iteration (APUS-4.4).
     *
     * @param[in] nowMs  Current millis() timestamp.
     */
    void poll(uint32_t nowMs);

private:
    // Two full frames of buffer space so a partial-frame tail can coexist
    // with a newly arriving complete frame without blocking RX.
    static constexpr uint16_t kRxBufLen =
        static_cast<uint16_t>(proto::MAX_FRAME_LEN) * 2U;

    // Maximum frames extracted per poll() call (PO10-2: bounded loop).
    static constexpr uint16_t kMaxFramesPerPoll =
        kRxBufLen / static_cast<uint16_t>(proto::MIN_FRAME_LEN);

    static_assert(kRxBufLen >= (2U * static_cast<uint16_t>(proto::MAX_FRAME_LEN)),
                  "RX buffer must fit at least two maximum-size frames");

    // ── References ──────────────────────────────────────────
    RadioInterface&               radio_;
    ares::ams::MissionScriptEngine& engine_;

    // ── Receive state ────────────────────────────────────────
    uint8_t  rxBuf_[kRxBufLen];  ///< Byte accumulation buffer (static, PO10-3).
    uint16_t rxLen_     = 0U;    ///< Number of valid bytes currently in rxBuf_.
    uint8_t  txSeq_     = 0U;    ///< TX sequence counter for outgoing ACK/NACK/HBEAT frames.
    uint8_t  lastRxSeq_ = 0U;    ///< SEQ of the last accepted COMMAND (APUS-4.7).
    bool     seenFirst_ = false; ///< True after the first COMMAND is accepted.

    // ── Internal helpers ─────────────────────────────────────

    /**
     * @brief Scan rxBuf_ for complete APUS frames and dispatch each one.
     *
     * Searches for the SYNC marker, validates declared payload length,
     * attempts full decode(), and either dispatches or skips forward.
     * Compacts the buffer in-place after each consumed frame.
     *
     * @param[in] nowMs  Forwarded to dispatchFrame().
     */
    void processBuffer(uint32_t nowMs);

    /**
     * @brief Route a decoded frame to the correct handler (APUS-14.1).
     *
     * Verifies that the destination NODE is NODE_ROCKET or NODE_BROADCAST
     * (APUS-10.4), then dispatches by TYPE via O(1) switch.
     *
     * @param[in] frame  Fully decoded APUS frame.
     * @param[in] nowMs  Current millis() timestamp.
     */
    void dispatchFrame(const proto::Frame& frame, uint32_t nowMs);

    /**
     * @brief Handle a TYPE == COMMAND frame (APUS-7, APUS-9).
     *
     * Sequence:
     *   1. Duplicate-SEQ check — discard silently if duplicate (APUS-4.7).
     *   2. Acceptance ACK — sent unconditionally (APUS-9.1).
     *   3. executeCommand() — returns FailureCode result.
     *   4. Completion ACK/NACK — sent only if FLAG_ACK_REQ (APUS-9.3).
     *
     * @param[in] frame  Decoded COMMAND frame.
     * @param[in] nowMs  Current millis() timestamp.
     */
    void handleCommand(const proto::Frame& frame, uint32_t nowMs);

    /**
     * @brief Handle a TYPE == HEARTBEAT frame — echo back to sender.
     *
     * Builds a HEARTBEAT response with NODE_ROCKET and increments txSeq_.
     *
     * @param[in] frame  Decoded HEARTBEAT frame.
     * @param[in] nowMs  Current millis() timestamp (unused, reserved).
     */
    void handleHeartbeat(const proto::Frame& frame, uint32_t nowMs);

    /**
     * @brief Build and transmit an ACK or NACK frame (APUS-9).
     *
     * Populates an AckPayload with @p code and @p failureData, wraps it
     * in an APUS Frame with TYPE == ACK, and calls encode() + radio_.send().
     * FailureCode::NONE produces an ACK; any other value produces a NACK.
     *
     * @param[in] originalSeq   SEQ of the frame being verified (APUS-9.5).
     * @param[in] originalNode  NODE of the frame sender.
     * @param[in] code          FailureCode::NONE = ACK; non-zero = NACK.
     * @param[in] failureData   Command-specific diagnostic byte (may be 0).
     */
    void sendAckNack(uint8_t            originalSeq,
                     uint8_t            originalNode,
                     proto::FailureCode code,
                     uint8_t            failureData);

    /**
     * @brief Execute the command encoded in a decoded COMMAND frame.
     *
     * Validates arming pre-conditions (APUS-7.2), checks CRITICAL-flag
     * requirement (APUS-7.3), and injects recognised commands into the
     * mission engine.
     *
     * @param[in] frame  Decoded COMMAND frame (len >= sizeof(CommandHeader)).
     * @param[in] nowMs  Current millis() timestamp (used for immediate TM responses).
     * @return FailureCode::NONE on success, or a specific FailureCode on error.
     */
    proto::FailureCode executeCommand(const proto::Frame& frame, uint32_t nowMs);
};

} // namespace ares

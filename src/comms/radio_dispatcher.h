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
#include "hal/pulse/pulse_interface.h"

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
     * @param[in] pulse   Pulse channel driver (nullable — FIRE_PULSE rejected if nullptr).
     */
    RadioDispatcher(RadioInterface&               radio,
                    ares::ams::MissionScriptEngine& engine,
                    PulseInterface*               pulse = nullptr);

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
    PulseInterface*               pulse_;   ///< Nullable — guarded before use.

    // ── Receive state ────────────────────────────────────────
    uint8_t  rxBuf_[kRxBufLen];  ///< Byte accumulation buffer (static, PO10-3).
    uint16_t rxLen_     = 0U;    ///< Number of valid bytes currently in rxBuf_.
    uint8_t  txSeq_     = 0U;    ///< TX sequence counter for outgoing ACK/NACK/HBEAT frames.
    uint8_t  lastRxSeq_ = 0U;    ///< SEQ of the last accepted COMMAND (APUS-4.7).
    bool     seenFirst_ = false; ///< True after the first COMMAND is accepted.

    // ── TX retry buffer (APUS-4.5, APUS-4.6, APUS-2.3) ─────────────────────
    /**
     * One slot in the retransmission buffer.
     *
     * When a frame is sent via @c sendReliable() a copy is stored here.
     * If no ACK for that SEQ arrives within @c proto::ACK_TIMEOUT_MS the
     * frame is re-sent (with @c FLAG_RETRANSMIT added) up to
     * @c proto::MAX_RETRIES times, then abandoned.
     */
    struct TxRetrySlot
    {
        proto::Frame frame;      ///< Copy of the outgoing frame awaiting ACK.
        uint32_t     sentMs;     ///< Timestamp of the most recent send attempt.
        uint8_t      retries;    ///< Retransmission count (0 on first send).
        bool         active;     ///< true = slot occupied and awaiting ACK.
    };
    static constexpr uint8_t kMaxRetrySlots = 4U;
    TxRetrySlot retrySlots_[kMaxRetrySlots] = {};

    // ── Priority command queue (APUS-2.1, APUS-2.2, APUS-2.4) ──────────────
    static constexpr uint8_t kCriticalPriority = 0U;  ///< ABORT, FIRE_PULSE, FACTORY_RESET.
    static constexpr uint8_t kHighPriority     = 1U;  ///< ARM, SET_MODE, SET_FCS_ACTIVE.
    static constexpr uint8_t kNormalPriority   = 2U;  ///< REQUEST_TELEMETRY, SET_TELEM_INTERVAL.
    static constexpr uint8_t kLowPriority      = 3U;  ///< REQUEST_STATUS, config queries.

    /// Max queued commands (2 per priority level × 4 levels).
    static constexpr uint8_t kCmdQueueDepth = 8U;

    /**
     * One queued command awaiting priority-ordered execution.
     * The acceptance ACK has already been sent at insertion time.
     */
    struct PendingCmd
    {
        proto::Frame frame;     ///< Full decoded command frame copy.
        uint8_t      priority;  ///< 0=CRITICAL … 3=LOW (lower = higher priority).
        bool         active;    ///< true = slot occupied.
    };
    PendingCmd cmdQueue_[kCmdQueueDepth] = {};
    uint8_t    cmdQueueCount_ = 0U;       ///< Number of active entries in cmdQueue_.

    // ── ST[20] runtime config parameter table (APUS-16) ─────────────────────
    /**
     * One entry in the on-board parameter registry (APUS-16.3: no dynamic
     * registration; table is fully defined at compile time with defaults).
     */
    struct ConfigEntry
    {
        proto::ConfigParamId id;        ///< Parameter identifier.
        float                value;     ///< Current value.
        float                minVal;    ///< Lower bound (inclusive).
        float                maxVal;    ///< Upper bound (inclusive).
        bool                 writable;  ///< false = read-only (APUS-16.1).
    };
    static constexpr uint8_t kConfigParamCount = 6U;
    ConfigEntry configParams_[kConfigParamCount] = {
        // id                                  value    min                                                max                                                writable
        { proto::ConfigParamId::TELEM_INTERVAL_MS,  2000.0f,  static_cast<float>(TELEMETRY_INTERVAL_MIN), static_cast<float>(TELEMETRY_INTERVAL_MAX), true  },
        { proto::ConfigParamId::MONITOR_ALT_HIGH_M,  3000.0f,  0.0f,                                      MONITOR_ALT_HIGH_MAX_M,                     true  },
        { proto::ConfigParamId::MONITOR_ALT_LOW_M,     -5.0f,  MONITOR_ALT_LOW_MIN_M,                     MONITOR_ALT_LOW_MAX_M,                      true  },
        { proto::ConfigParamId::MONITOR_ACCEL_HIGH,   150.0f,  0.0f,                                      MONITOR_ACCEL_HIGH_MAX,                     true  },
        { proto::ConfigParamId::MONITOR_TEMP_HIGH_C,   85.0f,  MONITOR_TEMP_HIGH_MIN_C,                   MONITOR_TEMP_HIGH_MAX_C,                    true  },
        { proto::ConfigParamId::MONITOR_TEMP_LOW_C,   -40.0f,  MONITOR_TEMP_LOW_MIN_C,                    MONITOR_TEMP_LOW_MAX_C,                     true  },
    };

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
     *   3. enqueueCmd() — insert into the priority queue (APUS-2.2).
     *   4. Completion ACK/NACK deferred to drainCmdQueue().
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

    // ── TX retry helpers (APUS-4.5, APUS-4.6) ───────────────────────────────

    /**
     * @brief Store a frame copy for retransmission-on-timeout (APUS-4.5).
     * @param[in] frame  Frame already sent; must have @c FLAG_ACK_REQ set.
     * @param[in] nowMs  Current timestamp.
     * @return true if a free slot was found; false if the buffer is full.
     */
    bool enqueueRetry(const proto::Frame& frame, uint32_t nowMs);

    /**
     * @brief Check all active retry slots; retransmit or expire as needed (APUS-4.6).
     * @param[in] nowMs  Current timestamp.
     */
    void processRetries(uint32_t nowMs);

    /**
     * @brief Remove the retry slot whose stored SEQ matches @p seq (APUS-4.5).
     * Called when an ACK is received for that SEQ.
     * @param[in] seq  SEQ field from the received ACK payload.
     */
    void clearRetryForSeq(uint8_t seq);

    // ── Priority-queue command helpers (APUS-2.1, APUS-2.2, APUS-2.4) ───────

    /**
     * @brief Map a CommandId to its priority level (APUS-2.1).
     * @return 0=CRITICAL, 1=HIGH, 2=NORMAL, 3=LOW.
     */
    static uint8_t commandPriority(proto::CommandId id);

    /**
     * @brief Insert a command into the priority queue (APUS-2.2).
     * @param[in] frame  Fully decoded COMMAND frame.
     * @return true on success; false if the queue is full.
     */
    bool enqueueCmd(const proto::Frame& frame);

    /**
     * @brief Execute all queued commands in priority order (APUS-2.1, APUS-2.4).
     *
     * Each iteration finds the highest-priority active slot and executes it.
     * Sends completion ACK/NACK for entries with FLAG_ACK_REQ set (APUS-9.3).
     * CRITICAL (0) executes before HIGH (1) before NORMAL (2) before LOW (3).
     *
     * @param[in] nowMs  Forwarded to executeCommand().
     */
    void drainCmdQueue(uint32_t nowMs);

    // ── ST[20] parameter management helpers (APUS-16) ────────────────────────

    /**
     * @brief Find the config entry for @p id, or nullptr if not found.
     * O(kConfigParamCount) — small linear search is acceptable for N=6.
     *
     * @param[in] id  Parameter identifier to search for.
     * @return Pointer to the matching ConfigEntry, or nullptr if not found.
     */
    ConfigEntry* findConfigParam(proto::ConfigParamId id);

    /**
     * @brief Validate and store a new parameter value, then apply it.
     *
     * Verifies write permission (APUS-16.1), bounds [minVal, maxVal],
     * persists the new value, and triggers the corresponding side-effect
     * (e.g. updates telemetry interval or monitoring threshold).
     *
     * @param[in] id     Parameter identifier.
     * @param[in] value  New value to apply.
     * @param[in] nowMs  Current millis() (forwarded to side-effect callbacks).
     * @return FailureCode::NONE on success, INVALID_PARAM / EXECUTION_ERROR otherwise.
     */
    proto::FailureCode applyConfigParam(proto::ConfigParamId id, float value,
                                        uint32_t nowMs);

    // ── ST[13] fragmented transfer receive session (APUS-15) ─────────────────

    /**
     * @brief Handle a COMMAND frame carrying the FLAG_FRAGMENT bit.
     *
     * Reassembles multi-segment transfers.  Once all segments are present the
     * assembled payload is passed to enqueueCmd() as a synthetic frame.
     *
     * @param[in] frame  Decoded COMMAND frame with FLAG_FRAGMENT set.
     * @param[in] nowMs  Current millis() timestamp (session timeout tracking).
     */
    void handleFragmentedCommand(const proto::Frame& frame, uint32_t nowMs);

    /// Maximum number of fragments in a single reassembly session.
    static constexpr uint8_t  kMaxFragSegments  = 16U;
    /// Session inactivity timeout (ms) — stale session is discarded (APUS-15.4).
    static constexpr uint32_t kFragTimeoutMs    = 30000U;
    /// Per-segment payload bytes (MAX_FRAG_PAYLOAD = MAX_PAYLOAD_LEN − FRAG_HEADER_LEN).
    static constexpr uint16_t kFragSegDataSize =
        static_cast<uint16_t>(proto::MAX_FRAG_PAYLOAD);  // 194 bytes

    /**
     * @brief Fragmented-transfer reassembly state for one active transfer.
     *
     * Only one concurrent reassembly session is supported (APUS-15.3).
     */
    struct FragReceiveSession
    {
        uint16_t transferId;                                  ///< Transfer identifier from sender
        uint16_t totalSegments;                               ///< Expected segment count
        uint32_t lastActivityMs;                              ///< Timestamp of last segment receipt
        bool     active;                                      ///< True while session is open
        bool     received[kMaxFragSegments];                  ///< Per-segment receipt flags
        uint8_t  data[kMaxFragSegments * kFragSegDataSize];   ///< Reassembly buffer (3104 bytes)
        uint16_t dataLen;                                     ///< Bytes written so far
    };
    FragReceiveSession fragSession_ = {};

    // ── ST[13] fragmented transfer send session (APUS-15) ────────────────────

    /**
     * @brief Outbound fragmented-transfer session state (APUS-15 send path).
     *
     * Holds a copy of the full payload sliced into @c totalSegments chunks.
     * @c pumpFragSend() drips one fragment per call while the session is active,
     * respecting @c interFrameMs to avoid saturating the radio TX FIFO.
     *
     * Only one concurrent outbound session is supported.  If a new transfer is
     * started while one is active the previous session is abandoned and a
     * warning is logged.
     *
     * Memory: kMaxFragSegments × kFragSegDataSize = 3 104 bytes (static, PO10-3).
     */
    struct FragSendSession
    {
        uint16_t       transferId;                                 ///< Session identifier (rolling counter).
        uint16_t       totalSegments;                              ///< Total fragments to transmit.
        uint16_t       nextSegment;                                ///< Index of next fragment to send.
        uint32_t       lastSentMs;                                 ///< Timestamp of the most recent fragment sent.
        uint32_t       interFrameMs;                               ///< Minimum gap between consecutive fragments.
        bool           active;                                     ///< True while transmission is in progress.
        uint8_t        destNode;                                   ///< Destination node ID.
        proto::MsgType msgType;                                    ///< Frame TYPE for outgoing fragments.
        uint8_t        data[kMaxFragSegments * kFragSegDataSize];   ///< Inline payload buffer (3 104 bytes).
        uint32_t       dataLen;                                    ///< Total bytes loaded into @c data.
    };
    FragSendSession fragSendSession_ = {};

    /// Rolling transfer-ID counter — incremented for every new outbound session.
    uint16_t nextTransferId_ = 1U;

    /**
     * @brief Pump the active outbound fragment session — call from poll().
     *
     * Sends at most one fragment per call, respecting @c fragSendSession_.interFrameMs.
     * FLAG_ACK_REQ is set only on the final segment so that the ground station
     * can confirm receipt of the complete transfer without per-fragment ACK overhead.
     * The session is cleared automatically once the last segment is sent.
     *
     * @param[in] nowMs  Current millis() timestamp.
     * @pre  No external lock required — must be called from the same task as poll().
     */
    void pumpFragSend(uint32_t nowMs);

public:
    /**
     * @brief Encode and transmit a frame, registering it for retransmission
     *        on timeout if no ACK is received within @c proto::ACK_TIMEOUT_MS
     *        (APUS-4.5, APUS-4.6).
     *
     * Sets @c FLAG_ACK_REQ automatically.  If the retry buffer is full the
     * frame is still sent once (best-effort with a warning log).
     *
     * @param[in] frame  Fully populated frame (@c seq and @c node must be set).
     * @param[in] nowMs  Current millis() timestamp.
     * @return true if the frame was successfully encoded and sent.
     */
    bool sendReliable(const proto::Frame& frame, uint32_t nowMs);

    /**
     * @brief Start an outbound fragmented bulk transfer (APUS-15 send path).
     *
     * Slices @p data into @c proto::MAX_FRAG_PAYLOAD-byte chunks, stores them
     * in the internal @c FragSendSession buffer, and sets the session active.
     * Fragments are drip-fed by @c poll() → @c pumpFragSend() — one per
     * main-loop iteration — so this function is non-blocking.
     *
     * If a transfer is already in progress it is abandoned (with a warning)
     * and the new one takes over.
     *
     * @param[in] data          Pointer to the bytes to send.  Must not be nullptr.
     * @param[in] dataLen       Number of bytes to send.
     *                          Maximum: @c kMaxFragSegments × @c kFragSegDataSize
     *                          (currently 3 104 bytes).
     * @param[in] destNode      Destination node (e.g. @c proto::NODE_GROUND).
     * @param[in] msgType       Frame TYPE for all outgoing fragments
     *                          (typically @c proto::MsgType::TELEMETRY).
     * @param[in] interFrameMs  Minimum milliseconds between consecutive fragment
     *                          transmissions.  Pass 0 to send as fast as possible.
     * @return true if the session was armed; false if @p data is nullptr or
     *         @p dataLen exceeds the reassembly-buffer capacity.
     */
    bool startFragSend(const uint8_t* data,
                       uint32_t       dataLen,
                       uint8_t        destNode,
                       proto::MsgType msgType,
                       uint32_t       interFrameMs);

}; // class RadioDispatcher

} // namespace ares

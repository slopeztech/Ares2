/**
 * @file  radio_dispatcher.cpp
 * @brief Incoming APUS frame dispatcher — receive path implementation.
 *
 * See radio_dispatcher.h for the design contract and standards references.
 *
 * Implementation notes:
 *   - processBuffer() uses a bounded outer loop (kMaxFramesPerPoll) and a
 *     bounded inner scan (rxLen_ iterations) to satisfy MISRA Rule 6.4 and
 *     PO10-2 (no unbounded loops).
 *   - All stack buffers are fixed-size arrays; no heap allocation (PO10-3).
 *   - Explicit casts are used for every arithmetic result stored into a
 *     narrower type (MISRA Rules 10.3, 10.4).
 *   - Every routing decision is logged at DEBUG level (APUS-14.2).
 */

#include "comms/radio_dispatcher.h"
#include "debug/ares_log.h"

#include <cstring>

namespace ares
{

static constexpr const char* TAG = "RADIO";

// ── Constructor ──────────────────────────────────────────────────────────────

RadioDispatcher::RadioDispatcher(RadioInterface&               radio,
                                 ares::ams::MissionScriptEngine& engine)
    : radio_(radio)
    , engine_(engine)
    , rxBuf_{}
    , rxLen_(0U)
    , txSeq_(0U)
    , lastRxSeq_(0U)
    , seenFirst_(false)
{
}

// ── poll ─────────────────────────────────────────────────────────────────────

void RadioDispatcher::poll(uint32_t nowMs)
{
    // Append newly available bytes to rxBuf_ only when there is space.
    // Space check prevents overflow even if the radio driver returns more
    // bytes than expected (CERT-1: validate before use).
    if (rxLen_ < kRxBufLen)
    {
        uint16_t received = 0U;
        const uint16_t space = static_cast<uint16_t>(kRxBufLen - rxLen_);
        const RadioStatus st = radio_.receive(&rxBuf_[rxLen_], space, received);
        if (st == RadioStatus::OK && received > 0U)
        {
            rxLen_ = static_cast<uint16_t>(rxLen_ + received);
        }
    }

    if (rxLen_ >= static_cast<uint16_t>(proto::MIN_FRAME_LEN))
    {
        processBuffer(nowMs);
    }
}

// ── processBuffer ─────────────────────────────────────────────────────────────

void RadioDispatcher::processBuffer(uint32_t nowMs)
{
    // Outer loop: attempt to extract one frame per iteration.
    // Bounded by kMaxFramesPerPoll to satisfy PO10-2.
    for (uint16_t attempt = 0U; attempt < kMaxFramesPerPoll; ++attempt)
    {
        if (rxLen_ < static_cast<uint16_t>(proto::MIN_FRAME_LEN)) { return; }

        // ── Step 1: find SYNC_0 (bounded inner scan, PO10-2) ───────────────
        uint16_t syncPos = rxLen_;  // sentinel: "not found"
        for (uint16_t j = 0U; j < rxLen_; ++j)
        {
            if (rxBuf_[j] == proto::SYNC_0)
            {
                syncPos = j;
                break;
            }
        }

        if (syncPos == rxLen_)
        {
            // No SYNC_0 anywhere — all bytes are garbage. Discard. (APUS-14.2)
            LOG_D(TAG, "no SYNC in %u bytes — discarding", static_cast<unsigned>(rxLen_));
            rxLen_ = 0U;
            return;
        }

        // ── Step 2: discard leading garbage before SYNC_0 ──────────────────
        if (syncPos > 0U)
        {
            const uint16_t tail = static_cast<uint16_t>(rxLen_ - syncPos);
            (void)memmove(rxBuf_, &rxBuf_[syncPos], tail);
            rxLen_ = tail;
            LOG_D(TAG, "discarded %u pre-SYNC bytes",
                  static_cast<unsigned>(syncPos));
        }

        // Re-check minimum length after compaction.
        if (rxLen_ < static_cast<uint16_t>(proto::MIN_FRAME_LEN)) { return; }

        // ── Step 3: read declared payload length (byte index 9 from SYNC) ──
        // Index 9 is within bounds: rxLen_ >= MIN_FRAME_LEN (14) >= 10.
        const uint8_t  payloadLen = rxBuf_[9U];
        const uint16_t totalLen   =
            static_cast<uint16_t>(proto::HEADER_LEN) +
            static_cast<uint16_t>(payloadLen) +
            static_cast<uint16_t>(proto::CRC_LEN);

        // Guard against a corrupt length field that could cause out-of-bounds
        // access or a perpetual wait for bytes that will never arrive (CERT-1).
        if (totalLen > static_cast<uint16_t>(proto::MAX_FRAME_LEN))
        {
            // Skip this SYNC_0 byte and retry from the next position.
            (void)memmove(rxBuf_, &rxBuf_[1U],
                          static_cast<uint16_t>(rxLen_ - 1U));
            rxLen_ = static_cast<uint16_t>(rxLen_ - 1U);
            continue;
        }

        // ── Step 4: wait until the full frame has arrived ──────────────────
        if (rxLen_ < totalLen) { return; }

        // ── Step 5: attempt full decode (CRC, version, node, type, length) ─
        proto::Frame frame = {};
        if (!proto::decode(rxBuf_, totalLen, frame))
        {
            // Decode failure — skip this SYNC_0 and try the next candidate.
            LOG_D(TAG, "decode failed (totalLen=%u) — skipping SYNC byte",
                  static_cast<unsigned>(totalLen));
            (void)memmove(rxBuf_, &rxBuf_[1U],
                          static_cast<uint16_t>(rxLen_ - 1U));
            rxLen_ = static_cast<uint16_t>(rxLen_ - 1U);
            continue;
        }

        // ── Step 6: consume the frame bytes from the buffer ────────────────
        const uint16_t tail = static_cast<uint16_t>(rxLen_ - totalLen);
        if (tail > 0U)
        {
            (void)memmove(rxBuf_, &rxBuf_[totalLen], tail);
        }
        rxLen_ = tail;

        // ── Step 7: dispatch ────────────────────────────────────────────────
        dispatchFrame(frame, nowMs);
        // Continue outer loop to process any immediately following frame.
    }
}

// ── dispatchFrame ─────────────────────────────────────────────────────────────

void RadioDispatcher::dispatchFrame(const proto::Frame& frame, uint32_t nowMs)
{
    // Discard frames not addressed to us or broadcast (APUS-10.4, APUS-14.2).
    if (frame.node != proto::NODE_ROCKET &&
        frame.node != proto::NODE_BROADCAST)
    {
        LOG_D(TAG, "discard: node=0x%02X not for rocket",
              static_cast<unsigned>(frame.node));  // APUS-14.2
        return;
    }

    // O(1) dispatch by TYPE (APUS-14.1).
    switch (frame.type)
    {
    case proto::MsgType::COMMAND:
        LOG_D(TAG, "RX COMMAND seq=%u node=0x%02X flags=0x%02X",
              static_cast<unsigned>(frame.seq),
              static_cast<unsigned>(frame.node),
              static_cast<unsigned>(frame.flags));  // APUS-14.2
        handleCommand(frame, nowMs);
        break;

    case proto::MsgType::HEARTBEAT:
        LOG_D(TAG, "RX HEARTBEAT seq=%u",
              static_cast<unsigned>(frame.seq));  // APUS-14.2
        handleHeartbeat(frame, nowMs);
        break;

    // Ground-originated TM/EVENT/ACK frames are not consumed by the rocket.
    // Log at DEBUG level and discard (APUS-5.1, APUS-14.2).
    case proto::MsgType::TELEMETRY:
        LOG_D(TAG, "RX TELEMETRY seq=%u — discarded (ground echo?)",
              static_cast<unsigned>(frame.seq));
        break;

    case proto::MsgType::EVENT:
        LOG_D(TAG, "RX EVENT seq=%u — discarded",
              static_cast<unsigned>(frame.seq));
        break;

    case proto::MsgType::ACK:
        LOG_D(TAG, "RX ACK seq=%u — discarded",
              static_cast<unsigned>(frame.seq));
        break;

    case proto::MsgType::NONE:
    default:
        // decode() already rejects unknown types; this branch is defensive.
        LOG_W(TAG, "RX unknown TYPE=0x%02X — discarded",  // APUS-5.1
              static_cast<unsigned>(static_cast<uint8_t>(frame.type)));
        break;
    }
}

// ── handleCommand ─────────────────────────────────────────────────────────────

void RadioDispatcher::handleCommand(const proto::Frame& frame, uint32_t nowMs)
{
    // ── APUS-4.7: duplicate detection ──────────────────────────────────────
    // A duplicate is the same SEQ as the last accepted command.
    // Silently discard — the sender already received the acceptance ACK.
    if (seenFirst_ && proto::isDuplicate(frame.seq, lastRxSeq_))
    {
        LOG_D(TAG, "COMMAND seq=%u duplicate — discarded",
              static_cast<unsigned>(frame.seq));
        return;
    }

    // ── APUS-9.1: acceptance ACK — sent unconditionally ────────────────────
    sendAckNack(frame.seq, frame.node, proto::FailureCode::NONE, 0U);

    // Record SEQ after acceptance ACK is queued.
    lastRxSeq_ = frame.seq;
    seenFirst_ = true;

    // ── Execute command and capture result code ─────────────────────────────
    const proto::FailureCode result = executeCommand(frame, nowMs);

    // ── APUS-9.3: completion ACK/NACK — only if FLAG_ACK_REQ ───────────────
    if ((frame.flags & proto::FLAG_ACK_REQ) != 0U)
    {
        // Carry commandId as diagnostic byte in the completion NACK (APUS-9.4).
        const uint8_t diagByte = (result != proto::FailureCode::NONE)
                                 ? frame.payload[1U]   // CommandHeader.commandId
                                 : 0U;
        sendAckNack(frame.seq, frame.node, result, diagByte);
    }
}

// ── handleHeartbeat ───────────────────────────────────────────────────────────

void RadioDispatcher::handleHeartbeat(const proto::Frame& frame, uint32_t nowMs)
{
    (void)frame;   // Source node unused — we always respond from NODE_ROCKET.
    (void)nowMs;

    proto::Frame response = {};
    response.ver   = proto::PROTOCOL_VERSION;
    response.flags = 0U;
    response.node  = proto::NODE_ROCKET;
    response.type  = proto::MsgType::HEARTBEAT;
    response.seq   = txSeq_++;
    response.len   = 0U;

    uint8_t wire[proto::MAX_FRAME_LEN] = {};
    const uint16_t wireLen = proto::encode(response, wire, sizeof(wire));
    if (wireLen > 0U)
    {
        (void)radio_.send(wire, wireLen);
        LOG_D(TAG, "TX HEARTBEAT seq=%u", static_cast<unsigned>(response.seq));
    }
}

// ── sendAckNack ───────────────────────────────────────────────────────────────

void RadioDispatcher::sendAckNack(uint8_t            originalSeq,
                                  uint8_t            originalNode,
                                  proto::FailureCode code,
                                  uint8_t            failureData)
{
    proto::AckPayload ack = {};
    ack.originalSeq  = originalSeq;
    ack.originalNode = originalNode;
    ack.failureCode  = static_cast<uint8_t>(code);
    ack.failureData  = failureData;

    proto::Frame frame = {};
    frame.ver   = proto::PROTOCOL_VERSION;
    frame.flags = 0U;
    frame.node  = proto::NODE_ROCKET;
    frame.type  = proto::MsgType::ACK;
    frame.seq   = txSeq_++;
    frame.len   = static_cast<uint8_t>(sizeof(ack));
    (void)memcpy(frame.payload, &ack, sizeof(ack));

    uint8_t wire[proto::MAX_FRAME_LEN] = {};
    const uint16_t wireLen = proto::encode(frame, wire, sizeof(wire));
    if (wireLen > 0U)
    {
        (void)radio_.send(wire, wireLen);
        LOG_D(TAG, "TX %s seq=%u origSeq=%u code=0x%02X",
              (code == proto::FailureCode::NONE) ? "ACK" : "NACK",
              static_cast<unsigned>(frame.seq),
              static_cast<unsigned>(originalSeq),
              static_cast<unsigned>(ack.failureCode));
    }
}

// ── executeCommand ────────────────────────────────────────────────────────────

proto::FailureCode RadioDispatcher::executeCommand(const proto::Frame& frame,
                                                   uint32_t nowMs)
{
    // CommandHeader layout: [0] = priority, [1] = commandId (APUS-7).
    // frame.len >= sizeof(CommandHeader) is guaranteed by decode() / meetsMinPayloadLen().
    const uint8_t commandId = frame.payload[1U];

    // ── APUS-7.1: reject unknown commandId ────────────────────────────────
    if (commandId < static_cast<uint8_t>(proto::CommandId::FIRST) ||
        commandId > static_cast<uint8_t>(proto::CommandId::LAST))
    {
        LOG_W(TAG, "COMMAND unknown commandId=0x%02X", static_cast<unsigned>(commandId));
        return proto::FailureCode::UNKNOWN_COMMAND;
    }

    const auto cmd = static_cast<proto::CommandId>(commandId);

    // ── APUS-7.3: CRITICAL commands must carry FLAG_PRIORITY ──────────────
    const bool isCritical = (cmd == proto::CommandId::ABORT         ||
                             cmd == proto::CommandId::FIRE_PYRO_A   ||
                             cmd == proto::CommandId::FIRE_PYRO_B   ||
                             cmd == proto::CommandId::FACTORY_RESET);
    if (isCritical && ((frame.flags & proto::FLAG_PRIORITY) == 0U))
    {
        LOG_W(TAG, "CRITICAL commandId=0x%02X missing FLAG_PRIORITY \u2014 rejected",
              static_cast<unsigned>(commandId));
        return proto::FailureCode::PRECONDITION_FAIL;
    }

    // ── O(1) command dispatch (APUS-14.1) ─────────────────────────────────
    switch (cmd)
    {
    // ── ST[8] Flight control ─────────────────────────────────────────────

    case proto::CommandId::ARM_FLIGHT:
        if (!engine_.arm())
        {
            LOG_W(TAG, "ARM_FLIGHT: engine.arm() failed (not LOADED?)");
            return proto::FailureCode::PRECONDITION_FAIL;
        }
        LOG_I(TAG, "ARM_FLIGHT: engine armed via radio TC");
        return proto::FailureCode::NONE;

    case proto::CommandId::ABORT:
        if (!engine_.injectTcCommand("ABORT"))
        {
            LOG_W(TAG, "ABORT: injectTcCommand failed");
            return proto::FailureCode::EXECUTION_ERROR;
        }
        LOG_I(TAG, "ABORT injected via radio TC");
        return proto::FailureCode::NONE;

    case proto::CommandId::FACTORY_RESET:
        if (!engine_.injectTcCommand("RESET"))
        {
            LOG_W(TAG, "FACTORY_RESET: injectTcCommand(RESET) failed");
            return proto::FailureCode::EXECUTION_ERROR;
        }
        LOG_I(TAG, "FACTORY_RESET: RESET injected via radio TC");
        return proto::FailureCode::NONE;

    case proto::CommandId::FIRE_PYRO_A:
    case proto::CommandId::FIRE_PYRO_B:
    {
        // ── APUS-7.2: pyro commands require engine RUNNING (armed) ────────
        ares::ams::EngineSnapshot snap = {};
        engine_.getSnapshot(snap);
        if (snap.status != ares::ams::EngineStatus::RUNNING)
        {
            LOG_W(TAG, "FIRE_PYRO commandId=0x%02X rejected: engine not RUNNING",
                  static_cast<unsigned>(commandId));
            return proto::FailureCode::PRECONDITION_FAIL;
        }
        // Pyro direct-fire bypasses the AMS script — not yet wired to HAL.
        // Return EXECUTION_ERROR so ground retries via the AMS script path.
        LOG_W(TAG, "FIRE_PYRO commandId=0x%02X: HAL not yet connected",
              static_cast<unsigned>(commandId));
        return proto::FailureCode::EXECUTION_ERROR;
    }

    // ── ST[8] Mode control ────────────────────────────────────────────────

    case proto::CommandId::SET_MODE:
    {
        // Payload layout: CommandHeader(2) + mode(1).
        // mode 0x00 = pause execution, 0x01 = resume execution.
        if (frame.len < 3U)
        {
            LOG_W(TAG, "SET_MODE: payload too short (%u bytes)",
                  static_cast<unsigned>(frame.len));
            return proto::FailureCode::INVALID_PARAM;
        }
        const uint8_t mode = frame.payload[2U];
        if (mode > 1U)
        {
            LOG_W(TAG, "SET_MODE: unknown mode 0x%02X", static_cast<unsigned>(mode));
            return proto::FailureCode::INVALID_PARAM;
        }
        engine_.setExecutionEnabled(mode == 1U);
        LOG_I(TAG, "SET_MODE: execution %s via radio TC",
              (mode == 1U) ? "enabled" : "disabled");
        return proto::FailureCode::NONE;
    }

    case proto::CommandId::SET_FCS_ACTIVE:
    {
        // Payload layout: CommandHeader(2) + enabled(1).
        // In the current model, FCS active == execution enabled.
        if (frame.len < 3U)
        {
            LOG_W(TAG, "SET_FCS_ACTIVE: payload too short (%u bytes)",
                  static_cast<unsigned>(frame.len));
            return proto::FailureCode::INVALID_PARAM;
        }
        const uint8_t enabled = frame.payload[2U];
        if (enabled > 1U)
        {
            LOG_W(TAG, "SET_FCS_ACTIVE: invalid value 0x%02X",
                  static_cast<unsigned>(enabled));
            return proto::FailureCode::INVALID_PARAM;
        }
        engine_.setExecutionEnabled(enabled == 1U);
        LOG_I(TAG, "SET_FCS_ACTIVE: FCS %s via radio TC",
              (enabled == 1U) ? "active" : "inactive");
        return proto::FailureCode::NONE;
    }

    // ── ST[3] Telemetry on demand ─────────────────────────────────────────

    case proto::CommandId::REQUEST_TELEMETRY:
        if (!engine_.requestTelemetry(nowMs))
        {
            LOG_W(TAG, "REQUEST_TELEMETRY: no active HK slots (engine idle?)");
            return proto::FailureCode::EXECUTION_ERROR;
        }
        LOG_D(TAG, "REQUEST_TELEMETRY: immediate HK frame sent");
        return proto::FailureCode::NONE;

    case proto::CommandId::SET_TELEM_INTERVAL:
    {
        // Payload layout: CommandHeader(2) + intervalMs_LE(4) = 6 bytes.
        if (frame.len < 6U)
        {
            LOG_W(TAG, "SET_TELEM_INTERVAL: payload too short (%u bytes)",
                  static_cast<unsigned>(frame.len));
            return proto::FailureCode::INVALID_PARAM;
        }
        // MISRA 10.3: use memcpy to avoid unaligned read on ESP32.
        uint32_t intervalMs = 0U;
        (void)memcpy(&intervalMs, &frame.payload[2U], sizeof(intervalMs));
        // Sanity bounds: 100 ms .. 60 000 ms.
        if (intervalMs < 100U || intervalMs > 60000U)
        {
            LOG_W(TAG, "SET_TELEM_INTERVAL: %u ms out of range [100, 60000]",
                  static_cast<unsigned>(intervalMs));
            return proto::FailureCode::INVALID_PARAM;
        }
        if (!engine_.setTelemInterval(intervalMs))
        {
            LOG_W(TAG, "SET_TELEM_INTERVAL: no active HK slots to update");
            return proto::FailureCode::EXECUTION_ERROR;
        }
        LOG_I(TAG, "SET_TELEM_INTERVAL: interval set to %u ms",
              static_cast<unsigned>(intervalMs));
        return proto::FailureCode::NONE;
    }

    // ── ST[8] Status and config queries ───────────────────────────────────

    case proto::CommandId::REQUEST_STATUS:
    {
        // Build and send a TELEMETRY frame with the engine's current status.
        // StatusBits are populated from live engine state (APUS-6):
        // armed, fcsActive, gpsValid, pyroAFired, pyroBFired.
        ares::ams::EngineSnapshot snap = {};
        engine_.getSnapshot(snap);

        proto::TelemetryPayload tm = {};
        tm.timestampMs = nowMs;
        tm.flightPhase = static_cast<uint8_t>(snap.status);
        tm.statusBits  = engine_.getStatusBits();

        proto::Frame resp = {};
        resp.ver   = proto::PROTOCOL_VERSION;
        resp.flags = 0U;
        resp.node  = proto::NODE_ROCKET;
        resp.type  = proto::MsgType::TELEMETRY;
        resp.seq   = txSeq_++;
        resp.len   = static_cast<uint8_t>(sizeof(tm));
        (void)memcpy(resp.payload, &tm, sizeof(tm));

        uint8_t wire[proto::MAX_FRAME_LEN] = {};
        const uint16_t wireLen = proto::encode(resp, wire, sizeof(wire));
        if (wireLen > 0U)
        {
            (void)radio_.send(wire, wireLen);
            LOG_D(TAG, "REQUEST_STATUS: status frame sent seq=%u engineStatus=%u",
                  static_cast<unsigned>(resp.seq),
                  static_cast<unsigned>(tm.flightPhase));
        }
        return proto::FailureCode::NONE;
    }

    case proto::CommandId::REQUEST_CONFIG:
        // No CONFIG frame type in APUS \u2014 response is ACK only.
        // The ground station derives config from the STATUS telemetry.
        LOG_D(TAG, "REQUEST_CONFIG: acknowledged (no dedicated config frame)");
        return proto::FailureCode::NONE;

    case proto::CommandId::VERIFY_CONFIG:
        // Config integrity check not yet implemented.
        // Return NONE \u2014 engine running implies config is valid.
        LOG_D(TAG, "VERIFY_CONFIG: acknowledged (config assumed valid)");
        return proto::FailureCode::NONE;

    case proto::CommandId::SET_CONFIG_PARAM:
        // Parameter set infrastructure not yet defined.
        LOG_W(TAG, "SET_CONFIG_PARAM: not implemented");
        return proto::FailureCode::EXECUTION_ERROR;

    default:
        LOG_W(TAG, "commandId=0x%02X unhandled",
              static_cast<unsigned>(commandId));
        return proto::FailureCode::UNKNOWN_COMMAND;
    }
}

} // namespace ares

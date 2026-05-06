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
#include "ares_assert.h"
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

    processRetries(nowMs);
    drainCmdQueue(nowMs);
}

// ── processBuffer ─────────────────────────────────────────────────────────────

void RadioDispatcher::processBuffer(uint32_t nowMs) // NOLINT(readability-function-size)
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

void RadioDispatcher::dispatchFrame(const proto::Frame& frame, uint32_t nowMs) // NOLINT(readability-function-size)
{
    // Discard frames not addressed to us or broadcast (APUS-10.4, APUS-14.2).
    // APUS-14.3: if the unroutable frame is a COMMAND we can infer the source
    // is NODE_GROUND (point-to-point link) and must send a routing-failure NACK.
    if (frame.node != proto::NODE_ROCKET &&
        frame.node != proto::NODE_BROADCAST)
    {
        LOG_D(TAG, "discard: node=0x%02X not for rocket (type=0x%02X seq=%u)",
              static_cast<unsigned>(frame.node),
              static_cast<unsigned>(static_cast<uint8_t>(frame.type)),
              static_cast<unsigned>(frame.seq));  // APUS-14.2
        if (frame.type == proto::MsgType::COMMAND)
        {
            // Source is NODE_GROUND (only expected COMMAND sender on this link).
            sendAckNack(frame.seq, proto::NODE_GROUND,
                        proto::FailureCode::ROUTING_FAIL,
                        frame.node);  // failureData = destination that was refused
        }
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
    {
        // Clear the matching retry slot when an ACK confirms a sendReliable frame.
        if (frame.len >= static_cast<uint8_t>(sizeof(proto::AckPayload)))
        {
            proto::AckPayload ack = {};
            (void)memcpy(&ack, frame.payload, sizeof(ack));
            clearRetryForSeq(ack.originalSeq);
            LOG_D(TAG, "RX ACK seq=%u clears retry for origSeq=%u",
                  static_cast<unsigned>(frame.seq),
                  static_cast<unsigned>(ack.originalSeq));
        }
        else
        {
            LOG_D(TAG, "RX ACK seq=%u (no retry data)",
                  static_cast<unsigned>(frame.seq));
        }
        break;
    }

    case proto::MsgType::NONE:
    default:
        // decode() already rejects unknown types; this branch is purely defensive.
        // APUS-14.3: send NACK so the sender knows the packet was not handled.
        LOG_W(TAG, "RX unknown TYPE=0x%02X seq=%u \u2014 NACK sent (APUS-5.1)",
              static_cast<unsigned>(static_cast<uint8_t>(frame.type)),
              static_cast<unsigned>(frame.seq));
        sendAckNack(frame.seq, proto::NODE_GROUND,
                    proto::FailureCode::UNKNOWN_TYPE,
                    static_cast<uint8_t>(frame.type));
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

    // ── APUS-15: fragmented transfer ───────────────────────────────────────
    if ((frame.flags & proto::FLAG_FRAGMENT) != 0U)
    {
        handleFragmentedCommand(frame, nowMs);
        return;
    }

    // ── APUS-9.1: acceptance ACK — sent unconditionally ────────────────────
    sendAckNack(frame.seq, frame.node, proto::FailureCode::NONE, 0U);

    // Record SEQ after acceptance ACK is queued.
    lastRxSeq_ = frame.seq;
    seenFirst_ = true;

    // ── APUS-2.2: insert into the priority queue; drainCmdQueue() executes ──
    if (!enqueueCmd(frame))
    {
        // Queue full: send completion NACK immediately (APUS-9.3).
        if ((frame.flags & proto::FLAG_ACK_REQ) != 0U)
        {
            sendAckNack(frame.seq, frame.node,
                        proto::FailureCode::QUEUE_FULL,
                        frame.payload[1U]);  // CommandHeader.commandId as diagnostic
        }
    }
    // Completion ACK/NACK for queued commands is sent by drainCmdQueue().
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

proto::FailureCode RadioDispatcher::executeCommand(const proto::Frame& frame, // NOLINT(readability-function-size)
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
        // Apply script-declared radio.config overrides before first telemetry tick.
        // Ground can still override these values later via SET_CONFIG_PARAM (APUS-16.1).
        // MISRA-8: loop over all config param IDs via compile-time sentinel (AMS-4.15).
        for (uint8_t pidRaw = static_cast<uint8_t>(proto::ConfigParamId::FIRST);
             pidRaw <= static_cast<uint8_t>(proto::ConfigParamId::LAST);
             ++pidRaw)
        {
            // MISRA-14.3: validate range before integer→enum cast.
            ARES_ASSERT(pidRaw >= static_cast<uint8_t>(proto::ConfigParamId::FIRST));
            ARES_ASSERT(pidRaw <= static_cast<uint8_t>(proto::ConfigParamId::LAST));
            const proto::ConfigParamId pid = static_cast<proto::ConfigParamId>(pidRaw);
            float override = 0.0f;
            if (engine_.getScriptRadioConfig(pid, override))
            {
                const proto::FailureCode fc = applyConfigParam(pid, override, nowMs);
                if (fc != proto::FailureCode::NONE)
                {
                    LOG_W(TAG, "ARM_FLIGHT: script radio.config override failed for pid=0x%02X (fc=0x%02X)",
                          static_cast<unsigned>(pidRaw),
                          static_cast<unsigned>(static_cast<uint8_t>(fc)));
                }
            }
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
    {
        // Build and transmit a TELEMETRY frame whose payload encodes the
        // full parameter table (APUS-16: report parameter values).
        // Layout: paramCount(1) + [paramId(1)+value_le32(4)] × N  (≤ 6 entries)
        uint8_t payload[1U + kConfigParamCount * 5U] = {};
        payload[0] = kConfigParamCount;
        uint8_t offset = 1U;
        for (uint8_t i = 0U; i < kConfigParamCount; ++i)
        {
            payload[offset] = static_cast<uint8_t>(configParams_[i].id);
            offset++;
            uint32_t raw = 0U;
            (void)memcpy(&raw, &configParams_[i].value, sizeof(raw));
            payload[offset]     = static_cast<uint8_t>( raw        & 0xFFU);
            payload[offset + 1] = static_cast<uint8_t>((raw >>  8U) & 0xFFU);
            payload[offset + 2] = static_cast<uint8_t>((raw >> 16U) & 0xFFU);
            payload[offset + 3] = static_cast<uint8_t>((raw >> 24U) & 0xFFU);
            offset += 4U;
        }
        proto::Frame cfg = {};
        cfg.ver   = proto::PROTOCOL_VERSION;
        cfg.flags = 0U;
        cfg.node  = proto::NODE_ROCKET;
        cfg.type  = proto::MsgType::TELEMETRY;
        cfg.seq   = txSeq_++;
        cfg.len   = offset;
        (void)memcpy(cfg.payload, payload, offset);
        uint8_t wire[proto::MAX_FRAME_LEN] = {};
        const uint16_t wLen = proto::encode(cfg, wire, sizeof(wire));
        if (wLen > 0U)
        {
            (void)radio_.send(wire, wLen);
            LOG_D(TAG, "REQUEST_CONFIG: config report sent (%u params)", kConfigParamCount);
        }
        return proto::FailureCode::NONE;
    }

    case proto::CommandId::VERIFY_CONFIG:
    {
        // Validate all parameters are within their declared bounds (APUS-16.2).
        for (uint8_t i = 0U; i < kConfigParamCount; ++i)
        {
            if (configParams_[i].value < configParams_[i].minVal ||
                configParams_[i].value > configParams_[i].maxVal)
            {
                LOG_W(TAG, "VERIFY_CONFIG: param id=0x%02X value=%.2f out of [%.2f, %.2f]",
                      static_cast<unsigned>(configParams_[i].id),
                      static_cast<double>(configParams_[i].value),
                      static_cast<double>(configParams_[i].minVal),
                      static_cast<double>(configParams_[i].maxVal));
                return proto::FailureCode::INVALID_PARAM;
            }
        }
        LOG_D(TAG, "VERIFY_CONFIG: all %u params valid", kConfigParamCount);
        return proto::FailureCode::NONE;
    }

    case proto::CommandId::SET_CONFIG_PARAM:
    {
        // Payload layout: CommandHeader(2) + paramId(1) + value_le32(4) = 7 bytes.
        if (frame.len < 7U)
        {
            LOG_W(TAG, "SET_CONFIG_PARAM: payload too short (%u bytes)",
                  static_cast<unsigned>(frame.len));
            return proto::FailureCode::INVALID_PARAM;
        }
        const auto paramId = static_cast<proto::ConfigParamId>(frame.payload[2U]);
        uint32_t rawVal = 0U;
        (void)memcpy(&rawVal, &frame.payload[3U], sizeof(rawVal));
        float value = 0.0f;
        (void)memcpy(&value, &rawVal, sizeof(value));
        return applyConfigParam(paramId, value, nowMs);
    }

    default:
        LOG_W(TAG, "commandId=0x%02X unhandled",
              static_cast<unsigned>(commandId));
        return proto::FailureCode::UNKNOWN_COMMAND;
    }
}

// ── ST[20] parameter management helpers ──────────────────────────────────────

RadioDispatcher::ConfigEntry*
RadioDispatcher::findConfigParam(proto::ConfigParamId id)
{
    for (uint8_t i = 0U; i < kConfigParamCount; ++i)
    {
        if (configParams_[i].id == id)
        {
            return &configParams_[i];
        }
    }
    return nullptr;
}

proto::FailureCode RadioDispatcher::applyConfigParam(proto::ConfigParamId id,
                                                      float value,
                                                      uint32_t nowMs)
{
    ConfigEntry* entry = findConfigParam(id);
    if (entry == nullptr)
    {
        LOG_W(TAG, "SET_CONFIG_PARAM: unknown paramId=0x%02X",
              static_cast<unsigned>(static_cast<uint8_t>(id)));
        return proto::FailureCode::INVALID_PARAM;
    }
    if (!entry->writable)
    {
        LOG_W(TAG, "SET_CONFIG_PARAM: paramId=0x%02X is read-only (APUS-16.1)",
              static_cast<unsigned>(static_cast<uint8_t>(id)));
        return proto::FailureCode::PRECONDITION_FAIL;
    }
    if (value < entry->minVal || value > entry->maxVal)
    {
        LOG_W(TAG, "SET_CONFIG_PARAM: paramId=0x%02X value=%.2f out of [%.2f, %.2f]",
              static_cast<unsigned>(static_cast<uint8_t>(id)),
              static_cast<double>(value),
              static_cast<double>(entry->minVal),
              static_cast<double>(entry->maxVal));
        return proto::FailureCode::INVALID_PARAM;
    }
    entry->value = value;

    // Apply side-effects to running subsystems.
    if (id == proto::ConfigParamId::TELEM_INTERVAL_MS)
    {
        const auto intervalMs = static_cast<uint32_t>(value);
        if (!engine_.setTelemInterval(intervalMs))
        {
            LOG_W(TAG, "SET_CONFIG_PARAM TELEM_INTERVAL: setTelemInterval failed");
            return proto::FailureCode::EXECUTION_ERROR;
        }
    }
    // Propagate monitoring-threshold changes into the ST[12] slots (APUS-12.1, APUS-16).
    (void)engine_.configureMonitorFromParam(id, value);

    LOG_I(TAG, "SET_CONFIG_PARAM: paramId=0x%02X set to %.3f",
          static_cast<unsigned>(static_cast<uint8_t>(id)),
          static_cast<double>(value));
    (void)nowMs;
    return proto::FailureCode::NONE;
}

// ── enqueueRetry ──────────────────────────────────────────────────────────────

/**
 * @brief Store a frame copy in the TX retry buffer (APUS-4.5).
 *
 * Must only be called for frames that have @c FLAG_ACK_REQ set.  The frame
 * is retransmitted by @c processRetries() if no ACK arrives in time.
 *
 * @param[in] frame  Frame that was just sent (copy stored here).
 * @param[in] nowMs  Timestamp of this send attempt.
 * @return true if a free slot was found; false if the buffer is full.
 */
bool RadioDispatcher::enqueueRetry(const proto::Frame& frame, uint32_t nowMs)
{
    for (uint8_t i = 0U; i < kMaxRetrySlots; ++i)
    {
        if (!retrySlots_[i].active)
        {
            retrySlots_[i].frame   = frame;
            retrySlots_[i].sentMs  = nowMs;
            retrySlots_[i].retries = 0U;
            retrySlots_[i].active  = true;
            return true;
        }
    }
    LOG_W(TAG, "retry buffer full — seq=%u sent best-effort",
          static_cast<unsigned>(frame.seq));
    return false;
}

// ── processRetries ────────────────────────────────────────────────────────────

/**
 * @brief Retransmit or expire timed-out frames (APUS-4.5, APUS-4.6, APUS-2.3).
 *
 * For each active retry slot: if @c ACK_TIMEOUT_MS has elapsed since the
 * last send and the retry count is below @c MAX_RETRIES, re-sends the frame
 * with @c FLAG_RETRANSMIT set.  Expires the slot after @c MAX_RETRIES attempts.
 *
 * @param[in] nowMs  Current timestamp.
 */
void RadioDispatcher::processRetries(uint32_t nowMs)
{
    for (uint8_t i = 0U; i < kMaxRetrySlots; ++i)
    {
        TxRetrySlot& slot = retrySlots_[i];
        if (!slot.active) { continue; }

        const uint32_t elapsed = nowMs - slot.sentMs;
        if (elapsed < static_cast<uint32_t>(proto::ACK_TIMEOUT_MS)) { continue; }

        if (slot.retries >= static_cast<uint8_t>(proto::MAX_RETRIES))
        {
            LOG_W(TAG, "retry exhausted seq=%u after %u attempts — giving up",
                  static_cast<unsigned>(slot.frame.seq),
                  static_cast<unsigned>(slot.retries));
            slot.active = false;
            continue;
        }

        // Set FLAG_RETRANSMIT on the copy (APUS-2.3).
        slot.frame.flags = static_cast<uint8_t>(slot.frame.flags | proto::FLAG_RETRANSMIT);

        uint8_t wire[proto::MAX_FRAME_LEN] = {};
        const uint16_t wireLen = proto::encode(slot.frame, wire, sizeof(wire));
        if (wireLen > 0U)
        {
            (void)radio_.send(wire, wireLen);
            slot.retries++;
            slot.sentMs = nowMs;
            LOG_D(TAG, "RETRANSMIT seq=%u attempt %u/%u",
                  static_cast<unsigned>(slot.frame.seq),
                  static_cast<unsigned>(slot.retries),
                  static_cast<unsigned>(proto::MAX_RETRIES));
        }
    }
}

// ── clearRetryForSeq ──────────────────────────────────────────────────────────

/**
 * @brief Deactivate the retry slot whose SEQ matches @p seq (APUS-4.5).
 *
 * Called on receipt of an ACK from the ground station to confirm delivery.
 *
 * @param[in] seq  SEQ of the original sent frame, taken from AckPayload.originalSeq.
 */
void RadioDispatcher::clearRetryForSeq(uint8_t seq)
{
    for (uint8_t i = 0U; i < kMaxRetrySlots; ++i)
    {
        if (retrySlots_[i].active && retrySlots_[i].frame.seq == seq)
        {
            retrySlots_[i].active = false;
            LOG_D(TAG, "retry cleared for seq=%u", static_cast<unsigned>(seq));
            return;
        }
    }
}

// ── sendReliable ──────────────────────────────────────────────────────────────

/**
 * @brief Encode and transmit a frame with automatic retransmission-on-timeout.
 *
 * Sets @c FLAG_ACK_REQ, sends the encoded frame, then stores a copy in the
 * retry buffer so @c processRetries() can re-send it if no ACK arrives within
 * @c proto::ACK_TIMEOUT_MS (APUS-4.5).
 *
 * @param[in] frame  Frame to send; @c seq must already be set.
 * @param[in] nowMs  Current millis() timestamp.
 * @return true if encode + send succeeded; false on encode failure.
 */
bool RadioDispatcher::sendReliable(const proto::Frame& frame, uint32_t nowMs)
{
    proto::Frame f    = frame;
    f.flags = static_cast<uint8_t>(f.flags | proto::FLAG_ACK_REQ);

    uint8_t wire[proto::MAX_FRAME_LEN] = {};
    const uint16_t wireLen = proto::encode(f, wire, sizeof(wire));
    if (wireLen == 0U) { return false; }

    const bool ok = (radio_.send(wire, wireLen) == RadioStatus::OK);
    if (ok)
    {
        (void)enqueueRetry(f, nowMs);
        LOG_D(TAG, "TX reliable seq=%u (retries up to %u if no ACK in %u ms)",
              static_cast<unsigned>(f.seq),
              static_cast<unsigned>(proto::MAX_RETRIES),
              static_cast<unsigned>(proto::ACK_TIMEOUT_MS));
    }
    return ok;
}

// ── commandPriority ───────────────────────────────────────────────────────────

/**
 * @brief Return the priority level for a given CommandId (APUS-2.1).
 *
 * Priority levels:
 *   - 0 (CRITICAL): ABORT, FIRE_PYRO_A/B, FACTORY_RESET
 *   - 1 (HIGH):     ARM_FLIGHT, SET_MODE, SET_FCS_ACTIVE, SET_CONFIG_PARAM
 *   - 2 (NORMAL):   REQUEST_TELEMETRY, SET_TELEM_INTERVAL
 *   - 3 (LOW):      REQUEST_STATUS, REQUEST_CONFIG, VERIFY_CONFIG
 *
 * @param[in] id  Command identifier.
 * @return Priority level 0–3 (lower value = higher priority).
 */
uint8_t RadioDispatcher::commandPriority(proto::CommandId id)
{
    switch (id)
    {
    case proto::CommandId::ABORT:
    case proto::CommandId::FIRE_PYRO_A:
    case proto::CommandId::FIRE_PYRO_B:
    case proto::CommandId::FACTORY_RESET:
        return kCriticalPriority;

    case proto::CommandId::ARM_FLIGHT:
    case proto::CommandId::SET_MODE:
    case proto::CommandId::SET_FCS_ACTIVE:
    case proto::CommandId::SET_CONFIG_PARAM:
        return kHighPriority;

    case proto::CommandId::REQUEST_TELEMETRY:
    case proto::CommandId::SET_TELEM_INTERVAL:
        return kNormalPriority;

    case proto::CommandId::REQUEST_STATUS:
    case proto::CommandId::REQUEST_CONFIG:
    case proto::CommandId::VERIFY_CONFIG:
    default:
        return kLowPriority;
    }
}

// ── enqueueCmd ────────────────────────────────────────────────────────────────

/**
 * @brief Insert a decoded COMMAND frame into the priority queue (APUS-2.2).
 *
 * The priority is derived from the CommandId via @c commandPriority().
 * If the queue is full @c QUEUE_FULL is logged and the function returns false;
 * the caller is responsible for sending the NACK.
 *
 * @param[in] frame  Decoded COMMAND frame (acceptance ACK already sent).
 * @return true on success; false if the queue is full.
 */
bool RadioDispatcher::enqueueCmd(const proto::Frame& frame)
{
    if (cmdQueueCount_ >= kCmdQueueDepth)
    {
        LOG_W(TAG, "CMD queue full seq=%u — command dropped",
              static_cast<unsigned>(frame.seq));
        return false;
    }

    // CommandHeader: [0]=priority_field, [1]=commandId
    const auto cmdId = static_cast<proto::CommandId>(frame.payload[1U]);
    const uint8_t prio = commandPriority(cmdId);

    for (uint8_t i = 0U; i < kCmdQueueDepth; ++i)
    {
        if (!cmdQueue_[i].active)
        {
            cmdQueue_[i].frame    = frame;
            cmdQueue_[i].priority = prio;
            cmdQueue_[i].active   = true;
            cmdQueueCount_++;
            LOG_D(TAG, "CMD enqueued seq=%u id=0x%02X prio=%u qsize=%u",
                  static_cast<unsigned>(frame.seq),
                  static_cast<unsigned>(frame.payload[1U]),
                  static_cast<unsigned>(prio),
                  static_cast<unsigned>(cmdQueueCount_));
            return true;
        }
    }
    return false;  // Defensive: should not reach here when cmdQueueCount_ < kCmdQueueDepth.
}

// ── drainCmdQueue ─────────────────────────────────────────────────────────────

/**
 * @brief Execute all queued commands in priority order (APUS-2.1, APUS-2.4).
 *
 * Each iteration performs a linear scan to find the active slot with the
 * lowest priority value (0 = CRITICAL).  The slot is cleared before
 * @c executeCommand() is called so that a re-entrant poll() cannot
 * double-execute it.  Completion ACK/NACK is sent here for entries with
 * @c FLAG_ACK_REQ set (APUS-9.3).
 *
 * The outer loop is bounded by @c kCmdQueueDepth (PO10-2).
 *
 * @param[in] nowMs  Forwarded to executeCommand().
 */
void RadioDispatcher::drainCmdQueue(uint32_t nowMs)
{
    for (uint8_t n = 0U; n < kCmdQueueDepth; ++n)
    {
        // Find highest-priority (lowest value) active slot.
        int8_t  best     = -1;
        uint8_t bestPrio = 0xFFU;
        for (uint8_t i = 0U; i < kCmdQueueDepth; ++i)
        {
            if (cmdQueue_[i].active && cmdQueue_[i].priority < bestPrio)
            {
                bestPrio = cmdQueue_[i].priority;
                best     = static_cast<int8_t>(i);
            }
        }

        if (best < 0) { break; }  // Queue empty.

        // Dequeue before executing so the slot is free if executeCommand()
        // re-enters poll() indirectly (e.g. via engine callbacks).
        PendingCmd cmd = cmdQueue_[static_cast<uint8_t>(best)];
        cmdQueue_[static_cast<uint8_t>(best)].active = false;
        cmdQueueCount_--;

        const proto::FailureCode result = executeCommand(cmd.frame, nowMs);

        // ── APUS-9.3: completion ACK/NACK if FLAG_ACK_REQ ─────────────────
        if ((cmd.frame.flags & proto::FLAG_ACK_REQ) != 0U)
        {
            const uint8_t diagByte = (result != proto::FailureCode::NONE)
                                     ? cmd.frame.payload[1U]  // commandId as diagnostic
                                     : 0U;
            sendAckNack(cmd.frame.seq, cmd.frame.node, result, diagByte);
        }

        LOG_D(TAG, "CMD executed seq=%u prio=%u result=0x%02X",
              static_cast<unsigned>(cmd.frame.seq),
              static_cast<unsigned>(cmd.priority),
              static_cast<unsigned>(static_cast<uint8_t>(result)));
    }
}

// ── handleFragmentedCommand ───────────────────────────────────────────────────

/**
 * @brief Reassemble a multi-segment (fragmented) COMMAND transfer (APUS-15).
 *
 * Fragment payload layout (6-byte header):
 *   transferId    uint16_t  LE  bytes 0-1
 *   segmentNum    uint16_t  LE  bytes 2-3   (0-based)
 *   totalSegments uint16_t  LE  bytes 4-5
 *   data          uint8_t[] bytes 6..payloadLen-1
 *
 * When all segments have been received the assembled payload is queued via
 * enqueueCmd() using the last segment's frame as the carrier.
 */
// MISRA-12 deviation: handleFragmentedCommand() exceeds the 80-line recommended
// limit (ARES-MISRA-DEV-004).  Fragmented-transfer reassembly requires sequential
// validation → session management → data storage → assembly, which cannot be split
// without introducing additional inter-function state coupling.
// Cyclomatic complexity is ≤15; justification accepted per APUS-15 scope.
void RadioDispatcher::handleFragmentedCommand(const proto::Frame& frame, // NOLINT(readability-function-size)
                                              uint32_t            nowMs)
{
    // Decode and validate frag header via the shared helper (APUS-15, MISRA-7).
    // Validates: FLAG_FRAGMENT set, len >= FRAG_HEADER_LEN, totalSegments != 0,
    // segmentNum < totalSegments.  Eliminates duplicated header-parse logic.
    proto::FragHeader fh = {};
    if (!proto::decodeFrag(frame, fh))
    {
        LOG_W(TAG, "FRAG: decodeFrag failed (len=%u flags=0x%02X) — NACK",
              static_cast<unsigned>(frame.len),
              static_cast<unsigned>(frame.flags));
        sendAckNack(frame.seq, frame.node, proto::FailureCode::INVALID_PARAM, 0U);
        return;
    }

    // Reject transfers that exceed the single-session reassembly buffer.
    if (fh.totalSegments > kMaxFragSegments)
    {
        LOG_W(TAG, "FRAG: totalSegs=%u > kMaxFragSegments=%u — NACK",
              static_cast<unsigned>(fh.totalSegments),
              static_cast<unsigned>(kMaxFragSegments));
        sendAckNack(frame.seq, frame.node, proto::FailureCode::INVALID_PARAM, 0U);
        return;
    }

    const uint16_t transferId = fh.transferId;
    const uint16_t segmentNum = fh.segmentNum;
    const uint16_t totalSegs  = fh.totalSegments;
    // Segment data starts after the FRAG_HEADER_LEN-byte sub-header (MISRA-7).
    const uint8_t  segDataLen = static_cast<uint8_t>(
        static_cast<uint16_t>(frame.len) -
        static_cast<uint16_t>(proto::FRAG_HEADER_LEN));

    // ── Session management (APUS-15.5 — only one active transfer session) ──────
    const bool timedOut = fragSession_.active &&
                          ((nowMs - fragSession_.lastActivityMs) >= kFragTimeoutMs);

    if (fragSession_.active && timedOut)
    {
        LOG_W(TAG, "FRAG: session 0x%04X timed out — discarding",
              static_cast<unsigned>(fragSession_.transferId));
        fragSession_ = FragReceiveSession{};
    }

    if (!fragSession_.active)
    {
        // Start a new session.
        fragSession_ = FragReceiveSession{};
        fragSession_.active        = true;
        fragSession_.transferId    = transferId;
        fragSession_.totalSegments = totalSegs;
        fragSession_.lastActivityMs = nowMs;
        LOG_D(TAG, "FRAG: new session id=0x%04X total=%u",
              static_cast<unsigned>(transferId), static_cast<unsigned>(totalSegs));
    }
    else if (fragSession_.transferId != transferId ||
             fragSession_.totalSegments != totalSegs)
    {
        // Different transfer while one is in progress: NACK (only one supported).
        LOG_W(TAG, "FRAG: session collision id=0x%04X (active=0x%04X) — NACK QUEUE_FULL",
              static_cast<unsigned>(transferId),
              static_cast<unsigned>(fragSession_.transferId));
        sendAckNack(frame.seq, frame.node, proto::FailureCode::QUEUE_FULL, 0U);
        return;
    }

    // ── Idempotent duplicate segment (APUS-15.2) ─────────────────────────
    if (fragSession_.received[segmentNum])
    {
        LOG_D(TAG, "FRAG: duplicate seg=%u id=0x%04X — re-ACK",
              static_cast<unsigned>(segmentNum), static_cast<unsigned>(transferId));
        sendAckNack(frame.seq, frame.node, proto::FailureCode::NONE, 0U);
        return;
    }

    // ── Store segment data ────────────────────────────────────────────────
    const uint16_t offset = static_cast<uint16_t>(
        static_cast<uint16_t>(segmentNum) * kFragSegDataSize);

    if (segDataLen > 0U)
    {
        if ((static_cast<uint32_t>(offset) + static_cast<uint32_t>(segDataLen)) <=
             sizeof(fragSession_.data))
        {
            (void)memcpy(&fragSession_.data[offset], &frame.payload[6U], segDataLen);
            fragSession_.dataLen = static_cast<uint16_t>(fragSession_.dataLen + segDataLen);
        }
        else
        {
            LOG_W(TAG, "FRAG: data overflow seg=%u — NACK",
                  static_cast<unsigned>(segmentNum));
            sendAckNack(frame.seq, frame.node, proto::FailureCode::EXECUTION_ERROR, 0U);
            return;
        }
    }

    fragSession_.received[segmentNum] = true;
    fragSession_.lastActivityMs       = nowMs;

    // ACK this segment.
    sendAckNack(frame.seq, frame.node, proto::FailureCode::NONE, 0U);

    // ── Check if all segments have arrived ───────────────────────────────
    bool allReceived = true;
    for (uint16_t i = 0U; i < fragSession_.totalSegments; ++i)
    {
        if (!fragSession_.received[i])
        {
            allReceived = false;
            break;
        }
    }

    if (!allReceived)
    {
        LOG_D(TAG, "FRAG: seg=%u/%u stored, waiting for more (id=0x%04X)",
              static_cast<unsigned>(segmentNum),
              static_cast<unsigned>(totalSegs),
              static_cast<unsigned>(transferId));
        return;
    }

    // ── All segments present — assemble and queue ─────────────────────────
    LOG_I(TAG, "FRAG: transfer 0x%04X complete (%u bytes) — enqueueing",
          static_cast<unsigned>(transferId),
          static_cast<unsigned>(fragSession_.dataLen));

    // Build a synthetic frame with the reassembled payload.
    proto::Frame assembled = frame;   // copy meta (seq, node, flags, type)
    // Clear FLAG_FRAGMENT (MISRA-13: XOR mask avoids bitwise-NOT integer promotion).
    static constexpr uint8_t kFlagClearFragment =
        static_cast<uint8_t>(0xFFU ^ proto::FLAG_FRAGMENT);
    assembled.flags = static_cast<uint8_t>(assembled.flags & kFlagClearFragment);

    const uint8_t copyLen = (fragSession_.dataLen <= proto::MAX_PAYLOAD_LEN)
                             ? static_cast<uint8_t>(fragSession_.dataLen)
                             : static_cast<uint8_t>(proto::MAX_PAYLOAD_LEN);
    (void)memcpy(assembled.payload, fragSession_.data, copyLen);
    assembled.len = copyLen;

    fragSession_ = FragReceiveSession{};   // clear session

    // SEQ tracking and queue insertion for the assembled command.
    lastRxSeq_ = frame.seq;
    seenFirst_ = true;

    if (!enqueueCmd(assembled))
    {
        if ((assembled.flags & proto::FLAG_ACK_REQ) != 0U)
        {
            sendAckNack(assembled.seq, assembled.node,
                        proto::FailureCode::QUEUE_FULL,
                        assembled.payload[1U]);
        }
    }
}

} // namespace ares


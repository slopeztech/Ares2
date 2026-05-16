/**
 * @file  test_framing.cpp
 * @brief Unit tests for ares::proto::encode() and decode() (APUS-4).
 *
 * Covers:
 *   Encode:
 *     - Sync word placement (APUS-4.3)
 *     - Minimum frame length (zero-payload HEARTBEAT)
 *     - Rejection of reserved flags (APUS-4.2)
 *     - Rejection of oversized payload (> MAX_PAYLOAD_LEN)
 *     - Acceptance of all valid flag combinations (FLAG_ACK_REQ / RETRANSMIT / PRIORITY)
 *     - Round-trip with maximum payload (200 bytes)
 *     - Round-trips for TELEMETRY, EVENT, COMMAND, ACK message types
 *     - Rejection of null output buffer
 *     - Rejection of undersized output buffer
 *
 *   Decode:
 *     - Round-trip (encode → decode) for HEARTBEAT and with payload
 *     - Rejection on bad sync word
 *     - Rejection on wrong protocol version
 *     - Rejection on reserved flags set
 *     - Rejection on unknown node ID
 *     - Rejection on corrupt CRC (APUS-1)
 *     - Rejection when bufLen < MIN_FRAME_LEN (initial guard)
 *     - Rejection when declared payloadLen makes totalLen exceed bufLen (HEADER_LEN+payloadLen+CRC_LEN > bufLen)
 *     - Acceptance for every known node ID
 *     - Payload bytes are preserved exactly
 */
#include <unity.h>

#include "ares_radio_protocol.h"

#include <cstring>

using namespace ares::proto;

// ── Helper ──────────────────────────────────────────────────

/// Build a minimal valid HEARTBEAT frame into buf and return total frame length.
static uint16_t make_heartbeat(uint8_t*  buf,
                               uint16_t  bufLen,
                               uint8_t   node = NODE_ROCKET,
                               uint8_t   seq  = 0U)
{
    Frame tx = {};
    tx.ver   = PROTOCOL_VERSION;
    tx.flags = 0U;
    tx.node  = node;
    tx.type  = MsgType::HEARTBEAT;
    tx.seq   = seq;
    tx.len   = 0U;
    return encode(tx, buf, bufLen);
}

// ── Encode tests ────────────────────────────────────────────

/// Encode must write the four sync bytes at offsets 0–3 (APUS-4.3).
void test_encode_places_sync_word()
{
    uint8_t  buf[MAX_FRAME_LEN];
    const uint16_t len = make_heartbeat(buf, sizeof(buf));

    TEST_ASSERT_GREATER_THAN_UINT16(0U, len);
    TEST_ASSERT_EQUAL_UINT8(SYNC_0, buf[0]);
    TEST_ASSERT_EQUAL_UINT8(SYNC_1, buf[1]);
    TEST_ASSERT_EQUAL_UINT8(SYNC_2, buf[2]);
    TEST_ASSERT_EQUAL_UINT8(SYNC_3, buf[3]);
}

/// A zero-payload frame must be exactly HEADER_LEN + CRC_LEN bytes.
void test_encode_min_frame_length()
{
    uint8_t  buf[MAX_FRAME_LEN];
    const uint16_t len = make_heartbeat(buf, sizeof(buf));

    TEST_ASSERT_EQUAL_UINT16(MIN_FRAME_LEN, len);  // 10 + 4 = 14
}

/// encode() must return 0 when reserved flags bits are set (APUS-4.2).
void test_encode_rejects_reserved_flags()
{
    Frame tx = {};
    tx.ver   = PROTOCOL_VERSION;
    tx.flags = FLAGS_RESERVED;
    tx.node  = NODE_ROCKET;
    tx.type  = MsgType::HEARTBEAT;
    tx.len   = 0U;

    uint8_t buf[MAX_FRAME_LEN];
    TEST_ASSERT_EQUAL_UINT16(0U, encode(tx, buf, sizeof(buf)));
}

/// encode() must return 0 when payload length exceeds MAX_PAYLOAD_LEN.
void test_encode_rejects_oversized_payload()
{
    Frame tx = {};
    tx.ver  = PROTOCOL_VERSION;
    tx.node = NODE_ROCKET;
    tx.type = MsgType::HEARTBEAT;
    tx.len  = static_cast<uint8_t>(MAX_PAYLOAD_LEN + 1U);

    uint8_t buf[MAX_FRAME_LEN + 16U];
    TEST_ASSERT_EQUAL_UINT16(0U, encode(tx, buf, sizeof(buf)));
}

/// encode() must return 0 when the output buffer pointer is null.
void test_encode_rejects_null_buffer()
{
    Frame tx = {};
    tx.ver  = PROTOCOL_VERSION;
    tx.node = NODE_ROCKET;
    tx.type = MsgType::HEARTBEAT;
    tx.len  = 0U;

    TEST_ASSERT_EQUAL_UINT16(0U, encode(tx, nullptr, MAX_FRAME_LEN));
}

/// encode() must return 0 when the output buffer is too small for the frame.
void test_encode_rejects_undersized_buffer()
{
    Frame tx = {};
    tx.ver  = PROTOCOL_VERSION;
    tx.node = NODE_ROCKET;
    tx.type = MsgType::HEARTBEAT;
    tx.len  = 0U;

    uint8_t buf[MIN_FRAME_LEN - 1U];
    TEST_ASSERT_EQUAL_UINT16(0U, encode(tx, buf, sizeof(buf)));
}

// ── Decode tests ────────────────────────────────────────────

/// Encode then decode a HEARTBEAT — all header fields must survive the round-trip.
void test_decode_roundtrip_heartbeat()
{
    uint8_t  buf[MAX_FRAME_LEN];
    const uint16_t len = make_heartbeat(buf, sizeof(buf), NODE_ROCKET, /*seq=*/77U);

    Frame rx = {};
    TEST_ASSERT_TRUE(decode(buf, len, rx));
    TEST_ASSERT_EQUAL_UINT8(PROTOCOL_VERSION,                          rx.ver);
    TEST_ASSERT_EQUAL_UINT8(NODE_ROCKET,                               rx.node);
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(MsgType::HEARTBEAT),
                            static_cast<uint8_t>(rx.type));
    TEST_ASSERT_EQUAL_UINT8(77U,                                       rx.seq);
    TEST_ASSERT_EQUAL_UINT8(0U,                                        rx.len);
}

/// decode() must return false when the sync word is corrupted (APUS-4.3).
void test_decode_rejects_bad_sync()
{
    uint8_t  buf[MAX_FRAME_LEN];
    const uint16_t len = make_heartbeat(buf, sizeof(buf));

    buf[0] = static_cast<uint8_t>(~SYNC_0);  // corrupt first sync byte

    Frame rx = {};
    TEST_ASSERT_FALSE(decode(buf, len, rx));
}

/// decode() must return false when the protocol version field does not match.
/// Version check precedes the CRC check, so the corrupt CRC is irrelevant.
void test_decode_rejects_wrong_version()
{
    uint8_t  buf[MAX_FRAME_LEN];
    const uint16_t len = make_heartbeat(buf, sizeof(buf));

    buf[4] = static_cast<uint8_t>(PROTOCOL_VERSION - 1U);  // byte 4 = version

    Frame rx = {};
    TEST_ASSERT_FALSE(decode(buf, len, rx));
}

/// decode() must return false when reserved flag bits are set (APUS-4.2).
/// Reserved-flags check precedes the CRC check.
void test_decode_rejects_reserved_flags()
{
    uint8_t  buf[MAX_FRAME_LEN];
    const uint16_t len = make_heartbeat(buf, sizeof(buf));

    buf[5] |= FLAGS_RESERVED;  // byte 5 = flags

    Frame rx = {};
    TEST_ASSERT_FALSE(decode(buf, len, rx));
}

/// decode() must return false for an unknown node ID (APUS-10).
/// Node check precedes the CRC check.
void test_decode_rejects_unknown_node()
{
    uint8_t  buf[MAX_FRAME_LEN];
    const uint16_t len = make_heartbeat(buf, sizeof(buf));

    buf[6] = 0xABU;  // byte 6 = node — not in the known-node table

    Frame rx = {};
    TEST_ASSERT_FALSE(decode(buf, len, rx));
}

/// decode() must return false when the CRC-32 footer is corrupt (APUS-1).
void test_decode_rejects_corrupt_crc()
{
    uint8_t  buf[MAX_FRAME_LEN];
    const uint16_t len = make_heartbeat(buf, sizeof(buf));

    buf[len - 1U] ^= 0xFFU;  // flip all bits in the last CRC byte

    Frame rx = {};
    TEST_ASSERT_FALSE(decode(buf, len, rx));
}

/// decode() must return false when bufLen is less than MIN_FRAME_LEN.
void test_decode_rejects_short_buffer()
{
    uint8_t  buf[MAX_FRAME_LEN];
    make_heartbeat(buf, sizeof(buf));

    Frame rx = {};
    TEST_ASSERT_FALSE(decode(buf, MIN_FRAME_LEN - 1U, rx));
}

/// decode() must succeed for every defined known-node constant (APUS-10.1).
void test_decode_accepts_all_known_nodes()
{
    const uint8_t validNodes[] = {
        NODE_BROADCAST, NODE_ROCKET, NODE_GROUND, NODE_PAYLOAD
    };

    uint8_t buf[MAX_FRAME_LEN];
    for (uint8_t nodeId : validNodes)
    {
        const uint16_t len = make_heartbeat(buf, sizeof(buf), nodeId);

        Frame rx = {};
        TEST_ASSERT_TRUE(decode(buf, len, rx));
        TEST_ASSERT_EQUAL_UINT8(nodeId, rx.node);
    }
}

/// Payload bytes must be preserved exactly through encode → decode.
void test_encode_decode_preserves_payload()
{
    Frame tx = {};
    tx.ver      = PROTOCOL_VERSION;
    tx.flags    = 0U;
    tx.node     = NODE_ROCKET;
    tx.type     = MsgType::HEARTBEAT;  // minPayload = 0, accepts any len
    tx.seq      = 3U;
    tx.len      = 4U;
    tx.payload[0] = 0xDEU;
    tx.payload[1] = 0xADU;
    tx.payload[2] = 0xBEU;
    tx.payload[3] = 0xEFU;

    uint8_t buf[MAX_FRAME_LEN];
    const uint16_t len = encode(tx, buf, sizeof(buf));
    TEST_ASSERT_GREATER_THAN_UINT16(0U, len);

    Frame rx = {};
    TEST_ASSERT_TRUE(decode(buf, len, rx));
    TEST_ASSERT_EQUAL_UINT8(4U, rx.len);
    TEST_ASSERT_EQUAL_MEMORY(tx.payload, rx.payload, tx.len);
}

/// Sequence numbers must survive the round-trip unmodified.
void test_encode_decode_preserves_seq()
{
    uint8_t  buf[MAX_FRAME_LEN];
    for (uint8_t seq = 0U; seq < 5U; ++seq)
    {
        const uint16_t len = make_heartbeat(buf, sizeof(buf), NODE_ROCKET, seq);

        Frame rx = {};
        TEST_ASSERT_TRUE(decode(buf, len, rx));
        TEST_ASSERT_EQUAL_UINT8(seq, rx.seq);
    }
}

// ── Additional encode tests ─────────────────────────────────

/// encode() must succeed (and decode) for all individually valid flag bits
/// (FLAG_ACK_REQ, FLAG_RETRANSMIT, FLAG_PRIORITY) and their combination.
void test_encode_accepts_valid_flags()
{
    const uint8_t validFlags[] = {
        FLAG_ACK_REQ,
        FLAG_RETRANSMIT,
        FLAG_PRIORITY,
        static_cast<uint8_t>(FLAG_ACK_REQ | FLAG_RETRANSMIT | FLAG_PRIORITY),
    };

    uint8_t buf[MAX_FRAME_LEN];
    for (uint8_t flags : validFlags)
    {
        Frame tx = {};
        tx.ver   = PROTOCOL_VERSION;
        tx.flags = flags;
        tx.node  = NODE_ROCKET;
        tx.type  = MsgType::HEARTBEAT;
        tx.len   = 0U;

        const uint16_t len = encode(tx, buf, sizeof(buf));
        TEST_ASSERT_GREATER_THAN_UINT16(0U, len);

        Frame rx = {};
        TEST_ASSERT_TRUE(decode(buf, len, rx));
        TEST_ASSERT_EQUAL_UINT8(flags, rx.flags);
    }
}

/// encode() + decode() with a 200-byte payload (the maximum, MAX_FRAME_LEN total).
void test_encode_decode_max_payload()
{
    Frame tx = {};
    tx.ver  = PROTOCOL_VERSION;
    tx.node = NODE_ROCKET;
    tx.type = MsgType::HEARTBEAT;
    tx.seq  = 0U;
    tx.len  = MAX_PAYLOAD_LEN;
    for (uint8_t i = 0U; i < MAX_PAYLOAD_LEN; ++i)
    {
        tx.payload[i] = i;
    }

    uint8_t buf[MAX_FRAME_LEN];
    const uint16_t len = encode(tx, buf, sizeof(buf));
    TEST_ASSERT_EQUAL_UINT16(MAX_FRAME_LEN, len);

    Frame rx = {};
    TEST_ASSERT_TRUE(decode(buf, len, rx));
    TEST_ASSERT_EQUAL_UINT8(MAX_PAYLOAD_LEN, rx.len);
    TEST_ASSERT_EQUAL_MEMORY(tx.payload, rx.payload, MAX_PAYLOAD_LEN);
}

// ── Additional decode rejection tests ──────────────────────

/// decode() must reject an unknown MsgType value (e.g. NONE = 0x00).
void test_decode_rejects_unknown_msgtype()
{
    uint8_t  buf[MAX_FRAME_LEN];
    const uint16_t len = make_heartbeat(buf, sizeof(buf));

    buf[7] = 0x00U;  // overwrite type with NONE (invalid for transmission)

    Frame rx = {};
    TEST_ASSERT_FALSE(decode(buf, len, rx));
}

/// decode() must reject a frame whose payload is shorter than the type minimum.
/// TELEMETRY requires at least sizeof(TelemetryPayload) = 38 bytes.
/// encode() does NOT enforce this, so we can produce such a frame intentionally.
void test_decode_rejects_payload_too_short_for_type()
{
    Frame tx = {};
    tx.ver  = PROTOCOL_VERSION;
    tx.node = NODE_ROCKET;
    tx.type = MsgType::TELEMETRY;
    tx.len  = 0U;  // deliberately below the 38-byte minimum

    uint8_t buf[MAX_FRAME_LEN];
    const uint16_t len = encode(tx, buf, sizeof(buf));
    TEST_ASSERT_GREATER_THAN_UINT16(0U, len);  // encode succeeds

    Frame rx = {};
    TEST_ASSERT_FALSE(decode(buf, len, rx));   // decode correctly rejects
}

/// decode() must reject when the length field in the wire buffer exceeds MAX_PAYLOAD_LEN.
void test_decode_rejects_oversized_payload_len()
{
    uint8_t  buf[MAX_FRAME_LEN];
    make_heartbeat(buf, sizeof(buf));

    buf[9] = static_cast<uint8_t>(MAX_PAYLOAD_LEN + 1U);  // corrupt len field

    Frame rx = {};
    TEST_ASSERT_FALSE(decode(buf, sizeof(buf), rx));
}

/**
 * decode() must reject when buf[9] declares a payload length that is
 * individually valid (≤ MAX_PAYLOAD_LEN, meets the type minimum) but makes
 * HEADER_LEN + payloadLen + CRC_LEN exceed the actual buffer size by one byte.
 *
 * This exercises the `bufLen < totalLen` guard in validateDecodeHeader(),
 * which is distinct from:
 *   - the `bufLen < MIN_FRAME_LEN` initial check (tested by
 *     test_decode_rejects_short_buffer), and
 *   - the `payloadLen > MAX_PAYLOAD_LEN` check (tested by
 *     test_decode_rejects_oversized_payload_len).
 *
 * Construction: encode a HEARTBEAT with payloadLen=1 → wireLen=15.  Pass
 * bufLen = wireLen − 1 = 14 = MIN_FRAME_LEN.  The declared totalLen (15)
 * exceeds bufLen (14) by exactly one byte, so decode() must return false.
 */
void test_decode_rejects_declared_len_exceeds_actual_buffer()
{
    // Build a properly encoded HEARTBEAT with one payload byte so that
    // wireLen = HEADER_LEN + 1 + CRC_LEN = 15.  The CRC is valid.
    Frame tx = {};
    tx.ver      = PROTOCOL_VERSION;
    tx.node     = NODE_ROCKET;
    tx.type     = MsgType::HEARTBEAT;
    tx.seq      = 0U;
    tx.len      = 1U;
    tx.payload[0] = 0xAAU;  // arbitrary content

    uint8_t buf[MAX_FRAME_LEN];
    const uint16_t wireLen = encode(tx, buf, sizeof(buf));
    TEST_ASSERT_EQUAL_UINT16(
        static_cast<uint16_t>(HEADER_LEN + 1U + CRC_LEN), wireLen);

    // payloadLen (1) ≤ MAX_PAYLOAD_LEN and ≥ HEARTBEAT min (0): valid on its own.
    // But HEADER_LEN + 1 + CRC_LEN = wireLen > wireLen - 1 = bufLen.
    // validateDecodeHeader() must reject at the `bufLen < totalLen` check.
    Frame rx = {};
    TEST_ASSERT_FALSE(
        decode(buf, static_cast<uint16_t>(wireLen - 1U), rx));
}

// ── Round-trip tests for typed message payloads ─────────────

/// TELEMETRY (ST[3]) round-trip: all TelemetryPayload fields preserved (APUS-6).
void test_encode_decode_roundtrip_telemetry()
{
    TelemetryPayload telem = {};
    telem.timestampMs  = 12345U;
    telem.altitudeAglM = 123.4F;
    telem.batteryPct   = 87U;

    Frame tx = {};
    tx.ver  = PROTOCOL_VERSION;
    tx.node = NODE_ROCKET;
    tx.type = MsgType::TELEMETRY;
    tx.seq  = 42U;
    tx.len  = static_cast<uint8_t>(sizeof(TelemetryPayload));
    memcpy(tx.payload, &telem, sizeof(telem));

    uint8_t buf[MAX_FRAME_LEN];
    const uint16_t len = encode(tx, buf, sizeof(buf));
    TEST_ASSERT_GREATER_THAN_UINT16(0U, len);

    Frame rx = {};
    TEST_ASSERT_TRUE(decode(buf, len, rx));
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(MsgType::TELEMETRY),
                            static_cast<uint8_t>(rx.type));
    TEST_ASSERT_EQUAL_UINT8(42U,                             rx.seq);
    TEST_ASSERT_EQUAL_UINT8(sizeof(TelemetryPayload),        rx.len);
    TEST_ASSERT_EQUAL_MEMORY(tx.payload, rx.payload, tx.len);
}

/// EVENT (ST[5]) round-trip: EventHeader fields preserved (APUS-8).
void test_encode_decode_roundtrip_event()
{
    EventHeader evt = {};
    evt.timestampMs = 9999U;
    evt.severity    = static_cast<uint8_t>(EventSeverity::WARN);
    evt.eventId     = static_cast<uint8_t>(EventId::MODE_CHANGE);

    Frame tx = {};
    tx.ver  = PROTOCOL_VERSION;
    tx.node = NODE_ROCKET;
    tx.type = MsgType::EVENT;
    tx.seq  = 5U;
    tx.len  = static_cast<uint8_t>(sizeof(EventHeader));
    memcpy(tx.payload, &evt, sizeof(evt));

    uint8_t buf[MAX_FRAME_LEN];
    const uint16_t len = encode(tx, buf, sizeof(buf));
    TEST_ASSERT_GREATER_THAN_UINT16(0U, len);

    Frame rx = {};
    TEST_ASSERT_TRUE(decode(buf, len, rx));
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(MsgType::EVENT),
                            static_cast<uint8_t>(rx.type));
    TEST_ASSERT_EQUAL_MEMORY(tx.payload, rx.payload, tx.len);
}

/// COMMAND (ST[8]) round-trip: CommandHeader fields preserved (APUS-7).
void test_encode_decode_roundtrip_command()
{
    CommandHeader cmd = {};
    cmd.priority  = static_cast<uint8_t>(Priority::PRI_CRITICAL);
    cmd.commandId = static_cast<uint8_t>(CommandId::ARM_FLIGHT);

    Frame tx = {};
    tx.ver  = PROTOCOL_VERSION;
    tx.node = NODE_GROUND;
    tx.type = MsgType::COMMAND;
    tx.seq  = 1U;
    tx.len  = static_cast<uint8_t>(sizeof(CommandHeader));
    memcpy(tx.payload, &cmd, sizeof(cmd));

    uint8_t buf[MAX_FRAME_LEN];
    const uint16_t len = encode(tx, buf, sizeof(buf));
    TEST_ASSERT_GREATER_THAN_UINT16(0U, len);

    Frame rx = {};
    TEST_ASSERT_TRUE(decode(buf, len, rx));
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(MsgType::COMMAND),
                            static_cast<uint8_t>(rx.type));
    TEST_ASSERT_EQUAL_MEMORY(tx.payload, rx.payload, tx.len);
}

/// ACK (ST[1]) round-trip: AckPayload fields preserved (APUS-9).
void test_encode_decode_roundtrip_ack()
{
    AckPayload ack = {};
    ack.originalSeq  = 9U;
    ack.originalNode = NODE_GROUND;
    ack.failureCode  = static_cast<uint8_t>(FailureCode::NONE);
    ack.failureData  = 0U;

    Frame tx = {};
    tx.ver  = PROTOCOL_VERSION;
    tx.node = NODE_ROCKET;
    tx.type = MsgType::ACK;
    tx.seq  = 10U;
    tx.len  = static_cast<uint8_t>(sizeof(AckPayload));
    memcpy(tx.payload, &ack, sizeof(ack));

    uint8_t buf[MAX_FRAME_LEN];
    const uint16_t len = encode(tx, buf, sizeof(buf));
    TEST_ASSERT_GREATER_THAN_UINT16(0U, len);

    Frame rx = {};
    TEST_ASSERT_TRUE(decode(buf, len, rx));
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(MsgType::ACK),
                            static_cast<uint8_t>(rx.type));
    TEST_ASSERT_EQUAL_MEMORY(tx.payload, rx.payload, tx.len);
}

/// All three valid flags (FLAG_ACK_REQ | FLAG_RETRANSMIT | FLAG_PRIORITY) must
/// survive a full encode → decode round-trip without any bits dropped or added.
/// Verifies that the flags byte is stored verbatim in the wire frame (APUS-4.2).
void test_encode_decode_combined_flags_roundtrip()
{
    constexpr uint8_t kFlags =
        static_cast<uint8_t>(FLAG_ACK_REQ | FLAG_RETRANSMIT | FLAG_PRIORITY);

    Frame tx = {};
    tx.ver   = PROTOCOL_VERSION;
    tx.flags = kFlags;
    tx.node  = NODE_ROCKET;
    tx.type  = MsgType::HEARTBEAT;
    tx.seq   = 42U;
    tx.len   = 0U;

    uint8_t buf[MAX_FRAME_LEN];
    const uint16_t len = encode(tx, buf, sizeof(buf));
    TEST_ASSERT_GREATER_THAN_UINT16(0U, len);

    Frame rx = {};
    TEST_ASSERT_TRUE(decode(buf, len, rx));
    TEST_ASSERT_EQUAL_UINT8(kFlags, rx.flags);
    TEST_ASSERT_EQUAL_UINT8(42U, rx.seq);
}

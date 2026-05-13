/**
 * @file  test_misc.cpp
 * @brief Unit tests for isDuplicate(), decodeFrag(), and encodeFrag()
 *        (APUS-4.6 / APUS-15).
 *
 * Covers:
 *   isDuplicate():
 *     - Returns true when seq == lastSeq
 *     - Returns false when seq != lastSeq
 *     - Edge: both zero, seq 0xFF
 *
 *   decodeFrag():
 *     - Rejection when FLAG_FRAGMENT is not set
 *     - Rejection when payload is shorter than FRAG_HEADER_LEN
 *     - Rejection when totalSegments == 0
 *     - Rejection when segmentNum >= totalSegments
 *     - Correct field parsing for a valid fragment sub-header
 *
 *   encodeFrag() (APUS-15 send path):
 *     - Rejection when data pointer is nullptr
 *     - Rejection when dataLen > MAX_FRAG_PAYLOAD
 *     - Rejection when totalSegments == 0
 *     - Rejection when segmentNum >= totalSegments
 *     - FLAG_FRAGMENT is set in the output frame
 *     - FragHeader fields are written correctly in little-endian order
 *     - Segment data is appended after the sub-header
 *     - frame.len equals FRAG_HEADER_LEN + dataLen
 *     - encodeFrag() + decodeFrag() round-trip preserves all fields
 *     - Single-segment transfer (totalSegments == 1, segmentNum == 0)
 *     - Last segment with full MAX_FRAG_PAYLOAD data bytes
 */
#include <unity.h>

#include "ares_radio_protocol.h"

#include <cstring>

using namespace ares::proto;

// ── Helper ──────────────────────────────────────────────────

/// Encode a fragment frame with the given APUS-15 sub-header values.
static uint16_t make_frag_frame(uint8_t*  buf,
                                uint16_t  bufLen,
                                uint16_t  transferId,
                                uint16_t  segmentNum,
                                uint16_t  totalSegments)
{
    Frame tx = {};
    tx.ver    = PROTOCOL_VERSION;
    tx.flags  = FLAG_FRAGMENT;
    tx.node   = NODE_ROCKET;
    tx.type   = MsgType::HEARTBEAT;  // HEARTBEAT has min payload = 0
    tx.seq    = 0U;
    tx.len    = FRAG_HEADER_LEN;

    // Little-endian encoding (APUS-15)
    tx.payload[0] = static_cast<uint8_t>(transferId        & 0xFFU);
    tx.payload[1] = static_cast<uint8_t>(transferId   >> 8U);
    tx.payload[2] = static_cast<uint8_t>(segmentNum        & 0xFFU);
    tx.payload[3] = static_cast<uint8_t>(segmentNum   >> 8U);
    tx.payload[4] = static_cast<uint8_t>(totalSegments     & 0xFFU);
    tx.payload[5] = static_cast<uint8_t>(totalSegments >> 8U);

    return encode(tx, buf, bufLen);
}

// ── isDuplicate tests ───────────────────────────────────────

/// seq == lastSeq must be detected as a duplicate (APUS-4.6).
void test_is_duplicate_returns_true_same_seq()
{
    TEST_ASSERT_TRUE(isDuplicate(5U, 5U));
}

/// seq != lastSeq must NOT be a duplicate.
void test_is_duplicate_returns_false_different_seq()
{
    TEST_ASSERT_FALSE(isDuplicate(5U, 6U));
}

/// Both zero counts as a duplicate (seq 0 repeated).
void test_is_duplicate_both_zero()
{
    TEST_ASSERT_TRUE(isDuplicate(0U, 0U));
}

/// Edge: seq 0xFF is a duplicate of itself but not of 0x00.
void test_is_duplicate_seq_max()
{
    TEST_ASSERT_TRUE(isDuplicate(255U, 255U));
    TEST_ASSERT_FALSE(isDuplicate(255U, 0U));
}

// ── decodeFrag tests ────────────────────────────────────────

/// decodeFrag() must return false when FLAG_FRAGMENT is not set (APUS-15.1).
void test_decode_frag_rejects_no_flag()
{
    Frame frame = {};
    frame.flags = 0U;          // FLAG_FRAGMENT absent
    frame.len   = FRAG_HEADER_LEN;

    FragHeader frag = {};
    TEST_ASSERT_FALSE(decodeFrag(frame, frag));
}

/// decodeFrag() must return false when payload is shorter than FRAG_HEADER_LEN.
void test_decode_frag_rejects_short_payload()
{
    Frame frame = {};
    frame.flags = FLAG_FRAGMENT;
    frame.len   = static_cast<uint8_t>(FRAG_HEADER_LEN - 1U);

    FragHeader frag = {};
    TEST_ASSERT_FALSE(decodeFrag(frame, frag));
}

/// decodeFrag() must return false when totalSegments == 0 (APUS-15.3).
void test_decode_frag_rejects_zero_total_segments()
{
    uint8_t  buf[MAX_FRAME_LEN];
    const uint16_t len = make_frag_frame(buf, sizeof(buf),
                                         /*transferId=*/1U,
                                         /*segmentNum=*/0U,
                                         /*totalSegments=*/0U);

    Frame frame = {};
    TEST_ASSERT_TRUE(decode(buf, len, frame));  // wire decode succeeds

    FragHeader frag = {};
    TEST_ASSERT_FALSE(decodeFrag(frame, frag));  // fragment validation fails
}

/// decodeFrag() must return false when segmentNum >= totalSegments (APUS-15.3).
void test_decode_frag_rejects_segment_out_of_bounds()
{
    // segmentNum (2) is not < totalSegments (2) → invalid
    uint8_t  buf[MAX_FRAME_LEN];
    const uint16_t len = make_frag_frame(buf, sizeof(buf), 1U, 2U, 2U);

    Frame frame = {};
    TEST_ASSERT_TRUE(decode(buf, len, frame));

    FragHeader frag = {};
    TEST_ASSERT_FALSE(decodeFrag(frame, frag));
}

/// decodeFrag() must parse all three uint16_t fields correctly in little-endian.
void test_decode_frag_valid_header()
{
    uint8_t  buf[MAX_FRAME_LEN];
    const uint16_t len = make_frag_frame(buf, sizeof(buf),
                                         /*transferId=*/0x1234U,
                                         /*segmentNum=*/1U,
                                         /*totalSegments=*/5U);

    Frame frame = {};
    TEST_ASSERT_TRUE(decode(buf, len, frame));

    FragHeader frag = {};
    TEST_ASSERT_TRUE(decodeFrag(frame, frag));
    TEST_ASSERT_EQUAL_UINT16(0x1234U, frag.transferId);
    TEST_ASSERT_EQUAL_UINT16(1U,      frag.segmentNum);
    TEST_ASSERT_EQUAL_UINT16(5U,      frag.totalSegments);
}

// ── encodeFrag tests (APUS-15 send path) ───────────────────

/// Helper: build a minimal Frame skeleton (caller sets seq/node/type before encodeFrag).
static Frame make_frame_skeleton(uint8_t seq = 0U)
{
    Frame f = {};
    f.ver  = PROTOCOL_VERSION;
    f.node = NODE_ROCKET;
    f.type = MsgType::TELEMETRY;
    f.seq  = seq;
    return f;
}

/// encodeFrag() must return false when data pointer is nullptr.
void test_encode_frag_rejects_null_data()
{
    Frame frame = make_frame_skeleton();
    TEST_ASSERT_FALSE(encodeFrag(frame, 1U, 0U, 1U, nullptr, 0U));
}

/// encodeFrag() must return false when dataLen exceeds MAX_FRAG_PAYLOAD.
void test_encode_frag_rejects_oversized_data()
{
    Frame frame = make_frame_skeleton();
    uint8_t data[MAX_FRAG_PAYLOAD + 1U] = {};
    TEST_ASSERT_FALSE(encodeFrag(frame, 1U, 0U, 1U, data,
                                 static_cast<uint8_t>(MAX_FRAG_PAYLOAD + 1U)));
}

/// encodeFrag() must return false when totalSegments == 0.
void test_encode_frag_rejects_zero_total_segments()
{
    Frame  frame = make_frame_skeleton();
    uint8_t data[1] = {0xAAU};
    TEST_ASSERT_FALSE(encodeFrag(frame, 1U, 0U, 0U, data, 1U));
}

/// encodeFrag() must return false when segmentNum >= totalSegments.
void test_encode_frag_rejects_segment_out_of_range()
{
    Frame  frame = make_frame_skeleton();
    uint8_t data[1] = {0xBBU};
    // segmentNum 3 is not < totalSegments 3 → invalid
    TEST_ASSERT_FALSE(encodeFrag(frame, 1U, 3U, 3U, data, 1U));
}

/// encodeFrag() must set FLAG_FRAGMENT in the output frame.
void test_encode_frag_sets_flag_fragment()
{
    Frame  frame = make_frame_skeleton();
    uint8_t data[4] = {0x01U, 0x02U, 0x03U, 0x04U};
    TEST_ASSERT_TRUE(encodeFrag(frame, 1U, 0U, 2U, data, 4U));
    TEST_ASSERT_NOT_EQUAL(0U, frame.flags & FLAG_FRAGMENT);
}

/// encodeFrag() must preserve caller-set flags (e.g. FLAG_PRIORITY) while also
/// setting FLAG_FRAGMENT.
void test_encode_frag_preserves_existing_flags()
{
    Frame  frame = make_frame_skeleton();
    frame.flags  = FLAG_PRIORITY;  // pre-set by caller
    uint8_t data[2] = {0xAAU, 0xBBU};
    TEST_ASSERT_TRUE(encodeFrag(frame, 1U, 0U, 1U, data, 2U));
    TEST_ASSERT_NOT_EQUAL(0U, frame.flags & FLAG_FRAGMENT);
    TEST_ASSERT_NOT_EQUAL(0U, frame.flags & FLAG_PRIORITY);
}

/// encodeFrag() must write FragHeader fields in little-endian byte order.
void test_encode_frag_header_little_endian()
{
    Frame  frame = make_frame_skeleton();
    uint8_t data[1] = {0xFFU};
    TEST_ASSERT_TRUE(encodeFrag(frame, 0xABCDU, 0x0002U, 0x0005U, data, 1U));

    // transferId = 0xABCD → bytes [0]=0xCD, [1]=0xAB
    TEST_ASSERT_EQUAL_UINT8(0xCDU, frame.payload[0]);
    TEST_ASSERT_EQUAL_UINT8(0xABU, frame.payload[1]);
    // segmentNum = 2 → bytes [2]=0x02, [3]=0x00
    TEST_ASSERT_EQUAL_UINT8(0x02U, frame.payload[2]);
    TEST_ASSERT_EQUAL_UINT8(0x00U, frame.payload[3]);
    // totalSegments = 5 → bytes [4]=0x05, [5]=0x00
    TEST_ASSERT_EQUAL_UINT8(0x05U, frame.payload[4]);
    TEST_ASSERT_EQUAL_UINT8(0x00U, frame.payload[5]);
}

/// encodeFrag() must place segment data immediately after the 6-byte sub-header.
void test_encode_frag_data_placed_after_header()
{
    Frame   frame = make_frame_skeleton();
    uint8_t data[3] = {0x11U, 0x22U, 0x33U};
    TEST_ASSERT_TRUE(encodeFrag(frame, 1U, 0U, 1U, data, 3U));

    // Data starts at payload[FRAG_HEADER_LEN]
    TEST_ASSERT_EQUAL_UINT8(0x11U, frame.payload[FRAG_HEADER_LEN]);
    TEST_ASSERT_EQUAL_UINT8(0x22U, frame.payload[FRAG_HEADER_LEN + 1U]);
    TEST_ASSERT_EQUAL_UINT8(0x33U, frame.payload[FRAG_HEADER_LEN + 2U]);
}

/// frame.len must equal FRAG_HEADER_LEN + dataLen after encodeFrag().
void test_encode_frag_sets_correct_len()
{
    Frame   frame = make_frame_skeleton();
    uint8_t data[10] = {};
    TEST_ASSERT_TRUE(encodeFrag(frame, 1U, 0U, 1U, data, 10U));
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(FRAG_HEADER_LEN + 10U), frame.len);
}

/// encodeFrag() → decodeFrag() round-trip must recover all three header fields.
void test_encode_frag_decode_frag_roundtrip()
{
    Frame   frame = make_frame_skeleton();
    uint8_t data[8] = {0xA0U, 0xA1U, 0xA2U, 0xA3U,
                       0xA4U, 0xA5U, 0xA6U, 0xA7U};
    TEST_ASSERT_TRUE(encodeFrag(frame, 0x5A5AU, 2U, 7U, data, 8U));

    FragHeader fh = {};
    TEST_ASSERT_TRUE(decodeFrag(frame, fh));
    TEST_ASSERT_EQUAL_UINT16(0x5A5AU, fh.transferId);
    TEST_ASSERT_EQUAL_UINT16(2U,      fh.segmentNum);
    TEST_ASSERT_EQUAL_UINT16(7U,      fh.totalSegments);
}

/// Single-segment transfer: segmentNum == 0, totalSegments == 1.
void test_encode_frag_single_segment()
{
    Frame   frame = make_frame_skeleton();
    uint8_t data[5] = {1U, 2U, 3U, 4U, 5U};
    TEST_ASSERT_TRUE(encodeFrag(frame, 42U, 0U, 1U, data, 5U));

    // Wire-encode and then wire-decode to verify the full pipeline.
    uint8_t wireBuf[MAX_FRAME_LEN] = {};
    const uint16_t wireLen = encode(frame, wireBuf, sizeof(wireBuf));
    TEST_ASSERT_GREATER_THAN_UINT16(0U, wireLen);

    Frame decoded = {};
    TEST_ASSERT_TRUE(decode(wireBuf, wireLen, decoded));

    FragHeader fh = {};
    TEST_ASSERT_TRUE(decodeFrag(decoded, fh));
    TEST_ASSERT_EQUAL_UINT16(42U, fh.transferId);
    TEST_ASSERT_EQUAL_UINT16(0U,  fh.segmentNum);
    TEST_ASSERT_EQUAL_UINT16(1U,  fh.totalSegments);

    // Verify data integrity.
    TEST_ASSERT_EQUAL_MEMORY(data, &decoded.payload[FRAG_HEADER_LEN], 5U);
}

/// Last segment carrying exactly MAX_FRAG_PAYLOAD data bytes must succeed.
void test_encode_frag_max_payload_segment()
{
    Frame   frame = make_frame_skeleton();
    uint8_t data[MAX_FRAG_PAYLOAD] = {};
    // Fill with a pattern to detect off-by-one corruption.
    for (uint8_t i = 0U; i < MAX_FRAG_PAYLOAD; ++i) { data[i] = i; }

    TEST_ASSERT_TRUE(encodeFrag(frame, 7U, 3U, 4U, data, MAX_FRAG_PAYLOAD));
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(FRAG_HEADER_LEN + MAX_FRAG_PAYLOAD),
                            frame.len);

    // The full frame must still fit within MAX_FRAME_LEN.
    uint8_t wireBuf[MAX_FRAME_LEN] = {};
    const uint16_t wireLen = encode(frame, wireBuf, sizeof(wireBuf));
    TEST_ASSERT_GREATER_THAN_UINT16(0U, wireLen);
    TEST_ASSERT_LESS_OR_EQUAL_UINT16(MAX_FRAME_LEN, wireLen);

    // Data bytes must survive the encode/decode round-trip.
    Frame decoded = {};
    TEST_ASSERT_TRUE(decode(wireBuf, wireLen, decoded));
    TEST_ASSERT_EQUAL_MEMORY(data,
                             &decoded.payload[FRAG_HEADER_LEN],
                             MAX_FRAG_PAYLOAD);
}

/**
 * @file  test_misc.cpp
 * @brief Unit tests for isDuplicate() and decodeFrag() (APUS-4.6 / APUS-15).
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

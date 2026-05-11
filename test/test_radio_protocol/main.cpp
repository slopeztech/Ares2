/**
 * @file  main.cpp
 * @brief Unity test runner for the ARES radio protocol (native target).
 *
 * Entry point for `pio test -e native`.
 * Registers all test functions from test_crc.cpp and test_framing.cpp
 * and delegates execution to the Unity framework.
 *
 * setUp() and tearDown() are defined here once for the whole binary.
 * Each test function is forward-declared so the linker resolves them
 * from their respective translation units without requiring headers.
 */
#include <unity.h>

// ── Lifecycle hooks ─────────────────────────────────────────

void setUp()    {}   ///< Called by Unity before each test function.
void tearDown() {}   ///< Called by Unity after each test function.

// ── Forward declarations: CRC tests (test_crc.cpp) ──────────

extern void test_crc32_check_value();
extern void test_crc32_empty_input();
extern void test_crc32_detects_single_bit_flip();
extern void test_crc32_order_sensitive();
extern void test_crc32_single_byte_distinct();
extern void test_crc32_deterministic();

// ── Forward declarations: framing tests (test_framing.cpp) ──

extern void test_encode_places_sync_word();
extern void test_encode_min_frame_length();
extern void test_encode_rejects_reserved_flags();
extern void test_encode_rejects_oversized_payload();
extern void test_encode_rejects_null_buffer();
extern void test_encode_rejects_undersized_buffer();

extern void test_decode_roundtrip_heartbeat();
extern void test_decode_rejects_bad_sync();
extern void test_decode_rejects_wrong_version();
extern void test_decode_rejects_reserved_flags();
extern void test_decode_rejects_unknown_node();
extern void test_decode_rejects_corrupt_crc();
extern void test_decode_rejects_short_buffer();
extern void test_decode_accepts_all_known_nodes();
extern void test_encode_decode_preserves_payload();
extern void test_encode_decode_preserves_seq();

// ── Forward declarations: additional framing tests (test_framing.cpp) ──

extern void test_encode_accepts_valid_flags();
extern void test_encode_decode_max_payload();
extern void test_decode_rejects_unknown_msgtype();
extern void test_decode_rejects_payload_too_short_for_type();
extern void test_decode_rejects_oversized_payload_len();
extern void test_encode_decode_roundtrip_telemetry();
extern void test_encode_decode_roundtrip_event();
extern void test_encode_decode_roundtrip_command();
extern void test_encode_decode_roundtrip_ack();

// ── Forward declarations: misc tests (test_misc.cpp) ────────

extern void test_is_duplicate_returns_true_same_seq();
extern void test_is_duplicate_returns_false_different_seq();
extern void test_is_duplicate_both_zero();
extern void test_is_duplicate_seq_max();
extern void test_decode_frag_rejects_no_flag();
extern void test_decode_frag_rejects_short_payload();
extern void test_decode_frag_rejects_zero_total_segments();
extern void test_decode_frag_rejects_segment_out_of_bounds();
extern void test_decode_frag_valid_header();

// ── Runner ──────────────────────────────────────────────────

int main()
{
    UNITY_BEGIN();

    // CRC-32 (APUS-1)
    RUN_TEST(test_crc32_check_value);
    RUN_TEST(test_crc32_empty_input);
    RUN_TEST(test_crc32_detects_single_bit_flip);
    RUN_TEST(test_crc32_order_sensitive);
    RUN_TEST(test_crc32_single_byte_distinct);
    RUN_TEST(test_crc32_deterministic);

    // Frame encode (APUS-4)
    RUN_TEST(test_encode_places_sync_word);
    RUN_TEST(test_encode_min_frame_length);
    RUN_TEST(test_encode_rejects_reserved_flags);
    RUN_TEST(test_encode_rejects_oversized_payload);
    RUN_TEST(test_encode_rejects_null_buffer);
    RUN_TEST(test_encode_rejects_undersized_buffer);
    RUN_TEST(test_encode_accepts_valid_flags);
    RUN_TEST(test_encode_decode_max_payload);

    // Frame decode — rejection paths (APUS-4, APUS-1, APUS-10)
    RUN_TEST(test_decode_roundtrip_heartbeat);
    RUN_TEST(test_decode_rejects_bad_sync);
    RUN_TEST(test_decode_rejects_wrong_version);
    RUN_TEST(test_decode_rejects_reserved_flags);
    RUN_TEST(test_decode_rejects_unknown_node);
    RUN_TEST(test_decode_rejects_corrupt_crc);
    RUN_TEST(test_decode_rejects_short_buffer);
    RUN_TEST(test_decode_rejects_unknown_msgtype);
    RUN_TEST(test_decode_rejects_payload_too_short_for_type);
    RUN_TEST(test_decode_rejects_oversized_payload_len);

    // Frame decode — acceptance paths (APUS-10, APUS-4)
    RUN_TEST(test_decode_accepts_all_known_nodes);
    RUN_TEST(test_encode_decode_preserves_payload);
    RUN_TEST(test_encode_decode_preserves_seq);

    // Typed payload round-trips (APUS-5, APUS-6, APUS-7, APUS-8, APUS-9)
    RUN_TEST(test_encode_decode_roundtrip_telemetry);
    RUN_TEST(test_encode_decode_roundtrip_event);
    RUN_TEST(test_encode_decode_roundtrip_command);
    RUN_TEST(test_encode_decode_roundtrip_ack);

    // isDuplicate (APUS-4.6)
    RUN_TEST(test_is_duplicate_returns_true_same_seq);
    RUN_TEST(test_is_duplicate_returns_false_different_seq);
    RUN_TEST(test_is_duplicate_both_zero);
    RUN_TEST(test_is_duplicate_seq_max);

    // decodeFrag (APUS-15)
    RUN_TEST(test_decode_frag_rejects_no_flag);
    RUN_TEST(test_decode_frag_rejects_short_payload);
    RUN_TEST(test_decode_frag_rejects_zero_total_segments);
    RUN_TEST(test_decode_frag_rejects_segment_out_of_bounds);
    RUN_TEST(test_decode_frag_valid_header);

    return UNITY_END();
}

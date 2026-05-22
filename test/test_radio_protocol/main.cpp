/**
 * @file  main.cpp
 * @brief Unity runner for test_radio_protocol tests.
 *
 * GENERATED FILE — do not edit manually.
 * Run: python scripts/gen_unity_runner.py test/test_radio_protocol
 *
 * Test count: 76
 */
#include <unity.h>

void setUp()    {}
void tearDown() {}

// ── test_crc.cpp ─────────────────────────────────────────────────────────────

extern void test_crc32_check_value();
extern void test_crc32_empty_input();
extern void test_crc32_detects_single_bit_flip();
extern void test_crc32_order_sensitive();
extern void test_crc32_single_byte_distinct();
extern void test_crc32_deterministic();

// ── test_framing.cpp ─────────────────────────────────────────────────────────

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
extern void test_encode_accepts_valid_flags();
extern void test_encode_decode_max_payload();
extern void test_decode_rejects_unknown_msgtype();
extern void test_decode_rejects_payload_too_short_for_type();
extern void test_decode_rejects_oversized_payload_len();
extern void test_decode_rejects_declared_len_exceeds_actual_buffer();
extern void test_encode_decode_roundtrip_telemetry();
extern void test_encode_decode_roundtrip_event();
extern void test_encode_decode_roundtrip_command();
extern void test_encode_decode_roundtrip_ack();
extern void test_encode_decode_combined_flags_roundtrip();

// ── test_misc.cpp ────────────────────────────────────────────────────────────

extern void test_is_duplicate_returns_true_same_seq();
extern void test_is_duplicate_returns_false_different_seq();
extern void test_is_duplicate_both_zero();
extern void test_is_duplicate_seq_max();
extern void test_decode_frag_rejects_no_flag();
extern void test_decode_frag_rejects_short_payload();
extern void test_decode_frag_rejects_zero_total_segments();
extern void test_decode_frag_rejects_segment_out_of_bounds();
extern void test_decode_frag_valid_header();
extern void test_encode_frag_rejects_null_data();
extern void test_encode_frag_rejects_oversized_data();
extern void test_encode_frag_rejects_zero_total_segments();
extern void test_encode_frag_rejects_segment_out_of_range();
extern void test_encode_frag_sets_flag_fragment();
extern void test_encode_frag_preserves_existing_flags();
extern void test_encode_frag_header_little_endian();
extern void test_encode_frag_data_placed_after_header();
extern void test_encode_frag_sets_correct_len();
extern void test_encode_frag_decode_frag_roundtrip();
extern void test_encode_frag_single_segment();
extern void test_encode_frag_max_payload_segment();
extern void test_is_duplicate_wraparound_zero_after_max();

// SeqBitmap tests ([H5])
extern void test_seq_bitmap_first_seq_not_duplicate();
extern void test_seq_bitmap_same_seq_is_duplicate();
extern void test_seq_bitmap_sequential_no_duplicate();
extern void test_seq_bitmap_outside_window_rejected();
extern void test_seq_bitmap_back_edge_of_window_accepted();
extern void test_seq_bitmap_out_of_order_within_window();
extern void test_seq_bitmap_wraparound_255_to_0();
extern void test_seq_bitmap_jump_ahead_resets_window();

// ── test_radio_mac.cpp ─────────────────────────────────────────────────────

extern void test_computeHmac8_rfc4231_tc1();
extern void test_computeHmac8_rfc4231_tc2();
extern void test_computeHmac8_deterministic();
extern void test_computeHmac8_distinct_keys();
extern void test_computeHmac8_distinct_msgs();
extern void test_verifyHmac8_accepts_valid();
extern void test_verifyHmac8_rejects_corrupted();
extern void test_appendCommandMac_sets_flag_and_len();
extern void test_appendCommandMac_rejects_non_command();
extern void test_appendCommandMac_rejects_oversized();
extern void test_verifyCommandMac_roundtrip();
extern void test_verifyCommandMac_wrong_key();
extern void test_verifyCommandMac_no_flag();

// ── Runner ───────────────────────────────────────────────────────────────────

int main()
{
    UNITY_BEGIN();

    // crc
    RUN_TEST(test_crc32_check_value);
    RUN_TEST(test_crc32_empty_input);
    RUN_TEST(test_crc32_detects_single_bit_flip);
    RUN_TEST(test_crc32_order_sensitive);
    RUN_TEST(test_crc32_single_byte_distinct);
    RUN_TEST(test_crc32_deterministic);

    // framing
    RUN_TEST(test_encode_places_sync_word);
    RUN_TEST(test_encode_min_frame_length);
    RUN_TEST(test_encode_rejects_reserved_flags);
    RUN_TEST(test_encode_rejects_oversized_payload);
    RUN_TEST(test_encode_rejects_null_buffer);
    RUN_TEST(test_encode_rejects_undersized_buffer);
    RUN_TEST(test_decode_roundtrip_heartbeat);
    RUN_TEST(test_decode_rejects_bad_sync);
    RUN_TEST(test_decode_rejects_wrong_version);
    RUN_TEST(test_decode_rejects_reserved_flags);
    RUN_TEST(test_decode_rejects_unknown_node);
    RUN_TEST(test_decode_rejects_corrupt_crc);
    RUN_TEST(test_decode_rejects_short_buffer);
    RUN_TEST(test_decode_accepts_all_known_nodes);
    RUN_TEST(test_encode_decode_preserves_payload);
    RUN_TEST(test_encode_decode_preserves_seq);
    RUN_TEST(test_encode_accepts_valid_flags);
    RUN_TEST(test_encode_decode_max_payload);
    RUN_TEST(test_decode_rejects_unknown_msgtype);
    RUN_TEST(test_decode_rejects_payload_too_short_for_type);
    RUN_TEST(test_decode_rejects_oversized_payload_len);
    RUN_TEST(test_decode_rejects_declared_len_exceeds_actual_buffer);
    RUN_TEST(test_encode_decode_roundtrip_telemetry);
    RUN_TEST(test_encode_decode_roundtrip_event);
    RUN_TEST(test_encode_decode_roundtrip_command);
    RUN_TEST(test_encode_decode_roundtrip_ack);
    RUN_TEST(test_encode_decode_combined_flags_roundtrip);

    // misc
    RUN_TEST(test_is_duplicate_returns_true_same_seq);
    RUN_TEST(test_is_duplicate_returns_false_different_seq);
    RUN_TEST(test_is_duplicate_both_zero);
    RUN_TEST(test_is_duplicate_seq_max);
    RUN_TEST(test_decode_frag_rejects_no_flag);
    RUN_TEST(test_decode_frag_rejects_short_payload);
    RUN_TEST(test_decode_frag_rejects_zero_total_segments);
    RUN_TEST(test_decode_frag_rejects_segment_out_of_bounds);
    RUN_TEST(test_decode_frag_valid_header);
    RUN_TEST(test_encode_frag_rejects_null_data);
    RUN_TEST(test_encode_frag_rejects_oversized_data);
    RUN_TEST(test_encode_frag_rejects_zero_total_segments);
    RUN_TEST(test_encode_frag_rejects_segment_out_of_range);
    RUN_TEST(test_encode_frag_sets_flag_fragment);
    RUN_TEST(test_encode_frag_preserves_existing_flags);
    RUN_TEST(test_encode_frag_header_little_endian);
    RUN_TEST(test_encode_frag_data_placed_after_header);
    RUN_TEST(test_encode_frag_sets_correct_len);
    RUN_TEST(test_encode_frag_decode_frag_roundtrip);
    RUN_TEST(test_encode_frag_single_segment);
    RUN_TEST(test_encode_frag_max_payload_segment);
    RUN_TEST(test_is_duplicate_wraparound_zero_after_max);

    // SeqBitmap ([H5])
    RUN_TEST(test_seq_bitmap_first_seq_not_duplicate);
    RUN_TEST(test_seq_bitmap_same_seq_is_duplicate);
    RUN_TEST(test_seq_bitmap_sequential_no_duplicate);
    RUN_TEST(test_seq_bitmap_outside_window_rejected);
    RUN_TEST(test_seq_bitmap_back_edge_of_window_accepted);
    RUN_TEST(test_seq_bitmap_out_of_order_within_window);
    RUN_TEST(test_seq_bitmap_wraparound_255_to_0);
    RUN_TEST(test_seq_bitmap_jump_ahead_resets_window);

    // radio MAC (APUS-17, [C1])
    RUN_TEST(test_computeHmac8_rfc4231_tc1);
    RUN_TEST(test_computeHmac8_rfc4231_tc2);
    RUN_TEST(test_computeHmac8_deterministic);
    RUN_TEST(test_computeHmac8_distinct_keys);
    RUN_TEST(test_computeHmac8_distinct_msgs);
    RUN_TEST(test_verifyHmac8_accepts_valid);
    RUN_TEST(test_verifyHmac8_rejects_corrupted);
    RUN_TEST(test_appendCommandMac_sets_flag_and_len);
    RUN_TEST(test_appendCommandMac_rejects_non_command);
    RUN_TEST(test_appendCommandMac_rejects_oversized);
    RUN_TEST(test_verifyCommandMac_roundtrip);
    RUN_TEST(test_verifyCommandMac_wrong_key);
    RUN_TEST(test_verifyCommandMac_no_flag);

    return UNITY_END();
}

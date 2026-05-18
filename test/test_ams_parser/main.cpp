/**
 * @file  main.cpp
 * @brief Unity runner for test_ams_parser tests.
 *
 * GENERATED FILE — do not edit manually.
 * Run: python scripts/gen_unity_runner.py test/test_ams_parser
 *
 * Test count: 16
 */
#include <unity.h>

void setUp()    {}
void tearDown() {}

// ── test_ams_parser_corpus.cpp ───────────────────────────────────────────────

extern void test_parser_utf8_bom_rejected();
extern void test_parser_crlf_endings_accepted();
extern void test_parser_mixed_crlf_lf_accepted();
extern void test_parser_no_trailing_newline_accepted();
extern void test_parser_block_comment_rejected();
extern void test_parser_only_comments_no_states_error();
extern void test_parser_unicode_state_name_accepted();

// ── test_ams_parser_cond_and_meta.cpp ────────────────────────────────────────

extern void test_parser_radio_config_accepted();
extern void test_parser_falling_cond_accepted();
extern void test_parser_rising_cond_accepted();
extern void test_parser_delta_gt_accepted();
extern void test_parser_falling_unknown_alias_error();
extern void test_parser_delta_invalid_threshold_error();
extern void test_parser_delta_bad_operator_error();
extern void test_parser_delta_invalid_lhs_error();
extern void test_parser_sensor_cond_unknown_alias_error();

// ── Runner ───────────────────────────────────────────────────────────────────

int main()
{
    UNITY_BEGIN();

    // ams_parser_corpus
    RUN_TEST(test_parser_utf8_bom_rejected);
    RUN_TEST(test_parser_crlf_endings_accepted);
    RUN_TEST(test_parser_mixed_crlf_lf_accepted);
    RUN_TEST(test_parser_no_trailing_newline_accepted);
    RUN_TEST(test_parser_block_comment_rejected);
    RUN_TEST(test_parser_only_comments_no_states_error);
    RUN_TEST(test_parser_unicode_state_name_accepted);

    // ams_parser_cond_and_meta
    RUN_TEST(test_parser_radio_config_accepted);
    RUN_TEST(test_parser_falling_cond_accepted);
    RUN_TEST(test_parser_rising_cond_accepted);
    RUN_TEST(test_parser_delta_gt_accepted);
    RUN_TEST(test_parser_falling_unknown_alias_error);
    RUN_TEST(test_parser_delta_invalid_threshold_error);
    RUN_TEST(test_parser_delta_bad_operator_error);
    RUN_TEST(test_parser_delta_invalid_lhs_error);
    RUN_TEST(test_parser_sensor_cond_unknown_alias_error);

    return UNITY_END();
}

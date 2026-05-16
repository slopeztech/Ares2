/**
 * @file  main.cpp
 * @brief Unity runner for AMS helper unit tests on native target.
 */
#include <unity.h>

void setUp()    {}
void tearDown() {}

// isOnlyTrailingWhitespace()
extern void test_itw_nullptr_returns_false();
extern void test_itw_empty_string_returns_true();
extern void test_itw_single_space_returns_true();
extern void test_itw_single_tab_returns_true();
extern void test_itw_mixed_whitespace_returns_true();
extern void test_itw_plain_word_returns_false();
extern void test_itw_numeric_with_suffix_returns_false();
extern void test_itw_clean_numeric_remainder_returns_true();
extern void test_itw_strtoul_suffix_residuo_returns_false();
extern void test_itw_strtoul_trailing_space_returns_true();
extern void test_itw_strtoul_trailing_tab_returns_true();
extern void test_itw_strtoul_spaces_then_letters_returns_false();
extern void test_itw_leading_nonwhitespace_returns_false();

// pow10u()
extern void test_pow10u_zero_exponent();
extern void test_pow10u_exponent_one();
extern void test_pow10u_exponent_two();
extern void test_pow10u_exponent_five();
extern void test_pow10u_exponent_nine();

// formatScaledFloat()
extern void test_fsf_null_out_returns_false();
extern void test_fsf_zero_outsize_returns_false();
extern void test_fsf_integer_rounding_no_decimals();
extern void test_fsf_positive_two_decimals();
extern void test_fsf_negative_two_decimals();
extern void test_fsf_round_half_up();
extern void test_fsf_buffer_too_small_returns_false();
extern void test_fsf_negative_integer_rounding_no_decimals();
extern void test_fsf_large_value_formats();

int main()
{
    UNITY_BEGIN();

    RUN_TEST(test_itw_nullptr_returns_false);
    RUN_TEST(test_itw_empty_string_returns_true);
    RUN_TEST(test_itw_single_space_returns_true);
    RUN_TEST(test_itw_single_tab_returns_true);
    RUN_TEST(test_itw_mixed_whitespace_returns_true);
    RUN_TEST(test_itw_plain_word_returns_false);
    RUN_TEST(test_itw_numeric_with_suffix_returns_false);
    RUN_TEST(test_itw_clean_numeric_remainder_returns_true);
    RUN_TEST(test_itw_strtoul_suffix_residuo_returns_false);
    RUN_TEST(test_itw_strtoul_trailing_space_returns_true);
    RUN_TEST(test_itw_strtoul_trailing_tab_returns_true);
    RUN_TEST(test_itw_strtoul_spaces_then_letters_returns_false);
    RUN_TEST(test_itw_leading_nonwhitespace_returns_false);

    RUN_TEST(test_pow10u_zero_exponent);
    RUN_TEST(test_pow10u_exponent_one);
    RUN_TEST(test_pow10u_exponent_two);
    RUN_TEST(test_pow10u_exponent_five);
    RUN_TEST(test_pow10u_exponent_nine);

    RUN_TEST(test_fsf_null_out_returns_false);
    RUN_TEST(test_fsf_zero_outsize_returns_false);
    RUN_TEST(test_fsf_integer_rounding_no_decimals);
    RUN_TEST(test_fsf_positive_two_decimals);
    RUN_TEST(test_fsf_negative_two_decimals);
    RUN_TEST(test_fsf_round_half_up);
    RUN_TEST(test_fsf_buffer_too_small_returns_false);
    RUN_TEST(test_fsf_negative_integer_rounding_no_decimals);
    RUN_TEST(test_fsf_large_value_formats);

    return UNITY_END();
}

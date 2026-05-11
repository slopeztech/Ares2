/**
 * @file  test_format_scaled_float.cpp
 * @brief Unit tests for ares::ams::detail::formatScaledFloat().
 *
 * This helper formats floats using integer arithmetic (no %f), ensuring
 * predictable decimal output for telemetry/log strings.
 *
 * Covers:
 *   - Null output buffer / zero outSize guards
 *   - Integer formatting (decimals = 0)
 *   - Positive and negative decimal formatting
 *   - Rounding behavior (half-up)
 *   - Buffer-too-small rejection
 */
#include <unity.h>

#include "ams/mission_script_engine_helpers.h"

#include <cstring>

using ares::ams::detail::formatScaledFloat;

// ── formatScaledFloat tests ─────────────────────────────────

/// out == nullptr must be rejected safely.
void test_fsf_null_out_returns_false()
{
    TEST_ASSERT_FALSE(formatScaledFloat(1.23F, 2U, nullptr, 8U));
}

/// outSize == 0 must be rejected safely.
void test_fsf_zero_outsize_returns_false()
{
    char out[8] = {};
    TEST_ASSERT_FALSE(formatScaledFloat(1.23F, 2U, out, 0U));
}

/// decimals = 0 should produce integer output with half-up rounding.
void test_fsf_integer_rounding_no_decimals()
{
    char out[16] = {};
    TEST_ASSERT_TRUE(formatScaledFloat(12.6F, 0U, out, sizeof(out)));
    TEST_ASSERT_EQUAL_STRING("13", out);
}

/// Positive value with 2 decimals should match fixed-point output.
void test_fsf_positive_two_decimals()
{
    char out[16] = {};
    TEST_ASSERT_TRUE(formatScaledFloat(12.34F, 2U, out, sizeof(out)));
    TEST_ASSERT_EQUAL_STRING("12.34", out);
}

/// Negative value keeps the sign and decimal digits.
void test_fsf_negative_two_decimals()
{
    char out[16] = {};
    TEST_ASSERT_TRUE(formatScaledFloat(-7.5F, 2U, out, sizeof(out)));
    TEST_ASSERT_EQUAL_STRING("-7.50", out);
}

/// Half-up behavior near the second decimal place.
void test_fsf_round_half_up()
{
    char out[16] = {};
    TEST_ASSERT_TRUE(formatScaledFloat(1.005F, 2U, out, sizeof(out)));
    TEST_ASSERT_EQUAL_STRING("1.01", out);
}

/// When output does not fit, function must return false (no truncation accepted).
void test_fsf_buffer_too_small_returns_false()
{
    char out[4] = {};
    // "123.45" needs 7 chars including NUL; out[4] is intentionally too small.
    TEST_ASSERT_FALSE(formatScaledFloat(123.45F, 2U, out, sizeof(out)));
}

/// decimals = 0 with negative values keeps sign and rounds correctly.
void test_fsf_negative_integer_rounding_no_decimals()
{
    char out[16] = {};
    TEST_ASSERT_TRUE(formatScaledFloat(-3.6F, 0U, out, sizeof(out)));
    TEST_ASSERT_EQUAL_STRING("-4", out);
}

/// Large but representable value should still format when buffer is sufficient.
void test_fsf_large_value_formats()
{
    char out[32] = {};
    TEST_ASSERT_TRUE(formatScaledFloat(123456.78F, 2U, out, sizeof(out)));
    TEST_ASSERT_EQUAL_STRING("123456.78", out);
}

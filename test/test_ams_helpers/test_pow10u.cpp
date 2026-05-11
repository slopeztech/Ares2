/**
 * @file  test_pow10u.cpp
 * @brief Unit tests for ares::ams::detail::pow10u().
 *
 * pow10u() computes 10^digits using integer arithmetic (no FPU).
 * Used by formatScaledFloat() to scale float values before formatting.
 *
 * Covers:
 *   - 10^0 = 1 (identity)
 *   - 10^1 = 10
 *   - 10^2 = 100
 *   - 10^5 = 100000
 *   - 10^9 = 1000000000 (practical upper bound for uint32_t)
 */
#include <unity.h>

#include "ams/mission_script_engine_helpers.h"

using ares::ams::detail::pow10u;

// ── pow10u tests ─────────────────────────────────────────────

/// 10^0 must be 1 (the multiplicative identity).
void test_pow10u_zero_exponent()
{
    TEST_ASSERT_EQUAL_UINT32(1U, pow10u(0U));
}

/// 10^1 = 10.
void test_pow10u_exponent_one()
{
    TEST_ASSERT_EQUAL_UINT32(10U, pow10u(1U));
}

/// 10^2 = 100.
void test_pow10u_exponent_two()
{
    TEST_ASSERT_EQUAL_UINT32(100U, pow10u(2U));
}

/// 10^5 = 100000 — mid-range value used for 5 decimal places.
void test_pow10u_exponent_five()
{
    TEST_ASSERT_EQUAL_UINT32(100000U, pow10u(5U));
}

/// 10^9 = 1000000000 — fits in uint32_t (max ~4.29 × 10^9).
void test_pow10u_exponent_nine()
{
    TEST_ASSERT_EQUAL_UINT32(1000000000U, pow10u(9U));
}

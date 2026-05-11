/**
 * @file  test_trailing_whitespace.cpp
 * @brief Unit tests for ares::ams::detail::isOnlyTrailingWhitespace().
 *
 * This helper is used by the AMS parser to reject suffix characters
 * after numeric literals (e.g. "500ms" must fail; "500" must pass).
 *
 * Covers:
 *   - nullptr input
 *   - Empty string
 *   - Pure space / pure tab
 *   - Mixed spaces and tabs
 *   - Non-whitespace strings
 *   - Real parser use-cases: "500ms" vs "500"
 */
#include <unity.h>

#include "ams/mission_script_engine_helpers.h"

using ares::ams::detail::isOnlyTrailingWhitespace;

// ── isOnlyTrailingWhitespace tests ──────────────────────────

/// nullptr is not a valid empty string — must return false.
void test_itw_nullptr_returns_false()
{
    TEST_ASSERT_FALSE(isOnlyTrailingWhitespace(nullptr));
}

/// The empty string has no non-whitespace characters — must return true.
void test_itw_empty_string_returns_true()
{
    TEST_ASSERT_TRUE(isOnlyTrailingWhitespace(""));
}

/// A single space is purely whitespace.
void test_itw_single_space_returns_true()
{
    TEST_ASSERT_TRUE(isOnlyTrailingWhitespace(" "));
}

/// A single tab is purely whitespace.
void test_itw_single_tab_returns_true()
{
    TEST_ASSERT_TRUE(isOnlyTrailingWhitespace("\t"));
}

/// Mixed spaces and tabs — still purely whitespace.
void test_itw_mixed_whitespace_returns_true()
{
    TEST_ASSERT_TRUE(isOnlyTrailingWhitespace(" \t  \t"));
}

/// Any non-whitespace character makes it non-trailing — must return false.
void test_itw_plain_word_returns_false()
{
    TEST_ASSERT_FALSE(isOnlyTrailingWhitespace("hello"));
}

/// AMS parser use-case: "500ms" has a non-whitespace suffix — must return false.
void test_itw_numeric_with_suffix_returns_false()
{
    TEST_ASSERT_FALSE(isOnlyTrailingWhitespace("500ms"));
}

/// AMS parser use-case: "500" ends at the null terminator — must return true.
void test_itw_clean_numeric_remainder_returns_true()
{
    // After strtoul("500", &end, 10), *end == '\0'; this simulates that case.
    TEST_ASSERT_TRUE(isOnlyTrailingWhitespace(""));
}

/// A string with leading non-whitespace must return false regardless of trailing content.
void test_itw_leading_nonwhitespace_returns_false()
{
    TEST_ASSERT_FALSE(isOnlyTrailingWhitespace("x  "));
}

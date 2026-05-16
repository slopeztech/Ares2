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
#include <cstdlib>

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

/// AMS parser use-case: after strtoul("500", &end, 10), end points at the NUL
/// terminator of the original string — the real residuo must return true.
void test_itw_clean_numeric_remainder_returns_true()
{
    const char* s = "500";
    char* end     = nullptr;
    (void)strtoul(s, &end, 10);
    // end now points at s[3] == '\0'; the actual remainder seen by the parser.
    TEST_ASSERT_NOT_NULL(end);
    TEST_ASSERT_TRUE(isOnlyTrailingWhitespace(end));
}

/// AMS parser use-case: after strtoul("500ms", &end, 10), end points at "ms"
/// — a non-whitespace residuo must return false ("500ms" is a bad literal).
void test_itw_strtoul_suffix_residuo_returns_false()
{
    const char* s = "500ms";
    char* end     = nullptr;
    (void)strtoul(s, &end, 10);
    // end now points at s[3] == 'm'; this is what isOnlyTrailingWhitespace
    // receives in the real parser when it rejects "500ms" as an interval.
    TEST_ASSERT_NOT_NULL(end);
    TEST_ASSERT_FALSE(isOnlyTrailingWhitespace(end));
}

/// After strtoul("500 ", &end), end points at " " — a single trailing space
/// is acceptable whitespace that the parser should allow.
void test_itw_strtoul_trailing_space_returns_true()
{
    char buf[] = "500 ";
    char* end   = nullptr;
    (void)strtoul(buf, &end, 10);
    // end now points at buf[3] == ' '
    TEST_ASSERT_NOT_NULL(end);
    TEST_ASSERT_TRUE(isOnlyTrailingWhitespace(end));
}

/// After strtoul("500\t", &end), end points at "\t" — a single trailing tab
/// is acceptable whitespace that the parser should allow.
void test_itw_strtoul_trailing_tab_returns_true()
{
    char buf[] = "500\t";
    char* end   = nullptr;
    (void)strtoul(buf, &end, 10);
    // end now points at buf[3] == '\t'
    TEST_ASSERT_NOT_NULL(end);
    TEST_ASSERT_TRUE(isOnlyTrailingWhitespace(end));
}

/// After strtoul("500  ms", &end), end points at "  ms" — whitespace followed
/// by letters is not purely whitespace.  The parser must reject "500  ms".
void test_itw_strtoul_spaces_then_letters_returns_false()
{
    char buf[] = "500  ms";
    char* end   = nullptr;
    (void)strtoul(buf, &end, 10);
    // end now points at buf[3] == ' '; "  ms" contains non-whitespace
    TEST_ASSERT_NOT_NULL(end);
    TEST_ASSERT_FALSE(isOnlyTrailingWhitespace(end));
}

/// A string with leading non-whitespace must return false regardless of trailing content.
void test_itw_leading_nonwhitespace_returns_false()
{
    TEST_ASSERT_FALSE(isOnlyTrailingWhitespace("x  "));
}

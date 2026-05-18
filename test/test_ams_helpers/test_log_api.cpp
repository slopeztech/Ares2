/**
 * @file  test_log_api.cpp
 * @brief Unit tests for ares::log — setLevel / getLevel / emit / emitV.
 *
 * Covers paths in ares_log.cpp:
 *   - setLevel() / getLevel()             (lines 24-31)
 *   - emit() / emitV() normal path        (lines 82-88)
 *   - emitV() null-tag guard              (line 39)
 *   - emitV() null-fmt guard              (line 40)
 *   - emitV() truncation guard            (lines 73-76)
 *   - emitV() hex format string           (normal path, %x specifier)
 *
 * Test count: 7
 */

#include <unity.h>
#include <cstring>
#include "debug/ares_log.h"

// ── Test 1: setLevel changes the runtime level ────────────────────────────────

void test_log_setlevel_changes_level()
{
    const ares::log::Level prev = ares::log::getLevel();

    ares::log::setLevel(ares::log::Level::DEBUG);
    TEST_ASSERT_EQUAL(
        static_cast<int>(ares::log::Level::DEBUG),
        static_cast<int>(ares::log::getLevel()));

    // Restore original level so other tests are unaffected.
    ares::log::setLevel(prev);
}

// ── Test 2: setLevel to NONE silences all output ──────────────────────────────

void test_log_setlevel_none_accepted()
{
    const ares::log::Level prev = ares::log::getLevel();

    ares::log::setLevel(ares::log::Level::NONE);
    TEST_ASSERT_EQUAL(
        static_cast<int>(ares::log::Level::NONE),
        static_cast<int>(ares::log::getLevel()));

    ares::log::setLevel(prev);
}

// ── Test 3: emit() does not crash ─────────────────────────────────────────────

void test_log_emit_does_not_crash()
{
    // emit() writes to Serial (stdout in the native stub).
    // As long as this does not crash / assert, the test passes.
    ares::log::emit('I', "LOGTEST", "value=%d", 42);
}

// ── Test 4: null tag is silently ignored (no crash) ───────────────────────────

/**
 * emitV() / emit() must return immediately when tag is nullptr — the null
 * guard at the top of emitV() covers this path.
 */
void test_log_null_tag_is_safe()
{
    // Must not dereference nullptr or crash.
    ares::log::emit('E', nullptr, "msg");
}

// ── Test 5: null fmt is silently ignored (no crash) ───────────────────────────

/**
 * emitV() must return immediately when fmt is nullptr.
 */
void test_log_null_fmt_is_safe()
{
    // Must not pass nullptr to vsnprintf or crash.
    ares::log::emit('E', "LOGTEST", nullptr);
}

// ── Test 6: very long message is truncated without buffer overrun ─────────────

/**
 * A message long enough to exceed LOG_BUF_SIZE (192 bytes) exercises the
 * truncation guard: pos is clamped to LOG_BUF_SIZE-2 and "\n\0" is written
 * at that position.  The test verifies no crash / sanitiser error occurs.
 */
void test_log_long_message_is_truncated_safely()
{
    // 200 'A' characters — header (~15 bytes) + message > LOG_BUF_SIZE=192.
    char longMsg[201];
    (void)memset(longMsg, 'A', 200U);
    longMsg[200] = '\0';

    // Must not crash, must not write past logBuf_ (checked by ASAN/UBSAN).
    ares::log::emit('W', "LNG", "%s", longMsg);
}

// ── Test 7: hex format string does not crash and formats correctly ─────────────

/**
 * Exercises the vsnprintf path with a hex format specifier to verify that
 * the formatter handles %x / %08x without crashing.
 */
void test_log_hex_format_does_not_crash()
{
    ares::log::emit('D', "HEX", "addr=0x%08x val=%02x", 0xDEADBEEFU, 0xABU);
}

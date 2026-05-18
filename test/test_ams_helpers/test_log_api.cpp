/**
 * @file  test_log_api.cpp
 * @brief Unit tests for ares::log — setLevel / getLevel / emit.
 *
 * Covers the three lines of ares_log.cpp not reached by any other suite:
 *   - setLevel()  (lines 24-27)
 *   - emit()      (lines 82-88)
 *
 * Test count: 3
 */

#include <unity.h>
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

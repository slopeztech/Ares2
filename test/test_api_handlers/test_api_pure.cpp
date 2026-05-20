/**
 * @file  test_api_pure.cpp
 * @brief Unit tests for pure API-handler validation helpers.
 *
 * Tests the logic extracted into *_pure.h headers, covering:
 *   - flight_pure.h      : validateModeTransition
 *   - storage_pure.h     : isValidLogFilename, buildLogPath
 *   - mission_pure.h     : isValidMissionFilename, buildMissionPath, toStatusText
 *   - status_pure.h      : gpsStatusToString
 *   - http_parse_pure.h  : owsSkipLeading, owsTrimTrailing (RFC 7230 §3.2.3)
 *
 * No Arduino / WiFi / hardware dependencies — runs in [env:native].
 */
#include <unity.h>
#include <cstring>

#include "api/flight/flight_pure.h"
#include "api/storage/storage_pure.h"
#include "api/mission/mission_pure.h"
#include "api/status/status_pure.h"
#include "api/http_parse_pure.h"

using ares::OperatingMode;
using ares::api::validateModeTransition;
using ares::api::isValidLogFilename;
using ares::api::buildLogPath;
using ares::api::isValidMissionFilename;
using ares::api::buildMissionPath;
using ares::api::toStatusText;
using ares::api::gpsStatusToString;
using ares::api::owsSkipLeading;
using ares::api::owsTrimTrailing;

// ── validateModeTransition ────────────────────────────────────────────────────

void test_vmt_idle_from_test_ok()
{
    OperatingMode out = OperatingMode::ERROR;
    TEST_ASSERT_EQUAL_INT32(0, validateModeTransition("idle", OperatingMode::TEST, out));
    TEST_ASSERT_EQUAL_INT(static_cast<int>(OperatingMode::IDLE), static_cast<int>(out));
}

void test_vmt_idle_from_recovery_ok()
{
    OperatingMode out = OperatingMode::ERROR;
    TEST_ASSERT_EQUAL_INT32(0, validateModeTransition("idle", OperatingMode::RECOVERY, out));
    TEST_ASSERT_EQUAL_INT(static_cast<int>(OperatingMode::IDLE), static_cast<int>(out));
}

void test_vmt_idle_from_error_ok()
{
    OperatingMode out = OperatingMode::FLIGHT;
    TEST_ASSERT_EQUAL_INT32(0, validateModeTransition("idle", OperatingMode::ERROR, out));
    TEST_ASSERT_EQUAL_INT(static_cast<int>(OperatingMode::IDLE), static_cast<int>(out));
}

void test_vmt_test_from_idle_ok()
{
    OperatingMode out = OperatingMode::ERROR;
    TEST_ASSERT_EQUAL_INT32(0, validateModeTransition("test", OperatingMode::IDLE, out));
    TEST_ASSERT_EQUAL_INT(static_cast<int>(OperatingMode::TEST), static_cast<int>(out));
}

void test_vmt_flight_from_idle_ok()
{
    OperatingMode out = OperatingMode::ERROR;
    TEST_ASSERT_EQUAL_INT32(0, validateModeTransition("flight", OperatingMode::IDLE, out));
    TEST_ASSERT_EQUAL_INT(static_cast<int>(OperatingMode::FLIGHT), static_cast<int>(out));
}

void test_vmt_flight_from_test_ok()
{
    OperatingMode out = OperatingMode::ERROR;
    TEST_ASSERT_EQUAL_INT32(0, validateModeTransition("flight", OperatingMode::TEST, out));
    TEST_ASSERT_EQUAL_INT(static_cast<int>(OperatingMode::FLIGHT), static_cast<int>(out));
}

void test_vmt_idle_from_idle_rejects()
{
    OperatingMode out = OperatingMode::IDLE;
    TEST_ASSERT_EQUAL_INT32(409, validateModeTransition("idle", OperatingMode::IDLE, out));
}

void test_vmt_idle_from_flight_rejects()
{
    OperatingMode out = OperatingMode::IDLE;
    TEST_ASSERT_EQUAL_INT32(409, validateModeTransition("idle", OperatingMode::FLIGHT, out));
}

void test_vmt_test_from_flight_rejects()
{
    OperatingMode out = OperatingMode::IDLE;
    TEST_ASSERT_EQUAL_INT32(409, validateModeTransition("test", OperatingMode::FLIGHT, out));
}

void test_vmt_flight_from_recovery_rejects()
{
    OperatingMode out = OperatingMode::IDLE;
    TEST_ASSERT_EQUAL_INT32(409, validateModeTransition("flight", OperatingMode::RECOVERY, out));
}

void test_vmt_unknown_mode_returns_400()
{
    OperatingMode out = OperatingMode::IDLE;
    TEST_ASSERT_EQUAL_INT32(400, validateModeTransition("foobar", OperatingMode::IDLE, out));
}

void test_vmt_out_unchanged_on_400()
{
    // out must not be overwritten when the mode string is unknown.
    OperatingMode out = OperatingMode::RECOVERY;
    (void)validateModeTransition("???", OperatingMode::IDLE, out);
    TEST_ASSERT_EQUAL_INT(static_cast<int>(OperatingMode::RECOVERY), static_cast<int>(out));
}

// ── isValidLogFilename ────────────────────────────────────────────────────────

void test_log_fn_valid_normal()
{
    TEST_ASSERT_TRUE(isValidLogFilename("log_2025-01-01.bin"));
}

void test_log_fn_valid_single_char()
{
    TEST_ASSERT_TRUE(isValidLogFilename("a"));
}

void test_log_fn_valid_uppercase()
{
    TEST_ASSERT_TRUE(isValidLogFilename("LOG.TXT"));
}

void test_log_fn_null_rejects()
{
    TEST_ASSERT_FALSE(isValidLogFilename(nullptr));
}

void test_log_fn_empty_rejects()
{
    TEST_ASSERT_FALSE(isValidLogFilename(""));
}

void test_log_fn_dot_prefix_rejects()
{
    TEST_ASSERT_FALSE(isValidLogFilename(".hidden"));
}

void test_log_fn_slash_rejects()
{
    TEST_ASSERT_FALSE(isValidLogFilename("dir/file.bin"));
}

void test_log_fn_dotdot_rejects()
{
    // Mid-name ".." is a traversal component.
    TEST_ASSERT_FALSE(isValidLogFilename("log..bad"));
}

void test_log_fn_colon_rejects()
{
    TEST_ASSERT_FALSE(isValidLogFilename("bad:name"));
}

void test_log_fn_too_long_rejects()
{
    // 33 chars — one over LOG_FILENAME_MAX (32).
    TEST_ASSERT_FALSE(isValidLogFilename("abcdefghijklmnopqrstuvwxyz1234567"));
}

void test_log_fn_exactly_max_valid()
{
    // 32 chars — exactly LOG_FILENAME_MAX.
    TEST_ASSERT_TRUE(isValidLogFilename("abcdefghijklmnopqrstuvwxyz123456"));
}

// ── buildLogPath ──────────────────────────────────────────────────────────────

void test_log_path_normal()
{
    char buf[64] = {};
    TEST_ASSERT_TRUE(buildLogPath("test.bin", buf, sizeof(buf)));
    TEST_ASSERT_EQUAL_STRING("/logs/test.bin", buf);
}

void test_log_path_too_small_buf()
{
    // Buffer of 5 bytes cannot hold "/logs/test.bin".
    char tiny[5] = {};
    TEST_ASSERT_FALSE(buildLogPath("test.bin", tiny, sizeof(tiny)));
}

// ── isValidMissionFilename ────────────────────────────────────────────────────

void test_msn_fn_valid_ams()
{
    TEST_ASSERT_TRUE(isValidMissionFilename("flight.ams"));
}

void test_msn_fn_valid_uppercase()
{
    TEST_ASSERT_TRUE(isValidMissionFilename("MISSION_01.ams"));
}

void test_msn_fn_null_rejects()
{
    TEST_ASSERT_FALSE(isValidMissionFilename(nullptr));
}

void test_msn_fn_empty_rejects()
{
    TEST_ASSERT_FALSE(isValidMissionFilename(""));
}

void test_msn_fn_dot_prefix_rejects()
{
    TEST_ASSERT_FALSE(isValidMissionFilename(".hidden.ams"));
}

void test_msn_fn_slash_rejects()
{
    TEST_ASSERT_FALSE(isValidMissionFilename("sub/flight.ams"));
}

void test_msn_fn_dotdot_rejects()
{
    TEST_ASSERT_FALSE(isValidMissionFilename("a..b.ams"));
}

void test_msn_fn_too_long_rejects()
{
    // 33 chars — one over MISSION_FILENAME_MAX (32).
    TEST_ASSERT_FALSE(isValidMissionFilename("abcdefghijklmnopqrstuvwxyz1234567"));
}

void test_msn_fn_exactly_max_valid()
{
    // 32 chars — exactly MISSION_FILENAME_MAX.
    TEST_ASSERT_TRUE(isValidMissionFilename("abcdefghijklmnopqrstuvwxyz123456"));
}

// ── buildMissionPath ──────────────────────────────────────────────────────────

void test_msn_path_normal()
{
    char buf[64] = {};
    TEST_ASSERT_TRUE(buildMissionPath("flight.ams", buf, sizeof(buf)));
    TEST_ASSERT_EQUAL_STRING("/missions/flight.ams", buf);
}

void test_msn_path_too_small_buf()
{
    char tiny[5] = {};
    TEST_ASSERT_FALSE(buildMissionPath("flight.ams", tiny, sizeof(tiny)));
}

// ── toStatusText ──────────────────────────────────────────────────────────────

void test_to_status_idle()
{
    TEST_ASSERT_EQUAL_STRING("idle", toStatusText(ares::ams::EngineStatus::IDLE));
}

void test_to_status_loaded()
{
    TEST_ASSERT_EQUAL_STRING("loaded", toStatusText(ares::ams::EngineStatus::LOADED));
}

void test_to_status_running()
{
    TEST_ASSERT_EQUAL_STRING("running", toStatusText(ares::ams::EngineStatus::RUNNING));
}

void test_to_status_complete()
{
    TEST_ASSERT_EQUAL_STRING("complete", toStatusText(ares::ams::EngineStatus::COMPLETE));
}

void test_to_status_error()
{
    TEST_ASSERT_EQUAL_STRING("error", toStatusText(ares::ams::EngineStatus::ERROR));
}

void test_to_status_unknown()
{
    const auto bad = static_cast<ares::ams::EngineStatus>(0xFFU);
    TEST_ASSERT_EQUAL_STRING("unknown", toStatusText(bad));
}

// ── gpsStatusToString ─────────────────────────────────────────────────────────

void test_gps_str_ok()
{
    TEST_ASSERT_EQUAL_STRING("ok", gpsStatusToString(GpsStatus::OK));
}

void test_gps_str_no_fix()
{
    TEST_ASSERT_EQUAL_STRING("no_fix", gpsStatusToString(GpsStatus::NO_FIX));
}

void test_gps_str_error()
{
    TEST_ASSERT_EQUAL_STRING("error", gpsStatusToString(GpsStatus::ERROR));
}

void test_gps_str_not_ready()
{
    TEST_ASSERT_EQUAL_STRING("not_ready", gpsStatusToString(GpsStatus::NOT_READY));
}

void test_gps_str_timeout()
{
    TEST_ASSERT_EQUAL_STRING("timeout", gpsStatusToString(GpsStatus::TIMEOUT));
}

void test_gps_str_invalid_default()
{
    const auto bad = static_cast<GpsStatus>(0xFFU);
    TEST_ASSERT_EQUAL_STRING("invalid", gpsStatusToString(bad));
}

// ── owsSkipLeading (RFC 7230 §3.2.3) ──────────────────────────────────────────

// No whitespace — pointer unchanged.
void test_ows_skip_no_whitespace()
{
    const char* s = "value";
    TEST_ASSERT_EQUAL_PTR(s, owsSkipLeading(s, 64U));
}

// Single space stripped.
void test_ows_skip_single_space()
{
    const char* s = " value";
    TEST_ASSERT_EQUAL_PTR(s + 1, owsSkipLeading(s, 64U));
}

// Single tab stripped.
void test_ows_skip_single_tab()
{
    const char* s = "\tvalue";
    TEST_ASSERT_EQUAL_PTR(s + 1, owsSkipLeading(s, 64U));
}

// Mixed SP and HTAB, all stripped.
void test_ows_skip_mixed_sp_htab()
{
    const char* s = " \t value";   // SP + HTAB + SP = 3 OWS chars before 'v'
    TEST_ASSERT_EQUAL_PTR(s + 3, owsSkipLeading(s, 64U));
}

// More than 4 spaces stripped — key regression for M3 (old cap was 4).
void test_ows_skip_more_than_four_spaces()
{
    const char* s = "          value";  // 10 leading spaces
    TEST_ASSERT_EQUAL_PTR(s + 10, owsSkipLeading(s, 64U));
}

// All whitespace, bound reached — pointer advanced exactly max chars.
void test_ows_skip_all_whitespace_bound()
{
    const char* s = "     ";  // 5 spaces, max = 3
    TEST_ASSERT_EQUAL_PTR(s + 3, owsSkipLeading(s, 3U));
}

// Empty string — pointer unchanged.
void test_ows_skip_empty_string()
{
    const char* s = "";
    TEST_ASSERT_EQUAL_PTR(s, owsSkipLeading(s, 64U));
}

// ── owsTrimTrailing (RFC 7230 §3.2.3) ──────────────────────────────────────

// No trailing whitespace — string unchanged, length returned.
void test_ows_trim_no_trailing()
{
    char buf[] = "mytoken";
    size_t len = owsTrimTrailing(buf, 7U);
    TEST_ASSERT_EQUAL_size_t(7U, len);
    TEST_ASSERT_EQUAL_STRING("mytoken", buf);
}

// Single trailing space removed.
void test_ows_trim_single_trailing_space()
{
    char buf[] = "mytoken ";
    size_t len = owsTrimTrailing(buf, 8U);
    TEST_ASSERT_EQUAL_size_t(7U, len);
    TEST_ASSERT_EQUAL_STRING("mytoken", buf);
}

// Single trailing tab removed.
void test_ows_trim_single_trailing_tab()
{
    char buf[] = "mytoken\t";
    size_t len = owsTrimTrailing(buf, 8U);
    TEST_ASSERT_EQUAL_size_t(7U, len);
    TEST_ASSERT_EQUAL_STRING("mytoken", buf);
}

// Multiple trailing SP/HTAB all removed.
void test_ows_trim_multiple_trailing()
{
    char buf[] = "tok  \t ";
    size_t len = owsTrimTrailing(buf, 7U);
    TEST_ASSERT_EQUAL_size_t(3U, len);
    TEST_ASSERT_EQUAL_STRING("tok", buf);
}

// All whitespace — result is empty string.
void test_ows_trim_all_whitespace()
{
    char buf[] = "   ";
    size_t len = owsTrimTrailing(buf, 3U);
    TEST_ASSERT_EQUAL_size_t(0U, len);
    TEST_ASSERT_EQUAL_STRING("", buf);
}

// Empty string (len = 0) — no-op.
void test_ows_trim_empty()
{
    char buf[] = "";
    size_t len = owsTrimTrailing(buf, 0U);
    TEST_ASSERT_EQUAL_size_t(0U, len);
    TEST_ASSERT_EQUAL_STRING("", buf);
}

// Key regression for M14: token with trailing space must survive round-trip.
void test_ows_trim_token_trailing_space_accepted()
{
    // Simulate a header value copied as "secret " (trailing space).
    char token[] = "secret ";
    owsTrimTrailing(token, 7U);
    TEST_ASSERT_EQUAL_STRING("secret", token);
}

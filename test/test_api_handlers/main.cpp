/**
 * @file  main.cpp
 * @brief Unity runner for test_api_handlers tests.
 *
 * Test count: 48
 */
#include <unity.h>

void setUp()    {}
void tearDown() {}

// ── validateModeTransition (flight_pure.h) ────────────────────────────────────

extern void test_vmt_idle_from_test_ok();
extern void test_vmt_idle_from_recovery_ok();
extern void test_vmt_idle_from_error_ok();
extern void test_vmt_test_from_idle_ok();
extern void test_vmt_flight_from_idle_ok();
extern void test_vmt_flight_from_test_ok();
extern void test_vmt_idle_from_idle_rejects();
extern void test_vmt_idle_from_flight_rejects();
extern void test_vmt_test_from_flight_rejects();
extern void test_vmt_flight_from_recovery_rejects();
extern void test_vmt_unknown_mode_returns_400();
extern void test_vmt_out_unchanged_on_400();

// ── isValidLogFilename / buildLogPath (storage_pure.h) ───────────────────────

extern void test_log_fn_valid_normal();
extern void test_log_fn_valid_single_char();
extern void test_log_fn_valid_uppercase();
extern void test_log_fn_null_rejects();
extern void test_log_fn_empty_rejects();
extern void test_log_fn_dot_prefix_rejects();
extern void test_log_fn_slash_rejects();
extern void test_log_fn_dotdot_rejects();
extern void test_log_fn_colon_rejects();
extern void test_log_fn_too_long_rejects();
extern void test_log_fn_exactly_max_valid();
extern void test_log_path_normal();
extern void test_log_path_too_small_buf();

// ── isValidMissionFilename / buildMissionPath / toStatusText (mission_pure.h) ─

extern void test_msn_fn_valid_ams();
extern void test_msn_fn_valid_uppercase();
extern void test_msn_fn_null_rejects();
extern void test_msn_fn_empty_rejects();
extern void test_msn_fn_dot_prefix_rejects();
extern void test_msn_fn_slash_rejects();
extern void test_msn_fn_dotdot_rejects();
extern void test_msn_fn_too_long_rejects();
extern void test_msn_fn_exactly_max_valid();
extern void test_msn_path_normal();
extern void test_msn_path_too_small_buf();
extern void test_to_status_idle();
extern void test_to_status_loaded();
extern void test_to_status_running();
extern void test_to_status_complete();
extern void test_to_status_error();
extern void test_to_status_unknown();

// ── gpsStatusToString (status_pure.h) ────────────────────────────────────────

extern void test_gps_str_ok();
extern void test_gps_str_no_fix();
extern void test_gps_str_error();
extern void test_gps_str_not_ready();
extern void test_gps_str_timeout();
extern void test_gps_str_invalid_default();

int main(int /*argc*/, char** /*argv*/)
{
    UNITY_BEGIN();

    // validateModeTransition
    RUN_TEST(test_vmt_idle_from_test_ok);
    RUN_TEST(test_vmt_idle_from_recovery_ok);
    RUN_TEST(test_vmt_idle_from_error_ok);
    RUN_TEST(test_vmt_test_from_idle_ok);
    RUN_TEST(test_vmt_flight_from_idle_ok);
    RUN_TEST(test_vmt_flight_from_test_ok);
    RUN_TEST(test_vmt_idle_from_idle_rejects);
    RUN_TEST(test_vmt_idle_from_flight_rejects);
    RUN_TEST(test_vmt_test_from_flight_rejects);
    RUN_TEST(test_vmt_flight_from_recovery_rejects);
    RUN_TEST(test_vmt_unknown_mode_returns_400);
    RUN_TEST(test_vmt_out_unchanged_on_400);

    // isValidLogFilename + buildLogPath
    RUN_TEST(test_log_fn_valid_normal);
    RUN_TEST(test_log_fn_valid_single_char);
    RUN_TEST(test_log_fn_valid_uppercase);
    RUN_TEST(test_log_fn_null_rejects);
    RUN_TEST(test_log_fn_empty_rejects);
    RUN_TEST(test_log_fn_dot_prefix_rejects);
    RUN_TEST(test_log_fn_slash_rejects);
    RUN_TEST(test_log_fn_dotdot_rejects);
    RUN_TEST(test_log_fn_colon_rejects);
    RUN_TEST(test_log_fn_too_long_rejects);
    RUN_TEST(test_log_fn_exactly_max_valid);
    RUN_TEST(test_log_path_normal);
    RUN_TEST(test_log_path_too_small_buf);

    // isValidMissionFilename + buildMissionPath + toStatusText
    RUN_TEST(test_msn_fn_valid_ams);
    RUN_TEST(test_msn_fn_valid_uppercase);
    RUN_TEST(test_msn_fn_null_rejects);
    RUN_TEST(test_msn_fn_empty_rejects);
    RUN_TEST(test_msn_fn_dot_prefix_rejects);
    RUN_TEST(test_msn_fn_slash_rejects);
    RUN_TEST(test_msn_fn_dotdot_rejects);
    RUN_TEST(test_msn_fn_too_long_rejects);
    RUN_TEST(test_msn_fn_exactly_max_valid);
    RUN_TEST(test_msn_path_normal);
    RUN_TEST(test_msn_path_too_small_buf);
    RUN_TEST(test_to_status_idle);
    RUN_TEST(test_to_status_loaded);
    RUN_TEST(test_to_status_running);
    RUN_TEST(test_to_status_complete);
    RUN_TEST(test_to_status_error);
    RUN_TEST(test_to_status_unknown);

    // gpsStatusToString
    RUN_TEST(test_gps_str_ok);
    RUN_TEST(test_gps_str_no_fix);
    RUN_TEST(test_gps_str_error);
    RUN_TEST(test_gps_str_not_ready);
    RUN_TEST(test_gps_str_timeout);
    RUN_TEST(test_gps_str_invalid_default);

    return UNITY_END();
}

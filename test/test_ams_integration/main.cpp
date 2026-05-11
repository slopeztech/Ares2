/**
 * @file  main.cpp
 * @brief Unity runner for AMS SITL integration tests.
 *
 * Links all test groups and executes them as a single PlatformIO
 * native test run under [env:sim].
 */
#include <unity.h>

void setUp()    {}
void tearDown() {}

// ── test_ams_engine_lifecycle.cpp ────────────────────────────────────────────

extern void test_begin_returns_true();
extern void test_initial_status_is_idle();
extern void test_activate_missing_file_returns_false();
extern void test_activate_sets_status_loaded();
extern void test_activate_reports_correct_state_name();
extern void test_arm_transitions_engine_to_running();
extern void test_arm_on_idle_engine_returns_false();
extern void test_tick_consumes_launch_tc_and_transitions();
extern void test_terminal_state_marks_engine_complete();
extern void test_deactivate_returns_engine_to_idle();
extern void test_engine_reactivatable_after_deactivate();

// ── test_ams_flight_scenario.cpp ─────────────────────────────────────────────

extern void test_flight_script_activates_successfully();
extern void test_flight_arm_transitions_to_running();
extern void test_flight_wait_advances_to_flight_state();
extern void test_flight_emits_hk_telemetry_frames();
extern void test_flight_time_transition_fires_at_5s();
extern void test_flight_reaches_complete_after_end_state();
extern void test_sensor_transition_fires_on_altitude_threshold();
extern void test_multiple_hk_frames_accumulate_correctly();

// ── test_ams_engine_control.cpp ───────────────────────────────────────────────

extern void test_abort_tc_deactivates_engine();
extern void test_abort_tc_consumed_by_explicit_transition();
extern void test_pause_stops_tick_execution();
extern void test_resume_restores_running_status();
extern void test_pause_clears_transition_hold_windows();
extern void test_request_telemetry_emits_hk();
extern void test_notify_pulse_a_sets_status_bit();
extern void test_notify_pulse_b_sets_status_bit();

// ── test_ams_script_variants.cpp ──────────────────────────────────────────────

extern void test_parser_error_on_empty_script();
extern void test_fallback_transition_fires_on_timeout();
extern void test_guard_violation_sets_error_status();
extern void test_on_error_recovery_transition_replaces_halt();
extern void test_hold_window_prevents_early_fire();
extern void test_hold_window_fires_after_window();
extern void test_confirm_mode_requires_two_injects();
extern void test_list_scripts_returns_ams_files();

// ── test_ams_pulse.cpp ─────────────────────────────────────────────────────────

extern void test_pulse_fire_a_calls_driver_on_state_entry();
extern void test_pulse_fire_b_calls_driver_on_state_entry();
extern void test_pulse_fire_sets_status_bit_a();
extern void test_pulse_fire_sets_status_bit_b();
extern void test_pulse_fire_duration_override();
extern void test_pulse_fire_only_when_execution_enabled();
extern void test_pulse_fire_no_driver_no_crash();

// ── test_ams_gps_conditions.cpp ────────────────────────────────────────────────

extern void test_gps_sats_transition_fires_above_threshold();
extern void test_gps_hdop_transition_fires_below_threshold();
extern void test_gps_sats_blocks_when_below_threshold();
extern void test_gps_hdop_blocks_when_above_threshold();

// ── test_ams_on_exit_set.cpp ────────────────────────────────────────────────────

extern void test_on_exit_set_script_parses_successfully();
extern void test_on_exit_set_variable_used_in_next_state_condition();

int main()
{
    UNITY_BEGIN();

    // Lifecycle
    RUN_TEST(test_begin_returns_true);
    RUN_TEST(test_initial_status_is_idle);
    RUN_TEST(test_activate_missing_file_returns_false);
    RUN_TEST(test_activate_sets_status_loaded);
    RUN_TEST(test_activate_reports_correct_state_name);
    RUN_TEST(test_arm_transitions_engine_to_running);
    RUN_TEST(test_arm_on_idle_engine_returns_false);
    RUN_TEST(test_tick_consumes_launch_tc_and_transitions);
    RUN_TEST(test_terminal_state_marks_engine_complete);
    RUN_TEST(test_deactivate_returns_engine_to_idle);
    RUN_TEST(test_engine_reactivatable_after_deactivate);

    // Flight scenario
    RUN_TEST(test_flight_script_activates_successfully);
    RUN_TEST(test_flight_arm_transitions_to_running);
    RUN_TEST(test_flight_wait_advances_to_flight_state);
    RUN_TEST(test_flight_emits_hk_telemetry_frames);
    RUN_TEST(test_flight_time_transition_fires_at_5s);
    RUN_TEST(test_flight_reaches_complete_after_end_state);
    RUN_TEST(test_sensor_transition_fires_on_altitude_threshold);
    RUN_TEST(test_multiple_hk_frames_accumulate_correctly);

    // Engine control
    RUN_TEST(test_abort_tc_deactivates_engine);
    RUN_TEST(test_abort_tc_consumed_by_explicit_transition);
    RUN_TEST(test_pause_stops_tick_execution);
    RUN_TEST(test_resume_restores_running_status);
    RUN_TEST(test_pause_clears_transition_hold_windows);
    RUN_TEST(test_request_telemetry_emits_hk);
    RUN_TEST(test_notify_pulse_a_sets_status_bit);
    RUN_TEST(test_notify_pulse_b_sets_status_bit);

    // Script variants
    RUN_TEST(test_parser_error_on_empty_script);
    RUN_TEST(test_fallback_transition_fires_on_timeout);
    RUN_TEST(test_guard_violation_sets_error_status);
    RUN_TEST(test_on_error_recovery_transition_replaces_halt);
    RUN_TEST(test_hold_window_prevents_early_fire);
    RUN_TEST(test_hold_window_fires_after_window);
    RUN_TEST(test_confirm_mode_requires_two_injects);
    RUN_TEST(test_list_scripts_returns_ams_files);

    // Pulse channel commands (AMS-4.17)
    RUN_TEST(test_pulse_fire_a_calls_driver_on_state_entry);
    RUN_TEST(test_pulse_fire_b_calls_driver_on_state_entry);
    RUN_TEST(test_pulse_fire_sets_status_bit_a);
    RUN_TEST(test_pulse_fire_sets_status_bit_b);
    RUN_TEST(test_pulse_fire_duration_override);
    RUN_TEST(test_pulse_fire_only_when_execution_enabled);
    RUN_TEST(test_pulse_fire_no_driver_no_crash);

    // GPS sensor conditions — GPS.sats and GPS.hdop (AMS-4.5)
    RUN_TEST(test_gps_sats_transition_fires_above_threshold);
    RUN_TEST(test_gps_hdop_transition_fires_below_threshold);
    RUN_TEST(test_gps_sats_blocks_when_below_threshold);
    RUN_TEST(test_gps_hdop_blocks_when_above_threshold);

    // on_exit: set actions (AMS-4.9)
    RUN_TEST(test_on_exit_set_script_parses_successfully);
    RUN_TEST(test_on_exit_set_variable_used_in_next_state_condition);

    return UNITY_END();
}

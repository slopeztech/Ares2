/**
 * @file  main.cpp
 * @brief Unity runner for test_ams_integration tests.
 *
 * GENERATED FILE — do not edit manually.
 * Run: python scripts/gen_unity_runner.py test/test_ams_integration
 *
 * Test count: 119
 */
#include <unity.h>
#include "sim_clock.h"

void setUp()    { ares::sim::clock::reset(); }
void tearDown() {}

// ── test_ams_assertions.cpp ──────────────────────────────────────────────────

extern void test_assert_reachable_pass();
extern void test_assert_reachable_fail_unknown_state();
extern void test_assert_reachable_fail_isolated_state();
extern void test_assert_no_dead_states_pass();
extern void test_assert_no_dead_states_fail();
extern void test_assert_max_depth_pass();
extern void test_assert_max_depth_fail_equals_limit();
extern void test_assert_max_depth_cycle_detected();
extern void test_assert_multiple_all_pass();
extern void test_assert_multiple_second_fails();
extern void test_no_assert_block_activates_ok();
extern void test_assert_no_silent_terminals_pass();
extern void test_assert_no_silent_terminals_pass_with_hk();
extern void test_assert_no_silent_terminals_fail();

// ── test_ams_engine_control.cpp ──────────────────────────────────────────────

extern void test_abort_tc_deactivates_engine();
extern void test_abort_tc_consumed_by_explicit_transition();
extern void test_pause_stops_tick_execution();
extern void test_resume_restores_running_status();
extern void test_pause_clears_transition_hold_windows();
extern void test_pause_short_hold_window_not_auto_completed();
extern void test_request_telemetry_emits_hk();
extern void test_notify_pulse_a_sets_status_bit();
extern void test_notify_pulse_b_sets_status_bit();

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
extern void test_checkpoint_resume_running_restores_state();
extern void test_checkpoint_resume_paused_discarded_engine_idle();
extern void test_checkpoint_resume_exec_disabled_discarded_engine_idle();

// ── test_ams_flight_scenario.cpp ─────────────────────────────────────────────

extern void test_flight_script_activates_successfully();
extern void test_flight_arm_transitions_to_running();
extern void test_flight_wait_advances_to_flight_state();
extern void test_flight_emits_hk_telemetry_frames();
extern void test_flight_time_transition_fires_at_5s();
extern void test_flight_reaches_complete_after_end_state();
extern void test_sensor_transition_fires_on_altitude_threshold();
extern void test_multiple_hk_frames_accumulate_correctly();
extern void test_hk_slot_starvation_skips_samples();
extern void test_sensor_cache_within_ttl_is_reused();
extern void test_log_slot_header_retried_after_no_space();
extern void test_log_slot_row_has_crc8_field();

// ── test_ams_gps_conditions.cpp ──────────────────────────────────────────────

extern void test_gps_sats_transition_fires_above_threshold();
extern void test_gps_hdop_transition_fires_below_threshold();
extern void test_gps_sats_blocks_when_below_threshold();
extern void test_gps_hdop_blocks_when_above_threshold();

// ── test_ams_limits.cpp ──────────────────────────────────────────────────────

extern void test_limit_too_many_states();
extern void test_limit_at_max_states_accepted();
extern void test_limit_too_many_transitions();
extern void test_limit_too_many_vars();
extern void test_limit_too_many_consts();
extern void test_limit_too_many_hk_slots();
extern void test_limit_too_many_hk_fields();
extern void test_limit_too_many_includes();
extern void test_limit_script_at_max_bytes_accepted();
extern void test_limit_script_over_max_bytes_truncates();
extern void test_limit_max_depth_full_chain_pass();
extern void test_limit_max_depth_full_chain_fail();

// ── test_ams_on_exit_set.cpp ─────────────────────────────────────────────────

extern void test_on_exit_set_script_parses_successfully();
extern void test_on_exit_set_variable_used_in_next_state_condition();

// ── test_ams_pulse.cpp ───────────────────────────────────────────────────────

extern void test_pulse_fire_a_calls_driver_on_state_entry();
extern void test_pulse_fire_b_calls_driver_on_state_entry();
extern void test_pulse_fire_sets_status_bit_a();
extern void test_pulse_fire_sets_status_bit_b();
extern void test_pulse_fire_duration_override();
extern void test_pulse_fire_only_when_execution_enabled();
extern void test_pulse_fire_no_driver_no_crash();

// ── test_ams_script_variants.cpp ─────────────────────────────────────────────

extern void test_comment_slash_slash_skipped();
extern void test_comment_hash_skipped();
extern void test_parser_error_on_empty_script();
extern void test_fallback_transition_fires_on_timeout();
extern void test_on_timeout_beats_fallback_equal_threshold();
extern void test_guard_violation_sets_error_status();
extern void test_guard_violation_lastError_contains_condition();
extern void test_guard_skipped_when_var_not_set();
extern void test_shadowed_transition_parses_ok();
extern void test_shadowed_transition_dominator_fires();
extern void test_on_error_recovery_transition_replaces_halt();
extern void test_hold_window_prevents_early_fire();
extern void test_hold_window_fires_after_window();
extern void test_confirm_mode_requires_two_injects();
extern void test_list_scripts_returns_ams_files();
extern void test_parser_error_on_line_too_long();
extern void test_parser_error_include_after_state();
extern void test_parser_error_var_after_state();
extern void test_parser_error_const_after_state();
extern void test_parser_error_radio_config_after_state();
extern void test_parser_error_pus_service_after_state();
extern void test_parser_error_pus_apid_after_state();
extern void test_parser_error_pus_apid_out_of_range();
extern void test_parser_error_state_name_with_space();
extern void test_parser_error_state_name_with_hyphen();
extern void test_parser_error_duplicate_state_name();
extern void test_parser_error_duplicate_priorities();
extern void test_typo_state_name_case_suggests_correction();
extern void test_typo_state_name_single_char_suggests_correction();
extern void test_checkpoint_path_traversal_discarded();
extern void test_checkpoint_crc_corruption_discarded();
extern void test_checkpoint_truncated_record_discarded();
extern void test_checkpoint_v4_confirm_restored();
extern void test_checkpoint_millis_rollover_elapsed_wrap();
extern void test_abort_during_confirm_n_in_progress();
extern void test_checkpoint_resume_missing_file();
extern void test_negative_threshold_accel_z_transition_fires();
extern void test_negative_delta_threshold_accel_z_transition_fires();
extern void test_gyro_mag_transition_fires_on_tumbling();
extern void test_var_field_in_log_report();

// ── test_ams_tasks.cpp ───────────────────────────────────────────────────────

extern void test_task_state_filter_script_parses_ok();
extern void test_task_fires_in_filtered_state();
extern void test_task_skips_when_state_filtered_out();
extern void test_task_resumes_promptly_on_state_reentry();

// ── test_ams_tick_scheduling.cpp ─────────────────────────────────────────────

extern void test_wakeup_idle_engine_returns_sensor_rate();
extern void test_wakeup_loaded_engine_returns_sensor_rate();
extern void test_wakeup_state_with_transitions_returns_sensor_rate();
extern void test_wakeup_pure_reporting_state_capped_at_radio_max();
extern void test_wakeup_hk_slot_tightens_deadline();

// ── Runner ───────────────────────────────────────────────────────────────────

int main()
{
    UNITY_BEGIN();

    // ams_assertions
    RUN_TEST(test_assert_reachable_pass);
    RUN_TEST(test_assert_reachable_fail_unknown_state);
    RUN_TEST(test_assert_reachable_fail_isolated_state);
    RUN_TEST(test_assert_no_dead_states_pass);
    RUN_TEST(test_assert_no_dead_states_fail);
    RUN_TEST(test_assert_max_depth_pass);
    RUN_TEST(test_assert_max_depth_fail_equals_limit);
    RUN_TEST(test_assert_max_depth_cycle_detected);
    RUN_TEST(test_assert_multiple_all_pass);
    RUN_TEST(test_assert_multiple_second_fails);
    RUN_TEST(test_no_assert_block_activates_ok);
    RUN_TEST(test_assert_no_silent_terminals_pass);
    RUN_TEST(test_assert_no_silent_terminals_pass_with_hk);
    RUN_TEST(test_assert_no_silent_terminals_fail);

    // ams_engine_control
    RUN_TEST(test_abort_tc_deactivates_engine);
    RUN_TEST(test_abort_tc_consumed_by_explicit_transition);
    RUN_TEST(test_pause_stops_tick_execution);
    RUN_TEST(test_resume_restores_running_status);
    RUN_TEST(test_pause_clears_transition_hold_windows);
    RUN_TEST(test_pause_short_hold_window_not_auto_completed);
    RUN_TEST(test_request_telemetry_emits_hk);
    RUN_TEST(test_notify_pulse_a_sets_status_bit);
    RUN_TEST(test_notify_pulse_b_sets_status_bit);

    // ams_engine_lifecycle
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
    RUN_TEST(test_checkpoint_resume_running_restores_state);
    RUN_TEST(test_checkpoint_resume_paused_discarded_engine_idle);
    RUN_TEST(test_checkpoint_resume_exec_disabled_discarded_engine_idle);

    // ams_flight_scenario
    RUN_TEST(test_flight_script_activates_successfully);
    RUN_TEST(test_flight_arm_transitions_to_running);
    RUN_TEST(test_flight_wait_advances_to_flight_state);
    RUN_TEST(test_flight_emits_hk_telemetry_frames);
    RUN_TEST(test_flight_time_transition_fires_at_5s);
    RUN_TEST(test_flight_reaches_complete_after_end_state);
    RUN_TEST(test_sensor_transition_fires_on_altitude_threshold);
    RUN_TEST(test_multiple_hk_frames_accumulate_correctly);
    RUN_TEST(test_hk_slot_starvation_skips_samples);
    RUN_TEST(test_sensor_cache_within_ttl_is_reused);
    RUN_TEST(test_log_slot_header_retried_after_no_space);
    RUN_TEST(test_log_slot_row_has_crc8_field);

    // ams_gps_conditions
    RUN_TEST(test_gps_sats_transition_fires_above_threshold);
    RUN_TEST(test_gps_hdop_transition_fires_below_threshold);
    RUN_TEST(test_gps_sats_blocks_when_below_threshold);
    RUN_TEST(test_gps_hdop_blocks_when_above_threshold);

    // ams_limits
    RUN_TEST(test_limit_too_many_states);
    RUN_TEST(test_limit_at_max_states_accepted);
    RUN_TEST(test_limit_too_many_transitions);
    RUN_TEST(test_limit_too_many_vars);
    RUN_TEST(test_limit_too_many_consts);
    RUN_TEST(test_limit_too_many_hk_slots);
    RUN_TEST(test_limit_too_many_hk_fields);
    RUN_TEST(test_limit_too_many_includes);
    RUN_TEST(test_limit_script_at_max_bytes_accepted);
    RUN_TEST(test_limit_script_over_max_bytes_truncates);
    RUN_TEST(test_limit_max_depth_full_chain_pass);
    RUN_TEST(test_limit_max_depth_full_chain_fail);

    // ams_on_exit_set
    RUN_TEST(test_on_exit_set_script_parses_successfully);
    RUN_TEST(test_on_exit_set_variable_used_in_next_state_condition);

    // ams_pulse
    RUN_TEST(test_pulse_fire_a_calls_driver_on_state_entry);
    RUN_TEST(test_pulse_fire_b_calls_driver_on_state_entry);
    RUN_TEST(test_pulse_fire_sets_status_bit_a);
    RUN_TEST(test_pulse_fire_sets_status_bit_b);
    RUN_TEST(test_pulse_fire_duration_override);
    RUN_TEST(test_pulse_fire_only_when_execution_enabled);
    RUN_TEST(test_pulse_fire_no_driver_no_crash);

    // ams_script_variants
    RUN_TEST(test_comment_slash_slash_skipped);
    RUN_TEST(test_comment_hash_skipped);
    RUN_TEST(test_parser_error_on_empty_script);
    RUN_TEST(test_fallback_transition_fires_on_timeout);
    RUN_TEST(test_on_timeout_beats_fallback_equal_threshold);
    RUN_TEST(test_guard_violation_sets_error_status);
    RUN_TEST(test_guard_violation_lastError_contains_condition);
    RUN_TEST(test_guard_skipped_when_var_not_set);
    RUN_TEST(test_shadowed_transition_parses_ok);
    RUN_TEST(test_shadowed_transition_dominator_fires);
    RUN_TEST(test_on_error_recovery_transition_replaces_halt);
    RUN_TEST(test_hold_window_prevents_early_fire);
    RUN_TEST(test_hold_window_fires_after_window);
    RUN_TEST(test_confirm_mode_requires_two_injects);
    RUN_TEST(test_list_scripts_returns_ams_files);
    RUN_TEST(test_parser_error_on_line_too_long);
    RUN_TEST(test_parser_error_include_after_state);
    RUN_TEST(test_parser_error_var_after_state);
    RUN_TEST(test_parser_error_const_after_state);
    RUN_TEST(test_parser_error_radio_config_after_state);
    RUN_TEST(test_parser_error_pus_service_after_state);
    RUN_TEST(test_parser_error_pus_apid_after_state);
    RUN_TEST(test_parser_error_pus_apid_out_of_range);
    RUN_TEST(test_parser_error_state_name_with_space);
    RUN_TEST(test_parser_error_state_name_with_hyphen);
    RUN_TEST(test_parser_error_duplicate_state_name);
    RUN_TEST(test_parser_error_duplicate_priorities);
    RUN_TEST(test_typo_state_name_case_suggests_correction);
    RUN_TEST(test_typo_state_name_single_char_suggests_correction);
    RUN_TEST(test_checkpoint_path_traversal_discarded);
    RUN_TEST(test_checkpoint_crc_corruption_discarded);
    RUN_TEST(test_checkpoint_truncated_record_discarded);
    RUN_TEST(test_checkpoint_v4_confirm_restored);
    RUN_TEST(test_checkpoint_millis_rollover_elapsed_wrap);
    RUN_TEST(test_abort_during_confirm_n_in_progress);
    RUN_TEST(test_checkpoint_resume_missing_file);
    RUN_TEST(test_negative_threshold_accel_z_transition_fires);
    RUN_TEST(test_negative_delta_threshold_accel_z_transition_fires);
    RUN_TEST(test_gyro_mag_transition_fires_on_tumbling);
    RUN_TEST(test_var_field_in_log_report);

    // ams_tasks
    RUN_TEST(test_task_state_filter_script_parses_ok);
    RUN_TEST(test_task_fires_in_filtered_state);
    RUN_TEST(test_task_skips_when_state_filtered_out);
    RUN_TEST(test_task_resumes_promptly_on_state_reentry);

    // ams_tick_scheduling
    RUN_TEST(test_wakeup_idle_engine_returns_sensor_rate);
    RUN_TEST(test_wakeup_loaded_engine_returns_sensor_rate);
    RUN_TEST(test_wakeup_state_with_transitions_returns_sensor_rate);
    RUN_TEST(test_wakeup_pure_reporting_state_capped_at_radio_max);
    RUN_TEST(test_wakeup_hk_slot_tightens_deadline);

    return UNITY_END();
}

/**
 * @file  main.cpp
 * @brief Unity runner for test_dispatcher tests.
 *
 * Test count: 55
 */
#include <unity.h>

void setUp()    {}
void tearDown() {}

// ── test_fragment_reassembly.cpp ─────────────────────────────────────────────

extern void test_frag_two_segments_reassemble();
extern void test_frag_orphaned_session_no_phantom_execution();
extern void test_frag_session_timeout_discards_and_restarts();
extern void test_frag_session_collision_sends_nack();
extern void test_frag_orphaned_session_expires_and_slot_recycled();

// ── test_command_dispatch.cpp ─────────────────────────────────────────────────

// Non-fragmented COMMAND paths
extern void test_cmd_nonfrag_request_status();
extern void test_cmd_nonfrag_request_config();
extern void test_cmd_nonfrag_verify_config_clean();
extern void test_cmd_nonfrag_set_mode_enable();
extern void test_cmd_nonfrag_set_mode_disable();
extern void test_cmd_nonfrag_set_mode_invalid();
extern void test_cmd_nonfrag_set_fcs_active();
extern void test_cmd_nonfrag_set_config_param_valid();
extern void test_cmd_nonfrag_set_config_param_out_of_range();
extern void test_cmd_nonfrag_arm_flight_precondition_fail();
extern void test_cmd_nonfrag_abort_missing_priority_rejected();
extern void test_cmd_nonfrag_abort_with_priority_idle_engine();
extern void test_cmd_nonfrag_fire_pulse_a_engine_not_running();
extern void test_cmd_nonfrag_factory_reset_with_priority();
extern void test_cmd_nonfrag_request_telemetry_idle_engine();
extern void test_cmd_nonfrag_set_telem_interval_valid();
extern void test_cmd_flag_ack_req_triggers_completion_ack();
// Duplicate detection
extern void test_cmd_duplicate_is_discarded();
// Anti-replay ([H5])
extern void test_cmd_replay_previous_seq_rejected();
extern void test_cmd_replay_outside_window_rejected();
// HEARTBEAT
extern void test_heartbeat_gets_response();
// Discard paths
extern void test_telemetry_frame_silently_discarded();
extern void test_event_frame_silently_discarded();
extern void test_ack_frame_clears_retry_no_side_effect();
// Routing failure
extern void test_cmd_wrong_node_sends_routing_nack();
// Buffer-level corruption / garbage
extern void test_buffer_no_sync_byte_discards_all();
extern void test_buffer_leading_garbage_then_valid_frame();
extern void test_buffer_corrupt_crc_frame_skipped();
// ACK with no payload data
extern void test_ack_frame_short_no_payload();
// Unknown command ID
extern void test_cmd_nonfrag_unknown_commandid();
// Payload-too-short paths
extern void test_cmd_nonfrag_set_mode_short_payload();
extern void test_cmd_nonfrag_set_fcs_short_payload();
extern void test_cmd_nonfrag_set_fcs_invalid_value();
extern void test_cmd_nonfrag_set_telem_interval_out_of_range();
extern void test_cmd_nonfrag_set_config_param_short_payload();
// Corrupt declared-length field in processBuffer
extern void test_buffer_corrupt_length_field_skips_sync();
// Two consecutive frames (tests tail-memmove branch)
extern void test_buffer_two_consecutive_heartbeats();
// Queue-full NACK
extern void test_cmd_queue_full_sends_nack();
// SET_TELEM_INTERVAL short payload
extern void test_cmd_nonfrag_set_telem_interval_short_payload();
// Gap commandId falls to default case in switch
extern void test_cmd_nonfrag_gap_commandid_default_case();
// SET_CONFIG_PARAM unknown paramId and monitor-param success
extern void test_cmd_nonfrag_set_config_unknown_paramid();
extern void test_cmd_nonfrag_set_config_param_monitor_valid();
// FIRE_PULSE_B/C/D: FLAG_PRIORITY guard and PRECONDITION_FAIL paths
extern void test_cmd_nonfrag_fire_pulse_b_missing_priority_rejected();
extern void test_cmd_nonfrag_fire_pulse_c_missing_priority_rejected();
extern void test_cmd_nonfrag_fire_pulse_d_missing_priority_rejected();
extern void test_cmd_nonfrag_fire_pulse_c_engine_not_running();
extern void test_cmd_nonfrag_fire_pulse_d_engine_not_running();
// SET_TELEM_INTERVAL boundary tests ([M2])
extern void test_cmd_nonfrag_set_telem_interval_at_min();
extern void test_cmd_nonfrag_set_telem_interval_at_max();
extern void test_cmd_nonfrag_set_telem_interval_above_max();

// ── Runner ───────────────────────────────────────────────────────────────────

int main()
{
    UNITY_BEGIN();

    // fragment reassembly
    RUN_TEST(test_frag_two_segments_reassemble);
    RUN_TEST(test_frag_orphaned_session_no_phantom_execution);
    RUN_TEST(test_frag_session_timeout_discards_and_restarts);
    RUN_TEST(test_frag_session_collision_sends_nack);
    RUN_TEST(test_frag_orphaned_session_expires_and_slot_recycled);

    // non-fragmented command dispatch
    RUN_TEST(test_cmd_nonfrag_request_status);
    RUN_TEST(test_cmd_nonfrag_request_config);
    RUN_TEST(test_cmd_nonfrag_verify_config_clean);
    RUN_TEST(test_cmd_nonfrag_set_mode_enable);
    RUN_TEST(test_cmd_nonfrag_set_mode_disable);
    RUN_TEST(test_cmd_nonfrag_set_mode_invalid);
    RUN_TEST(test_cmd_nonfrag_set_fcs_active);
    RUN_TEST(test_cmd_nonfrag_set_config_param_valid);
    RUN_TEST(test_cmd_nonfrag_set_config_param_out_of_range);
    RUN_TEST(test_cmd_nonfrag_arm_flight_precondition_fail);
    RUN_TEST(test_cmd_nonfrag_abort_missing_priority_rejected);
    RUN_TEST(test_cmd_nonfrag_abort_with_priority_idle_engine);
    RUN_TEST(test_cmd_nonfrag_fire_pulse_a_engine_not_running);
    RUN_TEST(test_cmd_nonfrag_factory_reset_with_priority);
    RUN_TEST(test_cmd_nonfrag_request_telemetry_idle_engine);
    RUN_TEST(test_cmd_nonfrag_set_telem_interval_valid);
    RUN_TEST(test_cmd_flag_ack_req_triggers_completion_ack);

    // duplicate detection and anti-replay ([H5])
    RUN_TEST(test_cmd_duplicate_is_discarded);
    RUN_TEST(test_cmd_replay_previous_seq_rejected);
    RUN_TEST(test_cmd_replay_outside_window_rejected);

    // HEARTBEAT
    RUN_TEST(test_heartbeat_gets_response);

    // discard paths
    RUN_TEST(test_telemetry_frame_silently_discarded);
    RUN_TEST(test_event_frame_silently_discarded);
    RUN_TEST(test_ack_frame_clears_retry_no_side_effect);

    // routing failure
    RUN_TEST(test_cmd_wrong_node_sends_routing_nack);

    // buffer-level corruption / garbage
    RUN_TEST(test_buffer_no_sync_byte_discards_all);
    RUN_TEST(test_buffer_leading_garbage_then_valid_frame);
    RUN_TEST(test_buffer_corrupt_crc_frame_skipped);
    RUN_TEST(test_ack_frame_short_no_payload);
    RUN_TEST(test_cmd_nonfrag_unknown_commandid);
    RUN_TEST(test_cmd_nonfrag_set_mode_short_payload);
    RUN_TEST(test_cmd_nonfrag_set_fcs_short_payload);
    RUN_TEST(test_cmd_nonfrag_set_fcs_invalid_value);
    RUN_TEST(test_cmd_nonfrag_set_telem_interval_out_of_range);
    RUN_TEST(test_cmd_nonfrag_set_config_param_short_payload);
    RUN_TEST(test_buffer_corrupt_length_field_skips_sync);
    RUN_TEST(test_buffer_two_consecutive_heartbeats);
    RUN_TEST(test_cmd_queue_full_sends_nack);
    RUN_TEST(test_cmd_nonfrag_set_telem_interval_short_payload);
    RUN_TEST(test_cmd_nonfrag_gap_commandid_default_case);
    RUN_TEST(test_cmd_nonfrag_set_config_unknown_paramid);
    RUN_TEST(test_cmd_nonfrag_set_config_param_monitor_valid);

    // FIRE_PULSE_B/C/D
    RUN_TEST(test_cmd_nonfrag_fire_pulse_b_missing_priority_rejected);
    RUN_TEST(test_cmd_nonfrag_fire_pulse_c_missing_priority_rejected);
    RUN_TEST(test_cmd_nonfrag_fire_pulse_d_missing_priority_rejected);
    RUN_TEST(test_cmd_nonfrag_fire_pulse_c_engine_not_running);
    RUN_TEST(test_cmd_nonfrag_fire_pulse_d_engine_not_running);

    // SET_TELEM_INTERVAL boundary tests ([M2])
    RUN_TEST(test_cmd_nonfrag_set_telem_interval_at_min);
    RUN_TEST(test_cmd_nonfrag_set_telem_interval_at_max);
    RUN_TEST(test_cmd_nonfrag_set_telem_interval_above_max);

    return UNITY_END();
}

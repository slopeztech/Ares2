/**
 * @file  main.cpp
 * @brief Unity test runner for the api_routing test suite.
 *
 * Registers all routing, auth, and device-config test cases and runs them
 * via the Unity C unit-test framework.
 */

#include <unity.h>

// Test function declarations (defined in test_api_routing.cpp)
void test_unknown_path_returns_404(void);
void test_delete_status_returns_405(void);
void test_post_config_returns_405(void);
void test_options_arm_returns_204(void);
void test_abort_in_idle_returns_409(void);
void test_mode_invalid_json_returns_400(void);
void test_arm_no_mission_returns_500(void);
void test_auth_arm_no_token_returns_401(void);
void test_auth_mode_wrong_token_returns_401(void);
void test_auth_mission_upload_no_token_returns_401(void);

// Test function declarations (defined in test_device_config.cpp)
void test_checktoken_correct_accepted(void);
void test_checktoken_extra_byte_rejected(void);
void test_checktoken_short_token_rejected(void);
void test_checktoken_empty_token_rejected(void);
void test_checktoken_null_rejected(void);
void test_checktoken_auth_disabled_always_false(void);
void test_topublicjson_zero_size_returns_zero(void);
void test_topublicjson_small_size_returns_zero(void);
void test_topublicjson_normal_size_returns_json(void);
void test_empty_wifi_password_preserves_current(void);
void test_put_wifi_password_change_returns_202(void);
void test_put_no_wifi_password_change_returns_200(void);
void test_oversized_path_routerequest_returns_404(void);
void test_put_wifi_password_202_has_reboot_required(void);
void test_put_no_wifi_change_200_no_reboot_required(void);
void test_i2c_scan_blocked_in_flight(void);
void test_uart_scan_blocked_in_flight(void);
void test_status_200_reason_phrase_ok(void);
void test_put_device_config_202_reason_accepted(void);

// Test function declarations (defined in test_http_parser.cpp)
void test_hc_empty_request_no_response(void);
void test_hc_bare_crlf_no_response(void);
void test_hc_lf_only_line_endings_returns_200(void);
void test_hc_negative_content_length_returns_413(void);
void test_hc_oversized_content_length_returns_413(void);
void test_hc_truncated_body_returns_400(void);
void test_hc_body_without_content_type_returns_400(void);
void test_hc_uri_too_long_returns_414(void);
void test_hc_header_flood_returns_200(void);
void test_hc_malformed_header_no_colon_returns_200(void);

int main(void)
{
    UNITY_BEGIN();

    // ── Routing tests (shared server, non-mutating first) ────
    RUN_TEST(test_unknown_path_returns_404);
    RUN_TEST(test_delete_status_returns_405);
    RUN_TEST(test_post_config_returns_405);
    RUN_TEST(test_options_arm_returns_204);
    RUN_TEST(test_abort_in_idle_returns_409);
    RUN_TEST(test_mode_invalid_json_returns_400);
    // State-mutating shared-server test last:
    RUN_TEST(test_arm_no_mission_returns_500);

    // ── Auth tests (each creates its own server instance) ────
    RUN_TEST(test_auth_arm_no_token_returns_401);
    RUN_TEST(test_auth_mode_wrong_token_returns_401);
    RUN_TEST(test_auth_mission_upload_no_token_returns_401);

    // ── DeviceConfig security tests ───────────────────────────
    RUN_TEST(test_checktoken_correct_accepted);
    RUN_TEST(test_checktoken_extra_byte_rejected);
    RUN_TEST(test_checktoken_short_token_rejected);
    RUN_TEST(test_checktoken_empty_token_rejected);
    RUN_TEST(test_checktoken_null_rejected);
    RUN_TEST(test_checktoken_auth_disabled_always_false);
    RUN_TEST(test_topublicjson_zero_size_returns_zero);
    RUN_TEST(test_topublicjson_small_size_returns_zero);
    RUN_TEST(test_topublicjson_normal_size_returns_json);
    RUN_TEST(test_empty_wifi_password_preserves_current);

    // ── H7: PUT /api/device/config → 202 on wifi_password change ─
    RUN_TEST(test_put_wifi_password_change_returns_202);
    RUN_TEST(test_put_no_wifi_password_change_returns_200);
    RUN_TEST(test_put_wifi_password_202_has_reboot_required);
    RUN_TEST(test_put_no_wifi_change_200_no_reboot_required);

    // ── M8: oversized path handled gracefully at routing layer ───
    // (414 via parseRequestLine/handleClient verified by SITL)
    RUN_TEST(test_oversized_path_routerequest_returns_404);

    // ── H8: scan endpoints locked in FLIGHT ──────────────────
    RUN_TEST(test_i2c_scan_blocked_in_flight);
    RUN_TEST(test_uart_scan_blocked_in_flight);

    // ── H6: correct reason phrases in responses ───────────────
    RUN_TEST(test_status_200_reason_phrase_ok);
    RUN_TEST(test_put_device_config_202_reason_accepted);

    // ── P3-4: HTTP parser robustness (handleClient fuzz layer) ───
    RUN_TEST(test_hc_empty_request_no_response);
    RUN_TEST(test_hc_bare_crlf_no_response);
    RUN_TEST(test_hc_lf_only_line_endings_returns_200);
    RUN_TEST(test_hc_negative_content_length_returns_413);
    RUN_TEST(test_hc_oversized_content_length_returns_413);
    RUN_TEST(test_hc_truncated_body_returns_400);
    RUN_TEST(test_hc_body_without_content_type_returns_400);
    RUN_TEST(test_hc_uri_too_long_returns_414);
    RUN_TEST(test_hc_header_flood_returns_200);
    RUN_TEST(test_hc_malformed_header_no_colon_returns_200);

    return UNITY_END();
}

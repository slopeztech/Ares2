/**
 * @file  main.cpp
 * @brief Unity test runner for the api_routing test suite.
 *
 * Registers all ten routing and auth test cases and runs them via the
 * Unity C unit-test framework.
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

    return UNITY_END();
}

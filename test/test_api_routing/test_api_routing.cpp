/**
 * @file  test_api_routing.cpp
 * @brief HTTP routing integration tests for ApiServer::routeRequest().
 *
 * Tests inject hand-crafted requests by calling routeRequest() directly
 * on a SimWiFiClient — no live TCP stack, no RTOS task.  This isolates
 * the routing and auth logic from both hardware and FreeRTOS scheduling.
 *
 * Build environment: api_routing (see platformio.ini).
 * Requires: -DARES_NDEBUG (disables the task-handle assertion in handleClient).
 *
 * Test matrix:
 *   [R1] GET  /api/nonexistent          → 404 (unknown path falls through)
 *   [R2] DELETE /api/status             → 405 (only GET allowed)
 *   [R3] POST /api/config               → 405 (only GET/PUT allowed)
 *   [R4] OPTIONS /api/arm               → 204 (CORS preflight, no auth check)
 *   [A1] POST /api/arm   (auth, no token)   → 401
 *   [A2] POST /api/mode  (auth, bad token)  → 401
 *   [A3] PUT  /api/missions/test.ams (auth, no token) → 401
 *   [H1] POST /api/arm   (no auth, mission=nullptr) → 500
 *   [H2] POST /api/abort (no auth, mode=IDLE)       → 409
 *   [H3] POST /api/mode  (no auth, invalid JSON)    → 400
 */

#include <unity.h>

#include <cstring>

#include "WiFiClient.h"
#include "api/api_server.h"
#include "hal/baro/barometer_interface.h"
#include "hal/gps/gps_interface.h"
#include "hal/imu/imu_interface.h"
#include "sys/device_config/device_config.h"
#include "sys/wifi/wifi_ap.h"

// ── Null sensor stubs (inline, test-only) ────────────────────────────────────

class NullBaro final : public BarometerInterface
{
public:
    bool begin()                         override { return false; }
    BaroStatus read(BaroReading&)        override { return BaroStatus::NOT_READY; }
    void setSeaLevelPressure(float)      override {}
    const char* driverModel() const      override { return "NULL"; }
};

class NullGps final : public GpsInterface
{
public:
    bool begin()                         override { return false; }
    void update()                        override {}
    GpsStatus read(GpsReading&)          override { return GpsStatus::NOT_READY; }
    bool hasFix()              const     override { return false; }
    const char* driverModel()  const     override { return "NULL"; }
};

class NullImu final : public ImuInterface
{
public:
    bool begin()                         override { return false; }
    ImuStatus read(ImuReading&)          override { return ImuStatus::NOT_READY; }
    const char* driverModel()  const     override { return "NULL"; }
};

// ── Shared test fixtures (no auth, no optional deps) ─────────────────────────

static WifiAp      s_wifi;
static NullBaro    s_baro;
static NullGps     s_gps;
static NullImu     s_imu;
static DeviceConfig s_devCfg;  // default: auth disabled

// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
static ApiServer s_server{s_wifi, s_baro, s_gps, s_imu, s_devCfg};
// Note: s_server.begin() is intentionally NOT called — tests drive
// routeRequest() directly.  The RTOS task is never created, so the
// task-handle assertion in handleClient() is bypassed entirely.

// ── setUp / tearDown ─────────────────────────────────────────────────────────

void setUp(void)    {}
void tearDown(void) {}

// ── [R1] Unknown path → 404 ───────────────────────────────────────────────────

void test_unknown_path_returns_404(void)
{
    SimWiFiClient c{""};
    s_server.routeRequest(c, "GET", "/api/nonexistent", "", 0U, "");
    TEST_ASSERT_EQUAL_INT(404, c.statusCode());
}

// ── [R2] DELETE /api/status → 405 ────────────────────────────────────────────

void test_delete_status_returns_405(void)
{
    SimWiFiClient c{""};
    s_server.routeRequest(c, "DELETE", "/api/status", "", 0U, "");
    TEST_ASSERT_EQUAL_INT(405, c.statusCode());
}

// ── [R3] POST /api/config → 405 ──────────────────────────────────────────────

void test_post_config_returns_405(void)
{
    SimWiFiClient c{""};
    s_server.routeRequest(c, "POST", "/api/config", "", 0U, "");
    TEST_ASSERT_EQUAL_INT(405, c.statusCode());
}

// ── [R4] OPTIONS /api/arm → 204 ──────────────────────────────────────────────

void test_options_arm_returns_204(void)
{
    SimWiFiClient c{""};
    s_server.routeRequest(c, "OPTIONS", "/api/arm", "", 0U, "");
    TEST_ASSERT_EQUAL_INT(204, c.statusCode());
}

// ── [H2] POST /api/abort, mode=IDLE → 409 ────────────────────────────────────
// Tested before [H1] because [H1] mutates the shared server's mode to ERROR.

void test_abort_in_idle_returns_409(void)
{
    SimWiFiClient c{""};
    s_server.routeRequest(c, "POST", "/api/abort", "", 0U, "");
    TEST_ASSERT_EQUAL_INT(409, c.statusCode());
}

// ── [H3] POST /api/mode, invalid JSON body → 400 ─────────────────────────────

void test_mode_invalid_json_returns_400(void)
{
    SimWiFiClient c{""};
    const char* body = "not-json";
    s_server.routeRequest(c, "POST", "/api/mode", body,
                           static_cast<uint32_t>(strlen(body)), "");
    TEST_ASSERT_EQUAL_INT(400, c.statusCode());
}

// ── [H1] POST /api/arm, mission=nullptr → 500 ────────────────────────────────
// State-mutating test: sets mode to ERROR.  Run last among shared-server tests.

void test_arm_no_mission_returns_500(void)
{
    SimWiFiClient c{""};
    s_server.routeRequest(c, "POST", "/api/arm", "", 0U, "");
    TEST_ASSERT_EQUAL_INT(500, c.statusCode());
}

// ── Auth tests (own server instance per test) ─────────────────────────────────

/**
 * Build a DeviceConfig with a known token so that isAuthEnabled() returns true.
 * Uses applyJson() which is the same validation path exercised in production.
 */
static void enableAuth(DeviceConfig& cfg)
{
    static const char kPatch[] = R"({"api_token":"testtoken99"})";
    char err[64] = {};
    (void)cfg.applyJson(kPatch, static_cast<uint32_t>(strlen(kPatch)),
                        err, static_cast<uint8_t>(sizeof(err)));
}

// ── [A1] POST /api/arm, auth enabled, no token → 401 ─────────────────────────

void test_auth_arm_no_token_returns_401(void)
{
    WifiAp wifi;
    NullBaro baro; NullGps gps; NullImu imu;
    DeviceConfig devCfg;
    enableAuth(devCfg);

    ApiServer srv{wifi, baro, gps, imu, devCfg};
    SimWiFiClient c{""};
    srv.routeRequest(c, "POST", "/api/arm", "", 0U, "");  // empty token
    TEST_ASSERT_EQUAL_INT(401, c.statusCode());
}

// ── [A2] POST /api/mode, auth enabled, wrong token → 401 ─────────────────────

void test_auth_mode_wrong_token_returns_401(void)
{
    WifiAp wifi;
    NullBaro baro; NullGps gps; NullImu imu;
    DeviceConfig devCfg;
    enableAuth(devCfg);

    ApiServer srv{wifi, baro, gps, imu, devCfg};
    SimWiFiClient c{""};
    const char* body = R"({"mode":"test"})";
    srv.routeRequest(c, "POST", "/api/mode", body,
                     static_cast<uint32_t>(strlen(body)), "wrongtoken");
    TEST_ASSERT_EQUAL_INT(401, c.statusCode());
}

// ── [A3] PUT /api/missions/test.ams, auth enabled, no token → 401 ────────────

void test_auth_mission_upload_no_token_returns_401(void)
{
    WifiAp wifi;
    NullBaro baro; NullGps gps; NullImu imu;
    DeviceConfig devCfg;
    enableAuth(devCfg);

    ApiServer srv{wifi, baro, gps, imu, devCfg};
    SimWiFiClient c{""};
    srv.routeRequest(c, "PUT", "/api/missions/test.ams", "body", 4U, "");
    TEST_ASSERT_EQUAL_INT(401, c.statusCode());
}

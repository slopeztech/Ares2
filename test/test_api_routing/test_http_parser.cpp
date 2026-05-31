/**
 * @file  test_http_parser.cpp
 * @brief HTTP parser robustness tests (P3-4).
 *
 * Exercises ApiServer::handleClient() with malformed / fuzz HTTP inputs via
 * SimWiFiClient — no live TCP stack, no RTOS task.  Tests verify that
 * parseRequestLine, parseHeaders, and readRequestBody handle adversarial
 * inputs without crashing or producing undefined behaviour.
 *
 * Test matrix:
 *   [HP1] Empty request                       → no response (early return)
 *   [HP2] Bare CRLF only                      → no response (empty line)
 *   [HP3] LF-only line endings                → 200 (parser handles LF-only)
 *   [HP4] Negative Content-Length             → 413 (wraps to ULONG_MAX > limit)
 *   [HP5] Content-Length exceeds API maximum  → 413
 *   [HP6] Body shorter than Content-Length    → 400 (incomplete body)
 *   [HP7] Body present but no Content-Type    → 400 (missing Content-Type)
 *   [HP8] URI longer than HTTP_PATH_MAX       → 414 (path truncation detected)
 *   [HP9] More than HTTP_MAX_HEADERS headers  → 200 (capped gracefully, no hang)
 *   [HP10] Header line without colon          → 200 (ignored, no crash)
 */

#include <cstring>
#include <string>

#include <unity.h>

#include "WiFiClient.h"
#include "api/api_server.h"
#include "hal/baro/barometer_interface.h"
#include "hal/gps/gps_interface.h"
#include "hal/imu/imu_interface.h"
#include "sys/device_config/device_config.h"
#include "sys/wifi/wifi_ap.h"

// ── Null sensor stubs (hp_ prefix avoids ODR collision with test_api_routing) ─

namespace {

class HpNullBaro final : public BarometerInterface
{
public:
    bool begin()                     override { return false; }
    BaroStatus read(BaroReading&)    override { return BaroStatus::NOT_READY; }
    void setSeaLevelPressure(float)  override {}
    const char* driverModel() const  override { return "NULL"; }
};

class HpNullGps final : public GpsInterface
{
public:
    bool begin()                     override { return false; }
    void update()                    override {}
    GpsStatus read(GpsReading&)      override { return GpsStatus::NOT_READY; }
    bool hasFix()          const     override { return false; }
    const char* driverModel() const  override { return "NULL"; }
};

class HpNullImu final : public ImuInterface
{
public:
    bool begin()                     override { return false; }
    ImuStatus read(ImuReading&)      override { return ImuStatus::NOT_READY; }
    const char* driverModel() const  override { return "NULL"; }
};

} // namespace

// ── Shared fixture (begin() intentionally not called — handleClient is
//    exercised directly, RTOS task is never started) ─────────────────────────

static WifiAp       s_hpWifi;
static HpNullBaro   s_hpBaro;
static HpNullGps    s_hpGps;
static HpNullImu    s_hpImu;
static DeviceConfig s_hpDevCfg;

// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
static ApiServer s_hpServer{s_hpWifi, s_hpBaro, s_hpGps, s_hpImu, s_hpDevCfg};

// ── setUp / tearDown ──────────────────────────────────────────────────────────

// (setUp/tearDown are defined once in test_api_routing.cpp — Unity resolves
//  them globally across all TUs in the test binary.)

// ── [HP1] Empty request → no response ────────────────────────────────────────

void test_hc_empty_request_no_response(void)
{
    SimWiFiClient c{""};
    s_hpServer.handleClient(c);
    // parseRequestLine reads zero bytes → returns false → handleClient returns
    // immediately without writing anything to the client.
    TEST_ASSERT_EQUAL_INT(0, c.statusCode());
}

// ── [HP2] Bare CRLF only → no response ───────────────────────────────────────

void test_hc_bare_crlf_no_response(void)
{
    // "\r\n" → readBytesUntil reads "\r", strips '\r' → lineLen = 0 → early return.
    SimWiFiClient c{"\r\n"};
    s_hpServer.handleClient(c);
    TEST_ASSERT_EQUAL_INT(0, c.statusCode());
}

// ── [HP3] LF-only line endings → 200 ─────────────────────────────────────────

void test_hc_lf_only_line_endings_returns_200(void)
{
    // RFC 7230 §3.5: CRLF is canonical but bare LF should be tolerated.
    // readBytesUntil('\n') consumes until '\n'; no '\r' to strip → fine.
    SimWiFiClient c{"GET /api/status HTTP/1.1\n\n"};
    s_hpServer.handleClient(c);
    TEST_ASSERT_EQUAL_INT(200, c.statusCode());
}

// ── [HP4] Negative Content-Length → 413 ──────────────────────────────────────

void test_hc_negative_content_length_returns_413(void)
{
    // strtoul("-1") returns ULONG_MAX; cast to uint32_t → 4 294 967 295.
    // That value exceeds API_MAX_REQUEST_BODY (1 024) → 413 before body read.
    SimWiFiClient c{
        "POST /api/mode HTTP/1.1\r\n"
        "Content-Type: application/json\r\n"
        "Content-Length: -1\r\n"
        "\r\n"
    };
    s_hpServer.handleClient(c);
    TEST_ASSERT_EQUAL_INT(413, c.statusCode());
}

// ── [HP5] Content-Length exceeds API maximum → 413 ───────────────────────────

void test_hc_oversized_content_length_returns_413(void)
{
    // 9 999 > API_MAX_REQUEST_BODY (1 024) → 413 without reading any body bytes.
    SimWiFiClient c{
        "PUT /api/config HTTP/1.1\r\n"
        "Content-Type: application/json\r\n"
        "Content-Length: 9999\r\n"
        "\r\n"
    };
    s_hpServer.handleClient(c);
    TEST_ASSERT_EQUAL_INT(413, c.statusCode());
}

// ── [HP6] Body shorter than Content-Length → 400 ─────────────────────────────

void test_hc_truncated_body_returns_400(void)
{
    // Content-Length claims 20 bytes but only 3 are present in the stream.
    // readRequestBody receives fewer bytes than requested → 400.
    SimWiFiClient c{
        "PUT /api/config HTTP/1.1\r\n"
        "Content-Type: application/json\r\n"
        "Content-Length: 20\r\n"
        "\r\n"
        "abc"  // only 3 bytes
    };
    s_hpServer.handleClient(c);
    TEST_ASSERT_EQUAL_INT(400, c.statusCode());
    TEST_ASSERT_NOT_NULL(strstr(c.response().c_str(), "incomplete"));
}

// ── [HP7] Body present but no Content-Type header → 400 ──────────────────────

void test_hc_body_without_content_type_returns_400(void)
{
    // REST-2.4: POST/PUT with a body must include Content-Type: application/json.
    // Missing header → handleClient rejects before calling routeRequest.
    SimWiFiClient c{
        "POST /api/mode HTTP/1.1\r\n"
        "Content-Length: 5\r\n"
        "\r\n"
        "hello"
    };
    s_hpServer.handleClient(c);
    TEST_ASSERT_EQUAL_INT(400, c.statusCode());
    TEST_ASSERT_NOT_NULL(strstr(c.response().c_str(), "Content-Type"));
}

// ── [HP8] URI longer than HTTP_PATH_MAX → 414 ────────────────────────────────

void test_hc_uri_too_long_returns_414(void)
{
    // HTTP_PATH_MAX = 64; the path below is "/api/" (5) + 70 'x' = 75 chars.
    // parseRequestLine detects truncation → handleClient sends 414.
    SimWiFiClient c{
        "GET /api/xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx"
        " HTTP/1.1\r\n"
        "\r\n"
    };
    s_hpServer.handleClient(c);
    TEST_ASSERT_EQUAL_INT(414, c.statusCode());
}

// ── [HP9] More headers than HTTP_MAX_HEADERS → 200 ───────────────────────────

void test_hc_header_flood_returns_200(void)
{
    // HTTP_MAX_HEADERS = 16; send 20 junk headers before the blank line.
    // Parser stops reading at 16, the remaining headers are left unread in
    // the stream but do not affect routing — GET /api/status must still 200.
    std::string req = "GET /api/status HTTP/1.1\r\n";
    for (int i = 0; i < 20; ++i)
    {
        char hdr[32];
        snprintf(hdr, sizeof(hdr), "X-Fuzz-%02d: ignore\r\n", i);
        req += hdr;
    }
    req += "\r\n";  // end-of-headers marker

    SimWiFiClient c{req.c_str()};
    s_hpServer.handleClient(c);
    TEST_ASSERT_EQUAL_INT(200, c.statusCode());
}

// ── [HP10] Header line without a colon → 200 ────────────────────────────────

void test_hc_malformed_header_no_colon_returns_200(void)
{
    // A header line that contains no ':' matches no known field name prefix
    // and is silently skipped. The server must not crash or hang.
    SimWiFiClient c{
        "GET /api/status HTTP/1.1\r\n"
        "ThisHeaderHasNoColon\r\n"
        "\r\n"
    };
    s_hpServer.handleClient(c);
    TEST_ASSERT_EQUAL_INT(200, c.statusCode());
}

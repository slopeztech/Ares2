/**
 * @file  device_config_handler.cpp
 * @brief GET/PUT /api/device/config endpoint implementations.
 *
 * GET  returns the current device security configuration (api_token omitted).
 * PUT  validates all supplied fields, applies them, persists to LittleFS,
 *      and refreshes the live CORS header buffer.
 *
 * Implements ApiServer::handleDeviceConfigGet and handleDeviceConfigPut
 * (declared in api_server.h).
 *
 * Security notes:
 *   - api_token is never returned in GET responses (write-only field).
 *   - wifi_password and api_token changes take effect immediately.
 *     The WiFi AP password change requires a device reboot because the
 *     soft-AP is initialised once at startup.
 *   - All field validation follows REST-5 (validate-all before mutating).
 */

#include "api/api_server.h"
#include "api/api_common.h"
#include "debug/ares_log.h"

#include <WiFiClient.h>
#include <cinttypes>

static constexpr const char* TAG = "API.DCFG";

// ── GET /api/device/config ────────────────────────────────────

void ApiServer::handleDeviceConfigGet(WiFiClient& client)
{
    char buf[ares::API_MAX_RESPONSE_BODY] = {};
    const uint32_t len = devCfg_.toPublicJson(buf,
                            static_cast<uint32_t>(sizeof(buf)));
    sendJson(client, 200U, buf, len);
}

// ── PUT /api/device/config ────────────────────────────────────

void ApiServer::handleDeviceConfigPut(WiFiClient& client,
                                      const char* body,
                                      uint32_t    bodyLen)
{
    if (bodyLen == 0U)
    {
        sendError(client, 400, "request body required");
        LOG_W(TAG, "PUT /api/device/config 400: empty body");
        return;
    }

    // ── Snapshot current wifi_password to detect a change after apply ─────
    char oldWifiPass[ares::DEVICE_WIFI_PASS_MAX] = {};
    (void)strncpy(oldWifiPass, devCfg_.wifiPassword(),
                  sizeof(oldWifiPass) - 1U);
    oldWifiPass[sizeof(oldWifiPass) - 1U] = '\0';

    // ── Phase 1: validate + apply ──────────────────────────────
    static char errBuf[96] = {};
    if (!devCfg_.applyJson(body, bodyLen,
                           errBuf, static_cast<uint8_t>(sizeof(errBuf))))
    {
        sendError(client, 400, errBuf);
        LOG_W(TAG, "PUT /api/device/config 400: %s", errBuf);
        return;
    }

    // ── Phase 2: persist to LittleFS ──────────────────────────
    const bool saved = devCfg_.save(storage_);
    if (!saved && storage_ != nullptr)
    {
        // storage_ is set but write failed — the change is in RAM only and
        // will be lost on reboot.  Return 500 so the client knows to retry.
        sendError(client, 500, "config applied but persist failed — changes are transient");
        LOG_E(TAG, "PUT /api/device/config 500: LittleFS write error");
        return;
    }
    if (!saved)
    {
        // storage_ is null (no-storage build / test environment) — expected.
        LOG_W(TAG, "PUT /api/device/config: no storage backend, running in-memory only");
    }

    // ── Phase 3: refresh runtime state ────────────────────────
    // CORS headers must be updated so subsequent responses reflect the
    // new Access-Control-Allow-Origin value immediately.
    refreshCorsHeaders();

    // ── Phase 4: respond ──────────────────────────────────────
    // H7: When wifi_password was changed, return 202 Accepted and inject
    // "reboot_required":true into the response body so the client knows the
    // change is pending a device restart (soft-AP password takes effect on
    // next boot only).
    const bool wifiPassChanged =
        (strcmp(oldWifiPass, devCfg_.wifiPassword()) != 0);

    char resBuf[ares::API_MAX_RESPONSE_BODY] = {};
    const uint32_t resLen = devCfg_.toPublicJson(resBuf,
                                static_cast<uint32_t>(sizeof(resBuf)));

    if (wifiPassChanged)
    {
        // Inject reboot_required flag: strip trailing '}' from the config JSON,
        // append the extra field, then close.  toPublicJson always returns a
        // valid NUL-terminated JSON object so resLen >= 2 ("{}").
        char resp202[ares::API_MAX_RESPONSE_BODY] = {};
        const uint32_t baseLen = (resLen > 1U) ? resLen - 1U : 0U;
        const int n = snprintf(resp202, sizeof(resp202),
                               "%.*s,\"reboot_required\":true}",
                               static_cast<int>(baseLen), resBuf);
        const uint32_t resp202Len = (n > 0) ? static_cast<uint32_t>(n) : 0U;
        sendJson(client, 202U, resp202, resp202Len);
        LOG_I(TAG, "PUT /api/device/config 202: wifi_password changed — reboot required");
    }
    else
    {
        sendJson(client, 200U, resBuf, resLen);
        LOG_I(TAG, "PUT /api/device/config 200 (persisted=%s)",
              saved ? "yes" : "no storage");
    }
}

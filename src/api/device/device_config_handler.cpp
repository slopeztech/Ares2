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

    // ── Phase 1: validate + apply ──────────────────────────────
    char errBuf[96] = {};
    if (!devCfg_.applyJson(body, bodyLen,
                           errBuf, static_cast<uint8_t>(sizeof(errBuf))))
    {
        sendError(client, 400, errBuf);
        LOG_W(TAG, "PUT /api/device/config 400: %s", errBuf);
        return;
    }

    // ── Phase 2: persist to LittleFS ──────────────────────────
    const bool saved = devCfg_.save(storage_);
    if (!saved)
    {
        // Applied in RAM but not persisted — warn in the response.
        LOG_W(TAG, "PUT /api/device/config: applied but save failed");
    }

    // ── Phase 3: refresh runtime state ────────────────────────
    // CORS headers must be updated so subsequent responses reflect the
    // new Access-Control-Allow-Origin value immediately.
    refreshCorsHeaders();

    // ── Phase 4: respond ──────────────────────────────────────
    char resBuf[ares::API_MAX_RESPONSE_BODY] = {};
    const uint32_t resLen = devCfg_.toPublicJson(resBuf,
                                static_cast<uint32_t>(sizeof(resBuf)));
    sendJson(client, 200U, resBuf, resLen);

    LOG_I(TAG, "PUT /api/device/config 200 (persisted=%s)",
          saved ? "yes" : "no — save failed");

    if (saved)
    {
        LOG_I(TAG, "  note: wifi_password changes require a reboot");
    }
}

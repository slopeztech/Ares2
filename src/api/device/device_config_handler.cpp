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
#include <cstdio>
#include <cstring>

static constexpr const char* TAG = "API.DCFG";

namespace
{

bool persistDeviceConfig(const DeviceConfig& devCfg,
                                StorageInterface* storage)
{
    const bool saved = devCfg.save(storage);
    if (!saved && storage != nullptr)
    {
        LOG_E(TAG, "PUT /api/device/config: LittleFS write error");
        return false;
    }
    if (!saved)
    {
        LOG_W(TAG, "PUT /api/device/config: no storage backend, running in-memory only");
    }
    return true;
}

void applyRuntimeDriverSelections(const DeviceConfig& devCfg,
                                  ImuSelectionInterface* imuSelector,
                                  DriverSelectionInterface* gpsSelector,
                                  DriverSelectionInterface* baroSelector,
                                  DriverSelectionInterface* comSelector)
{
    if (imuSelector != nullptr)
    {
        (void)imuSelector->selectDriverByName(devCfg.defaultImuDriver());
    }
    if (gpsSelector != nullptr)
    {
        (void)gpsSelector->selectDriverByName(devCfg.defaultGpsDriver());
    }
    if (baroSelector != nullptr)
    {
        (void)baroSelector->selectDriverByName(devCfg.defaultBaroDriver());
    }
    if (comSelector != nullptr)
    {
        (void)comSelector->selectDriverByName(devCfg.defaultComDriver());
    }
}

uint16_t buildDeviceConfigResponse(const DeviceConfig& devCfg,
                                   bool wifiPassChanged,
                                   char* outBuf,
                                   uint32_t outBufSize,
                                   uint32_t& outLen)
{
    if (outBuf == nullptr || outBufSize == 0U)
    {
        outLen = 0U;
        return 200U;
    }

    char resBuf[ares::API_MAX_RESPONSE_BODY] = {};
    const uint32_t resLen = devCfg.toPublicJson(resBuf,
                                static_cast<uint32_t>(sizeof(resBuf)));

    if (wifiPassChanged)
    {
        const uint32_t baseLen = (resLen > 1U) ? resLen - 1U : 0U;
        const int n = snprintf(outBuf, outBufSize,
                               "%.*s,\"reboot_required\":true}",
                               static_cast<int>(baseLen), resBuf);
        if (n <= 0)
        {
            outLen = 0U;
        }
        else if (static_cast<uint32_t>(n) >= outBufSize)
        {
            outLen = outBufSize - 1U;
        }
        else
        {
            outLen = static_cast<uint32_t>(n);
        }
        return 202U;
    }

    (void)strncpy(outBuf, resBuf, outBufSize - 1U);
    outBuf[outBufSize - 1U] = '\0';
    outLen = static_cast<uint32_t>(strlen(outBuf));
    return 200U;
}

} // namespace

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
    if (!persistDeviceConfig(devCfg_, storage_))
    {
        sendError(client, 500, "config applied but persist failed — changes are transient");
        LOG_E(TAG, "PUT /api/device/config 500: LittleFS write error");
        return;
    }

    // ── Phase 3: refresh runtime state ────────────────────────
    // CORS headers must be updated so subsequent responses reflect the
    // new Access-Control-Allow-Origin value immediately.
    refreshCorsHeaders();

    applyRuntimeDriverSelections(devCfg_, imuSelector_, gpsSelector_, baroSelector_, comSelector_);

    const bool wifiPassChanged =
        (strcmp(oldWifiPass, devCfg_.wifiPassword()) != 0);

    char outBuf[ares::API_MAX_RESPONSE_BODY] = {};
    uint32_t outLen = 0U;
    const uint16_t status = buildDeviceConfigResponse(devCfg_, wifiPassChanged,
                                                      outBuf,
                                                      static_cast<uint32_t>(sizeof(outBuf)),
                                                      outLen);
    sendJson(client, status, outBuf, outLen);
    if (status == 202U)
    {
        LOG_I(TAG, "PUT /api/device/config 202: wifi_password changed — reboot required");
    }
    else
    {
        LOG_I(TAG, "PUT /api/device/config 200");
    }
}

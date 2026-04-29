/**
 * @file  config_handler.cpp
 * @brief GET/PUT /api/config endpoint implementations.
 *
 * GET  returns the current runtime configuration.
 * PUT  validates all fields first, then applies under mutex
 *      (REST-5: validate-all, apply-after).
 *
 * Implements ApiServer::handleConfigGet and handleConfigPut
 * (declared in api_server.h).
 */

#include "api/api_server.h"
#include "api/api_common.h"
#include "debug/ares_log.h"

#include <ArduinoJson.h>
#include <WiFiClient.h>
#include <cinttypes>

static constexpr const char* TAG = "API.CFG";

void ApiServer::handleConfigGet(WiFiClient& client)
{
    ScopedLock guard(cfgMtx_,
                     pdMS_TO_TICKS(ares::API_CFG_MUTEX_TIMEOUT_MS));
    if (!guard.acquired())
    {
        sendError(client, 500, "resource busy");
        LOG_E(TAG, "GET /api/config 500: mutex timeout");
        return;
    }

    JsonDocument doc;
    doc["telemetryIntervalMs"] = config_.telemetryIntervalMs;
    doc["nodeId"]              = config_.nodeId;
    doc["ledBrightness"]       = config_.ledBrightness;

    char buf[ares::API_MAX_RESPONSE_BODY] = {};
    const size_t len = serializeJson(doc, buf, sizeof(buf));
    ARES_ASSERT(len < sizeof(buf));
    sendJson(client, 200U, buf, static_cast<uint32_t>(len));
}

/**
 * @brief Parse and validate all configurable fields from a JSON document.
 *
 * Implements REST-5.3 Phase 1: validates every present field before any
 * mutation.  Sends the appropriate error response and returns false on
 * validation failure.
 *
 * @param errFn  Callable with signature (WiFiClient&, uint16_t, const char*).
 */
template<typename ErrFn>
static bool parseConfigFields(const JsonDocument& doc,
                               WiFiClient&          client,
                               uint32_t&            newInterval,
                               uint8_t&             newNodeId,
                               uint8_t&             newBright,
                               ErrFn                errFn)
{
    if (doc["telemetryIntervalMs"].is<uint32_t>())
    {
        const uint32_t val = doc["telemetryIntervalMs"].as<uint32_t>();
        if (val < ares::TELEMETRY_INTERVAL_MIN || val > ares::TELEMETRY_INTERVAL_MAX)
        {
            errFn(client, 400, "telemetryIntervalMs out of range (100-60000)");
            LOG_W(TAG, "PUT /api/config 400: interval out of range");
            return false;
        }
        newInterval = val;
    }
    else if (!doc["telemetryIntervalMs"].isNull())
    {
        errFn(client, 400, "telemetryIntervalMs must be uint32");
        LOG_W(TAG, "PUT /api/config 400: bad type");
        return false;
    }

    if (doc["nodeId"].is<uint8_t>())
    {
        const uint8_t val = doc["nodeId"].as<uint8_t>();
        if (val < ares::NODE_ID_MIN || val > ares::NODE_ID_MAX)
        {
            errFn(client, 400, "nodeId out of range (1-253)");
            LOG_W(TAG, "PUT /api/config 400: nodeId out of range");
            return false;
        }
        newNodeId = val;
    }
    else if (!doc["nodeId"].isNull())
    {
        errFn(client, 400, "nodeId must be uint8");
        LOG_W(TAG, "PUT /api/config 400: bad type");
        return false;
    }

    if (doc["ledBrightness"].is<uint8_t>())
    {
        newBright = doc["ledBrightness"].as<uint8_t>();
    }
    else if (!doc["ledBrightness"].isNull())
    {
        errFn(client, 400, "ledBrightness must be uint8");
        LOG_W(TAG, "PUT /api/config 400: bad type");
        return false;
    }

    return true;
}

void ApiServer::handleConfigPut(WiFiClient& client,
                                 const char* body, uint32_t bodyLen)
{
    // REST-6.2: flight lock
    if (isFlightLocked())
    {
        sendError(client, 409, "operation locked during flight");
        LOG_W(TAG, "PUT /api/config 409: flight locked");
        return;
    }

    // REST-4.2: parse or reject
    JsonDocument doc;
    const DeserializationError err = deserializeJson(doc, body, bodyLen);
    if (err != DeserializationError::Ok)
    {
        sendError(client, 400, "invalid JSON");
        LOG_W(TAG, "PUT /api/config 400: invalid JSON");
        return;
    }

    // REST-5.3: Phase 1 — validate all fields before apply
    uint32_t newInterval = config_.telemetryIntervalMs;
    uint8_t  newNodeId   = config_.nodeId;
    uint8_t  newBright   = config_.ledBrightness;

    if (!parseConfigFields(doc, client, newInterval, newNodeId, newBright,
                           [this](WiFiClient& c, uint16_t code, const char* msg)
                           {
                               sendError(c, code, msg);
                           }))
    {
        return;
    }

    // Phase 2: apply all under mutex (REST-8.3)
    {
        ScopedLock guard(cfgMtx_,
                         pdMS_TO_TICKS(ares::API_CFG_MUTEX_TIMEOUT_MS));
        if (!guard.acquired())
        {
            sendError(client, 500, "resource busy");
            LOG_E(TAG, "PUT /api/config 500: mutex timeout");
            return;
        }
        config_.telemetryIntervalMs = newInterval;
        config_.nodeId              = newNodeId;
        config_.ledBrightness       = newBright;
    }

    // Echo new config (REST-6.3 step 4)
    handleConfigGet(client);
    LOG_I(TAG, "PUT /api/config 200: interval=%" PRIu32 " node=%u bright=%u",
          newInterval,
          static_cast<uint32_t>(newNodeId),
          static_cast<uint32_t>(newBright));
}

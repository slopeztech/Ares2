/**
 * @file  pulse_handler.cpp
 * @brief REST handler for GET /api/pulse/status (AMS-4.17).
 *
 * Returns the continuity and fired state of both electric-pulse channels
 * (A and B).  The endpoint is read-only; firing is performed
 * exclusively by the AMS engine on state entry.
 *
 * Response schema (HTTP 200):
 * @code{.json}
 * {
 *   "ch_a": { "continuity": true,  "fired": false },
 *   "ch_b": { "continuity": true,  "fired": false }
 * }
 * @endcode
 *
 * HTTP 503 is returned when no PulseInterface was provided to ApiServer.
 */

#include "api/api_server.h"
#include "api/api_common.h"
#include "debug/ares_log.h"

#include <ArduinoJson.h>
#include <WiFiClient.h>
#include <cinttypes>

static constexpr const char* TAG = "PULSE";

// ── handlePulseStatus ────────────────────────────────────────────────────────

void ApiServer::handlePulseStatus(WiFiClient& client)
{
    if (pulse_ == nullptr)
    {
        sendError(client, 503, "pulse subsystem not available");
        LOG_W(TAG, "GET /api/pulse/status 503 (no driver)");
        return;
    }

    JsonDocument doc;

    JsonObject chA = doc["ch_a"].to<JsonObject>();
    chA["continuity"] = pulse_->readContinuity(PulseChannel::CH_A);
    chA["fired"]      = pulse_->isFired(PulseChannel::CH_A);

    JsonObject chB = doc["ch_b"].to<JsonObject>();
    chB["continuity"] = pulse_->readContinuity(PulseChannel::CH_B);
    chB["fired"]      = pulse_->isFired(PulseChannel::CH_B);

    char buf[ares::API_MAX_RESPONSE_BODY] = {};
    const uint32_t len = static_cast<uint32_t>(
        serializeJson(doc, buf, sizeof(buf)));

    sendJson(client, 200, buf, len);
    LOG_I(TAG, "GET /api/pulse/status 200");
}

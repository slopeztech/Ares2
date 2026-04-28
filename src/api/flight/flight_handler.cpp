/**
 * @file  flight_handler.cpp
 * @brief POST /api/mode, /api/arm, /api/abort endpoint implementations.
 *
 * Handles operating-mode transitions with a validated transition
 * matrix (REST-6.1), arming gate (flight mode only), and abort
 * (returns to IDLE).
 *
 * Implements ApiServer::handleMode, handleArm, handleAbort
 * (declared in api_server.h).
 */

#include "api/api_server.h"
#include "api/api_common.h"
#include "debug/ares_log.h"

#include <ArduinoJson.h>
#include <WiFiClient.h>
#include <cstring>

static constexpr const char* TAG = "API.FLT";

void ApiServer::handleMode(WiFiClient& client,
                            const char* body, uint32_t bodyLen)
{
    // REST-6.2: flight lock
    if (isFlightLocked())
    {
        sendError(client, 409, "operation locked during flight");
        LOG_W(TAG, "POST /api/mode 409: flight locked");
        return;
    }

    JsonDocument doc;
    const DeserializationError err = deserializeJson(doc, body, bodyLen);
    if (err != DeserializationError::Ok)
    {
        sendError(client, 400, "invalid JSON");
        LOG_W(TAG, "POST /api/mode 400: invalid JSON");
        return;
    }

    if (!doc["mode"].is<const char*>())
    {
        sendError(client, 400, "mode field required (string)");
        LOG_W(TAG, "POST /api/mode 400: missing mode");
        return;
    }

    const char* target = doc["mode"].as<const char*>();
    const auto current = getMode();
    const uint8_t rawCurrent = static_cast<uint8_t>(current);
    ARES_ASSERT(rawCurrent <= static_cast<uint8_t>(ares::OperatingMode::LAST));

    // REST-6.1: mode transition matrix
    if (strcmp(target, "idle") == 0)
    {
        if (current != ares::OperatingMode::TEST
            && current != ares::OperatingMode::RECOVERY
            && current != ares::OperatingMode::ERROR)
        {
            sendError(client, 409,
                      "cannot transition to idle from current mode");
            LOG_W(TAG, "POST /api/mode 409: bad transition to idle");
            return;
        }
        setMode(ares::OperatingMode::IDLE);
        armed_.store(false);
    }
    else if (strcmp(target, "test") == 0)
    {
        if (current != ares::OperatingMode::IDLE)
        {
            sendError(client, 409, "test mode requires idle");
            LOG_W(TAG, "POST /api/mode 409: bad transition to test");
            return;
        }
        setMode(ares::OperatingMode::TEST);
    }
    else if (strcmp(target, "flight") == 0)
    {
        if (current != ares::OperatingMode::IDLE
            && current != ares::OperatingMode::TEST)
        {
            sendError(client, 409,
                      "flight mode requires idle or test");
            LOG_W(TAG, "POST /api/mode 409: bad transition to flight");
            return;
        }
        armed_.store(false);
        setMode(ares::OperatingMode::FLIGHT);
    }
    else
    {
        sendError(client, 400, "unknown mode (idle|test|flight)");
        LOG_W(TAG, "POST /api/mode 400: unknown mode");
        return;
    }

    handleStatus(client);
    LOG_I(TAG, "POST /api/mode 200: -> %s", target);
}

void ApiServer::handleArm(WiFiClient& client)
{
    const uint8_t rawMode = static_cast<uint8_t>(getMode());
    ARES_ASSERT(rawMode <= static_cast<uint8_t>(ares::OperatingMode::LAST));

    if (armed_.load())
    {
        sendError(client, 409, "already armed");
        LOG_W(TAG, "POST /api/arm 409: already armed");
        return;
    }

    if (mission_ == nullptr)
    {
        setMode(ares::OperatingMode::ERROR);
        sendError(client, 500, "mission engine unavailable");
        LOG_E(TAG, "POST /api/arm 500: mission engine unavailable -> mode=ERROR");
        return;
    }

    ares::ams::EngineSnapshot before = {};
    mission_->getSnapshot(before);

    if (before.status == ares::ams::EngineStatus::RUNNING)
    {
        // AMS is already executing (e.g. checkpoint restored on boot and
        // notifyMissionResumed() wasn't reached, or a duplicate arm call).
        // Adopt the running state rather than faulting into ERROR mode.
        armed_.store(true);
        setMode(ares::OperatingMode::FLIGHT);
        LOG_I(TAG, "POST /api/arm: AMS already RUNNING — adopting running state");
        handleStatus(client);
        return;
    }

    if (before.status != ares::ams::EngineStatus::LOADED)
    {
        setMode(ares::OperatingMode::ERROR);
        sendError(client, 409, "cannot arm: no mission loaded (activate first)");
        LOG_E(TAG,
              "POST /api/arm 409: AMS status=%u state=%s (expected LOADED) -> mode=ERROR",
              static_cast<uint32_t>(before.status), before.stateName);
        return;
    }

    // Enable execution and inject LAUNCH atomically.  A single mutex
    // acquisition prevents tick() from grabbing the lock between the two
    // operations and causing a mutex timeout on the LAUNCH inject.
    if (!mission_->arm())
    {
        setMode(ares::OperatingMode::ERROR);
        sendError(client, 500, "arm failed: mutex timeout or status changed");
        LOG_E(TAG, "POST /api/arm 500: arm() failed -> mode=ERROR");
        return;
    }

    ares::ams::EngineSnapshot after = {};
    mission_->getSnapshot(after);
    if (after.status != ares::ams::EngineStatus::RUNNING)
    {
        mission_->setExecutionEnabled(false);
        setMode(ares::OperatingMode::ERROR);
        sendError(client, 409, "cannot arm: AMS entered error state");
        LOG_E(TAG,
              "POST /api/arm 409: AMS status=%u state=%s error=%s -> mode=ERROR",
              static_cast<uint32_t>(after.status), after.stateName, after.lastError);
        return;
    }

    armed_.store(true);
    setMode(ares::OperatingMode::FLIGHT);  // blue LED: AMS is now executing
    LOG_I(TAG, "POST /api/arm: armed");

    handleStatus(client);
    LOG_I(TAG, "POST /api/arm 200: armed");
}

void ApiServer::handleAbort(WiFiClient& client)
{
    const auto mode = getMode();
    const uint8_t rawMode = static_cast<uint8_t>(mode);
    ARES_ASSERT(rawMode <= static_cast<uint8_t>(ares::OperatingMode::LAST));
    if (mode != ares::OperatingMode::FLIGHT)
    {
        sendError(client, 409, "abort requires flight mode");
        LOG_W(TAG, "POST /api/abort 409: not in flight");
        return;
    }

    // Inject ABORT TC so AMS can handle it gracefully (script-defined ABORT
    // transition fires if present; otherwise the engine intercepts the pending
    // token on the next tick and calls deactivateLocked()).
    // The armed flag and mode are updated immediately on the API side.
    if (mission_ != nullptr)
    {
        const bool injected = mission_->injectTcCommand("ABORT");
        if (!injected)
        {
            // Fallback: direct deactivation if injection failed.
            mission_->setExecutionEnabled(false);
            mission_->deactivate();
            LOG_W(TAG, "POST /api/abort: ABORT injection failed, deactivated directly");
        }
        else
        {
            LOG_I(TAG, "POST /api/abort: ABORT TC injected into AMS");
        }
    }

    armed_.store(false);
    setMode(ares::OperatingMode::IDLE);
    handleStatus(client);
    LOG_I(TAG, "POST /api/abort 200: aborted");
}

/**
 * @file  status_handler.cpp
 * @brief GET /api/status endpoint implementation.
 *
 * Returns system status, sensor snapshots, and health flags.
 * Implements ApiServer::handleStatus (declared in api_server.h).
 */

#include "api/api_server.h"
#include "api/api_common.h"
#include "debug/ares_log.h"

#include <Arduino.h>
#include <ArduinoJson.h>
#include <WiFiClient.h>

static constexpr const char* TAG = "API.STS";

static void appendBaroSnapshot(BarometerInterface& baro,
                                JsonDocument& doc)
{
    BaroReading baroData;
    const BaroStatus baroSt = baro.read(baroData);
    const bool baroOk = (baroSt == BaroStatus::OK);
    doc["health"]["baro"] = baroOk;
    if (baroOk)
    {
        JsonObject obj = doc["baro"].to<JsonObject>();
        obj["pressurePa"]   = baroData.pressurePa;
        obj["temperatureC"] = baroData.temperatureC;
        obj["altitudeM"]    = baroData.altitudeM;
    }
}

static void appendGpsSnapshot(GpsInterface& gps, JsonDocument& doc)
{
    GpsReading gpsData;
    const GpsStatus gpsSt = gps.read(gpsData);
    const bool gpsOk = (gpsSt == GpsStatus::OK);
    doc["health"]["gps"] = gpsOk;
    if (gpsOk)
    {
        JsonObject obj = doc["gps"].to<JsonObject>();
        obj["fix"]        = true;
        obj["latitude"]   = gpsData.latitude;
        obj["longitude"]  = gpsData.longitude;
        obj["altitudeM"]  = gpsData.altitudeM;
        obj["speedKmh"]   = gpsData.speedKmh;
        obj["courseDeg"]  = gpsData.courseDeg;
        obj["hdop"]       = gpsData.hdop;
        obj["satellites"] = gpsData.satellites;
    }
}

static void appendImuSnapshot(ImuInterface& imu, JsonDocument& doc)
{
    ImuReading imuData;
    const ImuStatus imuSt = imu.read(imuData);
    const bool imuOk = (imuSt == ImuStatus::OK);
    doc["health"]["imu"] = imuOk;
    if (imuOk)
    {
        JsonObject obj = doc["imu"].to<JsonObject>();
        obj["accelX"] = imuData.accelX;
        obj["accelY"] = imuData.accelY;
        obj["accelZ"] = imuData.accelZ;
        obj["gyroX"]  = imuData.gyroX;
        obj["gyroY"]  = imuData.gyroY;
        obj["gyroZ"]  = imuData.gyroZ;
        obj["tempC"]  = imuData.tempC;
    }
}

void ApiServer::handleStatus(WiFiClient& client)
{
    const ares::OperatingMode mode = getMode();
    const uint8_t rawMode = static_cast<uint8_t>(mode);
    ARES_ASSERT(rawMode <= static_cast<uint8_t>(ares::OperatingMode::LAST));

    JsonDocument doc;

    doc["board"]       = ares::BOARD_NAME;
    doc["version"]     = ARES_VERSION_STRING;
    doc["uptimeMs"]    = millis();
    doc["freeHeap"]    = ESP.getFreeHeap();
    doc["mode"]        = modeToString(mode);
    doc["armed"]       = armed_.load();
    doc["wifiClients"] = wifi_.clientCount();

    doc["health"]["wifi"] = wifi_.isReady();

    appendBaroSnapshot(baro_, doc);
    appendGpsSnapshot(gps_, doc);
    appendImuSnapshot(imu_, doc);

    // Config snapshot under mutex
    RuntimeConfig cfg;
    if (getConfigCopy(cfg))
    {
        doc["telemetryIntervalMs"] = cfg.telemetryIntervalMs;
        doc["nodeId"]              = cfg.nodeId;
        doc["ledBrightness"]       = cfg.ledBrightness;
    }

    char buf[ares::API_MAX_RESPONSE_BODY] = {};
    const size_t len = serializeJson(doc, buf, sizeof(buf));
    ARES_ASSERT(len < sizeof(buf));
    sendJson(client, 200U, buf, static_cast<uint32_t>(len));
}

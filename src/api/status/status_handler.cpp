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

    // Health flags
    JsonObject health = doc["health"].to<JsonObject>();
    health["wifi"] = wifi_.isReady();

    // Barometer snapshot
    BaroReading baroData;
    const BaroStatus baroSt = baro_.read(baroData);
    const bool baroOk = (baroSt == BaroStatus::OK);
    health["baro"] = baroOk;

    if (baroOk)
    {
        JsonObject baro = doc["baro"].to<JsonObject>();
        baro["pressurePa"]   = baroData.pressurePa;
        baro["temperatureC"] = baroData.temperatureC;
        baro["altitudeM"]    = baroData.altitudeM;
    }

    // GPS snapshot
    GpsReading gpsData;
    const GpsStatus gpsSt = gps_.read(gpsData);
    const bool gpsOk = (gpsSt == GpsStatus::OK);
    health["gps"] = gpsOk;

    if (gpsOk)
    {
        JsonObject gps = doc["gps"].to<JsonObject>();
        gps["fix"]        = true;
        gps["latitude"]   = gpsData.latitude;
        gps["longitude"]  = gpsData.longitude;
        gps["altitudeM"]  = gpsData.altitudeM;
        gps["speedKmh"]   = gpsData.speedKmh;
        gps["courseDeg"]  = gpsData.courseDeg;
        gps["hdop"]       = gpsData.hdop;
        gps["satellites"] = gpsData.satellites;
    }

    // IMU snapshot
    ImuReading imuData;
    const ImuStatus imuSt = imu_.read(imuData);
    const bool imuOk = (imuSt == ImuStatus::OK);
    health["imu"] = imuOk;

    if (imuOk)
    {
        JsonObject imu = doc["imu"].to<JsonObject>();
        imu["accelX"] = imuData.accelX;
        imu["accelY"] = imuData.accelY;
        imu["accelZ"] = imuData.accelZ;
        imu["gyroX"]  = imuData.gyroX;
        imu["gyroY"]  = imuData.gyroY;
        imu["gyroZ"]  = imuData.gyroZ;
        imu["tempC"]  = imuData.tempC;
    }

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

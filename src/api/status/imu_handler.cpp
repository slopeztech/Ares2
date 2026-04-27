/**
 * @file  imu_handler.cpp
 * @brief GET /api/imu and GET /api/imu/health endpoint implementation.
 */

#include "api/api_server.h"
#include "ares_assert.h"

#include <Arduino.h>
#include <ArduinoJson.h>
#include <WiFiClient.h>

/** @brief Convert ImuStatus enum to a lowercase JSON-safe string.
 *  @param[in] st  Status value to convert.
 *  @return        Null-terminated constant string (never NULL).
 */
static const char* imuStatusToString(ImuStatus st)
{
    switch (st)
    {
        case ImuStatus::OK:
            return "ok";
        case ImuStatus::ERROR:
            return "error";
        case ImuStatus::NOT_READY:
            return "not_ready";
        default:
            return "invalid";
    }
}

void ApiServer::handleImuGet(WiFiClient& client)
{
    ImuReading imuData = {};
    const ImuStatus st = imu_.read(imuData);

    JsonDocument doc;
    doc["ok"] = (st == ImuStatus::OK);
    doc["status"] = imuStatusToString(st);
    doc["timestampMs"] = millis();

    if (st == ImuStatus::OK)
    {
        doc["accelX"] = imuData.accelX;
        doc["accelY"] = imuData.accelY;
        doc["accelZ"] = imuData.accelZ;
        doc["gyroX"]  = imuData.gyroX;
        doc["gyroY"]  = imuData.gyroY;
        doc["gyroZ"]  = imuData.gyroZ;
        doc["tempC"]  = imuData.tempC;
    }

    char buf[ares::API_MAX_RESPONSE_BODY] = {};
    const size_t len = serializeJson(doc, buf, sizeof(buf));
    ARES_ASSERT(len < sizeof(buf));
    sendJson(client, 200U, buf, static_cast<uint32_t>(len));
}

void ApiServer::handleImuHealth(WiFiClient& client)
{
    ImuReading imuData = {};
    const ImuStatus st = imu_.read(imuData);

    JsonDocument doc;
    doc["ok"] = (st == ImuStatus::OK);
    doc["status"] = imuStatusToString(st);
    doc["timestampMs"] = millis();

    char buf[ares::API_MAX_RESPONSE_BODY] = {};
    const size_t len = serializeJson(doc, buf, sizeof(buf));
    ARES_ASSERT(len < sizeof(buf));
    sendJson(client, 200U, buf, static_cast<uint32_t>(len));
}

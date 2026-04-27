/**
 * @file  bus_scan_handler.cpp
 * @brief POST /api/scans/i2c and POST /api/scans/uart endpoints.
 *
 * Performs bounded, on-demand bus diagnostics for ground debugging.
 */

#include "api/api_server.h"
#include "ares_assert.h"
#include "debug/ares_log.h"

#include <Arduino.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <WiFiClient.h>
#include <cstdio>

static constexpr const char* TAG = "API.SCAN";

namespace
{

static constexpr uint8_t I2C_ADDR_FIRST = 0x03;
static constexpr uint8_t I2C_ADDR_LAST = 0x77;
static constexpr uint8_t I2C_MAX_REPORTED = 24;

struct I2cScanResult
{
    uint8_t addresses[I2C_MAX_REPORTED] = {};
    uint8_t storedCount = 0;
    uint8_t totalCount = 0;
};

static I2cScanResult scanI2cBus(TwoWire& bus)
{
    I2cScanResult result = {};

    for (uint8_t addr = I2C_ADDR_FIRST; addr <= I2C_ADDR_LAST; addr++)
    {
        bus.beginTransmission(addr);
        const uint8_t err = bus.endTransmission();
        if (err != 0U)
        {
            continue;
        }

        result.totalCount++;
        if (result.storedCount < I2C_MAX_REPORTED)
        {
            result.addresses[result.storedCount] = addr;
            result.storedCount++;
        }
    }

    return result;
}

/** @brief Convert GpsStatus enum to a lowercase JSON-safe string.
 *  @param[in] st  Status value to convert.
 *  @return        Null-terminated constant string (never NULL).
 */
static const char* gpsStatusToString(GpsStatus st)
{
    switch (st)
    {
    case GpsStatus::OK: return "ok";
    case GpsStatus::NO_FIX: return "no_fix";
    case GpsStatus::ERROR: return "error";
    case GpsStatus::NOT_READY: return "not_ready";
    case GpsStatus::TIMEOUT: return "timeout";
    default: return "invalid";
    }
}

/** @brief Scan one I2C bus and append the result as a JSON object to @p buses.
 *  @param[in,out] buses        JSON array to append to.
 *  @param[in]     name         Human-readable bus label.
 *  @param[in]     sda          SDA GPIO number.
 *  @param[in]     scl          SCL GPIO number.
 *  @param[in]     frequencyHz  Bus clock frequency in Hz.
 *  @param[in]     bus          TwoWire instance to scan.
 */
static void appendI2cBus(JsonArray& buses,
                         const char* name,
                         uint8_t sda,
                         uint8_t scl,
                         uint32_t frequencyHz,
                         TwoWire& bus)
{
    const I2cScanResult result = scanI2cBus(bus);

    JsonObject item = buses.add<JsonObject>();
    item["name"] = name;
    item["sda"] = sda;
    item["scl"] = scl;
    item["frequencyHz"] = frequencyHz;
    item["foundCount"] = result.totalCount;
    item["reportedCount"] = result.storedCount;
    item["truncated"] = (result.totalCount > result.storedCount);

    JsonArray found = item["found"].to<JsonArray>();
    for (uint8_t i = 0; i < result.storedCount; i++)
    {
        found.add(result.addresses[i]);
    }
}

/** @brief Append UART port status as a JSON object to @p ports.
 *  @param[in,out] ports      JSON array to append to.
 *  @param[in]     name       Human-readable port label.
 *  @param[in]     uartIndex  Hardware UART index (0-based).
 *  @param[in]     baud       Configured baud rate.
 *  @param[in]     serial     Pointer to HardwareSerial instance, or NULL if unavailable.
 */
static void appendUartPort(JsonArray& ports,
                           const char* name,
                           uint8_t uartIndex,
                           uint32_t baud,
                           HardwareSerial* serial)
{
    JsonObject item = ports.add<JsonObject>();
    item["name"] = name;
    item["uart"] = uartIndex;
    item["baud"] = baud;

    if (serial == nullptr)
    {
        item["available"] = false;
        item["rxBuffered"] = 0;
        return;
    }

    const int32_t available = serial->available();
    const bool hasData = (available > 0);

    item["available"] = true;
    item["rxBuffered"] = (available > 0) ? available : 0;
    item["hasData"] = hasData;

    if (hasData)
    {
        const int32_t peekByte = serial->peek();
        char firstByteHex[5] = {};
        if (peekByte >= 0)
        {
            const int written = snprintf(firstByteHex,
                                         sizeof(firstByteHex),
                                         "%02X",
                                         static_cast<uint8_t>(peekByte));
            if (written > 0)
            {
                item["peekHex"] = firstByteHex;
            }
        }
    }
}

} // namespace

void ApiServer::handleI2cScan(WiFiClient& client)
{
    if (i2c0_ == nullptr && i2c1_ == nullptr)
    {
        sendError(client, 500, "i2c buses unavailable");
        LOG_E(TAG, "POST /api/scans/i2c 500: no buses");
        return;
    }

    JsonDocument doc;
    doc["scan"] = "i2c";
    doc["timestampMs"] = millis();

    JsonArray buses = doc["buses"].to<JsonArray>();

    if (i2c0_ != nullptr)
    {
        appendI2cBus(buses,
                     "i2c0",
                     ares::PIN_I2C_SDA,
                     ares::PIN_I2C_SCL,
                     ares::I2C_FREQ,
                     *i2c0_);
    }

    if (i2c1_ != nullptr)
    {
        appendI2cBus(buses,
                     "i2c1",
                     ares::PIN_IMU_SDA,
                     ares::PIN_IMU_SCL,
                     ares::I2C_FREQ_IMU,
                     *i2c1_);
    }

    char buf[ares::API_MAX_RESPONSE_BODY] = {};
    const size_t len = serializeJson(doc, buf, sizeof(buf));
    ARES_ASSERT(len < sizeof(buf));
    sendJson(client, 200U, buf, static_cast<uint32_t>(len));

    LOG_I(TAG, "POST /api/scans/i2c 200");
}

void ApiServer::handleUartScan(WiFiClient& client)
{
    if (gpsUart_ == nullptr && loraUart_ == nullptr)
    {
        sendError(client, 500, "uart ports unavailable");
        LOG_E(TAG, "POST /api/scans/uart 500: no ports");
        return;
    }

    JsonDocument doc;
    doc["scan"] = "uart";
    doc["timestampMs"] = millis();

    JsonArray ports = doc["ports"].to<JsonArray>();

    appendUartPort(ports,
                   "gps",
                   ares::GPS_UART_PORT,
                   ares::GPS_BAUD,
                   gpsUart_);

    appendUartPort(ports,
                   "lora",
                   ares::LORA_UART_PORT,
                   ares::LORA_UART_BAUD,
                   loraUart_);

    GpsReading gpsReading = {};
    const GpsStatus gpsSt = gps_.read(gpsReading);
    doc["gpsDriver"] = gps_.driverModel();
    doc["gpsStatus"] = gpsStatusToString(gpsSt);

    if (radio_ != nullptr)
    {
        doc["radioDriver"] = radio_->driverModel();
        doc["radioReady"] = radio_->ready();
    }

    char buf[ares::API_MAX_RESPONSE_BODY] = {};
    const size_t len = serializeJson(doc, buf, sizeof(buf));
    ARES_ASSERT(len < sizeof(buf));
    sendJson(client, 200U, buf, static_cast<uint32_t>(len));

    LOG_I(TAG, "POST /api/scans/uart 200");
}

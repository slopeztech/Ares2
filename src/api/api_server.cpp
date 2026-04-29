/**
 * @file  api_server.cpp
 * @brief HTTP REST API server core — RTOS task, HTTP parsing,
 *        routing, and response helpers.
 *
 * Route handler implementations live in subdirectories:
 *   status/   — GET /api/status
 *   config/   — GET/PUT /api/config
 *   flight/   — POST /api/mode, /api/arm, /api/abort
 *   storage/  — GET/DELETE /api/logs[/:id]
 */

#include "api/api_server.h"
#include "api/api_common.h"
#include "ares_assert.h"
#include "debug/ares_log.h"
#include "sys/led/status_led.h"

#include <Arduino.h>
#include <ArduinoJson.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiServer.h>
#include <cstring>
#include <cinttypes>
#include <esp_task_wdt.h>
#include <freertos/task.h>

// ── Log tag ─────────────────────────────────────────────────
static constexpr const char* TAG = "API";
static constexpr uint32_t API_STACK_MONITOR_INTERVAL_MS = 5000;
static constexpr uint32_t API_STACK_WARN_BYTES = 1024;

// ── Static WiFiServer instance (one per firmware) ───────────
static WiFiServer httpServer(ares::WIFI_API_PORT);

// Mission file uploads exceed API_MAX_REQUEST_BODY; keep in static storage
// so the body does not live on the API-task stack.  The API server is
// single-threaded (one request at a time), so no mutex is needed here.
static char g_missionUploadBuf[ares::AMS_MAX_SCRIPT_BYTES + 1U] = {};

static ares::OperatingMode decodeOperatingMode(uint8_t rawMode)
{
    ares::OperatingMode mode = ares::OperatingMode::ERROR;

    if (rawMode <= static_cast<uint8_t>(ares::OperatingMode::LAST))
    {
        mode = static_cast<ares::OperatingMode>(rawMode);
    }

    return mode;
}

// ── Constructor ─────────────────────────────────────────────

ApiServer::ApiServer(WifiAp& wifi, BarometerInterface& baro,
                     GpsInterface& gps, ImuInterface& imu,
                     StorageInterface* storage,
                     ares::ams::MissionScriptEngine* mission,
                     StatusLed* statusLed,
                     TwoWire* i2c0,
                     TwoWire* i2c1,
                     HardwareSerial* gpsUart,
                     HardwareSerial* loraUart,
                     RadioInterface* radio)
        : wifi_(wifi), baro_(baro), gps_(gps), imu_(imu),
          storage_(storage), mission_(mission), statusLed_(statusLed),
          i2c0_(i2c0), i2c1_(i2c1), gpsUart_(gpsUart),
          loraUart_(loraUart), radio_(radio)
{
}

// ── Public API ──────────────────────────────────────────────

bool ApiServer::begin()
{
    // Create config mutex with priority inheritance (RTOS-4.1)
    cfgMtx_ = xSemaphoreCreateMutexStatic(&cfgMtxBuf_);
    ARES_ASSERT(cfgMtx_ != nullptr);

    httpServer.begin();
    httpServer.setNoDelay(true);

    TaskHandle_t handle = xTaskCreateStaticPinnedToCore(
        taskFn,
        "api",                              // REST-13.1
        sizeof(stack_),                     // RTOS-7.2: bytes
        this,
        ares::TASK_PRIORITY_API,
        stack_,
        &tcb_,
        tskNO_AFFINITY);                   // RTOS-13

    ARES_ASSERT(handle != nullptr);

    LOG_I(TAG, "task started (stack=%u pri=%u port=%u)",
          static_cast<uint32_t>(sizeof(stack_)),
          static_cast<uint32_t>(ares::TASK_PRIORITY_API),
          static_cast<uint32_t>(ares::WIFI_API_PORT));

    return handle != nullptr;
}

void ApiServer::setMode(ares::OperatingMode mode)
{
    mode_.store(static_cast<uint8_t>(mode));
    if (statusLed_ != nullptr)
    {
        statusLed_->setMode(mode);
    }
}

ares::OperatingMode ApiServer::getMode() const
{
    return decodeOperatingMode(mode_.load());
}

void ApiServer::notifyMissionComplete()
{
    armed_.store(false);
    setMode(ares::OperatingMode::IDLE);
}

void ApiServer::notifyMissionResumed()
{
    armed_.store(true);
    setMode(ares::OperatingMode::FLIGHT);
}

const RuntimeConfig& ApiServer::config() const
{
    return config_;
}

bool ApiServer::getConfigCopy(RuntimeConfig& out) const
{
    // const_cast needed: xSemaphoreTake is non-const but our
    // mutex handle is conceptually shared-read-safe.
    ScopedLock guard(cfgMtx_,
                     pdMS_TO_TICKS(ares::API_CFG_MUTEX_TIMEOUT_MS));
    if (!guard.acquired())
    {
        return false;
    }
    out = config_;
    return true;
}

// ── RTOS task ───────────────────────────────────────────────

void ApiServer::taskFn(void* param)
{
    auto* self = static_cast<ApiServer*>(param);
    ARES_ASSERT(self != nullptr);
    self->run();
}

void ApiServer::run()
{
    const esp_err_t wdtAddErr = esp_task_wdt_add(nullptr);
    if (wdtAddErr != ESP_OK)
    {
        LOG_E(TAG, "esp_task_wdt_add failed: err=%d", static_cast<int>(wdtAddErr));
    }
    uint32_t lastStackLogMs = 0;

    while (true)  // RTOS task loop — blocks on vTaskDelay (RTOS-1.3)
    {
        WiFiClient client = httpServer.accept();
        if (client.connected())
        {
            client.setTimeout(ares::WIFI_CONN_TIMEOUT_MS);  // REST-10.2
            handleClient(client);
            client.stop();  // REST-10.3: close after response
        }

        const uint32_t nowMs = millis();
        if ((nowMs - lastStackLogMs) >= API_STACK_MONITOR_INTERVAL_MS)
        {
            const UBaseType_t minFreeWords = uxTaskGetStackHighWaterMark(nullptr);
            const uint32_t minFreeBytes = static_cast<uint32_t>(minFreeWords)
                                       * static_cast<uint32_t>(sizeof(StackType_t));
            if (minFreeBytes < API_STACK_WARN_BYTES)
            {
                LOG_W(TAG, "stack low-water warning: free=%u B", minFreeBytes);
            }
            else
            {
                LOG_D(TAG, "stack low-water: free=%u B", minFreeBytes);
            }
            lastStackLogMs = nowMs;
        }

        const esp_err_t wdtResetErr = esp_task_wdt_reset();
        if (wdtResetErr != ESP_OK)
        {
            LOG_E(TAG, "esp_task_wdt_reset failed: err=%d",
                  static_cast<int>(wdtResetErr));
        }

        vTaskDelay(pdMS_TO_TICKS(ares::API_RATE_MS));
    }
}

// ── HTTP parsing helpers (PO10-4.1 decomposition) ───────────

/**
 * Read and parse the HTTP request line (e.g. "GET /api/status HTTP/1.1").
 * Exits early on empty request or timeout (PO10-2, REST-10.2).
 * @return true if a non-empty request line was successfully parsed.
 */
static bool parseRequestLine(WiFiClient& client,
                              char* method, char* path)
{
    char line[HTTP_LINE_MAX] = {};
    size_t lineLen = client.readBytesUntil('\n', line, HTTP_LINE_MAX - 1U);
    if (lineLen > 0U && line[lineLen - 1U] == '\r')
    {
        lineLen--;
    }
    line[lineLen] = '\0';

    if (lineLen == 0)
    {
        return false;
    }

    // Extract method (REST-11.5: manual parse, no sscanf)
    uint16_t idx = 0;
    uint8_t mIdx = 0;
    while (idx < lineLen && line[idx] != ' '
           && mIdx < (HTTP_METHOD_MAX - 1U))
    {
        method[mIdx] = line[idx];
        mIdx++;
        idx++;
    }
    method[mIdx] = '\0';

    // Skip space
    if (idx < lineLen) { idx++; }

    // Extract path
    uint8_t pIdx = 0;
    while (idx < lineLen && line[idx] != ' ' && line[idx] != '?'
           && pIdx < (HTTP_PATH_MAX - 1U))
    {
        path[pIdx] = line[idx];
        pIdx++;
        idx++;
    }
    path[pIdx] = '\0';

    return true;
}

/**
 * Read HTTP headers, extracting Content-Length and Content-Type.
 * Stops at the first empty line (end of headers) or when
 * HTTP_MAX_HEADERS headers have been read (REST-3.1, PO10-2).
 */
static void parseHeaders(WiFiClient& client,
                          uint32_t& contentLength,
                          bool& hasJsonContentType)
{
    contentLength = 0;
    hasJsonContentType = false;
    uint8_t headerCount = 0;

    while (headerCount < HTTP_MAX_HEADERS)  // PO10-2: bounded
    {
        char hdr[HTTP_HEADER_MAX] = {};
        size_t hLen = client.readBytesUntil('\n', hdr, HTTP_HEADER_MAX - 1U);
        if (hLen > 0U && hdr[hLen - 1U] == '\r')
        {
            hLen--;
        }
        hdr[hLen] = '\0';

        if (hLen == 0U) { break; }  // Empty line, close, or timeout

        // Content-Length (case-insensitive prefix match)
        if (strncasecmp(hdr, "Content-Length:", 15) == 0)
        {
            const char* val = hdr + 15;
            uint8_t skip = 0;
            while (*val == ' ' && skip < 4U) { val++; skip++; }
            contentLength = static_cast<uint32_t>(
                strtoul(val, nullptr, 10));
        }
        // REST-2.4: Content-Type
        else if (strncasecmp(hdr, "Content-Type:", 13) == 0)
        {
            const char* val = hdr + 13;
            uint8_t skip = 0;
            while (*val == ' ' && skip < 4U) { val++; skip++; }
            hasJsonContentType =
                (strncasecmp(val, "application/json", 16) == 0);
        }

        headerCount++;
    }
}

/**
 * Read the request body into a caller-provided buffer.
 * @return true if the body was read within the timeout,
 *         false on timeout (PO10-2).
 */
static bool readRequestBody(WiFiClient& client,
                             char* body, uint32_t maxLen,
                             uint32_t contentLength,
                             uint32_t& bodyLen)
{
    bodyLen = 0;
    const uint32_t toRead =
        (contentLength < maxLen) ? contentLength : maxLen;
    bodyLen = static_cast<uint32_t>(client.readBytes(body, toRead));
    body[bodyLen] = '\0';
    return bodyLen == toRead;
}

static bool allowNonJsonBody(const char* method, const char* path)
{
    return (strcmp(method, "PUT") == 0
            && strncmp(path, "/api/missions/", 14) == 0);
}

// ── HTTP request handling ───────────────────────────────────

void ApiServer::handleClient(WiFiClient& client)
{
    // Phase 1: Parse request line (PO10-4.1)
    char method[HTTP_METHOD_MAX] = {};
    char path[HTTP_PATH_MAX]     = {};

    if (!parseRequestLine(client, method, path))
    {
        return;  // Empty or timed-out request
    }

    // Phase 2: Parse headers
    uint32_t contentLength = 0;
    bool hasJsonContent = false;
    parseHeaders(client, contentLength, hasJsonContent);

    // Phase 3: Read body if present
    //
    // Mission file uploads (PUT /api/missions/*) are allowed up to
    // AMS_MAX_SCRIPT_BYTES and use a static buffer to avoid stack pressure.
    // All other endpoints are bounded by API_MAX_REQUEST_BODY.
    const bool isMissionUpload = (strcmp(method, "PUT") == 0
                                  && strncmp(path, "/api/missions/", 14) == 0);
    const uint32_t bodyLimit = isMissionUpload
                               ? static_cast<uint32_t>(ares::AMS_MAX_SCRIPT_BYTES)
                               : static_cast<uint32_t>(ares::API_MAX_REQUEST_BODY);

    char  smallBody[ares::API_MAX_REQUEST_BODY + 1U] = {};
    char* body = isMissionUpload ? g_missionUploadBuf : smallBody;
    uint32_t bodyLen = 0;

    if (isMissionUpload)
    {
        memset(g_missionUploadBuf, 0, sizeof(g_missionUploadBuf));
    }

    if (contentLength > 0)
    {
        // REST-3.1: check size before reading
        if (contentLength > bodyLimit)
        {
            sendError(client, 413, "payload too large");
            LOG_W(TAG, "%s %s 413: payload too large", method, path);
            return;
        }
        if (!readRequestBody(client, body, bodyLimit, contentLength, bodyLen))
        {
            LOG_W(TAG, "%s %s: body read timeout", method, path);
            return;
        }
    }

    // REST-2.4: PUT/POST with body must send Content-Type: application/json
    if (bodyLen > 0 && !hasJsonContent
        && !allowNonJsonBody(method, path))
    {
        sendError(client, 400,
                  "Content-Type must be application/json");
        LOG_W(TAG, "%s %s 400: bad content-type", method, path);
        return;
    }

    // Phase 4: Route request (REST-11.1: whitelist)
    routeRequest(client, method, path, body, bodyLen);
}

void ApiServer::routeRequest(WiFiClient& client,
                             const char* method,
                             const char* path,
                             const char* body,
                             uint32_t    bodyLen)
{
    if (strcmp(method, "OPTIONS") == 0)
    {
        sendNoContent(client, 204);
        LOG_D(TAG, "OPTIONS %s 204", path);
        return;
    }

    if (routeStatusAndConfigRequest(client, method, path, body, bodyLen)) { return; }
    if (routeFlightRequest(client, method, path, body, bodyLen)) { return; }

    if (strncmp(path, "/api/logs", 9) == 0)
    {
        routeLogRequest(client, method, path + 9);
        return;
    }

    if (routeMissionTopLevelRequest(client, method, path, body, bodyLen)) { return; }

    sendError(client, 404, "not found");
    LOG_W(TAG, "%s %s 404", method, path);
}

bool ApiServer::routeStatusAndConfigRequest(WiFiClient& client,
                                            const char* method,
                                            const char* path,
                                            const char* body,
                                            uint32_t    bodyLen)
{
    if (strcmp(path, "/api/status") == 0)
    {
        if (strcmp(method, "GET") != 0)
        {
            sendError(client, 405, "method not allowed");
            LOG_W(TAG, "%s %s 405", method, path);
            return true;
        }
        handleStatus(client);
        LOG_D(TAG, "GET /api/status 200");
        return true;
    }
    if (strcmp(path, "/api/imu") == 0)
    {
        if (strcmp(method, "GET") != 0)
        {
            sendError(client, 405, "method not allowed");
            LOG_W(TAG, "%s %s 405", method, path);
            return true;
        }
        handleImuGet(client);
        LOG_D(TAG, "GET /api/imu 200");
        return true;
    }
    if (strcmp(path, "/api/imu/health") == 0)
    {
        if (strcmp(method, "GET") != 0)
        {
            sendError(client, 405, "method not allowed");
            LOG_W(TAG, "%s %s 405", method, path);
            return true;
        }
        handleImuHealth(client);
        LOG_D(TAG, "GET /api/imu/health 200");
        return true;
    }
    if (strcmp(path, "/api/config") == 0)
    {
        if (strcmp(method, "GET") == 0)
        {
            handleConfigGet(client);
            LOG_D(TAG, "GET /api/config 200");
        }
        else if (strcmp(method, "PUT") == 0)
        {
            handleConfigPut(client, body, bodyLen);
        }
        else
        {
            sendError(client, 405, "method not allowed");
            LOG_W(TAG, "%s %s 405", method, path);
        }
        return true;
    }
    return false;
}

bool ApiServer::routeFlightRequest(WiFiClient& client,
                                   const char* method,
                                   const char* path,
                                   const char* body,
                                   uint32_t    bodyLen)
{
    if (strcmp(path, "/api/mode") == 0)
    {
        if (strcmp(method, "POST") != 0)
        {
            sendError(client, 405, "method not allowed");
            LOG_W(TAG, "%s %s 405", method, path);
            return true;
        }
        handleMode(client, body, bodyLen);
        return true;
    }
    if (strcmp(path, "/api/arm") == 0)
    {
        if (strcmp(method, "POST") != 0)
        {
            sendError(client, 405, "method not allowed");
            LOG_W(TAG, "%s %s 405", method, path);
            return true;
        }
        handleArm(client);
        return true;
    }
    if (strcmp(path, "/api/abort") == 0)
    {
        if (strcmp(method, "POST") != 0)
        {
            sendError(client, 405, "method not allowed");
            LOG_W(TAG, "%s %s 405", method, path);
            return true;
        }
        handleAbort(client);
        return true;
    }
    if (strcmp(path, "/api/scans/i2c") == 0)
    {
        if (strcmp(method, "POST") != 0)
        {
            sendError(client, 405, "method not allowed");
            LOG_W(TAG, "%s %s 405", method, path);
            return true;
        }
        handleI2cScan(client);
        return true;
    }
    if (strcmp(path, "/api/scans/uart") == 0)
    {
        if (strcmp(method, "POST") != 0)
        {
            sendError(client, 405, "method not allowed");
            LOG_W(TAG, "%s %s 405", method, path);
            return true;
        }
        handleUartScan(client);
        return true;
    }
    return false;
}

bool ApiServer::routeMissionTopLevelRequest(WiFiClient& client,
                                            const char* method,
                                            const char* path,
                                            const char* body,
                                            uint32_t    bodyLen)
{
    if (strcmp(path, "/api/storage/health") == 0)
    {
        if (strcmp(method, "GET") != 0)
        {
            sendError(client, 405, "method not allowed");
            LOG_W(TAG, "%s %s 405", method, path);
            return true;
        }
        handleStorageHealth(client);
        return true;
    }
    if (strcmp(path, "/api/mission") == 0 || strcmp(path, "/api/missions/active") == 0)
    {
        if (strcmp(method, "GET") != 0)
        {
            sendError(client, 405, "method not allowed");
            LOG_W(TAG, "%s %s 405", method, path);
            return true;
        }
        handleMissionStatus(client);
        return true;
    }
    if (strncmp(path, "/api/missions", 13) == 0)
    {
        routeMissionRequest(client, method, path + 13, body, bodyLen);
        return true;
    }
    if (strcmp(path, "/api/mission/activate") == 0)
    {
        if (strcmp(method, "POST") != 0)
        {
            sendError(client, 405, "method not allowed");
            LOG_W(TAG, "%s %s 405", method, path);
            return true;
        }
        handleMissionActivate(client, body, bodyLen);
        return true;
    }
    if (strcmp(path, "/api/mission/deactivate") == 0)
    {
        if (strcmp(method, "POST") != 0)
        {
            sendError(client, 405, "method not allowed");
            LOG_W(TAG, "%s %s 405", method, path);
            return true;
        }
        handleMissionDeactivate(client);
        return true;
    }
    if (strcmp(path, "/api/mission/command") == 0)
    {
        if (strcmp(method, "POST") != 0)
        {
            sendError(client, 405, "method not allowed");
            LOG_W(TAG, "%s %s 405", method, path);
            return true;
        }
        handleMissionCommand(client, body, bodyLen);
        return true;
    }
    return false;
}

void ApiServer::routeLogRequest(WiFiClient& client,
                                const char* method,
                                const char* sub)
{
    if (sub[0] == '\0')
    {
        if (strcmp(method, "GET") == 0)
        {
            handleLogsList(client);
        }
        else if (strcmp(method, "DELETE") == 0)
        {
            handleLogDeleteAll(client);
        }
        else
        {
            sendError(client, 405, "method not allowed");
            LOG_W(TAG, "%s /api/logs 405", method);
        }
        return;
    }

    if (sub[0] != '/')
    {
        sendError(client, 404, "not found");
        LOG_W(TAG, "%s /api/logs%s 404", method, sub);
        return;
    }

    const char* filename = sub + 1;
    if (filename[0] == '\0')
    {
        sendError(client, 400, "missing filename");
        LOG_W(TAG, "%s /api/logs/ 400: missing filename", method);
        return;
    }

    if (strcmp(method, "GET") == 0)
    {
        handleLogDownload(client, filename);
    }
    else if (strcmp(method, "DELETE") == 0)
    {
        handleLogDelete(client, filename);
    }
    else
    {
        sendError(client, 405, "method not allowed");
        LOG_W(TAG, "%s /api/logs/%s 405", method, filename);
    }
}

void ApiServer::routeMissionRequest(WiFiClient& client,
                                     const char* method,
                                     const char* sub,
                                     const char* body,
                                     uint32_t bodyLen)
{
    if (sub[0] == '\0')
    {
        if (strcmp(method, "GET") == 0)
        {
            handleMissionList(client);
        }
        else
        {
            sendError(client, 405, "method not allowed");
            LOG_W(TAG, "%s /api/missions 405", method);
        }
        return;
    }

    if (sub[0] != '/')
    {
        sendError(client, 404, "not found");
        LOG_W(TAG, "%s /api/missions%s 404", method, sub);
        return;
    }

    const char* filename = sub + 1;
    if (filename[0] == '\0')
    {
        sendError(client, 400, "missing filename");
        LOG_W(TAG, "%s /api/missions/ 400: missing filename", method);
        return;
    }

    if (strcmp(method, "GET") == 0)
    {
        handleMissionDownload(client, filename);
    }
    else if (strcmp(method, "PUT") == 0)
    {
        handleMissionUpload(client, filename, body, bodyLen);
    }
    else if (strcmp(method, "DELETE") == 0)
    {
        handleMissionDelete(client, filename);
    }
    else
    {
        sendError(client, 405, "method not allowed");
        LOG_W(TAG, "%s /api/missions/%s 405", method, filename);
    }
}

// ── Response helpers ────────────────────────────────────────

void ApiServer::sendJson(WiFiClient& client, uint16_t code,
                          const char* json, uint32_t len)
{
    client.printf("HTTP/1.1 %u OK\r\n", code);
    client.print("Content-Type: application/json\r\n");
    client.printf("Content-Length: %" PRIu32 "\r\n", len);
    client.print(CORS_HEADERS);
    client.print("Connection: close\r\n");
    client.print("\r\n");
    client.write(json, len);
}

void ApiServer::sendError(WiFiClient& client, uint16_t code,
                           const char* message)
{
    // REST-2.2: error envelope
    JsonDocument doc;
    doc["error"] = message;

    char buf[256] = {};
    const size_t len = serializeJson(doc, buf, sizeof(buf));
    ARES_ASSERT(len < sizeof(buf));

    const char* reason = "Error";
    switch (code)
    {
    case 400: reason = "Bad Request";        break;
    case 404: reason = "Not Found";          break;
    case 405: reason = "Method Not Allowed"; break;
    case 409: reason = "Conflict";           break;
    case 413: reason = "Payload Too Large";  break;
    case 500: reason = "Internal Server Error"; break;
    default:  break;  // CERT-6.3
    }

    client.printf("HTTP/1.1 %u %s\r\n", code, reason);
    client.print("Content-Type: application/json\r\n");
    client.printf("Content-Length: %" PRIu32 "\r\n", static_cast<uint32_t>(len));
    client.print(CORS_HEADERS);
    client.print("Connection: close\r\n");
    client.print("\r\n");
    client.write(buf, len);
}

void ApiServer::sendNoContent(WiFiClient& client, uint16_t code)
{
    client.printf("HTTP/1.1 %u No Content\r\n", code);
    client.print(CORS_HEADERS);
    client.print("Connection: close\r\n");
    client.print("\r\n");
}

// ── Helpers ─────────────────────────────────────────────────

bool ApiServer::isFlightLocked() const
{
    const auto mode = getMode();
    return (mode == ares::OperatingMode::FLIGHT
            || mode == ares::OperatingMode::RECOVERY);
}

const char* ApiServer::modeToString(ares::OperatingMode mode)
{
    switch (mode)
    {
    case ares::OperatingMode::IDLE:     return "IDLE";
    case ares::OperatingMode::TEST:     return "TEST";
    case ares::OperatingMode::FLIGHT:   return "FLIGHT";
    case ares::OperatingMode::RECOVERY: return "RECOVERY";
    case ares::OperatingMode::ERROR:    return "ERROR";
    default:                            return "UNKNOWN";  // CERT-6.3
    }
}

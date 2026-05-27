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
#include "api/http_parse_pure.h"
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

// ── CORS header buffer (updated from DeviceConfig) ───────────
// Initialised with the open-mode default; refreshed by ApiServer::refreshCorsHeaders()
// at begin() and after every successful PUT /api/device/config.
// Written exclusively from setup() (before tasks start) and the API task
// (single-threaded) — no mutex required.
static char s_corsHeadersBuf[256] =
    "Access-Control-Allow-Origin: *\r\n"
    "Access-Control-Allow-Methods: GET, PUT, POST, DELETE, OPTIONS\r\n"
    "Access-Control-Allow-Headers: Content-Type, X-ARES-Token\r\n";

const char* getCorsHeaders() noexcept
{
    return s_corsHeadersBuf;
}

// ── Mission upload buffer ──────────────────────────
/**
 * @brief  Mission-file upload scratchpad.
 *
 * OWNERSHIP INVARIANT: the API task is the sole accessor of this buffer.
 * Any concurrent access is a data race -- this buffer has no mutex.
 *
 * Rationale: AMS script uploads exceed API_MAX_REQUEST_BODY (~512 B).
 * Placing the ~4 KB buffer in BSS avoids stack exhaustion (PO10-3) while
 * keeping heap usage at zero.
 *
 * Enforcement: get() asserts xTaskGetCurrentTaskHandle() == expected at
 * every call site.  If the assertion fires, a refactor has routed the
 * mission-upload handler through a different task -- add a mutex or
 * redesign the ownership model before removing this assert.
 */
struct MissionUploadBuf
{
    MissionUploadBuf()                                   = delete;
    ~MissionUploadBuf()                                  = delete;
    MissionUploadBuf(const MissionUploadBuf&)            = delete;
    MissionUploadBuf& operator=(const MissionUploadBuf&) = delete;
    MissionUploadBuf(MissionUploadBuf&&)                 = delete;
    MissionUploadBuf& operator=(MissionUploadBuf&&)      = delete;

    static constexpr size_t SIZE = ares::AMS_MAX_SCRIPT_BYTES + 1U;

    /**
     * @brief  Return the upload buffer after asserting task ownership.
     * @param  expected  The only TaskHandle allowed to access this buffer.
     * @return Pointer to the static buffer (SIZE bytes, never null).
     * @pre    xTaskGetCurrentTaskHandle() == expected; aborts otherwise.
     */
    static char* get(TaskHandle_t expected)
    {
        ARES_ASSERT(xTaskGetCurrentTaskHandle() == expected);
        return buf_;
    }

private:
    static char buf_[SIZE];
};
char MissionUploadBuf::buf_[MissionUploadBuf::SIZE] = {};

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
                     DeviceConfig& devCfg,
                     StorageInterface* storage,
                     ares::ams::MissionScriptEngine* mission,
                     StatusLed* statusLed,
                     TwoWire* i2c0,
                     TwoWire* i2c1,
                     HardwareSerial* gpsUart,
                     HardwareSerial* loraUart,
                     RadioInterface* radio,
                     PulseInterface* pulse,
                     ares::RadioDispatcher* dispatcher)
        : wifi_(wifi), baro_(baro), gps_(gps), imu_(imu),
          devCfg_(devCfg),
          storage_(storage), mission_(mission), statusLed_(statusLed),
          i2c0_(i2c0), i2c1_(i2c1), gpsUart_(gpsUart),
          loraUart_(loraUart), radio_(radio), pulse_(pulse),
          dispatcher_(dispatcher)
{
}

// ── Public API ──────────────────────────────────────────────

void ApiServer::refreshCorsHeaders()
{
    devCfg_.buildCorsHeader(s_corsHeadersBuf, sizeof(s_corsHeadersBuf));
}

bool ApiServer::begin()
{
    // Populate CORS header buffer from the loaded DeviceConfig before
    // the task starts accepting connections.
    refreshCorsHeaders();

    // Create config mutex with priority inheritance (RTOS-4.1)
    cfgMtx_ = xSemaphoreCreateMutexStatic(&cfgMtxBuf_);
    ARES_ASSERT(cfgMtx_ != nullptr);

    httpServer.begin();
    httpServer.setNoDelay(true);

    taskHandle_ = xTaskCreateStaticPinnedToCore(
        taskFn,
        "api",                              // REST-13.1
        sizeof(stack_),                     // RTOS-7.2: bytes
        this,
        ares::TASK_PRIORITY_API,
        stack_,
        &tcb_,
        tskNO_AFFINITY);                   // RTOS-13

    ARES_ASSERT(taskHandle_ != nullptr);

    LOG_I(TAG, "task started (stack=%u pri=%u port=%u)",
          static_cast<uint32_t>(sizeof(stack_)),
          static_cast<uint32_t>(ares::TASK_PRIORITY_API),
          static_cast<uint32_t>(ares::WIFI_API_PORT));

    return taskHandle_ != nullptr;
}

void ApiServer::setMode(ares::OperatingMode mode)
{
    mode_.store(static_cast<uint8_t>(mode));
    if (statusLed_ != nullptr)
    {
        statusLed_->setMode(mode);
    }

    // C5: Disable the WiFi AP on entering FLIGHT to reduce RF interference
    // and close the REST API attack surface during the ascent phase.
    // Re-enable when returning to any ground-operations mode (IDLE/TEST/ERROR).
    if constexpr (ares::WIFI_DISABLE_IN_FLIGHT)
    {
        if (mode == ares::OperatingMode::FLIGHT)
        {
            wifi_.end();
        }
        else if (!wifi_.isReady())
        {
            // Returning from FLIGHT — restart the AP so the operator regains
            // REST access.  Failure is non-fatal: the device keeps operating;
            // the operator must power-cycle to recover WiFi access.
            if (!wifi_.begin(devCfg_))
            {
                LOG_W(TAG, "WiFi AP restart after flight failed");
            }
        }
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
 * @param[out] pathTruncated  Set to true when the URI was longer than
 *                            HTTP_PATH_MAX-1 bytes and was silently cut.
 *                            Caller must send 414 URI Too Long in that case.
 * @return true if a non-empty request line was successfully parsed.
 */
static bool parseRequestLine(WiFiClient& client,
                              char* method, char* path,
                              bool& pathTruncated)
{
    pathTruncated = false;
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

    // Extract path — detect buffer overflow (M8: URI Too Long).
    uint8_t pIdx = 0;
    while (idx < lineLen && line[idx] != ' ' && line[idx] != '?'
           && pIdx < (HTTP_PATH_MAX - 1U))
    {
        path[pIdx] = line[idx];
        pIdx++;
        idx++;
    }
    path[pIdx] = '\0';

    // If there are still unread non-terminator bytes after the path buffer
    // is full, the URI was longer than HTTP_PATH_MAX — signal 414.
    if (pIdx >= (HTTP_PATH_MAX - 1U)
        && idx < lineLen
        && line[idx] != ' '
        && line[idx] != '?'
        && line[idx] != '\0')
    {
        pathTruncated = true;
    }

    return true;
}

/**
 * Read HTTP headers, extracting Content-Length, Content-Type, and
 * the X-ARES-Token authentication header.
 * Stops at the first empty line (end of headers) or when
 * HTTP_MAX_HEADERS headers have been read (REST-3.1, PO10-2).
 *
 * @param authToken     Output buffer for the X-ARES-Token value.
 * @param authTokenMax  Capacity of @p authToken (including NUL).
 */
static void parseHeaders(WiFiClient& client,
                          uint32_t& contentLength,
                          bool& hasJsonContentType,
                          char* authToken,
                          uint8_t authTokenMax)
{
    contentLength = 0;
    hasJsonContentType = false;
    authToken[0] = '\0';
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
            // RFC 7230 §3.2.3: strip leading OWS (any number of SP/HTAB).
            const char* val = ares::api::owsSkipLeading(hdr + 15, HTTP_HEADER_MAX);
            contentLength = static_cast<uint32_t>(
                strtoul(val, nullptr, 10));
        }
        // REST-2.4: Content-Type
        else if (strncasecmp(hdr, "Content-Type:", 13) == 0)
        {
            // RFC 7230 §3.2.3: strip leading OWS (any number of SP/HTAB).
            const char* val = ares::api::owsSkipLeading(hdr + 13, HTTP_HEADER_MAX);
            hasJsonContentType =
                (strncasecmp(val, "application/json", 16) == 0);
        }
        // X-ARES-Token: bearer token for API authentication
        else if (strncasecmp(hdr, "X-ARES-Token:", 13) == 0)
        {
            // RFC 7230 §3.2.3: strip leading OWS (any number of SP/HTAB).
            const char* val = ares::api::owsSkipLeading(hdr + 13, HTTP_HEADER_MAX);
            // Copy at most authTokenMax-1 chars (CERT-4.1: bounded copy).
            uint8_t tIdx = 0;
            while (*val != '\0' && tIdx < (authTokenMax - 1U))
            {
                authToken[tIdx] = *val;
                tIdx++;
                val++;
            }
            authToken[tIdx] = '\0';
            // RFC 7230 §3.2.3: strip trailing OWS so checkToken() never
            // rejects a valid token that arrived with trailing spaces/tabs.
            ares::api::owsTrimTrailing(authToken, tIdx);
        }

        headerCount++;
    }
}

/**
 * Read the request body into a caller-provided buffer.
 * @return true  if exactly toRead bytes were received.
 *         false if the client closed early or the socket timed out before
 *               all bytes arrived (premature EOF).  The caller must send
 *               a 4xx response in this case — do NOT process a partial body.
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
    body[bodyLen] = '\0';   // safe: buffer is always toRead+1 or larger
    return bodyLen == toRead;
}

static bool allowNonJsonBody(const char* method, const char* path)
{
    return (strcmp(method, "PUT") == 0
            && strncmp(path, "/api/missions/", 14) == 0);
}

// ── HTTP request helpers ────────────────────────────────────

bool ApiServer::requirePost(WiFiClient& client,
                            const char* method, const char* path)
{
    if (strcmp(method, "POST") == 0) { return true; }
    sendError(client, 405, "method not allowed");
    LOG_W(TAG, "%s %s 405", method, path);
    return false;
}

bool ApiServer::readBody(WiFiClient& client,
                         const char* method, const char* path,
                         uint32_t contentLength, uint32_t bodyLimit,
                         char* body, uint32_t& bodyLen)
{
    // REST-3.1: reject oversized payloads before reading.
    if (contentLength > bodyLimit)
    {
        sendError(client, 413, "payload too large");
        LOG_W(TAG, "%s %s 413: payload too large", method, path);
        return false;
    }
    if (!readRequestBody(client, body, bodyLimit, contentLength, bodyLen))
    {
        sendError(client, 400, "request body incomplete");
        LOG_W(TAG, "%s %s 400: body incomplete (got %" PRIu32 " of %" PRIu32 " bytes)",
              method, path, bodyLen, contentLength);
        return false;
    }
    return true;
}

// ── HTTP request handling ───────────────────────────────────

void ApiServer::handleClient(WiFiClient& client)
{
    // Phase 1: Parse request line (PO10-4.1)
    char method[HTTP_METHOD_MAX] = {};
    char path[HTTP_PATH_MAX]     = {};
    bool pathTruncated = false;

    if (!parseRequestLine(client, method, path, pathTruncated))
    {
        return;  // Empty or timed-out request
    }

    // M8: URI Too Long — reject before allocating body buffer or routing.
    if (pathTruncated)
    {
        sendError(client, 414, "URI Too Long");
        LOG_W(TAG, "414: URI Too Long (path buffer exhausted)");
        return;
    }

    // Phase 2: Parse headers (including auth token)
    uint32_t contentLength = 0;
    bool hasJsonContent = false;
    char authToken[HTTP_TOKEN_MAX] = {};
    parseHeaders(client, contentLength, hasJsonContent,
                 authToken, static_cast<uint8_t>(sizeof(authToken)));

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
    char* body = smallBody;
    uint32_t bodyLen = 0;

    if (isMissionUpload)
    {
        body = MissionUploadBuf::get(taskHandle_);
        memset(body, 0, MissionUploadBuf::SIZE);
    }

    if (contentLength > 0
        && !readBody(client, method, path,
                     contentLength, bodyLimit, body, bodyLen))
    {
        return;
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
    routeRequest(client, method, path, body, bodyLen, authToken);
}

void ApiServer::routeRequest(WiFiClient& client,
                             const char* method,
                             const char* path,
                             const char* body,
                             uint32_t    bodyLen,
                             const char* authToken)
{
    // OPTIONS preflight: always allowed (must respond before auth check so
    // the browser can learn which headers are permitted).
    if (strcmp(method, "OPTIONS") == 0)
    {
        sendNoContent(client, 204);
        LOG_D(TAG, "OPTIONS %s 204", path);
        return;
    }

    // ── Authentication guard ──────────────────────────────────
    // When an api_token is configured, all endpoints except a small set of
    // read-only GET paths require a valid X-ARES-Token header.
    if (devCfg_.isAuthEnabled())
    {
        // Public read-only endpoints that do not require authentication.
        const bool isPublicGet =
            (strcmp(method, "GET") == 0)
            && (strcmp(path, "/api/status")     == 0
                || strcmp(path, "/api/imu")     == 0
                || strncmp(path, "/api/imu/", 9) == 0);

        if (!isPublicGet && !devCfg_.checkToken(authToken))
        {
            sendError(client, 401, "unauthorized — missing or invalid X-ARES-Token");
            LOG_W(TAG, "%s %s 401: bad token", method, path);
            return;
        }
    }

    if (routeStatusAndConfigRequest(client, method, path, body, bodyLen)) { return; }
    if (routeDeviceConfigRequest(client, method, path, body, bodyLen))    { return; }
    if (routeFlightRequest(client, method, path, body, bodyLen)) { return; }

    if (strncmp(path, "/api/pulse", 10) == 0)
    {
        if (strcmp(path, "/api/pulse/status") == 0)
        {
            if (strcmp(method, "GET") != 0)
            {
                sendError(client, 405, "method not allowed");
                LOG_W(TAG, "%s /api/pulse/status 405", method);
                return;
            }
            handlePulseStatus(client);
            return;
        }
        sendError(client, 404, "not found");
        LOG_W(TAG, "%s %s 404", method, path);
        return;
    }

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

bool ApiServer::routeDeviceConfigRequest(WiFiClient& client,
                                          const char* method,
                                          const char* path,
                                          const char* body,
                                          uint32_t    bodyLen)
{
    if (strcmp(path, "/api/device/config") != 0) { return false; }

    if (strcmp(method, "GET") == 0)
    {
        handleDeviceConfigGet(client);
        LOG_D(TAG, "GET /api/device/config 200");
    }
    else if (strcmp(method, "PUT") == 0)
    {
        handleDeviceConfigPut(client, body, bodyLen);
    }
    else
    {
        sendError(client, 405, "method not allowed");
        LOG_W(TAG, "%s %s 405", method, path);
    }
    return true;
}

bool ApiServer::routeFlightRequest(WiFiClient& client,
                                   const char* method,
                                   const char* path,
                                   const char* body,
                                   uint32_t    bodyLen)
{
    if (strcmp(path, "/api/mode") == 0)
    {
        if (!requirePost(client, method, path)) { return true; }
        handleMode(client, body, bodyLen);
        return true;
    }
    if (strcmp(path, "/api/arm") == 0)
    {
        if (!requirePost(client, method, path)) { return true; }
        handleArm(client);
        return true;
    }
    if (strcmp(path, "/api/abort") == 0)
    {
        if (!requirePost(client, method, path)) { return true; }
        handleAbort(client);
        return true;
    }
    if (strcmp(path, "/api/scans/i2c") == 0)
    {
        if (!requirePost(client, method, path)) { return true; }
        // H8: I2C scan triggers bus activity that must not run during flight.
        if (isFlightLocked())
        {
            sendError(client, 409, "operation locked during flight");
            LOG_W(TAG, "POST /api/scans/i2c 409: flight locked");
            return true;
        }
        handleI2cScan(client);
        return true;
    }
    if (strcmp(path, "/api/scans/uart") == 0)
    {
        if (!requirePost(client, method, path)) { return true; }
        // H8: UART scan probes ports also used by GPS/radio during flight.
        if (isFlightLocked())
        {
            sendError(client, 409, "operation locked during flight");
            LOG_W(TAG, "POST /api/scans/uart 409: flight locked");
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
    // H6: map every code to its correct HTTP reason phrase.
    const char* reason = "OK";
    switch (code)
    {
    case 200: reason = "OK";                  break;
    case 201: reason = "Created";             break;
    case 202: reason = "Accepted";            break;
    case 204: reason = "No Content";          break;
    case 400: reason = "Bad Request";         break;
    case 401: reason = "Unauthorized";        break;
    case 404: reason = "Not Found";           break;
    case 405: reason = "Method Not Allowed";  break;
    case 409: reason = "Conflict";            break;
    case 413: reason = "Payload Too Large";   break;
    case 414: reason = "URI Too Long";        break;
    case 500: reason = "Internal Server Error"; break;
    default:  reason = "Error";               break;  // CERT-6.3
    }
    client.printf("HTTP/1.1 %u %s\r\n", code, reason);
    client.print("Content-Type: application/json\r\n");
    client.printf("Content-Length: %" PRIu32 "\r\n", len);
    client.print(getCorsHeaders());
    client.print("Connection: close\r\n");
    client.print("\r\n");
    client.write(json, len);
}

void ApiServer::sendError(WiFiClient& client, uint16_t code,
                           const char* message)
{
    // M11: Truncate message to prevent stack-buffer overflow when
    // ARES_ASSERT is compiled out in release builds.
    static constexpr uint8_t MSG_SAFE_MAX = 220U;
    char safeMsg[MSG_SAFE_MAX + 1U] = {};
    if (message != nullptr)
    {
        (void)strncpy(safeMsg, message, MSG_SAFE_MAX);
        safeMsg[MSG_SAFE_MAX] = '\0';
    }

    // REST-2.2: error envelope
    JsonDocument doc;
    doc["error"] = safeMsg;

    char buf[256] = {};
    const size_t len = serializeJson(doc, buf, sizeof(buf) - 1U);
    buf[len] = '\0';

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
    client.print(getCorsHeaders());
    client.print("Connection: close\r\n");
    client.print("\r\n");
    client.write(buf, len);
}

void ApiServer::sendNoContent(WiFiClient& client, uint16_t code)
{
    client.printf("HTTP/1.1 %u No Content\r\n", code);
    client.print(getCorsHeaders());
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

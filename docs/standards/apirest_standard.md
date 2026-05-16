# ARES REST API Standard

**Amateur Rocket Embedded System**
Design and implementation rules for the on-board HTTP REST API
used for ground configuration, system monitoring, data download,
and pre-flight operations over WiFi.
Companion to the [ARES Coding Standard](ares_coding_standard.md).

---

## Context

ARES exposes an HTTP/1.1 REST API via the ESP32-S3 WiFi soft-AP.
The API is the **only** interactive channel available on the
launch pad — it is used to configure the rocket, review sensor
health, download flight logs, update parameters, and manage
operating modes.

### Design constraints

| Constraint                    | Impact                                        |
|-------------------------------|-----------------------------------------------|
| ESP32-S3 soft-AP, max 4 STA  | One server, few clients, no internet          |
| 512 KB SRAM                  | Fixed-size JSON buffers, no streaming (PO10-3)|
| Battery-powered               | WiFi TX ≤ 8 dBm, keep AP alive short          |
| Safety-critical context       | Config changes locked during flight (CERT-8)  |
| Ground-only usage             | API disabled after launch detect              |
| No TLS library on target      | Physical proximity is the trust boundary      |
| ArduinoJson for serialisation | Static `JsonDocument` with bounded capacity   |

### Threat model

The WiFi link is a local, short-range channel with WPA2-PSK
encryption. However, any device that knows the passphrase can
connect. The API must defend against:

| Threat                   | Mitigation                          | Rule        |
|--------------------------|--------------------------------------|-------------|
| Oversized request body   | Content-Length limit                 | REST-3      |
| Malformed JSON           | Parse-or-reject, no partial apply   | REST-4      |
| Out-of-range values      | Per-field range validation           | REST-5      |
| Invalid state transition | FSM guard in mode/arm/abort          | REST-6      |
| Config change in flight  | 409 Conflict lock                    | REST-6      |
| Replay / double-arm      | Idempotent design                    | REST-2      |
| Slowloris / stale conn   | Connection timeout                   | REST-10     |
| Path traversal           | Whitelist routes, no FS path mapping | REST-11     |
| Concurrent requests      | Mutex with bounded timeout (RTOS-4)  | REST-8      |
| Unauthorised API access  | Optional bearer token (`X-ARES-Token`)| REST-12    |
| Token timing side-channel| Constant-time comparison (XOR acc.)  | REST-12     |
| Token leakage in logs    | Request bodies never logged          | REST-9.4    |

---

## REST-1 — Resource-Oriented Design

Every API endpoint must map to a **resource** (noun), not an action
(verb). Operations on resources use standard HTTP methods.

### REST-1.1 — URL structure

```
/api/<resource>[/<id>]
```

- All endpoints live under the `/api/` prefix.
- Resource names are **plural nouns** in `lowercase`.
- Path segments use `snake_case` when multi-word.
- No trailing slashes.
- No query strings for state-changing operations.
- Maximum URL length: 64 characters (CERT-3.2).

### REST-1.2 — HTTP method mapping

| Method   | Semantics                                    | Safe | Idempotent |
|----------|----------------------------------------------|------|------------|
| `GET`    | Read resource state — no side effects        | Yes  | Yes        |
| `PUT`    | Replace / update resource (full or partial)  | No   | Yes        |
| `POST`   | Trigger action or create resource            | No   | No*        |
| `DELETE` | Remove resource                              | No   | Yes        |
| `OPTIONS`| CORS preflight                               | Yes  | Yes        |

\* POST endpoints that trigger idempotent actions (e.g., arm) must
document this explicitly.

### REST-1.3 — Resource catalogue

| Resource              | GET                      | PUT                  | POST              | DELETE       |
|-----------------------|--------------------------|----------------------|--------------------|---------------|
| `/api/status`         | System status            | —                    | —                  | —             |
| `/api/config`         | Read runtime config      | Update runtime config| —                  | —             |
| `/api/device/config`  | Read device security cfg | Update device security cfg | —          | —             |
| `/api/mode`           | —                        | —                    | Change mode        | —             |
| `/api/arm`            | —                        | —                    | Arm flight FSM     | —             |
| `/api/abort`          | —                        | —                    | Abort flight       | —             |
| `/api/logs`           | List log files           | —                    | —                  | Delete logs  |
| `/api/logs/:id`       | Download log file        | —                    | —                  | Delete log   |

### Rules

| ID        | Rule                                                            |
|-----------|-----------------------------------------------------------------|
| REST-1.1  | All endpoints must be under `/api/`.                            |
| REST-1.2  | Use nouns for resources, HTTP methods for verbs.                |
| REST-1.3  | `GET` must be side-effect free — never modify state.            |
| REST-1.4  | Every resource must be documented in the resource catalogue.    |
| REST-1.5  | Unknown routes must return `404 Not Found`.                     |

---

## REST-2 — Response Format

All responses use `application/json` with a consistent envelope.
There is **no** XML, HTML, or plain-text mode.

### REST-2.1 — Success response

```json
{
  "field1": "value1",
  "field2": 42
}
```

Successful responses return the resource representation directly
(no wrapper object). The HTTP status code conveys success.

### REST-2.2 — Error response

```json
{
  "error": "descriptive, actionable message"
}
```

Error responses always contain exactly one field: `"error"`.
The message must help the client understand **what went wrong**
and **how to fix it** — not expose internal implementation.

### REST-2.3 — HTTP status codes

| Code | Meaning                          | When to Use                                |
|------|----------------------------------|--------------------------------------------|
| 200  | OK                               | Successful GET, PUT, POST                  |
| 204  | No Content                       | OPTIONS preflight, DELETE success           |
| 400  | Bad Request                      | Malformed JSON, missing field, out of range |
| 404  | Not Found                        | Unknown route or resource                  |
| 405  | Method Not Allowed               | Valid route, wrong HTTP method              |
| 409  | Conflict                         | Invalid state transition, locked resource  |
| 413  | Payload Too Large                | Request body exceeds limit                 |
| 500  | Internal Server Error            | Unexpected failure (should never occur)    |

### REST-2.4 — Content-Type header

| Direction | Header                              |
|-----------|-------------------------------------|
| Response  | `Content-Type: application/json`    |
| Request   | Must be `application/json` for PUT/POST with body |

### Rules

| ID        | Rule                                                            |
|-----------|-----------------------------------------------------------------|
| REST-2.1  | All responses must be valid JSON with `Content-Type: application/json`. |
| REST-2.2  | Error responses must contain a single `"error"` string field.   |
| REST-2.3  | HTTP status codes must match the table above — no custom codes. |
| REST-2.4  | PUT/POST requests without `Content-Type: application/json` must return 400. |
| REST-2.5  | Response bodies must not exceed the JSON buffer capacity.       |

---

## REST-3 — Request Size Limits

The ESP32-S3 has limited SRAM. Every incoming request body is parsed
into a fixed-size buffer. Oversized requests must be rejected
**before** parsing.

### REST-3.1 — Limits

| Limit                    | Value             | Rationale                    |
|--------------------------|-------------------|------------------------------|
| Max request body         | 1024 bytes        | ArduinoJson static doc       |
| Max URL length           | 64 bytes          | Stack buffer (CERT-3.2)      |
| Max header count         | 16                | Prevent header flooding      |
| Max concurrent clients   | 4                 | ESP32 soft-AP limit          |

### REST-3.2 — Enforcement

```cpp
// ✅ Check Content-Length before reading body
const uint32_t contentLen = request.contentLength();
if (contentLen > MAX_REQUEST_BODY)
{
    sendError(client, 413, "payload too large");
    return;
}
```

### Rules

| ID        | Rule                                                            |
|-----------|-----------------------------------------------------------------|
| REST-3.1  | Request body size must be checked before parsing.               |
| REST-3.2  | Requests exceeding limits must return 413 immediately.          |
| REST-3.3  | All size limits must be named constants in `config.h` (MISRA-7).|
| REST-3.4  | No dynamic allocation for request buffers (PO10-3).             |

---

## REST-4 — JSON Handling

JSON parsing and serialisation use ArduinoJson with statically
allocated documents. Parse failures reject the entire request —
no partial application.

### REST-4.1 — Parsing discipline

```cpp
// ✅ Static document — no heap (PO10-3)
StaticJsonDocument<MAX_REQUEST_BODY> doc;

DeserializationError err = deserializeJson(doc, body, bodyLen);
if (err != DeserializationError::Ok)
{
    sendError(client, 400, "invalid JSON");
    return;
}
```

### REST-4.2 — Serialisation discipline

```cpp
// ✅ Static output buffer
StaticJsonDocument<MAX_RESPONSE_BODY> resp;
resp["board"] = ares::BOARD_NAME;
resp["version"] = ARES_VERSION_STRING;

char buf[MAX_RESPONSE_BODY] = {};
const uint32_t len = serializeJson(resp, buf, sizeof(buf));
```

### REST-4.3 — Field access safety

Never assume a field exists. Always use `containsKey()` or
check for null before use:

```cpp
// ❌ Crashes if field missing
uint32_t interval = doc["telemetryIntervalMs"];

// ✅ Safe field access with default
uint32_t interval = doc["telemetryIntervalMs"] | currentInterval;
```

### Rules

| ID        | Rule                                                            |
|-----------|-----------------------------------------------------------------|
| REST-4.1  | JSON documents must be `StaticJsonDocument` — no heap.          |
| REST-4.2  | `deserializeJson` errors must return 400 — no partial parsing.  |
| REST-4.3  | Missing fields must use current value as default — never zero.  |
| REST-4.4  | String values from JSON must be length-checked before copy (CERT-3). |
| REST-4.5  | JSON document capacity must be a named constant.                |

---

## REST-5 — Input Validation

Every field in a PUT/POST request must be validated against
documented constraints **before** applying to system state.
This is the REST-specific extension of CERT-1.

### REST-5.1 — Validation pattern

```cpp
if (doc.containsKey("telemetryIntervalMs"))
{
    const uint32_t val = doc["telemetryIntervalMs"];
    if (val < TELEMETRY_INTERVAL_MIN || val > TELEMETRY_INTERVAL_MAX)
    {
        sendError(client, 400, "telemetryIntervalMs out of range");
        return;
    }
    config.telemetryIntervalMs = val;
}
```

### REST-5.2 — Validation requirements

| Check               | Enforcement                                    |
|----------------------|------------------------------------------------|
| Type correctness     | ArduinoJson `is<T>()` before cast              |
| Numeric range        | Min/max constants per field                    |
| String length        | `strnlen` ≤ field buffer size (CERT-3)         |
| Enum membership      | Check against FIRST/LAST sentinels (CERT-6)    |
| Cross-field logic    | Validate dependencies after individual checks  |

### REST-5.3 — Reject-first, apply-after

All fields must be validated **before** any are applied. A request
with one invalid field must be rejected entirely — no partial update.

```cpp
// ✅ Phase 1: validate all fields
// Phase 2: apply all fields (only reached if Phase 1 passes)
```

### Rules

| ID        | Rule                                                            |
|-----------|-----------------------------------------------------------------|
| REST-5.1  | Every mutable field must have documented min/max or allowed values.|
| REST-5.2  | Type, range, and length must be checked before use.             |
| REST-5.3  | Validation failure returns 400 with a descriptive error message.|
| REST-5.4  | All validation bounds must be named constants (MISRA-7).        |
| REST-5.5  | Partial updates are forbidden — validate all, then apply all.   |

---

## REST-6 — State Guards

Certain API operations are only valid in specific operating modes.
Every state-changing endpoint must check the current mode and
reject invalid transitions with `409 Conflict`.

### REST-6.1 — Mode transition matrix

| Endpoint          | Allowed From              | Forbidden From      |
|-------------------|---------------------------|----------------------|
| PUT /api/config   | IDLE, TEST, ERROR         | FLIGHT, RECOVERY     |
| POST /api/mode    | See transition table       | See transition table |
| POST /api/arm     | FLIGHT (PAD phase)        | All others           |
| POST /api/abort   | FLIGHT                    | All others           |
| DELETE /api/logs  | IDLE                      | All others           |

### REST-6.2 — Flight lock

During FLIGHT and RECOVERY modes, the API is restricted to
**read-only** operations (GET endpoints only). All PUT, POST,
and DELETE requests must return `409 Conflict`:

```json
{
  "error": "operation locked during flight"
}
```

### REST-6.3 — Config persistence

Configuration changes applied via PUT /api/config must be:
1. Validated (REST-5).
2. Applied to the runtime config struct under mutex (REST-8).
3. Persisted to flash (LittleFS) so they survive reboot.
4. Confirmed in the response by echoing the new config.

A flash write failure must not roll back the runtime config —
log a warning and report success (the config is active even if
not persisted). The next PUT will retry persistence.

### Rules

| ID        | Rule                                                            |
|-----------|-----------------------------------------------------------------|
| REST-6.1  | State-changing endpoints must verify the current operating mode.|
| REST-6.2  | Invalid transitions must return 409 with a descriptive error.   |
| REST-6.3  | All write endpoints are locked during FLIGHT and RECOVERY.      |
| REST-6.4  | Config changes must be persisted to flash when possible.        |
| REST-6.5  | Flash persistence failure must not block the runtime update.    |

---

## REST-7 — CORS

Cross-Origin Resource Sharing headers are required so that a
browser-based ground station (served from a different origin or
`file://`) can access the API.

### REST-7.1 — Required headers

All responses must include CORS headers built from the live
`DeviceConfig::corsOrigin()` value. The default produces:

```
Access-Control-Allow-Origin: *
Access-Control-Allow-Methods: GET, PUT, POST, DELETE, OPTIONS
Access-Control-Allow-Headers: Content-Type, X-ARES-Token
```

`X-ARES-Token` must always appear in `Access-Control-Allow-Headers` so
browser preflight checks succeed when token auth is enabled. The origin
value is runtime-configurable via `PUT /api/device/config`.

### REST-7.2 — Preflight handling

`OPTIONS` requests on any route must return `204 No Content`
with the CORS headers above. No body, no JSON. OPTIONS requests
are never subject to auth checks (REST-12.3).

### REST-7.3 — CORS header buffer

The full CORS header block is stored in a static char array
(`s_corsHeadersBuf` in `api_server.cpp`) and rebuilt at `begin()`
and after every successful `PUT /api/device/config`. It is written
and read exclusively from the API task — no mutex required.

### Rules

| ID        | Rule                                                            |
|-----------|-----------------------------------------------------------------|
| REST-7.1  | Every response must include CORS headers.                       |
| REST-7.2  | OPTIONS preflight must return 204 with CORS headers only.       |
| REST-7.3  | `Access-Control-Allow-Headers` must include `X-ARES-Token`.     |
| REST-7.4  | CORS headers must be generated from `DeviceConfig::buildCorsHeader()`, not hardcoded. |

---

## REST-8 — Thread Safety

The API task runs on its own FreeRTOS task. Shared state (config,
operating mode, sensor data) must be accessed through RTOS
primitives.

### REST-8.1 — Concurrency model

| Shared Resource      | Protection              | Timeout      |
|----------------------|--------------------------|--------------|
| Runtime config       | Mutex (priority inherit) | 50 ms        |
| Operating mode       | `std::atomic<uint8_t>`   | Lock-free    |
| Sensor readings      | Mutex or copy-on-read    | 50 ms        |
| Flight log file list | Storage mutex (RTOS-4)   | 1000 ms      |

### REST-8.2 — Mutex discipline

```cpp
// ✅ ScopedLock — RAII, never leaks (CERT-18.1)
ScopedLock guard(cfgMtx_, pdMS_TO_TICKS(50));
if (!guard.acquired())
{
    sendError(client, 500, "resource busy");
    return;
}
config_.telemetryIntervalMs = newInterval;
```

### Rules

| ID        | Rule                                                            |
|-----------|-----------------------------------------------------------------|
| REST-8.1  | All shared state must be protected by RTOS primitives (RTOS-4). |
| REST-8.2  | Mutex acquisition must use bounded timeout (RTOS-8, CERT-10).  |
| REST-8.3  | Mutex must use RAII guards — no manual lock/unlock (CERT-18.1). |
| REST-8.4  | Timeout failure must return an error response, never block.     |

---

## REST-9 — Logging and Diagnostics

Every request must be logged for post-flight analysis.  The log
level depends on the outcome.

### REST-9.1 — Logging levels

| Event                  | Level    | Example                             |
|------------------------|----------|--------------------------------------|
| Successful GET         | DEBUG    | `GET /api/status 200`               |
| Successful PUT/POST    | INFO     | `PUT /api/config 200`               |
| Client error (4xx)     | WARNING  | `PUT /api/config 400: invalid JSON` |
| Server error (5xx)     | ERROR    | `GET /api/status 500: mutex timeout`|
| State-change action    | INFO     | `POST /api/arm 200`                 |

### REST-9.2 — Log format

```
[uptime] I API: METHOD /path CODE [detail]
```

### Rules

| ID        | Rule                                                            |
|-----------|-----------------------------------------------------------------|
| REST-9.1  | Every request must be logged with method, path, and status code.|
| REST-9.2  | 4xx/5xx responses must include the error reason in the log.     |
| REST-9.3  | Logging must not block the response — log after sending.        |
| REST-9.4  | Request bodies must **not** be logged (may contain secrets).    |

---

## REST-10 — Connection Management

The ESP32 soft-AP has limited resources. Connections must be
actively managed to prevent resource exhaustion.

### REST-10.1 — Connection limits

| Parameter              | Value          | Rationale                  |
|------------------------|----------------|----------------------------|
| Max concurrent clients | 4              | ESP32 soft-AP limit        |
| Connection timeout     | 5000 ms        | Prevent stale connections  |
| Keep-alive             | Disabled       | Simplify resource mgmt    |
| Max requests/client    | 1 (close after)| No HTTP pipelining         |

### REST-10.2 — Timeout enforcement

Every socket read must have a bounded timeout. A client that sends
headers but never finishes the request must be disconnected after
the connection timeout.

### Rules

| ID         | Rule                                                            |
|------------|-----------------------------------------------------------------|
| REST-10.1  | Max clients must match `WIFI_AP_MAX_CLIENTS` in `config.h`.    |
| REST-10.2  | Every socket read must have a timeout (CERT-10).                |
| REST-10.3  | Connections must be closed after response — no keep-alive.      |
| REST-10.4  | Stale connections must be timed out and cleaned up.             |

---

## REST-11 — Security

The API runs on an isolated soft-AP with WPA2-PSK. Physical
proximity is the primary trust boundary. Nevertheless, input
validation and state guards prevent accidental misuse.

### REST-11.1 — Security controls

| Control                     | Implementation                                |
|-----------------------------|-----------------------------------------------|
| Network authentication      | WPA2-PSK (AP password, configurable)          |
| API authentication          | Optional `X-ARES-Token` bearer token (REST-12)|
| Input validation            | REST-5 (type, range, length)                  |
| State guards                | REST-6 (mode-locked operations)               |
| Path traversal prevention   | Whitelist routing — no filesystem mapping      |
| Injection prevention        | No eval, no SQL, no shell commands            |
| Buffer overflow prevention  | Fixed buffers, Content-Length check (REST-3)  |
| Information leakage         | No stack traces, no internal paths in errors  |
| Timing side-channel         | Constant-time token comparison (REST-12.2)    |

### REST-11.2 — Route handling

Routes are matched against a **whitelist** of known paths. Any
request to an unregistered path returns 404. There is no wildcard
routing, no regex routing, and no filesystem path mapping.

```cpp
// ✅ Explicit route table
if (path == "/api/status")       { handleStatus(client); }
else if (path == "/api/config")  { handleConfig(client, method); }
else                              { sendError(client, 404, "not found"); }
```

### REST-11.3 — Sensitive operations

| Operation           | Additional Safeguard                         |
|---------------------|----------------------------------------------|
| ARM flight          | Only in FLIGHT/PAD mode                      |
| Change mode         | Transition matrix validation (REST-6.1)      |
| Delete logs         | Only in IDLE mode                            |
| Config update       | Locked in FLIGHT/RECOVERY (REST-6.2)         |
| Device config update| Requires valid token when auth enabled (REST-12) |
| `api_token` field   | Never returned in GET responses (write-only) |

### Rules

| ID         | Rule                                                            |
|------------|-----------------------------------------------------------------|
| REST-11.1  | All routes must be explicitly whitelisted — no wildcard routing.|
| REST-11.2  | Unknown routes must return 404 — never expose internals.        |
| REST-11.3  | Error messages must not reveal file paths or stack traces.      |
| REST-11.4  | No shell command execution from API input.                      |
| REST-11.5  | Request data must never be used in `snprintf` format strings.   |
| REST-11.6  | Security-sensitive fields (`api_token`) must never appear in responses. |

---

## REST-12 — Token Authentication

The API supports an optional bearer token that adds an application-level
authentication factor on top of the WPA2-PSK network password.
The token is configured via `PUT /api/device/config` and stored in
LittleFS as part of the `DeviceConfig` object.

### REST-12.1 — Token lifecycle

| Event                            | Behaviour                                         |
|----------------------------------|---------------------------------------------------|
| No `api_token` configured        | Auth disabled — all endpoints open (legacy mode) |
| `api_token` set (8–64 chars)      | Auth enabled — protected endpoints require header |
| `api_token` set to `""`          | Auth disabled — reverts to open mode             |
| Device reboots                   | Token reloaded from LittleFS — no reset           |
| `/ares_device.json` absent       | `setDefaults()` runs — open mode, factory password|

### REST-12.2 — Token validation

The `X-ARES-Token` header value must be compared against the stored
token using a **constant-time** algorithm to prevent timing
side-channel attacks (OWASP A07:2021):

```cpp
// ✔️ Constant-time comparison — always iterates full stored-token length
uint8_t diff = 0;
for (uint8_t i = 0; i < storedLen; i++)
{
    diff |= static_cast<uint8_t>(stored[i]) ^
            static_cast<uint8_t>(
                (i < providedLen) ? provided[i] : 0x00);
}
diff |= static_cast<uint8_t>(storedLen ^ providedLen);
return diff == 0;
```

A variable-time comparison (`strcmp`, `strncmp`) is forbidden for
token validation.

### REST-12.3 — Public endpoints

The following endpoints are always accessible regardless of token state:

| Endpoint          | Reason                                       |
|-------------------|----------------------------------------------|
| `OPTIONS *`       | CORS preflight — must precede auth            |
| `GET /api/status` | Ground station heartbeat                     |
| `GET /api/imu`    | Passive sensor read                          |
| `GET /api/imu/*`  | IMU sub-paths                                |

All other endpoints require the token when auth is enabled.

### REST-12.4 — Token constraints

| Constraint         | Value                                               |
|--------------------|-----------------------------------------------------|
| Minimum length     | 8 characters                                        |
| Maximum length     | 64 characters                                       |
| Character set      | Printable ASCII (0x20–0x7E, excluding control chars)|
| Storage            | Plain-text in LittleFS JSON (physical security only)|
| Transport          | HTTP header (WPA2 encrypted at network layer)       |
| Logging            | Never — request bodies must not be logged (REST-9.4)|

### REST-12.5 — Response on auth failure

```json
{ "error": "unauthorized — missing or invalid X-ARES-Token" }
```

HTTP status: `401 Unauthorized`. Log at `WARNING` level with method and
path but **not** the supplied token value.

### Rules

| ID         | Rule                                                              |
|------------|-------------------------------------------------------------------|
| REST-12.1  | Token comparison must be constant-time (XOR accumulator).         |
| REST-12.2  | `strcmp`/`strncmp` are forbidden for token comparison.            |
| REST-12.3  | When auth is disabled (empty token) all endpoints are open.       |
| REST-12.4  | OPTIONS preflight must never be subject to auth checks.           |
| REST-12.5  | 401 responses must not reveal whether the token is wrong vs absent.|
| REST-12.6  | `api_token` must never appear in any GET response or log entry.   |

---

## REST-12 — Data Download

Flight log files stored on LittleFS may be downloaded via the API.
Due to SRAM constraints, large files must be sent in chunks.

### REST-12.1 — Log listing

`GET /api/logs` returns an array of available log files:

```json
[
  { "name": "/logs/flight_001.bin", "size": 32768 },
  { "name": "/logs/flight_002.bin", "size": 16384 }
]
```

### REST-12.2 — Log download

`GET /api/logs/:id` returns the raw file content with
`Content-Type: application/octet-stream`.

The file is read and sent in fixed-size chunks to avoid
allocating the entire file in SRAM:

```cpp
constexpr uint32_t CHUNK_SIZE = 512;  // bytes
uint8_t chunk[CHUNK_SIZE] = {};

while (remaining > 0)  // PO10-2: bounded by file size (STORAGE_MAX_FILE)
{
    const uint32_t toRead = (remaining < CHUNK_SIZE)
                          ? remaining : CHUNK_SIZE;
    // ... read and send chunk ...
    remaining -= toRead;
}
```

### REST-12.3 — Download constraints

| Constraint               | Value                  | Rationale              |
|--------------------------|------------------------|------------------------|
| Max file size            | 64 KiB (STORAGE_MAX_FILE) | Partition limit     |
| Chunk size               | 512 bytes              | Fits in task stack     |
| Concurrent downloads     | 1                      | Storage mutex blocks   |
| Download during flight   | Forbidden (REST-6.2)   | Performance / safety   |

### Rules

| ID         | Rule                                                            |
|------------|-----------------------------------------------------------------|
| REST-12.1  | Log listing must include filename and size for each entry.      |
| REST-12.2  | Large files must be sent in fixed-size chunks — no full-file alloc.|
| REST-12.3  | Chunk buffer must be stack-allocated, not heap (PO10-3).        |
| REST-12.4  | File paths from client input must be validated against the log directory.|
| REST-12.5  | Only files under `/logs/` are downloadable — no arbitrary paths.|

---

## REST-13 — API Task Architecture

The API runs as a dedicated FreeRTOS task following RTOS-2
(single responsibility).

### REST-13.1 — Task parameters

| Property       | Value                           | Standard       |
|----------------|---------------------------------|----------------|
| Task name      | `"api"`                         | RTOS-2.1       |
| Stack size     | `TASK_STACK_SIZE_API` (8 KiB)   | RTOS-7.2       |
| Priority       | `TASK_PRIORITY_API` (Low)       | RTOS-5.1       |
| Core affinity  | `tskNO_AFFINITY`                | RTOS-13        |
| Poll rate      | `API_RATE_MS` (50 ms)           | RTOS-6.3       |
| Allocation     | Static (`xTaskCreateStatic`)    | PO10-3, RTOS-7 |

### REST-13.2 — Task lifecycle

```
setup()
  └── WiFi AP init
  └── API task create (static)
        └── while (true)        // RTOS: task loop
              ├── accept client
              ├── parse request
              ├── route + handle
              ├── send response
              ├── close connection
              └── vTaskDelay(API_RATE_MS)
```

### REST-13.3 — WiFi lifecycle

WiFi is enabled during IDLE, TEST, and ERROR modes. Upon entering
FLIGHT mode, the WiFi AP **may** be disabled to save power and
eliminate RF interference. This is configurable:

```cpp
constexpr bool WIFI_DISABLE_IN_FLIGHT = true;  ///< Kill AP on launch.
```

### Rules

| ID         | Rule                                                            |
|------------|-----------------------------------------------------------------|
| REST-13.1  | The API must run as a dedicated RTOS task — not in `loop()`.    |
| REST-13.2  | Task stack and TCB must be statically allocated (PO10-3).       |
| REST-13.3  | Task parameters must be defined in `config.h` (RTOS-5).        |
| REST-13.4  | WiFi shutdown in flight must be configurable via `config.h`.    |

---

## REST-14 — Versioning and Compatibility

The API is versioned to allow future evolution without breaking
existing ground station software.

### REST-14.1 — Version embedding

The firmware version string (`ARES_VERSION_STRING`) is included in
`GET /api/status`. Ground station software can use it to detect
capability changes.

### REST-14.2 — Backward compatibility

| Change Type            | Policy                                      |
|------------------------|----------------------------------------------|
| Add new field to GET   | Allowed — clients must ignore unknown fields |
| Add new endpoint       | Allowed — unknown routes already return 404  |
| Remove field from GET  | Forbidden — breaks existing clients          |
| Change field type      | Forbidden — breaks existing clients          |
| Rename endpoint        | Forbidden — add new, deprecate old           |
| Add required PUT field | Forbidden — breaks existing clients          |

### Rules

| ID         | Rule                                                            |
|------------|-----------------------------------------------------------------|
| REST-14.1  | The firmware version must be included in status responses.      |
| REST-14.2  | Existing response fields must never be removed or type-changed. |
| REST-14.3  | New PUT/POST fields must be optional with a documented default. |
| REST-14.4  | Clients must ignore unknown response fields.                    |

---

## Rule Summary

| ID       | Rule                                                              | Category   |
|----------|-------------------------------------------------------------------|------------|
| REST-1   | Resource-oriented URL design under `/api/`                        | Design     |
| REST-2   | Consistent JSON responses with proper HTTP status codes           | Format     |
| REST-3   | Request size limits enforced before parsing                       | Security   |
| REST-4   | Static JSON documents, parse-or-reject, no partial apply          | Safety     |
| REST-5   | Validate type, range, length on every input field                 | Security   |
| REST-6   | State guards — mode-locked operations, flight write lock          | Safety     |
| REST-7   | CORS headers on all responses, 204 for OPTIONS                    | Interop    |
| REST-8   | RAII mutex for shared state, bounded timeouts                     | Concurrency|
| REST-9   | Log every request with method, path, and status code              | Diagnostics|
| REST-10  | Connection limits, timeouts, no keep-alive                        | Reliability|
| REST-11  | Whitelist routing, no path traversal, no info leakage             | Security   |
| REST-12  | Chunked download for large files, validated log paths             | Data       |
| REST-13  | Dedicated RTOS task, static allocation, configurable WiFi lifecycle| RTOS      |
| REST-14  | Version embedding, backward-compatible evolution                  | Compat     |

**Total: 14 rules, 53 sub-rules.**

---

## Cross-References

| ARES Standard | Relevant REST Rules                                |
|---------------|----------------------------------------------------|
| PO10-3        | REST-3.4, REST-4.1, REST-12.3, REST-13.2           |
| RTOS-4        | REST-8.1                                            |
| RTOS-5        | REST-13.3                                           |
| RTOS-7        | REST-13.2                                           |
| RTOS-8        | REST-8.2                                            |
| CERT-1        | REST-5 (all)                                        |
| CERT-3        | REST-4.4, REST-5.2                                  |
| CERT-6        | REST-5.2                                            |
| CERT-8        | REST-6.2                                            |
| CERT-10       | REST-8.2, REST-10.2                                 |
| CERT-18       | REST-8.3                                            |
| MISRA-7       | REST-3.3, REST-4.5, REST-5.4                        |
| DOX-3         | All handler functions                                |

---

## References

- Fielding, R.T. (2000). *Architectural Styles and the Design of
  Network-Based Software Architectures* (Doctoral dissertation, UC Irvine).
  Chapter 5: Representational State Transfer (REST).
- RFC 7231 — *Hypertext Transfer Protocol (HTTP/1.1): Semantics and Content*.
- RFC 7807 — *Problem Details for HTTP APIs* (informative — ARES uses
  a simplified `{"error": "..."}` envelope instead).
- OWASP REST Security Cheat Sheet —
  <https://cheatsheetseries.owasp.org/cheatsheets/REST_Security_Cheat_Sheet.html>
- OWASP API Security Top 10 (2023) —
  <https://owasp.org/API-Security/>
- ArduinoJson v7 documentation —
  <https://arduinojson.org/v7/>
- ESP-IDF HTTP Server —
  <https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/api-reference/protocols/esp_http_server.html>

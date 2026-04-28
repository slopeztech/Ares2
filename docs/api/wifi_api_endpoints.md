# ARES WiFi API Endpoints Guide

Complete reference for the REST API served by the ESP32 WiFi Access Point. Each section explains what the endpoint does, when to use it, why, and what to expect.

Source of truth: `src/api/api_server.cpp` and the per-domain handlers in `src/api/`.

---

## Connection

The ESP32 acts as a WiFi AP. No router required — connect directly from the ground computer.

| Parameter   | Value                            |
|-------------|----------------------------------|
| SSID        | `ARES-XXXX` (last 2 bytes MAC)   |
| Password    | `ares1234`                       |
| IP          | `192.168.4.1`                    |
| Port        | `80` (HTTP/1.1)                  |
| Max clients | 4 simultaneous                   |

All responses are `application/json` with CORS headers (`Access-Control-Allow-Origin: *`). The single exception is mission/log file downloads, which respond `text/plain` with a `Content-Disposition: attachment` header.

---

## Flight Lock

Several endpoints are blocked while the system is in `FLIGHT` or `RECOVERY` mode. These return `409 Conflict` with `"operation locked during flight"`. This is a safety measure to prevent configuration changes or mission interference while the vehicle is airborne. The lock is enforced by `isFlightLocked()` internally.

Endpoints subject to the flight lock: `PUT /api/config`, `POST /api/mode`, `POST /api/mission/activate`, `POST /api/mission/deactivate`, and log downloads (with an exception when AMS status is `complete` — see below).

---

## Operational Phases

| Phase       | Description |
|-------------|-------------|
| **Pre-flight** | Bench checks, hardware verification, mission upload, activation, configuration, and arming. This is the only phase where all endpoints are accessible. |
| **Flight** | System is in `FLIGHT` or `RECOVERY` mode. Configuration and structural changes are locked. Status monitoring is still available. |
| **Post-flight** | Vehicle recovered. Log extraction, cleanup, and mission teardown. |

---

## Endpoints by Phase

### Pre-flight: System Health

---

#### `GET /api/status`

**When to use:** First call after connecting to the AP and after recovery. Use it as the primary health dashboard.

**Why:** Returns a full snapshot of the system in a single call — operating mode, arm state, free heap, sensor readings, health flags per subsystem, and current runtime config. Checking this before any other operation tells you whether sensors have initialized correctly and whether storage and WiFi are healthy.

**Response:**
```json
{
  "board": "ESP32-S3 Zero Mini",
  "version": "0.1.0",
  "uptimeMs": 5340,
  "freeHeap": 245760,
  "mode": "IDLE",
  "armed": false,
  "wifiClients": 1,
  "telemetryIntervalMs": 500,
  "nodeId": 1,
  "ledBrightness": 80,
  "health": {
    "wifi": true,
    "baro": true,
    "gps": true,
    "imu": true
  },
  "baro": {
    "pressurePa": 101325.0,
    "temperatureC": 22.5,
    "altitudeM": 0.42
  },
  "gps": {
    "fix": true,
    "latitude": 00.000000,
    "longitude": 00.000000,
    "altitudeM": 650.2,
    "speedKmh": 0.0,
    "courseDeg": 180.0,
    "hdop": 1.2,
    "satellites": 8
  },
  "imu": {
    "accelX": 0.12, "accelY": -0.03, "accelZ": 9.81,
    "gyroX": 0.8, "gyroY": -0.2, "gyroZ": 0.1,
    "tempC": 31.4
  }
}
```

`baro`, `gps`, and `imu` objects are only present when the corresponding `health` flag is `true`.

---

#### `GET /api/imu`

**When to use:** When you need a dedicated, timestamped IMU reading during bench validation — for example, to check axis orientation or verify vibration levels with the motor idle.

**Why:** `/api/status` already includes IMU data, but this endpoint provides a focused read with an explicit `ok` flag and `status` string (`ok` / `error` / `not_ready`), which makes it easier to poll in a script or diagnostic tool without parsing the full status payload.

**Response (ok):**
```json
{
  "ok": true,
  "status": "ok",
  "timestampMs": 4500,
  "accelX": 0.12, "accelY": -0.03, "accelZ": 9.81,
  "gyroX": 0.8, "gyroY": -0.2, "gyroZ": 0.1,
  "tempC": 31.4
}
```

If `ok` is `false`, only `ok`, `status`, and `timestampMs` are present.

---

#### `GET /api/imu/health`

**When to use:** Lightweight probe — use it in a pre-launch checklist script that just needs to confirm the IMU is alive without the overhead of the full reading.

**Why:** Performs the same `imu_.read()` call as `/api/imu` but only returns `ok`, `status`, and `timestampMs`. Useful when polling rapidly or when bandwidth is constrained.

**Response:**
```json
{ "ok": true, "status": "ok", "timestampMs": 4500 }
```

---

#### `POST /api/scans/i2c`

**When to use:** After first assembly or any hardware change. Confirm that BMP280 (`0x76` or `0x77`) and MPU-6050 (`0x68` or `0x69`) are present and correctly wired before flight.

**Why:** An I2C device missing from the scan means a wiring fault, a missing pull-up, or a broken solder joint. Catching this on the bench avoids a silent sensor failure mid-flight. The scan covers the full valid I2C address range (`0x03–0x77`) across all registered buses.

**Request:** No body.

**Response:**
```json
{
  "scan": "i2c",
  "timestampMs": 1230,
  "buses": [
    {
      "name": "i2c0",
      "sda": 1, "scl": 2,
      "frequencyHz": 400000,
      "foundCount": 2,
      "reportedCount": 2,
      "truncated": false,
      "found": [104, 118]
    }
  ]
}
```

`truncated: true` means more than 24 devices were found on a bus (unlikely in normal hardware).

---

#### `POST /api/scans/uart`

**When to use:** Verify that the GPS (BN-220, UART1) and LoRa radio (DX-LR03, UART2) are connected and returning data before the mission starts.

**Why:** A UART device is passive — the firmware cannot know if a cable is disconnected without attempting to read. This endpoint checks whether there is data in the RX buffer for each configured UART port, gives the first byte as hex (`peekHex`), and also reports the GPS driver status and radio readiness. Useful to catch a disconnected antenna cable or a broken UART RX line before leaving the pad.

**Request:** No body.

**Response:**
```json
{
  "scan": "uart",
  "timestampMs": 1560,
  "ports": [
    {
      "name": "gps", "uart": 1, "baud": 9600,
      "available": true, "rxBuffered": 62, "hasData": true, "peekHex": "24"
    },
    {
      "name": "lora", "uart": 2, "baud": 9600,
      "available": true, "rxBuffered": 0, "hasData": false
    }
  ],
  "gpsDriver": "BN220",
  "gpsStatus": "ok",
  "radioDriver": "LORA",
  "radioReady": true
}
```

The GPS NMEA stream starts with `$` (0x24), so `"peekHex": "24"` is a good sign. LoRa is passive at idle so `hasData: false` is normal.

---

#### `GET /api/storage/health`

**When to use:** Confirm that LittleFS is mounted and available before uploading missions or expecting logs to be written.

**Why:** If storage is unavailable (mount failure, filesystem corruption), mission scripts cannot be loaded from flash and logs will not be saved. This endpoint returns a quick health flag without listing all files.

---

### Pre-flight: Mission Preparation

---

#### `GET /api/missions`

**When to use:** After connecting, to see which `.ams` scripts are already stored in flash.

**Why:** Before uploading a new script, verify that the intended file is not already there from a previous session, or that old files do not occupy space. Returns a JSON array with name and size in bytes for each file.

**Response:**
```json
[
  { "name": "flight.ams", "size": 842 },
  { "name": "test.ams",   "size": 310 }
]
```

---

#### `PUT /api/missions/<filename>.ams`

**When to use:** Upload a new or updated mission script to the flight computer before activation.

**Why:** The `.ams` file is the mission program — states, transitions, guard conditions, telemetry cadence. It must be in flash before the engine can load it. This is the only endpoint that does **not** use `application/json` as `Content-Type`; send the raw AMS text directly. The maximum script size is `AMS_MAX_SCRIPT_BYTES`. The server responds `204 No Content` on success.

**Request:**
```
PUT /api/missions/flight.ams HTTP/1.1
Content-Type: text/plain
Content-Length: <byte count>

include BN220 as GPS
include BMP280 as BARO
...
```

**Responses:** `204` on success, `413` if the script exceeds the size limit, `400` for invalid filename characters (only `a-z A-Z 0-9 _ - .` allowed, no `..`).

---

#### `GET /api/missions/<filename>.ams`

**When to use:** Verify that the script stored in flash matches what you intended to upload, before activating it.

**Why:** A corrupt transfer or a leftover old file can cause the engine to load the wrong logic. Downloading and diffing against the source is a cheap sanity check on the pad.

**Response:** Raw `.ams` text with `Content-Disposition: attachment`.

---

#### `DELETE /api/missions/<filename>.ams`

**When to use:** Remove an obsolete or incorrect script file before flight, especially if flash space is tight.

**Response:** `204 No Content` on success, `404` if the file does not exist.

---

#### `POST /api/mission/activate`

**When to use:** After uploading the script and verifying it, activate it so the engine parses and loads it into memory.

**Why:** Activation parses the `.ams` source, builds the state machine in RAM, and puts the engine into `LOADED` state — paused and ready but not yet executing. The engine does **not** start running at this point; that happens on `POST /api/arm`. This two-step design (activate → arm) gives you a chance to inspect the parsed state with `GET /api/mission` before committing to execution.

**Request:**
```json
{ "file": "flight.ams" }
```

**Response:** AMS engine status (same as `GET /api/mission`):
```json
{ "status": "loaded", "activeFile": "flight.ams", "state": "WAIT", "error": "" }
```

**Errors:** `409` if the script cannot be loaded (parse error, file not found).

---

#### `GET /api/mission` / `GET /api/missions/active`

**When to use:** After activation, to confirm the engine loaded correctly and is sitting in the expected initial state. Also useful mid-mission (if WiFi is still up) to monitor AMS progression.

**Why:** This shows the internal engine snapshot: `status` (`idle` / `loaded` / `running` / `complete` / `error`), `activeFile`, current `state` name, and `lastError`. If activation produced a parse error, it will appear here. Checking this before arming avoids arming a mission that is already in `error`.

**Response:**
```json
{
  "status": "loaded",
  "activeFile": "flight.ams",
  "state": "WAIT",
  "error": ""
}
```

---

#### `POST /api/mission/deactivate`

**When to use:** Before switching to a different mission script, or to reset the engine back to `idle` after a test run.

**Why:** Deactivating stops any running execution, clears the loaded script, and forces the system back to `IDLE` mode (solid green LED). It is the safe way to undo an activation without rebooting. Must be called before activating a different script if one is already loaded.

**Response:** AMS engine status showing `status: "idle"`.

---

### Pre-flight: Configuration

---

#### `GET /api/config`

**When to use:** Before modifying configuration, to read the current values in effect.

**Response:**
```json
{
  "telemetryIntervalMs": 500,
  "nodeId": 1,
  "ledBrightness": 80
}
```

---

#### `PUT /api/config`

**When to use:** Adjust telemetry cadence, node ID, or LED brightness before the mission. **Blocked during flight** (`409` if mode is `FLIGHT` or `RECOVERY`).

**Why:** The telemetry interval controls how often the AMS `HK.report` blocks fire over LoRa. Lowering it increases radio traffic and battery use; raising it reduces ground visibility. The node ID identifies this vehicle in multi-node deployments. LED brightness affects power consumption on the pad.

The server validates all fields first and only applies changes if all are valid (validate-then-apply). Partial JSON is supported — omit any field you do not want to change.

**Request (partial example):**
```json
{ "telemetryIntervalMs": 1000 }
```

**Field constraints:**

| Field                 | Type   | Range       | Default |
|-----------------------|--------|-------------|---------|
| `telemetryIntervalMs` | uint32 | 100–60 000  | 500     |
| `nodeId`              | uint8  | 1–253       | 1       |
| `ledBrightness`       | uint8  | 0–255       | 80      |

**Response:** Echoes the full updated config.

---

### Pre-flight: Mode Management

---

#### `POST /api/mode`

**When to use:** Switch operating mode before arming. Useful to enter `test` mode for sensor and actuator validation without committing to flight.

**Why:** The firmware enforces a mode transition matrix. Not all transitions are valid:

| From        | To       | Allowed? |
|-------------|----------|----------|
| `IDLE`      | `test`   | Yes      |
| `IDLE`      | `flight` | Yes      |
| `TEST`      | `idle`   | Yes      |
| `TEST`      | `flight` | Yes      |
| `RECOVERY`  | `idle`   | Yes      |
| `ERROR`     | `idle`   | Yes      |
| `FLIGHT`    | anything | **No** (flight locked) |

**Request:**
```json
{ "mode": "test" }
```

Valid values: `"idle"`, `"test"`, `"flight"`.

**Response:** Full system status (same as `GET /api/status`).

**Errors:** `409` if the transition is not permitted from the current mode.

---

### Launch Sequence

---

#### `POST /api/arm`

**When to use:** Final step on the pad before launch. Only call this when the mission is activated, the rocket is in position, and personnel are clear.

**Why:** This is the irreversible step. Internally, it:
1. Checks the AMS engine is in `LOADED` state (requires a prior successful `POST /api/mission/activate`).
2. Enables engine execution (`LOADED → RUNNING`).
3. Injects the `LAUNCH` telecommand into the AMS engine.
4. Verifies the engine responds `RUNNING` after injection.
5. Sets the system to `FLIGHT` mode (blue LED).

If any of those steps fails — engine not loaded, injection rejected, engine enters error — the system immediately transitions to `ERROR` mode and returns a `409` or `500` describing the failure. This ensures you never leave the system half-armed.

**Request:** No body.

**Response:** Full system status with `"mode": "FLIGHT"` and `"armed": true`.

**Errors:**
- `409` — already armed, or AMS engine is not in `LOADED` state.
- `500` — mission engine unavailable, or `LAUNCH` injection failed internally.

---

### In-flight

WiFi may be disabled automatically when transitioning to `FLIGHT` mode (`WIFI_DISABLE_IN_FLIGHT = true` in `config.h`). If it remains enabled, only monitoring endpoints are available. All configuration and mission structure endpoints are flight-locked.

---

#### `POST /api/abort`

**When to use:** Emergency stop from the ground station while the system is in `FLIGHT` mode.

**Why:** Injects an `ABORT` telecommand into the AMS engine so the script can handle it gracefully (an AMS `transition ... when TC.command == ABORT` fires if defined). If the injection itself fails, the engine is deactivated directly as a fallback. Either way, `armed` is cleared and the mode returns to `IDLE`.

**Request:** No body.

**Errors:** `409` if not in `FLIGHT` mode.

---

#### `POST /api/mission/command`

**When to use:** Inject an arbitrary telecommand into the running AMS engine — either during a test run to drive state transitions manually, or during flight (if WiFi is up) to send named commands that the script is listening for.

**Why:** The AMS engine can transition on `TC.command == <NAME>`. This endpoint is the delivery mechanism. `POST /api/arm` uses this internally to inject `LAUNCH`. You can use it directly for any other command defined in your script.

**Request:**
```json
{ "command": "RECOVER" }
```

Command name max length: 16 characters.

**Response:** `204 No Content` on success. `400` if the command is empty, too long, or not recognized by the engine.

---

### Post-flight: Log Retrieval

---

#### `GET /api/logs`

**When to use:** Immediately after landing and reconnecting over WiFi, to see what log files were produced during the flight.

**Why:** The AMS engine writes CSV logs to LittleFS via `LOG.report` blocks at the cadence defined in the script. Before downloading, you need to know the filenames and their sizes. Returns an array; an empty array `[]` means the log directory does not yet exist (no logs were written).

**Response:**
```json
[
  { "name": "flight_001.csv", "size": 48320 },
  { "name": "flight_002.csv", "size": 12800 }
]
```

---

#### `GET /api/logs/<filename>`

**When to use:** Download a specific flight log to the ground computer for analysis.

**Why:** Log files are CSV with timestamps and the sensor fields specified in the `LOG.report` blocks of the AMS script. This is the primary post-flight data source for IMU axis data, altitude, and any other high-rate sensor you configured. The response is a chunked `text/plain` download with `Content-Disposition: attachment`.

**Flight lock exception:** Downloads are normally blocked during flight (`409`). The exception is when the AMS engine status is `complete` — meaning the script reached a terminal state (a state with no outgoing transitions) and stopped executing. In that case, the lock is lifted and logs are accessible without rebooting.

**Errors:** `404` if the file does not exist, `400` for invalid filename (path traversal check).

---

#### `DELETE /api/logs/<filename>`

**When to use:** Remove a specific log file after you have confirmed a successful download, to free flash space for the next flight.

**Response:** `204 No Content` on success, `404` if not found.

---

#### `DELETE /api/logs`

**When to use:** Wipe all logs at once before a flight if flash space is low, or after a test session when you do not need to keep the data.

**Response:** `204 No Content`.

> **Warning:** There is no undo. Confirm downloads are complete before calling this.

---

## Recommended Sequence per Operation

### Standard pre-flight to launch

```
1. GET  /api/status                      # verify sensors and mode = IDLE
2. POST /api/scans/i2c                   # confirm BMP280 + MPU-6050 addresses
3. POST /api/scans/uart                  # confirm GPS NMEA stream + LoRa ready
4. GET  /api/storage/health              # confirm LittleFS mounted
5. GET  /api/missions                    # list stored scripts
6. PUT  /api/missions/flight.ams         # upload mission script (raw AMS text)
7. GET  /api/missions/flight.ams         # verify stored script matches source
8. GET  /api/config                      # review config
9. PUT  /api/config                      # adjust if needed (e.g. telemetry cadence)
10. POST /api/mission/activate {"file":"flight.ams"}   # parse + load into engine
11. GET  /api/mission                    # confirm status = "loaded", state = "WAIT"
12. POST /api/arm                        # inject LAUNCH, enter FLIGHT mode
```

### After recovery

```
1. GET  /api/status                      # verify mode (RECOVERY or IDLE)
2. POST /api/mode {"mode":"idle"}        # if still in RECOVERY/ERROR, return to IDLE
3. GET  /api/logs                        # list flight logs
4. GET  /api/logs/<filename>             # download each log
5. POST /api/mission/deactivate          # clear engine state
6. DELETE /api/logs/<filename>           # optional cleanup after confirmed download
```

### Bench test run

```
1. GET  /api/status
2. POST /api/mode {"mode":"test"}
3. PUT  /api/missions/test.ams
4. POST /api/mission/activate {"file":"test.ams"}
5. POST /api/mission/command {"command":"LAUNCH"}   # drive transitions manually
6. GET  /api/mission                     # watch state progression
7. POST /api/mission/deactivate
8. POST /api/mode {"mode":"idle"}
```

---

## Error Reference

| Code | Meaning |
|------|---------|
| `400` | Bad request — invalid JSON, missing required field, filename with illegal characters, value out of range. |
| `404` | Route or file not found. |
| `405` | Wrong HTTP method for this route. |
| `409` | Conflict — flight lock active, invalid mode transition, engine not in expected state, or already armed. |
| `413` | Payload too large — script exceeds `AMS_MAX_SCRIPT_BYTES`. |
| `500` | Internal error — storage failure, mutex timeout, mission engine not initialized. |

# ARES WiFi REST API

**Ground configuration and monitoring over WiFi AP**

Source: [src/net/wifi_api.h](../src/net/wifi_api.h),
        [src/net/wifi_api.cpp](../src/net/wifi_api.cpp)

---

## Connection

| Parameter   | Value                                        |
|-------------|----------------------------------------------|
| SSID        | `ARES-XXXX` (last 2 bytes of MAC)            |
| Password    | `ares1234`                                   |
| IP          | `192.168.4.1` (ESP32 softAP default)         |
| Port        | `80`                                         |
| Protocol    | HTTP/1.1                                     |
| Max clients | 4                                            |
| TX Power    | 8 dBm (configurable in config.h)             |
| Channel     | 1                                            |

All responses are `application/json` with CORS headers
(`Access-Control-Allow-Origin: *`).

---

## Endpoints

### GET /api/status

System status and flight information.

**Response** `200 OK`:

```json
{
  "board": "ESP32-S3 Zero Mini",
  "version": "0.1.0",
  "uptimeMs": 123456,
  "freeHeap": 245760,
  "mode": "IDLE",
  "flightPhase": "PAD",
  "testPhase": "NONE",
  "armed": false,
  "inFlight": false,
  "maxAltitude": 0.0,
  "maxSpeed": 0.0,
  "flightTimeMs": 0,
  "wifiClients": 1,
  "txActive": false,
  "txCount": 0,
  "health": {
    "led": true,
    "lora": true,
    "wifi": true,
    "state": true,
    "storage": true,
    "baro": true,
    "gps": true,
    "imu": true
  },
  "baro": {
    "pressurePa": 101325.0,
    "temperatureC": 22.5,
    "altitudeM": 0.42,
    "timestampMs": 123400
  },
  "gps": {
    "fix": true,
    "latitude": 40.416775,
    "longitude": -3.703790,
    "altitudeM": 650.2,
    "speedKmh": 0.0,
    "courseDeg": 180.0,
    "hdop": 1.2,
    "satellites": 8,
    "timestampMs": 123400
  },
  "imu": {
    "accelX": 0.12,
    "accelY": -0.03,
    "accelZ": 9.81,
    "gyroX": 0.8,
    "gyroY": -0.2,
    "gyroZ": 0.1,
    "tempC": 31.4
  }
}
```

The `baro` object is only present when `health.baro` is `true`.  
The `gps` object is only present when `health.gps` is `true`.
The `imu` object is only present when `health.imu` is `true`.

---

### GET /api/imu

Dedicated IMU snapshot endpoint.

**Response** `200 OK`:

```json
{
  "ok": true,
  "status": "ok",
  "timestampMs": 123456,
  "accelX": 0.12,
  "accelY": -0.03,
  "accelZ": 9.81,
  "gyroX": 0.8,
  "gyroY": -0.2,
  "gyroZ": 0.1,
  "tempC": 31.4
}
```

If `ok` is `false`, the payload includes `status` and `timestampMs` only.

---

### GET /api/imu/health

Lightweight IMU health endpoint.

**Response** `200 OK`:

```json
{
  "ok": true,
  "status": "ok",
  "timestampMs": 123456
}
```

---

### GET /api/config

Current runtime configuration.

**Response** `200 OK`:

```json
{
  "telemetryIntervalMs": 500,
  "nodeId": 1,
  "ledBrightness": 80
}
```

---

### PUT /api/config

Update runtime configuration (partial JSON — only include fields to change).

**Request body:**

```json
{
  "telemetryIntervalMs": 1000,
  "nodeId": 5,
  "ledBrightness": 128
}
```

**Field constraints:**

| Field                | Type   | Range      | Default |
|----------------------|--------|------------|---------|
| `telemetryIntervalMs`| uint32 | 100–60000  | 500     |
| `nodeId`             | uint8  | 1–253      | 1       |
| `ledBrightness`      | uint8  | 0–255      | 80      |

**Responses:**

| Code | Condition                              |
|------|----------------------------------------|
| 200  | Config updated — echoes GET /api/config|
| 400  | Invalid JSON or field out of range      |
| 409  | Config locked during FLIGHT mode        |

**Thread safety:** Validated values are applied under mutex (`cfgMtx_`)
with a 50 ms timeout (RTOS-4).

---

### POST /api/mode

Change operating mode.

**Request body:**

```json
{
  "mode": "idle|test|flight",
  "test": "radio_ping"
}
```

The `test` field is only required when `mode` is `"test"`.

**Mode transition rules:**

| Target   | Required Current Mode | Notes                     |
|----------|-----------------------|---------------------------|
| `idle`   | TEST, RECOVERY, ERROR | Cannot idle during FLIGHT |
| `test`   | IDLE                  | Starts the named test     |
| `flight` | IDLE or TEST          | Enters PAD phase          |

**Available test types:** `radio_ping`

**Responses:**

| Code | Condition                          |
|------|------------------------------------|
| 200  | Mode changed — echoes GET /api/status |
| 400  | Unknown mode or test type           |
| 409  | Invalid transition from current mode|

---

### POST /api/arm

Arm the flight FSM (required before PAD → BOOST).

**Request:** No body required.

**Responses:**

| Code | Condition                  |
|------|----------------------------|
| 200  | Armed — echoes GET /api/status |
| 409  | Not in FLIGHT mode or already armed |

---

### POST /api/abort

Abort the current flight.

**Request:** No body required.

**Responses:**

| Code | Condition                   |
|------|-----------------------------|
| 200  | Aborted — echoes GET /api/status |
| 409  | Not in FLIGHT mode           |

---

## Error Responses

All errors follow the format:

```json
{
  "error": "descriptive message"
}
```

---

## CORS

All responses include:

```
Access-Control-Allow-Origin: *
Access-Control-Allow-Methods: GET, POST, PUT, OPTIONS
Access-Control-Allow-Headers: Content-Type
```

`OPTIONS` requests return `204 No Content` for preflight.

---

## Examples (curl)

```bash
# Base URL
API="http://192.168.4.1"

# Status
curl $API/api/status

# Read config
curl $API/api/config

# Update telemetry interval
curl -X PUT $API/api/config \
  -H "Content-Type: application/json" \
  -d '{"telemetryIntervalMs": 1000}'

# Enter test mode
curl -X POST $API/api/mode \
  -H "Content-Type: application/json" \
  -d '{"mode": "test", "test": "radio_ping"}'

# Enter flight mode
curl -X POST $API/api/mode \
  -H "Content-Type: application/json" \
  -d '{"mode": "flight"}'

# Arm
curl -X POST $API/api/arm

# Abort
curl -X POST $API/api/abort

# Return to idle
curl -X POST $API/api/mode \
  -H "Content-Type: application/json" \
  -d '{"mode": "idle"}'
```

---

## RTOS Integration

The WiFi API runs as a dedicated FreeRTOS task:

| Property   | Value                       |
|------------|-----------------------------|
| Task name  | `api`                       |
| Priority   | 1 (Low)                     |
| Stack      | 4096 bytes                  |
| Core       | 1                           |
| Poll rate  | 50 ms (`API_RATE_MS`)       |

The API is initialised **before** LoRa in `setup()` so it is always
accessible for diagnostics, even if other subsystems fail.

# AMS Programming Guide

This guide explains how to write production-ready AMS mission scripts.
It is practical and workflow-oriented.

Normative behavior is defined in:
- docs/standards/ams_standard.md

---

## 1. Quick Start

Minimal valid script:

```ams
include GPS as BN220
include BARO as BMP280
include COM as LORA

pus.apid = 1
pus.service 3 as HK
pus.service 5 as EVENT
pus.service 1 as TC

state WAIT:
  on_enter:
    EVENT.info "WAITING"
  transition to FLIGHT when TC.command == LAUNCH

state FLIGHT:
  on_enter:
    EVENT.info "LIFTOFF"
  every 1000ms:
    HK.report {
      gps_alt: GPS.alt
      baro_alt: BARO.alt
      temp: BARO.temp
    }
```

---

## 2. AMS Building Blocks

### 2.1 State

Use one `state` block per flight phase.

```ams
state DESCENT:
```

### 2.2 on_enter Event

Use `on_enter` to publish one event when state changes.

```ams
on_enter:
  EVENT.info "DESCENDING"
```

### 2.3 Housekeeping Telemetry (Radio)

Use `every` + `HK.report` for PUS telemetry over radio.

```ams
every 1000ms:
  HK.report {
    gps_alt: GPS.alt
    baro_alt: BARO.alt
  }
```

### 2.4 Local Sensor Logging (File)

Use `log_every` + `LOG.report` for local text logs.

```ams
log_every 200ms:
  LOG.report {
    gps_alt: GPS.alt
    baro_alt: BARO.alt
    pressure: BARO.pressure
  }

### 2.5 conditions + on_error

Use `conditions:` to define guard checks that must stay true while the state is active.
Use `on_error:` to emit an event if any guard fails.

```ams
state FLIGHT:
  conditions:
    BARO.temp > -40
    BARO.temp < 85
    GPS.speed < 250
  on_error:
    EVENT.error "FLIGHT guard violated"
```

Current profile notes:
- Up to `ares::AMS_MAX_CONDITIONS` guards per state
- `on_error:` currently supports only `EVENT.*`
- `TC.command` is not valid inside `conditions:`
```

---

## 3. Independent Cadence Strategy

Common pattern:
- HK every `1000ms` (lower bandwidth, radio-safe)
- LOG every `200ms` (higher local resolution)

```ams
every 1000ms:
  HK.report { ... }

log_every 200ms:
  LOG.report { ... }
```

Recommendation:
- Keep LOG faster than HK during dynamic phases (ascent/descent)
- Reduce both rates during LANDED to save power and storage

---

## 4. Priority and Budget Tuning

AMS arbitrates EVENT, HK, and LOG per tick.

```ams
priorities event=4 hk=3 log=1 budget=2
```

Meaning:
- Higher number = higher prioritya
- `budget` = max actions executed per tick

Recommended profile (PUS-first):
- `event=4`
- `hk=3`
- `log=1`
- `budget=2`

When to increase budget to 3:
- Lab/testing runs where latency is more important than CPU margin

When to keep budget at 1 or 2:
- Flight-critical conservative runtime with tighter timing bounds

---

## 5. Transition Design

### 5.1 Telecommand-driven

```ams
transition to FLIGHT when TC.command == LAUNCH
```

### 5.2 Sensor-driven

```ams
transition to DESCENT when BARO.alt < 50
transition to LANDED when GPS.speed < 1
```

Use sensor transitions for autonomy and TC transitions for operator gates.

### 5.3 Guard Conditions (in-state safety)

Transitions move the FSM to the next phase.
Guards (`conditions:`) enforce safety invariants while remaining in the same phase.

```ams
state TEST:
  conditions:
    BARO.temp > -40
    BARO.temp < 85
    TIME.elapsed > 30000
  on_error:
    EVENT.error "TEST guard violated"
```

Runtime behavior:
- Transition checks run first
- Guard checks run next
- If any guard fails, AMS emits `on_error` event (if defined) and enters `ERROR`
- Periodic actions (EVENT/HK/LOG arbitration) run only when guards pass

---

## 6. Supported Sensor Expressions

Valid expressions for HK.report and LOG.report:
- `GPS.lat`
- `GPS.lon`
- `GPS.alt`
- `GPS.speed` — ground speed (km/h)
- `BARO.alt`
- `BARO.temp`
- `BARO.pressure`
- `IMU.accel_x` — X-axis acceleration (m/s²)
- `IMU.accel_y` — Y-axis acceleration (m/s²)
- `IMU.accel_z` — Z-axis acceleration (m/s²)
- `IMU.accel_mag` — Acceleration magnitude √(x²+y²+z²) (m/s²); also written to the TelemetryPayload HK frame
- `IMU.gyro_x` — X angular rate (deg/s)
- `IMU.gyro_y` — Y angular rate (deg/s)
- `IMU.gyro_z` — Z angular rate (deg/s)
- `IMU.temp` — On-chip temperature (°C)

IMU transitions support `IMU.accel_x`, `IMU.accel_y`, `IMU.accel_z`, and `IMU.accel_mag`.
If no MPU-6050 is connected at runtime, all IMU values read as 0 and transitions evaluate false.

`TIME.elapsed` is available in `conditions:` and `transition` guards only (not in `HK.report`/`LOG.report`):
- `TIME.elapsed > <ms>` — true when the active state has been running for more than the given milliseconds

```ams
state HOLD:
  every 1000ms:
    HK.report { baro_alt: BARO.alt }
  conditions:
    TIME.elapsed > 10000
  on_error:
    EVENT.error "HOLD timeout exceeded"
  transition to DESCENT when BARO.alt < 50
```

**Example — launch detect via IMU:**

```ams
state PAD:
  every 500ms:
    HK.report { accel_mag: IMU.accel_mag }
  log_every 200ms:
    LOG.report { ax: IMU.accel_x  ay: IMU.accel_y  az: IMU.accel_z
                 gx: IMU.gyro_x  gy: IMU.gyro_y  gz: IMU.gyro_z
                 imu_t: IMU.temp }
  transition to FLIGHT when IMU.accel_mag > 30.0
  conditions:
    IMU.accel_mag < 200
  on_error:
    EVENT.error "IMU overload on PAD"
```

---

## 7. Full Example (Flight + Descent + Landed)

```ams
include GPS as BN220
include BARO as BMP280
include COM as LORA

pus.apid = 1
pus.service 3 as HK
pus.service 5 as EVENT
pus.service 1 as TC

state WAIT:
  on_enter:
    EVENT.info "WAITING FOR LAUNCH"
  priorities event=4 hk=3 log=1 budget=2
  transition to FLIGHT when TC.command == LAUNCH

state FLIGHT:
  on_enter:
    EVENT.info "LIFTOFF"
  priorities event=4 hk=3 log=1 budget=2
  every 1000ms:
    HK.report {
      gps_alt: GPS.alt
      baro_alt: BARO.alt
      temp: BARO.temp
    }
  log_every 200ms:
    LOG.report {
      gps_alt: GPS.alt
      baro_alt: BARO.alt
      temp: BARO.temp
      pressure: BARO.pressure
    }
  transition to DESCENT when BARO.alt < 50

state DESCENT:
  on_enter:
    EVENT.info "DESCENDING"
  priorities event=4 hk=3 log=1 budget=2
  every 2000ms:
    HK.report {
      gps_lat: GPS.lat
      gps_lon: GPS.lon
      gps_alt: GPS.alt
      baro_alt: BARO.alt
      temp: BARO.temp
      pressure: BARO.pressure
    }
  log_every 500ms:
    LOG.report {
      gps_lat: GPS.lat
      gps_lon: GPS.lon
      gps_alt: GPS.alt
      baro_alt: BARO.alt
      pressure: BARO.pressure
    }
  transition to LANDED when GPS.speed < 1

state LANDED:
  on_enter:
    EVENT.info "LANDED"
  priorities event=4 hk=2 log=1 budget=2
  every 10000ms:
    HK.report {
      gps_lat: GPS.lat
      gps_lon: GPS.lon
      baro_alt: BARO.alt
      temp: BARO.temp
      pressure: BARO.pressure
    }
  log_every 2000ms:
    LOG.report {
      gps_lat: GPS.lat
      gps_lon: GPS.lon
      baro_alt: BARO.alt
      temp: BARO.temp
    }
```

---

## 8. Upload, Activate, and Run

Typical API flow:
1. Upload script (raw `.ams` body, no JSON wrapper)
  - `PUT /api/missions/<name>.ams`
2. List available scripts
  - `GET /api/missions`
3. Activate script
   - `POST /api/mission/activate {"file":"<name>.ams"}`
4. Set flight mode
   - `POST /api/mode {"mode":"flight"}`
5. Arm system (enables AMS execution and injects LAUNCH)
   - `POST /api/arm`

Abort flow:
- `POST /api/mission/deactivate` stops execution and deactivates the active mission
- `POST /api/mission/command {"command":"..."}` injects an arbitrary TC command

---

## 9. Troubleshooting

### Script parse fails

Check:
- State names and `transition to` target names
- Missing `every` before `HK.report`
- Missing `log_every` before `LOG.report`
- Invalid `priorities` values
- Invalid `conditions:` syntax or unsupported guard expression
- `TC.command` used inside `conditions:` (not supported)
- Unsupported statements inside `on_error:` (only `EVENT.*`)

### Mission stays in WAIT

Check:
- Mission is activated
- Execution is enabled by arming flow
- TC command token was injected (`LAUNCH`) or transition condition is reachable

### Mission enters ERROR unexpectedly

Check:
- Active state's `conditions:` are realistic for current sensors/environment
- Threshold units are correct (e.g. `GPS.speed` in km/h, `TIME.elapsed` in ms)
- `on_error` message and AMS snapshot `error` field for root cause

### Log file not useful

Check:
- `LOG.report` block exists in target state
- `log_every` is configured
- Selected fields match supported expressions

---

## 10. Authoring Best Practices

- Keep state names short and explicit (`WAIT`, `FLIGHT`, `DESCENT`, `LANDED`)
- Use event messages that are operationally meaningful
- Start with conservative rates and tighten after test evidence
- Prefer barometer for short-term phase transitions, GPS for geo context
- Keep event priority highest in flight-critical phases
- Keep local LOG below HK priority in nominal PUS-oriented operation

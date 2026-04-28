# AMS Programming Guide

This guide explains how to write production-ready AMS mission scripts.
It is practical and workflow-oriented.

Normative behavior is defined in:
- docs/standards/ams_standard.md

---

## 1. Quick Start

Minimal valid script:

```ams
include BN220 as GPS
include BMP280 as BARO
include LORA as COM

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
```

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

### 5.3 Sensor-driven with Persistence (`for <N>ms`)

Sensor readings can contain momentary noise that would cause a transition to
fire prematurely.  Add `for <N>ms` to require the condition to hold
continuously for a minimum window before the transition triggers.

```ams
transition to APOGEE when BARO.alt > 3000 for 500ms
```

In this example the engine only transitions to `APOGEE` after altitude stays
above 3000 m for at least 500 ms without interruption.  If the reading drops
below the threshold at any point during the window, the timer resets.

Rules:
- `for Nms` is optional.  Omitting it preserves the existing single-sample behaviour.
- The minimum value is `1 ms`.  `0` is rejected at parse time.
- The modifier applies only to sensor (`BARO.*`, `GPS.*`, `IMU.*`) and
  `TIME.elapsed` transitions.  It has no effect on `TC.command` transitions.

Recommended values:
- Short bursts of noise (barometer jitter): `for 200ms` to `for 500ms`
- Stable phase detection (apogee, landing): `for 500ms` to `for 2000ms`

### 5.4 Delta / Trend Operators

Sometimes you need to detect that a sensor is **going up or down** rather than
comparing it against a fixed absolute threshold.  The `delta` operator, and its
syntactic shortcuts `falling` / `rising`, test the **inter-sample change**
computed between two consecutive ticks.

#### 5.4.1 Syntax

```ams
ALIAS.field delta < VALUE     // fires when (current - previous) < VALUE
ALIAS.field delta > VALUE     // fires when (current - previous) > VALUE
ALIAS.field falling           // sugar for: delta < 0  (decreasing)
ALIAS.field rising            // sugar for: delta > 0  (increasing)
```

#### 5.4.2 Typical Use-cases

**Detect apogee without knowing the deployment altitude:**

```ams
transition to DESCENT when BARO.alt falling for 500ms
```

The rocket crosses apogee and the barometer starts decreasing — no hard-coded
altitude constant is needed.  The `for 500ms` window prevents a transient noise
spike from triggering early.

**Detect significant altitude drop (dual-sensor redundancy):**

```ams
transition to DESCENT when BARO.alt delta < -2 or GPS.alt delta < -2
```

Fires when either barometer or GPS registers a drop of more than 2 m in one
tick.  If either sensor fails, the other still covers the transition.

**Detect rapid acceleration at launch:**

```ams
transition to POWERED_FLIGHT when IMU.accel_mag delta > 5
```

Fires when the acceleration magnitude increases by more than 5 m/s² in one
sample period.

#### 5.4.3 Behaviour Notes

- **First tick in a state** always evaluates to false — there is no previous
  sample after a state entry, so no spurious transition can occur.
- **Failed sensor read**: the condition evaluates to false and the previous
  value baseline is preserved for the next tick.
- **Combined with `for Nms`**: the compound delta condition must remain true on
  every consecutive tick for the full hold window.
- **`falling` / `rising`**: these are exact aliases for `delta < 0` and
  `delta > 0`; they accept the same field names as any other sensor condition.

### 5.5 Compound Conditions (`or` / `and`)

Multiple conditions can be joined in a single transition using `or` or `and`.
This allows sensor-fusion logic and redundancy without additional states.

```ams
transition to DESCENT when BARO.alt falling or GPS.alt falling
transition to SAFE when BARO.alt < 10 and GPS.speed < 2
```

Rules:
- Logic must be **homogeneous**: you cannot mix `or` and `and` in one
  `transition` line.  Create a helper state if you need mixed logic.
- Maximum **4 sub-conditions** per transition (`AMS_MAX_TRANSITION_CONDS`).
- `TC.command` may be combined with other conditions; the token is consumed
  only when the overall compound evaluates to true and the transition fires.
- `for Nms` applies to the **compound result** — all sub-conditions must remain
  satisfied simultaneously throughout the hold window (AND logic), or at least
  one must remain satisfied (OR logic).

**Example — multi-sensor apogee:**

```ams
state COAST:
  every 500ms:
    HK.report { baro_alt: BARO.alt, gps_alt: GPS.alt }
  transition to APOGEE when BARO.alt falling or GPS.alt falling for 500ms
```

**Example — dual-gate operator abort:**

```ams
state FLIGHT:
  transition to ABORT when TC.command == ABORT or IMU.accel_mag > 350
```

### 5.6 Guard Conditions (in-state safety)

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

## 5.7 Global Variables and Calibration (AMS-4.8)

Global variables allow a mission script to capture sensor readings at runtime
and use them as dynamic thresholds in subsequent transitions or guard conditions.

### Declaration

Declare variables in the metadata section (before any `state` block):

```ams
apid 0x01
node_id 1
var ground_alt = 0.0
var apogee_alt = 0.0
```

The initial value is a fallback only — variables are marked **invalid** until a
`set` action fires.  Conditions that reference invalid variables evaluate to
`false` (they cannot fire), which protects against spurious transitions during
early states before calibration has occurred.

### Set actions

Assign a sensor reading inside an `on_enter:` block:

```ams
state PAD:
  on_enter:
    EVENT.info "Capturing ground altitude"
    set ground_alt = CALIBRATE(BARO.alt, 5)
```

`CALIBRATE(ALIAS.field, N)` reads the sensor N times (1–10) and stores the
arithmetic mean of valid readings.  Use `set ground_alt = BARO.alt` for a
single instantaneous read.

### Using variables as thresholds

Reference a variable on the right-hand side of a transition or guard condition:

```ams
state DESCENT:
  transition to LANDED when BARO.alt < ground_alt
  transition to LANDED when BARO.alt < (ground_alt + 10)
```

Parentheses with an additive or subtractive offset are supported:
`(varname + 50)`, `(varname - 20)`.

### Full example: apogee-relative landing detection

```ams
apid 0x01
node_id 1

var ground_alt = 0.0

state PAD:
  on_enter:
    EVENT.info "PAD: calibrating ground alt"
    set ground_alt = CALIBRATE(BARO.alt, 10)
  transition to BOOST when IMU.accel_mag > 30

state BOOST:
  on_enter:
    EVENT.info "BOOST: engine firing"
  transition to COAST when IMU.accel_mag < 5

state COAST:
  on_enter:
    EVENT.info "COAST: coasting"
  transition to DESCENT when falling BARO.alt > 2

state DESCENT:
  on_enter:
    EVENT.info "DESCENT: descending"
  transition to LANDED when BARO.alt < (ground_alt + 15)

state LANDED:
  on_enter:
    EVENT.info "LANDED: mission complete"
  every 5000ms:
    HK.report { baro_alt: BARO.alt }
```

### NaN guard rules

If a sensor read fails during a `set` or `CALIBRATE` action:
1. The variable retains its current value (or remains invalid if never set).
2. An `EVENT.warning` is queued (no ERROR state for a failed `set`).
3. Conditions referencing a still-invalid variable evaluate to `false` —
   safe default for early mission states before calibration completes.

### Checkpoint persistence

Variables are saved to the AMS checkpoint file (v2 format) on every periodic
save cycle.  On firmware reboot mid-mission, variables are restored along with
the state index so relative-altitude logic continues correctly.

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
If no MPU-6050 is connected (or read fails) at runtime:
- Transition conditions using IMU fields evaluate as false (no transition trigger).
- Guard conditions in `conditions:` treat missing sensor data as "holds" (no immediate ERROR).
- LOG.report writes `nan` for unreadable IMU fields to preserve CSV column alignment.
- HK.report only maps supported payload fields (for IMU, `accel_mag` when available).

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
include BN220 as GPS
include BMP280 as BARO
include LORA as COM

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

Resume flow after reboot:
1. On boot, AMS attempts to restore `/missions/.ams_resume.chk`.
2. Restore is accepted only for a valid in-flight snapshot (`running=1`, `executionEnabled=1`, `status=RUNNING`).
3. If accepted, mission state index and elapsed timers are recovered, and execution continues automatically.
4. Main startup mirrors this in API mode by setting FLIGHT when restored status is RUNNING.
5. If checkpoint is invalid/corrupt/stale, it is discarded and AMS remains non-running.

Checkpoint cadence details:
- Forced checkpoint when entering a new state.
- Forced checkpoint when execution is enabled/disabled.
- Periodic checkpoint while RUNNING every `AMS_CHECKPOINT_INTERVAL_MS`.
- Checkpoint is removed on mission completion (`COMPLETE`) and on explicit deactivation.

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

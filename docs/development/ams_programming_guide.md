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

A single `every` block per state is sufficient for one cadence:

```ams
every 1000ms:
  HK.report {
    gps_alt: GPS.alt
    baro_alt: BARO.alt
  }
```

**Multiple `every` blocks per state (AMS-4.3.1):** Up to `AMS_MAX_HK_SLOTS` (currently 4) independent
cadences can be declared in the same state. Each block generates its own independent PUS-3 HK frame:

```ams
state FLIGHT:
  every 1000ms:
    HK.report { gps_lat: GPS.lat  gps_lon: GPS.lon  gps_alt: GPS.alt }
  every 50ms:
    HK.report { ax: IMU.accel_x  ay: IMU.accel_y  az: IMU.accel_z }
```

Each slot fires at its own cadence independently. The minimum interval is `TELEMETRY_INTERVAL_MIN` (100 ms).

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

**Multiple `log_every` blocks per state (AMS-4.3.1):** Similarly, up to `AMS_MAX_HK_SLOTS` (4) log cadences
can coexist per state. Each slot appends its own rows to the same CSV file. The CSV includes a `slot` column
(0-based index) to distinguish cadences in post-flight analysis:

```ams
state FLIGHT:
  log_every 1000ms:
    LOG.report { gps_lat: GPS.lat  gps_lon: GPS.lon }
  log_every 10ms:
    LOG.report { ax: IMU.accel_x  ay: IMU.accel_y  az: IMU.accel_z }
```

CSV row format: `t_ms, state, slot, <fields...>`

The slot header (`t_ms,state,slot,...`) is written once per slot on first use.

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

Common pattern — one HK cadence over radio, one LOG cadence to file:
- HK every `1000ms` (lower bandwidth, radio-safe)
- LOG every `200ms` (higher local resolution)

```ams
every 1000ms:
  HK.report { ... }

log_every 200ms:
  LOG.report { ... }
```

For scenarios requiring multiple independent sensor cadences within the same state,
use multiple `every` / `log_every` blocks (AMS-4.3.1):

```ams
state FLIGHT:
  every 1000ms:
    HK.report { gps_lat: GPS.lat  gps_lon: GPS.lon  baro_alt: BARO.alt }
  every 100ms:
    HK.report { ax: IMU.accel_x  ay: IMU.accel_y  az: IMU.accel_z }
  log_every 1000ms:
    LOG.report { gps_lat: GPS.lat  gps_lon: GPS.lon }
  log_every 10ms:
    LOG.report { ax: IMU.accel_x  ay: IMU.accel_y  az: IMU.accel_z }
```

Constraints:
- Max `AMS_MAX_HK_SLOTS = 4` blocks of each type per state
- Each slot is independent — its timer resets when the state is activated
- Budget arbitration (APUS-19.2) still applies: if multiple slots and tasks are due
  in the same tick, the engine dispatches up to `actionBudget` actions in priority order

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
If no IMU is connected (or if the active IMU read fails) at runtime:
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

---

## 11. Fault Tolerance Features (AMS-4.9 / AMS-4.10)

### 11.1 Sensor Retry (`retry=N`) — AMS-4.9.1

Add `retry=N` to an `include` directive to enable automatic re-attempts when a
sensor read fails.  The engine retries up to `N` extra times (1 ≤ N ≤ 5) before
treating the read as definitively failed.

Optional `timeout=Nms` documents the per-attempt deadline but is not enforced by
the engine beyond the underlying I2C hardware timeout.

```ams
include BMP280 as BARO retry=3 timeout=500ms
include BN220 as GPS  retry=2
```

When retries are exhausted the engine falls back to the normal failure path for
that sensor reading (variable remains unchanged, EVENT.warn is emitted).

### 11.2 Fallback Transition — AMS-4.9.2

A *fallback transition* fires unconditionally if no regular transition condition
has been satisfied within a configurable time after entering the state.  Use it
to prevent permanent traps when a sensor stops responding.

```ams
state FLIGHT:
  transition to APOGEE when BARO.alt > 500
  fallback transition to SAFE after 300000ms
```

Only one fallback per state is allowed.  The `after Nms` clause is mandatory.
The fallback fires *before* terminal-state detection, so a state with a fallback
but no regular transition is kept alive until the timeout expires.

### 11.3 Error Recovery Transition (`on_error: transition to`) — AMS-4.10.2

By default a guard-condition violation halts the engine in the `ERROR` state.
Add a `transition to STATE` directive inside an `on_error:` block to enter a
recovery state instead:

```ams
state FLIGHT:
  conditions:
    BARO.alt > 0
  on_error:
    EVENT.error "BARO out of range"
    transition to SAFE_RECOVERY
```

Both `EVENT.*` and `transition to` can coexist inside a single `on_error:` block.
Only one recovery transition per state is allowed.

---

## 12. Derived Variables (AMS-4.8.3–4.8.5)

Beyond the basic snapshot and `CALIBRATE` forms, `set` actions support three
derived-value operators that update every time the state is *entered*:

### 12.1 Delta (`ALIAS.field delta`) — AMS-4.8.3

Stores the difference between the current and the previous sensor reading.
On the first entry the delta is 0 and the baseline is primed with the current
reading.

```ams
state DESCENT:
  on_enter:
    set descent_rate = BARO.alt delta
```

### 12.2 Running Maximum (`max`) — AMS-4.8.4

Stores the greatest sensor value seen since first entry.  On first entry the
variable is initialised with the current reading.

```ams
state FLIGHT:
  on_enter:
    set max_alt = max(max_alt, BARO.alt)
```

The first argument **must** match the target variable name (self-referential).

### 12.3 Running Minimum (`min`) — AMS-4.8.5

```ams
state FLIGHT:
  on_enter:
    set min_speed = min(min_speed, GPS.speed)
```

All three operators require the target variable to be declared with a `var`
directive before the first state block.

### 12.4 Complete Fault-Tolerance Example

```ams
include BMP280 as BARO retry=3
include BN220 as GPS  retry=2

pus.apid = 1
pus.service 5 as EVENT

var max_alt
var descent_rate

state FLIGHT:
  conditions:
    BARO.alt > 0
  on_enter:
    EVENT.info "FLIGHT"
    set max_alt = max(max_alt, BARO.alt)
  on_error:
    EVENT.error "Baro failed"
    transition to RECOVERY
  transition to APOGEE when BARO.alt > 500
  fallback transition to RECOVERY after 120000ms

state APOGEE:
  on_enter:
    EVENT.info "APOGEE"
    set descent_rate = BARO.alt delta
  transition to DESCENT when BARO.alt < 480

state DESCENT:
  on_enter:
    EVENT.info "DESCENT"
  transition to LANDED when GPS.speed < 1

state RECOVERY:
  on_enter:
    EVENT.warn "RECOVERY"
  transition to LANDED when GPS.speed < 1

state LANDED:
  on_enter:
    EVENT.info "LANDED"
```

---

## 13. TC Debounce (AMS-4.11)

LoRa and other radio links can deliver duplicate telecommand packets.  Without
debounce, a single retransmitted `LAUNCH` TC would fire a transition immediately
on the second copy. AMS-4.11 provides two debounce modifiers for `TC.command`
conditions.

### 13.1 `once` � Explicit One-Shot (AMS-4.11.1)

```ams
transition to FLIGHT when TC.command == LAUNCH once
```

`once` is a self-documenting alias for the default behavior.  The transition
fires on the **first** `LAUNCH` injection, then the TC token is consumed.
Semantically identical to `transition to FLIGHT when TC.command == LAUNCH`.

### 13.2 `confirm N` � Multi-Injection Gate (AMS-4.11.2)

```ams
transition to FLIGHT when TC.command == LAUNCH confirm 2
transition to ABORT  when TC.command == ABORT  confirm 3
```

`confirm N` counts how many times the specified TC has been injected since the
last state entry (or since the last transition fired).  The condition becomes
true only once the count reaches N.  Valid range: **2�10**.

Counter resets:
- On every `enterStateLocked()` call (new state = clean slate).
- When a `CONFIRM` transition fires.

### 13.3 Combined with other conditions

```ams
transition to FLIGHT when TC.command == LAUNCH confirm 2 and BARO.alt > 100
```

`confirm` is parsed per-condition, independently of compound `and`/`or` logic.

### 13.4 Constraints

| Modifier  | Min N | Max N | Token consumed after fire? |
|-----------|-------|-------|---------------------------|
| (default) | �     | �     | Yes (first match)         |
| `once`    | �     | �     | Yes (first match)         |
| `confirm N` | 2   | 10    | Yes (counter reset)       |

---

## 14. Mission Constants (AMS-4.12)

Named constants let you define mission-specific thresholds once at the top
of a script and reference them by name in all transitions.

### 14.1 Declaration syntax

```ams
const APOGEE_THRESHOLD = 3000
const LANDING_SPEED    = 1
const MAX_ACCEL_SAFE   = 50.0
```

- Must appear in the **metadata section** (before the first `state` block).
- Name rules: letters, digits and `_`; no dots; not `TIME` or `TC`.
- Value must be a numeric literal (integer or float).
- At most `AMS_MAX_CONSTS` (8) constants per script.
- Constants are **immutable** � there is no runtime mutation.

### 14.2 Usage in transitions

```ams
state ASCENT:
  transition to DESCENT when BARO.alt > APOGEE_THRESHOLD

state DESCENT:
  transition to LANDED  when GPS.speed < LANDING_SPEED
```

Constants are resolved and inlined into the condition threshold at parse time.
The runtime has zero overhead compared to a literal float value.

### 14.3 Offset forms

Constants support the same variable-offset syntax as `var` references:

```ams
transition to WARN when BARO.alt > (APOGEE_THRESHOLD - 200)
```

### 14.4 Complete example

```ams
apid 0x01

include BMP280 as BARO
include BN220  as GPS

const APOGEE_THRESHOLD = 3000
const LANDING_SPEED    = 1.0
const MIN_LAUNCH_ALT   = 100

state PAD:
  transition to FLIGHT when TC.command == LAUNCH confirm 2 and BARO.alt > MIN_LAUNCH_ALT

state FLIGHT:
  transition to DESCENT when BARO.alt > APOGEE_THRESHOLD

state DESCENT:
  transition to LANDED when GPS.speed < LANDING_SPEED

state LANDED:
  on_enter:
    EVENT.info "Landed"
```

---

## 15. Radio Configuration (AMS-4.13)

The `radio.config` directive lets a script override the compile-time default values of
radio and monitoring parameters **at arm time**, before the first telemetry tick.
The ground station can still refine these values later by sending a
`SET_CONFIG_PARAM` telecommand (APUS-16.1).

### 15.1 Syntax

```ams
radio.config PARAM_NAME = VALUE
```

- Must appear **at the top level** of the script (outside any `state`, `task`, or
  `assert` block).
- Each `radio.config` line sets one parameter.
- Multiple lines are allowed; each must use a different `PARAM_NAME`.

### 15.2 Supported parameters

| `PARAM_NAME`         | Description                                    | Min      | Max    | Unit  |
|----------------------|------------------------------------------------|----------|--------|-------|
| `telem_interval`     | Telemetry report period                        | 100      | 60000  | ms    |
| `monitor.alt.high`   | Altitude high-alarm threshold (ST\[12\])       | 0        | 15000  | m     |
| `monitor.alt.low`    | Altitude low-alarm threshold (ST\[12\])        | −500     | 1000   | m     |
| `monitor.accel.max`  | Acceleration magnitude alarm threshold (ST\[12\]) | 0     | 1000   | m/s²  |
| `monitor.temp.high`  | Temperature high-alarm threshold (ST\[12\])    | −40      | 150    | °C    |
| `monitor.temp.low`   | Temperature low-alarm threshold (ST\[12\])     | −100     | 50     | °C    |

### 15.3 Override priority

```
Compile-time defaults  ←  radio.config (at arm)  ←  SET_CONFIG_PARAM (ground, runtime)
```

A `radio.config` value **replaces** the compile-time default.  A subsequent
`SET_CONFIG_PARAM` from the ground **replaces** the `radio.config` value.
If no `radio.config` line is present for a given parameter, the compile-time
default remains active.

### 15.4 Validation

The parser validates every `radio.config` value during script load.
If the value is outside the allowed range, the script is **rejected** (status
`ERROR`) and an error message is stored.  The script will not activate.

### 15.5 Example

```ams
# ── Radio / Monitoring overrides ─────────────────────────────
radio.config telem_interval    = 1000    # 1 s instead of 2 s
radio.config monitor.alt.high  = 4500    # High-altitude pad
radio.config monitor.alt.low   = -10     # Slight depression in terrain
radio.config monitor.accel.max = 200     # Competition rocket profile
radio.config monitor.temp.high = 90      # Avionics bay thermal limit
radio.config monitor.temp.low  = -30     # Cold-soaked motor case

state PAD:
  …
```

### 15.6 Interaction with ST\[12\] monitoring

Monitoring thresholds set by `radio.config` are applied to the engine's
monitoring slots (see §2.3 and §6.x) using the same internal path as a
`SET_CONFIG_PARAM` telecommand.  The ST\[12\] monitoring FSM will use these
values from the first evaluation tick onward.

---

## 16. Background Tasks (AMS-11)

Background tasks run on a fixed timer **regardless of the active state**.
They are ideal for cross-cutting concerns — thermal monitoring, battery watch,
data archiving — that should not be modelled inside every individual state.

### 15.1 Basic syntax

```ams
task thermal_monitor:
  every 1000ms:
    if BARO.temp > 80:
      EVENT.warning "OVERHEAT"
```

A `task` block requires exactly one `every Nms:` line and one or more `if COND:` rules.
Each `if` rule may contain an `EVENT.*` action, a `set` action, or both.

### 15.2 State filter (`when in`)

Use `when in STATE…` to restrict the task to specific states:

```ams
task battery_guard when in FLIGHT ASCENT DESCENT:
  every 2000ms:
    if BARO.pressure < 5000:
      EVENT.error "LOW PRESSURE"
```

The timer resets whenever the current state is outside the filter, so the task
fires promptly on the first tick after re-entering an active state.

### 15.3 Combining EVENT and set in one rule

```ams
task peak_tracker:
  every 500ms:
    if BARO.alt > peak_alt:
      set peak_alt = max(peak_alt, BARO.alt)
      EVENT.info "New peak"
```

### 15.4 Constraints

| Item | Limit |
|------|-------|
| Max tasks per script | `AMS_MAX_TASKS` (4) |
| Max `if` rules per task | `AMS_MAX_TASK_RULES` (4) |
| Max `when in` states | `AMS_MAX_TASK_ACTIVE_STATES` (6) |
| Min period | `TELEMETRY_INTERVAL_MIN` (100 ms) |
| TC conditions | **Not allowed** in task `if` |

Tasks may not trigger state transitions — use `transition` inside state blocks for that.

---

## 17. Formal Validation (`assert:`) (AMS-15)

The optional `assert:` block runs **at parse time** before the script activates.
If any assertion fails, the script is rejected with a descriptive error — it will
never enter the running state.

### 16.1 Supported assertions

```ams
assert:
  reachable LANDED
  no_dead_states
  max_transition_depth < 20
```

| Directive | Meaning |
|-----------|---------|
| `reachable STATE` | STATE must be reachable from the initial state via some transition path |
| `no_dead_states` | Every declared state must be reachable from the initial state |
| `max_transition_depth < N` | The longest acyclic path through the state graph must be < N hops |

### 16.2 When to use

Always include at least `no_dead_states` in production scripts.  
Add `reachable LANDED` (or your terminal state) to guarantee the mission can complete.  
Use `max_transition_depth` to cap graph complexity for review purposes.

### 16.3 Full example with tasks and assertions

```ams
apid 0x01

include BMP280 as BARO
include BN220  as GPS

var peak_temp = 0

state PAD:
  transition to FLIGHT when TC.command == LAUNCH

state FLIGHT:
  transition to DESCENT when BARO.alt > 3000

state DESCENT:
  transition to LANDED when GPS.speed < 1.5

state LANDED:
  on_enter:
    EVENT.info "Landed safely"

task thermal_monitor:
  every 1000ms:
    if BARO.temp > 75:
      EVENT.warning "THERMAL"
      set peak_temp = max(peak_temp, BARO.temp)

assert:
  reachable LANDED
  no_dead_states
  max_transition_depth < 10
```

---

## 18. Ground-to-Rocket Command Reception via Radio (APUS)

The AMS engine exposes a set of public methods that the `RadioDispatcher`
class calls when it receives APUS command frames from the GCS.  This section
documents the full receive path and the mapping between APUS command IDs and
engine API calls.

### 17.1 Receive path overview

```
Radio UART  →  RadioDispatcher::poll()          (called every loop() tick)
                │
                ├─ processBuffer()              SYNC scan + frame assembly
                ├─ proto::decode()              CRC-32 validation
                └─ dispatchFrame()
                    ├─ handleCommand()          duplicate check → accept ACK
                    │   └─ executeCommand()     maps CommandId → engine call
                    │       └─ sendAckNack()    completion ACK/NACK if FLAG_ACK_REQ
                    └─ handleHeartbeat()        echo HEARTBEAT from NODE_ROCKET
```

`RadioDispatcher::poll()` is non-blocking (drains UART FIFO only) and is
called from `loop()` **before** `missionEngine.tick()`.

### 17.2 Command-to-engine mapping

| APUS CommandId        | Engine call / action                                                  | Precondition check               |
|-----------------------|-----------------------------------------------------------------------|----------------------------------|
| `ARM_FLIGHT`          | `engine_.arm()`                                                       | None                             |
| `ABORT`               | `engine_.injectTcCommand("ABORT")`                                    | None                             |
| `FACTORY_RESET`       | `engine_.injectTcCommand("RESET")`                                    | None                             |
| `FIRE_PYRO_A`         | HAL not yet connected — returns `EXECUTION_ERROR`                     | `status == RUNNING`              |
| `FIRE_PYRO_B`         | HAL not yet connected — returns `EXECUTION_ERROR`                     | `status == RUNNING`              |
| `SET_MODE`            | `engine_.setExecutionEnabled(payload[2] != 0)`                        | None                             |
| `SET_FCS_ACTIVE`      | `engine_.setExecutionEnabled(payload[2] != 0)`                        | None                             |
| `REQUEST_TELEMETRY`   | `engine_.requestTelemetry(nowMs)` — sends all active HK slots now     | None                             |
| `SET_TELEM_INTERVAL`  | `engine_.setTelemInterval(interval_ms)` — range [100, 60 000] ms     | None                             |
| `REQUEST_STATUS`      | Builds `TelemetryPayload` via `engine_.getStatusBits()` / `getSnapshot()` and sends directly | None        |
| `REQUEST_CONFIG`      | ACK only (no dedicated config frame in APUS)                          | None                             |
| `VERIFY_CONFIG`       | ACK only (config assumed valid while RUNNING)                         | None                             |
| `SET_CONFIG_PARAM`    | `EXECUTION_ERROR` — not implemented                                   | None                             |
| `FACTORY_RESET`       | `engine_.injectTcCommand("RESET")`                                    | None                             |
| Unknown ID            | `EXECUTION_ERROR`                                                     | None                             |

### 17.3 ACK/NACK protocol (APUS-9)

Every accepted command receives an **acceptance ACK** unconditionally
(`FailureCode::NONE`).  A **completion ACK/NACK** is sent only when the sender
sets `FLAG_ACK_REQ` in the frame flags.

| FailureCode         | Meaning                                                   |
|---------------------|-----------------------------------------------------------|
| `NONE`              | Success                                                   |
| `PRECONDITION_FAIL` | Rejected because a precondition was not met (e.g. not RUNNING for pyro) |
| `EXECUTION_ERROR`   | Command accepted but runtime failed (HAL not connected, etc.) |

### 17.4 Interaction with AMS mission scripts

Commands received over radio can interact with a running AMS mission in three
ways:

1. **Direct state change** — `ARM_FLIGHT`, `SET_MODE`, `SET_FCS_ACTIVE` change
   the engine's execution state immediately, which takes effect on the next
   `tick()`.
2. **TC token injection** — `ABORT` and `FACTORY_RESET` inject a TC token
   (`TcCommand::ABORT` / `TcCommand::RESET`) that scripts can listen to via
   `TC.command` transition conditions:

   ```ams
   transition to ABORT when TC.command == ABORT
   ```

3. **Telemetry on demand** — `REQUEST_TELEMETRY` triggers an immediate HK
   transmission for every active slot in the current state, bypassing the
   normal cadence timer.  Useful for GCS to verify sensor data at any time.

### 17.5 StatusBits population (APUS-6 gap resolution)

Prior to this implementation, all five `statusBits` flags were always `0`
(default zero-initialised struct).  The fix introduced:

- `MissionScriptEngine::buildStatusBitsLocked()` — private locked helper that
  derives all five bits from live engine state and GPS driver `hasFix()`.
- `MissionScriptEngine::getStatusBits()` — public thread-safe wrapper used by
  `RadioDispatcher`.
- `MissionScriptEngine::notifyPyroFired(channel)` — call this after a
  successful pyro GPIO pulse to latch `pyroAFired` / `pyroBFired`.

All three telemetry TX paths (`sendHkReportLocked`, `sendHkReportSlotLocked`,
and the `REQUEST_STATUS` handler) now call `buildStatusBitsLocked()` so the GCS
always receives accurate status flags.


# AMS Standard (Ares Mission Script)

This document defines the normative AMS behavior implemented in firmware.
It is the reference for syntax, runtime semantics, safety constraints,
and PUS/APUS mapping.

Implementation reference:
- src/ams/mission_script_engine.h
- src/ams/mission_script_engine.cpp

Companion development guide:
- docs/development/ams_programming_guide.md

---

## AMS-1 Scope

AMS provides a deterministic state-machine language for mission control.

AMS supports:
- State declaration and transition evaluation
- Per-state guard conditions (`conditions:`)
- Per-state failure handler (`on_error:` + `EVENT.*`)
- PUS ST[3] telemetry emission (HK.report)
- PUS ST[5] event emission (EVENT.*)
- TC token gating from API commands (TC.command)
- Local sensor logging to text files (LOG.report)
- Priority arbitration among EVENT, HK, and LOG

AMS does not use dynamic allocation and is interpreted at runtime.

---

## AMS-2 Storage Model

- Script extension: `.ams`
- Script directory: `/missions`
- One active script at a time
- Engine snapshot fields: `status`, `activeFile`, `state`, `error`

Filename constraints:
- Allowed: `a-z A-Z 0-9 _ - .`
- Forbidden: `..`
- Maximum length: `ares::MISSION_FILENAME_MAX`

Mission local log file:
- Path: `/logs/mission_<script>.txt`
- Created or overwritten on activation
- Line format: `t_ms=<ms>,state=<state>,field=value,...`

Persistent resume checkpoint:
- Path: `/missions/.ams_resume.chk`
- Write policy:
  - Forced write on state entry and execution enable/disable changes
  - Periodic write while running using `AMS_CHECKPOINT_INTERVAL_MS`
- Clear policy:
  - On explicit `deactivate()`
  - On terminal mission completion (`COMPLETE`)
  - On invalid/corrupt checkpoint record

**Checkpoint format v1** (legacy — accepted on restore, no longer written):
```
1|file|stateIdx|executionEnabled|running|status|seq|stateElapsed|hkElapsed|logElapsed
```

**Checkpoint format v2** (current — `AMS_RESUME_VERSION = 2`):
```
2|file|stateIdx|executionEnabled|running|status|seq|stateElapsed|hkElapsed|logElapsed[|varCount|name1=value1=valid1|...|nameN=valueN=validN]
```
Fields:
| Field | Description |
|---|---|
| `version` | Format version (`1` legacy, `2` current) |
| `file` | Script filename (e.g. `flight_test.ams`) |
| `stateIdx` | Zero-based index of active state |
| `executionEnabled` | `1` if armed and executing; `0` if paused |
| `running` | `1` if engine is active |
| `status` | Numeric `EngineStatus` value |
| `seq` | HK frame sequence counter |
| `stateElapsed` | ms since state entry |
| `hkElapsed` | ms since last HK transmission |
| `logElapsed` | ms since last LOG write |
| `varCount` | Number of variable entries (v2 only) |
| `name=value=valid` | Per-variable: name, float value, `0`/`1` valid flag (v2 only) |

Restore rules:
- v1 checkpoints are restored without variable data
- v2 checkpoints restore global variable state
- Checkpoint is discarded if `!(running && executionEnabled && status == RUNNING)`

---

## AMS-3 Engine Lifecycle

Engine status values:
- `IDLE`
- `LOADED`
- `RUNNING`
- `ERROR`
- `COMPLETE`

Lifecycle:
1. `activate(file)` loads, parses, resolves transitions, and prepares log file
2. Initial state is `WAIT` if present, otherwise first declared state
3. Runtime progression is blocked until `executionEnabled == true`
4. `setExecutionEnabled(true)` enables transition/action execution
5. `deactivate()` resets active state and returns to `IDLE`

Flight API integration:
- `POST /api/arm` enables execution and injects `LAUNCH`
- `POST /api/abort` disables execution and deactivates script

Boot restore integration:
1. `begin()` attempts restore from `/missions/.ams_resume.chk`
2. Restore is accepted only when all conditions hold:
  - checkpoint version matches firmware parser version
  - script can be loaded from storage
  - state index is valid
  - status enum is within declared range
  - restored tuple is exactly: `running=1`, `executionEnabled=1`, `status=RUNNING`
3. If accepted, AMS continues from the persisted state/timers and rewrites checkpoint immediately
4. If rejected, checkpoint is discarded and AMS remains non-running

---

## AMS-4 Language Grammar (Current Profile)

### AMS-4.1 Metadata

Accepted metadata lines:

```ams
include BN220 as GPS
include BMP280 as BARO
include LORA as COM

pus.apid = 1
pus.service 3 as HK
pus.service 5 as EVENT
pus.service 1 as TC
```

Notes:
- `include` and `pus.service` lines are accepted as metadata
- `pus.apid` is validated and mapped to `nodeId`

### AMS-4.2 State Block

```ams
state FLIGHT:
```

Allowed statements inside a state:
- `on_enter:`
- `EVENT.info|warning|error "text"`
- `conditions:`
- `<LHS> <OP> <RHS>` (inside `conditions:`)
- `on_error:`
- `every Nms:`
- `HK.report { ... }`
- `log_every Nms:`
- `LOG.report { ... }`
- `priorities ...`
- `transition to <STATE> when <COND>`

### AMS-4.3 Independent HK/LOG Cadence

```ams
every 1000ms:
  HK.report { ... }

log_every 200ms:
  LOG.report { ... }
```

Rules:
- `HK.report` requires a preceding `every` block
- `LOG.report` requires a preceding `log_every` block
- HK and LOG use independent per-state timers

### AMS-4.4 Priorities and Tick Budget

Supported forms:

```ams
priorities event=4 hk=3 log=1 budget=2
priorities event=4 hk=3 log=1
priorities hk=3 log=1 budget=2
priorities hk=3 log=1
```

Ranges:
- `event`, `hk`, `log`: `0..9`
- `budget`: `1..3`

State defaults:
- `eventPriority = 4`
- `hkPriority = 2`
- `logPriority = 1`
- `actionBudget = 2`

Arbitration behavior per tick:
- Due actions are calculated for pending EVENT, HK, LOG
- Up to `budget` actions are executed
- Highest priority executes first
- Tie-break order is deterministic: EVENT, HK, LOG

### AMS-4.5 Sensor Expressions

Supported expressions in both HK and LOG blocks:
- `GPS.lat`
- `GPS.lon`
- `GPS.alt`
- `BARO.alt`
- `BARO.temp`
- `BARO.pressure`
- `IMU.accel_x` — acceleration X axis (m/s²)
- `IMU.accel_y` — acceleration Y axis (m/s²)
- `IMU.accel_z` — acceleration Z axis (m/s²)
- `IMU.accel_mag` — acceleration vector magnitude √(x²+y²+z²) (m/s²), also mapped to TelemetryPayload
- `IMU.gyro_x` — angular rate X axis (deg/s)
- `IMU.gyro_y` — angular rate Y axis (deg/s)
- `IMU.gyro_z` — angular rate Z axis (deg/s)
- `IMU.temp` — on-chip IMU temperature (°C), LOG only

### AMS-4.6 Transitions

Full extended syntax:

```ams
transition to <STATE> when <COND> [or|and <COND> ...] [for <N>ms]
```

Where each `<COND>` may be one of:

| Form | Meaning |
|---|---|
| `TC.command == LAUNCH\|ABORT\|RESET` | TC one-shot gate |
| `BARO.alt < x` / `BARO.alt > x` | Absolute sensor threshold |
| `BARO.temp < x` / `BARO.pressure < x` | (and any other BARO/GPS/IMU field) |
| `ALIAS.field delta < x` | Inter-sample delta < x (AMS-4.6.2) |
| `ALIAS.field delta > x` | Inter-sample delta > x |
| `ALIAS.field falling` | Sugar: delta < 0 |
| `ALIAS.field rising` | Sugar: delta > 0 |

Supported fields for standard comparisons:
- `TC.command == LAUNCH|ABORT|RESET`
- `BARO.alt < x`, `BARO.alt > x`
- `BARO.temp < x`, `BARO.temp > x`
- `BARO.pressure < x`, `BARO.pressure > x`
- `GPS.alt < x`, `GPS.alt > x`
- `GPS.speed < x`, `GPS.speed > x`
- `IMU.accel_x < x`, `IMU.accel_x > x`
- `IMU.accel_y < x`, `IMU.accel_y > x`
- `IMU.accel_z < x`, `IMU.accel_z > x`
- `IMU.accel_mag < x`, `IMU.accel_mag > x`

Transition targets are resolved after parse.
Unknown targets cause `ERROR` status.

### AMS-4.6.1 Persistence Modifier (`for <N>ms`)

An optional persistence window can be appended to any sensor transition:

```ams
transition to <STATE> when <LHS> <OP> <RHS> for <N>ms
```

Example:

```ams
transition to APOGEE when BARO.alt > 3000 for 500ms
```

Semantics:
- The **compound** condition must evaluate as **true on every consecutive tick**
  for at least `N` milliseconds before the transition fires.
- If the compound falls false before the window elapses, the hold timer resets.
- Omitting `for Nms` preserves the existing single-sample (immediate) behaviour.
- `for Nms` is **ignored** for transitions whose only condition is `TC.command`.
- Minimum value: `1 ms`. Zero is rejected at parse time.

Runtime state:
- `transitionCondHolding_` — set when the compound first becomes true.
- `transitionCondMetMs_` — timestamp of the first true sample.
- Both fields are reset on every state entry and on `deactivate()`.

### AMS-4.6.2 Delta / Trend Conditions

Delta conditions compare the **difference between the current sensor reading
and the previous reading** (taken on the previous tick) against a threshold.

Forms:

```ams
ALIAS.field delta < VALUE
ALIAS.field delta > VALUE
ALIAS.field falling         // delta < 0 (descending)
ALIAS.field rising          // delta > 0 (ascending)
```

Semantics:
- `delta = current_reading − previous_reading`
- On the first tick after entering a state, no previous reading is available —
  the condition evaluates to **false** (no spurious trigger on entry).
- If the sensor read fails, the condition evaluates to **false** and the
  previous value is **not** updated (the baseline is preserved for the next tick).
- The delta previous-value table is reset on every state entry.

### AMS-4.6.3 Compound Conditions (`or` / `and`)

Multiple conditions can be combined in a single transition using `or` or `and`:

```ams
transition to DESCENT when BARO.alt falling or GPS.alt falling
transition to ABORT when BARO.temp > 90 and IMU.accel_mag > 200
```

Rules:
- Up to `AMS_MAX_TRANSITION_CONDS` (4) sub-conditions per transition.
- Logic must be **homogeneous**: mixing `or` and `and` in one transition is
  a parse error.
- `TC.command` may appear in compound conditions, but is consumed only when
  the overall compound evaluates to true and the transition fires.
- `for Nms` applies to the **compound result**: the compound must stay true
  for the full window.
- Single-condition transitions remain fully backward-compatible.

### AMS-4.7 Guard Conditions and on_error

Guard conditions are optional and state-local.

Example:

```ams
state FLIGHT:
  conditions:
    BARO.temp > -40
    BARO.temp < 85
    GPS.speed < 250
  on_error:
    EVENT.error "FLIGHT guard violated"
```

Rules:
- `conditions:` accepts up to `ares::AMS_MAX_CONDITIONS` lines per state.
- Condition expression support matches sensor/time transition operators:
  - `BARO.alt < x`, `BARO.alt > x`
  - `BARO.temp < x`, `BARO.temp > x`
  - `BARO.pressure < x`, `BARO.pressure > x`
  - `GPS.alt < x`, `GPS.alt > x`
  - `GPS.speed < x`, `GPS.speed > x`
  - `TIME.elapsed > x`
  - Variable RHS forms: `BARO.alt < varname`, `BARO.alt < (varname + offset)` (AMS-4.8).
- `TC.command` is not valid inside `conditions:`.
- `on_error:` currently accepts only one action type: `EVENT.info|warning|error "text"`.

---

## AMS-4.8 Global Variables

### AMS-4.8.1 Declaration

Global variables are declared in the metadata section (before any `state` block):

```ams
var NAME = VALUE
```

- `NAME`: identifier, up to 15 characters (no dots, no reserved names `TIME`/`TC`).
- `VALUE`: floating-point literal (initial value); variable is **invalid** until
  a `set` action fires.
- Maximum `AMS_MAX_VARS` (8) variables per script.
- Duplicate names are rejected at parse time.

### AMS-4.8.2 Set Actions

Variables are assigned inside an `on_enter:` block using `set`:

```ams
on_enter:
  set ground_alt = BARO.alt
```

Or using the calibration form (N-sample average, 1 ≤ N ≤ 10):

```ams
on_enter:
  set ground_alt = CALIBRATE(BARO.alt, 5)
```

Rules:
- The variable must be declared before its first use.
- `ALIAS.field` must reference a declared alias and a valid field for that alias.
- `set` actions execute synchronously at state entry, before the `on_enter` EVENT is dispatched.
- Up to `AMS_MAX_SET_ACTIONS` (4) set actions per `on_enter` block.
- For `CALIBRATE(ALIAS.field, N)`: the engine reads the sensor N times and stores
  the arithmetic mean of valid readings.  Sensors with firmware-level caching may
  return the same sample on rapid successive reads.
- If all reads fail (sensor error / NaN): the variable is left unchanged, an
  `EVENT.warning` is queued, and execution continues (NaN guard — AMS-5.7).

### AMS-4.8.3 Variable RHS in Conditions

Transition conditions and guard conditions may use a declared variable as the
right-hand side threshold:

```ams
transition to LANDED when BARO.alt < ground_alt
transition to LANDED when BARO.alt < (ground_alt + 10)
transition to LANDED when BARO.alt < (ground_alt - 5)
```

- The variable reference is validated at **parse time** (variable must be declared).
- At **runtime**, if the variable has not yet been set (`.valid == false`), the
  condition evaluates to **false** (cannot fire).  This prevents spurious
  transitions before the calibration state has executed.
- The offset form `(varname ± offset)` is evaluated as `var.value + offset`.

### AMS-4.8.4 Checkpoint Persistence (v2)

Variables are persisted in the AMS checkpoint file (version 2):

```
2|fileName|stateIdx|exec|running|status|seq|stateElap|hkElap|logElap|varCount|name1=val1=valid1|...|nameN=valN=validN
```

On restore:
- Variables are matched by name against the loaded program's variable table.
- If a persisted variable is not found in the current script, it is silently skipped.
- v1 checkpoints (no variable data) are still accepted; variables start invalid.

---

## AMS-5 Runtime Semantics

### AMS-5.1 Transition-First Rule

Tick execution order:
1. Validate runtime context
2. Evaluate each sub-condition in the transition (up to `AMS_MAX_TRANSITION_CONDS`):
   - For delta conditions: read current value, compare against previous; update previous on success.
   - For sensor/TC/time conditions: evaluate as defined in AMS-4.6.
3. Combine sub-condition results using `or`/`and` logic (AMS-4.6.3).
4. If `for Nms` is active and the compound is true: arm/check hold timer; proceed
   only when the window has fully elapsed (AMS-4.6.1).  If compound drops false,
   reset the hold timer.
5. If transition fires: consume TC token (if any), enter next state, end tick.
6. Evaluate guard conditions (`conditions:`)
7. If any condition is violated, emit `on_error` event (if defined) and set engine `ERROR`
8. Otherwise, evaluate due actions and run arbitration

### AMS-5.2 On-Enter Event Queueing

`on_enter` event is queued, not sent immediately.
It is consumed by arbitration as an EVENT due action.

> **Note (AMS-4.8):** `set` actions execute before the event is queued.
> The variable value is therefore available to the arbitration tick that
> dispatches the event.

### AMS-5.7 Variable NaN Guard

If a `set` or `CALIBRATE` action fails to produce a valid reading:

- The target variable retains its previous value (or remains invalid if never set).
- An `EVENT.warning` frame is queued.
- Execution continues; the engine does not enter ERROR state for a failed `set`.
- Transition/guard conditions that reference a still-invalid variable evaluate to
  **false** (condition cannot fire), providing safe default behaviour.

### AMS-5.3 TC One-Shot Consumption

`TC.command` token is consumed once when a matching transition fires.

### AMS-5.4 HK Behavior

`HK.report` builds a binary telemetry frame and sends through radio protocol
encoder (`MsgType::TELEMETRY`).

### AMS-5.5 LOG Behavior

`LOG.report` appends local text records to mission log file.
If append fails, a warning is logged and runtime continues.

### AMS-5.6 Guard Failure Behavior

When any active state guard condition fails:
- `on_error` event is sent immediately if defined for that state
- Engine status becomes `ERROR`
- Runtime progression stops until external recovery/deactivation

#### Sensor-unavailable (NaN) behavior

If a sensor referenced in a `conditions:` block cannot be read (driver failure,
I2C timeout, GPS loss-of-fix), the guard condition evaluates to **"holds"** —
no false alarm is raised.  This is the fail-safe default: transient sensor
outages do not trigger `on_error`.

The same applies to `transition` conditions: if the sensor read fails, the
transition **does not fire**.

> ⚠️ **Implication**: a permanently-failed sensor will never violate its guard
> condition.  If a mission relies on `IMU.accel_mag > 50` to detect launch, a
> dead IMU will silently prevent that transition from ever firing.  Pair sensor
> guards with a `TIME.elapsed` watchdog as backup.

#### Design recommendation — RECOVERY / SAFE state

The AMS engine halts on guard violation and emits an `on_error` event, but
provides **no built-in automatic recovery**.  In a real mission, halting without
periodic telemetry leaves the vehicle unreachable.

**Recommended pattern**: define a `SAFE` or `RECOVERY` state and add an
`ABORT` transition in every critical state:

```ams
state FLIGHT:
  conditions:
    BARO.temp < 85
    GPS.speed < 400
  on_error:
    EVENT.error "FLIGHT guard violated — operator intervention required"

  transition to SAFE when TC.command == ABORT
  transition to RECOVERY when TIME.elapsed > 180000

state SAFE:
  on_enter:
    EVENT.warning "Entered safe mode"
  priorities event=4 hk=3 log=1 budget=2
  // No guard conditions — engine must never halt in SAFE
  every 5000ms:
    HK.report {
      gps_lat: GPS.lat
      gps_lon: GPS.lon
      gps_alt: GPS.alt
    }
  // Terminal: awaits ground recovery, no automatic transition
```

Rules for fault-tolerant AMS missions:
- Every state that can fail must have `transition to SAFE when TC.command == ABORT`.
- The `SAFE`/`RECOVERY` state must have **no** `conditions:` block so the
  engine never stops while waiting for ground intervention.
- Use `EVENT.error` in `on_error:` to alert the ground station.
- The ground operator injects `ABORT` via
  `POST /api/mission/command {"command":"ABORT"}` to force the SAFE transition
  from any state that does not already handle its guard violation via `on_error`.


---

## AMS-6 PUS/APUS Mapping

- `EVENT.info|warning|error`
  - PUS ST[5] Event Reporting
  - `MsgType::EVENT`

- `HK.report`
  - PUS ST[3] Housekeeping
  - `MsgType::TELEMETRY`

- `TC.command == ...`
  - ST[1] input gate for telecommand-driven transition

Radio transport format is binary ARES frame with protocol CRC, not JSON.

---

## AMS-7 Mission API Surface

Implemented endpoints:
- `GET /api/mission`
- `GET /api/missions`
- `GET /api/missions/active`
- `GET /api/missions/{file}`
- `PUT /api/missions/{file}`
- `DELETE /api/missions/{file}`
- `POST /api/mission/activate {"file":"test.ams"}`
- `POST /api/mission/deactivate`
- `POST /api/mission/command {"command":"LAUNCH"}`

---

## AMS-8 Safety and Determinism Constraints

| ID | Rule |
|---|---|
| AMS-8.1 | Parser/runtime do not allocate from heap. |
| AMS-8.2 | Parser/runtime loops are bounded by compile-time limits. |
| AMS-8.3 | Runtime control APIs are mutex-protected with timeout. |
| AMS-8.4 | Parse errors or invalid transition targets force ERROR state. |
| AMS-8.5 | Script/log buffers are bounded by configuration constants. |
| AMS-8.6 | Script filenames are strictly validated before storage access. |
| AMS-8.7 | Guard condition failure forces ERROR state (fail-safe). |

---

## AMS-9 APID to Node Mapping

Implemented mapping:
- `0 -> NODE_BROADCAST`
- `1 -> NODE_ROCKET`
- `2 -> NODE_GROUND`
- `3 -> NODE_PAYLOAD`

Any APID outside this mapping is rejected.

---

## AMS-10 Out of Scope (Current Profile)

Not implemented in current AMS profile:
- Multiple active scripts
- Boolean compound conditions (`AND`/`OR`)
- Complex arithmetic expressions in fields
- Absolute time scheduling
- Automatic log rotation/partitioning---

## AMS-11 Execution Log

The AMS engine emits structured diagnostic messages through the serial ARES log
system (`debug/ares_log.h`).  All messages use the `AMS` subsystem tag.

Output line format: `[uptime_ms] L AMS: message`
- `uptime_ms` — `millis()`, right-aligned 7 digits.
- `L` — single-character severity: `E` error, `W` warning, `I` info.

### AMS-11.1 Parse-time messages

Parse errors include the 1-based source line number of the failing statement.

| Severity | Pattern | Meaning |
|---|---|---|
| `E` | `Parse Error (line N): <reason>` | Parser rejected a line or detected a semantic error |
| `I` | `activated script=<file> states=<N>` | Script loaded successfully |

Examples:
```
[  1024] E AMS: Parse Error (line 42): Unknown state 'FLIGTH'
[  1025] E AMS: Parse Error (line 17): TC.command not valid in conditions block
[  1026] E AMS: Parse Error (line 5): script has no states
[  2001] I AMS: activated script=flight_test.ams states=3
```

### AMS-11.2 Runtime messages

| Severity | Pattern | Meaning |
|---|---|---|
| `I` | `state -> <NAME>` | State entered (initial entry or after a transition) |
| `I` | `Transition: '<FROM>' -> '<TO>' at t=<ms>ms` | State transition condition fired |
| `W` | `Guard Violation: State '<S>', Condition '<C>' failed (Value: <V>)` | Guard condition violated; engine entering ERROR |
| `W` | `ABORT TC not consumed by state=<S>: force-deactivating` | Unhandled ABORT telecommand; engine deactivated |
| `I` | `mission complete: terminal state=<S>` | Mission reached a terminal state (status → COMPLETE) |
| `I` | `deactivated` | Engine deactivated via API or ABORT |
| `W` | `resumed AMS from checkpoint: file=<F> state=<S>` | Engine state restored from persistent checkpoint |

Examples:
```
[ 60042] I AMS: Transition: 'FLIGHT_TEST' -> 'RECOVERY' at t=60042ms
[ 60042] I AMS: state -> RECOVERY
[ 91500] W AMS: Guard Violation: State 'FLIGHT', Condition 'BARO.temp < 85' failed (Value: 92.400)
[ 91500] W AMS: Guard Violation: State 'ASCENT', Condition 'TIME.elapsed > 120000' failed (Value: 125300.000ms)
[180001] I AMS: mission complete: terminal state=END
[180001] I AMS: deactivated
```

### AMS-11.3 Guard condition text format

| Condition kind | Log format |
|---|---|
| Sensor limit | `<ALIAS>.<field> < <threshold>` or `<ALIAS>.<field> > <threshold>` |
| Time watchdog | `TIME.elapsed > <threshold>` |

The `Value` field shows the sensor reading or elapsed time (in ms for
`TIME.elapsed`) at the moment the violation was detected.

If a sensor is **unavailable** when a guard is evaluated, the condition is
treated as "holds" — no log message is emitted and no error is raised
(see AMS-5.6 sensor-unavailable behavior).


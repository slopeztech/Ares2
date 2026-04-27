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
- Format: `version|file|stateIdx|executionEnabled|running|status|seq|stateElapsed|hkElapsed|logElapsed`
- Write policy:
  - Forced write on state entry and execution enable/disable changes
  - Periodic write while running using `AMS_CHECKPOINT_INTERVAL_MS`
- Clear policy:
  - On explicit `deactivate()`
  - On terminal mission completion (`COMPLETE`)
  - On invalid/corrupt checkpoint record

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

General form:

```ams
transition to <STATE> when <LHS> <OP> <RHS>
```

Supported conditions:
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
- `TC.command` is not valid inside `conditions:`.
- `on_error:` currently accepts only one action type: `EVENT.info|warning|error "text"`.

---

## AMS-5 Runtime Semantics

### AMS-5.1 Transition-First Rule

Tick execution order:
1. Validate runtime context
2. Evaluate transition condition
3. If transition occurs, enter next state and end tick
4. Evaluate guard conditions (`conditions:`)
5. If any condition is violated, emit `on_error` event (if defined) and set engine `ERROR`
6. Otherwise, evaluate due actions and run arbitration

### AMS-5.2 On-Enter Event Queueing

`on_enter` event is queued, not sent immediately.
It is consumed by arbitration as an EVENT due action.

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
- Automatic log rotation/partitioning

---

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


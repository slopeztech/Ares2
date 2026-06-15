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
- Per-state Wi-Fi/API directives (`wifi.enable`, `wifi.disable`, `api.enable`, `api.disable`)
- PUS ST[3] telemetry emission (HK.report)
- PUS ST[5] event emission (EVENT.*)
- TC token gating from API commands (TC.command)
- Local sensor logging to text files (LOG.report)
- Priority arbitration among EVENT, HK, and LOG
- Per-slot COM routing via `every Nms via ALIAS:` (A2-3)

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

**Checkpoint format v2** (legacy — accepted on restore, no longer written):
```
2|file|stateIdx|executionEnabled|running|status|seq|stateElapsed|hkElapsed|logElapsed[|varCount|name1=value1=valid1|...|nameN=valueN=validN]
```

**Checkpoint format v3** (current — `AMS_RESUME_VERSION = 3`):
```
3|file|stateIdx|executionEnabled|running|status|seq|stateElapsed|hkElapsed|logElapsed[|varCount|name1=value1=valid1|...|nameN=valueN=validN]|hkSlotCount|hkSlotElap0|...|logSlotCount|logSlotElap0|...
```
Fields:
| Field | Description |
|---|---|
| `version` | Format version (`1`/`2` legacy, `3` current) |
| `file` | Script filename (e.g. `flight_test.ams`) |
| `stateIdx` | Zero-based index of active state |
| `executionEnabled` | `1` if armed and executing; `0` if paused |
| `running` | `1` if engine is active |
| `status` | Numeric `EngineStatus` value |
| `seq` | HK frame sequence counter |
| `stateElapsed` | ms since state entry |
| `hkElapsed` | ms since last global HK transmission |
| `logElapsed` | ms since last global LOG write |
| `varCount` | Number of variable entries (v2/v3, omitted if 0) |
| `name=value=valid` | Per-variable: name, float value, `0`/`1` valid flag (v2/v3) |
| `hkSlotCount` | Number of per-slot HK elapsed entries (v3) |
| `hkSlotElapN` | ms elapsed since last HK for slot N (v3) |
| `logSlotCount` | Number of per-slot LOG elapsed entries (v3) |
| `logSlotElapN` | ms elapsed since last LOG for slot N (v3) |

Restore rules:
- v1 checkpoints are restored without variable or slot data; slot timers default to `nowMs`
- v2 checkpoints restore global variable state; slot timers default to `nowMs`
- v3 checkpoints restore global variable state and precise per-slot HK/LOG timers
- Checkpoint is discarded if `!(running && executionEnabled && status == RUNNING)`

**State NOT persisted (always reset on restore):**

| Field | Reset behaviour |
|---|---|
| `tcConfirmCount_[]` | Zeroed on every `enterStateLocked()` call — TC confirm counters always restart from 0 after a power cycle |
| `transitionCondHolding_[]` / `transitionCondMetMs_[]` | Reset on every `enterStateLocked()` — any in-progress `for Nms` hold window restarts from zero after restore |
| Sensor cache (`imuCacheValid_`, etc.) | Marked invalid on restore — the first tick after restore will re-read all sensors from the HAL |

Consequence: a `for Nms` hold window that was partially elapsed at power-down will
restart from zero after restore.  A `TC.command confirm N` counter that had
accumulated partial confirmations is also lost.  Mission scripts that depend on
these counters must account for this after a power cycle.

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
5. If the restore succeeds **and** the MCU reset cause is abnormal (panic, WDT, brownout —
   see AMS-4.6.4), `TC.RESET_ABNORMAL` is automatically injected before the first tick
   so the active AMS state can respond to the unexpected reboot

---

## AMS-4 Language Grammar (Current Profile)

### AMS-4.0 Comments

Two single-line comment styles are supported.  There are no multi-line comments.

| Style | Syntax | Notes |
|-------|--------|-------|
| C-style | `// text` | Line must start with `//` after leading whitespace is stripped |
| Shell-style | `# text` | Line must start with `#` after leading whitespace is stripped |

Comment lines are discarded by the parser before any directive is evaluated.  A comment may appear anywhere a full line is expected — in the metadata section, between state blocks, or inside a state block.  Inline (end-of-line) comments are not supported; the entire line must be a comment.

```ams
// This is a C-style comment
# This is a shell-style comment

state INIT:       // NOT valid — inline comments are not supported
  transition to END when TIME.elapsed > 1000  # also NOT valid inline
```

### AMS-4.1 Metadata

Accepted metadata lines:

```ams
include BN220 as GPS
include BMP280 as BARO
include DXLR03 as COM

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
- `on_exit:`
- `EVENT.info|warning|error "text"`
- `conditions:`
- `<LHS> <OP> <RHS>` (inside `conditions:`)
- `on_error:`
- `on_timeout Nms:`
- `every Nms:`
- `every Nms via ALIAS:` (routes HK slot to the COM bound to ALIAS — AMS-4.3.2)
- `HK.report { ... }`
- `log_every Nms:`
- `LOG.report { ... }`
- `SERIAL.report { ... }`
- `priorities ...`
- `wifi.enable`, `wifi.disable`, `api.enable`, `api.disable`
- `transition to <STATE> when <COND>`

### AMS-4.3 Independent HK/LOG Cadence

A state may declare a single pair of `every` / `log_every` blocks:

```ams
every 1000ms:
  HK.report { ... }

log_every 200ms:
  LOG.report { ... }

log_every 250ms:
  SERIAL.report { ... }
```

#### AMS-4.3.1 Multiple cadences per state

Each state may declare up to `AMS_MAX_HK_SLOTS` (currently 4) independent `every` blocks
and up to `AMS_MAX_HK_SLOTS` independent `log_every` blocks. Each block constitutes a
**slot** with its own cadence timer and its own field list:

```ams
state FLIGHT:
  every 1000ms:
    HK.report { gps_lat: GPS.lat  gps_lon: GPS.lon  baro_alt: BARO.alt }
  every 50ms:
    HK.report { ax: IMU.accel_x  ay: IMU.accel_y  az: IMU.accel_z }
  log_every 1000ms:
    LOG.report { gps_lat: GPS.lat  gps_lon: GPS.lon }
  log_every 10ms:
    LOG.report { ax: IMU.accel_x  ay: IMU.accel_y  az: IMU.accel_z }
```

Normative constraints:
- Each `every` block generates an independent PUS-3 HK frame at its declared cadence.
- Each `log_every` block is an independent slot and may output to either:
  - `LOG.report`   → append row to mission CSV file.
  - `SERIAL.report` → emit one formatted line to serial monitor.
- Slot timers reset to zero on state activation (`activate()`).
- The minimum allowed cadence is `TELEMETRY_INTERVAL_MIN` (100 ms).
- All slot arrays are statically allocated; no heap allocation (PO10-3).
- Exceeding `AMS_MAX_HK_SLOTS` in a single state is a parse-time error.
- CSV rows include a `slot` column (0-based) to distinguish cadences: `t_ms, state, slot, <fields...>`.
- Each slot's CSV header is written once on its first active row.

Rules:
- `HK.report` requires a preceding `every` block
- `LOG.report` requires a preceding `log_every` block
- `SERIAL.report` requires a preceding `log_every` block
- HK and LOG slots use independent per-slot per-state timers

#### AMS-4.2.1 State-level Wi-Fi / API directives

The AMS language accepts four simple state directives that affect the live
Wi-Fi/AP and REST API policy when entering a state:

- `wifi.enable`
- `wifi.disable`
- `api.enable`
- `api.disable`

These directives are evaluated on state entry. They are intended to override
or refine the global policy defined in `src/config.h` (`WIFI_DISABLE_IN_FLIGHT`)
for a specific mission phase. If a state does not specify any of these lines,
then the global default remains in force.

Example:

```ams
state GROUND:
  wifi.enable
  api.enable

state FLIGHT:
  wifi.disable
  api.disable
```

#### AMS-4.3.2 Per-slot COM routing (`via ALIAS`)

An `every` block may optionally target a specific COM peripheral instead of the
primary radio interface:

```ams
every 500ms via COM2:
  HK.report { gps_alt: GPS.alt  baro_alt: BARO.alt }
```

Syntax:

```
every <N>ms via <ALIAS>:
  HK.report { … }
```

Normative constraints:
- `ALIAS` must be declared with `include MODEL as ALIAS` in the metadata section.
- The alias must be of kind `COM` (radio peripheral).  A non-COM alias (e.g. `GPS`, `BARO`) is rejected at parse time with error `"every via: alias is not a COM include"`.
- The alias name must be 1–15 characters.  An empty or over-length name is a parse error.
- No characters may follow the colon after `ALIAS:` (trailing garbage is rejected).
- The minimum interval is `TELEMETRY_INTERVAL_MIN` (100 ms), same as a plain `every` block.
- Omitting `via ALIAS` is equivalent to routing through `primaryCom_` (unchanged behaviour).
- At runtime, if the alias cannot be resolved (e.g. the driver was registered with a different index), a `LOG_W` is emitted and the frame falls back to `primaryCom_`.
- `via` routing applies only to `HK.report` slots; `log_every` slots are unaffected.
  - `LOG.report` writes to local filesystem.
  - `SERIAL.report` writes to serial monitor.
- Multiple `every` blocks in the same state may target different COM aliases:

```ams
state FLIGHT:
  every 1000ms:                   // routes to primaryCom_
    HK.report { gps_alt: GPS.alt }
  every 500ms via BACKUP_COM:     // routes to BACKUP_COM
    HK.report { ax: IMU.accel_x  ay: IMU.accel_y }
```

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

#### Terminology

- **Action slot** — one `every Nms:` reporting block within a state.  A state
  may have up to `AMS_MAX_HK_SLOTS` HK slots and the same number of LOG slots,
  each with its own independent cadence timer.
- **Action group** — one of the three logical dispatch units: EVENT, HK, and
  LOG.  All HK slots belong to the HK group; all LOG slots belong to the LOG
  group.  The `budget` counter tracks **groups**, not individual slots.

#### Action groups

The engine defines exactly **three action groups** per state:

| Group index | Name  | Contents |
|-------------|-------|----------|
| 0           | EVENT | At most one pending `on_enter:` event frame |
| 1           | HK    | All due HK slots (`every Nms: HK.report { … }`) |
| 2           | LOG   | All due LOG slots (`every Nms: LOG.report { … }`) |

#### `budget` semantics (NORMATIVE)

`budget` counts **action groups dispatched per tick**, not individual slots.
When the HK group is selected, **all** HK slots that are currently due fire
within that single budget step.  The same applies to the LOG group.

Consequence: with `budget=1` and four simultaneously-due HK slots, all four
slots are served in one tick — the budget is not exhausted by individual slots.
This is intentional: HK slots within the same group share a single radio
transmission window; splitting them across ticks would create artificial skew.

Maximum throughput per tick by budget level:

| `budget` | Groups dispatched | Maximum actions in one tick |
|----------|-------------------|---------------------------------|
| 1        | 1                 | 1 EVENT **or** N HK slots **or** M LOG slots |
| 2        | 2                 | Any 2 of the 3 groups |
| 3        | 3                 | All 3 groups |

#### Arbitration behavior per tick

- Due actions are calculated for pending EVENT, HK, LOG
- Up to `budget` **groups** are dispatched per tick
- Highest-priority group executes first
- Tie-break order is deterministic: EVENT > HK > LOG
- Within a selected group all due slots fire before the next group is considered

### AMS-4.5 Sensor Expressions

Supported expressions in both HK and LOG blocks:
- `GPS.lat`
- `GPS.lon`
- `GPS.alt`
- `GPS.speed` — ground speed (km/h)
- `GPS.sats` — satellites in use (integer, compared as float)
- `GPS.hdop` — horizontal dilution of precision (dimensionless; lower is better)
- `BARO.alt`
- `BARO.temp`
- `BARO.pressure`
- `IMU.accel_x` — acceleration X axis (m/s²)
- `IMU.accel_y` — acceleration Y axis (m/s²)
- `IMU.accel_z` — acceleration Z axis (m/s²)
- `IMU.accel_mag` — acceleration vector magnitude √(x²+y²+z²) (m/s²)
- `IMU.gyro_x` — angular rate X axis (deg/s)
- `IMU.gyro_y` — angular rate Y axis (deg/s)
- `IMU.gyro_z` — angular rate Z axis (deg/s)
- `IMU.gyro_mag` — angular rate vector magnitude √(gx²+gy²+gz²) (deg/s)
- `IMU.temp` — on-chip IMU temperature (°C)

### AMS-4.6 Transitions

Full extended syntax:

```ams
transition to <STATE> when <COND> [or|and <COND> ...] [for <N>ms]
```

A state may declare **up to `AMS_MAX_TRANSITIONS` (4) independent `transition to` directives**.
All transitions are evaluated every tick in declaration order.  The first one whose
conditions hold (and whose hold window has fully elapsed) fires; subsequent transitions
are not evaluated that tick.

```ams
state ASCENT:
  transition to SAFE     when TC.command == ABORT
  transition to RECOVERY when TIME.elapsed > 180000
  transition to APOGEE   when BARO.alt > 3000 for 500ms
```

Rules:
- Transitions are evaluated in declaration order; **first match wins**.
- If no transition fires, the state continues on the next tick.
- Exceeding `AMS_MAX_TRANSITIONS` in one state is a **parse-time error**.
- The `for Nms` hold window (AMS-4.6.1) is **per-transition** — each slot
  maintains its own hold-window state independently.

Where each `<COND>` may be one of:

| Form | Meaning |
|---|---|
| `TC.command == LAUNCH\|ABORT\|RESET\|RESET_ABNORMAL` | TC one-shot gate |
| `BARO.alt < x` / `BARO.alt > x` | Absolute sensor threshold |
| `BARO.temp < x` / `BARO.pressure < x` | (and any other BARO/GPS/IMU field) |
| `ALIAS.field delta < x` | Inter-sample delta < x (AMS-4.6.2) |
| `ALIAS.field delta > x` | Inter-sample delta > x |
| `ALIAS.field falling` | Sugar: delta < 0 |
| `ALIAS.field rising` | Sugar: delta > 0 |

Supported fields for standard comparisons:
- `TC.command == LAUNCH|ABORT|RESET|RESET_ABNORMAL`
- `BARO.alt < x`, `BARO.alt > x`
- `BARO.temp < x`, `BARO.temp > x`
- `BARO.pressure < x`, `BARO.pressure > x`
- `GPS.alt < x`, `GPS.alt > x`
- `GPS.speed < x`, `GPS.speed > x`
- `GPS.sats > x` — satellite count (integer compared as float; e.g. `GPS.sats > 6`)
- `GPS.hdop < x` — horizontal dilution of precision (e.g. `GPS.hdop < 2.0`)
- `IMU.accel_x < x`, `IMU.accel_x > x`
- `IMU.accel_y < x`, `IMU.accel_y > x`
- `IMU.accel_z < x`, `IMU.accel_z > x`
- `IMU.accel_mag < x`, `IMU.accel_mag > x`
- `IMU.gyro_mag < x`, `IMU.gyro_mag > x`

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
- `transitionCondHolding_[ti]` — set when transition `ti`'s compound first becomes true.
- `transitionCondMetMs_[ti]`   — timestamp of that first true sample.
- Both arrays are reset on every state entry and on `deactivate()`.

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

### AMS-4.6.4 TC.RESET_ABNORMAL — Abnormal Boot Detection

`TC.RESET_ABNORMAL` is a firmware-injected TC token that fires automatically on
the **first tick** after a successful checkpoint restore following an abnormal
MCU reset.  It allows the AMS script to react to in-flight crashes, watchdog
faults, or brownout events without any ground-station intervention.

#### Trigger conditions

The token is injected when **all** of the following are true at boot:

1. A valid checkpoint is restored (`status == RUNNING`, `executionEnabled == 1`).
2. The MCU reset cause (from `esp_reset_reason()`) is one of:

| Cause | Description |
|---|---|
| `ESP_RST_PANIC` | Software exception / assertion failure |
| `ESP_RST_INT_WDT` | Interrupt watchdog timeout |
| `ESP_RST_TASK_WDT` | Task watchdog timeout |
| `ESP_RST_WDT` | Other watchdog timeout |
| `ESP_RST_BROWNOUT` | Power supply brownout |

Normal reboots (`ESP_RST_POWERON`, `ESP_RST_SW`) are **not** treated as abnormal
and never trigger this TC.

#### Syntax

```ams
transition to <STATE> when TC.command == RESET_ABNORMAL
```

Shorthand is identical to other TC tokens:

```ams
transition to EMERGENCY_RECOVERY on TC.RESET_ABNORMAL
```

> Currently `on TC.*` is not distinct syntax — use the `TC.command ==` form.

#### Semantics

- `TC.RESET_ABNORMAL` is a **one-shot** gate: it is consumed the first time a
  matching transition fires.  If no transition in the active state tests for it,
  the token is discarded on the next tick.
- The `for Nms` hold modifier is **ignored** for `TC.RESET_ABNORMAL` (same rule
  as all TC tokens — AMS-4.6.1).
- The token may appear in compound conditions (`and` / `or`) — see AMS-4.6.3.
- A `LOG_W` entry is written to the system log at boot when the token is injected.

#### Recommended usage pattern

```ams
state ASCENT:
  every 500ms:
    HK.report { baro_alt: BARO.alt  imu_az: IMU.accel_z }
  transition to EMERGENCY_RECOVERY when TC.command == RESET_ABNORMAL
  transition to APOGEE             when BARO.alt delta < 0 for 500ms
  transition to SAFE               when TC.command == ABORT

state EMERGENCY_RECOVERY:
  on_enter:
    EVENT.error "Abnormal reset during flight — deploying drogue"
  every 2000ms:
    HK.report { baro_alt: BARO.alt  gps_lat: GPS.lat  gps_lon: GPS.lon }
  // No conditions: block — engine must not halt during recovery
```

#### Interaction with checkpoint restore

The checkpoint mechanism (AMS-2) guarantees the engine resumes in the same
state it occupied before the reset, with the same timers and variables.  The
`TC.RESET_ABNORMAL` token is injected **after** the restore completes, so the
transition is evaluated with the fully-restored context on the first tick.

#### Safety consideration

If the recovery state itself crashes the MCU again (e.g. due to a hardware
fault), the checkpoint will be rewritten when the engine enters the new state.
On the next boot the engine will resume in the recovery state; `TC.RESET_ABNORMAL`
will fire again if the reset cause is still abnormal.  Design recovery states to
be minimal and fault-tolerant (no `conditions:` block, no complex sensor expressions).

---

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
  - `GPS.sats > x` — pre-arm fix quality check (satellite count)
  - `GPS.hdop < x` — pre-arm fix quality check (HDOP)
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

Or using the calibration form (N-sample average, 1 ≤ N ≤ `AMS_CALIBRATE_MAX_SAMPLES`):

```ams
on_enter:
  set ground_alt = CALIBRATE(BARO.alt, 5)
```

Rules:
- The variable must be declared before its first use.
- `ALIAS.field` must reference a declared alias and a valid field for that alias.
- Non-CALIBRATE `set` actions execute synchronously at state entry, before the `on_enter` EVENT is dispatched.
- Up to `AMS_MAX_SET_ACTIONS` (4) set actions per `on_enter` block.
- **`CALIBRATE` is asynchronous (AMS-4.8.2 async mode):**  On state entry the
  engine starts the accumulator and collects the first sample.  Each subsequent
  `tick()` call collects one additional sample via `stepPendingCalibrationsLocked`.
  After `N` tick-cycles the arithmetic mean of valid readings is written to the
  variable.  The typical completion time is `N × SENSOR_RATE_MS` (e.g. 10 × 50 ms
  = 500 ms for N = 10).  The engine continues evaluating transitions and guard
  conditions during calibration; the calibrated variable remains unchanged (with
  its previous value, or invalid) until all samples are gathered.  This design
  bounds the per-tick mutex-hold time to a single sensor read (~25 ms) regardless
  of N, satisfying AMS-8.3.
- If all reads fail (sensor error / NaN): the variable is left unchanged, an
  `EVENT.warning` is queued, and execution continues (NaN guard — AMS-5.7).
- `enterStateLocked()` resets the calibration accumulator on every state entry

> **Warning — synchronous blocking:** Non-CALIBRATE `set` actions execute in the
> same tick as state entry.  If the sensor HAL takes significant time (e.g. I²C
> bus reads up to ~25 ms per read), back-to-back `set` actions stall the tick for
> their cumulative duration.  Keep the number of synchronous `set` actions small
> (≤ 2 fast reads) and use `CALIBRATE` for multi-sample or slow sensors.

> **Warning — CALIBRATE latency:** The calibrated variable is **not available**
> until all N samples have been collected (N × `SENSOR_RATE_MS` ms after state
> entry).  Transition conditions that reference the variable evaluate to **false**
> until then (NaN guard, AMS-4.8.3).  If the mission logic depends on the
> calibrated value immediately, use a `for Nms` hold window on the transition to
> guarantee the calibration has completed before the condition is tested.
  (including re-entries), so CALIBRATE always restarts from scratch.
- On state transitions the previous state's in-progress calibration is abandoned;
  the new state's set actions start their own accumulation.

### AMS-4.8.3 Variable NaN Guard

If a `set`, `CALIBRATE`, `delta`, `max`, or `min` action fails to obtain a valid
sensor reading, the variable is left unchanged and an `EVENT.warning` is queued.
Full rules: see **AMS-5.7**.

### AMS-4.8.4 Derived Variable — Delta

The `delta` operator stores the difference between the current and the previous
sensor reading at each state entry:

```ams
on_enter:
  set descent_rate = BARO.alt delta
```

- On the **first** execution the delta is `0.0` and the baseline is primed with
  the current reading.
- On subsequent entries: `result = current - baseline; baseline = current`.
- If the sensor read fails, the variable is left unchanged (NaN guard, AMS-4.8.3).

### AMS-4.8.5 Derived Variable — Running Maximum

```ams
on_enter:
  set max_alt = max(max_alt, BARO.alt)
```

- The first argument **must** match the target variable name (self-referential
  running accumulator).
- On the first valid execution: the variable is initialised with the current reading.
- On subsequent executions: `result = max(current_variable, sensor_reading)`.

### AMS-4.8.6 Derived Variable — Running Minimum

```ams
on_enter:
  set min_speed = min(min_speed, GPS.speed)
```

Same rules as AMS-4.8.5 with `min` semantics.

### AMS-4.8.7 Variables in HK / LOG Field Lists

Declared variables may appear as fields in `HK.report` and `LOG.report` blocks
using the bare variable name (no `ALIAS.` prefix) as the expression:

```ams
var ground_alt = 0.0

state WAIT:
  on_enter:
    set ground_alt = BARO.alt
  transition to FLIGHT when TC.command == LAUNCH

state FLIGHT:
  log_every 50ms:
    LOG.report {
      ga: ground_alt
    }
```

Rules:

- The variable must be declared with `var` before the first `state` block.
- A bare identifier in a field expression that contains no `.` is treated as a
  variable reference; any alias lookup is skipped.
- In `LOG.report` (CSV): the current floating-point value of the variable is
  formatted with 2 decimal places.  If the variable is not yet valid (no `set`
  action has fired), the placeholder `nan` is written to keep CSV columns aligned.
- In `HK.report` (binary TM frame): the variable has no fixed slot in
  `TelemetryPayload`; the field is silently omitted from the radio frame.  Use
  `LOG.report` when variable values must be persisted.
- The variable name is limited to 15 characters (`AMS_VAR_NAME_LEN - 1`).

### AMS-4.8.8 Arithmetic Expression in `set` Statements

A `set` statement may use an arithmetic expression as its right-hand side,
allowing derived values to be computed from two or three operands:

```ams
set alt_agl  = BARO.alt - ground_alt
set vvel_avg = ( BARO.alt - prev_alt ) / 0.5
set alt_off  = BARO.alt + 10.0
```

**Syntax** (after stripping all `(` and `)` characters, which are purely cosmetic):

```
set VARNAME = TERM op TERM [op TERM]
```

Where:
- `TERM` is one of: `ALIAS.field` (sensor reading), a declared variable name, or a
  float literal.
- `op` is one of `+`, `-`, `*`, `/` (a single character, space-separated).
- Two or three terms are supported.

**Evaluation order:** Strictly left-to-right.  `A - B / C` evaluates as
`(A - B) / C`, not `A - (B / C)`.  Parentheses have no effect on precedence;
they are stripped before tokenisation and may be used as documentation only.

**Parse-time errors** (prevent `activate()` from succeeding):
- The divisor operator `/` with a literal `0` or `0.0` as the right operand.
- An unrecognised peripheral alias in a sensor term (e.g. `NOPE.alt`).

**Runtime error policy** — if any of the following conditions occur, an
`EVENT.warning` with code `SENSOR_FAILURE` is emitted and the target variable
is left **unchanged** (its previous value and validity are preserved):
- A sensor read fails (driver returns an error).
- A variable operand has not yet been set (`valid == false`).
- The result is ±∞ or NaN (e.g. runtime division by zero).

**Constraints:**
- A maximum of three terms (two operators) per expression.
- Each term is resolved at runtime; no constant folding is performed at parse time.
- Expressions may appear in `on_enter:`, `on_exit:`, and task `if:` set actions.
- The target variable is counted toward the `AMS_MAX_SET_ACTIONS` limit per block.

> *Implementation note:* The engine represents this form internally as
> `SetActionKind::EXPR` (value 5).

---

## AMS-4.9 Sensor Fault Tolerance

### AMS-4.9.1 Sensor Retry

The `retry=N` suffix on an `include` directive instructs the engine to retry the
HAL read up to N extra times (total attempts = N + 1) before treating a read as
failed.

```ams
include BMP280 as BARO retry=3
include BN220 as GPS  retry=2
```

- `N` must be in the range 1–`AMS_MAX_SENSOR_RETRY` (default 5).
- Retry is applied for every context where that alias is read: transitions,
  conditions, `set` actions, `LOG/HK` fields.

### AMS-4.9.2 Fallback Transition

A *fallback transition* fires unconditionally if no regular transition has fired
within a given number of milliseconds after entering the state.

```ams
state FLIGHT:
  transition to APOGEE when BARO.alt > 500
  fallback transition to SAFE after 300000ms
```

Rules:
- Exactly one fallback transition per state (duplicate is a parse error).
- The `after Nms` clause is mandatory; `N` must be a positive integer.
- The fallback target must be a declared state in the same script.
- The fallback fires **after** the regular transition check each tick.
- A state with only a fallback (no regular `transition`) is not terminal — the
  engine keeps ticking until the timeout fires.
- The fallback does **not** consume a TC token even if one is pending.

---

## AMS-4.10 Error Recovery

### AMS-4.10.1 On-Error Event

`on_error:` blocks may contain an `EVENT.*` directive (existing):

```ams
on_error:
  EVENT.error "guard violated"
```

### AMS-4.10.2 On-Error Transition

`on_error:` blocks may additionally contain `transition to STATE` to enter a
recovery state instead of halting the engine:

```ams
state FLIGHT:
  conditions:
    BARO.alt > 0
  on_error:
    EVENT.error "BARO failed"
    transition to RECOVERY
```

Rules:
- `EVENT.*` and `transition to` may coexist in the same `on_error:` block.
- Exactly one `transition to` per `on_error:` block (duplicate is a parse error).
- The target state must be declared in the same script.
- When the recovery transition fires, the engine enters the target state and
  **does not** enter `ERROR` status.  From the recovery state, normal tick
  semantics apply.
- If no `transition to` is present and a guard condition is violated, the engine
  still halts in `ERROR` (existing behaviour, AMS-5.6).

### AMS-4.10.3 On-Timeout Forced Transition

An `on_timeout Nms:` block forces a transition if no regular transition fires
within `N` milliseconds after entering the state.

```ams
state HOLD:
  transition to FLIGHT when TC.command == arm
  on_timeout 30000ms:
    EVENT.warning "arm timeout — aborting hold"
    transition to SAFE
```

Rules:
- `N` must be a positive integer (milliseconds); `0` is a parse error.
- Exactly one `on_timeout` block per state (duplicate is a parse error).
- A `transition to TARGET` directive is **required** inside the block.
- An optional `EVENT.info|warning|error "text"` may precede the transition.
- The target state must be declared in the same script.
- `on_timeout` fires **after** regular transitions are evaluated and **before**
  the fallback transition on each tick.
- `on_timeout` fires even if the engine has a fallback transition; they are
  independent mechanisms with different evaluation order.
- **Tie-breaking (equal threshold):** If `on_timeout Nms` and `fallback after Nms`
  are both defined with the same value of N, `on_timeout` always fires because
  it is evaluated at step 6 of AMS-5.1 before the fallback at step 7.
  The `fallback` target is then permanently unreachable from that state.
  Authors should use distinct values to express independent safety nets.
- When `on_timeout` fires: the optional EVENT is sent, `on_exit:` executes
  for the current state, then the engine enters the target state.
- The transition is resolved at parse time; an unknown target state is a
  parse error.

---

## AMS-5 Runtime Semantics

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

### AMS-4.8.9 Checkpoint Persistence (v3)

Variables and per-slot HK/LOG timers are persisted in the AMS checkpoint file (version 3):

```
3|fileName|stateIdx|exec|running|status|seq|stateElap|hkElap|logElap[|varCount|name1=val1=valid1|...|nameN=valN=validN]|hkSlotCount|hkSlotElap0|...|logSlotCount|logSlotElap0|...
```

On restore:
- Variables are matched by name against the loaded program's variable table.
- If a persisted variable is not found in the current script, it is silently skipped.
- Per-slot elapsed times are clamped to `AMS_MAX_HK_SLOTS` entries.
- v1/v2 checkpoints are still accepted; slot timers fall back to `nowMs` (conservative — next HK/LOG fires at the normal interval from restore time).

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
5. If transition fires:
   a. Consume TC token (if the firing condition included a `TC.command` match).
   b. Execute `on_exit:` set actions and EVENT for the current state (AMS-4.16).
   c. Enter next state: reset all per-state counters and timers; execute `on_enter:`
      set actions synchronously; fire `PULSE.fire` channels (AMS-4.17); queue
      `on_enter:` EVENT for next arbitration cycle (AMS-5.2).
   d. End tick.
6. If `on_timeout` is defined and elapsed time ≥ `onTimeoutMs`: emit on_timeout
   EVENT; execute `on_exit:` set actions and EVENT (AMS-4.16); enter timeout target
   state (steps c–d above); end tick (AMS-4.10.3).
7. If fallback transition is defined and elapsed time ≥ fallback threshold: execute
   `on_exit:` set actions and EVENT (AMS-4.16); enter fallback target state
   (steps c–d above); end tick (AMS-4.9.2).
   **Note:** When both `on_timeout` and `fallback` share the same threshold N,
   step 6 fires first; the fallback target is then unreachable (see AMS-4.10.3).
8. Evaluate guard conditions (`conditions:`)
9. If any condition is violated, emit `on_error` event (if defined) and set engine `ERROR`
   (or enter recovery state if `transition to` is defined — AMS-4.10.2).
10. Otherwise, evaluate due actions and run arbitration

### AMS-5.2 On-Enter Event Queueing

`on_enter` event is queued, not sent immediately.
It is consumed by arbitration as an EVENT due action.

> **Note (AMS-4.8):** `set` actions execute before the event is queued.
> The variable value is therefore available to the arbitration tick that
> dispatches the event.

### AMS-5.8 On-Exit Dispatch Order

`on_exit:` set actions and EVENT are executed **synchronously and immediately**
(not queued) inside `exitStateLocked()`, which runs **before**
`enterStateLocked()` of the incoming state.  The full ordering guarantee across
every transition site is:

```
[consume TC token if TC-triggered]
  → exitStateLocked(old state)
      ├─ on_exit: set actions  (synchronous)
      └─ on_exit: EVENT        (synchronous, transmitted immediately)
  → enterStateLocked(new state)
      ├─ reset per-state counters and timers
      ├─ on_enter: set actions (synchronous, AMS-4.8.2)
      ├─ PULSE.fire channels   (synchronous, AMS-4.17)
      └─ queue on_enter: EVENT (dispatched on next arbitration cycle, AMS-5.2)
```

This means the `on_exit:` EVENT frame is transmitted on the bus before the
new state's `on_enter:` event is queued, which in turn fires on the next
arbitration cycle.  TC token consumption precedes both exit and entry handlers.

### AMS-5.7 Variable NaN Guard

If a `set` or `CALIBRATE` action fails to produce a valid reading:

- The target variable retains its previous value (or remains invalid if never set).
- An `EVENT.warning` frame is queued.
- Execution continues; the engine does not enter ERROR state for a failed `set`.
- Transition/guard conditions that reference a still-invalid variable evaluate to
  **false** (condition cannot fire), providing safe default behaviour.

### AMS-5.3 TC One-Shot Consumption

`TC.command` token is consumed once when a matching transition fires.

### AMS-5.9 Adaptive Tick Scheduling

The main loop calls `MissionScriptEngine::nextWakeupMs(nowMs)` immediately
after each `tick()` to determine the next required wakeup timestamp, then
sleeps for exactly that duration instead of a fixed period.

Scheduling rules (evaluated in priority order):

| Condition | Sleep duration |
|---|---|
| Engine not running or status ≠ `RUNNING` | `SENSOR_RATE_MS` |
| `pendingOnEnterEvent_` set | `SENSOR_RATE_MS` |
| Active state has transitions or guard conditions | `SENSOR_RATE_MS` |
| Active state has no conditions | min(next HK/LOG slot, next task, `on_timeout`, `fallback`) capped at `kRadioMaxSleepMs` |

`kRadioMaxSleepMs = 50 ms` — the absolute upper bound on sleep duration.
This guarantees that a TC command (LAUNCH, ABORT) injected over radio is
processed within 50 ms even in states that have no sensor conditions.

**Normative implications for script authors:**
- States with at least one `transition` or `conditions:` line always tick at
  `SENSOR_RATE_MS` — latency for TC commands in those states is ≤ `SENSOR_RATE_MS`.
- Pure HK/LOG reporting states (no transitions, no conditions) may defer a
  tick by up to `kRadioMaxSleepMs`.  Scripts that require sub-50 ms ABORT
  response must add at least one transition condition to the relevant state.
- `nextWakeupMs()` is a scheduling hint — it does not affect correctness.
  The engine evaluates all due actions on every `tick()` regardless of the
  sleep duration actually used.

### AMS-5.4 HK Behavior

`HK.report` builds a binary telemetry frame and sends through the radio protocol
encoder (`MsgType::TELEMETRY`).

**COM routing (AMS-4.3.2):** By default every HK slot transmits through
`primaryCom_` — the first COM driver registered with the engine.  When a slot
is declared with `every Nms via ALIAS:`, the engine resolves the alias at
dispatch time and transmits through that driver instead.  If resolution fails
at dispatch time (driver not registered or alias mismatch), a `LOG_W` is
emitted and `primaryCom_` is used as fallback.

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
- Add `transition to SAFE when TC.command == RESET_ABNORMAL` in every critical
  state to handle automatic recovery from in-flight crashes or watchdog faults
  (see AMS-4.6.4).
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
| AMS-8.3 | Runtime control APIs are mutex-protected with timeout. File I/O (LittleFS checkpoint writes, CSV log appends) is staged inside the critical section and flushed via `flushPendingIoUnlocked()` after lock release, keeping blocking storage latency outside the mutex. `CALIBRATE` set actions are asynchronous (one sensor read per tick, via `stepPendingCalibrationsLocked`), so the per-tick mutex-hold time is bounded by a single sensor read (~25 ms). The `AMS_MUTEX_TIMEOUT_MS` value must exceed the worst-case non-I/O hold time (sensor reads only; CALIBRATE loop removed). |
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


---

## AMS-4.11 TC Debounce

### AMS-4.11.1 Default behavior

A `TC.command == VALUE` condition fires on the first call to `injectTcCommand`
that delivers the matching command.  The internal TC token is consumed immediately
after the transition fires.

### AMS-4.11.2 `once` modifier

Syntax: `TC.command == VALUE once`

Explicit one-shot; semantically identical to AMS-4.11.1.  Intended for scripts
where the intent must be self-documenting.

### AMS-4.11.3 `confirm N` modifier

Syntax: `TC.command == VALUE confirm N`  (N = 2�10)

The condition becomes true only after the TC has been injected at least N times.
An internal per-TC counter is incremented on every `injectTcCommand` call for
that command value.

Counter reset policy:
- Reset to 0 when a state transition fires the `CONFIRM` condition.
- Reset to 0 on every `enterStateLocked` (state change).

This behavior protects against spurious LoRa packet repetition while still
allowing intentional multi-press confirmation.

---

## AMS-4.12 Mission Constants

### AMS-4.12.1 Declaration

Syntax: `const NAME = VALUE`

- Must appear in the metadata section (before any `state` block).
- NAME follows the same identifier rules as `var` names (AMS-4.8).
- VALUE is a numeric literal (integer or float).
- At most `AMS_MAX_CONSTS` (8) constants per script.
- Constants are immutable: no `set` action or runtime mutation is allowed.

### AMS-4.12.2 Usage in conditions

A constant name may appear as the RHS of any transition or guard condition in
place of a numeric literal:

```
ALIAS.field OP CONSTANT_NAME
ALIAS.field OP (CONSTANT_NAME + offset)
ALIAS.field OP (CONSTANT_NAME - offset)
```

Constants are resolved and inlined into `CondExpr::threshold` at parse time.
There is no runtime overhead relative to a literal float value.

### AMS-4.12.3 Error conditions

| Condition | Error |
|---|---|
| Duplicate constant name | Parse error |
| Name conflicts with an existing `var` | Parse error |
| Name contains `.` | Parse error |
| Name is `TIME` or `TC` | Parse error |
| Value is not a numeric literal | Parse error |
| More than `AMS_MAX_CONSTS` constants | Parse error |

---

## AMS-4.13 Background Tasks

### AMS-4.13.1 Syntax

```
task <name> [when in <STATE> …]:
  every <N>ms:
    if <COND>:
      [EVENT.<verb> "<text>"]
      [set <var> = <expr>]
```

`<name>` is a human-readable identifier used in log messages.

### AMS-4.13.2 Execution model

A task fires when the elapsed time since its last execution equals or exceeds
the configured period. Tasks are evaluated inside `tick()` after
`executeDueActionsLocked()` and before `saveResumePointLocked()`.  The mutex
is held for the full duration; each `if`-rule is evaluated in declaration order.

### AMS-4.13.3 State filter

If `when in STATE…` is specified, the task only fires when `currentState_`
matches one of the listed states.  State names are resolved at parse time;
an unknown name is a parse error.  When the current state is outside the
filter, the per-task timer is reset so the task fires promptly on re-entry.

### AMS-4.13.4 Allowed actions inside task if-rules

| Action | Allowed |
|--------|---------|
| `EVENT.info/warning/error "text"` | Yes |
| `set VAR = ALIAS.field` (and derived forms) | Yes |
| State transitions | **No** |
| `TC.command` in condition | **No** |
| `CALIBRATE(…)` | Yes |
| `max(…)` / `min(…)` / `delta(…)` | Yes |

### AMS-4.13.5 Limits

| Parameter | Constant | Default |
|-----------|----------|---------|
| Max tasks | `AMS_MAX_TASKS` | 4 |
| Max rules per task | `AMS_MAX_TASK_RULES` | 4 |
| Max `when in` states | `AMS_MAX_TASK_ACTIVE_STATES` | 6 |
| Minimum period | `TELEMETRY_INTERVAL_MIN` | 100 ms |

Exceeding any limit is a parse error.

---

## AMS-4.14 Formal Assertions

### AMS-4.14.1 Purpose

The `assert:` block performs static analysis of the state graph at parse time.
If any assertion fails the script is rejected before activation.

### AMS-4.14.2 Syntax

```
assert:
  reachable <STATE>
  no_dead_states
  max_transition_depth < <N>
```

### AMS-4.14.3 Semantics

| Directive | Analysis |
|-----------|---------|
| `reachable STATE` | BFS from `state[0]`; error if `STATE` not reached |
| `no_dead_states` | BFS from `state[0]`; error if any declared state is unreachable |
| `max_transition_depth < N` | Iterative DFS (cycle-safe via path bitmask); error if longest acyclic path ≥ N |

### AMS-4.14.4 Algorithms

BFS uses a `uint32_t` reachability bitmask (hard ceiling: 32 states; `AMS_MAX_STATES = 16` is the current practical limit).  
DFS uses a bounded stack of `DfsFrame{state, depth, pathMask, child}` structs allocated
on the C++ stack — no heap.

### AMS-4.14.5 Limits

| Parameter | Constant | Default |
|-----------|----------|---------|
| Max asserts | `AMS_MAX_ASSERTS` | 8 |

More than `AMS_MAX_ASSERTS` directives in one `assert:` block is a parse error.  
Only one `assert:` block per script is allowed.

---

## AMS-4.15 Radio Configuration

### AMS-4.15.1 Purpose

The `radio.config` directive overrides compile-time default values for APUS
configuration parameters (ST[20]) from the script metadata section.
Overrides are applied by `RadioDispatcher` immediately after a successful
`ARM_FLIGHT` command, before the first telemetry tick.
Ground can still override any value afterwards via `SET_CONFIG_PARAM` (APUS-16.1).

### AMS-4.15.2 Syntax

```
radio.config PARAM_NAME = VALUE
```

One directive per line.  Must appear in the top-level metadata section
(before any `state[…]` block).  All tokens are case-sensitive.

### AMS-4.15.3 Parameter table

| PARAM_NAME         | APUS ConfigParamId    | Unit  | Valid range (inclusive) |
|--------------------|-----------------------|-------|-------------------------|
| `telem_interval`   | `TELEM_INTERVAL_MS`   | ms    | 100 – 60 000            |
| `monitor.alt.high` | `MONITOR_ALT_HIGH_M`  | m     | 0 – 15 000              |
| `monitor.alt.low`  | `MONITOR_ALT_LOW_M`   | m     | −500 – 1 000            |
| `monitor.accel.max`| `MONITOR_ACCEL_HIGH`  | m/s²  | 0 – 1 000               |
| `monitor.temp.high`| `MONITOR_TEMP_HIGH_C` | °C    | −40 – 150               |
| `monitor.temp.low` | `MONITOR_TEMP_LOW_C`  | °C    | −100 – 50               |

### AMS-4.15.4 Validation

Bounds are checked at **parse time** against the same named constants
(`MONITOR_*` in `config.h`) used by `RadioDispatcher::applyConfigParam()`
at runtime.  A value outside the allowed range is a parse error and causes
the script load to fail.

### AMS-4.15.5 Priority model

1. Compile-time defaults in `RadioDispatcher::configParams_[]` (lowest priority).
2. `radio.config` script overrides — applied at `ARM_FLIGHT`.
3. Ground `SET_CONFIG_PARAM` telecommands — applied on receipt (highest priority).

### AMS-4.15.6 Limits

| Parameter          | Value |
|--------------------|-------|
| Max directives     | 6 (one per `ConfigParamId`) |
| Duplicate handling | Last declaration wins       |

### AMS-4.15.7 Error conditions

| Condition                         | Engine response                       |
|-----------------------------------|---------------------------------------|
| Unknown `PARAM_NAME`              | Parse error; script rejected          |
| Value outside allowed range       | Parse error; script rejected          |
| Missing `=` or missing value      | Parse error; script rejected          |
| `radio.config` in a state block   | Parse error; script rejected          |
| More than 6 directives            | Last one per param wins; no error     |

---

## AMS-4.16 On-Exit Handler

An `on_exit:` block defines actions to execute synchronously when the engine
leaves a state via **any** transition — normal, fallback, or error-recovery.
It provides a symmetric counterpart to `on_enter:` for farewell telemetry,
variable snapshots, or state-departure audit events.

### AMS-4.16.1 Syntax

```ams
state FLIGHT:
  on_exit:
    set landing_alt = BARO.alt          -- optional set action(s)
    EVENT.info "Leaving FLIGHT"         -- optional departure event
```

Allowed inside `on_exit:`:
- `set VARNAME = EXPR` — same forms as `on_enter:` (AMS-4.8.2).  Up to
  `AMS_MAX_SET_ACTIONS` (4) set actions per block.
- `EVENT.info|warning|error "text"` — exactly zero or one event per block.
- `transition to` inside `on_exit:` is a **parse error**.

### AMS-4.16.2 Semantics

Execution order when a transition fires:
1. Execute `on_exit:` set actions (bounded loop, PO10-2).
2. Dispatch `on_exit:` EVENT synchronously (not queued — arrives on the bus
   before the new state's `on_enter:` event).
3. Enter the new state (`enterStateLocked`), which executes `on_enter:` set
   actions and queues the `on_enter:` event.

The `on_exit:` handler fires in all of the following transition sites:
| Transition type | Fires? |
|---|---|
| Normal condition-based transition | ✅ Yes |
| Fallback timeout transition (AMS-4.9.2) | ✅ Yes |
| Error-recovery transition (`on_error: transition to`) | ✅ Yes |
| Initial `activateLocked` (no previous state) | ❌ No |
| `deactivate()` | ❌ No |
| Engine enters ERROR with no recovery transition | ❌ No |
| Checkpoint restore (`RESET_ABNORMAL`) | ❌ No |

### AMS-4.16.3 Compliance

| Constraint | Implementation |
|---|---|
| PO10-2 Bounded loops | `for` loop iterates at most `AMS_MAX_SET_ACTIONS` (4) |
| PO10-3 No dynamic allocation | Static `onExitSetActions[]` array in `StateDef` |
| RTOS-4 Mutex-protected state | Called under `mutex_` inside `exitStateLocked` |

### AMS-4.16.4 Error conditions

| Condition | Error |
|---|---|
| `transition to` inside `on_exit:` | Parse error |
| More than one `EVENT.*` per block | Parse error |
| More than `AMS_MAX_SET_ACTIONS` set actions | Parse error |
| Unknown EVENT verb | Parse error |

---

## AMS-4.17 Pulse Channel Commands

Pulse (`PULSE.fire`) commands allow a mission script to activate a generic
electric-pulse channel synchronously when entering a state.  Commands are
placed in the `on_enter:` block of any state definition and are executed before
the `EVENT.*` telemetry frame is emitted.

> **Safety note**: Once a channel is marked as fired, the hardware driver
> rejects further fire requests for that channel for the lifetime of the power
> cycle.  This is an intentional safety interlock — a pulse channel must never
> receive a second activation.

Before a `PULSE.fire` command can be used, the channel must be **declared** in
the script metadata section using a `pulse.channel` directive (AMS-4.18).
Using an undeclared channel or unknown alias in `PULSE.fire` is a parse error.

### AMS-4.17.1 Syntax

```
on_enter:
    PULSE.fire <channel>
    PULSE.fire <channel> <duration>ms
```

| Token | Values | Description |
|---|---|---|
| `<channel>` | `A`, `B`, `C`, `D`, or a declared label | Channel letter or alias declared by `pulse.channel` (AMS-4.18) |
| `<duration>ms` | positive integer | Optional pulse width in milliseconds.  Omit to use `FIRE_DURATION_MS` (default 1000 ms). |

A maximum of `AMS_MAX_PULSE_ACTIONS` (4) `PULSE.fire` directives may appear in a
single `on_enter:` block.

**Examples:**

```ams
pulse.channel A as DROGUE
pulse.channel B as MAIN

state DEPLOY:
  on_enter:
    PULSE.fire DROGUE        # channel A at default duration
    PULSE.fire MAIN          # channel B at default duration
    EVENT.info "DEPLOYED"
```

```ams
pulse.channel A
pulse.channel C

state STAGE_SEP:
  on_enter:
    PULSE.fire A 500ms       # channel A with 500 ms pulse override
    PULSE.fire C 1000ms      # auxiliary channel C
```

### AMS-4.17.2 Semantics

When the engine transitions into a state that contains `PULSE.fire` directives,
`executePulseActionsLocked` is called during `sendOnEnterEventLocked` (before the
`EVENT.*` frame is sent).  For each directive:

1. The compile-time default `FIRE_DURATION_MS` is used if no override was given
   (`durationMs == 0` in the stored `PulseAction`).
2. `PulseInterface::fire(channel, durationMs)` is called on the registered driver.
3. On success, the corresponding status bit (`STATUS_PULSE_A_FIRED`,
   `STATUS_PULSE_B_FIRED`, `STATUS_PULSE_C_FIRED`, or `STATUS_PULSE_D_FIRED`)
   is set (visible via `getStatusBits()` and in APUS HK frames).
4. On failure (driver rejects the call), `setErrorLocked` is invoked and the
   engine transitions to `ERROR` status.

If no `PulseInterface` was registered (constructor `pulseIface = nullptr`), all
`PULSE.fire` directives are silently skipped and the engine continues normally.
This allows flight-software unit tests and ground-station builds to run without
pulse hardware.

### AMS-4.17.3 Compliance

| Standard | Clause | Requirement |
|---|---|---|
| PO10-3 | No heap allocation | `PulseAction` array is statically sized; no dynamic allocation |
| PO10-2 | Bounded loops | Loop iterates at most `AMS_MAX_PULSE_ACTIONS` (4) times |
| RTOS-4 | Mutex on critical sections | `executePulseActionsLocked` executes under the engine mutex |
| DO-178C | Deterministic execution | Fire sequence is synchronous and order-deterministic |
| APUS-6 | Pulse status in HK frames | `STATUS_PULSE_A/B/C/D_FIRED` bits reported |

### AMS-4.17.4 Error conditions

| Condition | Error |
|---|---|
| Unknown channel token or undeclared alias | Parse error: `PULSE.fire: unknown channel or undeclared alias` |
| Channel referenced before `pulse.channel` declaration | Parse error: `PULSE.fire: channel used but not declared with pulse.channel` |
| Channel token too long | Parse error: `PULSE.fire: channel token too long` |
| More than `AMS_MAX_PULSE_ACTIONS` directives in one block | Parse error: `too many PULSE.fire actions in on_enter block` |
| Duration is zero or not a positive integer | Parse error: `PULSE.fire: duration must be a positive integer` |
| Duration suffix missing `ms` | Parse error: `PULSE.fire: duration must be in the form Nms (e.g. 500ms)` |
| Driver rejects the fire request (e.g. channel already fired) | Runtime error: `PULSE.fire: driver rejected fire command` → engine ERROR |

---

## AMS-4.18 Pulse Channel Declaration

The `pulse.channel` directive declares a physical pulse channel for use in the
script.  It must appear in the **metadata section** (before any `state` block)
and serves two purposes:

1. **Intent declaration** — explicitly documents which channels the mission
   script will use, making safety review straightforward.
2. **Parse-time enforcement** — any `PULSE.fire` referencing an undeclared
   channel or unknown alias is rejected at `activate()` time with a parse error.

### AMS-4.18.1 Syntax

```
pulse.channel <letter>
pulse.channel <letter> as <label>
```

| Token | Values | Description |
|---|---|---|
| `<letter>` | `A`, `B`, `C`, or `D` | Physical channel identifier |
| `<label>` | alphanumeric + `_`, max 15 chars | Optional human-readable alias (e.g. `DROGUE`, `MAIN`) |

If the `as <label>` clause is omitted, the channel letter itself is used as the
label in log messages.

### AMS-4.18.2 Rules

| Rule | Detail |
|---|---|
| Must precede all `state` blocks | `pulse.channel` after the first `state:` line is a parse error |
| One declaration per channel | Duplicate `pulse.channel A` is a parse error |
| Labels must be unique | Two channels with the same label is a parse error |
| Label charset | `[A-Za-z0-9_]` only; other characters are a parse error |
| Label length | 1–15 characters (NUL-terminated in `AMS_PULSE_LABEL_LEN` = 16) |
| Undeclared use | `PULSE.fire` on an undeclared channel → parse error |
| Declared-but-unused | `pulse.channel` declared but never used in `PULSE.fire` → parser warning (LOG_W) |

### AMS-4.18.3 Physical mapping

Channel letters map to physical GPIO pins defined in `config.h`:

| Channel | Config constant | Default pin | Typical use |
|---|---|---|---|
| A | `PIN_PULSE_A` | 4  | Defined by `pulse.channel A as LABEL` in script |
| B | `PIN_PULSE_B` | 15 | Defined by `pulse.channel B as LABEL` in script |
| C | `PIN_PULSE_C` | 16 | Defined by `pulse.channel C as LABEL` in script |
| D | `PIN_PULSE_D` | 17 | Defined by `pulse.channel D as LABEL` in script |

### AMS-4.18.4 Examples

```ams
# Minimal: bare channel letters
pulse.channel A
pulse.channel B

state DEPLOY:
  on_enter:
    PULSE.fire A
    PULSE.fire B
```

```ams
# With aliases
pulse.channel A as DROGUE
pulse.channel B as MAIN
pulse.channel C as STAGE_SEP

state BOOST:
  on_enter:
    PULSE.fire STAGE_SEP 200ms

state APOGEE:
  on_enter:
    PULSE.fire DROGUE
    PULSE.fire MAIN 1500ms
```

### AMS-4.18.5 Compliance

| Standard | Clause | Requirement |
|---|---|---|
| PO10-3 | No heap allocation | `PulseChannelDecl` array is statically sized (`PulseChannel::COUNT` = 4) |
| DO-178C | Traceable requirements | Declaration provides explicit audit trail for pyrotechnic use |
| CERT | Validate all inputs | Label charset and length validated at parse time |

### AMS-4.18.6 Error conditions

| Condition | Error |
|---|---|
| Channel letter not A/B/C/D | Parse error: `pulse.channel: channel must be A, B, C, or D` |
| Characters directly after channel letter (not `\0` or space) | Parse error: `pulse.channel: unexpected characters after channel letter` |
| Space-separated token after channel letter that is not `as` (e.g. `pulse.channel A FOO`) | Parse error: `pulse.channel: unexpected token after channel letter; expected end-of-line or 'as LABEL'` |
| Duplicate declaration | Parse error: `pulse.channel: duplicate declaration for the same channel` |
| Empty label after `as` | Parse error: `pulse.channel: empty label after 'as'` |
| Invalid character in label | Parse error: `pulse.channel: label contains invalid character` |
| Label too long | Parse error: `pulse.channel: label exceeds maximum length` |
| Label already used by another channel | Parse error: `pulse.channel: label already used by another channel` |
| `pulse.channel` after a `state` block | Parse error: `'pulse.channel' must appear before any state block` |

---

## AMS-4.19 Pulse Safety System

The pulse safety system provides five independent safety gates that must all be
satisfied before `PULSE.fire` sends electrical energy to a pyrotechnic output
channel. All directives are **pre-state** (they must appear before the first
`state` block). All limits are enforced at **parse time** where possible; the
remaining limits are enforced at **runtime** on every `PULSE.fire` call.

### Overview

| Directive | AMS section | Gate type |
|---|---|---|
| `pulse.safe_delay <ms>` | AMS-4.19.5 | Time — minimum elapsed since `arm()` |
| `pulse.min_altitude <m>` | AMS-4.19.2 | Sensor — barometric altitude |
| `pulse.require_continuity <ch>` | AMS-4.19.4 | Hardware — bridgewire continuity pin |
| `PULSE.arm <ch>` (action) | AMS-4.19.1 | Logical — two-phase arm–fire |
| `pulse.arm_timeout <ms>` | AMS-4.19.3 | Time — auto-disarm after N ms |
| `pulse.no_baro_policy allow\|block` | AMS-4.19.6 | Policy — behaviour when no baro driver |

All gates are evaluated in the order listed above. If any gate rejects the
fire, the pulse is suppressed silently and a warning is logged via the event
service; **no error state is entered**.

All pulse safety runtime state — arm tokens, arm timestamps, the activation
timestamp used by `pulse.safe_delay`, and the per-channel fired-status flags —
is **reset to its initial value on every call to `deactivate()`**. This means a
mission script that is deactivated and then re-activated with the same file
starts each session with a clean slate; status bits reported by `getStatusBits()`
reflect only the current session.

---

### AMS-4.19.1 Two-phase arm–fire (`PULSE.arm`)

#### Purpose

Prevents accidental pyrotechnic discharge by requiring an explicit arm action
in an `on_enter` block before any `PULSE.fire` on the same channel is allowed.

#### Syntax (action inside an `on_enter` block)

```
PULSE.arm <channel>
```

Where `<channel>` is a raw letter (A–D) or a label declared by `pulse.channel`.

#### Rules

1. `PULSE.arm` may only appear inside an `on_enter` block.
2. If `PULSE.arm <ch>` appears anywhere in the script, every subsequent
   `PULSE.fire <ch>` in the same or a later state **requires** a prior arm.
3. The arm token is consumed (cleared) immediately after a successful
   `PULSE.fire`.
4. `PULSE.arm` and `PULSE.fire` may appear in the same `on_enter` block;
   arm actions are executed before fire actions.
5. If `pulse.arm_timeout` is declared, the arm token expires after the
   specified number of milliseconds (AMS-4.19.3).

#### Examples

```ams
state ARM:
  on_enter:
    PULSE.arm DROGUE       // sets arm token for DROGUE channel

state FIRE:
  on_enter:
    PULSE.fire DROGUE      // only executes if arm token is present
```

#### Error conditions

| Condition | Error |
|---|---|
| `PULSE.arm` outside `on_enter` | Parse error: `PULSE.arm only allowed inside on_enter` |
| Channel not declared | Parse error: `PULSE.arm: channel not declared` |

---

### AMS-4.19.2 Minimum altitude (`pulse.min_altitude`)

#### Purpose

Prevents pyrotechnic discharge below a configurable barometric altitude,
protecting against ground-level or low-altitude accidental triggering.

#### Syntax

```
pulse.min_altitude <altitude_m>
```

Where `<altitude_m>` is an integer in the range **[1, 50 000]** metres.

#### Rules

1. Must appear in the script metadata section (before the first `state` block).
2. Only one `pulse.min_altitude` directive is allowed per script.
3. At runtime, the altitude gate is evaluated using the first declared barometric
   driver (`SIM_BARO` or a real `BARO` include). If the driver returns an error
   or the current altitude is below the threshold, the fire is suppressed.
4. The altitude is compared against the **absolute barometric altitude above
   sea level** reported by the driver (not AGL).
5. If no barometric driver is registered in the engine at construction time
   (`baroCount = 0`), the altitude gate **always suppresses fire** regardless of
   the threshold value. A `W`-level warning is emitted:
   `PULSE ch=N blocked: pulse.min_altitude set but no baro driver registered`.

#### Examples

```ams
pulse.channel A as DROGUE
pulse.min_altitude 300        // PULSE.fire allowed only at or above 300 m ASL
```

#### Error conditions

| Condition | Error |
|---|---|
| Value < 1 | Parse error: `pulse.min_altitude: value out of range [1, 50000]` |
| Value > 50 000 | Parse error: `pulse.min_altitude: value out of range [1, 50000]` |
| Directive after a `state` block | Parse error: `'pulse.min_altitude' must appear before any state block` |
| Duplicate directive | Parse error: `pulse.min_altitude: duplicate directive` |

---

### AMS-4.19.3 Arm timeout (`pulse.arm_timeout`)

#### Purpose

Limits the window during which a channel is considered armed. If the arm token
is not consumed by a `PULSE.fire` within `<ms>` milliseconds, it is automatically
cleared, preventing stale arming from persisting across unexpected delays.

#### Syntax

```
pulse.arm_timeout <ms>
```

Where `<ms>` is an integer in the range **[500, 300 000]** milliseconds.

#### Rules

1. Must appear before the first `state` block.
2. `pulse.arm_timeout` applies **globally** to all channels in the script.
3. The timeout is checked on every `engine.tick()` call; expired arm tokens are
   cleared at the start of each tick before state actions are evaluated.
4. If `PULSE.arm` is never declared, `pulse.arm_timeout` has no effect.

#### Examples

```ams
pulse.channel A as DROGUE
pulse.arm_timeout 5000          // arm expires after 5 s if PULSE.fire not reached
```

#### Error conditions

| Condition | Error |
|---|---|
| Value < 500 | Parse error: `pulse.arm_timeout: value out of range [500, 300000]` |
| Value > 300 000 | Parse error: `pulse.arm_timeout: value out of range [500, 300000]` |
| Directive after a `state` block | Parse error: `'pulse.arm_timeout' must appear before any state block` |
| Duplicate directive | Parse error: `pulse.arm_timeout: duplicate directive` |

---

### AMS-4.19.4 Continuity check (`pulse.require_continuity`)

#### Purpose

Verifies electrical continuity of the bridgewire (e.g., an e-match or
initiator) before allowing discharge. A broken circuit (open bridgewire) will
suppress the fire, preventing energy release into a disconnected load.

#### Hardware wiring

The continuity-sense GPIO for each channel is set in `src/config.h` via
`PIN_PULSE_x_CONT`.  With only four pulse GPIOs available, a common pattern is
to reassign unused fire channels as sense inputs — set the unwanted fire pin to
`PIN_NO_FIRE` and assign its GPIO number to `PIN_PULSE_y_CONT`:

```cpp
// 2-fire + 2-cont example (src/config.h)
constexpr uint8_t PIN_PULSE_A = 4;           // A fires
constexpr uint8_t PIN_PULSE_B = 15;          // B fires
constexpr uint8_t PIN_PULSE_C = PIN_NO_FIRE; // GPIO 16 freed for cont
constexpr uint8_t PIN_PULSE_D = PIN_NO_FIRE; // GPIO 17 freed for cont
constexpr uint8_t PIN_PULSE_A_CONT = 16;     // bridgewire return for A
constexpr uint8_t PIN_PULSE_B_CONT = 17;     // bridgewire return for B
```

See [hardware/pin_map.md](../hardware/pin_map.md) for wiring details.

#### Syntax

```
pulse.require_continuity <channel>
```

Where `<channel>` is a raw channel letter (A–D) or a declared label.

#### Rules

1. Must appear before the first `state` block.
2. The directive is validated **at parse time**: if the hardware continuity pin
   constant for the referenced channel is `PIN_NO_CONT` (0xFF), the script
   activation fails with an error. This prevents deploying a script that relies
   on a hardware feature that is not wired.
3. Multiple channels can each have their own `pulse.require_continuity` line.
4. At runtime, the continuity pin is sampled at the moment `PULSE.fire` is
   evaluated; if the pin reads as open, the fire is suppressed.

#### Examples

```ams
pulse.channel A as DROGUE
pulse.require_continuity A    // PULSE.fire on A blocked if bridgewire is open
```

#### Error conditions

| Condition | Error |
|---|---|
| Channel has no hardware continuity pin wired | Parse error: `pulse.require_continuity: channel A has no hardware continuity pin` |
| Channel not declared | Parse error: `pulse.require_continuity: channel not declared` |
| Directive after a `state` block | Parse error: `'pulse.require_continuity' must appear before any state block` |

---

### AMS-4.19.5 Safe delay (`pulse.safe_delay`)

#### Purpose

Enforces a minimum elapsed time between the moment `engine.arm()` is called and
the moment any `PULSE.fire` is evaluated. This prevents accidental discharge
immediately after the mission engine is armed (e.g., due to spurious transitions
or a race condition in the initial state).

#### Syntax

```
pulse.safe_delay <ms>
```

Where `<ms>` is an integer in the range **[100, 60 000]** milliseconds.

#### Rules

1. Must appear before the first `state` block.
2. The safe delay is measured from the instant `engine.arm()` is called, using
   the monotonic 64-bit millisecond counter (`millis64()`).
3. The gate applies to **all** channels declared in the script.
4. Once the delay has elapsed it does not re-arm; it is a one-shot guard.

#### Examples

```ams
pulse.channel A as DROGUE
pulse.safe_delay 2000          // no PULSE.fire allowed within first 2 s of arm()
```

#### Error conditions

| Condition | Error |
|---|---|
| Value < 100 | Parse error: `pulse.safe_delay: value out of range [100, 60000]` |
| Value > 60 000 | Parse error: `pulse.safe_delay: value out of range [100, 60000]` |
| Directive after a `state` block | Parse error: `'pulse.safe_delay' must appear before any state block` |
| Duplicate directive | Parse error: `pulse.safe_delay: duplicate directive` |

---

### AMS-4.19.6 No-baro policy (`pulse.no_baro_policy`)

#### Purpose

Controls the behaviour of the `pulse.min_altitude` gate when no barometric
driver is registered in the engine. Because barometric hardware may be absent
in some configurations (e.g. a propulsion board without an altitude sensor),
the script author can explicitly choose what is safer for their deployment:
block fire until baro is available, or allow fire and rely on the other safety
gates.

#### Syntax

```
pulse.no_baro_policy allow|block
```

| Value | Behaviour when no baro driver is registered |
|---|---|
| `block` | `PULSE.fire` is suppressed; `W`-level warning logged. **(Default when directive is absent.)** |
| `allow` | The altitude gate is skipped; `I`-level note logged; other gates still apply. |

#### Rules

1. Must appear before the first `state` block.
2. Only effective when `pulse.min_altitude` is also declared. If
   `pulse.min_altitude` is absent, this directive is accepted but has no
   runtime effect.
3. When the directive is absent, the default policy is `block`.

#### Examples

```ams
// Flight test without a baro sensor — altitude check skipped if baro absent
pulse.channel A as DROGUE
pulse.min_altitude    200
pulse.no_baro_policy  allow
```

```ams
// Production flight — explicitly require baro (same as default)
pulse.channel A as DROGUE
pulse.min_altitude    500
pulse.no_baro_policy  block
```

#### Error conditions

| Condition | Error |
|---|---|
| Value is neither `allow` nor `block` | Parse error: `pulse.no_baro_policy: invalid value (expected 'allow' or 'block')` |
| Directive after a `state` block | Parse error: `'pulse.no_baro_policy' must appear before any state block` |
| Duplicate `pulse.no_baro_policy` directive | Parse error: `pulse.no_baro_policy: duplicate directive (only one allowed per script)` |

---

### AMS-4.19.7 Combined example

The following script activates all six safety directives on channel A:

```ams
include GPS as GPS
include BARO as BARO
include COM as COM
include IMU as IMU

pus.apid = 42
pus.service 3 as HK
pus.service 5 as EVENT
pus.service 1 as TC

pulse.channel A as DROGUE

pulse.safe_delay    3000        // AMS-4.19.5: no fire for 3 s after arm()
pulse.min_altitude  500         // AMS-4.19.2: must be at or above 500 m MSL
pulse.no_baro_policy block      // AMS-4.19.6: block fire if baro absent (default, explicit here)
pulse.require_continuity A      // AMS-4.19.4: bridgewire must be intact
pulse.arm_timeout   10000       // AMS-4.19.3: arm token expires after 10 s

state WAIT:
  on_enter:
    EVENT.info "Waiting for launch command"
  transition to ARM when TC.command == LAUNCH

state ARM:
  on_enter:
    PULSE.arm DROGUE            // AMS-4.19.1: set arm token
    EVENT.info "Channel armed"
  transition to FIRE when TC.command == FIRE

state FIRE:
  on_enter:
    PULSE.fire DROGUE           // all 5 gates evaluated here
    EVENT.info "Drogue deployed"
```

---

## AMS-4.20 Buzzer Output

The buzzer subsystem provides non-blocking acoustic feedback from AMS scripts
through a hardware-abstracted `BuzzerInterface` HAL.  Two driver implementations
are available: `ActiveBuzzerDriver` (GPIO on/off) and `PassiveBuzzerDriver`
(ESP32 LEDC PWM).  The script author uses the same syntax for both.

### Overview

| Feature | AMS section |
|---|---|
| `BUZZER.beep Nms` — single beep on state entry | AMS-4.20.1 |
| `BUZZER.beep Nms Fhz` — beep with explicit frequency | AMS-4.20.1 |
| `BUZZER.beep Nms [Fhz] Rx` — multi-beep repeat pattern | AMS-4.20.2 |
| `BUZZER.beep` inside `every:` cadence slot | AMS-4.20.3 |

---

### AMS-4.20.1 Single beep on state entry

#### Purpose

Triggers one buzzer pulse when the engine enters a state, providing a
deterministic acoustic signal at a defined mission phase transition.

#### Syntax

```
BUZZER.beep <duration>ms [<frequency>hz]
```

| Part | Required | Constraints |
|---|---|---|
| `<duration>ms` | Yes | Integer; `BUZZER_MIN_DURATION_MS` (50) – `BUZZER_MAX_DURATION_MS` (10 000). |
| `<frequency>hz` | No | Integer; `BUZZER_MIN_FREQ_HZ` (100) – `BUZZER_MAX_FREQ_HZ` (10 000). Omit to use driver default. Active buzzers ignore this field. |

#### Rules

1. `BUZZER.beep` may appear inside `on_enter:` blocks only (in this form).
2. At most `AMS_MAX_BUZZER_ACTIONS` (2) `BUZZER.beep` lines are allowed per
   `on_enter:` block; a third line is a parse error.
3. Actions are dispatched in declaration order via
   `BuzzerInterface::beep(durationMs, freqHz, count=1)`.  Each call is
   **non-blocking**; a FreeRTOS timer silences the buzzer after the period.
4. If no `BuzzerInterface*` is registered (null), all `BUZZER.beep` actions
   are silently skipped at runtime — not an error.
5. `BUZZER.beep` actions are suppressed when `executionEnabled_ == false`.

#### Examples

```ams
state DEPLOY:
  on_enter:
    BUZZER.beep 500ms
```

```ams
state DEPLOY:
  on_enter:
    BUZZER.beep 300ms 3000hz    // passive buzzer at 3 kHz
```

#### Error conditions

| Condition | Error |
|---|---|
| Duration absent or not followed by `ms` | `BUZZER.beep: missing or malformed duration (expected Nms)` |
| Duration out of range | `BUZZER.beep: duration N ms is out of range [50, 10000]` |
| Frequency out of range | `BUZZER.beep: frequency N hz is out of range [100, 10000]` |
| More than 2 lines in same `on_enter:` | `too many BUZZER.beep actions (max AMS_MAX_BUZZER_ACTIONS)` |

---

### AMS-4.20.2 Repeat count (`Nx`)

#### Purpose

Plays R consecutive beeps with equal-length silent gaps, all non-blocking,
using a single FreeRTOS timer state machine (TONE/GAP phases).  Useful for
coded acoustic signals (e.g. three short beeps at deployment).

#### Syntax

```
BUZZER.beep <duration>ms [<frequency>hz] <count>x
```

| Part | Required | Constraints |
|---|---|---|
| `<count>x` | No (default = 1) | Integer 1 – `BUZZER_MAX_REPEAT_COUNT` (8). 0 or > 8 is a parse error. |

The `<count>x` token must follow the duration (and optional frequency if
present). Argument order: `Nms [Fhz] [Rx]`.

#### Rules

1. Valid in both `on_enter:` and `every:` contexts (see AMS-4.20.3).
2. When `count > 1` the driver emits the following sequence:
   - Beep for `durationMs`.
   - Gap (silence) for `durationMs`.
   - Repeat (beep + gap) until `count` beeps have been played.
3. The total acoustic duration is `count × 2 × durationMs − durationMs`
   (last gap is omitted by convention in the driver implementation).
4. A new `BUZZER.beep` call issued while a multi-beep sequence is still
   running cancels the current sequence and starts the new one immediately.

#### Examples

```ams
state DEPLOY:
  on_enter:
    BUZZER.beep 200ms 3x        // 3 beeps, 200 ms each, 200 ms gaps
```

```ams
state ABORT:
  on_enter:
    BUZZER.beep 100ms 2500hz 5x // 5 beeps at 2.5 kHz
```

#### Error conditions

| Condition | Error |
|---|---|
| `<count>` is 0 or > 8 | `BUZZER.beep: repeat count N is out of range [1, 8]` |

---

### AMS-4.20.3 Periodic buzzer in `every:` cadence slots

#### Purpose

Allows the buzzer to fire once per cadence tick, co-located with or
independent of HK telemetry reporting.  Typical use: periodic descent
beep during recovery for acoustic tracking.

#### Syntax

`BUZZER.beep` may appear at state scope immediately after an `every Nms:`
header or after a `HK.report { }` block that belongs to that cadence scope:

```ams
// Alongside HK telemetry
every 1000ms:
  HK.report { baro_alt: BARO.alt }
  BUZZER.beep 100ms

// Buzzer-only slot (no HK frame sent)
every 500ms:
  BUZZER.beep 150ms 2x
```

#### Rules

1. At most **one** `BUZZER.beep` per `every:` slot; a second occurrence in
   the same slot is a parse error.
2. The buzzer fires **before** the HK telemetry frame is built and
   transmitted.  If the slot has no `HK.report` fields, no radio packet
   is sent — only the buzzer fires.
3. A buzzer-only slot (no `HK.report {}`) is valid.  It participates in
   the cadence scheduler (`hkSlotDue`) because `hasBuzzerAction == true`
   is sufficient to mark it as due (see AMS-4.3.1).
4. The `executionEnabled_` guard and `status_ == RUNNING` check apply;
   cadence-slot buzzer calls are suppressed when execution is disabled.
5. The `via ALIAS:` routing keyword applies only to the HK radio path.
   `BUZZER.beep` is not affected by `via`.

#### Examples

```ams
state DESCENT:
  every 2000ms:
    HK.report { baro_alt: BARO.alt  gps_lat: GPS.lat }
    BUZZER.beep 200ms           // fires every 2 s alongside HK frame

state RECOVERY:
  every 500ms:
    BUZZER.beep 100ms 3x        // 3-beep pattern every 500 ms, no HK sent
```

#### Error conditions

| Condition | Error |
|---|---|
| Second `BUZZER.beep` in same `every:` slot | `BUZZER.beep: slot already has a buzzer action` |

---

### AMS-4.20.4 Configuration constants (`src/config.h`)

| Constant | Default | Description |
|---|---|---|
| `PIN_BUZZER` | `0xFFU` | GPIO pin. `0xFF` = not connected (no-op driver path). |
| `BUZZER_LEDC_CHANNEL` | `0U` | ESP32 LEDC channel (passive buzzer only). |
| `BUZZER_DEFAULT_FREQ_HZ` | `2000U` | Default frequency when `<frequency>hz` is omitted. |
| `BUZZER_MIN_DURATION_MS` | `50U` | Minimum accepted duration. |
| `BUZZER_MAX_DURATION_MS` | `10000U` | Maximum accepted duration. |
| `BUZZER_MIN_FREQ_HZ` | `100U` | Minimum accepted frequency. |
| `BUZZER_MAX_FREQ_HZ` | `10000U` | Maximum accepted frequency. |
| `BUZZER_MAX_REPEAT_COUNT` | `8U` | Maximum repeat count for `Nx` syntax. |
| `AMS_MAX_BUZZER_ACTIONS` | `2U` | Maximum `BUZZER.beep` lines per `on_enter:` block. |

---

## AMS-4.21 Runtime Data Alias (`RUNTIME`)

`RUNTIME` is a reserved virtual alias that can be used wherever `ALIAS.field`
expressions are accepted (report fields, guards/conditions, transition
conditions, and `set` expressions).

### AMS-4.21.1 Supported fields

- `RUNTIME.uptime_ms`
- `RUNTIME.state_idx`
- `RUNTIME.state_elapsed_ms`
- `RUNTIME.script_elapsed_ms`
- `RUNTIME.status_bits`
- `RUNTIME.engine_status`
- `RUNTIME.exec_enabled`

### AMS-4.21.2 Reporting semantics

- In `LOG.report` and `SERIAL.report`, all `RUNTIME.*` fields are emitted as
  numeric values.
- In `HK.report` (binary APUS frame), only fields with native
  `TelemetryPayload` slots are serialized directly:
  - `RUNTIME.uptime_ms` -> `timestampMs`
  - `RUNTIME.state_idx` -> `flightPhase`
  - `RUNTIME.status_bits` -> `statusBits`
- Remaining runtime fields are valid in script grammar but have no dedicated
  slot in fixed `TelemetryPayload` v2 (78 bytes).

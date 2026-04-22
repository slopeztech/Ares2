# ARES DO-178C Standard (Adapted)

**Amateur Rocket Embedded System**
Safety-critical development process inspired by DO-178C /
RTCA DO-178C (Software Considerations in Airborne Systems and
Equipment Certification, 2011).
Companion to the [ARES Coding Standard](ares_coding_standard.md).

---

## Context

DO-178C is the industry standard for safety-critical avionics software.
ARES adapts its principles to amateur scale — not for formal
certification, but because the discipline prevents failures that
endanger people and hardware. A parachute that does not deploy, a pyro
that fires on the pad, or a flight computer that locks up mid-flight
are unacceptable outcomes that process rigor can prevent.

### Design Assurance Level

ARES targets **DAL-C equivalent** (major failure condition):

| DO-178C Objective Category         | DAL-C Applicability | ARES Practice              |
|------------------------------------|---------------------|----------------------------|
| Planning                           | Required            | DO-9                       |
| Requirements development           | Required            | DO-1                       |
| Requirements traceability          | Required            | DO-2                       |
| Software design                    | Required            | DO-3                       |
| Source code standards              | Required            | PO10, MISRA, CERT          |
| Configuration management           | Required            | DO-4                       |
| Reviews and analyses               | Required            | DO-5, DO-13                |
| Requirements-based testing         | Required            | DO-5                       |
| Structural coverage (statement)    | Required            | DO-11                      |
| Structural coverage (decision)     | With justification  | DO-11                      |
| Structural coverage (MC/DC)        | Not required         | —                          |
| Problem reporting                  | Required            | DO-8                       |
| Quality assurance                  | Required            | DO-13                      |
| Tool qualification                 | With justification  | DO-12                      |
| Transition criteria                | Required            | DO-17                      |

### Scope

This standard governs the **process** by which ARES firmware is
planned, developed, verified, and maintained. It does not duplicate
coding rules (covered by PO10, MISRA, CERT, RTOS) but defines the
engineering framework around them.

---

## DO-1 — Requirements-Based Development

**DO-178C §5.1 — Software Requirements Process**

Every line of code must trace back to a documented, verifiable
requirement. Requirements are the single source of truth for what
the system shall do.

### DO-1.1 — Requirement format

Requirements live in `docs/requirements/` as numbered items using
the format:

```
REQ-<subsystem>-<NNN>: <shall-statement>
  Rationale : <why this requirement exists>
  Acceptance: <measurable pass/fail criterion>
  Priority  : CRITICAL | HIGH | MEDIUM | LOW
```

Example:

```
REQ-FSM-001: The system shall detect launch when acceleration
             exceeds LAUNCH_ACCEL_THRESHOLD for at least
             LAUNCH_CONFIRM_MS milliseconds.
  Rationale : Prevent false trigger from vibration or handling.
  Acceptance: Unit test injects 3g for 200 ms → state = POWERED_ASCENT.
  Priority  : CRITICAL
```

### DO-1.2 — Requirement categories

| Category        | Prefix     | Example                              |
|-----------------|------------|--------------------------------------|
| Functional      | `REQ-FSM`  | State transitions, flight logic      |
| Safety          | `REQ-SAF`  | Pyro interlocks, abort conditions    |
| Performance     | `REQ-PERF` | Sensor rate, loop period, latency    |
| Interface       | `REQ-IF`   | UART protocol, SPI timing, radio API |
| Data integrity  | `REQ-DAT`  | CRC, ECC, redundant storage          |
| Environmental   | `REQ-ENV`  | Temperature, vibration, altitude     |

### DO-1.3 — Derived requirements

Requirements that arise from design decisions (not from the mission)
must be marked `[DERIVED]` and justified:

```
REQ-PERF-012 [DERIVED]: The sensor task shall run at 100 Hz.
  Rationale : IMU Nyquist at 50 Hz vibration mode.
```

### DO-1.4 — Fundamental rule

> **No code without a requirement. No requirement without a test.**

Code comments reference requirement IDs: `// REQ-FSM-001`.

---

## DO-2 — Traceability

**DO-178C §5.5 — Traceability**

Bidirectional traceability must exist between all levels:
requirements ↔ design ↔ code ↔ tests.

### DO-2.1 — Source file headers

Each source file header must list the requirement IDs it implements:

```cpp
/**
 * @file  flight_state.cpp
 * @brief Flight state machine.
 * @reqs  REQ-FSM-001, REQ-FSM-002, REQ-SAF-003
 */
```

### DO-2.2 — Test case headers

Test cases must reference the requirement(s) they verify:

```cpp
/**
 * @test  test_launch_detection
 * @verifies REQ-FSM-001
 */
TEST(FlightState, DetectsLaunch) { ... }
```

### DO-2.3 — Traceability matrix

The file `docs/traceability.md` maintains the full bidirectional
mapping:

| Requirement   | Design Artifact          | Source File(s)         | Test Case(s)              | Status     |
|---------------|--------------------------|------------------------|---------------------------|------------|
| REQ-FSM-001   | `docs/architecture.md`   | `flight_state.cpp`     | `test_launch_detection`   | Verified   |
| REQ-SAF-003   | `docs/architecture.md`   | `pyro_driver.cpp`      | `test_pyro_interlock`     | Verified   |

### DO-2.4 — Traceability gaps

A requirement with no test is **untested**. A test with no
requirement is **unanchored**. Both are defects that must be
resolved before release.

---

## DO-3 — Software Design Levels

**DO-178C §5.2 — Software Design Process**

Three levels of design, each documented:

| Level    | Artifact                  | Content                                              |
|----------|--------------------------|------------------------------------------------------|
| High     | `docs/architecture.md`   | System decomposition, task model, data flow, FSM diagram |
| Low      | Module header (`.h`)     | Public API, data types, pre/postconditions, error codes  |
| Code     | Implementation (`.cpp`)  | Algorithms, state transitions, inline `@reqs` tags       |

### DO-3.1 — High-level design

The high-level design must include:

- **System block diagram**: modules, tasks, hardware interfaces.
- **Data flow diagram**: sensor → processing → actuation → telemetry.
- **RTOS task table**: task names, priorities, periods, stack sizes
  (see RTOS-2, RTOS-5).
- **Inter-task communication map**: queues, semaphores, event groups
  (see RTOS-4).
- **FSM state diagram**: all flight states and transitions with
  guard conditions.

### DO-3.2 — Low-level design

Each module header (`.h`) serves as low-level design and must contain:

- Module purpose (one-sentence summary).
- Public API: function signatures with `@pre` / `@post` / `@param`
  / `@return` documentation.
- Data types: enums, structs with field descriptions.
- Error codes returned by each function.
- Thread-safety annotations: which functions are ISR-safe,
  which require mutex, which are single-task only.

### DO-3.3 — Design-to-code consistency

- Every public function declared in a `.h` must be implemented in
  the corresponding `.cpp`.
- No public function may exist without a header declaration.
- No "surprise" side effects: if a function modifies shared state,
  it must be documented in the header.

---

## DO-4 — Configuration Management

**DO-178C §7 — Software Configuration Management Process**

All software artifacts (source, tests, docs, configs) are under
version control (git). Configuration management ensures
reproducibility and auditability.

### DO-4.1 — Commit discipline

- Every change is a git commit with a descriptive message.
- Commit messages reference requirement IDs, issue numbers, or
  rule IDs:
  ```
  fix(fsm): debounce launch detection [REQ-FSM-001] (#42)
  ```
- Atomic commits: one logical change per commit.
- No uncommitted code flies. Period.

### DO-4.2 — Branching strategy

| Branch         | Purpose                           | Merge to    |
|----------------|-----------------------------------|-------------|
| `main`         | Release-ready, tested             | —           |
| `develop`      | Integration branch                | `main`      |
| `feature/*`    | New features                      | `develop`   |
| `fix/*`        | Bug fixes                         | `develop`   |
| `release/*`    | Release candidate stabilization   | `main`      |

### DO-4.3 — Baselines and releases

- Tagged releases follow semantic versioning: `vMAJOR.MINOR.PATCH`.
- Each tag has a changelog entry in `CHANGELOG.md`.
- A release baseline includes: source, compiled binary (`.bin`),
  partition table, documentation snapshot, and test results.
- **Flight firmware** must only be built from a tagged release on
  `main` — never from a development branch.

### DO-4.4 — Build reproducibility

- `platformio.ini` pins exact framework and library versions.
- The build environment (PlatformIO version, toolchain version)
  is documented in each release.
- The same commit must produce the same binary (deterministic build).

---

## DO-5 — Verification Strategy

**DO-178C §6 — Software Verification Process**

Verification demonstrates that the software satisfies its
requirements and that the design is correctly implemented. ARES uses
a multi-layer verification approach.

### DO-5.1 — Verification layers

| Layer               | Method                        | Timing           | Rule          |
|---------------------|-------------------------------|-------------------|---------------|
| L0 — Compile-time   | `-Wall -Wextra -Werror`       | Every build       | PO10-10       |
| L1 — Static analysis| Compiler warnings + lint      | Every build       | PO10-10       |
| L2 — Unit tests     | PlatformIO `test/` framework  | Every commit      | DO-5.2        |
| L3 — Integration    | Multi-module tests            | Every merge       | DO-5.3        |
| L4 — HIL tests      | Hardware-in-the-loop          | Pre-release       | DO-5.4        |
| L5 — Flight review  | Manual checklist              | Pre-flight        | DO-5.5        |

### DO-5.2 — Unit testing

- Each service and driver has tests in `test/`.
- Tests are requirements-based: each test traces to at least one
  `REQ-*` (see DO-2.2).
- Normal-range tests: verify correct behavior with valid inputs.
- Abnormal-range tests: verify graceful handling of out-of-range,
  null, overflow, corrupt inputs.
- Boundary tests: exact limits (e.g., `len == 0`, `len == MAX`).

### DO-5.3 — Integration testing

- Tests verify correct interaction between modules over RTOS
  primitives (queues, semaphores).
- Include timing tests: verify task periods are met under load.
- Include failure injection: simulate sensor failure, radio
  timeout, queue full.

### DO-5.4 — Hardware-in-the-loop (HIL) testing

- Real ESP32-S3 hardware with actual peripherals connected.
- Test pyro continuity circuits, GPS lock, IMU calibration,
  radio link margin.
- Flight-critical sequences: simulate full launch → apogee →
  deployment → landing.

### DO-5.5 — Pre-flight review checklist

Before any flight, the following must be verified:

- [ ] Firmware built from tagged `main` release (DO-4.3).
- [ ] All unit and integration tests pass.
- [ ] No open CRITICAL or HIGH issues.
- [ ] Pyro continuity verified via hardware test.
- [ ] Radio link tested at expected range.
- [ ] Battery voltage within limits.
- [ ] Flight parameters (thresholds, timeouts) reviewed.
- [ ] Traceability matrix up to date (DO-2.3).

### DO-5.6 — Coverage targets

Statement + branch coverage targets by criticality:

| Module Category                    | Statement | Branch |
|------------------------------------|-----------|--------|
| Flight-critical (FSM, deployment)  | ≥ 90%     | ≥ 80%  |
| Drivers (sensors, radio, flash)    | ≥ 80%     | ≥ 70%  |
| Utilities (CRC, encoding, logging) | ≥ 70%     | ≥ 60%  |

---

## DO-6 — Robustness and Abnormal Conditions

**DO-178C §6.3.3 — Robustness Testing**

Every function must handle both normal and abnormal inputs. The
system must remain safe under any failure combination.

### DO-6.1 — Input validation

- All external data validated before use (CERT-1).
- Sensor readings checked against physical limits:

| Sensor       | Parameter     | Valid Range               | Action on Invalid        |
|--------------|---------------|---------------------------|--------------------------|
| IMU          | Acceleration  | [0, 200] m/s²             | Mark stale, use last good|
| Barometer    | Altitude      | [-500, 100 000] m         | Mark stale, use last good|
| GPS          | Latitude      | [-90, +90]°               | Discard sentence         |
| GPS          | Longitude     | [-180, +180]°             | Discard sentence         |
| Thermometer  | Temperature   | [-40, +85] °C             | Mark stale               |
| Battery      | Voltage       | [2.5, 4.3] V              | Warning / abort          |

### DO-6.2 — Timeout discipline

Timeouts on all blocking operations (see CERT-10, RTOS-8):

| Interface     | Typical Timeout | Max Retries |
|---------------|-----------------|-------------|
| I²C read      | 10 ms           | 3           |
| SPI transfer  | 5 ms            | 2           |
| UART sentence | 200 ms          | 1           |
| Radio TX      | 500 ms          | 1           |
| Radio RX      | configurable    | —           |
| Flash write   | 50 ms           | 2           |
| Mutex acquire | 20 ms           | 1           |

### DO-6.3 — Watchdog

- Hardware watchdog timer enabled with a period shorter than the
  longest allowed task latency.
- Critical tasks must periodically feed the watchdog.
- If a task stalls, the system reboots into a known safe state.

### DO-6.4 — Safe state definition

On unexpected state, unrecoverable error, or assertion failure:

| Subsystem      | Safe State                                      |
|----------------|------------------------------------------------|
| Pyro           | All channels DISARMED — never fire on error     |
| FSM            | Transition to `ERROR` state                     |
| Radio          | Transmit last-known telemetry + error flag       |
| Logging        | Flush buffer to flash, stop                      |
| Sensors        | Mark all readings stale                          |
| Watchdog       | If no recovery within timeout → controlled reset |

> **Cardinal rule:** On any unexpected state, do **NOT** fire pyros.

---

## DO-7 — Independence of Safety-Critical Functions

**DO-178C §2.3.2 — Software Partitioning, §6.3.4 — Independence**

Safety-critical functions must be independent of non-critical
functions. A failure in telemetry, logging, or ground-link must
never block, delay, or corrupt deployment logic.

### DO-7.1 — Task isolation

Different RTOS tasks for each subsystem (see RTOS-2):

| Task              | Criticality | Can Affect Deployment? |
|-------------------|-------------|------------------------|
| `control_task`    | CRITICAL    | Yes (it decides)       |
| `deploy_task`     | CRITICAL    | Yes (it actuates)      |
| `sensor_task`     | HIGH        | Indirect (provides data)|
| `radio_tx_task`   | MEDIUM      | No                     |
| `radio_rx_task`   | MEDIUM      | No (commands filtered) |
| `logging_task`    | LOW         | No                     |
| `led_task`        | LOW         | No                     |

### DO-7.2 — Data isolation

- Parachute deployment logic must not share mutable state with
  telemetry or logging.
- Shared data from sensors → FSM flows through RTOS queues, not
  global variables (RTOS-4).
- Queues to non-critical tasks (logging, telemetry) must use
  `xQueueSend` with zero timeout (`pdMS_TO_TICKS(0)`) — drop the
  message rather than block the critical path.

### DO-7.3 — Failure containment

- A failure in comms or logging must never block or delay deployment.
- If the radio task crashes or stalls, the control task continues
  unaffected.
- Watchdog monitors each task independently; a single task reset
  does not cascade to unrelated tasks (RTOS-9).

---

## DO-8 — Problem Reporting

**DO-178C §8 — Software Quality Assurance, §9 — Certification
Liaison (adapted as Problem Reporting)**

All bugs, anomalies, and unexpected behaviors are tracked through
a structured lifecycle.

### DO-8.1 — Issue format

Each issue must document:

| Field              | Content                                         |
|--------------------|-------------------------------------------------|
| **ID**             | `BUG-<NNN>` or git issue number                |
| **Symptom**        | What was observed                               |
| **Expected**       | What should have happened                       |
| **Root cause**     | Why it happened (after investigation)           |
| **Fix**            | What was changed (commit SHA)                   |
| **Affected reqs**  | Which `REQ-*` are impacted                      |
| **Regression test**| Test case added to prevent recurrence           |
| **Severity**       | CRITICAL / HIGH / MEDIUM / LOW                  |

### DO-8.2 — Severity classification

| Severity   | Definition                                       | Response             |
|------------|--------------------------------------------------|----------------------|
| CRITICAL   | Safety hazard: pyro misfire, loss of control     | Immediate fix, re-test all safety reqs |
| HIGH       | Loss of function: sensor failure, radio blackout | Fix before next flight|
| MEDIUM     | Degraded performance: timing jitter, log gaps    | Fix before next release|
| LOW        | Cosmetic or minor: LED color, log format         | Fix when convenient  |

### DO-8.3 — Regression discipline

- Every resolved bug must have a regression test.
- Regression tests run automatically on every commit (DO-5.2).
- A bug without a regression test is **not closed**.

---

## DO-9 — Software Planning

**DO-178C §4 — Software Planning Process**

Before development begins (or when a new subsystem is added),
planning defines what will be built, how it will be verified, and
what standards apply.

### DO-9.1 — Planning artifacts

ARES maintains lightweight equivalents of DO-178C plans:

| DO-178C Plan                           | ARES Equivalent                              |
|----------------------------------------|----------------------------------------------|
| Plan for Software Aspects of Cert.     | `docs/development/` — project overview       |
| Software Development Plan (SDP)        | `docs/architecture.md` + this standard       |
| Software Verification Plan (SVP)       | `docs/development/how_to_test.md` + DO-5     |
| Software Config. Management Plan       | DO-4 (this document) + git workflow          |
| Software Quality Assurance Plan        | DO-13 (this document)                        |
| Software Requirements Standards        | DO-1 (this document)                         |
| Software Design Standards              | DO-3 + PO10 + MISRA                          |
| Software Code Standards                | PO10 + MISRA + CERT + RTOS                   |

### DO-9.2 — Planning updates

Plans are living documents. When a subsystem is added or the
architecture changes significantly, the affected planning artifacts
must be updated **before** coding starts.

---

## DO-10 — Software Life Cycle

**DO-178C §3 — Software Life Cycle**

ARES follows a simplified iterative life cycle with defined phases
and exit criteria.

### DO-10.1 — Life cycle phases

```
Requirements → Design → Implementation → Verification → Release
     ↑                                         |
     └─────────── Feedback / Bug Fixes ────────┘
```

| Phase            | Inputs                          | Outputs                         | Exit Criteria (DO-17) |
|------------------|---------------------------------|---------------------------------|-----------------------|
| Requirements     | Mission objectives, safety goals| `docs/requirements/REQ-*.md`    | DO-17.1               |
| Design           | Requirements                    | Architecture doc, `.h` headers  | DO-17.2               |
| Implementation   | Design                          | `.cpp` source, `config.h`      | DO-17.3               |
| Verification     | Code + requirements             | Test results, coverage reports  | DO-17.4               |
| Release          | Verified code                   | Tagged binary, changelog        | DO-17.5               |

### DO-10.2 — Iteration

Iteration is expected: a verification finding may require a design
change, which flows back to requirements. All changes follow the
same traceability (DO-2) and configuration management (DO-4) rules
regardless of lifecycle phase.

---

## DO-11 — Structural Coverage Analysis

**DO-178C §6.4.4 — Structural Coverage Analysis**

Structural coverage measures which parts of the code have been
exercised by tests. It complements requirements-based testing
(DO-5) by revealing untested code paths.

### DO-11.1 — Coverage levels

| Level                       | Definition                                  | ARES Requirement     |
|-----------------------------|---------------------------------------------|----------------------|
| Statement coverage          | Every statement executed at least once       | Required (DAL-C)     |
| Decision coverage           | Every Boolean decision TRUE and FALSE        | Recommended          |
| MC/DC                       | Each condition independently affects outcome | Not required         |

### DO-11.2 — Coverage targets

See DO-5.6 for per-category targets.

### DO-11.3 — Handling uncovered code

If structural coverage reveals code not exercised by requirements-
based tests:

1. **Missing test**: Write a test for the uncovered path.
2. **Dead code**: The code is unreachable → remove it (DO-15).
3. **Defensive code**: The code handles an abnormal condition that
   is difficult to trigger in tests → document as justified
   gap with a comment `// COVERAGE: defensive, see REQ-SAF-xxx`.

### DO-11.4 — Coverage tooling

- Use PlatformIO native test runner with `gcov` / `lcov` for
  desktop-target builds.
- Generate HTML coverage reports as part of CI/release.
- Coverage reports are attached to the release baseline (DO-4.3).

---

## DO-12 — Tool Qualification

**DO-178C §12.2 — Tool Qualification**

Tools whose output is directly used in the final software (or whose
failure could introduce errors that go undetected) must be qualified
or their use justified.

### DO-12.1 — Tool classification

| Tool                  | Category        | Qualification Approach              |
|-----------------------|-----------------|-------------------------------------|
| GCC / Xtensa toolchain| Compiler        | Verified by test results; compiler bugs tracked as issues |
| PlatformIO            | Build system    | Pinned version; build reproducibility verified (DO-4.4) |
| ESP-IDF / FreeRTOS    | RTOS framework  | Pinned version; known errata tracked |
| `gcov` / `lcov`       | Coverage tool   | Spot-checked against manual analysis |
| Custom scripts        | Test harness    | Reviewed and version-controlled     |

### DO-12.2 — Tool version pinning

- Compiler and framework versions are pinned in `platformio.ini`.
- Tool upgrades are treated as a change: tested, reviewed, and
  documented before adoption.
- Known tool bugs or limitations are documented in
  `docs/development/tool_errata.md`.

### DO-12.3 — Compiler settings

The following compiler flags are mandatory for all builds:

```ini
# platformio.ini
build_flags =
    -Wall
    -Wextra
    -Werror
    -Wconversion
    -Wshadow
    -Wdouble-promotion
    -fno-exceptions
    -fno-rtti
```

See PO10-10 for the zero-warning policy.

---

## DO-13 — Software Quality Assurance

**DO-178C §8 — Software Quality Assurance Process**

Quality assurance ensures that processes are followed and that
outputs conform to standards. In a small team, QA is implemented
through checklists and peer review rather than an independent
department.

### DO-13.1 — Review types

| Review Type       | When                      | What Is Checked                        |
|-------------------|---------------------------|----------------------------------------|
| Self-review       | Before commit             | Code compiles, tests pass, standards followed |
| Peer review       | Before merge to `develop` | Design correctness, requirement coverage, coding standards |
| Safety review     | Before merge to `main`    | Safety requirements, pyro logic, interlock correctness |
| Pre-flight review | Before each flight        | Full checklist (DO-5.5)                |

### DO-13.2 — Review checklist

Every peer review must verify:

- [ ] Requirement IDs referenced in source and tests (DO-2).
- [ ] No compiler warnings (PO10-10).
- [ ] No forbidden constructs (PO10-1, PO10-3, RTOS-12).
- [ ] Timeouts on all blocking calls (RTOS-8, CERT-10).
- [ ] Fixed-width types only (MISRA-1).
- [ ] No magic numbers (MISRA-7, PO10-8).
- [ ] Functions ≤ 60 lines (PO10-4, MISRA-12).
- [ ] External input validated (CERT-1).
- [ ] Traceability updated (DO-2.3).

### DO-13.3 — Conformity records

Review outcomes are recorded as:
- Approved merge (git merge commit with reviewer noted).
- Comments on the merge request with findings.
- Deviations documented with justification and the rule relaxed.

---

## DO-14 — Previously Developed Software

**DO-178C §12.3 — Reuse of Previously Developed Software**

Third-party libraries and previously developed code carry risk:
they were not developed under ARES standards. Their use must be
justified and controlled.

### DO-14.1 — Approved third-party components

| Component          | Source      | Version    | Justification                          |
|--------------------|------------|------------|----------------------------------------|
| ESP-IDF / FreeRTOS | Espressif  | Pinned     | RTOS kernel; extensively validated      |
| SPI / I²C drivers  | ESP-IDF    | Pinned     | Hardware abstraction; no alternative    |
| RadioLib           | jgromes    | Pinned     | Multi-radio abstraction; well-tested    |
| TinyGPS++          | mikalhart  | Pinned     | GPS parsing; widely used, small codebase|

### DO-14.2 — Rules for third-party code

- Pin to an exact version (no floating `^` or `~` ranges).
- Wrap in an ARES adapter layer — never call third-party APIs
  directly from application logic.
- Validate outputs of third-party code as if they were external
  input (CERT-1).
- Track known bugs and CVEs for all third-party dependencies.
- Upgrades follow the same change process (DO-4, DO-5).

### DO-14.3 — Adapter pattern

```
Application logic
       │
  ARES adapter (validates, converts, enforces contracts)
       │
  Third-party library
```

The adapter is the trust boundary. It translates library outputs
into validated, ARES-typed data. If the library changes or is
replaced, only the adapter changes.

---

## DO-15 — Deactivated Code and Dead Code

**DO-178C §6.4.4.3 — Deactivated Code**

Dead code and deactivated code are liabilities: they bloat the
binary, confuse reviewers, and may accidentally execute.

### DO-15.1 — Definitions

| Term              | Definition                                      |
|-------------------|-------------------------------------------------|
| Dead code         | Code that can never execute (unreachable paths)  |
| Deactivated code  | Code disabled by `#if 0`, `#ifdef DEBUG`, etc.  |
| Orphan code       | Functions/variables declared but never referenced|

### DO-15.2 — Rules

- **Dead code** must be removed. If structural coverage reveals
  an unreachable path, delete it or justify it (DO-11.3).
- **Deactivated code** must be:
  - Guarded by a named `#ifdef` (e.g., `ARES_DEBUG_TELEMETRY`),
    not raw `#if 0`.
  - Documented with purpose and conditions for activation.
  - Tested independently when activated.
  - Listed in a deactivated-code registry in `docs/development/`.
- **Orphan code** must be removed. Unused `#include` directives,
  unreferenced functions, and unreferenced variables are not
  allowed.
- **Commented-out code** is forbidden (CERT-20). If code is not
  needed, delete it — git preserves history.

### DO-15.3 — Debug-only code

Debug instrumentation (extra logging, profiling, test hooks) is
acceptable only under a compile-time guard:

```cpp
#ifdef ARES_DEBUG
    Serial.printf("FSM state=%d\n", static_cast<int>(state));
#endif
```

Debug code must never affect timing or behavior of release builds.
The release binary must be built **without** `ARES_DEBUG` defined.

---

## DO-16 — Partitioning (Memory and Time)

**DO-178C §2.3.2 — Software Partitioning**

Partitioning ensures that one function cannot interfere with
another in memory or in time. On the ESP32-S3 (no MMU, no memory
protection unit), partitioning is enforced by design discipline
rather than hardware.

### DO-16.1 — Memory partitioning

- Each module owns its data: no module may write to another
  module's static buffers.
- Inter-module data flows only through RTOS primitives (DO-7.2,
  RTOS-4).
- Stack sizes are explicitly allocated per task (RTOS-7, CERT-14):

| Task              | Stack (bytes) | Justification                  |
|-------------------|---------------|--------------------------------|
| `control_task`    | 4096          | FSM + validation logic         |
| `sensor_task`     | 4096          | I²C/SPI driver buffers         |
| `radio_tx_task`   | 3072          | Frame encoding + SPI           |
| `radio_rx_task`   | 3072          | Frame decoding + validation    |
| `logging_task`    | 2048          | Flash write buffer             |
| `led_task`        | 1024          | Minimal GPIO                   |

- Stack watermarks monitored at runtime (CERT-14). If any task
  uses > 80% of its stack, it is a defect.

### DO-16.2 — Time partitioning

- Each task has a defined period and worst-case execution time
  (WCET).
- The RTOS scheduler enforces preemption; no task may disable
  the scheduler (except brief critical sections ≤ 10 μs).
- Periodic tasks use `vTaskDelayUntil()` for drift-free timing
  (RTOS-6).

| Task              | Period   | WCET Budget  | Deadline   |
|-------------------|----------|--------------|------------|
| `sensor_task`     | 10 ms    | 3 ms         | 10 ms      |
| `control_task`    | 20 ms    | 5 ms         | 20 ms      |
| `radio_tx_task`   | 100 ms   | 15 ms        | 100 ms     |
| `logging_task`    | 50 ms    | 10 ms        | soft       |

- Deadline misses must be detected and logged (RTOS-11). Three
  consecutive misses on a critical task trigger watchdog
  intervention (RTOS-9).

### DO-16.3 — Dual-core discipline

The ESP32-S3 has two cores. Core affinity rules:

| Core   | Pinned Tasks                        | Rationale                     |
|--------|-------------------------------------|-------------------------------|
| Core 0 | WiFi/BT stack (ESP-IDF), `led_task` | ESP-IDF requires core 0       |
| Core 1 | `sensor_task`, `control_task`, `deploy_task` | Flight-critical, deterministic |
| Any    | `radio_tx_task`, `logging_task`     | Non-critical, flexible        |

Flight-critical tasks pinned to Core 1 are shielded from WiFi/BT
interrupt jitter on Core 0.

---

## DO-17 — Transition Criteria

**DO-178C §4.6 — Transition Criteria**

Each lifecycle phase (DO-10) has entry and exit criteria. Moving
to the next phase without meeting exit criteria is a process
violation.

### DO-17.1 — Requirements → Design

- [ ] All mission-level requirements captured in `docs/requirements/`.
- [ ] Each requirement has an acceptance criterion.
- [ ] Safety requirements identified and flagged `CRITICAL`.
- [ ] Requirements reviewed (DO-13.1).

### DO-17.2 — Design → Implementation

- [ ] Architecture document updated (DO-3.1).
- [ ] Module headers (`.h`) define the public API (DO-3.2).
- [ ] Task table, priority assignments, and IPC map defined.
- [ ] Design reviewed (DO-13.1).

### DO-17.3 — Implementation → Verification

- [ ] Code compiles with zero warnings (PO10-10, DO-12.3).
- [ ] All requirement IDs tagged in source and test files (DO-2).
- [ ] Self-review completed (DO-13.1).

### DO-17.4 — Verification → Release

- [ ] All unit and integration tests pass (DO-5.2, DO-5.3).
- [ ] Structural coverage meets targets (DO-5.6, DO-11).
- [ ] No open CRITICAL or HIGH issues (DO-8.2).
- [ ] Traceability matrix complete — no gaps (DO-2.4).
- [ ] Peer review + safety review completed (DO-13.1).

### DO-17.5 — Release → Flight

- [ ] Firmware built from tagged `main` commit (DO-4.3).
- [ ] Pre-flight checklist passed (DO-5.5).
- [ ] All deactivated code documented (DO-15.2).
- [ ] Release notes and changelog published.

---

## DO-18 — Compliance Evidence

**DO-178C §11 — Software Life Cycle Data**

ARES maintains the following artifacts as compliance evidence.
These are not for formal certification but provide auditability
and learning value.

### DO-18.1 — Required artifacts

| Artifact                    | Location                          | Updated When           |
|-----------------------------|-----------------------------------|------------------------|
| Requirements                | `docs/requirements/`              | Requirements phase     |
| Architecture document       | `docs/architecture.md`            | Design phase           |
| Module headers (`.h`)       | `src/`, `lib/`                    | Design / implementation|
| Source code                  | `src/`, `lib/`                    | Implementation         |
| Test cases                  | `test/`                           | Verification           |
| Test results                | CI output / release notes         | Every test run         |
| Coverage reports            | Generated by `gcov`/`lcov`        | Verification           |
| Traceability matrix         | `docs/traceability.md`            | All phases             |
| Changelog                   | `CHANGELOG.md`                    | Each release           |
| Issue tracker               | Git issues                        | Continuous             |
| Review records              | Merge request comments            | Each merge             |
| Deactivated code registry   | `docs/development/`               | When code is disabled  |
| Tool errata                 | `docs/development/tool_errata.md` | When tool bugs found   |

### DO-18.2 — Retention

All artifacts are version-controlled in git. Tagged release
baselines (DO-4.3) serve as immutable snapshots of the full
project state at each release point.

---

## Anti-Patterns (Forbidden)

| Anti-Pattern                                      | Rule      |
|---------------------------------------------------|-----------|
| Code without a requirement                        | DO-1.4    |
| Requirement without a test                        | DO-1.4    |
| Source file without `@reqs` header                | DO-2.1    |
| Test without `@verifies` tag                      | DO-2.2    |
| Undocumented public API in header                 | DO-3.2    |
| Uncommitted code in a flight binary               | DO-4.1    |
| Flight firmware from non-`main` branch            | DO-4.3    |
| Floating library version in `platformio.ini`      | DO-4.4    |
| Module without unit tests                         | DO-5.2    |
| Untested abnormal / boundary inputs               | DO-5.2    |
| Sensor value used without range check             | DO-6.1    |
| Blocking call without timeout                     | DO-6.2    |
| Pyro fired on unexpected state                    | DO-6.4    |
| Logging task blocking deployment task             | DO-7.3    |
| Bug closed without regression test                | DO-8.3    |
| Third-party API called without adapter            | DO-14.2   |
| `#if 0` without named guard                       | DO-15.2   |
| Commented-out code                                | DO-15.2   |
| Module writing to another module's static buffer  | DO-16.1   |
| Task stack > 80% usage without investigation      | DO-16.1   |
| Phase transition without meeting exit criteria     | DO-17     |
| Missing artifact in release baseline               | DO-18.1   |

---

## Cross-References

| DO Rule | DO-178C Section | Related ARES Rules                                  |
|---------|-----------------|-----------------------------------------------------|
| DO-1    | §5.1            | —                                                   |
| DO-2    | §5.5            | —                                                   |
| DO-3    | §5.2            | RTOS-2, RTOS-5 (task design)                        |
| DO-4    | §7              | —                                                   |
| DO-5    | §6              | PO10-10 (zero warnings), CERT-21 (assertions)       |
| DO-6    | §6.3.3          | RTOS-1, RTOS-8 (timeouts), CERT-1, CERT-10, CERT-16|
| DO-7    | §2.3.2, §6.3.4  | RTOS-2, RTOS-4 (task isolation, IPC)                |
| DO-8    | §8, §9          | —                                                   |
| DO-9    | §4              | All standards (planning references)                 |
| DO-10   | §3              | DO-17 (transition criteria)                         |
| DO-11   | §6.4.4          | DO-5 (verification), CERT-20 (dead code)            |
| DO-12   | §12.2           | PO10-10 (compiler flags), DO-4 (version pinning)   |
| DO-13   | §8              | PO10, MISRA, CERT, RTOS (standards enforced)        |
| DO-14   | §12.3           | CERT-1 (validate third-party outputs)               |
| DO-15   | §6.4.4.3        | CERT-20 (dead code), MISRA-11 (switch completeness) |
| DO-16   | §2.3.2          | RTOS-2, RTOS-5, RTOS-6 (task scheduling), CERT-14  |
| DO-17   | §4.6            | DO-1 … DO-16 (criteria reference all rules)         |
| DO-18   | §11             | DO-4 (config management)                            |

---

## References

- RTCA DO-178C, *Software Considerations in Airborne Systems and
  Equipment Certification*, 2011.
- RTCA DO-331, *Model-Based Development and Verification Supplement
  to DO-178C*, 2011.
- ECSS-Q-ST-80C, *Software Product Assurance*, ESA, 2017.
- ESP-IDF FreeRTOS documentation:
  https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-reference/system/freertos.html
- NASA JPL Institutional Coding Standard for the C Programming
  Language, JPL D-60411, 2009.

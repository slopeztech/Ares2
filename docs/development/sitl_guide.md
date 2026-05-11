# ARES SITL — Software-In-The-Loop Simulator

**Location:** `test/test_ams_integration/`  
**PlatformIO env:** `[env:sim]`  
**Run command:** `pio test -e sim`

---

## 1. Overview

The SITL framework replaces all hardware HAL drivers with deterministic,
in-memory substitutes so that the full AMS (`MissionScriptEngine`) stack can
be compiled and tested on a host PC without any embedded hardware.

The architecture follows the **Opción A — Deterministic Replay Drivers**
strategy: each simulated sensor reads its values by interpolating linearly
between entries in a static `FlightProfile` table.  The sim clock advances
at test-controlled steps, giving fully reproducible results regardless of
host CPU speed.

```
┌─────────────────────────────────────────────────────────────┐
│  test/test_ams_integration/                                 │
│  ┌──────────────────────────────────────────────────────┐   │
│  │  Unity test runner (main.cpp)                        │   │
│  │  test_ams_engine_lifecycle.cpp                       │   │
│  │  test_ams_flight_scenario.cpp                        │   │
│  └──────────────────────────────────────────────────────┘   │
│                        │                                    │
│                        ▼                                    │
│  ┌──────────────────────────────────────────────────────┐   │
│  │  MissionScriptEngine  (src/ams/)                     │   │
│  └──────────────────────────────────────────────────────┘   │
│          │          │          │         │          │        │
│          ▼          ▼          ▼         ▼          ▼        │
│  SimGps  SimBaro  SimImu  SimRadio  SimStorage      │        │
│       (test/stubs/hal_sim/)                         │        │
│                                                     │        │
│  FreeRTOS stub (test/stubs/freertos/)  ◄────────────┘        │
└─────────────────────────────────────────────────────────────┘
```

---

## 2. Directory Structure

```
test/
└── stubs/
│   ├── Arduino.h                    # millis() + Serial stub (pre-existing)
│   ├── freertos/
│   │   ├── FreeRTOS.h               # BaseType_t, TickType_t, pdTRUE, …
│   │   └── semphr.h                 # xSemaphoreCreateMutexStatic, Take, Give
│   └── hal_sim/
│       ├── sim_clock.h              # Shared sim time (inline C++17 variable)
│       ├── sim_clock.cpp            # Intentionally empty (inline in header)
│       ├── flight_profile.h         # FlightSample, FlightProfile, sampleProfile()
│       ├── sim_gps_driver.h         # SimGpsDriver   : GpsInterface
│       ├── sim_baro_driver.h        # SimBaroDriver  : BarometerInterface
│       ├── sim_imu_driver.h         # SimImuDriver   : ImuInterface
│       ├── sim_radio_driver.h       # SimRadioDriver : RadioInterface
│       └── sim_storage_driver.h     # SimStorageDriver : StorageInterface
└── test_ams_integration/
    ├── main.cpp                     # Unity runner
    ├── sim_ams_scripts.h            # Inline AMS scripts for tests
    ├── test_ams_engine_lifecycle.cpp
    └── test_ams_flight_scenario.cpp
```

---

## 3. Building and Running

### Prerequisites

- PlatformIO installed (`pio` command available).
- GCC on native platform (Linux/macOS: system GCC; Windows: w64devkit in PATH).

### Run All SITL Tests

```bash
pio test -e sim
```

### Run Only a Specific Test File

```bash
pio test -e sim -f test_ams_integration
```

### Run SITL and Unit Tests Together

```bash
pio test -e native -e sim
```

### Expected Output (all passing)

```
test/test_ams_integration/test_ams_engine_lifecycle.cpp:86:test_begin_returns_true         PASS
test/test_ams_integration/test_ams_engine_lifecycle.cpp:95:test_initial_status_is_idle    PASS
...
19 Tests 0 Failures 0 Ignored
```

---

## 4. Core Components

### 4.1 Simulation Clock (`sim_clock.h`)

All sim drivers derive their timestamps from a single `uint32_t g_simTimeMs`
variable defined as a C++17 `inline` variable.  Test code controls time by
calling:

| Function | Effect |
|---|---|
| `ares::sim::clock::reset()` | Reset clock to 0 ms |
| `ares::sim::clock::advanceMs(n)` | Add `n` ms to the clock |
| `ares::sim::clock::nowMs()` | Read the current sim time |

**Critical:** always call `clock::reset()` at the start of each test to
prevent timing state leakage between tests.

```cpp
ares::sim::clock::reset();
engine.arm();

for (uint32_t t = 0; t <= 6000U; t += 100U)
{
    ares::sim::clock::reset();
    ares::sim::clock::advanceMs(t);
    engine.tick(ares::sim::clock::nowMs());
}
```

### 4.2 Flight Profile (`flight_profile.h`)

A `FlightProfile` is a `FlightSample[SIM_MAX_PROFILE_SAMPLES]` array with
a populated `count`.  Samples must be stored in strictly ascending `timeMs`
order.  `sampleProfile()` performs a binary search and returns linearly
interpolated values.

```cpp
static const ares::sim::FlightProfile kMyProfile = {
    .count   = 3U,
    .samples = {
        { .timeMs = 0U,    .baroAltM = 0.0f,   .accelZ = 9.81f },
        { .timeMs = 3000U, .baroAltM = 150.0f, .accelZ = 15.0f },
        { .timeMs = 6000U, .baroAltM = 300.0f, .accelZ = 9.81f },
    }
};
```

All sensors (GPS, baro, IMU) read from the same `FlightProfile` instance,
keeping them physically consistent.

### 4.3 Sim Drivers

| Driver | Interface | Model Name | Notes |
|---|---|---|---|
| `SimGpsDriver` | `GpsInterface` | `SIM_GPS` | Returns fix when `FlightSample::gpsFix == true` |
| `SimBaroDriver` | `BarometerInterface` | `SIM_BARO` | `setSeaLevelPressure()` is a no-op |
| `SimImuDriver` | `ImuInterface` | `SIM_IMU` | |
| `SimRadioDriver` | `RadioInterface` | `SIM_COM` | Captures last transmitted frame |
| `SimStorageDriver` | `StorageInterface` | — | Serves registered in-memory files |

The **model name** must match the `include <MODEL> as <ALIAS>` directive
in the AMS script exactly.

### 4.4 Sim Storage Driver (`sim_storage_driver.h`)

The AMS engine reads `.ams` files from the storage layer.  In the sim,
call `registerFile()` before `engine.begin()`:

```cpp
ares::sim::SimStorageDriver storage;
storage.begin();
storage.registerFile("/missions/my_script.ams", kMyAmsContent);
```

Write operations (`writeFile`, `appendFile`, `removeFile`, `renameFile`)
silently succeed — they are used internally by the engine for the resume
checkpoint and log file management.  No real I/O occurs.

### 4.5 FreeRTOS Stub (`test/stubs/freertos/`)

The engine uses FreeRTOS mutexes via `ScopedLock`.  In the single-threaded
sim:

- `xSemaphoreCreateMutexStatic()` initialises the `StaticSemaphore_t` in place.
- `xSemaphoreTake()` toggles `locked = true` and returns `pdTRUE`.
- `xSemaphoreGive()` toggles `locked = false` and returns `pdTRUE`.
- `vTaskDelay()` is a no-op.

Double-take bugs (acquiring an already-held mutex) return `pdFALSE`, which
causes `ScopedLock::acquired()` to return `false` — the same behaviour as a
real FreeRTOS timeout.

---

## 5. Writing New Tests

### 5.1 Add a New AMS Script

Add a new `static const char kScriptXxx[]` to
`test/test_ams_integration/sim_ams_scripts.h`.  Use the sim driver model
names:

```
include SIM_GPS  as GPS
include SIM_BARO as BARO
include SIM_COM  as COM
include SIM_IMU  as IMU
```

### 5.2 Create a Flight Profile

Define a `static const ares::sim::FlightProfile` with the required sensor
trace.  A minimum of two samples is required (start and end of the profile).
Ensure samples are in ascending `timeMs` order.

```cpp
static const ares::sim::FlightProfile kMyProfile = {
    .count   = 2U,
    .samples = {
        { .timeMs = 0U,     .baroAltM = 0.0f   },
        { .timeMs = 10000U, .baroAltM = 500.0f },
    }
};
```

### 5.3 Assemble the Engine Fixture

```cpp
ares::sim::SimStorageDriver storage;
ares::sim::SimGpsDriver     gps{kMyProfile};
ares::sim::SimBaroDriver    baro{kMyProfile};
ares::sim::SimImuDriver     imu{kMyProfile};
ares::sim::SimRadioDriver   radio;

GpsEntry  gpsEntry  = { "SIM_GPS",  &gps  };
BaroEntry baroEntry = { "SIM_BARO", &baro };
ComEntry  comEntry  = { "SIM_COM",  &radio};
ImuEntry  imuEntry  = { "SIM_IMU",  &imu  };

MissionScriptEngine engine{
    storage,
    &gpsEntry, 1U, &baroEntry, 1U, &comEntry, 1U, &imuEntry, 1U
};

storage.begin();
storage.registerFile("/missions/my_script.ams", kMyScriptContent);
engine.begin();
engine.activate("my_script.ams");
engine.arm();
```

### 5.4 Advance the Clock and Tick

```cpp
ares::sim::clock::reset();

for (uint32_t t = 0U; t <= 10000U; t += 100U)
{
    ares::sim::clock::reset();
    ares::sim::clock::advanceMs(t);
    engine.tick(ares::sim::clock::nowMs());
}
```

### 5.5 Assert State and Telemetry

```cpp
ares::ams::EngineSnapshot snap{};
engine.getSnapshot(snap);
TEST_ASSERT_EQUAL_STRING("RECOVERY", snap.stateName);

// Verify radio frames were emitted
TEST_ASSERT_GREATER_THAN(0U, radio.sendCount());
```

### 5.6 Register the Test in `main.cpp`

Declare the function as `extern void test_xxx();` and add
`RUN_TEST(test_xxx);` to `main()`.

---

## 6. Limitations and Known Constraints

| Constraint | Reason |
|---|---|
| Sensor values are frozen at interpolated profile values | No noise model — add profile samples for higher-fidelity testing |
| Radio `receive()` always returns 0 bytes | Inbound TC injection must be done via `engine.injectTcCommand()` |
| Storage writes are silently discarded | Resume-point and log files are not persisted between tests |
| `millis()` always returns 0 in the Arduino stub | Time is driven exclusively by `engine.tick(nowMs)` parameter |
| Max 64 profile samples per profile (`SIM_MAX_PROFILE_SAMPLES`) | Increase the constant for very long mission profiles |
| Max 16 registered files per storage instance (`SIM_STORAGE_MAX_FILES`) | Increase the constant if more files are needed |

---

## 7. Adding New Sim Drivers

To add a new sensor type (e.g. a magnetometer):

1. Define the HAL interface in `src/hal/mag/magnetometer_interface.h`.
2. Add the registry entry in `src/ams/ams_driver_registry.h`.
3. Add sensor fields to `FlightSample` in `flight_profile.h`.
4. Update `sampleProfile()` to populate the new fields.
5. Create `test/stubs/hal_sim/sim_mag_driver.h` implementing the new interface.
6. Add the corresponding `MagEntry` to the engine constructor in test fixtures.

---

## 8. Standards Compliance

| Standard | Requirement | Implementation |
|---|---|---|
| PO10-3 | No dynamic allocation | All sim drivers use stack/static storage only |
| PO10-2 | Bounded loops | `sampleProfile()` binary search, bounded by `count` |
| CERT-13 | External synchronisation note | Documented in each driver header |
| MISRA-4 | All fields default-initialised | `FlightSample` and `FlightProfile` use C++ member init |
| CERT-6.1 | Range validation sentinels | Not applicable to sim (host-only, no untrusted input) |

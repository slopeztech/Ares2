# ARES — Amateur Rocket Embedded System

WARNING: for now this is a beta version — use it at your own risk until a final and LTS version is released!

**Avionics firmware for amateur rockets on ESP32-S3**

[![Version](https://img.shields.io/badge/version-2.0.0-blue)](#)
[![Build](https://img.shields.io/badge/build-passing-brightgreen)](#build-and-flash)
[![Tests](https://img.shields.io/badge/tests-7%20suites-brightgreen)](#testing)
[![Platform](https://img.shields.io/badge/platform-ESP32--S3-orange)](#hardware)
[![Language](https://img.shields.io/badge/C%2B%2B-17-informational)](#)
[![Radio](https://img.shields.io/badge/radio-LoRa%20868%20MHz-blueviolet)](#radio-telemetry)
[![Standards](https://img.shields.io/badge/standards-11%20enforced-red)](#coding-standards)
[![License](https://img.shields.io/badge/license-GPL--v3-green)](LICENSE)

---

## Table of Contents

1. [What is ARES?](#what-is-ares)
2. [Hardware](#hardware)
3. [Firmware Architecture](#firmware-architecture)
4. [AMS — Ares Mission Script](#ams--ares-mission-script)
5. [Radio Telemetry](#radio-telemetry)
6. [WiFi REST API](#wifi-rest-api)
7. [Coding Standards](#coding-standards)
8. [Requirements Management](#requirements-management)
9. [Build and Flash](#build-and-flash)
10. [Testing](#testing)
11. [Repository Structure](#repository-structure)
12. [Future Work](#future-work)

---

## What is ARES?

ARES is an embedded flight computer for amateur rockets, built around a single compact ESP32-S3 module. The whole system is designed around one core idea: **the flight logic does not live in the firmware, it lives in a mission script** that you write, validate on the ground, upload over WiFi, and execute on the vehicle without recompiling.

That script is **AMS — Ares Mission Script** — a small, declarative, state-machine language tailored to rocketry. With AMS you describe:

- the states of the flight (`WAIT`, `FLIGHT`, `DESCENT`, `LANDED`, …),
- the sensor- and command-driven transitions between them,
- the housekeeping telemetry that goes over the radio,
- the high-rate data that is logged locally,
- the safety guards that abort the mission if a measured value falls outside its allowed range,
- the pulse outputs that fire parachutes or other actuators.

The firmware around AMS is intentionally thin. It provides:

- **Sensor reading** through a hardware abstraction layer (barometer, GPS, IMU).
- **Flight logging** to an internal LittleFS partition on the ESP32-S3 flash.
- **Wireless telemetry** with a custom binary protocol (ARES Radio Protocol v2) over LoRa.
- **A programmable mission engine** that interprets AMS scripts on every main loop tick.
- **Ground configuration** over an integrated WiFi Access Point and a REST API.

The design applies safety-critical software disciplines (NASA JPL Power of 10, DO-178C, MISRA C, CERT C, ESA PUS) adapted to the amateur scale, because **a software failure can destroy the rocket or injure people**. Formal certification is not the goal — the discipline it enforces is.

In practical terms the application runs as a small flight stack with a very explicit lifecycle:

1. Boot the board and bring up sensors, storage, radio, WiFi AP, and status LED.
2. Connect from the ground station over WiFi to inspect status and upload an AMS mission.
3. Activate the mission, switch to flight mode, and arm the system.
4. Let the AMS engine evaluate transitions and guards on every main loop tick while RTOS tasks handle the REST API, the LED, and the radio.
5. Downlink housekeeping and events over LoRa while keeping higher-rate logs in LittleFS for post-flight analysis.

Related documents:
- [Hardware registry](docs/hardware/hardware_registry.md)
- [Pin map](docs/hardware/pin_map.md)
- [AMS programming guide](docs/development/ams_programming_guide.md)
- [ARES radio protocol](docs/architecture/ares_radio_protocol.md)

---

## Hardware

### Flight Computer

| Property  | Value                                         |
|-----------|-----------------------------------------------|
| MCU       | ESP32-S3 (dual-core Xtensa LX7, 240 MHz)     |
| Module    | Waveshare ESP32-S3 Zero Mini (~21×52 mm)      |
| Flash     | 4 MB QSPI                                     |
| SRAM      | 320 KB (+ optional PSRAM)                     |
| RTOS      | FreeRTOS IDF SMP (dual-core)                  |
| WiFi      | 2.4 GHz 802.11 b/g/n onboard                  |
| USB       | Native CDC + CH340 UART                       |
| Firmware  | v2.0.0                                         |

### Current Drivers

Each driver implements a HAL interface, so the application layer never depends on a specific chip. Swapping a driver does not require recompiling AMS or the engine.

| Driver              | Interface | HAL contract           | Function                                            |
|---------------------|-----------|------------------------|-----------------------------------------------------|
| **DxLr03Driver**    | UART2     | `RadioInterface`       | LoRa radio (868 MHz current profile), telemetry TX  |
| **Bn220Driver**     | UART1     | `GpsInterface`         | u-blox GNSS receiver, NMEA 0183 (GGA + RMC)         |
| **Bmp280Driver**    | I2C0      | `BarometerInterface`   | Barometric pressure + temperature → altitude        |
| **Adxl375Driver**   | I2C1      | `ImuInterface`         | 3-axis high-g accelerometer (±200 g, shock-rated)   |
| **Mpu6050Driver**   | I2C1      | `ImuInterface`         | 6-axis IMU (accelerometer + gyroscope)              |
| **NeopixelDriver**  | GPIO 21   | `LedInterface`         | WS2812B RGB status LED                              |
| **LittleFsStorage** | Flash     | `StorageInterface`     | Mission scripts and flight logs on LittleFS         |

### Pin Map

| GPIO | Function       | Notes                              |
|------|----------------|------------------------------------|
| 1    | I2C0 SDA       | BMP280 (barometer bus)             |
| 2    | I2C0 SCL       | 400 kHz fast mode                  |
| 12   | I2C1 SDA       | ADXL375 + MPU-6050 (IMU bus)       |
| 13   | I2C1 SCL       | 50 kHz                             |
| 5    | GPS UART1 RX   | ← BN-220 TX                        |
| 6    | GPS UART1 TX   | → BN-220 RX                        |
| 7    | LoRa UART2 TX  | → DX-LR03 RX                       |
| 8    | LoRa UART2 RX  | ← DX-LR03 TX                       |
| 9    | LoRa AUX       | HIGH = module idle                 |
| 4    | Pulse DROGUE   | Drogue parachute channel           |
| 15   | Pulse MAIN     | Main parachute channel             |
| 21   | WS2812B LED    | RGB status                         |

Detailed hardware references:
- [Hardware registry](docs/hardware/hardware_registry.md)
- [Pin map](docs/hardware/pin_map.md)

---

## Firmware Architecture

The firmware follows a strict **HAL → Driver → Engine** layering. Application code never depends on a concrete chip: it talks only to pure virtual interfaces, so a sensor or radio can be replaced without recompiling the engine, the API, or the tests.

The code base is intentionally split by responsibility rather than by hardware:

- **`src/main.cpp`** — platform bootstrap and the only place where concrete drivers are statically allocated and wired to interfaces. Runs the main `loop()` (GPS update + AMS tick).
- **`src/config.h`** — single source of truth for compile-time limits (AMS slots, telemetry cadence floors, pulse durations, RTOS stacks, GPIO mapping). Centralising this keeps the rest of the code free of magic numbers.
- **`src/hal/`** — pure virtual contracts: `BarometerInterface`, `GpsInterface`, `ImuInterface`, `RadioInterface`, `StorageInterface`, `LedInterface`. These are the only types the engine and tests see.
- **`src/drivers/`** — concrete implementations of HAL interfaces (`Bmp280Driver`, `Bn220Driver`, `Adxl375Driver`, `Mpu6050Driver`, `DxLr03Driver`). One file per chip, no cross-talk.
- **`src/sys/`** — platform-side adapters that are not chip-specific: WiFi AP (`WifiAp`), LittleFS storage (`LittleFsStorage`), status LED (`NeopixelDriver`/`StatusLed`).
- **`src/ams/`** — the AMS mission engine: parser, AST, runtime state machine, telemetry scheduler, log scheduler, guard evaluator. This is the brain of the flight.
- **`src/comms/`** — ARES Radio Protocol v2 framing, CRC-32, fragmentation, deduplication, all transport-agnostic.
- **`src/api/`** — REST API split per domain (`flight/`, `mission/`, `config/`, `status/`, `storage/`), with a tiny dispatcher that maps HTTP routes to handlers.
- **`src/debug/`** — structured logger used everywhere (`ARES_LOGI/W/E/D`).
- **`src/ares_assert.h`** + **`src/rtos_guard.h`** — the only place where flight-time invariants and lock/timeout patterns are defined.

```
┌─────────────────────────────────────────────────────────────┐
│                         main.cpp                            │
│  Static allocation · bootstrap · main loop                  │
└──────────────┬──────────────────────────────────────────────┘
               │
   ┌───────────▼────────────┐   ┌──────────────────────────┐
   │    AMS Engine           │   │      API Server           │
   │  MissionScriptEngine    │   │  REST HTTP (WiFi AP)      │
   │  states, transitions    │   │  /api/status, /config,    │
   │  PUS HK + EVENT via LoRa│   │  /mission, /arm, /storage │
   └───────────┬────────────┘   └──────────────────────────┘
               │
   ┌───────────▼────────────────────────────────────────────┐
   │                   HAL Interfaces                        │
   │  BarometerInterface · GpsInterface · ImuInterface       │
   │  RadioInterface · StorageInterface · LedInterface       │
   └───────────┬────────────────────────────────────────────┘
               │
   ┌───────────▼────────────────────────────────────────────┐
   │                  Concrete Drivers                       │
   │  Bmp280Driver · Bn220Driver                            │
   │  Adxl375Driver · Mpu6050Driver                          │
   │  DxLr03Driver · LittleFsStorage · NeopixelDriver        │
   └────────────────────────────────────────────────────────┘
```

**Concurrency model.** Concurrency is intentionally minimal and deterministic: the main loop owns the AMS tick and the sensor reads, while a small number of fixed-priority FreeRTOS tasks handle work that must not block the tick — REST + storage I/O, telemetry TX, and the status LED. All shared state is guarded by mutexes with timeouts (no infinite waits), in line with the RTOS standard.

**Why HAL:** Any sensor or radio can be swapped without touching mission logic. The radio protocol unit tests compile on native desktop (no Arduino dependency) thanks to this separation, and the AMS engine can be exercised against fake HAL implementations in the `sim` PlatformIO environment.

### Runtime Flow

`main.cpp` performs static allocation and initialises the platform. Once boot is complete, the system runs as a small cooperative pipeline whose only "hot" path is the main loop:

1. **Boot.** The bootstrap brings up serial logging, configures GPIO and I²C/UART buses, instantiates each concrete driver into its HAL interface, mounts LittleFS, opens the WiFi AP, and starts the RTOS tasks (API, LED, comms). A startup banner reports per-subsystem status.
2. **Idle / ground operations.** With no mission active the engine sits in `IDLE`. The REST API is fully available: the ground station inspects status, edits configuration, uploads or lists AMS scripts, downloads logs, and runs pre-flight checks.
3. **Mission load.** `POST /api/mission/activate` parses the selected AMS script into the engine's static AST. Any parse or validation error keeps the engine in `IDLE` and surfaces a structured error to the API caller.
4. **Arming.** `POST /api/arm` validates the engine state, injects the `TC.command == LAUNCH` telecommand, and re-checks that the engine has transitioned out of `WAIT`. If the engine does not respond `RUNNING`, the system goes to `ERROR` and refuses to fire any pulse output.
5. **Flight tick.** On every loop iteration the engine pulls fresh samples from the barometer driver, the GNSS driver, and the IMU drivers through their HAL interfaces; evaluates the active state's transitions and guard conditions; runs `on_enter` actions on state changes (events, set, pulse fire); and schedules `every` telemetry and `log_every` logging windows.
6. **Downlink and logging.** The comms task drains telemetry/event frames into the radio driver as compact APUS-shaped binary frames. Higher-rate `LOG.report` data is written to LittleFS at an independent cadence so it survives even if the radio link drops.
7. **Recovery and post-flight.** Once the mission reaches a terminal state (e.g. `LANDED`), WiFi is re-enabled if it was disabled for flight, and the ground station downloads logs and event records over the REST API for offline analysis.

### Memory Management

- **Zero heap in flight.** All objects are `static`; no `new` or `malloc` is called after initialisation (rule PO10-3). Free heap at boot is ~245 KB and remains constant.
- **Custom flash partition** (`partitions.csv`): dedicated LittleFS filesystem for flight logs and mission scripts.

### RTOS Tasks

| Task         | Stack  | Priority  | Function                          |
|--------------|--------|-----------|-----------------------------------|
| API task     | 12 KB  | 1 (low)   | REST HTTP polling + LittleFS I/O  |
| LED task     | 2 KB   | 1 (low)   | NeoPixel status blink             |
| Comms task   | 4 KB   | 2 (medium)| LoRa telemetry TX                 |
| `loop()`     | —      | —         | GPS update + AMS tick             |

Watchdog enabled (5 s timeout) with `CORE_PANIC` on stack overflow.

### Operating Modes and LED

| Mode      | LED pattern                              |
|-----------|------------------------------------------|
| IDLE      | Solid green                              |
| TEST      | Slow cyan blink (500 ms on/off)          |
| FLIGHT    | Solid blue                               |
| RECOVERY  | Slow green blink (300 ms on / 700 ms off)|
| ERROR     | Fast red triple-blink (3×100 ms)         |
| BOOT      | Fast green blink (300 ms on / 300 ms off)|

---

## AMS — Ares Mission Script

AMS is the core differentiating feature of ARES. Instead of hard-coding the flight algorithm in C++ and re-flashing the rocket every time a threshold changes, the entire mission is described in a small declarative `.ams` script that lives on the on-board LittleFS partition and is interpreted on every loop tick.

This matters because:

- Tuning a flight does **not** require recompiling firmware. You edit the script on the laptop, upload it over WiFi, and re-activate.
- The same firmware build can fly very different missions: a sensor characterisation flight, a parachute deployment flight, a high-altitude profile, a ground test.
- The script is **static, bounded, and statically analysable**. The parser enforces hard limits (max states, max transitions, max guards, max background tasks, max set/pulse actions per `on_enter`, max include aliases, etc.) defined in `src/config.h`. No dynamic allocation, no unbounded loops, no recursion.
- Safety guards (`conditions:` + `on_error:`) live in the same place as the flight logic, so a contributor cannot add a new state and forget to add its safety envelope.

A script describes:

- a **state machine** (`state WAIT`, `state FLIGHT`, …) with explicit `on_enter` actions,
- **transitions** driven by sensor expressions or by telecommands (`TC.command == LAUNCH`),
- **housekeeping telemetry** scheduled with `every <ms>: HK.report { … }` and downlinked as APUS ST[3] frames,
- **events** raised with `EVENT.info/warn/error`, downlinked as APUS ST[5] frames,
- **local logging** scheduled with `log_every <ms>: LOG.report { … }`, written to LittleFS at an independent cadence,
- **guard conditions** (`conditions:` + `on_error:`) that continuously validate measured values against allowed ranges and escalate to `ERROR` when violated,
- **pulse actions** (`PULSE.fire`) that drive parachute / actuator channels under strict per-state limits.

### Compile-time limits (from `src/config.h`)

These are the hard ceilings the parser enforces. They are visible to the contributor so the script remains predictable in size and timing.

| Constant | Value | Meaning |
|----------|-------|---------|
| `AMS_MAX_SCRIPT_BYTES` | 4096 | Max bytes read from a script file |
| `AMS_MAX_STATES` | 10 | States per mission |
| `AMS_MAX_STATE_NAME` | 16 | State name length |
| `AMS_MAX_EVENT_TEXT` | 64 | `EVENT.*` message length |
| `AMS_MAX_HK_FIELDS` | 16 | Fields per `HK.report` / `LOG.report` block |
| `AMS_MAX_HK_SLOTS` | 4 | `every` / `log_every` blocks per state |
| `AMS_MAX_CONDITIONS` | 4 | Guard conditions per state |
| `AMS_MAX_TRANSITION_CONDS` | 4 | Sub-conditions in a single `transition to` |
| `AMS_MAX_TRANSITIONS` | 4 | `transition to` directives per state |
| `AMS_MAX_VARS` | 8 | Global script variables |
| `AMS_MAX_CONSTS` | 8 | Named constants |
| `AMS_MAX_TASKS` | 4 | Background `task` blocks |
| `AMS_MAX_TASK_RULES` | 4 | `if` rules per background task |
| `AMS_MAX_TASK_ACTIVE_STATES` | 6 | States in a task `when in` filter |
| `AMS_MAX_ASSERTS` | 8 | `assert` directives per script |
| `AMS_MAX_SET_ACTIONS` | 4 | `set` actions per `on_enter` block |
| `AMS_MAX_PULSE_ACTIONS` | 2 | `PULSE.fire` actions per `on_enter` block |
| `AMS_MAX_SENSOR_RETRY` | 5 | Sensor read retries declared in `include` |
| `AMS_MAX_INCLUDES` | 8 | Hardware aliases per script |
| `AMS_MAX_LINE_LEN` | 128 | Parser line buffer |

### Features

- **Declarative state machine** with sensor-driven or TC-command transitions
- **Automatic telemetry** — `HK.report` emits PUS ST[3] frames over LoRa
- **Events** — `EVENT.info/warn/error` emits PUS ST[5] frames
- **Local logging** — `LOG.report` writes CSV to LittleFS at an independent cadence
- **Guard conditions** — continuous safety range validation with `on_error`
- **Supported sensor tokens:**

| Token             | Unit  | Description                                    |
|-------------------|-------|------------------------------------------------|
| `GPS.lat`         | °     | Latitude                                       |
| `GPS.lon`         | °     | Longitude                                      |
| `GPS.alt`         | m     | GPS altitude                                   |
| `GPS.speed`       | km/h  | Ground speed                                   |
| `BARO.alt`        | m     | Barometric altitude                            |
| `BARO.temp`       | °C    | Ambient temperature                            |
| `BARO.pressure`   | Pa    | Atmospheric pressure                           |
| `IMU.accel_x/y/z` | m/s²  | Per-axis acceleration                          |
| `IMU.accel_mag`   | m/s²  | Acceleration vector magnitude √(x²+y²+z²)     |
| `IMU.gyro_x/y/z`  | °/s   | Per-axis angular rate                          |
| `IMU.temp`        | °C    | IMU on-chip temperature                        |

### Script Example

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
  transition to FLIGHT when TC.command == LAUNCH

state FLIGHT:
  on_enter:
    EVENT.info "LIFTOFF"
  every 1000ms:
    HK.report { gps_alt: GPS.alt  baro_alt: BARO.alt  accel: IMU.accel_mag }
  log_every 200ms:
    LOG.report { ax: IMU.accel_x  ay: IMU.accel_y  az: IMU.accel_z
                 gx: IMU.gyro_x  gy: IMU.gyro_y  gz: IMU.gyro_z }
  transition to DESCENT when BARO.alt < 50
  conditions:
    IMU.accel_mag < 200
  on_error:
    EVENT.error "Overload detected"

state DESCENT:
  on_enter:
    EVENT.info "DESCENDING"
  every 2000ms:
    HK.report { gps_lat: GPS.lat  gps_lon: GPS.lon  gps_alt: GPS.alt }
  transition to LANDED when GPS.speed < 1

state LANDED:
  on_enter:
    EVENT.info "LANDED"
```

### AMS Engine Lifecycle

```
IDLE ──[POST /api/mission/activate]──► RUNNING ──[error/abort]──► ERROR
                                           │
                                    tick(now_ms): evaluates conditions
                                    and transitions on every loop() cycle
```

ARM (`POST /api/arm`) injects the TC `LAUNCH` command and validates the engine state before and after injection. If the engine does not respond `RUNNING`, the system transitions to ERROR.

### Example Real Flight Plan

The expected operational workflow is to define the mission on the ground from a simulated flight profile, fly with the AMS already tuned, and then analyse the recorded data offline.

1. **Plan the flight from simulation**
   - Build the rocket model in OpenRocket or a similar simulator.
   - Estimate key events such as launch acceleration, burnout, apogee, descent, and landing.
   - Translate those expected events into AMS thresholds and cadence choices.
   - Example decisions:
     - use `IMU.accel_mag > 30` as a launch detect threshold
     - use `BARO.alt < 50` to detect low-altitude descent
     - log IMU data every `200ms` and send HK every `1000ms`

2. **Prepare the AMS mission**
   - Write a mission file with states such as `WAIT`, `FLIGHT`, `DESCENT`, and `LANDED`.
   - Encode the transitions and safety guards derived from the simulation.
   - Keep radio telemetry compact and reserve higher-rate data for local logging.

   ```ams
   state WAIT:
     on_enter:
       EVENT.info "READY ON PAD"
     transition to FLIGHT when TC.command == LAUNCH

   state FLIGHT:
     every 1000ms:
       HK.report { baro_alt: BARO.alt  gps_alt: GPS.alt  accel: IMU.accel_mag }
     log_every 200ms:
       LOG.report { ax: IMU.accel_x  ay: IMU.accel_y  az: IMU.accel_z
                    gx: IMU.gyro_x  gy: IMU.gyro_y  gz: IMU.gyro_z }
     conditions:
       IMU.accel_mag < 200
     on_error:
       EVENT.error "FLIGHT LIMIT EXCEEDED"
     transition to DESCENT when BARO.alt < 50
   ```

3. **Upload and activate the mission on the vehicle**
   - Connect to the ARES WiFi AP from the ground computer.
   - Upload the script with `PUT /api/missions/<name>.ams`.
   - Check the mission list with `GET /api/missions`.
   - Activate it with `POST /api/mission/activate`.
   - Verify status with `GET /api/status` and `GET /api/mission` before arming.

4. **Fly the mission**
   - Put the system into flight mode and arm it.
   - During flight, ARES evaluates AMS transitions in the main loop, emits housekeeping and event frames over LoRa, and stores higher-rate logs in LittleFS.
   - The radio link gives operational awareness in real time; the onboard log preserves the detailed post-flight record.

5. **Download and inspect the data on the computer**
   - After recovery, reconnect over WiFi.
   - List stored files with `GET /api/storage/list`.
   - Download the relevant log with `GET /api/storage/download?file=<name>`.
   - Plot or inspect the CSV on the computer using Python, a notebook, or a spreadsheet to compare:
     - simulated altitude vs. barometric altitude
     - expected launch acceleration vs. measured `IMU.accel_mag`
     - descent timing vs. GPS speed and altitude

This closes the loop between simulation, mission authoring, flight execution, and post-flight analysis. In practice, each recovered log should feed the next OpenRocket tuning pass and the next AMS revision.

Further reading:
- [AMS programming guide](docs/development/ams_programming_guide.md)
- [AMS standard](docs/standards/ams_standard.md)

---

## Radio Telemetry

### ARES Radio Protocol v2

CRC-32 binary protocol over LoRa UART. Transport-agnostic — works over any UART-based radio backend.

```
[SYNC 4B][VER][FLAGS][NODE][TYPE][SEQ][LEN][PAYLOAD 0-200B][CRC-32 4B]
  0xAE55C31A
```

| Field   | Size    | Description                              |
|---------|---------|------------------------------------------|
| SYNC    | 4 B     | Frame marker `0xAE 0x55 0xC3 0x1A`      |
| VER     | 1 B     | Protocol version (`0x02`)                |
| FLAGS   | 1 B     | ACK, retransmission, fragmentation       |
| NODE    | 1 B     | Sender node ID                           |
| TYPE    | 1 B     | Message type (HK, EVENT, ACK…)           |
| SEQ     | 1 B     | Sequence number 0–255 (wrapping)         |
| LEN     | 1 B     | Payload length (0–200)                   |
| PAYLOAD | 0–200 B | PUS ST[3] or ST[5] data                 |
| CRC-32  | 4 B     | Over bytes [4..9+LEN] (Ethernet poly)    |

The HK telemetry payload is a fixed 38-byte struct (APUS-3.6), aligned with the adapted PUS standard (APUS). The `accelMag` field of the active IMU is the only IMU value mapped directly into the telemetry payload; all other IMU axes are available in local logs only.

ARES does not implement the full ESA PUS stack. Instead it uses an embedded-friendly adaptation called APUS, which keeps the service-oriented structure of PUS while reducing framing and runtime complexity for a small flight computer. In practice that means:

- housekeeping telemetry is mapped to PUS service type ST[3]
- event reporting is mapped to ST[5]
- telecommands and verification use the same service-oriented layout without pulling in a heavyweight packet stack
- the radio frame remains transport-agnostic, so the same protocol logic can sit on LoRa today and another UART radio later

Protocol references:
- [ARES radio protocol](docs/architecture/ares_radio_protocol.md)
- [APUS standard](docs/standards/apus_standard.md) — the embedded adaptation actually implemented by the firmware

### Current Radio

| Parameter     | Value                     |
|---------------|---------------------------|
| Module        | DX-LR03-900               |
| Frequency     | 868 MHz                   |
| TX Power      | 20 dBm (configurable)     |
| Air Data Rate | 2.4 kbps                  |
| UART baud     | 9600                      |

The radio layer is transport-agnostic: only the `DxLr03Driver` and the channel configuration in `config.h` are radio-specific. Swapping to a different UART-based radio module only requires a new driver implementing `RadioInterface`.

---

## WiFi REST API

The ESP32 runs a WiFi Access Point. The ground station connects directly to configure, monitor, and operate the system before launch.

| Parameter | Default value                  |
|-----------|--------------------------------|
| SSID      | `ARES-XXXX` (last 2 bytes MAC) |
| Password  | `ares1234`                     |
| IP        | `192.168.4.1`                  |
| Port      | `80` (HTTP)                    |
| Clients   | Max. 4 simultaneous            |

> The values above are **factory defaults**. SSID, password, the REST API authentication token (`X-ARES-Token` header), and CORS settings are all runtime-configurable via `PUT /api/device/config` and persisted to LittleFS. After a successful update the AP restarts with the new credentials — plan your re-connection accordingly.

Complete endpoint documentation (including when to use each route in pre-flight, flight, and post-flight operations) is available in:

- [WiFi API endpoint guide](docs/api/wifi_api_endpoints.md)

WiFi is automatically disabled when transitioning to flight mode (`WIFI_DISABLE_IN_FLIGHT = true`).

---

## Coding Standards

ARES applies eleven complementary standards, all checked in under `docs/standards/`. The underlying philosophy is: *if a tool cannot verify a property, the code is wrong.* The standards are not decorative — each one closes a specific class of failure that has historically destroyed real flight systems.

### Why These Standards in an Amateur Rocket

A software failure in flight can mean:
- No parachute deployment → loss of rocket or injury
- Accidental pulse channel firing on the pad → serious injury risk
- Telemetry loss → impossible recovery

Professional standards exist precisely to prevent these scenarios. They are not applied for formalism — they are applied because **the physical consequences are real**.

### Standards in the repository

Each standard lives in `docs/standards/` and is explained below with the reason it is enforced in ARES.

- **[DO-178C standard](docs/standards/do178c_standard.md)** — FAA/RTCA process model for safety-critical avionics software. Used as the *process backbone*: requirements → design → code → verification → evidence. ARES does not pursue formal certification, but adopts the discipline of documented requirements, traceability, and per-change verification evidence so that flight-critical changes are reviewable.
- **[Power of 10 (PO10) standard](docs/standards/po10_standard.md)** — NASA JPL's ten rules for safety-critical code. Bans dynamic memory after init, recursion, unbounded loops, `goto`/`setjmp`, ignored return values, and macro abuse. Adopted because it is the smallest set of rules that makes embedded C/C++ statically reasonable.
- **[RTOS standard](docs/standards/rtos_standard.md)** — FreeRTOS SMP discipline: fixed-priority tasks, bounded stacks, mutex with timeout on every critical section, no infinite blocks, watchdog protection. Without it the system cannot make timing guarantees.
- **[MISRA standard](docs/standards/misra_standard.md)** — MISRA C 2012 type-safety subset, enforced by Cppcheck with the MISRA addon (see also the [MISRA C++ review profile](docs/development/misra_cpp_review_profile.md)). Eliminates implicit conversions, signedness traps, and undefined behaviour that compilers will silently accept.
- **[CERT standard](docs/standards/cert_standard.md)** — CERT C secure coding rules adapted to the embedded context (CERT/CC). Focused on input validation and defence against hostile or malformed data, which on ARES means malformed AMS scripts, REST payloads, and RF frames.
- **[APUS standard](docs/standards/apus_standard.md)** — *Amateur Packet Utilisation Standard*. The embedded adaptation of ESA PUS that the firmware actually implements: ST[3] housekeeping, ST[5] events, ST[1]/ST[6] command verification, fixed binary layouts, transport-agnostic framing. This is the contract between flight and ground.
- **[PUS standard](docs/standards/pus_standard.md)** — reference document for the ECSS PUS service model that APUS adapts. Kept in-tree so the lineage of design decisions stays visible and so future contributors can see what was deliberately removed for embedded use.
- **[AMS standard](docs/standards/ams_standard.md)** — the language definition for Ares Mission Script: lexical rules, allowed directives, hard parser limits, validation rules, runtime semantics. This is what makes AMS statically analysable and bounded in size and timing.
- **[APIREST standard](docs/standards/apirest_standard.md)** — in-house contract for the WiFi REST API: route shape, JSON envelope, error model, authentication via `X-ARES-Token`, idempotency rules, behaviour during flight mode. Keeps the ground station independent from firmware revisions.
- **[ARES coding standard](docs/standards/ares_coding_standard.md)** — the umbrella standard that consolidates the rest into a single style: naming, headers, ownership, logging, assertions, deviation markers. It is the document a new contributor reads first.
- **[DOX standard](docs/standards/dox_standard.md)** — Doxygen comment requirements (file/class/function blocks, units, ownership notes). Code without `@brief`/`@param` for public interfaces fails review.

### Standards Summary Table

| Standard   | Origin                 | Focus                                                       |
|------------|------------------------|-------------------------------------------------------------|
| **DO-178C**| FAA/RTCA (avionics)    | Safety-critical software process backbone                    |
| **PO10**   | NASA JPL               | Deterministic and verifiable control flow                    |
| **RTOS**   | FreeRTOS SMP           | Task discipline, mutex+timeout, watchdog                     |
| **MISRA C**| MISRA (automotive)     | Type safety, no implicit conversions, no UB                  |
| **CERT C** | CERT/CC                | Input validation, hostile data defence                       |
| **APUS**   | ESA PUS (adapted)      | Packet integrity, RF link discipline (firmware contract)     |
| **PUS**    | ECSS-E-ST-70-41C       | Reference standard that APUS adapts                          |
| **AMS**    | In-house               | Mission scripting language, parser limits, runtime semantics |
| **APIREST**| In-house               | HTTP REST API contract over WiFi                             |
| **ARES**   | In-house               | Umbrella coding/style standard                               |
| **DOX**    | Doxygen                | Code documentation requirements                              |

### Key Principles

| Principle | Reference rules |
|-----------|----------------|
| **Zero heap in flight** | PO10-3, MISRA-9, RTOS-7 |
| **All loops bounded** | PO10-2, RTOS-1 |
| **No C++ exceptions** | PO10-1, MISRA-21 |
| **No `goto`, `setjmp`, unbounded recursion** | PO10-1 |
| **Return value checked on every non-void call** | PO10-7 |
| **`-Wall -Wextra -Werror`** | DO-178C applied |
| **No `double` (single-precision `float` only)** | MISRA-7, MCU has no double FPU |
| **Mutex + timeout on every critical section** | RTOS-4 |
| **Deviations documented in code** | DO-3 |

Standard deviations (e.g. third-party Arduino code that does not comply with MISRA) are marked with a comment `// PO10-X: <justification>` at the point of use.

Standards references:
- [ARES coding standard](docs/standards/ares_coding_standard.md)
- [AMS standard](docs/standards/ams_standard.md)
- [APIREST standard](docs/standards/apirest_standard.md)
- [APUS standard](docs/standards/apus_standard.md) — embedded adaptation actually implemented
- [PUS standard](docs/standards/pus_standard.md) — ECSS reference that APUS adapts
- [MISRA standard](docs/standards/misra_standard.md) · [MISRA C++ review profile](docs/development/misra_cpp_review_profile.md)
- [CERT standard](docs/standards/cert_standard.md)
- [DO-178C standard](docs/standards/do178c_standard.md)
- [PO10 standard](docs/standards/po10_standard.md)
- [RTOS standard](docs/standards/rtos_standard.md)
- [DOX standard](docs/standards/dox_standard.md)

---

## Requirements Management

> **Work in progress.** The requirements management workflow is still being implemented. The SRS skeleton, the StrictDoc tooling, and the trace links to source code are in place, but coverage is partial and the document is being filled in iteratively as features land. Treat the current SRS as a *living draft*, not as a frozen baseline.

Software requirements are tracked in plain-text [StrictDoc](https://strictdoc.readthedocs.io/) format, which provides bidirectional traceability between requirements and source code, and can export to HTML or PDF.

The initial SRS is derived from the coding standards and from the behaviour implemented in the current firmware (v2.0.0):

- [docs/requirements/SRS.sdoc](docs/requirements/SRS.sdoc) — Software Requirements Specification

**Viewing the requirements locally:**

```bash
pip install strictdoc
strictdoc server docs/requirements/
# Open http://127.0.0.1:5111 in your browser
```

**Exporting to static HTML:**

```bash
strictdoc export docs/requirements/ --output-dir docs/requirements/html/
```

---

## Build and Flash

### Requirements

- [PlatformIO](https://platformio.org/) CLI or VS Code extension
- ESP32-S3 Zero Mini connected via USB

### Build

```bash
pio run
```

Active compiler flags: `-Wall -Wextra -Werror -Wdouble-promotion -std=gnu++17 -fno-exceptions -fno-rtti`

Current memory usage:

| Resource | Usage  | Total  |
|----------|--------|--------|
| Flash    | 27.1%  | 3 MB   |
| RAM      | 24.0%  | 320 KB |

### Flash

```bash
pio run -t upload --upload-port COM3
```

### Serial Monitor

```bash
pio device monitor --baud 115200
```

Structured log format:

```
[   2000] I MAIN: ═══ ARES v2.0.0 ═══
[   2000] I MAIN: board: ESP32-S3 Zero Mini
[   2100] I MAIN: Init complete — LED:OK WiFi:OK LoRa:OK State:OK
```

Log levels: `E` Error · `W` Warning · `I` Info · `D` Debug  
Active level: `ARES_LOG_LEVEL=4` (Debug) in `platformio.ini`.

---

## Testing

Verification is centralised in a single VS Code task, **`CI: Master pipeline`**, which runs every quality check in sequence and produces a unified `master.txt` report under `evidence/<date>/` and `evidence/latest/`. The pipeline exits non-zero if any stage fails, so it is suitable both for local pre-push checks and as the basis for CI.

### Master pipeline stages

| # | Stage                          | Backend                              | What it verifies |
|---|--------------------------------|--------------------------------------|------------------|
| 1 | **Build**                      | `pio run`                            | Firmware compiles for ESP32-S3 under `-Wall -Wextra -Werror -Wdouble-promotion -fno-exceptions -fno-rtti`. Compiler warnings break the build. |
| 2 | **Compile DB**                 | `pio run -t compiledb`               | Generates `compile_commands.json` so the static analysers see exactly the flags the real build uses. |
| 3 | **MISRA Cppcheck**             | `run_cppcheck_misra.ps1`             | Cppcheck with the MISRA C 2012 addon over the compile DB. Counts `[misra-cXX-Y.Z]` violations and writes them to `evidence/latest/cppcheck-misra-addon.txt`. |
| 4 | **Cppcheck Practical (XML)**   | `run_cppcheck_practical_xml.ps1`     | Cppcheck without the MISRA addon: `error`, `warning`, `style`, `performance`, `portability`. XML report at `evidence/latest/cppcheck-practical.xml`. |
| 5 | **clang-tidy**                 | `run_clang_tidy.ps1` (LLVM 22)       | `bugprone-*`, `cppcoreguidelines-*` (subset), `misc-const-correctness`, `modernize-*`, `performance-*`, `readability-*`. Output at `evidence/latest/clang-tidy.txt`. |
| 6 | **Unity tests (`pio test -e sim`)** | `run_sitl_sim.ps1`              | Host-side unit + integration test suite. Counts `X Tests Y Failures Z Ignored`. |
| 7 | **Coverage**                   | `run_coverage.ps1` (gcov)            | Line coverage of the Unity test run, summarised as covered / total / percentage in `evidence/latest/coverage.txt`. |

Individual stages can also be launched on their own from the VS Code task list (`Build: PlatformIO`, `MISRA: Cppcheck (MISRA addon)`, `Cppcheck: Practical (no MISRA addon)`, `clang-tidy: Analyze`, `Test: Native unit tests`, `Coverage: gcov report`, `Test: SITL sim (AMS integration)`).

Latest results of every stage are always mirrored under `evidence/latest/` so the most recent run is one click away regardless of date.

### Test suites under `test/`

The `pio test -e sim` environment runs all of the following Unity suites:

| Suite                         | Scope |
|-------------------------------|-------|
| `test_radio_protocol`         | ARES Radio Protocol v2: framing, CRC-32, fragmentation/reassembly, sequence handling, duplicate suppression. |
| `test_ams_parser`             | AMS lexer/parser: encoding rules, line endings, comment handling, state and transition syntax, hard parser limits from `config.h`. |
| `test_ams_parser_cross`       | Cross-checks the C++ parser against the Python reference parser in `scripts/ams_parser.py` to catch divergence. |
| `test_ams_helpers`            | Internal helper functions of the AMS engine (token resolution, expression evaluation, time scheduling). |
| `test_ams_integration`        | Full mission scripts executed against fake HAL implementations: state transitions, guard violation → `on_error`, `every`/`log_every` cadence, pulse action limits. |
| `test_api_handlers`           | REST handlers in `src/api/**` against an in-memory transport, including `X-ARES-Token` enforcement, config persistence, and storage routes. |
| `test_dispatcher`             | HTTP route dispatcher: method/path matching, 404/405 behaviour, body size limits. |

Shared fakes live under `test/stubs/` and `test/test_ams_helpers/`, so each suite stays focused on one subsystem.

### Hardware-in-the-loop

The `Test: SITL sim (AMS integration)` task additionally runs an AMS-driven scenario simulation on the host (no real radio, no real sensors) and feeds canned profiles from `scripts/mission_*.csv`. This is the closest thing to a flight rehearsal without instrumented hardware and is invoked as part of the master pipeline.

### Static analysis evidence

Full usage, suppression syntax, and evidence management:
- [Static analysis guide](docs/development/static_analysis.md)
- [MISRA C++ review profile](docs/development/misra_cpp_review_profile.md)

VS Code parses Cppcheck and clang-tidy findings directly into the **Problems** panel.

---

## Repository Structure

```
Ares2/
├── src/
│   ├── main.cpp                   # Bootstrap, static allocation
│   ├── config.h                   # All hardware/SW configuration
│   ├── ares_assert.h              # ARES_ASSERT macros
│   ├── ams/                       # AMS engine (state machine)
│   ├── api/                       # REST API (WiFi)
│   │   ├── flight/                # ARM, flight control
│   │   ├── mission/               # AMS activation/query
│   │   ├── config/                # Runtime configuration
│   │   ├── status/                # System status
│   │   └── storage/               # Logs, download
│   ├── comms/                     # ARES Radio Protocol v2
│   ├── debug/                     # Structured logger
│   ├── drivers/
│   │   ├── baro/                  # Bmp280Driver
│   │   ├── gps/                   # Bn220Driver
│   │   ├── imu/                   # Adxl375Driver (primary), Mpu6050Driver (secondary)
│   │   └── radio/                 # DxLr03Driver
│   ├── hal/
│   │   ├── baro/                  # BarometerInterface
│   │   ├── gps/                   # GpsInterface
│   │   ├── imu/                   # ImuInterface
│   │   ├── led/                   # LedInterface
│   │   ├── radio/                 # RadioInterface
│   │   └── storage/               # StorageInterface
│   └── sys/
│       ├── led/                   # NeopixelDriver, StatusLed
│       ├── storage/               # LittleFsStorage
│       └── wifi/                  # WifiAp
├── docs/
│   ├── api/                       # REST API documentation
│   ├── architecture/              # Radio protocol
│   ├── development/               # Development and testing guides
│   ├── hardware/                  # Hardware registry, pin map
│   └── standards/                 # 8 documented standards
├── ams_examples/                  # AMS script examples
├── scripts/                       # Python tools (AMS parser, TUI)
├── evidence/latest/               # Latest static analysis results
├── platformio.ini                 # Build configuration
└── partitions.csv                 # Flash partition map
```

---

## Future Work

### 1. Stronger DO-178C-Inspired Practices

- Expand verification evidence beyond build + static analysis into a more complete test pyramid.
- Add clearer traceability from requirement -> AMS behavior -> test case -> evidence artifact.
- Introduce stricter review/checklist gates for flight-critical changes (arming, state transitions, telemetry integrity, storage integrity).
- Formalize qualification-style regression packs for release candidates.

### 2. New Hardware Roadmap

- Evaluate and integrate additional sensors/modules needed for higher-fidelity flight control and post-flight analysis.
- Extend HAL interfaces first, then provide concrete drivers to preserve application-layer portability.
- Prioritized hardware candidates:
  - Redundant barometric/GNSS sensing for cross-check and fault detection.
  - Storage expansion (e.g., SD over SPI) for longer high-rate logs.
  - Future radio options while keeping the same ARES protocol framing model.
- Document each hardware addition in registry, pin map, and standards/deviation notes before flight use.

---

## License

This project is free software: you can redistribute it and/or modify it under the terms of the
[GNU General Public License v3.0](https://www.gnu.org/licenses/gpl-3.0.html) as published by the
Free Software Foundation, either version 3 of the License, or (at your option) any later version.

This software is distributed in the hope that it will be useful, but **WITHOUT ANY WARRANTY**;
without even the implied warranty of **MERCHANTABILITY** or **FITNESS FOR A PARTICULAR PURPOSE**.
See the [LICENSE](LICENSE) file for more details.

> Amateur development project. Not intended for commercial use or certified applications.

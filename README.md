# ARES — Amateur Rocket Embedded System

WARNING: for now this is a beta version — use it at your own risk until a final and LTS version is released!

**Avionics firmware for amateur rockets on ESP32-S3**

[![Build](https://img.shields.io/badge/build-passing-brightgreen)](#build-and-flash)
[![Version](https://img.shields.io/badge/version-0.1.0-blue)](#)
[![Platform](https://img.shields.io/badge/platform-ESP32--S3-orange)](#hardware)
[![Standard](https://img.shields.io/badge/standard-138%20rules-red)](#coding-standards)

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

ARES is an embedded flight computer for amateur rockets. It combines in a single compact module (ESP32-S3):

- **Sensor reading** — barometer, GPS, IMU (accelerometer/gyroscope)
- **Flight logging** — logs stored on internal flash (LittleFS)
- **Wireless telemetry** — LoRa downlink with a custom binary protocol (ARES Radio Protocol v2)
- **Programmable missions** — AMS script engine that lets you define flight logic in plain text
- **Ground configuration** — HTTP REST API over an integrated WiFi AP

The design applies safety-critical software disciplines (NASA JPL Power of 10, DO-178C, MISRA C, CERT C) adapted to the amateur scale, because **a software failure can destroy the rocket or injure people**. Formal certification is not the goal — the discipline it enforces is.

In practical terms, the application runs as a small flight stack with a very explicit lifecycle:

1. Boot the board and bring up sensors, storage, radio, WiFi AP, and status LED.
2. Connect from the ground station over WiFi to inspect status and upload an AMS mission.
3. Activate the mission, switch to flight mode, and arm the system.
4. Let the AMS engine evaluate transitions and guards on every main loop tick while RTOS tasks handle API, LED, and radio work.
5. Downlink housekeeping and events over LoRa while keeping higher-rate logs in LittleFS for post-flight analysis.

Related documents:
- [Hardware registry](docs/hardware/hardware_registry.md)
- [Pin map](docs/hardware/pin_map.md)
- [AMS programming guide](docs/development/ams_programming_guide.md)
- [ARES radio protocol](docs/architecture/ares_radio_protocol.md)
- [MISRA C++ review profile](docs/development/misra_cpp_review_profile.md)

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
| USB       | Native CDC + CH340 UART (COM3/COM6)           |
| Firmware  | v0.1.0                                         |

### Current Peripherals

| Peripheral          | Interface | Function                                       |
|---------------------|-----------|------------------------------------------------|
| **DX-LR03-433T30D** | UART2     | LoRa 433/868 MHz (current profile), telemetry downlink |
| **BN-220**          | UART1     | u-blox GNSS, NMEA 0183 (GGA + RMC)            |
| **BMP280**          | I2C       | Barometer + temperature (barometric altitude)  |
| **ADXL375**         | I2C       | Primary IMU — 3-axis accelerometer ±200 g (shock-rated) |
| **MPU-6050**        | I2C       | Secondary IMU — 6-axis (accelerometer ±2g + gyro ±250°/s) |
| **WS2812B**         | GPIO 21   | RGB status LED (Neopixel)                      |

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
| 4    | Pyro DROGUE    | Drogue parachute channel (reserved)|
| 15   | Pyro MAIN      | Main parachute channel (reserved)  |
| 21   | WS2812B LED    | RGB status                         |

Detailed hardware references:
- [Hardware registry](docs/hardware/hardware_registry.md)
- [Pin map](docs/hardware/pin_map.md)

---

## Firmware Architecture

The firmware follows a **HAL → Driver → Engine** pattern where all application logic operates against pure virtual interfaces, with no direct dependency on concrete hardware.

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

**Why HAL:** Any sensor or radio can be swapped without touching mission logic. The radio protocol unit tests compile on native desktop (no Arduino dependency) thanks to this separation.

### Runtime Flow

`main.cpp` performs static allocation and initialises the platform. After boot:

- The AMS engine owns mission state, transition evaluation, guard checks, and local logging/telemetry scheduling.
- The API server exposes a ground-operations surface over WiFi for configuration, mission upload/activation, and arming.
- HAL interfaces isolate application logic from specific devices such as the BMP280, BN-220, ADXL375, MPU-6050, and DX-LR03.
- The radio path emits compact PUS-adapted binary frames, while LittleFS preserves higher-rate logs that do not fit the telemetry budget.

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

AMS is the ARES mission language. It lets you define flight logic in a plain-text `.ams` file that is uploaded to flash and executed without recompiling the firmware.

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

The HK telemetry payload is a fixed 38-byte struct (APUS-3.6), aligned with the adapted PUS standard (APUS). The `accelMag` field of the active IMU (ADXL375 by default) is the only IMU value mapped directly into the telemetry payload; all other IMU axes are available in local logs only.

ARES does not implement the full ESA PUS stack. Instead it uses an embedded-friendly adaptation called APUS, which keeps the service-oriented structure of PUS while reducing framing and runtime complexity for a small flight computer. In practice that means:

- housekeeping telemetry is mapped to PUS service type ST[3]
- event reporting is mapped to ST[5]
- telecommands and verification use the same service-oriented layout without pulling in a heavyweight packet stack
- the radio frame remains transport-agnostic, so the same protocol logic can sit on LoRa today and another UART radio later

Protocol references:
- [ARES radio protocol](docs/architecture/ares_radio_protocol.md)
- [APUS standard](docs/standards/apus_standard.md)
- [PUS standard](docs/standards/pus_standard.md)

### Current Radio

| Parameter     | Value                     |
|---------------|---------------------------|
| Module        | DX-LR03-433T30D           |
| Frequency     | 433.125 MHz (CH23, current) / 868 MHz (optional profile with compatible module/settings) |
| TX Power      | 20 dBm (configurable)     |
| Air Data Rate | 2.4 kbps                  |
| UART baud     | 9600                      |
| WiFi in flight| Disabled (REST-13.3)      |

The current hardware profile is 433 MHz. An 868 MHz profile is also supported in documentation/standards when using a compatible radio module and regional configuration.

---

## WiFi REST API

The ESP32 runs a WiFi Access Point. The ground station connects directly to configure, monitor, and operate the system before launch.

| Parameter | Value                          |
|-----------|--------------------------------|
| SSID      | `ARES-XXXX` (last 2 bytes MAC) |
| Password  | `ares1234`                     |
| IP        | `192.168.4.1`                  |
| Port      | `80` (HTTP)                    |
| Clients   | Max. 4 simultaneous            |

Complete endpoint documentation (including when to use each route in pre-flight, flight, and post-flight operations) is available in:

- [WiFi API endpoint guide](docs/api/wifi_api_endpoints.md)

WiFi is automatically disabled when transitioning to flight mode (`WIFI_DISABLE_IN_FLIGHT = true`).

---

## Coding Standards

ARES enforces **138 rules** organised across eight complementary standards. The underlying philosophy is: *if a tool cannot verify a property, the code is wrong.*

The standards are not decorative. They shape the architecture directly:

- MISRA-style restrictions push the code toward explicit types, bounded control flow, and predictable behaviour on a microcontroller with limited observability.
- ESA PUS and the local APUS adaptation give telemetry, events, and telecommands a service-oriented structure instead of ad-hoc packet definitions.
- Power of 10 and RTOS rules constrain concurrency, memory use, and failure handling so that the runtime remains understandable under stress.

### Why These Standards in an Amateur Rocket

A software failure in flight can mean:
- No parachute deployment → loss of rocket or injury
- Accidental pyro firing on the pad → serious injury risk
- Telemetry loss → impossible recovery

Professional standards exist precisely to prevent these scenarios. They are not applied for formalism — they are applied because **the physical consequences are real**.

### Standards Table

| Standard   | Origin                 | Focus                                        | Rules |
|------------|------------------------|----------------------------------------------|-------|
| **DO-178C**| FAA/RTCA (avionics)    | Safety-critical software development process | 18    |
| **PO10**   | NASA JPL               | Deterministic and verifiable control flow    | 10    |
| **RTOS**   | FreeRTOS SMP           | Task discipline and concurrency              | 19    |
| **MISRA C**| MISRA (automotive)     | Type safety, predictable behaviour           | 22    |
| **CERT C** | CERT/CC                | Input validation, hostile data defence       | 22    |
| **APUS**   | ESA PUS (adapted)      | Packet integrity, RF link discipline         | 25    |
| **DOX**    | Doxygen                | Code documentation                           | 8     |
| **REST**   | In-house               | HTTP REST API over WiFi                      | 14    |

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

#### MISRA in This Project

MISRA here is used as an engineering constraint, not as a branding claim. The practical goal is to remove classes of bugs that are expensive to debug in embedded flight software: implicit narrowing conversions, ambiguous ownership, unchecked return paths, surprising control flow, and implementation-defined behaviour. The project is not claiming formal MISRA certification; instead it uses a review profile and deviation process so that non-compliant code is visible, justified, and contained.

#### ESA PUS and APUS in This Project

ESA PUS is the reference model behind the telemetry and command vocabulary. ARES uses APUS, a reduced adaptation for this embedded context, to keep the useful parts of PUS:

- stable service numbering for housekeeping, events, verification, and commands
- predictable packet semantics between the rocket and ground station
- traceable mapping between AMS actions and radio frames

That makes the telemetry link easier to test, version, and inspect than a one-off binary protocol with no service model.

Standards references:
- [ARES coding standard](docs/standards/ares_coding_standard.md)
- [MISRA standard](docs/standards/misra_standard.md)
- [MISRA C++ review profile](docs/development/misra_cpp_review_profile.md)
- [APUS standard](docs/standards/apus_standard.md)
- [PUS standard](docs/standards/pus_standard.md)
- [DO-178C standard](docs/standards/do178c_standard.md)
- [PO10 standard](docs/standards/po10_standard.md)
- [RTOS standard](docs/standards/rtos_standard.md)

---

## Requirements Management

Software requirements are tracked in plain-text [StrictDoc](https://strictdoc.readthedocs.io/) format, which provides bidirectional traceability between requirements and source code, and can export to HTML or PDF.

The initial SRS is derived from the coding standards and from the behaviour implemented in the current firmware (v0.1.0):

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
[   2000] I MAIN: ═══ ARES v0.1.0 ═══
[   2000] I MAIN: board: ESP32-S3 Zero Mini
[   2100] I MAIN: Init complete — LED:OK WiFi:OK LoRa:OK State:OK
```

Log levels: `E` Error · `W` Warning · `I` Info · `D` Debug  
Active level: `ARES_LOG_LEVEL=4` (Debug) in `platformio.ini`.

---

## Testing

Formal unit and hardware integration tests are planned for a future milestone. The test infrastructure (PlatformIO Unity runner, native environment for protocol tests) is already in place and will be expanded as the project matures.

### Static analysis

Static analysis is the primary verification tool today. Two analysers are configured and run via VS Code tasks:

#### Cppcheck

| Task | Description |
|------|-------------|
| `Cppcheck: Practical (no MISRA addon)` | Fast day-to-day check: warning, style, performance, portability |
| `MISRA: Build + Cppcheck` | Full pipeline: build → compile database → MISRA C 2012 addon |
| `Build + Cppcheck practical` | Full pipeline without MISRA |

#### clang-tidy (LLVM 22)

| Task | Description |
|------|-------------|
| `clang-tidy: Analyze` | Runs checks on `src/` using `.clang-tidy` config (requires fresh `compile_commands.json`) |
| `clang-tidy: Build + Analyze` | Full pipeline: build → compile database → clang-tidy |

Active check groups: `bugprone-*`, `cppcoreguidelines-*` (subset), `misc-const-correctness`, `modernize-*` (idioms), `performance-*`, `readability-*`. Checks that generate noise from Arduino/ESP-IDF framework headers are suppressed. VS Code parses findings directly into the **Problems** panel.

Latest analysis output is stored in `evidence/latest/`.

Full usage, suppression syntax, and evidence management:
- [Static analysis guide](docs/development/static_analysis.md)
- [MISRA C++ review profile](docs/development/misra_cpp_review_profile.md)

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

### 2. Pyro Actions and Flight Actuation

- Implement explicit AMS actions for pyrotechnic channels (DROGUE / MAIN), not only reserved pins.
- Add interlocks and arming conditions for pyro commands (mode, altitude/time windows, one-shot behavior, timeout watchdog).
- Add dedicated telemetry/events for pyro state transitions and fault reasons.
- Define and test fail-safe behavior for abort/error paths when pyro conditions are not met.

### 3. New Hardware Roadmap

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

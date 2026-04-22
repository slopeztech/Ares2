# ARES Coding Standard

**Amateur Rocket Embedded System**
Coding standard for ESP32-S3 / FreeRTOS avionics firmware.
Inspired by NASA JPL Power of 10, DO-178C, MISRA C, and CERT C,
adapted for hobby rocketry with safety-critical priorities.

---

## Context

Embedded software for amateur rocket avionics. Not the Voyager, but a
failure can destroy a rocket and injure people. Priorities: readability,
verifiability, determinism.

These standards are a goal to strive for, not a rigid mandate. Code
should comply as closely as possible; deviations are acceptable only
when justified by hardware constraints or third-party library limitations,
and must be documented with a comment citing the rule that is relaxed
(see Deviation Procedure below).

---

## Target Platform

| Property               | Detail                                        |
|------------------------|-----------------------------------------------|
| MCU                    | ESP32-S3 (dual-core Xtensa LX7, 240 MHz)     |
| RTOS                   | IDF FreeRTOS (based on Vanilla v10.5.1, SMP)  |
| SRAM                   | ~512 KB                                       |
| Flash                  | 8–16 MB (QSPI)                                |
| FPU                    | Single-precision only (`float`); no `double`  |
| MMU                    | None (flat address space)                     |
| Toolchain              | PlatformIO + ESP-IDF framework                |
| Language               | C++17 (limited subset — see MISRA/CERT rules) |

---

## Standards Index

ARES follows eight complementary coding standards. Each is documented
in its own file with detailed rules, examples, and cross-references.

| Standard | Focus | Rules | Count |
|----------|-------|-------|-------|
| [DO-178C](do178c_standard.md) | Safety-critical development process | DO-1 … DO-18 | 18 |
| [PO10](po10_standard.md) | Deterministic, verifiable code (NASA JPL) | PO10-1 … PO10-10 | 10 |
| [RTOS](rtos_standard.md) | FreeRTOS task discipline (ESP32-S3 SMP) | RTOS-1 … RTOS-19 | 19 |
| [MISRA C](misra_standard.md) | Type safety, predictable behavior | MISRA-1 … MISRA-22 | 22 |
| [CERT C](cert_standard.md) | Input validation, hostile data defense | CERT-1 … CERT-22 | 22 |
| [APUS](pus_standard.md) | Packet integrity, RF discipline (ESA PUS) | APUS-1 … APUS-25 | 25 |
| [DOX](dox_standard.md) | Doxygen documentation discipline | DOX-1 … DOX-8 | 8 |
| [REST API](apirest_standard.md) | HTTP REST API for WiFi ground config | REST-1 … REST-14 | 14 |

**Total: 138 rules** across eight standards.

### Quick-Reference: Key Principles

1. **No dynamic memory after init** (PO10-3, MISRA-9, RTOS-7)
2. **All loops bounded** (PO10-2, RTOS-1)
3. **All external input validated** (PO10-5, CERT-1, CERT-5)
4. **Timeouts on all blocking calls** (DO-6, RTOS-8, CERT-10)
5. **Fixed-width types only** (MISRA-1)
6. **Zero warnings** (PO10-10, DO-5)
7. **Tasks isolated via RTOS primitives** (DO-7, RTOS-4)
8. **Named constants, no magic numbers** (PO10-8, MISRA-7, RTOS-5)
9. **Functions ≤ 60 lines** (PO10-4, MISRA-12)
10. **Fail safe on error — never fire pyros on unexpected state** (DO-6, CERT-8)

### When to Consult Each Standard

| Situation                                         | Start With        |
|---------------------------------------------------|-------------------|
| Creating or modifying a FreeRTOS task              | RTOS              |
| Writing an ISR or hardware driver                  | RTOS, MISRA       |
| Handling external input (radio, UART, sensors)     | CERT              |
| Choosing types, casts, or arithmetic operations    | MISRA             |
| Designing the radio protocol or packet format      | APUS              |
| Planning test strategy or verification             | DO-178C           |
| General code structure, loops, memory, assertions  | PO10              |
| Safety analysis or failure modes                   | DO-178C, CERT     |
| Documenting a public API, file, or struct          | DOX               |
| Designing or implementing the WiFi REST API        | REST API          |
| Code review checklist                              | All — Key Principles |

---

## Naming Conventions

| Element      | Style          | Example                              |
|------------- |--------------- |--------------------------------------|
| Files        | `snake_case`   | `led_driver.h`, `flight_state.cpp`   |
| Classes      | `PascalCase`   | `FlightState`, `GpsDriver`           |
| Functions    | `camelCase`    | `getData()`, `setColor()`            |
| Constants    | `UPPER_SNAKE`  | `MAX_ALTITUDE`, `PIN_LED_RGB`        |
| Variables    | `camelCase`    | `sensorData`, `loopCount`            |
| Members      | `trailing_`    | `data_`, `pin_`                      |
| Enums        | `PascalCase` type, `UPPER_SNAKE` values | `FlightPhase::BOOST` |
| Namespaces   | `snake_case`   | `hal`, `colors`, `ares::priority`    |
| Macros       | `ARES_` prefix | `ARES_ASSERT`, `ARES_VERSION_STRING` |
| Task names   | `snake_case`   | `"sensor"`, `"radio_tx"`             |

---

## Formatting

- Indentation: 4 spaces, never tabs.
- Braces: BSD/Allman style (opening brace on its own line).
- Maximum line length: 100 columns.
- One space after keywords (`if`, `for`, `while`, `switch`).
- No space after function name in calls.
- File header: module name and one-line description.
- Blank line between logical sections inside a function.
- No trailing whitespace.

---

## File Structure

Each module (driver, service, etc.) has:
- `module.h` — public declaration (struct, class, functions)
- `module.cpp` — implementation

**Header (.h):**
1. `#pragma once`
2. System includes (`<stdint.h>`, `<freertos/FreeRTOS.h>`)
3. Project includes (`"config.h"`, etc.)
4. Forward declarations if needed
5. Constants and types
6. Class or function declarations

**Source (.cpp):**
1. Include its own `.h`
2. Additional includes as needed
3. `static` file-scope constants
4. Implementation

---

## Language Policy

- All source code, comments, documentation, and commit messages must be
  written in **English**.

---

## Documentation (Doxygen)

See [DOX Standard](dox_standard.md) for the complete set of
Doxygen documentation rules (DOX-1 … DOX-8).

---

## Deviation Procedure

When a rule cannot be followed, the deviation must be documented
**in the code** at the point of deviation:

```cpp
// DEVIATION: RTOS-7.1 — Using xTaskCreate (dynamic) because
// third-party library X requires it. Stack size bounded to 4096.
// Approved: [initials] [date]
xTaskCreate(thirdPartyTask, "lib_x", 4096, nullptr, 3, nullptr);
```

Requirements for a valid deviation:
1. **Rule ID**: Cite the exact rule being relaxed.
2. **Justification**: Why the rule cannot be followed.
3. **Mitigation**: What alternative safeguard is in place.
4. **Approval**: Initials and date of the reviewer who approved it.

Deviations are never retroactive — they must be documented before
the code is merged.

---

## Build and Tooling

| Tool           | Purpose                              | Reference          |
|----------------|--------------------------------------|---------------------|
| PlatformIO     | Build system, dependency management  | `platformio.ini`    |
| ESP-IDF        | Framework, FreeRTOS, HAL             | v5.x                |
| Compiler       | Xtensa GCC (`-Wall -Wextra -Werror`) | PO10-10, DO-5       |
| Static analysis| Compiler warnings as errors          | PO10-10             |
| Cppcheck       | Temporary MISRA-oriented code audits | `.vscode/tasks.json` |
| Partitions     | Flash partition layout               | `partitions.csv`    |

### Temporary MISRA Audit Policy

Until a certifiable MISRA C++ toolchain is fully integrated, ARES uses
Cppcheck tasks as the practical audit entry point for MISRA-related
review cycles.

This is a temporary project policy and does not replace formal
MISRA C++ compliance evidence requirements.

### Static Analysis Output Persistence

All static-analysis tasks added for testing/review must persist their
results to files so findings can be reviewed offline and traced in
release evidence.

Current required output directory:
- `evidence/latest/`

Historical retention (date rotation) for each run:
- `evidence/YYYY-MM-DD/`

Current required outputs from Cppcheck tasks:
- `cppcheck-practical.txt`
- `cppcheck-practical.xml`
- `cppcheck-misra-addon.txt`

Each run must write to the dated folder and then update the matching
file under `latest/`.

### Required compiler flags

```ini
# platformio.ini
build_flags =
    -Wall
    -Wextra
    -Werror
    -Wno-unused-parameter
    -DCONFIG_FREERTOS_CHECK_STACKOVERFLOW=2
```

---

## Related Documentation

| Document | Location |
|----------|----------|
| Pin map | [hardware/pin_map.md](../hardware/pin_map.md) |
| Hardware registry | [hardware/hardware_registry.md](../hardware/hardware_registry.md) |
| GPS module | [hardware/gps_bn220.md](../hardware/gps_bn220.md) |
| Radio protocol | [architecture/ares_radio_protocol.md](../architecture/ares_radio_protocol.md) |
| Wi-Fi API | [api/ares_wifi_api.md](../api/ares_wifi_api.md) |
| Testing guide | [development/how_to_test.md](../development/how_to_test.md) |

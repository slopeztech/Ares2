# ARES DOX Standard — Doxygen Documentation Discipline

**Amateur Rocket Embedded System**
Documentation rules for Doxygen-compatible comments in
ESP32-S3 avionics firmware.
Companion to the [ARES Coding Standard](ares_coding_standard.md).

---

## Context

All public API surfaces must be documented using Doxygen-compatible
comments. The documentation is the **contract** between modules —
reviewers and static analysers rely on it.

---

## Comment Style

Use `/** ... */` (Javadoc style) for Doxygen blocks and `///` for
single-line member documentation. Never use `/*! */` or `//!`.

```cpp
/**
 * @brief One-sentence summary.
 *
 * Extended description (optional, only when non-obvious).
 */

uint8_t satellites = 0;  ///< Satellites in use.
```

---

## DOX-1 — File Headers

Every `.h` and `.cpp` file must begin with a Doxygen file block:

```cpp
/**
 * @file  filename.h
 * @brief One-sentence purpose of this module.
 *
 * Extended description: design rationale, thread safety
 * guarantees, hardware assumptions, or references to
 * datasheets / standards.
 */
```

| ID      | Rule                                                         |
|---------|--------------------------------------------------------------|
| DOX-1.1 | `@file` must match the actual filename.                      |
| DOX-1.2 | `@brief` must be a single sentence — no implementation details. |
| DOX-1.3 | Thread safety and ISR context must be stated when relevant.  |

---

## DOX-2 — Classes and Structs

Every `class` and `struct` must have a Doxygen block explaining its
purpose and usage context.

```cpp
/**
 * A single barometer measurement.
 * All fields default-initialised to zero (MISRA-4).
 */
struct BaroReading { ... };
```

| ID      | Rule                                                         |
|---------|--------------------------------------------------------------|
| DOX-2.1 | Document invariants, ownership, and lifetime.                |
| DOX-2.2 | If the struct is serialised to wire format, reference the APUS rule. |
| DOX-2.3 | Members with non-obvious units must use `///< Unit.` trailing comments. |

---

## DOX-3 — Functions and Methods

All public and protected functions must have a Doxygen block.
Private helpers may use a plain `//` comment if trivial.

```cpp
/**
 * Read the latest measurement from the sensor.
 * @param[out] out  Populated with the reading on BaroStatus::OK.
 * @return Status code indicating result (see BaroStatus).
 * @pre  begin() returned true.
 * @post Sensor registers are idle after read completes.
 */
virtual BaroStatus read(BaroReading& out) = 0;
```

| ID      | Rule                                                         |
|---------|--------------------------------------------------------------|
| DOX-3.1 | `@brief` (or first sentence) must describe **what**, not **how**. |
| DOX-3.2 | Every parameter must have `@param[in]`, `@param[out]`, or `@param[in,out]`. |
| DOX-3.3 | Non-void functions must have `@return`.                      |
| DOX-3.4 | Preconditions → `@pre`; postconditions → `@post`.            |
| DOX-3.5 | Functions that may fail must document failure modes.          |

---

## DOX-4 — Enums

Document the enum type and each enumerator that is not self-evident.

```cpp
/**
 * Status codes returned by BarometerInterface::read().
 */
enum class BaroStatus : uint8_t
{
    OK        = 0,   ///< Valid reading available.
    ERROR     = 1,   ///< Communication or hardware error.
    NOT_READY = 2,   ///< Sensor not yet initialised.
    ...
};
```

| ID      | Rule                                                         |
|---------|--------------------------------------------------------------|
| DOX-4.1 | Every enum type must have a Doxygen block.                   |
| DOX-4.2 | Sentinel values (`FIRST`, `LAST`) may use `//` instead of `///`. |

---

## DOX-5 — Constants and Configuration

Named constants in `config.h` and namespace-scope `constexpr` values
must have a trailing `///< ...` comment or a preceding Doxygen block.

```cpp
constexpr uint32_t SERIAL_WAIT_TIMEOUT_MS = 3000U;  ///< Max wait for USB serial.
```

| ID      | Rule                                                         |
|---------|--------------------------------------------------------------|
| DOX-5.1 | Units must be explicit in the comment or the constant name.  |
| DOX-5.2 | If a constant comes from a datasheet, cite the section.      |

---

## DOX-6 — Deviations and Warnings

Use `@warning` or `@note` for deviation comments that also need
to be visible in generated documentation.

```cpp
/**
 * @warning DEVIATION: MISRA-18.1 — double used for lat/lon.
 *          Approved: SL 2026-04-20.
 */
```

| ID      | Rule                                                         |
|---------|--------------------------------------------------------------|
| DOX-6.1 | Deviations visible in the API must use `@warning DEVIATION:`. |
| DOX-6.2 | Implementation-only deviations may use plain `// DEVIATION:`. |

---

## DOX-7 — Prohibited Practices

| ID      | Rule                                                         |
|---------|--------------------------------------------------------------|
| DOX-7.1 | Do not document obvious code (`i++; ///< Increment i`).     |
| DOX-7.2 | Do not duplicate the function signature in prose.            |
| DOX-7.3 | Do not add `@author` or `@date` — use version control.      |
| DOX-7.4 | Do not leave `@todo` in released code — use the issue tracker. |

---

## DOX-8 — Inline Implementation Comments

Non-Doxygen comments (`//`) inside function bodies are encouraged
for complex logic, state machines, multi-step algorithms, and
register manipulations.

| ID      | Rule                                                         |
|---------|--------------------------------------------------------------|
| DOX-8.1 | Comment **why**, not **what**, unless the code is non-obvious. |
| DOX-8.2 | Multi-step processes should use numbered phase comments (`// Phase 1: ...`). |
| DOX-8.3 | Register-level operations must reference the datasheet section. |
| DOX-8.4 | Magic numbers from datasheets must cite the source (or use named constants per MISRA-7). |

---

## Tag Quick Reference

| Tag            | Usage                                      |
|----------------|--------------------------------------------|
| `@file`        | File-level documentation                   |
| `@brief`       | One-sentence summary                       |
| `@param[dir]`  | Function parameter (`in`, `out`, `in,out`) |
| `@return`      | Return value description                   |
| `@pre`         | Precondition                               |
| `@post`        | Postcondition                              |
| `@warning`     | Safety or deviation notice                 |
| `@note`        | Important usage note                       |
| `@see`         | Cross-reference to related symbol/document |
| `@retval`      | Specific return value meaning              |

---

## Cross-References

| DOX Rule | Related Standards                        |
|----------|------------------------------------------|
| DOX-1    | PO10-4 (file structure)                  |
| DOX-2    | MISRA-4 (initialisation), APUS-6 (wire)  |
| DOX-3    | DO-8 (traceability), CERT-5 (validation) |
| DOX-5    | MISRA-7 (named constants), PO10-8        |
| DOX-6    | Deviation Procedure (ares_coding_standard)|
| DOX-8    | MISRA-7 (magic numbers)                  |

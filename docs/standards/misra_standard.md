# ARES MISRA C/C++ Standard

**Amateur Rocket Embedded System**
Determinism, type safety, and predictable behavior for ESP32-S3
embedded firmware. Derived from MISRA C:2023 (Amendment 4) and
MISRA C++:2023, adapted for amateur rocketry on FreeRTOS.
Companion to the [ARES Coding Standard](ares_coding_standard.md).

---

## Context

MISRA guidelines target safety-critical embedded systems where
undefined behavior, ambiguous types, or hidden side effects can
cause catastrophic failures. ARES selects the rules most relevant
to its hardware and mission profile.

### Platform constraints

| Constraint                | Impact on Rules                              |
|---------------------------|----------------------------------------------|
| ESP32-S3 — 32-bit Xtensa  | `int` = 32 bits, but **portability** demands `stdint.h` types |
| No FPU for `double`       | `float` only; `double` silently goes to software emulation    |
| 512 KB SRAM               | Every byte counts; fixed-size buffers, no heap fragmentation  |
| Dual-core, FreeRTOS       | Shared-state rules critical (see CERT-13, RTOS-4)             |
| Safety-critical (pyros)   | Deterministic behavior is a safety requirement, not a style preference |

### MISRA rule categorization

Each ARES MISRA rule maps to one or more official MISRA C:2023 /
MISRA C++:2023 directives or rules. The categorization follows:

| Category    | Meaning                                          |
|-------------|--------------------------------------------------|
| **Required**| Must be followed; deviations need formal justification |
| **Advisory**| Should be followed; deviations documented in code |

All ARES MISRA rules are **Required** unless explicitly marked
Advisory.

---

## MISRA-1 — Fixed-Width Types Only

**MISRA C:2023 Dir 4.6 — Use typedefs for basic numerical types**
**Category:** Required

Use only `<stdint.h>` / `<cstdint>` fixed-width types. Platform-
dependent types make code behavior unpredictable across compilers
and architectures.

### MISRA-1.1 — Allowed types

| Type Family   | Allowed Types                                    |
|---------------|--------------------------------------------------|
| Unsigned int  | `uint8_t`, `uint16_t`, `uint32_t`, `uint64_t`   |
| Signed int    | `int8_t`, `int16_t`, `int32_t`, `int64_t`       |
| Boolean       | `bool`                                            |
| Floating      | `float` only (see MISRA-18)                      |
| Size/index    | `size_t` (only for `sizeof` results and stdlib)  |
| Byte          | `uint8_t` (for raw byte buffers)                 |

### MISRA-1.2 — Forbidden types

| Forbidden              | Replacement        | Reason                   |
|------------------------|--------------------|--------------------------|
| `int`                  | `int32_t`          | Size depends on platform |
| `unsigned int`         | `uint32_t`         | Size depends on platform |
| `long`                 | `int32_t`/`int64_t`| Size is 32 or 64 bits depending on ABI |
| `short`                | `int16_t`          | Size depends on platform |
| `char` (for arithmetic)| `uint8_t`/`int8_t`| Signedness is implementation-defined |
| `double`               | `float`            | No HW FPU on ESP32-S3   |

**Exception:** `char` is allowed for null-terminated C strings and
string literals. `int` is allowed in `main()` return type and where
required by third-party API signatures (wrapped in adapter layer,
see DO-14).

```cpp
// ❌
int counter;
unsigned long timestamp;

// ✅
uint32_t counter = 0;
uint32_t timestamp = 0;
```

---

## MISRA-2 — No Implicit Conversions

**MISRA C:2023 Rule 10.1–10.8 — Essential type model / Conversions**
**Category:** Required

All type conversions must be explicit. Implicit conversions hide
data loss, sign changes, and precision changes that cause silent
bugs.

### MISRA-2.1 — Dangerous implicit conversions

| Conversion              | Risk                                    | Example                |
|-------------------------|-----------------------------------------|------------------------|
| Signed → unsigned       | Negative becomes large positive         | `-1` → `0xFFFFFFFF`   |
| Unsigned → signed       | Large value wraps to negative           | `0x80000000` → `-2^31`|
| Wide → narrow           | Truncation / data loss                  | `uint32_t` → `uint8_t`|
| Float → integer         | Fractional part silently discarded      | `3.7f` → `3`          |
| Integer → float         | Precision loss for large values         | `16777217` → `16777216.0f` |

### MISRA-2.2 — Correct conversion pattern

```cpp
// ❌ Implicit narrowing — silent truncation
uint8_t len = totalSize;

// ✅ Explicit cast — intent is clear, reviewer can verify
ARES_ASSERT(totalSize <= UINT8_MAX);
uint8_t len = static_cast<uint8_t>(totalSize);
```

### MISRA-2.3 — Arithmetic promotion awareness

In C/C++, arithmetic on types smaller than `int` is promoted to
`int` before computation. Be explicit about the final type:

```cpp
uint8_t a = 200;
uint8_t b = 100;

// ❌ Result is int (300), then truncated to uint8_t (44)
uint8_t c = a + b;

// ✅ Explicit: check overflow first, then cast
uint16_t sum = static_cast<uint16_t>(a) + static_cast<uint16_t>(b);
ARES_ASSERT(sum <= UINT8_MAX);
uint8_t c = static_cast<uint8_t>(sum);
```

---

## MISRA-3 — No Multiple Side Effects

**MISRA C:2023 Rule 13.1–13.6 — Side effects**
**Category:** Required

One operation per statement. Compound expressions with multiple
side effects create evaluation-order dependencies that are
unspecified or undefined.

### MISRA-3.1 — Forbidden patterns

```cpp
// ❌ Two side effects in one expression
a = b++ + func();

// ❌ Side effect in function argument
process(buffer, idx++);

// ❌ Side effect in conditional
if (--retries > 0 && sendPacket())

// ❌ Comma operator
for (i = 0, j = 0; i < n; i++, j++)
```

### MISRA-3.2 — Correct patterns

```cpp
// ✅ One side effect per statement
b++;
a = b + func();

// ✅ Separate side effect from call
uint8_t currentIdx = idx;
idx++;
process(buffer, currentIdx);

// ✅ Side effect before conditional
retries--;
if (retries > 0 && sendPacket())
```

**Exception:** The comma operator is allowed in `for` loop
increment expressions **only** when both variables are tightly
coupled loop counters and the logic is trivial.

---

## MISRA-4 — All Variables Initialized

**MISRA C:2023 Rule 9.1 — Initialisation**
**Category:** Required

No uninitialized variables. Ever. An uninitialized variable
contains whatever was in that memory location — a silent,
non-deterministic bug.

### MISRA-4.1 — Initialization rules

| Declaration Type   | Required Initialization                    |
|--------------------|--------------------------------------------|
| Local scalar       | At declaration: `uint32_t x = 0;`         |
| Local struct       | Brace init: `SensorData data = {};`       |
| Local array        | Zero init: `uint8_t buf[64] = {};`         |
| Static variable    | Compiler zero-inits, but be explicit       |
| Loop counter       | In the `for` init: `for (uint8_t i = 0; …)`|
| Pointer            | `nullptr` or valid address                 |

### MISRA-4.2 — Struct initialization

```cpp
// ❌ Partial initialization — remaining fields undefined in C++
TelemetryFrame frame;
frame.id = 0x01;

// ✅ Full brace initialization — all fields zeroed
TelemetryFrame frame = {};
frame.id = 0x01;

// ✅ Designated initializers (C++20, supported by ESP-IDF)
TelemetryFrame frame = {
    .id     = 0x01,
    .seqNum = 0,
    .len    = 0,
    .crc    = 0
};
```

---

## MISRA-5 — Single Exit Point

**MISRA C:2023 Rule 15.5 — Single exit point**
**Category:** Required

One `return` per function. Multiple returns create hidden
control-flow paths that complicate review and structural coverage
analysis.

**Exception:** Early return for argument validation guards
(PO10-5) is allowed at the **top** of the function only.

### MISRA-5.1 — Guard pattern

```cpp
bool process(const uint8_t* data, uint8_t len)
{
    // Guards — allowed early returns (top of function only)
    if (data == nullptr) { return false; }
    if (len == 0)        { return false; }

    // --- No more returns below this line ---

    bool result = false;

    // ... main logic ...
    result = validateAndProcess(data, len);

    return result;  // single functional return
}
```

### MISRA-5.2 — Result variable pattern

For complex functions, use a `result` variable instead of
multiple returns:

```cpp
FlightPhase getNextPhase(const SensorData& sensors)
{
    FlightPhase next = FlightPhase::ERROR;  // safe default

    if (detectLaunch(sensors))
    {
        next = FlightPhase::POWERED_ASCENT;
    }
    else if (detectApogee(sensors))
    {
        next = FlightPhase::DESCENT;
    }
    else
    {
        next = FlightPhase::COAST;
    }

    return next;
}
```

---

## MISRA-6 — No Hidden Behavior

**MISRA C:2023 Rule 1.3 — No undefined / unspecified behavior**
**MISRA C:2023 Rule 13.2 — Evaluation order**
**Category:** Required

Code must not depend on behavior that is undefined, unspecified,
or implementation-defined by the C/C++ standard.

### MISRA-6.1 — Undefined behavior (UB) — forbidden

| UB Category               | Example                         | Rule          |
|---------------------------|---------------------------------|---------------|
| Signed integer overflow   | `INT32_MAX + 1`                 | CERT-4        |
| Null pointer dereference  | `*ptr` when `ptr == nullptr`    | CERT-22       |
| Out-of-bounds access      | `buf[len]` when `len >= size`   | CERT-2        |
| Use after free            | `*ptr` after `delete ptr`       | CERT-22       |
| Uninitialized read        | `return x;` when x is uninit    | MISRA-4       |
| Double evaluation order   | `f(a++, a++)`                   | MISRA-3       |
| Strict aliasing violation | `*(float*)&intVal`              | CERT-11       |

### MISRA-6.2 — Implementation-defined behavior — documented

| Behavior                  | ESP32-S3 / Xtensa Value        | Action         |
|---------------------------|--------------------------------|----------------|
| `char` signedness         | Signed                         | Use `uint8_t`  |
| `int` size                | 32 bits                        | Use `int32_t`  |
| Bit-field layout          | Implementation-defined         | Avoid bit-fields; use explicit masks |
| Right-shift of signed     | Arithmetic (sign-extended)     | Use unsigned types for shifts |
| Struct padding            | Aligned to 4 bytes             | Use `__attribute__((packed))` only for wire formats |

### MISRA-6.3 — Forbidden constructs

```cpp
// ❌ Evaluation order undefined
func(a++, a++);

// ✅
a++;
func(a, a);

// ❌ Relying on signed overflow wrapping
int32_t x = INT32_MAX;
x++;  // UB

// ✅
if (x < INT32_MAX) { x++; }
```

---

## MISRA-7 — No Magic Numbers

**MISRA C:2023 Rule 7.1–7.4 — Constants and literals**
**Category:** Required

All numeric constants must be named. Magic numbers make code
unreadable and unmaintainable (see also PO10-8).

### MISRA-7.1 — Naming rules

```cpp
// ❌ What is 42? What is 0x1A?
if (temp > 42)
buffer[0] = 0x1A;

// ✅
constexpr float TEMP_WARNING_C = 42.0f;
constexpr uint8_t FRAME_SYNC_BYTE = 0x1A;

if (temp > TEMP_WARNING_C)
buffer[0] = FRAME_SYNC_BYTE;
```

### MISRA-7.2 — `constexpr` over `#define`

In C++ code, prefer `constexpr` — it is type-safe, scoped, and
visible to the debugger:

```cpp
// ❌ Preprocessor — no type, no scope, no debugger
#define MAX_ALTITUDE 100000

// ✅ constexpr — typed, scoped, debuggable
constexpr int32_t MAX_ALTITUDE_M = 100000;
```

**Exception:** `#define` is acceptable for:
- Include guards.
- Conditional compilation (`#ifdef ARES_DEBUG`).
- Macros that cannot be replaced by `constexpr` or `inline`
  functions (e.g., `ARES_ASSERT`, `IRAM_ATTR` wrappers).

### MISRA-7.3 — Allowed literal exceptions

| Literal  | Where Allowed                              |
|----------|--------------------------------------------|
| `0`      | Initialization, null-like values           |
| `1`      | Increment/decrement, unit step             |
| `nullptr`| Pointer initialization                     |
| `true`/`false` | Boolean expressions                 |

All other literals require a named constant.

### MISRA-7.4 — Suffix discipline for numeric literals

| Type        | Suffix  | Example          |
|-------------|---------|------------------|
| `unsigned`  | `U`     | `0x00FFU`        |
| `float`     | `f`     | `3.14f`          |
| `long`      | `L`     | Forbidden (MISRA-1) |
| `unsigned long` | `UL` | Forbidden (MISRA-1) |

Always use `U` suffix for unsigned constants in bitwise
operations to prevent sign-extension surprises.

---

## MISRA-8 — Strict Boolean Usage

**MISRA C:2023 Rule 14.4 — Boolean operands**
**MISRA C++:2023 Rule 8.2.4 — Boolean conversions**
**Category:** Required

Conditions must be explicit boolean expressions. Implicit
conversion to bool hides intent and can mask bugs.

### MISRA-8.1 — Comparison rules

| Context       | Forbidden         | Required                   |
|---------------|-------------------|----------------------------|
| Integer       | `if (x)`          | `if (x != 0)`              |
| Pointer       | `if (ptr)`        | `if (ptr != nullptr)`      |
| Bit flag      | `if (flags & 0x04)` | `if ((flags & 0x04U) != 0U)` |
| Function call | `if (read())`     | `if (read() != 0)`         |

### MISRA-8.2 — Boolean operators

Boolean operators (`&&`, `||`, `!`) must only be applied to
operands of boolean type:

```cpp
// ❌ Bitwise used where boolean intended
if (flagA & flagB)

// ✅ Boolean operators for boolean logic
if (flagA && flagB)

// ❌ Mixing bitwise and boolean
if ((status & MASK) && ready)

// ✅ Each operand is a boolean expression
if (((status & MASK) != 0U) && ready)
```

---

## MISRA-9 — No Dynamic Memory After Init

**MISRA C:2023 Dir 4.12 — Dynamic memory allocation**
**MISRA C:2023 Rule 21.3 — stdlib memory functions**
**Category:** Required

`malloc`, `free`, `new`, `delete` are **forbidden** after
`setup()` completes (see PO10-3, RTOS-7).

### MISRA-9.1 — Heap fragmentation risk

On the ESP32-S3 with 512 KB SRAM and no memory compaction, heap
fragmentation is a time bomb. After hours of allocate/free cycles,
a 256-byte allocation can fail even with 10 KB free.

### MISRA-9.2 — Allowed patterns

| Need                         | Forbidden               | Allowed Alternative         |
|------------------------------|-------------------------|-----------------------------|
| Variable-size buffer         | `new uint8_t[len]`      | `static uint8_t buf[MAX];` + `len` parameter |
| String building              | `std::string`, `String` | `char buf[MAX]; snprintf()`  |
| Collection of items          | `std::vector`           | `std::array<T, MAX>` + count |
| Temporary object             | `new MyObj()`           | Stack variable or static pool|
| RTOS queue buffer            | Dynamic queue            | `xQueueCreateStatic()`      |

### MISRA-9.3 — Init-time allocation

Allocation during `setup()` (before the scheduler starts) is
acceptable for:
- RTOS task stacks (via `xTaskCreateStatic`).
- Static queue/semaphore storage.
- One-time hardware driver initialization buffers.

After `vTaskStartScheduler()` — **no more allocations**.

---

## MISRA-10 — Controlled Pointers

**MISRA C:2023 Rule 11.1–11.9 — Pointer conversions**
**MISRA C:2023 Rule 18.1–18.4 — Pointer arithmetic**
**Category:** Required

Pointers are the #1 source of embedded bugs. Their use must be
minimal, explicit, and always validated.

### MISRA-10.1 — Pointer rules

| Rule                          | Rationale                              |
|-------------------------------|----------------------------------------|
| Validate before dereference   | Null dereference → hard fault          |
| No pointer arithmetic         | Off-by-one → buffer overflow           |
| No void* except RTOS params   | Type safety lost                       |
| No function pointers via casts| Wrong signature → stack corruption     |
| No pointer-to-int casts       | Size mismatch on other platforms       |

### MISRA-10.2 — Validation pattern

```cpp
// ❌ No check
data->field = value;

// ✅ Always check
if (data == nullptr)
{
    ARES_ASSERT(false);
    return false;
}
data->field = value;
```

### MISRA-10.3 — Array access over pointer arithmetic

```cpp
// ❌ Pointer arithmetic — error-prone
uint8_t* p = buffer + offset;
*p = value;

// ✅ Array indexing — bounds are visible
ARES_ASSERT(offset < BUFFER_SIZE);
buffer[offset] = value;
```

### MISRA-10.4 — Pointer casting restrictions

| Cast                      | Allowed? | Condition                         |
|---------------------------|----------|-----------------------------------|
| `T*` → `const T*`        | Yes      | Always safe (adding const)        |
| `const T*` → `T*`        | No       | Violates const correctness        |
| `void*` → `T*`           | Limited  | Only for RTOS task parameters     |
| `T*` → `U*` (unrelated)  | No       | Strict aliasing violation         |
| `T*` → `uintptr_t`       | No       | Non-portable                      |

---

## MISRA-11 — Switch Completeness

**MISRA C:2023 Rule 16.1–16.7 — Switch statements**
**Category:** Required

All `switch` statements must be complete, deterministic, and
free of fall-through.

### MISRA-11.1 — Completeness rules

- Every `switch` on an enum must handle **all** enumerators
  explicitly **and** include a `default` case.
- The `default` case exists to catch future enum additions and
  corrupt values — it must always call `ARES_ASSERT(false)`.

### MISRA-11.2 — Fall-through

Fall-through between cases is **forbidden**. Every case must end
with `break`, `return`, or `ARES_ASSERT(false)`.

```cpp
// ❌ Fall-through — silent bug
case State::IDLE:
    initSensors();
case State::ARMED:  // executes even when IDLE!
    checkPyros();
    break;

// ✅ Explicit break
case State::IDLE:
    initSensors();
    break;
case State::ARMED:
    checkPyros();
    break;
```

### MISRA-11.3 — Complete switch example

```cpp
switch (phase)
{
case FlightPhase::PAD_IDLE:       handlePadIdle();      break;
case FlightPhase::ARMED:          handleArmed();        break;
case FlightPhase::POWERED_ASCENT: handlePowered();      break;
case FlightPhase::COAST:          handleCoast();        break;
case FlightPhase::APOGEE:         handleApogee();       break;
case FlightPhase::DROGUE_DESCENT: handleDrogue();       break;
case FlightPhase::MAIN_DESCENT:   handleMain();         break;
case FlightPhase::LANDED:         handleLanded();       break;
case FlightPhase::ERROR:          handleError();        break;
default:
    ARES_ASSERT(false);  // corrupt or new enumerator
    break;
}
```

---

## MISRA-12 — Function Size and Complexity

**MISRA C:2023 Dir 4.1 — Run-time failures minimised**
**Category:** Required

Long, complex functions are harder to review, test, and verify.
They correlate with higher defect density.

### MISRA-12.1 — Limits

| Metric                  | Hard Limit | Recommended |
|-------------------------|------------|-------------|
| Lines per function      | 80 max     | ≤ 60        |
| Cyclomatic complexity   | 15 max     | ≤ 10        |
| Nesting depth           | 4 max      | ≤ 3         |
| Parameters              | 6 max      | ≤ 4         |

### MISRA-12.2 — Refactoring patterns

| Smell                          | Refactoring                         |
|--------------------------------|-------------------------------------|
| Long function                  | Extract helper with descriptive name|
| Deep nesting                   | Early return guards (MISRA-5.1)     |
| Many parameters                | Group into struct                   |
| Switch with logic per case     | Extract case handler functions      |
| Repeated code blocks           | Extract common helper               |

### MISRA-12.3 — Exception

ISR handlers and RTOS task entry functions may exceed the line
limit if the logic is linear and not easily decomposed. Document
with `// MISRA-12: ISR/task entry — linear flow`.

---

## MISRA-13 — Explicit Casts for Bitwise Operations

**MISRA C:2023 Rule 10.1 — Operands of essential type**
**MISRA C:2023 Rule 12.2 — Shift range**
**Category:** Required

Bit operations must use explicit unsigned types. Implicit widening
and sign extension cause silent, platform-dependent bugs.

### MISRA-13.1 — Shift rules

```cpp
// ❌ Signed literal 1 shifted — undefined if result overflows int
flags = 1 << 31;

// ✅ Unsigned literal, explicit type
uint32_t flags = 1U << 31;

// ❌ Shift amount ≥ type width — UB
uint8_t x = 1U << 8;

// ✅ Keep shift within type width
uint8_t x = static_cast<uint8_t>(1U << 7);
```

### MISRA-13.2 — Mask rules

```cpp
// ❌ Implicit int promotion, then narrowing
uint8_t masked = value & 0xFF;

// ✅ Explicit unsigned constant + explicit type
uint8_t masked = static_cast<uint8_t>(value & 0x00FFU);
```

### MISRA-13.3 — Bit-field prohibition

Bit-fields (`struct { uint8_t x : 3; }`) are **forbidden** for
data structures. Their layout (bit order, padding, alignment) is
implementation-defined and non-portable.

**Alternative:** Use explicit masks and shifts:

```cpp
// ❌ Bit-field — layout varies by compiler
struct Flags {
    uint8_t armed : 1;
    uint8_t ready : 1;
};

// ✅ Explicit masks — deterministic
constexpr uint8_t FLAG_ARMED = 0x01U;
constexpr uint8_t FLAG_READY = 0x02U;

bool isArmed = (flags & FLAG_ARMED) != 0U;
```

---

## MISRA-14 — Enum Safety

**MISRA C:2023 Rule 10.1 — Essential type model (enum)**
**MISRA C++:2023 Rule 10.2.1 — Scoped enumerations**
**Category:** Required

Enums are the primary mechanism for state representation in ARES.
They must be type-safe, bounded, and never silently converted.

### MISRA-14.1 — Scoped enums only

```cpp
// ❌ Unscoped enum — pollutes namespace, implicit int conversion
enum Color { RED, GREEN, BLUE };

// ✅ Scoped enum — no implicit conversion, no name collisions
enum class Color : uint8_t { RED, GREEN, BLUE };
```

### MISRA-14.2 — Underlying type

All enums must specify an explicit underlying type:

```cpp
enum class FlightPhase : uint8_t
{
    PAD_IDLE       = 0,
    ARMED          = 1,
    POWERED_ASCENT = 2,
    COAST          = 3,
    APOGEE         = 4,
    DROGUE_DESCENT = 5,
    MAIN_DESCENT   = 6,
    LANDED         = 7,
    ERROR          = 8
};
```

### MISRA-14.3 — Enum-to-integer conversion

- Enum → integer: Explicit `static_cast` only.
- Integer → enum: **Forbidden** without range validation.

```cpp
// ❌ Raw cast — could be out of range
FlightPhase p = static_cast<FlightPhase>(rawByte);

// ✅ Validate range, then cast
if (rawByte <= static_cast<uint8_t>(FlightPhase::ERROR))
{
    FlightPhase p = static_cast<FlightPhase>(rawByte);
}
else
{
    // reject invalid value
}
```

---

## MISRA-15 — Declaration and Scope

**MISRA C:2023 Rule 8.9 — Minimal scope**
**MISRA C:2023 Rule 8.13 — Const if possible**
**Category:** Required

Variables must be declared in the smallest possible scope and
as close to first use as possible.

### MISRA-15.1 — Scope rules

```cpp
// ❌ Declared far from use, broader scope than needed
uint32_t result = 0;
// ... 30 lines of unrelated code ...
result = compute();

// ✅ Declared at point of first use
// ... 30 lines of unrelated code ...
uint32_t result = compute();
```

### MISRA-15.2 — One declaration per line

```cpp
// ❌ Multiple declarations — confusing, especially with pointers
uint8_t *ptr, count, buffer[64];

// ✅ One per line — unambiguous
uint8_t* ptr = nullptr;
uint8_t  count = 0;
uint8_t  buffer[64] = {};
```

### MISRA-15.3 — Static for file-scope

Functions and variables that are only used within a single `.cpp`
file must be declared `static` (C) or in an anonymous namespace
(C++):

```cpp
// ✅ File-local helper — not visible outside this translation unit
namespace {
    uint16_t computeChecksum(const uint8_t* data, uint8_t len)
    {
        // ...
    }
}
```

---

## MISRA-16 — Expression Rules

**MISRA C:2023 Rule 12.1 — Precedence**
**MISRA C:2023 Rule 13.5 — Persistent side effects in &&/||**
**Category:** Required

Complex expressions must be unambiguous to human readers, not
just to the compiler.

### MISRA-16.1 — Explicit parentheses

Do not rely on operator precedence — use parentheses to make
intent explicit:

```cpp
// ❌ Does this mean (a & b) == c  or  a & (b == c)?
if (a & b == c)

// ✅ Unambiguous
if ((a & b) == c)
```

### MISRA-16.2 — Ternary operator

The ternary operator (`?:`) is allowed only for simple value
selection. It must not contain side effects or nested ternaries:

```cpp
// ❌ Nested ternary — unreadable
uint8_t x = (a > b) ? (c > d ? 1 : 2) : 3;

// ✅ Simple selection
uint8_t x = (altitude > THRESHOLD) ? HIGH_RATE : LOW_RATE;

// ❌ Side effect in ternary
uint8_t x = condition ? sendPacket() : 0;

// ✅ Use if/else for side effects
if (condition) { sendPacket(); }
```

### MISRA-16.3 — Assignment in expressions

Assignment inside conditionals, arguments, or return statements
is **forbidden**:

```cpp
// ❌ Assignment in condition
if (result = readSensor())

// ✅ Separate assignment and test
result = readSensor();
if (result != 0)
```

---

## MISRA-17 — Preprocessor Discipline

**MISRA C:2023 Rule 20.1–20.14 — Preprocessing directives**
**Category:** Required

The preprocessor is powerful but dangerous: it operates on text,
not on typed code. Its use must be minimal and disciplined.

### MISRA-17.1 — Allowed preprocessor uses

| Use                          | Allowed | Alternative if Forbidden     |
|------------------------------|---------|------------------------------|
| Include guards / `#pragma once` | Yes  | —                            |
| `#include`                   | Yes     | —                            |
| `#ifdef` / `#ifndef` feature guards | Yes | —                       |
| `ARES_ASSERT` macro          | Yes     | —                            |
| `constexpr` replacement      | No      | `constexpr` variable         |
| Function-like macros         | No      | `inline` function            |
| Token pasting (`##`)         | No      | Templates or overloads       |
| `#undef`                     | No      | Restructure includes         |

### MISRA-17.2 — Include guard format

```cpp
// ✅ Preferred — simple, portable
#pragma once

// ✅ Alternative — traditional guard
#ifndef ARES_FLIGHT_STATE_H
#define ARES_FLIGHT_STATE_H
// ...
#endif // ARES_FLIGHT_STATE_H
```

### MISRA-17.3 — Macro safety rules

When a macro is unavoidable:

```cpp
// ❌ Unsafe — no parentheses around arguments
#define SQUARE(x) x * x   // SQUARE(a+b) → a+b * a+b

// ✅ Parenthesized arguments and expression
#define SQUARE(x) ((x) * (x))

// Better: use constexpr/inline
constexpr int32_t square(int32_t x) { return x * x; }
```

---

## MISRA-18 — Floating-Point Restrictions

**MISRA C:2023 Rule 1.4 — Implementation-defined FP behaviour**
**Category:** Required

Floating-point arithmetic is inherently imprecise. On the ESP32-S3
(no `double` FPU), additional constraints apply.

### MISRA-18.1 — `float` only

`double` is **forbidden**. The ESP32-S3 has a single-precision FPU
only; `double` operations use software emulation, which is ~10×
slower and bloats the binary.

```cpp
// ❌ double — software emulated, wastes cycles
double altitude = 1500.0;

// ✅ float — hardware accelerated
float altitude = 1500.0f;
```

> **Warning:** Literal `1500.0` is `double` by default. Always use
> the `f` suffix: `1500.0f`.

### MISRA-18.2 — No equality comparison

```cpp
// ❌ Float equality — almost always fails
if (altitude == targetAlt)

// ✅ Epsilon comparison
constexpr float EPSILON = 0.1f;
if (fabsf(altitude - targetAlt) < EPSILON)
```

### MISRA-18.3 — NaN and Inf guards

See CERT-16 for detailed NaN/Inf handling. At minimum:

```cpp
if (!isfinite(sensorValue))
{
    // Mark stale, use last known good value
}
```

### MISRA-18.4 — Float-to-integer conversion

Always round explicitly before converting:

```cpp
// ❌ Implicit truncation — 3.9f → 3
int32_t alt = altitude;

// ✅ Explicit rounding + range check + cast
float rounded = roundf(altitude);
ARES_ASSERT(rounded >= static_cast<float>(INT32_MIN) &&
            rounded <= static_cast<float>(INT32_MAX));
int32_t alt = static_cast<int32_t>(rounded);
```

---

## MISRA-19 — Assembly and Compiler Extensions

**MISRA C:2023 Rule 1.2 — Language extensions**
**MISRA C:2023 Dir 4.3 — Assembly language**
**Category:** Required

Inline assembly and compiler extensions bypass type checking and
static analysis. Their use must be minimal and isolated.

### MISRA-19.1 — Inline assembly

Inline assembly is **forbidden** in application code. If required
for hardware access, it must be:
- Isolated in a dedicated driver file.
- Wrapped in a C++ function with documented pre/postconditions.
- Marked with `// MISRA-19: inline asm — <justification>`.

### MISRA-19.2 — Allowed compiler extensions

| Extension                  | Allowed | Context                        |
|----------------------------|---------|--------------------------------|
| `__attribute__((packed))`  | Yes     | Wire-format structs only       |
| `IRAM_ATTR`                | Yes     | ISR functions only (CERT-17)   |
| `__attribute__((aligned))` | Yes     | DMA buffers only (CERT-15)     |
| `__attribute__((weak))`    | No      | Creates hidden dependencies    |
| `__builtin_*` functions    | Limited | Only `__builtin_expect` (branch hints) |

### MISRA-19.3 — Portability annotations

When a compiler extension is used, document the portability
impact:

```cpp
// MISRA-19: packed struct for radio wire format.
// Non-portable: assumes little-endian, no padding.
// Review required if porting to different MCU.
struct __attribute__((packed)) RadioHeader
{
    uint8_t  sync;
    uint8_t  version;
    uint16_t length;
    uint16_t crc;
};
```

---

## MISRA-20 — Struct and Union Discipline

**MISRA C:2023 Rule 19.2 — Union usage**
**MISRA C++:2023 Rule 12.3.1 — Union restrictions**
**Category:** Required

Structs and unions are fundamental data types for embedded
protocol handling. Their usage must be controlled to prevent
type-punning bugs and layout surprises.

### MISRA-20.1 — Struct rules

- All fields initialized (MISRA-4.2).
- Use `__attribute__((packed))` only for wire-format structures
  (MISRA-19.2) — never for internal data.
- Document byte order assumptions for multi-byte fields in
  wire-format structs.
- Prefer `constexpr` size constants over `sizeof()` on packed
  structs (padding can vary).

### MISRA-20.2 — Union restrictions

Unions are **forbidden** for type-punning (reinterpreting bits of
one type as another). Use `memcpy` instead:

```cpp
// ❌ Type-punning via union — UB in C++
union {
    float    f;
    uint32_t u;
} converter;
converter.f = altitude;
uint32_t bits = converter.u;

// ✅ memcpy — well-defined behavior
float altitude = 1500.0f;
uint32_t bits = 0;
memcpy(&bits, &altitude, sizeof(bits));
```

**Exception:** `union` is allowed for:
- Variant-type message payloads **if** the active member is
  tracked by a discriminator field and only the active member
  is accessed.
- Hardware register overlays documented as
  `// MISRA-20: HW register union — <peripheral>`.

### MISRA-20.3 — Struct size awareness

On a memory-constrained system, struct sizes matter:

```cpp
// ❌ Wasteful padding (12 bytes due to alignment)
struct Bad {
    uint8_t  type;     // 1 + 3 padding
    uint32_t value;    // 4
    uint8_t  flags;    // 1 + 3 padding
};

// ✅ Ordered by decreasing alignment (8 bytes, no padding)
struct Good {
    uint32_t value;    // 4
    uint8_t  type;     // 1
    uint8_t  flags;    // 1 + 2 padding
};
```

Order struct fields by decreasing alignment to minimize padding.

---

## MISRA-21 — Error Handling Model

**MISRA C:2023 Dir 4.7 — Check return values**
**Category:** Required

Every function that can fail must communicate failure to its
caller. Silent failures are the root cause of cascading bugs.

### MISRA-21.1 — Return value rules

- **Every** function return value must be checked.
- Functions that cannot fail return `void`.
- Functions that can fail return either:
  - `bool` (true = success), or
  - An error enum (e.g., `AresError`).

```cpp
// ❌ Return value ignored — sensor failure undetected
readImu();

// ✅ Return value checked
if (!readImu(&imuData))
{
    markSensorStale(SensorId::IMU);
}
```

### MISRA-21.2 — Error enum pattern

```cpp
enum class AresError : uint8_t
{
    OK             = 0,
    TIMEOUT        = 1,
    INVALID_PARAM  = 2,
    HW_FAILURE     = 3,
    BUFFER_FULL    = 4,
    CRC_MISMATCH   = 5
};
```

### MISRA-21.3 — C++ exceptions

C++ exceptions (`throw` / `try` / `catch`) are **forbidden**.
They require heap allocation, produce non-deterministic timing,
and bloat the binary. The compiler flag `-fno-exceptions` enforces
this (see DO-12.3).

### MISRA-21.4 — RTOS API error checking

FreeRTOS API calls return `pdTRUE`/`pdFALSE` or `errQUEUE_FULL`.
These must always be checked:

```cpp
// ❌ Queue send without check
xQueueSend(telemetryQueue, &frame, pdMS_TO_TICKS(10));

// ✅ Check result
if (xQueueSend(telemetryQueue, &frame, pdMS_TO_TICKS(10)) != pdTRUE)
{
    droppedFrames++;
}
```

---

## MISRA-22 — Include and Dependency Order

**MISRA C:2023 Rule 20.1 — #include after code**
**Category:** Advisory

Header inclusion order affects build reproducibility, compile
times, and hidden dependencies.

### MISRA-22.1 — Include order

```cpp
// 1. Own module header (proves it is self-contained)
#include "flight_state.h"

// 2. ARES project headers
#include "config.h"
#include "ares_assert.h"

// 3. Third-party / library headers
#include <RadioLib.h>
#include <TinyGPS++.h>

// 4. ESP-IDF / FreeRTOS system headers
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// 5. C/C++ standard library headers
#include <cstdint>
#include <cstring>
```

### MISRA-22.2 — Self-containment

Every header must compile independently. This is verified by
including the module's own header **first** in its `.cpp` file.
If it fails to compile alone, it has missing includes.

### MISRA-22.3 — Forbidden practices

- No header includes another header that it does not directly
  use (transitive dependency).
- No circular includes.
- No `using namespace` in headers — pollutes every file that
  includes it.

---

## Anti-Patterns (Forbidden)

| Anti-Pattern                                     | Rule      |
|--------------------------------------------------|-----------|
| `int`, `long`, `short` types                     | MISRA-1   |
| `double` type                                    | MISRA-1, MISRA-18 |
| Implicit narrowing or sign conversions           | MISRA-2   |
| `a = b++ + c++`                                  | MISRA-3   |
| Uninitialized variables                          | MISRA-4   |
| Multiple returns (non-guard)                     | MISRA-5   |
| Relying on evaluation order or UB                | MISRA-6   |
| Literal numbers in logic                         | MISRA-7   |
| `#define` for typed constants                    | MISRA-7   |
| `if (ptr)` instead of `if (ptr != nullptr)`      | MISRA-8   |
| `new` / `malloc` at runtime                      | MISRA-9   |
| `std::string`, `std::vector` at runtime          | MISRA-9   |
| Unchecked pointer dereference                    | MISRA-10  |
| Pointer arithmetic in application code           | MISRA-10  |
| `switch` without `default`                       | MISRA-11  |
| Fall-through in switch cases                     | MISRA-11  |
| 200-line functions or complexity > 15            | MISRA-12  |
| Uncast bitwise shifts                            | MISRA-13  |
| Bit-fields for data structures                   | MISRA-13  |
| Unscoped `enum` (plain `enum`)                   | MISRA-14  |
| Enum without explicit underlying type            | MISRA-14  |
| Raw integer → enum cast without validation       | MISRA-14  |
| Multiple declarations per line                   | MISRA-15  |
| Relying on operator precedence                   | MISRA-16  |
| Nested ternary operators                         | MISRA-16  |
| Assignment inside conditions                     | MISRA-16  |
| Function-like macros                             | MISRA-17  |
| `#define` for constants                          | MISRA-17  |
| `double` literal without `f` suffix              | MISRA-18  |
| `if (altitude == target)` (float equality)       | MISRA-18  |
| Inline assembly in application code              | MISRA-19  |
| Type-punning via union                           | MISRA-20  |
| Struct with wasteful padding                     | MISRA-20  |
| Ignored return value                             | MISRA-21  |
| C++ exceptions (`throw`/`catch`)                 | MISRA-21  |
| `using namespace` in header file                 | MISRA-22  |
| Circular header includes                         | MISRA-22  |

---

## Cross-References

| MISRA Rule | MISRA C:2023 / C++:2023        | Related ARES Rules                          |
|------------|--------------------------------|---------------------------------------------|
| MISRA-1    | Dir 4.6                       | CERT-16 (float only), MISRA-18              |
| MISRA-2    | Rule 10.1–10.8                | CERT-4 (integer overflow)                   |
| MISRA-3    | Rule 13.1–13.6                | CERT-11 (UB avoidance)                      |
| MISRA-4    | Rule 9.1                      | PO10-5 (defensive coding)                   |
| MISRA-5    | Rule 15.5                     | PO10-1 (simple control flow)                |
| MISRA-6    | Rule 1.3, 13.2                | CERT-11 (UB), CERT-4 (signed overflow)      |
| MISRA-7    | Rule 7.1–7.4                  | PO10-8 (named constants), RTOS-5            |
| MISRA-8    | Rule 14.4, C++ 8.2.4          | CERT-19 (const correctness)                 |
| MISRA-9    | Dir 4.12, Rule 21.3           | PO10-3, RTOS-7 (no dynamic alloc)           |
| MISRA-10   | Rule 11.1–11.9, 18.1–18.4    | PO10-5 (null checks), CERT-22 (pointer safety) |
| MISRA-11   | Rule 16.1–16.7                | CERT-6 (validate enums), CERT-20 (exhaustiveness) |
| MISRA-12   | Dir 4.1                       | PO10-4 (short functions)                    |
| MISRA-13   | Rule 10.1, 12.2               | CERT-4 (overflow awareness)                 |
| MISRA-14   | Rule 10.1, C++ 10.2.1         | CERT-6 (enum validation)                    |
| MISRA-15   | Rule 8.9, 8.13                | CERT-19 (const correctness)                 |
| MISRA-16   | Rule 12.1, 13.5               | MISRA-3 (side effects)                      |
| MISRA-17   | Rule 20.1–20.14               | PO10-8 (constexpr over define)              |
| MISRA-18   | Rule 1.4                      | CERT-16 (float discipline)                  |
| MISRA-19   | Rule 1.2, Dir 4.3             | CERT-15 (volatile/HW access), CERT-17 (ISR)|
| MISRA-20   | Rule 19.2, C++ 12.3.1         | CERT-11 (strict aliasing)                   |
| MISRA-21   | Dir 4.7                       | CERT-8 (fail safe), RTOS-8 (timeout check)  |
| MISRA-22   | Rule 20.1                     | DO-3.2 (header discipline)                  |

---

## References

- MISRA C:2023 — *Guidelines for the Use of the C Language in
  Critical Systems*, Fourth Edition, MISRA, 2023.
- MISRA C++:2023 — *Guidelines for the Use of C++17 in Critical
  Systems*, MISRA, 2023.
- MISRA Compliance:2020 — *Achieving Compliance with MISRA
  Coding Guidelines*, MISRA, 2020.
- ESP-IDF Coding Style Guide:
  https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/contribute/style-guide.html
- SEI CERT C Coding Standard, 2016 Edition (companion standard).

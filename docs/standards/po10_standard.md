# ARES PO10 Standard (NASA JPL Power of 10)

**Amateur Rocket Embedded System**
Deterministic, verifiable embedded C++ rules.
Derived from *The Power of 10: Rules for Developing Safety-Critical
Code* (Gerard J. Holzmann, NASA/JPL, IEEE Computer, June 2006).
Companion to the [ARES Coding Standard](ares_coding_standard.md).

---

## Context

The NASA JPL "Power of 10" rules were created at the Laboratory for
Reliable Software to enable static verification of safety-critical C
code. They complement MISRA C and form the foundation of the JPL
Institutional Coding Standard (JPL D-60411).

ARES adopts all 10 rules. Where amateur rocketry constraints require
a deviation from the original rule, the deviation is documented and
justified per rule.

### Design rationale

The rules share a common philosophy: **if a tool cannot verify a
property, the code is wrong.** This shifts the burden from the
developer ("I believe this is safe") to the toolchain ("this is
provably safe").

| Principle               | Rules That Enforce It               |
|-------------------------|-------------------------------------|
| Verifiable control flow | PO10-1, PO10-2                      |
| Bounded resource usage  | PO10-2, PO10-3, PO10-4             |
| Defensive programming   | PO10-5, PO10-7                      |
| Minimal complexity      | PO10-4, PO10-6, PO10-8, PO10-9     |
| Tool-checkable code     | PO10-2, PO10-5, PO10-10            |

---

## PO10-1 — Simple Control Flow

**Original rule (Holzmann #1):** *Restrict all code to very simple
control flow constructs — do not use `goto` statements, `setjmp` or
`longjmp` constructs, or direct or indirect recursion.*

### PO10-1.1 — Forbidden constructs

| Construct             | Status      | Reason                             |
|-----------------------|-------------|------------------------------------|
| `goto`                | Forbidden   | Unstructured flow, unverifiable    |
| `setjmp` / `longjmp`  | Forbidden   | Non-local jumps, stack corruption  |
| Direct recursion      | Forbidden   | Unbounded stack growth             |
| Indirect recursion    | Forbidden   | Even harder to detect/bound        |
| C++ exceptions        | Forbidden   | Non-local flow, heap use (MISRA-21)|

### PO10-1.2 — Control flow style

- Simple `if`/`else`, `for`, `while`, `switch` only.
- No deeply nested logic (max 4 levels, see MISRA-12).
- Each function has a clear, linear flow that a reviewer can follow
  top-to-bottom.

> **ARES deviation from original:** The original rule forbids **all**
> recursion. ARES also forbids recursion in production code. However,
> if recursion is absolutely unavoidable in a utility function (e.g.,
> a tree traversal during init), it must:
> - Include an explicit `maxDepth` parameter with `ARES_ASSERT`.
> - Be documented with `// PO10-1: bounded recursion — <justification>`.
> - Never be called from flight-critical paths.
> - Be verified to terminate via code review.

```cpp
// ❌ Forbidden — unbounded recursion
void traverse(Node* node)
{
    if (node == nullptr) { return; }
    process(node);
    traverse(node->left);
    traverse(node->right);
}

// ✅ If unavoidable — bounded, documented, init-only
// PO10-1: bounded recursion — config tree parse at boot
void traverse(Node* node, uint8_t maxDepth)
{
    if (node == nullptr || maxDepth == 0) { return; }
    process(node);
    traverse(node->left,  maxDepth - 1);
    traverse(node->right, maxDepth - 1);
}
```

---

## PO10-2 — Loops With Fixed Upper Bound

**Original rule (Holzmann #2):** *Give all loops a fixed upper bound.
It must be trivially possible for a checking tool to prove statically
that the loop cannot exceed a preset upper bound on the number of
iterations. If a tool cannot prove the loop bound statically, the
rule is considered violated.*

### PO10-2.1 — Loop bound rules

Every loop must have an upper bound that is:
- Known at compile time, **or**
- Derived from a named `constexpr` constant, **or**
- Enforced by a decrementing counter.

A static analysis tool must be able to determine the maximum iteration
count by inspecting the loop alone, without inter-procedural analysis.

### PO10-2.2 — Correct loop patterns

```cpp
// ✅ Compile-time bound
for (uint8_t i = 0; i < ares::MAX_SENSORS; i++)
{
    readSensor(i);
}

// ✅ Named constant bound
constexpr uint16_t MAX_GPS_CHARS = 512;
uint16_t remaining = MAX_GPS_CHARS;
while (serial.available() && remaining > 0)
{
    remaining--;
    processByte(serial.read());
}

// ✅ Fixed retry count
constexpr uint8_t MAX_RETRIES = 3;
for (uint8_t attempt = 0; attempt < MAX_RETRIES; attempt++)
{
    if (radioSend(frame)) { break; }
}
```

### PO10-2.3 — Forbidden loop patterns

```cpp
// ❌ No bound — depends on external state
while (serial.available())
{
    processByte(serial.read());
}

// ❌ Bound depends on function call — not statically provable
for (int i = 0; i < getCount(); i++) { ... }

// ❌ Iterator loop with no obvious bound
while (it != end) { ... }
```

### PO10-2.4 — RTOS task loops

`while (true)` is permitted **only** in RTOS task entry functions.
It must be documented and must contain a blocking RTOS call:

```cpp
void sensorTask(void* param)
{
    TickType_t lastWake = xTaskGetTickCount();

    while (true)  // RTOS: task loop — blocks on vTaskDelayUntil
    {
        readAndValidateSensors();
        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(10));
    }
}
```

Every RTOS task loop must contain at least one blocking call
(`vTaskDelay`, `vTaskDelayUntil`, `xQueueReceive`, `xSemaphoreTake`)
to yield the CPU. A task loop without a blocking call will starve
lower-priority tasks and trigger the watchdog.

---

## PO10-3 — No Dynamic Memory After Init

**Original rule (Holzmann #3):** *Do not use dynamic memory allocation
after initialization.*

### PO10-3.1 — Phase separation

| Phase              | Dynamic Allocation | Examples                              |
|--------------------|--------------------|---------------------------------------|
| `setup()` / init   | Allowed            | `xTaskCreateStatic`, static pool init |
| After scheduler    | **Forbidden**      | No `malloc`, `new`, `calloc`, `realloc`|

### PO10-3.2 — Forbidden constructs

| Construct                  | Reason                              | Alternative             |
|----------------------------|-------------------------------------|-------------------------|
| `malloc` / `free`          | Heap fragmentation                  | Static buffer           |
| `new` / `delete`           | Heap fragmentation                  | Stack or static object  |
| `calloc` / `realloc`       | Heap fragmentation                  | Fixed-size buffer       |
| `std::string`              | Heap-backed, unbounded              | `char buf[MAX]`         |
| `std::vector`              | Heap-backed, unbounded              | `std::array<T, MAX>`    |
| `std::map` / `std::set`    | Heap-backed, tree nodes             | Sorted `std::array`     |
| Arduino `String`           | Heap-backed, fragments aggressively | `char buf[MAX]`         |
| `pvPortMalloc` at runtime  | FreeRTOS heap, same fragmentation   | `xQueueCreateStatic`    |

### PO10-3.3 — Static allocation patterns

```cpp
// ✅ Fixed-size telemetry buffer
static uint8_t txBuffer[ares::MAX_FRAME_LEN] = {};

// ✅ Static RTOS queue
static StaticQueue_t queueStorage;
static uint8_t queueBuffer[QUEUE_LEN * sizeof(SensorData)];
QueueHandle_t sensorQueue = xQueueCreateStatic(
    QUEUE_LEN, sizeof(SensorData), queueBuffer, &queueStorage);

// ✅ Static pool for reusable objects
static TelemetryFrame framePool[ares::MAX_FRAMES] = {};
static uint8_t poolIndex = 0;
```

### PO10-3.4 — Verification

The linker map file (`build/firmware.map`) should be inspected at
release time to verify that no `malloc`/`free` symbols are linked
into the binary (except ESP-IDF internals that run during init).

---

## PO10-4 — Short Functions

**Original rule (Holzmann #4):** *No function should be longer than
what can be printed on a single sheet of paper in a standard format
with one line per statement and one line per declaration. Typically,
this means no more than about 60 lines of code per function.*

### PO10-4.1 — Limits

| Metric                  | Hard Limit | Recommended |
|-------------------------|------------|-------------|
| Lines per function      | 80 max     | ≤ 60        |
| Cyclomatic complexity   | 15 max     | ≤ 10        |
| Nesting depth           | 4 max      | ≤ 3         |
| Parameters              | 6 max      | ≤ 4         |

Line count excludes:
- Opening/closing braces on their own line.
- Blank lines and comments.
- The function signature.

### PO10-4.2 — Why short functions matter

- **Verifiability:** Shorter functions can be fully reviewed and
  tested. Coverage of a 60-line function is tractable; coverage of
  a 300-line function is not.
- **Determinism:** Shorter functions have predictable stack usage
  and execution time.
- **Maintainability:** Each function fits on one screen — no
  scrolling needed for full context.

### PO10-4.3 — Refactoring strategy

When a function exceeds the limit:

1. Extract logically distinct blocks into named helper functions.
2. Name each helper after **what** it does, not **how**.
3. Keep helpers `static` / in anonymous namespace if used only
   in one file (MISRA-15.3).
4. Do **not** create trivial single-line wrappers just to reduce
   line count — the goal is understandability, not metrics gaming.

```cpp
// ❌ 120-line monolith
void processTelemetry() { /* 120 lines of mixed logic */ }

// ✅ Decomposed into meaningful steps
void processTelemetry()
{
    SensorData data = collectSensorData();
    FlightPhase phase = updateFlightState(data);
    TelemetryFrame frame = encodeTelemetry(data, phase);
    enqueueForTransmission(frame);
}
```

### PO10-4.4 — Exception

ISR handlers and RTOS task entry functions may exceed 60 lines if
the logic is linear and cannot be meaningfully decomposed. Document
with `// PO10-4: ISR/task entry — linear flow, <N> lines`.

---

## PO10-5 — Assertions

**Original rule (Holzmann #5):** *The code's assertion density should
average to minimally two assertions per function. Assertions must be
used to check for anomalous conditions that should never happen in
real-life executions. Assertions must be side-effect free and should
be defined as Boolean tests. When an assertion fails, an explicit
recovery action must be taken. Any assertion for which a static
checking tool can prove that it can never fail or never hold violates
this rule.*

### PO10-5.1 — Assertion density

- Minimum **2 assertions** per non-trivial function.
- Trivial functions (< 5 lines, single operation) may have 1 or 0
  assertions if the inputs are type-constrained.

### PO10-5.2 — Assertion properties

| Property           | Requirement                                      |
|--------------------|--------------------------------------------------|
| Side-effect free   | `ARES_ASSERT(x)` must not modify any state       |
| Boolean test       | Must evaluate to `true` or `false`               |
| Not tautological   | Must be theoretically falsifiable                 |
| Not impossible     | Must be theoretically satisfiable                 |
| Recovery action    | On failure: log, set safe state, return error     |

An assertion that a static checker can prove **always true** or
**always false** is a bug — it means the assertion is tautological
(useless) or the code path is dead.

```cpp
// ❌ Tautological — always true for uint8_t
ARES_ASSERT(len >= 0);  // uint8_t is always ≥ 0

// ❌ Side effect inside assertion
ARES_ASSERT(++counter < MAX);

// ✅ Meaningful, side-effect free
ARES_ASSERT(len <= MAX_PAYLOAD_LEN);
ARES_ASSERT(buffer != nullptr);
```

### PO10-5.3 — What to assert

| Category                | Example                                        |
|-------------------------|-------------------------------------------------|
| Pointer validity        | `ARES_ASSERT(data != nullptr);`                 |
| Array bounds            | `ARES_ASSERT(idx < ARRAY_SIZE);`                |
| Enum range              | `ARES_ASSERT(phase <= FlightPhase::ERROR);`     |
| Pre-conditions          | `ARES_ASSERT(len > 0 && len <= MAX);`           |
| Post-conditions         | `ARES_ASSERT(result.isValid());`                |
| Invariants              | `ARES_ASSERT(freeSlots + usedSlots == TOTAL);`  |
| Physical plausibility   | `ARES_ASSERT(altitude_m < 100000);`             |
| RTOS API success        | `ARES_ASSERT(xQueueSend(...) == pdTRUE);`       |

### PO10-5.4 — ARES_ASSERT implementation

`ARES_ASSERT` is defined in `ares_assert.h`:

- **Debug builds:** Logs file, line, expression; then halts or
  resets (depending on configuration).
- **Release builds (`-DARES_NDEBUG`):** Assertions can be compiled
  out for code size, but this is **not recommended** for flight
  firmware. Flight firmware should keep assertions active and
  trigger a safe-state transition on failure.

### PO10-5.5 — Assert vs. validate

| Situation                | Use                  | Reason                          |
|--------------------------|----------------------|---------------------------------|
| Internal invariant       | `ARES_ASSERT`        | "This can never happen"         |
| External/untrusted data  | `if` + error return  | "This might happen" (CERT-1)    |
| Configuration constant   | `static_assert`      | Compile-time guarantee          |
| RTOS contract            | `configASSERT`       | FreeRTOS kernel-level check     |

> Never use `ARES_ASSERT` to validate external data. External data
> **will** be invalid — it is not an anomaly, it is expected.

---

## PO10-6 — Minimal Scope

**Original rule (Holzmann #6):** *Declare all data objects at the
smallest possible level of scope.*

### PO10-6.1 — Scope hierarchy

From most preferred (narrowest) to least preferred (widest):

| Scope                | Example                        | When to Use            |
|----------------------|--------------------------------|------------------------|
| Block scope          | `{ uint8_t temp = 0; ... }`    | Default for locals     |
| Function scope       | Local variable                 | Used across blocks     |
| File scope (`static`)| `static uint32_t counter = 0;` | Shared within one file |
| Namespace scope      | `namespace ares { ... }`       | Shared across files    |
| Global scope         | **Forbidden**                  | Never                  |

### PO10-6.2 — Declare at point of first use

```cpp
// ❌ Declared far from use — C89 style
uint32_t result = 0;
uint8_t  flags  = 0;
// ... 40 lines later ...
result = computeAltitude();
flags  = getStatusFlags();

// ✅ Declared at point of use
// ... 40 lines of other logic ...
uint32_t result = computeAltitude();
uint8_t  flags  = getStatusFlags();
```

### PO10-6.3 — No mutable globals

Mutable global variables are **forbidden**. They create hidden
coupling between modules, make testing difficult, and introduce
race conditions in multi-task systems.

| Need                        | Forbidden                  | Allowed Alternative             |
|-----------------------------|----------------------------|---------------------------------|
| Shared sensor data          | `SensorData g_sensors;`   | RTOS queue (RTOS-4)             |
| Configuration               | `Config g_config;`        | `const` namespace variable      |
| Module-internal state       | `int count;`              | `static` file-scope variable    |
| Cross-module communication  | Global flag                | RTOS event group / queue        |

### PO10-6.4 — `const` and `constexpr` by default

- Prefer `const` for variables that don't change after init.
- Prefer `constexpr` for compile-time constants.
- Treat mutability as the exception, not the default (CERT-19).

```cpp
// ✅ const by default
const float altitude = computeAltitude();
constexpr uint8_t MAX_RETRIES = 3;
```

---

## PO10-7 — Check All Return Values

**Original rule (Holzmann #7):** *Each calling function must check
the return value of non-void functions, and each called function
must check the validity of all parameters provided by the caller.*

### PO10-7.1 — Caller responsibility

Every non-void function call must have its return value checked.
A return value that is deliberately ignored must be cast to `void`
with a comment:

```cpp
// ❌ Return value silently ignored
driver.begin();

// ✅ Return value checked
bool ok = driver.begin();
ARES_ASSERT(ok);

// ✅ Deliberately ignored (rare, justified)
(void)Serial.printf("debug: %d\n", value);
// PO10-7: printf return not safety-relevant
```

### PO10-7.2 — Callee responsibility

Every function must validate its parameters before use:

```cpp
bool writeFrame(const uint8_t* data, uint8_t len)
{
    // Parameter validation — callee's duty
    if (data == nullptr)     { return false; }
    if (len == 0)            { return false; }
    if (len > MAX_FRAME_LEN) { return false; }

    // Postcondition: data is valid, len is in range
    // ... safe to proceed ...
    return true;
}
```

### PO10-7.3 — Error propagation pattern

When a function calls another function that can fail, the error
must be propagated — not swallowed:

```cpp
// ❌ Error swallowed
bool initSubsystems()
{
    initImu();     // might fail — ignored
    initBaro();    // might fail — ignored
    return true;   // always reports success
}

// ✅ Errors propagated
bool initSubsystems()
{
    if (!initImu())  { return false; }
    if (!initBaro()) { return false; }
    return true;
}
```

### PO10-7.4 — RTOS API return checks

FreeRTOS APIs return status codes that must be checked:

| API Call                    | Check                           | On Failure             |
|-----------------------------|---------------------------------|------------------------|
| `xQueueSend`                | `== pdTRUE`                     | Increment drop counter |
| `xQueueReceive`             | `== pdTRUE`                     | Handle timeout         |
| `xSemaphoreTake`            | `== pdTRUE`                     | Handle contention      |
| `xTaskCreate`               | `== pdPASS`                     | Assert / abort         |
| `xTimerStart`               | `== pdPASS`                     | Assert / log           |

---

## PO10-8 — Limited Preprocessor

**Original rule (Holzmann #8):** *The use of the preprocessor must
be limited to the inclusion of header files and simple macro
definitions. Token pasting, variable argument lists (ellipses), and
recursive macro calls are not allowed. All macros must expand into
complete syntactic units. The use of conditional compilation
directives must be kept to a minimum.*

### PO10-8.1 — Allowed preprocessor uses

| Use                                | Allowed | Example                        |
|------------------------------------|---------|--------------------------------|
| `#include`                          | Yes     | `#include "config.h"`          |
| `#pragma once` / include guards     | Yes     | `#pragma once`                 |
| `#define` for simple constants      | Limited | Prefer `constexpr` (MISRA-7.2) |
| `#ifdef` feature guards             | Limited | `#ifdef ARES_DEBUG`            |
| `ARES_ASSERT` macro                 | Yes     | Debugging/safety macro         |
| Platform detection (`#ifdef ESP32`) | Limited | Hardware abstraction           |

### PO10-8.2 — Forbidden preprocessor uses

| Use                                    | Reason                             |
|----------------------------------------|------------------------------------|
| Token pasting (`##`)                   | Generates unreadable, unverifiable code |
| Variadic macros (`__VA_ARGS__`, `...`) | Complex, hard to debug and verify   |
| Recursive macro expansion              | Unverifiable expansion depth        |
| `#define` for typed constants          | No type safety; use `constexpr`     |
| Function-like macros                   | No type checking; use `inline`      |
| `#undef`                               | Creates fragile include-order dependencies |
| Macros that generate incomplete syntax | Unverifiable partial statements     |

### PO10-8.3 — Conditional compilation discipline

Conditional compilation (`#ifdef`) must be kept to a minimum
because each `#ifdef` doubles the number of code paths that must
be tested.

Allowed guards:
- `ARES_DEBUG` — debug instrumentation.
- `ARES_NDEBUG` — assertion control.
- Hardware variants: `ARES_HW_V1`, `ARES_HW_V2`.
- Platform: `ESP32`, `NATIVE` (for desktop test builds).

```cpp
// ❌ Too many #ifdefs — combinatorial explosion
#ifdef FEATURE_A
  #ifdef FEATURE_B
    #ifdef PLATFORM_X
      // ... 3 guards deep = 8 code paths
    #endif
  #endif
#endif

// ✅ One level, clear purpose
#ifdef ARES_DEBUG
    logDebug("altitude: %.1f m", altitude);
#endif
```

### PO10-8.4 — Macro completeness

Every macro must expand into a **complete syntactic unit** — a full
expression or a full statement:

```cpp
// ❌ Incomplete — generates dangling else
#define CHECK(x) if (!(x)) return false

// ✅ Complete syntactic unit
#define CHECK(x) do { if (!(x)) { return false; } } while(0)

// Better: inline function instead of macro
inline bool check(bool condition) { return condition; }
```

---

## PO10-9 — Restricted Pointer Use

**Original rule (Holzmann #9):** *The use of pointers must be
restricted. Specifically, no more than one level of dereference
should be used. Pointer dereference operations may not be hidden in
macro definitions or inside typedef declarations. Function pointers
are not permitted.*

### PO10-9.1 — Indirection limit

Maximum **one** level of pointer indirection:

```cpp
// ✅ One level — allowed
uint8_t* buffer = getBuffer();

// ❌ Two levels — forbidden
uint8_t** bufferArray = getBuffers();

// ❌ Three levels — absolutely forbidden
uint8_t*** volume = getVolume();
```

**Exception:** `const char*[]` (array of string literals) is allowed
for lookup tables:

```cpp
// ✅ Allowed exception — string literal table
constexpr const char* PHASE_NAMES[] = {
    "PAD_IDLE", "ARMED", "POWERED_ASCENT", "COAST",
    "APOGEE", "DROGUE", "MAIN", "LANDED", "ERROR"
};
```

### PO10-9.2 — No hidden dereferences

Pointer dereferences must not be hidden inside macros or typedefs:

```cpp
// ❌ Hidden pointer in typedef
typedef uint8_t* BytePtr;
BytePtr data;  // reader doesn't see the pointer

// ✅ Pointer is visible
uint8_t* data = nullptr;

// ❌ Hidden dereference in macro
#define GET_VALUE(p) (*(p))

// ✅ Explicit dereference — visible to reviewer
uint8_t value = *ptr;
// Or better: array indexing
uint8_t value = buffer[0];
```

### PO10-9.3 — Function pointers

> **ARES deviation from original:** The original Holzmann rule
> forbids function pointers entirely. ARES relaxes this because
> FreeRTOS task creation, ISR registration, and driver callback
> interfaces require function pointers.

**Allowed uses:**
- RTOS task entry: `xTaskCreate(sensorTask, ...)`.
- ISR registration: `attachInterrupt(pin, isr, RISING)`.
- Driver callback interfaces defined in `.h` files with
  `typedef` / `using`.

**Forbidden uses:**
- General-purpose dispatch tables.
- Function pointer arithmetic.
- Storing function pointers in arrays (except const driver
  vtables with ≤ 8 entries).
- Casting between function pointer types.

```cpp
// ✅ Allowed — RTOS task creation
using TaskFunction = void (*)(void*);
xTaskCreate(sensorTask, "sensor", 4096, nullptr, 5, nullptr);

// ✅ Allowed — driver callback typedef
using OnFrameReceived = void (*)(const uint8_t*, uint8_t);

// ❌ Forbidden — general dispatch table
void (*handlers[256])(Packet*);
handlers[pkt.type](pkt);
```

### PO10-9.4 — Prefer references over pointers

In C++ code, prefer `const&` references for read-only access.
References cannot be null, eliminating an entire class of bugs:

```cpp
// ✅ Reference — cannot be null, clear intent
float computeAltitude(const SensorData& data);

// Use pointer only when:
// - The value can legitimately be absent (nullable)
// - The API is C-compatible
// - RTOS requires void*
```

---

## PO10-10 — Zero Warnings, Static Analysis

**Original rule (Holzmann #10):** *All code must be compiled, from
the first day of development, with all compiler warnings enabled at
the most pedantic setting available. All code must compile without
warnings. All code must also be checked daily with at least one, but
preferably more than one, strong static source code analyzer and
should pass all analyses with zero warnings.*

### PO10-10.1 — Compiler flags

The following flags are mandatory for **every** build (see DO-12.3):

```ini
# platformio.ini
build_flags =
    -Wall
    -Wextra
    -Werror
    -Wconversion
    -Wshadow
    -Wdouble-promotion
    -Wformat=2
    -Wundef
    -fno-exceptions
    -fno-rtti
```

| Flag               | Purpose                                        |
|--------------------|------------------------------------------------|
| `-Wall`            | Standard warnings                              |
| `-Wextra`          | Additional warnings                            |
| `-Werror`          | Treat all warnings as errors                   |
| `-Wconversion`     | Implicit conversion warnings (MISRA-2)         |
| `-Wshadow`         | Variable shadowing (MISRA-15)                  |
| `-Wdouble-promotion`| Implicit `float` → `double` (MISRA-18)       |
| `-Wformat=2`       | Format string security (CERT-3)                |
| `-Wundef`          | Undefined macro in `#if` — catches typos       |
| `-fno-exceptions`  | Disable C++ exceptions (MISRA-21.3)            |
| `-fno-rtti`        | Disable RTTI — saves flash, prevents `dynamic_cast` |

### PO10-10.2 — Zero-warning policy

- **No** `#pragma GCC diagnostic ignored` to silence warnings.
- **No** `-Wno-*` flags to selectively disable warnings.
- If a third-party library produces warnings, isolate it behind
  an adapter (DO-14.3) and suppress warnings **only** for that
  include:
  ```cpp
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wconversion"
  #include <ThirdPartyLib.h>
  #pragma GCC diagnostic pop
  ```

### PO10-10.3 — Static analysis

In addition to compiler warnings, code must be checked with at
least one static analysis tool:

| Tool               | Type             | When                          |
|--------------------|------------------|-------------------------------|
| GCC/Xtensa warnings| Compiler built-in| Every build                   |
| `cppcheck`         | Static analyzer  | Pre-commit or CI              |
| `clang-tidy`       | Linter + analyzer| Desktop builds (native target)|
| PVS-Studio (optional)| Deep analyzer | Periodic audit                |

**Target:** Zero warnings from all enabled analyzers. New warnings
introduced by a commit block the merge.

### PO10-10.4 — First-day compliance

This rule applies from the **first day of development**. Code
must compile cleanly from the very first commit. "We'll fix the
warnings later" is not acceptable — warnings accumulate, normalize,
and hide real bugs.

---

## Anti-Patterns (Forbidden)

| Anti-Pattern                                     | Rule      |
|--------------------------------------------------|-----------|
| `goto`, `setjmp`, `longjmp`                      | PO10-1    |
| Unguarded recursion                              | PO10-1    |
| C++ exceptions (`throw`/`catch`)                 | PO10-1    |
| Loop without static upper bound                  | PO10-2    |
| `while (serial.available())` without counter     | PO10-2    |
| RTOS task loop without blocking call              | PO10-2    |
| `malloc`/`new` after init                        | PO10-3    |
| `std::string`, `std::vector`, Arduino `String`   | PO10-3    |
| Functions > 80 lines                             | PO10-4    |
| Function with < 2 assertions (non-trivial)       | PO10-5    |
| Tautological assertion (`ARES_ASSERT(len >= 0)` for unsigned) | PO10-5 |
| Side effects inside assertions                   | PO10-5    |
| `ARES_ASSERT` on external data                   | PO10-5    |
| Mutable global variable                          | PO10-6    |
| Variable declared far from first use             | PO10-6    |
| Ignored return value                             | PO10-7    |
| Function that doesn't validate parameters        | PO10-7    |
| Swallowed error (no propagation)                 | PO10-7    |
| Token pasting or variadic macros                 | PO10-8    |
| Function-like macro (use `inline`)               | PO10-8    |
| Deeply nested `#ifdef` (> 1 level)              | PO10-8    |
| Pointer-to-pointer (`**`)                        | PO10-9    |
| Hidden pointer in typedef or macro               | PO10-9    |
| General-purpose function pointer table           | PO10-9    |
| `#pragma GCC diagnostic ignored` in source       | PO10-10   |
| Warning silenced via `-Wno-*` flag               | PO10-10   |
| Code merged with static analysis warnings        | PO10-10   |

---

## Cross-References

| PO10 Rule | Holzmann # | Related ARES Rules                                    |
|-----------|------------|-------------------------------------------------------|
| PO10-1    | 1          | MISRA-5 (single exit), MISRA-21 (no exceptions)       |
| PO10-2    | 2          | RTOS-1 (deterministic), RTOS-6 (periodic tasks)       |
| PO10-3    | 3          | MISRA-9 (no dynamic alloc), RTOS-7 (static stacks)    |
| PO10-4    | 4          | MISRA-12 (function size/complexity)                    |
| PO10-5    | 5          | CERT-1, CERT-2 (validation), CERT-21 (assert discipline)|
| PO10-6    | 6          | MISRA-15 (declaration/scope), CERT-19 (const correctness)|
| PO10-7    | 7          | DO-6 (robustness), MISRA-21 (error handling)           |
| PO10-8    | 8          | MISRA-7 (constexpr), MISRA-17 (preprocessor discipline)|
| PO10-9    | 9          | MISRA-10 (controlled pointers), CERT-22 (pointer safety)|
| PO10-10   | 10         | DO-5 (verification), DO-12 (tool qualification)        |

---

## References

- G.J. Holzmann, "The Power of 10: Rules for Developing
  Safety-Critical Code," *IEEE Computer*, vol. 39, no. 6,
  pp. 95–99, June 2006. DOI: 10.1109/MC.2006.212.
- NASA JPL Institutional Coding Standard for the C Programming
  Language, JPL D-60411, Revision 1.0, March 2009.
- MISRA C:2023 — *Guidelines for the Use of the C Language in
  Critical Systems*, Fourth Edition (companion standard).
- ESP-IDF FreeRTOS documentation:
  https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-reference/system/freertos.html

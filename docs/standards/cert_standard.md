# ARES CERT C/C++ Standard

**Amateur Rocket Embedded System**
Secure handling of hostile inputs, RF corruption, external data,
concurrency hazards, and hardware interaction on ESP32-S3.
Derived from SEI CERT C Coding Standard (2016 Edition) and
SEI CERT C++ Coding Standard, adapted for amateur rocketry.
Companion to the [ARES Coding Standard](ares_coding_standard.md).

---

## Context

Radio links are inherently unreliable: RF noise, bit flips, partial
frames, and malformed data are expected. Every byte from an external
source (radio, UART, sensors) must be treated as potentially hostile.

Additionally, the ESP32-S3 is a dual-core microcontroller running
FreeRTOS. Concurrent access to shared resources, interrupt service
routines, hardware registers, and limited stack memory introduce
failure modes that are invisible in single-threaded development but
catastrophic in flight.

This standard covers **all** code that handles external data, shared
state, hardware peripherals, and safety-critical operations. Each
rule cites the relevant SEI CERT rule identifier(s) for traceability.

### Scope

| Data Source / Concern      | Threat Model                            | Rules           |
|----------------------------|-----------------------------------------|-----------------|
| Radio frames               | Corrupted, truncated, replayed, forged  | CERT-1 … CERT-8 |
| UART (GPS, config)         | Malformed sentences, buffer overflow    | CERT-1 … CERT-3 |
| Sensor readings            | Out-of-range, stuck, NaN                | CERT-1, CERT-16 |
| WiFi API requests          | Injection, oversized payloads           | CERT-1, CERT-3  |
| Shared RTOS state          | Race conditions, priority inversion     | CERT-13          |
| ISR ↔ task boundary        | Non-atomic access, stack corruption     | CERT-15, CERT-17|
| Hardware registers         | Volatile access, reorder hazards        | CERT-15          |
| Stack memory               | Overflow, deep call chains              | CERT-14          |
| Floating-point arithmetic  | NaN propagation, comparison traps       | CERT-16          |

---

## CERT-1 — Validate All External Input

**SEI CERT:** EXP34-C, INT04-C, STR31-C

Everything from radio, UART, sensors, and WiFi must be validated
**before** use. No external data enters application logic unchecked.

### Validation checklist

Every external input path must enforce:

| Check               | Description                              | Example                    |
|----------------------|------------------------------------------|----------------------------|
| Range               | Value within valid bounds                | `0 ≤ phase ≤ 7`           |
| Length               | Size ≤ buffer capacity                   | `len ≤ MAX_PAYLOAD_LEN`   |
| Type                 | Correct data type / encoding             | Valid PTC/PFC (APUS-11)    |
| Consistency          | Cross-field coherence                    | `len` matches actual bytes |
| Integrity            | CRC / checksum valid                     | CRC-32 (APUS-1)           |

```cpp
// ❌ No validation
processPayload(rxBuffer, rxLen);

// ✅ Validate before use
if (rxLen < MIN_FRAME_LEN || rxLen > MAX_FRAME_LEN)
{
    LOG_W(TAG, "frame length out of range: %u", rxLen);
    return false;
}
if (!crc32Verify(rxBuffer, rxLen))
{
    return false;
}
processPayload(rxBuffer, rxLen);
```

### Sources requiring validation

| Source         | Primary Risk                    | Minimum Checks                    |
|----------------|---------------------------------|-----------------------------------|
| Radio frames   | Corruption, truncation, replay  | CRC, length, type, sequence       |
| UART GPS       | Malformed NMEA, missing fields  | Sentence ID, checksum, field count|
| Sensor I²C/SPI | Stuck value, out-of-range       | Range, delta from previous, valid flag |
| WiFi HTTP body | Injection, oversized payload    | Content-Length, JSON schema       |
| Flash NVS      | Corrupted after power loss      | CRC, magic byte, version field    |

---

## CERT-2 — Bounds Checking Always

**SEI CERT:** ARR30-C, ARR38-C, ARR39-C

No buffer access without bounds check. Every index must be validated
against the buffer size **before** the access occurs.

```cpp
// ❌ Unchecked index
buffer[i] = data;

// ✅ Bounds-checked access
if (i < BUFFER_SIZE)
{
    buffer[i] = data;
}
```

### Common patterns

```cpp
// Array iteration — always use named constant
for (uint8_t i = 0; i < SENSOR_COUNT; i++)
{
    readings[i] = readSensor(i);  // SENSOR_COUNT == array size
}
static_assert(SENSOR_COUNT <= sizeof(readings) / sizeof(readings[0]),
              "SENSOR_COUNT exceeds readings array");

// Ring buffer — mask-based bounds (power-of-two size)
static constexpr uint16_t BUF_SIZE = 256;  // must be power of 2
static_assert((BUF_SIZE & (BUF_SIZE - 1)) == 0, "must be power of 2");
buffer[index & (BUF_SIZE - 1)] = value;

// memcpy — destination size governs
memcpy(dest, src, min(srcLen, sizeof(dest)));
```

### Rules

| ID       | Rule                                                           |
|----------|----------------------------------------------------------------|
| CERT-2.1 | Every array index must be checked against the array bound.     |
| CERT-2.2 | `memcpy`/`memmove` length must never exceed destination size.  |
| CERT-2.3 | Ring buffers must use power-of-two sizes with bitmask indexing. |
| CERT-2.4 | `static_assert` must verify array size vs named constant.      |

---

## CERT-3 — No Unsafe String Functions

**SEI CERT:** STR31-C, STR32-C, STR38-C

Unbounded string functions are the most common source of buffer
overflows in C/C++. They are unconditionally **forbidden**.

### Forbidden functions

| Forbidden   | Replacement              | Notes                              |
|-------------|--------------------------|-------------------------------------|
| `strcpy`    | `strncpy` + null-term    | Always `dest[n-1] = '\0'`          |
| `sprintf`   | `snprintf`               | Always pass `sizeof(buf)`          |
| `gets`      | —                        | Removed in C11. Never use.         |
| `strcat`    | `strncat` or manual copy | Calculate remaining space first    |
| `scanf`     | `sscanf` with width spec | `%16s` instead of `%s`             |
| `strlen` on untrusted data | `strnlen`  | Pass max expected length           |

### Examples

```cpp
// ❌ Unbounded
sprintf(buf, "IP=%s", ip);
strcpy(dest, src);

// ✅ Bounded
snprintf(buf, sizeof(buf), "IP=%s", ip);
strncpy(dest, src, sizeof(dest) - 1);
dest[sizeof(dest) - 1] = '\0';
```

### Null termination

| ID       | Rule                                                            |
|----------|------------------------------------------------------------------|
| CERT-3.1 | All strings must be null-terminated after copy or format.       |
| CERT-3.2 | String buffers must be sized to include the null terminator.    |
| CERT-3.3 | `strnlen()` must be used when measuring untrusted string length.|
| CERT-3.4 | String literals in flash (`PROGMEM` / `constexpr`) are preferred.|

---

## CERT-4 — Integer Overflow Awareness

**SEI CERT:** INT30-C, INT31-C, INT32-C, INT33-C

All arithmetic on sizes, lengths, offsets, and counters must consider
overflow. External data controls these values — treat them as hostile.

### Unsigned overflow (wrapping)

```cpp
// ❌ Silent wrap on overflow
uint16_t total = headerLen + payloadLen;

// ✅ Check before arithmetic
if (payloadLen > UINT16_MAX - headerLen)
{
    return false;
}
uint16_t total = static_cast<uint16_t>(headerLen + payloadLen);
```

### Signed overflow (undefined behavior)

```cpp
// ❌ UB if a + b overflows
int32_t result = a + b;

// ✅ Check before arithmetic
if ((b > 0 && a > INT32_MAX - b) ||
    (b < 0 && a < INT32_MIN - b))
{
    return false;
}
int32_t result = a + b;
```

### Division and modulo

```cpp
// ❌ Division by zero
uint32_t avg = sum / count;

// ✅ Guard divisor
if (count == 0)
{
    return 0;
}
uint32_t avg = sum / count;
```

### Shift operations

```cpp
// ❌ Shift amount may exceed type width
uint8_t result = value << shift;

// ✅ Validate shift amount
ARES_ASSERT(shift < 8);
uint8_t result = static_cast<uint8_t>(value << shift);
```

### Rules

| ID       | Rule                                                             |
|----------|------------------------------------------------------------------|
| CERT-4.1 | All addition/subtraction on external sizes must check overflow.  |
| CERT-4.2 | Division/modulo must check for zero divisor.                     |
| CERT-4.3 | Shift amount must be `< bit_width` of the shifted type.          |
| CERT-4.4 | Multiplication of external values must use wider intermediate type.|
| CERT-4.5 | Signed arithmetic must not rely on wrapping behavior (UB).       |

---

## CERT-5 — Defensive Parsing

**SEI CERT:** EXP34-C, MEM35-C

Never trust packet structure. Validate total length, then validate
each field offset before accessing it. A truncated or malformed
frame must never cause an out-of-bounds read.

```cpp
// ❌ Assumes correct packet layout
type = buffer[0];

// ✅ Validate length first
if (bufLen < MIN_FRAME_LEN)
{
    return false;
}
type = buffer[0];
```

### Multi-field parsing discipline

```cpp
bool parseFrame(const uint8_t* buf, uint16_t len, Frame& out)
{
    // 1. Minimum length
    if (len < HEADER_LEN + CRC_LEN)
    {
        return false;
    }

    // 2. Sync marker
    if (memcmp(buf, SYNC_MARKER, SYNC_LEN) != 0)
    {
        return false;
    }

    // 3. Declared payload length vs actual bytes
    const uint8_t payloadLen = buf[OFFSET_LEN];
    if (payloadLen > MAX_PAYLOAD_LEN)
    {
        return false;
    }
    if (len < HEADER_LEN + payloadLen + CRC_LEN)
    {
        return false;
    }

    // 4. CRC integrity
    if (!crc32Verify(buf + SYNC_LEN,
                     HEADER_LEN - SYNC_LEN + payloadLen))
    {
        return false;
    }

    // 5. Only now access individual fields
    out.version = buf[OFFSET_VER];
    out.type    = buf[OFFSET_TYPE];
    // ...
    return true;
}
```

### Rules

| ID       | Rule                                                              |
|----------|-------------------------------------------------------------------|
| CERT-5.1 | Never access a field without first verifying the buffer contains it.|
| CERT-5.2 | Parsing must proceed sequentially: length → sync → declared len → CRC → fields. |
| CERT-5.3 | Parser functions must return `bool` or an error code, never void. |
| CERT-5.4 | Partial parses are forbidden — either the entire frame is valid or it is discarded. |

The radio protocol already implements this pattern — see
[ares_radio_protocol.md](../architecture/ares_radio_protocol.md).

---

## CERT-6 — Validate Enum Values

**SEI CERT:** INT36-C, EXP37-C

Received enum values are untrusted integers. An out-of-range cast
produces undefined behavior in C++. Always range-check before cast.

```cpp
// ❌ Direct cast of wire data — UB if out of range
MsgType t = static_cast<MsgType>(wire[7]);

// ✅ Range-check first
const uint8_t raw = wire[7];
if (raw < static_cast<uint8_t>(MsgType::FIRST) ||
    raw > static_cast<uint8_t>(MsgType::LAST))
{
    return false;
}
MsgType t = static_cast<MsgType>(raw);
```

### Rules

| ID       | Rule                                                             |
|----------|------------------------------------------------------------------|
| CERT-6.1 | Every enum type must define `FIRST` and `LAST` sentinel values.  |
| CERT-6.2 | Cast from integer to enum requires a range check.                |
| CERT-6.3 | `switch` on enums must have a `default` that logs + rejects.     |
| CERT-6.4 | Enum underlying types must be explicit (`enum class X : uint8_t`).|

---

## CERT-7 — No Trust in Length Fields

**SEI CERT:** INT04-C, ARR38-C

A length field inside a packet is **attacker-controlled data**. It
must be cross-checked against the actual received byte count and
the protocol's maximum payload size.

```cpp
const uint8_t payloadLen = buf[OFFSET_LEN];

// Check 1: declared length ≤ protocol maximum
if (payloadLen > MAX_PAYLOAD_LEN)
{
    return false;
}

// Check 2: declared length ≤ actual received bytes (minus header + CRC)
const uint16_t expectedTotal = HEADER_LEN + payloadLen + CRC_LEN;
if (bufLen < expectedTotal)
{
    return false;
}

// Only now is it safe to access buf[HEADER_LEN .. HEADER_LEN + payloadLen]
```

### Rules

| ID       | Rule                                                             |
|----------|------------------------------------------------------------------|
| CERT-7.1 | Declared length must be checked against protocol maximum.        |
| CERT-7.2 | Declared length must be checked against actual received bytes.   |
| CERT-7.3 | Length fields must be unsigned — negative lengths are impossible. |
| CERT-7.4 | Computed offsets using length fields must be overflow-checked.    |

---

## CERT-8 — Fail Safe on Error

**SEI CERT:** ERR33-C, ERR34-C

On invalid data the system must fail to a **safe state**, never to an
undefined one. The safe default is: discard, log, continue.

### Error handling discipline

1. **Discard** the entire frame / operation.
2. **Never** partially process corrupt data.
3. **Log** the error with category and context for post-flight analysis.
4. **Continue** normal operation — do not halt unless safety requires it.
5. **Never fire pyros** or change flight phase on unexpected state.

```cpp
if (!decode(wire, wireLen, frame))
{
    LOG_W(TAG, "frame decode failed (len=%u) — discarding", wireLen);
    stats.decodeErrors++;
    return;  // discard and continue
}
```

### Safe state table

| Subsystem      | Safe Default                        | Rationale                   |
|----------------|-------------------------------------|-----------------------------|
| Flight phase   | Remain in current phase             | Never advance on bad data   |
| Pyro channels  | Remain disarmed                     | CERT-12 redundancy applies  |
| Telemetry      | Continue with last known good data  | Better stale than wrong     |
| Radio TX       | Skip frame, retry next cycle        | Transient loss is acceptable|
| FCS engine     | Maintain current rules              | No rule changes on bad input|

### Rules

| ID       | Rule                                                             |
|----------|------------------------------------------------------------------|
| CERT-8.1 | Corrupt data must never cause a state transition.                |
| CERT-8.2 | All error paths must log with a diagnostic tag.                  |
| CERT-8.3 | Error counters must be maintained for post-flight analysis.      |
| CERT-8.4 | Functions must return error codes — silent failures are forbidden.|

---

## CERT-9 — Zero Sensitive Buffers

**SEI CERT:** MEM33-C, MSC06-C

Clear buffers after use to prevent stale data from leaking into
subsequent operations. This is critical for reused TX/RX buffers
where a shorter new frame could leave old payload bytes in the tail.

```cpp
// After processing a received frame
memset(rxBuffer, 0, sizeof(rxBuffer));

// Before building a new TX frame
memset(txBuffer, 0, sizeof(txBuffer));
```

### When to zero

| Buffer Type             | Zero When                              |
|-------------------------|----------------------------------------|
| Radio RX buffer         | After frame dispatch (before next RX)  |
| Radio TX buffer         | Before building each new frame         |
| Config command buffer   | After processing command               |
| Parsed frame struct     | On declaration (use `= {}` in C++)     |
| Crypto / auth material  | Immediately after use                  |

### Rules

| ID       | Rule                                                            |
|----------|-----------------------------------------------------------------|
| CERT-9.1 | RX buffers must be zeroed after dispatch.                       |
| CERT-9.2 | TX buffers must be zeroed before encoding.                      |
| CERT-9.3 | Structs received from external sources must use `= {}` init.    |
| CERT-9.4 | `memset` of sensitive data must not be optimized away — use     |
|          | `volatile` pointer cast or ESP-IDF `esp_fill_random` for crypto.|

---

## CERT-10 — Timeouts Everywhere

**SEI CERT:** CON33-C, POS44-C

All external communication and blocking operations must have bounded
timeouts. No operation blocks forever. An unbounded wait on a failed
peripheral can freeze the entire flight computer.

| Operation         | Timeout          | Notes                        |
|-------------------|------------------|------------------------------|
| UART read         | Bounded by len   | Byte-count or time limit     |
| Radio send        | `TX_TIMEOUT_MS`  | Typically 2000 ms            |
| AUX pin wait      | `AUX_TIMEOUT_MS` | Radio module busy signal     |
| Semaphore take    | 50 ms typical    | Never `portMAX_DELAY`        |
| Mutex acquire     | 100 ms typical   | Log on timeout               |
| ACK wait          | `ACK_TIMEOUT_MS` | 1000 ms (APUS-4.5)          |
| I²C transaction   | `I2C_TIMEOUT_MS` | 50 ms per transaction        |
| SPI transaction   | `SPI_TIMEOUT_MS` | 10 ms per transaction        |
| Flash write       | `NVS_TIMEOUT_MS` | 500 ms max                   |

### Forbidden patterns

```cpp
// ❌ FORBIDDEN — blocks forever
xSemaphoreTake(sem, portMAX_DELAY);
while (!radioReady()) {}
Wire.requestFrom(addr, len);  // no timeout parameter

// ✅ Bounded
if (xSemaphoreTake(sem, pdMS_TO_TICKS(50)) != pdTRUE)
{
    LOG_W(TAG, "semaphore timeout");
    return false;
}
```

### Rules

| ID        | Rule                                                            |
|-----------|-----------------------------------------------------------------|
| CERT-10.1 | `portMAX_DELAY` is **forbidden** in flight-critical code.       |
| CERT-10.2 | Every `while` waiting on hardware must have a timeout counter.  |
| CERT-10.3 | Timeout values must be named constants, not magic numbers.      |
| CERT-10.4 | Timeout expiry must be logged and counted for diagnostics.      |
| CERT-10.5 | I²C/SPI drivers must use ESP-IDF timeout-aware APIs.            |

---

## CERT-11 — Avoid Undefined Behavior

**SEI CERT:** EXP30-C, EXP33-C, INT34-C, INT36-C

Undefined behavior (UB) in C/C++ means the compiler is free to do
**anything** — including generating code that appears to work in
testing but fails silently in flight. Every potential UB source
must be explicitly guarded.

### UB prevention matrix

| UB Source             | Prevention                             | Example Guard             |
|-----------------------|----------------------------------------|---------------------------|
| Out-of-bounds access  | Bounds check before every access       | `if (i < N)`              |
| Invalid shift         | Shift amount < type width              | `ARES_ASSERT(shift < 8)`  |
| Division by zero      | Check divisor ≠ 0                      | `if (count == 0) return`  |
| Null dereference      | `ARES_ASSERT(ptr != nullptr)`          | Check at function entry   |
| Signed overflow       | Pre-check or use unsigned (MISRA-1)    | `if (a > MAX - b) return` |
| Unaligned access      | Use `memcpy` for unaligned reads       | `memcpy(&val, buf+3, 4)` |
| Use-after-free        | Forbidden (no dynamic memory, PO10-3)  | Static allocation only    |
| Double free           | Forbidden (no dynamic memory, PO10-3)  | Static allocation only    |
| Uninitialized read    | Initialize all variables at declaration| `uint8_t x = 0;`         |
| Strict aliasing       | Use `memcpy` instead of pointer casts  | See example below         |

### Strict aliasing

```cpp
// ❌ Aliasing violation — UB
float f = *reinterpret_cast<float*>(&intVal);

// ✅ Safe type punning via memcpy
float f;
memcpy(&f, &intVal, sizeof(float));
static_assert(sizeof(float) == sizeof(uint32_t), "size mismatch");
```

### Unaligned memory access

```cpp
// ❌ May fault on some architectures (ESP32-S3 tolerates but is slower)
uint32_t val = *reinterpret_cast<uint32_t*>(buf + 3);

// ✅ Safe unaligned read
uint32_t val;
memcpy(&val, buf + 3, sizeof(val));
```

### Rules

| ID        | Rule                                                            |
|-----------|-----------------------------------------------------------------|
| CERT-11.1 | All local variables must be initialized at declaration.         |
| CERT-11.2 | Pointer casts for type punning are forbidden — use `memcpy`.    |
| CERT-11.3 | Unaligned reads must use `memcpy`, not pointer dereference.     |
| CERT-11.4 | `-Wall -Wextra -Werror` must be enabled in all build configs.   |
| CERT-11.5 | Compiler warnings are treated as errors — zero warnings policy. |

---

## CERT-12 — Redundancy for Critical Data

**SEI CERT:** MSC12-C (adapted)

Critical commands (pyro firing, flight abort) require multiple layers
of validation. A single corrupted byte must never cause a pyrotechnic
event.

### Validation layers

```
Command received
  └── CRC valid?          → NO → discard
  └── Fields in range?    → NO → discard + NACK
  └── Duplicate seq?      → YES → discard (already executed)
  └── Priority correct?   → NO → discard + NACK
  └── Armed state?        → NO → discard + NACK
  └── Confirmed?          → NO → wait for confirmation
  └── Execute
```

### Two-command arming pattern

Pyro channels must never fire from a single command. The required
sequence is:

```
Ground ── ARM_FLIGHT ──► Rocket    (arms the FSM)
          ... time passes ...
Ground ── FIRE_PYRO_A ──► Rocket   (only accepted if armed)
```

### Additional redundancy mechanisms

| Mechanism                  | Applies To              | Description                       |
|----------------------------|-------------------------|-----------------------------------|
| CRC-32 integrity           | All frames              | APUS-1                            |
| Priority verification      | CRITICAL commands       | Must carry `FLAG_PRIORITY`        |
| Armed state gate           | Pyro commands           | Rejected unless armed (APUS-7.2)  |
| Sequence deduplication     | All commands            | SEQ-based duplicate detection     |
| Retransmission (3×)        | CRITICAL commands       | Sender retransmits (APUS-2.3)     |
| Hardware interlock         | Pyro channels           | Physical continuity check         |

### Rules

| ID        | Rule                                                             |
|-----------|------------------------------------------------------------------|
| CERT-12.1 | Pyro channels require ARM + FIRE sequence (never single command).|
| CERT-12.2 | CRITICAL commands must pass all 6 validation layers.             |
| CERT-12.3 | Hardware interlocks must be verified before pyro firing.         |
| CERT-12.4 | Arming must timeout — auto-disarm after 60 s without firing.     |
| CERT-12.5 | All pyro events must be logged with timestamp and full context.  |

---

## CERT-13 — Concurrency Safety

**SEI CERT:** CON31-C, CON32-C, CON33-C, CON34-C, CON37-C

The ESP32-S3 runs FreeRTOS on dual cores. Shared data between tasks
and between tasks and ISRs must be protected. A race condition in
the flight state machine can cause catastrophic failure.

### Shared data protection

| Access Pattern                    | Required Protection               |
|-----------------------------------|-----------------------------------|
| Task ↔ Task (same core)          | Mutex or critical section         |
| Task ↔ Task (cross-core)         | Mutex with `portMUX_TYPE` spinlock|
| Task ↔ ISR                       | `taskENTER_CRITICAL` / `portMUX`  |
| Read-only after init             | None (immutable data)             |
| Single-writer, single-reader     | `volatile` + atomic load/store    |
| Multi-writer or multi-reader     | Mutex required                    |

### Critical section patterns

```cpp
// ❌ Unprotected shared state
flightPhase_ = newPhase;  // written by task A, read by task B

// ✅ Protected by mutex
if (xSemaphoreTake(phaseMutex_, pdMS_TO_TICKS(50)) == pdTRUE)
{
    flightPhase_ = newPhase;
    xSemaphoreGive(phaseMutex_);
}

// ✅ Atomic for simple single-writer scenarios
std::atomic<uint8_t> flightPhase_{0};
flightPhase_.store(newPhase, std::memory_order_release);
uint8_t phase = flightPhase_.load(std::memory_order_acquire);
```

### Forbidden patterns

```cpp
// ❌ Global mutable state without protection
volatile bool armed = false;  // volatile is NOT a synchronisation primitive

// ❌ Lock ordering violation (deadlock risk)
xSemaphoreTake(mutexA, ...);
xSemaphoreTake(mutexB, ...);  // if another task takes B then A → deadlock

// ❌ Unbounded critical section
taskENTER_CRITICAL(&mux);
slowFlashWrite();  // FORBIDDEN — blocks other tasks/ISRs
taskEXIT_CRITICAL(&mux);
```

### Rules

| ID        | Rule                                                            |
|-----------|-----------------------------------------------------------------|
| CERT-13.1 | All mutable shared state must have a documented owner or lock.  |
| CERT-13.2 | Mutexes must be acquired in a global, documented order.         |
| CERT-13.3 | Critical sections must be ≤ 10 μs (no I/O, no logging).        |
| CERT-13.4 | `volatile` alone is **not** a synchronisation mechanism.        |
| CERT-13.5 | Use `std::atomic` for lockless single-writer/single-reader.     |
| CERT-13.6 | Mutex acquire must have a timeout — never `portMAX_DELAY`.      |

---

## CERT-14 — Stack Overflow Protection

**SEI CERT:** MEM05-C, ARR32-C

The ESP32-S3 has limited RAM (~512 KB). Each FreeRTOS task gets a
fixed stack allocation. Stack overflow corrupts memory silently and
is extremely difficult to debug post-crash.

### Prevention strategies

| Strategy                     | Implementation                          |
|------------------------------|-----------------------------------------|
| Fixed stack sizes            | Named constants per task in `config.h`  |
| Stack watermark monitoring   | `uxTaskGetStackHighWaterMark()` periodic |
| No VLAs                      | Forbidden (MISRA rule, PO10-3)          |
| No `alloca`                  | Forbidden                               |
| No large local arrays        | Max 64 bytes on stack; larger → static  |
| No deep recursion            | Forbidden (PO10-1)                      |
| Compiler stack analysis      | `-fstack-usage` flag                    |

### Stack size guidelines

| Task                  | Minimum Stack | Rationale                       |
|-----------------------|---------------|---------------------------------|
| Flight control (FCS)  | 4096 bytes    | Sensor reads + state machine    |
| Radio TX/RX           | 4096 bytes    | Frame encode/decode buffers     |
| Telemetry             | 2048 bytes    | Sensor sampling + formatting    |
| GPS parser            | 2048 bytes    | NMEA sentence parsing           |
| Logging               | 2048 bytes    | Flash writes + formatting       |
| LED / status          | 1024 bytes    | Minimal state display           |

### Monitoring

```cpp
// Periodic stack check (run in diagnostics task)
void checkStacks()
{
    const UBaseType_t watermark = uxTaskGetStackHighWaterMark(NULL);
    if (watermark < STACK_WATERMARK_THRESHOLD)
    {
        LOG_W(TAG, "stack low: %u words remaining", watermark);
    }
}
```

### Rules

| ID        | Rule                                                            |
|-----------|-----------------------------------------------------------------|
| CERT-14.1 | Stack sizes must be defined as named constants in `config.h`.   |
| CERT-14.2 | VLAs and `alloca` are **forbidden**.                            |
| CERT-14.3 | Local arrays must not exceed 64 bytes — use `static` for larger.|
| CERT-14.4 | Stack watermark must be checked periodically and logged.        |
| CERT-14.5 | FreeRTOS stack overflow hook must be enabled in `sdkconfig`.    |

---

## CERT-15 — Volatile and Hardware Register Access

**SEI CERT:** EXP32-C, DCL17-C

Hardware registers (GPIO, SPI status, DMA descriptors) and ISR-shared
variables must use `volatile` to prevent the compiler from optimizing
away reads or reordering accesses.

### When `volatile` is required

| Scenario                              | Required? | Notes                      |
|---------------------------------------|-----------|----------------------------|
| Memory-mapped I/O register            | **Yes**   | Compiler must not cache    |
| Variable shared between task and ISR  | **Yes**   | Plus critical section      |
| Variable shared between tasks only    | No        | Use mutex or atomic instead|
| Loop variable checked by another task | No        | Use atomic                 |
| DMA buffer descriptors                | **Yes**   | DMA writes are invisible   |

### Correct ISR-shared variable pattern

```cpp
// ISR writes, task reads
volatile bool isrFlag_ = false;

// In ISR
void IRAM_ATTR onPinInterrupt()
{
    isrFlag_ = true;
}

// In task — must read inside critical section for multi-field coherence
taskENTER_CRITICAL(&mux_);
bool flag = isrFlag_;
isrFlag_ = false;
taskEXIT_CRITICAL(&mux_);
```

### Hardware register access

```cpp
// ❌ Compiler may optimize away the read
uint32_t status = *statusRegister;
while (status & BUSY_BIT) {
    status = *statusRegister;  // compiler may hoist this out of loop
}

// ✅ volatile pointer to register
volatile uint32_t* statusReg = reinterpret_cast<volatile uint32_t*>(REG_ADDR);
while (*statusReg & BUSY_BIT) {}  // read on every iteration
```

### Rules

| ID        | Rule                                                            |
|-----------|-----------------------------------------------------------------|
| CERT-15.1 | All ISR-shared variables must be declared `volatile`.           |
| CERT-15.2 | Hardware register pointers must be `volatile`-qualified.        |
| CERT-15.3 | `volatile` does **not** provide atomicity — pair with critical  |
|           | sections for multi-byte data.                                   |
| CERT-15.4 | DMA buffers must be declared `volatile` or flushed explicitly.  |
| CERT-15.5 | ESP-IDF register access macros (`REG_READ`, `REG_WRITE`) are   |
|           | preferred over raw pointer access.                              |

---

## CERT-16 — Floating-Point Discipline

**SEI CERT:** FLP30-C, FLP32-C, FLP34-C, FLP36-C

The ESP32-S3 has no hardware FPU double-precision support (only
single-precision). Floating-point errors (NaN, infinity, precision
loss) propagate silently and can corrupt sensor data, altitude
calculations, and FCS decisions.

### Forbidden patterns

```cpp
// ❌ Equality comparison on floats — almost always wrong
if (altitude == 0.0f) { ... }

// ✅ Epsilon-based comparison
constexpr float EPSILON = 1e-3f;
if (fabsf(altitude) < EPSILON) { ... }

// ❌ Unchecked sensor reading — may be NaN
float pressure = readPressure();
altitude = calculateAltitude(pressure);

// ✅ NaN/Inf guard
float pressure = readPressure();
if (!isfinite(pressure) || pressure < MIN_PRESSURE_PA ||
    pressure > MAX_PRESSURE_PA)
{
    LOG_W(TAG, "pressure invalid: %f", pressure);
    return lastKnownAltitude_;  // fail safe
}
altitude = calculateAltitude(pressure);
```

### Type discipline

| ID        | Rule                                                           |
|-----------|----------------------------------------------------------------|
| CERT-16.1 | Never compare floats with `==` or `!=` — use epsilon.         |
| CERT-16.2 | All sensor readings must be checked for NaN/Inf before use.    |
| CERT-16.3 | Use `float` (32-bit), never `double` (no HW support on S3).   |
| CERT-16.4 | Division must check for near-zero divisor (within epsilon).    |
| CERT-16.5 | Float-to-integer conversion must check range before cast.      |
| CERT-16.6 | Altitude, velocity, and pressure must have documented valid    |
|           | ranges; out-of-range values must be clamped or rejected.       |

### Valid range table

| Parameter      | Type    | Valid Range           | Invalid Action      |
|----------------|---------|-----------------------|---------------------|
| Pressure       | float   | 1000 – 120000 Pa     | Use last known      |
| Temperature    | float   | −40 – +85 °C         | Use last known      |
| Altitude AGL   | float   | −100 – 50000 m       | Clamp to bounds     |
| Vertical vel.  | float   | −500 – +500 m/s      | Use last known      |
| Acceleration   | float   | 0 – 300 m/s²         | Use last known      |
| Battery        | uint8_t | 0 – 100 %            | Clamp to bounds     |

---

## CERT-17 — ISR Safety

**SEI CERT:** SIG31-C, CON37-C, MSC14-C

Interrupt Service Routines (ISRs) on the ESP32-S3 run in a special
context with severe constraints. Violating these constraints causes
crashes, watchdog resets, or silent memory corruption.

### ISR constraints

| Constraint                   | Reason                               |
|------------------------------|--------------------------------------|
| No blocking calls            | ISRs cannot wait (no scheduler)      |
| No heap allocation           | `malloc`/`new` are not ISR-safe      |
| No FreeRTOS API (normal)     | Must use `FromISR` variants only     |
| Minimal execution time       | Blocks all lower-priority interrupts |
| No `printf` / logging        | Not ISR-safe on ESP32-S3             |
| IRAM_ATTR required           | ISR code must be in IRAM, not flash  |

### Correct ISR pattern

```cpp
// ISR — minimal work: set flag, notify task
void IRAM_ATTR onPpsInterrupt()
{
    BaseType_t higherPriorityWoken = pdFALSE;
    vTaskNotifyGiveFromISR(gpsTaskHandle_, &higherPriorityWoken);
    portYIELD_FROM_ISR(higherPriorityWoken);
}

// Task — does the actual work
void gpsTask(void* param)
{
    for (;;)  // RTOS: task loop
    {
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(2000));
        processGpsPps();  // heavy work here, not in ISR
    }
}
```

### Rules

| ID        | Rule                                                            |
|-----------|-----------------------------------------------------------------|
| CERT-17.1 | ISRs must execute in < 10 μs — defer work to tasks.            |
| CERT-17.2 | ISR functions must be marked `IRAM_ATTR`.                       |
| CERT-17.3 | Only `FromISR` variants of FreeRTOS APIs are permitted in ISRs. |
| CERT-17.4 | No heap allocation (`malloc`, `new`, `String`) in ISRs.         |
| CERT-17.5 | No logging (`LOG_*`, `printf`, `Serial.print`) in ISRs.         |
| CERT-17.6 | ISR-to-task communication must use notifications or queues.     |

---

## CERT-18 — Resource Cleanup (RAII)

**SEI CERT:** MEM31-C, ERR57-CPP

Resources (mutexes, peripherals, file handles) must be released on
every code path, including error paths. C++ RAII (Resource Acquisition
Is Initialization) ensures this automatically.

### RAII pattern for mutexes

```cpp
class MutexGuard
{
public:
    explicit MutexGuard(SemaphoreHandle_t mutex, TickType_t timeout)
        : mutex_(mutex)
        , acquired_(xSemaphoreTake(mutex_, timeout) == pdTRUE)
    {}

    ~MutexGuard()
    {
        if (acquired_)
        {
            xSemaphoreGive(mutex_);
        }
    }

    bool acquired() const { return acquired_; }

    MutexGuard(const MutexGuard&) = delete;
    MutexGuard& operator=(const MutexGuard&) = delete;

private:
    SemaphoreHandle_t mutex_;
    bool acquired_;
};

// Usage
void updateState()
{
    MutexGuard lock(stateMutex_, pdMS_TO_TICKS(50));
    if (!lock.acquired())
    {
        LOG_W(TAG, "state mutex timeout");
        return;
    }
    // ... state is protected, mutex auto-released on exit
}
```

### Rules

| ID        | Rule                                                            |
|-----------|-----------------------------------------------------------------|
| CERT-18.1 | Mutexes must use RAII wrappers — no manual `Give` on error paths.|
| CERT-18.2 | Peripheral init/deinit must be paired in constructors/destructors.|
| CERT-18.3 | Copy/move constructors must be deleted for resource-owning types.|
| CERT-18.4 | Early returns in mutex-protected code must not leak the lock.   |

---

## CERT-19 — Const Correctness

**SEI CERT:** DCL00-C, EXP40-C

Data that should not change must be marked `const`. This is both a
documentation tool and a compiler-enforced safety net.

### Application

```cpp
// ❌ Mutable pointer to data that should not change
void processFrame(uint8_t* data, uint16_t len);

// ✅ Const pointer — caller knows data won't be modified
void processFrame(const uint8_t* data, uint16_t len);

// ✅ Const member function — does not modify object state
uint8_t getPhase() const { return flightPhase_; }

// ✅ Compile-time constants
static constexpr uint16_t MAX_PAYLOAD_LEN = 200;
```

### Rules

| ID        | Rule                                                            |
|-----------|-----------------------------------------------------------------|
| CERT-19.1 | Function parameters that are not modified must be `const`.      |
| CERT-19.2 | Member functions that do not modify state must be `const`.      |
| CERT-19.3 | Compile-time constants must use `constexpr`, not `#define`.     |
| CERT-19.4 | Casting away `const` is **forbidden** unless interfacing with   |
|           | a legacy C API that does not use `const` correctly.             |

---

## CERT-20 — Dead Code and Unreachable Paths

**SEI CERT:** MSC07-C, MSC12-C, MSC15-C

Dead code (code that can never execute) and unhandled paths hide
bugs and make verification impossible.

### `switch` exhaustiveness

```cpp
// ❌ Missing cases — silent fall-through on new enum values
switch (phase)
{
    case IDLE:    break;
    case FLIGHT:  break;
}

// ✅ All cases handled, default catches future additions
switch (phase)
{
    case IDLE:    handleIdle();    break;
    case ARMED:   handleArmed();   break;
    case BOOST:   handleBoost();   break;
    case COAST:   handleCoast();   break;
    case DESCENT: handleDescent(); break;
    case LANDED:  handleLanded();  break;
    default:
        ARES_ASSERT(false && "unhandled flight phase");
        break;
}
```

### Rules

| ID        | Rule                                                            |
|-----------|-----------------------------------------------------------------|
| CERT-20.1 | Every `switch` on an enum must handle all values + `default`.   |
| CERT-20.2 | Unreachable code must be removed, not commented out.            |
| CERT-20.3 | Commented-out code is forbidden in `main` branch — use VCS.    |
| CERT-20.4 | `#ifdef` blocks for disabled features must have a tracking issue.|
| CERT-20.5 | All `if/else` chains must have a terminal `else` clause.        |

---

## CERT-21 — Assertion Discipline

**SEI CERT:** MSC11-C, ERR06-C

Assertions enforce invariants that must **always** be true. They
catch programmer errors (bugs), not runtime conditions (bad input).

### Assertion types

| Type              | Macro              | When                        |
|-------------------|--------------------|-----------------------------|
| Compile-time      | `static_assert`    | Type sizes, array bounds    |
| Runtime (debug)   | `ARES_ASSERT()`    | Invariants, preconditions   |
| Runtime (always)  | `configASSERT()`   | FreeRTOS kernel safety      |

### What to assert vs what to validate

| Condition                           | Mechanism          | Example                   |
|-------------------------------------|--------------------|---------------------------|
| Struct is expected size             | `static_assert`    | `sizeof(Frame) == 14`     |
| Function precondition (internal)    | `ARES_ASSERT`      | `ptr != nullptr`          |
| External data in range             | `if` + return error| `len <= MAX` (CERT-1)     |
| RTOS object creation succeeded      | `configASSERT`     | `mutex != NULL`           |

```cpp
// ✅ Compile-time: catches errors at build time
static_assert(sizeof(TelemetryPayload) == 38, "payload size changed");

// ✅ Runtime: catches programmer error
void enqueue(const Frame& frame)
{
    ARES_ASSERT(frame.type != MsgType::UNKNOWN);
    ARES_ASSERT(frame.len <= MAX_PAYLOAD_LEN);
    // ...
}

// ❌ NEVER assert on external data — use validation (CERT-1)
ARES_ASSERT(rxBuffer[0] == SYNC_BYTE);  // WRONG — attacker controls this
```

### Rules

| ID        | Rule                                                            |
|-----------|-----------------------------------------------------------------|
| CERT-21.1 | `static_assert` must verify all packed struct sizes.            |
| CERT-21.2 | `ARES_ASSERT` must be used for internal invariants only.        |
| CERT-21.3 | Assertions on external data are **forbidden** — use CERT-1.    |
| CERT-21.4 | Assertions must not have side effects in the condition.         |
| CERT-21.5 | Failed assertions must log the file, line, and expression.      |

---

## CERT-22 — Pointer Safety

**SEI CERT:** EXP34-C, MEM30-C, MEM34-C

Pointer misuse is the most dangerous category of bugs in C/C++.
In a system with no MMU (ESP32-S3 uses flat memory), a wild pointer
corrupts memory silently.

### Rules

| ID        | Rule                                                            |
|-----------|-----------------------------------------------------------------|
| CERT-22.1 | All pointers must be checked for `nullptr` at function entry.   |
| CERT-22.2 | Pointer arithmetic is only permitted on arrays with known size. |
| CERT-22.3 | Pointers must not be cast between unrelated types (use `memcpy`).|
| CERT-22.4 | Function pointers must be validated against a known table.      |
| CERT-22.5 | Pointers to local variables must not escape the function scope. |
| CERT-22.6 | `void*` usage is limited to FreeRTOS task parameters — always   |
|           | cast back to the documented type immediately.                   |

### Examples

```cpp
// ❌ No null check
void process(const Frame* frame)
{
    frame->dispatch();  // crash if null
}

// ✅ Null check at entry
void process(const Frame* frame)
{
    if (frame == nullptr)
    {
        ARES_ASSERT(false && "null frame pointer");
        return;
    }
    frame->dispatch();
}

// ❌ Pointer to local escapes
const uint8_t* getBuffer()
{
    uint8_t local[64];
    return local;  // dangling pointer
}

// ✅ Use static or caller-provided buffer
void getBuffer(uint8_t* out, uint16_t outSize)
{
    ARES_ASSERT(out != nullptr);
    // fill out buffer...
}
```

---

## Anti-Patterns (Forbidden)

| Anti-Pattern                               | Rule     |
|--------------------------------------------|----------|
| Unvalidated radio/UART/sensor data         | CERT-1   |
| `buffer[i]` without bounds check           | CERT-2   |
| `strcpy`, `sprintf`, `gets`, `scanf`       | CERT-3   |
| Unchecked arithmetic on external sizes     | CERT-4   |
| Accessing packet fields without length check| CERT-5   |
| Trusting received enum values              | CERT-6   |
| Using declared length without cross-check  | CERT-7   |
| Partial processing of corrupt frames       | CERT-8   |
| Reusing buffers without clearing           | CERT-9   |
| Blocking without timeout                   | CERT-10  |
| Unguarded shifts, division, access         | CERT-11  |
| Single-command pyro firing                 | CERT-12  |
| Unprotected shared mutable state           | CERT-13  |
| VLAs, `alloca`, large local arrays         | CERT-14  |
| Non-volatile access to ISR-shared vars     | CERT-15  |
| Float equality comparison (`==`)           | CERT-16  |
| Blocking or logging in ISRs               | CERT-17  |
| Manual mutex release on error paths        | CERT-18  |
| Mutable parameter that should be `const`   | CERT-19  |
| Unhandled `switch` cases                   | CERT-20  |
| Asserting on external (untrusted) data     | CERT-21  |
| Unchecked `nullptr` dereference            | CERT-22  |

---

## Cross-References

| CERT Rule | SEI CERT Rules                         | Related ARES Rules              |
|-----------|----------------------------------------|---------------------------------|
| CERT-1    | EXP34-C, INT04-C, STR31-C             | PO10-5, APUS-1, APUS-5         |
| CERT-2    | ARR30-C, ARR38-C, ARR39-C             | PO10-2, MISRA-1                 |
| CERT-3    | STR31-C, STR32-C, STR38-C             | PO10-3                          |
| CERT-4    | INT30-C, INT31-C, INT32-C, INT33-C    | MISRA-2                         |
| CERT-5    | EXP34-C, MEM35-C                      | APUS-1, APUS-5, APUS-7         |
| CERT-6    | INT36-C, EXP37-C                      | MISRA-2, APUS-5                 |
| CERT-7    | INT04-C, ARR38-C                      | APUS-1, CERT-2                  |
| CERT-8    | ERR33-C, ERR34-C                      | DO-6, PO10-5                    |
| CERT-9    | MEM33-C, MSC06-C                      | PO10-3                          |
| CERT-10   | CON33-C, POS44-C                      | RTOS-1, RTOS-8                  |
| CERT-11   | EXP30-C, EXP33-C, INT34-C             | MISRA-1, MISRA-13, PO10-10     |
| CERT-12   | MSC12-C                               | APUS-2, APUS-7, APUS-9         |
| CERT-13   | CON31-C, CON32-C, CON33-C             | RTOS-4, RTOS-5, RTOS-12        |
| CERT-14   | MEM05-C, ARR32-C                      | PO10-3, RTOS-7, MISRA-9        |
| CERT-15   | EXP32-C, DCL17-C                      | RTOS-4, MISRA-1                 |
| CERT-16   | FLP30-C, FLP32-C, FLP34-C             | MISRA-1, APUS-12                |
| CERT-17   | SIG31-C, CON37-C, MSC14-C             | RTOS-4, RTOS-7, PO10-3         |
| CERT-18   | MEM31-C, ERR57-CPP                    | RTOS-4, CERT-13                 |
| CERT-19   | DCL00-C, EXP40-C                      | MISRA-1, PO10-8                 |
| CERT-20   | MSC07-C, MSC12-C, MSC15-C             | DO-5, PO10-10                   |
| CERT-21   | MSC11-C, ERR06-C                      | MISRA-7, PO10-4                 |
| CERT-22   | EXP34-C, MEM30-C, MEM34-C             | CERT-11, PO10-3                 |

---

## References

- SEI CERT C Coding Standard — *Rules for Developing Safe, Reliable,
  and Secure Systems* (2016 Edition, Carnegie Mellon University)
  <https://wiki.sei.cmu.edu/confluence/display/c/>
- SEI CERT C++ Coding Standard — <https://wiki.sei.cmu.edu/confluence/display/cplusplus/>
- ECSS-Q-ST-80C — *Software product assurance* (security aspects)
- ESP-IDF Programming Guide — *FreeRTOS (IDF)* and *Concurrency*
  <https://docs.espressif.com/projects/esp-idf/>
- ARES Radio Protocol v2 — `src/comms/radio_protocol.h`
- ARES Assert — `src/ares_assert.h`

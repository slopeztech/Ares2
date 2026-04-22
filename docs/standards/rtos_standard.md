# ARES RTOS Standard

**Amateur Rocket Embedded System**
Real-time rules for ESP32-S3 dual-core SMP / IDF FreeRTOS,
companion to the [ARES Coding Standard](ares_coding_standard.md).

---

## Context

Deterministic, maintainable real-time behavior is mandatory. RTOS misuse
creates non-reproducible bugs that are extremely difficult to debug and
unsafe in flight systems.

ARES runs on an **ESP32-S3** — a dual-core Xtensa LX7 SoC with
symmetric multiprocessing (SMP). ESP-IDF FreeRTOS is based on
Vanilla FreeRTOS v10.5.1 but has significant modifications for
dual-core operation. These rules account for ESP32-S3 SMP behavior.

### ESP32-S3 SMP key facts

| Property                  | Detail                                     |
|---------------------------|--------------------------------------------|
| Cores                     | Core 0 (`PRO_CPU`) + Core 1 (`APP_CPU`)    |
| Scheduler                 | Per-core fixed-priority preemptive + SMP    |
| Tick source               | Core 0 only (keeps global time)            |
| Symmetric memory          | Yes (serialized by bus on simultaneous access) |
| Atomic operations         | Hardware compare-and-swap (ISA instruction) |
| Cross-core interrupt      | One core can trigger interrupt on the other |
| FPU                       | Single-precision only; `double` is software |
| Stack size unit (ESP-IDF) | **Bytes**, not words                       |

---

## RTOS-1 — Deterministic Execution

All tasks must have bounded, predictable execution time.

### RTOS-1.1 — Forbidden timing constructs

| Construct                    | Why Forbidden                         | Alternative                      |
|------------------------------|---------------------------------------|----------------------------------|
| `delay()`                    | Arduino busy-wait, blocks FreeRTOS    | `vTaskDelay(pdMS_TO_TICKS(ms))`  |
| `delayMicroseconds()`        | Busy-wait, starves lower-priority     | Hardware timer or `esp_timer`    |
| `while (!flag)` busy-wait   | CPU waste, blocks other tasks         | `xSemaphoreTake` / `xTaskNotifyWait` |
| `millis()` polling loop      | Non-deterministic, drift-prone        | `vTaskDelayUntil()`              |

### RTOS-1.2 — Bounded execution

- Every code path within a task iteration must have a worst-case
  execution time (WCET) that fits within the task period.
- All blocking operations must have a finite timeout (RTOS-8).
- No unbounded loops except RTOS task loops (see PO10-2).
- `configTICK_RATE_HZ` is 1000 (1 ms tick). Tasks must not assume
  sub-millisecond scheduling precision.

### RTOS-1.3 — Yield points

Every task loop iteration must contain at least one blocking RTOS
call that yields the CPU (`vTaskDelay`, `vTaskDelayUntil`,
`xQueueReceive`, `xSemaphoreTake`, `xTaskNotifyWait`). A task that
never yields will starve all equal-or-lower-priority tasks on the
same core and trigger the watchdog.

---

## RTOS-2 — Task Design

Each task has a **single responsibility**. Tasks must be small, simple,
and readable. Long or complex logic must be split into helper functions.
No "god tasks" handling multiple subsystems.

### RTOS-2.1 — Recommended task decomposition

| Task              | Responsibility               | Core  | Priority |
|-------------------|------------------------------|-------|----------|
| `sensor_task`     | Read and validate sensors    | 1     | High     |
| `control_task`    | FSM / flight logic           | 1     | High     |
| `radio_tx_task`   | Telemetry transmission       | 0     | Medium   |
| `radio_rx_task`   | Command reception            | 0     | Medium   |
| `gps_task`        | NMEA parsing from UART       | 0     | Medium   |
| `logging_task`    | Non-blocking data logging    | 1     | Low      |
| `led_task`        | Status LED heartbeat         | Any   | Low      |

Core assignments are recommendations. See RTOS-13 for affinity rules.

### RTOS-2.2 — Task function structure

```cpp
void sensorTask(void* param)
{
    // One-time init (sensor begin, calibrate)
    bool ok = imu.begin();
    ARES_ASSERT(ok);

    TickType_t lastWake = xTaskGetTickCount();

    while (true)  // RTOS: task loop — blocks on vTaskDelayUntil
    {
        SensorData data = readSensors();
        validateSensorData(data);
        xQueueSend(sensorQueue, &data, 0);

        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(10));
    }
}
```

Rules:
- Task function must never return.
- Init logic at the top, before the loop.
- Loop body should be ≤ 40 lines — extract helpers.
- One blocking call per loop (delay, queue receive, etc.).

---

## RTOS-3 — ISR Discipline

Interrupt Service Routines must be **minimal** — their only job is
to signal a task and get out.

### RTOS-3.1 — ISR rules

| Requirement                        | Detail                                    |
|------------------------------------|-------------------------------------------|
| Maximum ISR duration               | < 10 µs recommended                      |
| Must use `IRAM_ATTR`               | ISR code must be in internal RAM          |
| No blocking calls                  | No `vTaskDelay`, `xQueueReceive`, etc.    |
| No FreeRTOS API (non-FromISR)      | Only `...FromISR()` variants              |
| No `printf` / logging              | Too slow, may use heap                    |
| No memory allocation               | No `malloc`, `new`, `pvPortMalloc`        |
| No floating point                  | FPU state is tied to tasks, not ISRs      |

### RTOS-3.2 — ISR-to-task communication

ISRs must signal tasks via `FromISR` RTOS primitives:

```cpp
static portMUX_TYPE gpsMux = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR gpsPpsIsr()
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(ppsSemaphore, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
```

**Allowed `FromISR` functions:**

| Function                          | Purpose                    |
|-----------------------------------|----------------------------|
| `xQueueSendFromISR()`             | Send data to task          |
| `xQueueSendToBackFromISR()`       | Send data (back of queue)  |
| `xSemaphoreGiveFromISR()`         | Signal event               |
| `xTaskNotifyFromISR()`            | Lightweight signal         |
| `xTaskNotifyGiveFromISR()`        | Lightweight counting signal|
| `vRingbufferSendFromISR()`        | Send variable-size data    |
| `portYIELD_FROM_ISR()`            | Request context switch     |

### RTOS-3.3 — `portYIELD_FROM_ISR` discipline

Always call `portYIELD_FROM_ISR(xHigherPriorityTaskWoken)` at the
**end** of the ISR if any `FromISR` call set the flag to `pdTRUE`.
This ensures the scheduler immediately switches to the unblocked
higher-priority task instead of waiting for the next tick.

### RTOS-3.4 — Interrupt allocation

ESP32-S3 has limited high-priority interrupt slots. Use
`esp_intr_alloc()` with the lowest acceptable priority level.
Never use level 4+ (NMI) interrupts for application logic.

---

## RTOS-4 — Inter-Task Communication

Direct sharing of mutable global variables is **forbidden**. All
inter-task communication must use RTOS primitives.

### RTOS-4.1 — Primitive selection guide

| Primitive            | Use Case                                    | Thread-safe | ISR-safe  |
|----------------------|---------------------------------------------|-------------|-----------|
| Queue                | Typed data transfer between tasks           | Yes         | `FromISR` |
| Binary semaphore     | Event signaling (one-shot wake)             | Yes         | `FromISR` |
| Counting semaphore   | Resource counting / event counting          | Yes         | `FromISR` |
| Mutex                | Shared resource protection (**with priority inheritance**) | Yes | No  |
| Recursive mutex      | Re-entrant resource protection              | Yes         | No        |
| Event group          | Multi-bit event flags / task synchronization| Yes         | `FromISR` |
| Task notification    | Lightweight signal / counting semaphore     | Yes         | `FromISR` |
| Stream buffer        | Byte stream (single writer, single reader)  | 1:1 only    | `FromISR` |
| Message buffer       | Variable-length messages (single W, single R)| 1:1 only   | `FromISR` |
| Ring buffer (ESP-IDF)| Variable-size items, FIFO, DMA-friendly     | Yes         | `FromISR` |
| Spinlock (`portMUX_TYPE`) | Ultra-short critical sections (RTOS-14) | Yes       | Yes       |

### RTOS-4.2 — Queue best practices

```cpp
// ✅ Static queue — no runtime allocation (PO10-3)
static StaticQueue_t queueStorage;
static uint8_t queueBuffer[QUEUE_LEN * sizeof(SensorData)];
QueueHandle_t sensorQueue = xQueueCreateStatic(
    QUEUE_LEN, sizeof(SensorData), queueBuffer, &queueStorage);
```

- Queues copy data by value — pass structs, not pointers.
- Size the queue for worst-case burst: if a producer runs at 100 Hz
  and the consumer at 10 Hz, the queue must hold ≥ 10 items.
- Always check the return value of `xQueueSend` / `xQueueReceive`.
- If a queue is full, **do not block indefinitely** — use a bounded
  timeout and increment a drop counter.

### RTOS-4.3 — When to use what

| Scenario                            | Best Primitive          |
|--------------------------------------|-------------------------|
| Sensor data → processing task       | Queue                   |
| ISR signals "data ready"            | Binary semaphore or task notification |
| Protect SPI bus shared by 2 tasks   | Mutex                   |
| Multiple tasks wait for GPS fix     | Event group             |
| Byte stream from UART DMA          | Stream buffer or ring buffer |
| Simple "wake up" from ISR          | `xTaskNotifyGive`       |

- Polling is forbidden. Use event-driven design.

---

## RTOS-5 — Task Scheduling

Task priorities must be explicitly defined, documented, and centralized
in a single header or config file.

### RTOS-5.1 — Priority assignment

ESP-IDF FreeRTOS has `configMAX_PRIORITIES = 25` (0 = lowest,
24 = highest). ARES uses the following bands:

| Band       | Range  | Use Case                                     | Example Tasks             |
|------------|--------|----------------------------------------------|---------------------------|
| Critical   | 20–24  | **Reserved for ESP-IDF internals** (Wi-Fi, BT)| Do not use                |
| High       | 12–15  | Time-critical flight tasks                   | `sensor_task`, `control_task` |
| Medium     | 6–10   | Communication, non-safety processing         | `radio_tx_task`, `gps_task`   |
| Low        | 2–4    | Logging, LED, diagnostics                    | `logging_task`, `led_task`    |
| Idle       | 0–1    | **Reserved for FreeRTOS idle tasks**         | Do not use                |

### RTOS-5.2 — Priority rules

- Overuse of high-priority tasks is forbidden — at most 2–3 tasks
  at high priority.
- Two tasks at the same priority that are pinned to the same core
  will time-slice. If deterministic ordering is needed, use
  different priorities.
- On ESP32-S3 SMP, a lower-priority task on Core 1 can run
  **simultaneously** with a higher-priority task on Core 0. This
  is different from single-core FreeRTOS. Plan accordingly.
- Document all priority assignments in a centralized table
  (architecture doc or `config.h`).

### RTOS-5.3 — Priority definition pattern

```cpp
// config.h — centralized priorities
namespace ares::priority
{
    constexpr UBaseType_t SENSOR   = 14;
    constexpr UBaseType_t CONTROL  = 13;
    constexpr UBaseType_t RADIO_TX = 8;
    constexpr UBaseType_t RADIO_RX = 9;
    constexpr UBaseType_t GPS      = 7;
    constexpr UBaseType_t LOGGING  = 3;
    constexpr UBaseType_t LED      = 2;
}
```

---

## RTOS-6 — Periodic Tasks

Periodic tasks must use `vTaskDelayUntil()` for drift-free timing.
`vTaskDelay()` is only allowed for non-critical, approximate timing
(e.g., LED blink).

### RTOS-6.1 — Drift-free pattern

```cpp
void sensorTask(void* param)
{
    TickType_t lastWake = xTaskGetTickCount();

    while (true)  // RTOS: task loop
    {
        readAndValidateSensors();

        // Drift-free: compensates for task execution time
        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(10));
    }
}
```

### RTOS-6.2 — Deadline detection

`xTaskDelayUntil()` returns `pdTRUE` (via `xTaskDelayUntil`) if the
task was actually delayed. If the task already overran its period,
it returns immediately. Use this to detect missed deadlines:

```cpp
BaseType_t wasDelayed = xTaskDelayUntil(&lastWake, pdMS_TO_TICKS(10));
if (wasDelayed == pdFALSE)
{
    missedDeadlines++;
    // Log or assert — the task took longer than its period
}
```

### RTOS-6.3 — Recommended task rates

| Task            | Period       | Rationale                         |
|-----------------|--------------|-----------------------------------|
| `sensor_task`   | 10 ms (100 Hz) | IMU/baro sampling rate          |
| `control_task`  | 20 ms (50 Hz)  | FSM update, pyro logic          |
| `radio_tx_task` | 100 ms (10 Hz) | Telemetry frame rate            |
| `gps_task`      | Event-driven   | Wakes on UART data              |
| `logging_task`  | 50 ms (20 Hz)  | SD card write batching          |
| `led_task`      | 500 ms (2 Hz)  | Heartbeat visibility            |

---

## RTOS-7 — Memory Management

Dynamic memory allocation is **forbidden** after `setup()` completes
(see PO10-3).

### RTOS-7.1 — Static allocation

All RTOS objects must be statically allocated:

```cpp
// ✅ Static task
static StackType_t sensorStack[4096 / sizeof(StackType_t)];
static StaticTask_t sensorTcb;

TaskHandle_t sensorHandle = xTaskCreateStaticPinnedToCore(
    sensorTask, "sensor",
    sizeof(sensorStack),    // ESP-IDF: size in BYTES
    nullptr,
    ares::priority::SENSOR,
    sensorStack,
    &sensorTcb,
    1  // Core 1
);
```

### RTOS-7.2 — Stack sizing

ESP-IDF specifies stack sizes in **bytes** (not words like Vanilla
FreeRTOS).

| Task Type                | Recommended Stack | Notes                       |
|--------------------------|-------------------|-----------------------------|
| Simple I/O task          | 2048 bytes        | Minimal locals              |
| Sensor processing        | 4096 bytes        | Float arrays, filters       |
| Radio / protocol task    | 4096 bytes        | Frame buffers               |
| Logging (SD / Flash)     | 4096–8192 bytes   | File I/O, string formatting |
| Task using `printf`      | 4096+ bytes       | `printf` family uses ~1.5 KB|

### RTOS-7.3 — Stack overflow detection

Enable stack overflow detection in `sdkconfig` / `platformio.ini`:

```ini
# platformio.ini
build_flags =
    -DCONFIG_FREERTOS_CHECK_STACKOVERFLOW=2
```

Hook method 2 (pattern check) fills the stack with a known pattern
and detects corruption. It catches overflows more reliably than
method 1 (pointer check only).

Implement the overflow hook:

```cpp
extern "C" void vApplicationStackOverflowHook(
    TaskHandle_t xTask, char* pcTaskName)
{
    // Log task name, then reset
    esp_restart();
}
```

### RTOS-7.4 — Stack high-water mark monitoring

Periodically log high-water marks during development to right-size
stacks:

```cpp
UBaseType_t hwm = uxTaskGetStackHighWaterMark(nullptr);
// hwm is in BYTES (ESP-IDF). Values < 256 indicate danger.
```

Target: at least **25% free** stack at peak load. If high-water mark
drops below 512 bytes, increase the stack.

---

## RTOS-8 — Timeouts and Fail-Safe Behavior

All blocking calls must define a finite timeout.

### RTOS-8.1 — Timeout rules

| API                           | Timeout Pattern                              |
|-------------------------------|----------------------------------------------|
| `xQueueReceive`               | `pdMS_TO_TICKS(100)` — max 100 ms            |
| `xQueueSend`                  | `pdMS_TO_TICKS(10)` or `0` (non-blocking)    |
| `xSemaphoreTake` (mutex)      | `pdMS_TO_TICKS(50)`                           |
| `xSemaphoreTake` (binary sem) | `pdMS_TO_TICKS(100)`                          |
| `xTaskNotifyWait`             | `pdMS_TO_TICKS(200)`                          |
| UART `uart_read_bytes`        | `pdMS_TO_TICKS(100)`                          |
| SPI transaction               | Internal driver timeout                       |
| I2C transaction               | `pdMS_TO_TICKS(50)`                           |

### RTOS-8.2 — `portMAX_DELAY` policy

`portMAX_DELAY` is **forbidden** in flight-critical code. It is
allowed **only** in:

- Low-priority tasks that legitimately wait indefinitely for work
  (e.g., `logging_task` waiting on a queue — but even then, prefer
  a long finite timeout like 5000 ms so the task can check for
  shutdown signals).

### RTOS-8.3 — Timeout handling

On timeout, the system must:

1. Increment a timeout/error counter.
2. Continue operation if possible (degraded mode).
3. Enter safe state if the timeout indicates hardware failure.
4. **Never silently ignore** a timeout.

```cpp
if (xQueueReceive(sensorQueue, &data, pdMS_TO_TICKS(100)) == pdTRUE)
{
    processSensorData(data);
}
else
{
    sensorTimeouts++;
    if (sensorTimeouts >= MAX_CONSECUTIVE_TIMEOUTS)
    {
        enterSafeState(SafeReason::SENSOR_TIMEOUT);
    }
}
```

---

## RTOS-9 — Watchdog Integration

### RTOS-9.1 — ESP32-S3 Task Watchdog Timer (TWDT)

The ESP32-S3 has a Task Watchdog Timer (TWDT) that monitors tasks
for stalls. It is independent of the hardware watchdog (MWDT).

Configuration:

```ini
# platformio.ini
build_flags =
    -DCONFIG_ESP_TASK_WDT_EN=1
    -DCONFIG_ESP_TASK_WDT_TIMEOUT_S=5
    -DCONFIG_ESP_TASK_WDT_CHECK_IDLE_TASK_CPU0=1
    -DCONFIG_ESP_TASK_WDT_CHECK_IDLE_TASK_CPU1=1
```

### RTOS-9.2 — Subscribing tasks to TWDT

Critical tasks must subscribe to the TWDT and reset it each loop
iteration:

```cpp
#include "esp_task_wdt.h"

void sensorTask(void* param)
{
    // Subscribe this task to TWDT
    esp_task_wdt_add(nullptr);

    TickType_t lastWake = xTaskGetTickCount();
    while (true)
    {
        readSensors();

        // Feed the watchdog — proves this task is alive
        esp_task_wdt_reset();

        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(10));
    }
}
```

### RTOS-9.3 — Watchdog rules

- All high-priority tasks must be subscribed to the TWDT.
- TWDT timeout should be 2–5× the slowest critical task period.
- The TWDT panic handler (`CONFIG_ESP_TASK_WDT_PANIC`) should
  trigger a system reset in flight firmware.
- A stalled task must lead to a system reset — never ignore a
  watchdog timeout.

---

## RTOS-10 — No Hidden Coupling

- Tasks must not depend on implicit execution order.
- No assumptions about scheduler timing between tasks.
- All dependencies must be explicit via RTOS primitives
  (queues, semaphores, event groups, task notifications).

### RTOS-10.1 — Forbidden assumptions

| Assumption                                         | Why Wrong                                    |
|----------------------------------------------------|----------------------------------------------|
| "Task A always runs before Task B"                 | SMP: both may run simultaneously             |
| "This runs on Core 0, so no race"                  | Unpinned tasks can migrate                   |
| "Higher priority = runs first"                     | Only on same core; other core is independent |
| "Writes to `volatile` are atomic"                  | `volatile` ≠ thread-safe on multi-core       |
| "`vTaskSuspendAll()` prevents all other tasks"     | SMP: only suspends scheduler on current core |

### RTOS-10.2 — `volatile` is not enough

On a dual-core system, `volatile` only prevents compiler
optimizations. It does **not** provide memory ordering or atomicity
across cores. Use RTOS primitives or spinlocks (RTOS-14) for
shared data.

```cpp
// ❌ NOT thread-safe on SMP
volatile bool dataReady = false;

// ✅ Thread-safe signaling
xSemaphoreGive(dataReadySem);
```

---

## RTOS-11 — Logging and Debugging

Logging must **not** block execution of the calling task.

### RTOS-11.1 — Logging rules

- No logging inside time-critical sections or ISRs.
- Debug and monitoring tasks must run at low priority.
- Use queued/buffered logging: send log entries to a low-priority
  logging task via a queue.
- `ESP_LOG*` macros are acceptable during init but should be
  avoided in flight loops (they use `printf` internally).

### RTOS-11.2 — Runtime metrics

The system should expose runtime metrics during development:

| Metric                           | API                                      |
|----------------------------------|------------------------------------------|
| Per-task stack high-water mark   | `uxTaskGetStackHighWaterMark(handle)`    |
| Task list (state, priority, HWM) | `vTaskList(buffer)`                     |
| Per-task CPU time                | `vTaskGetRunTimeStats(buffer)`           |
| Queue fill level                 | `uxQueueMessagesWaiting(handle)`         |
| Queue free space                 | `uxQueueSpacesAvailable(handle)`         |
| Free heap (debug only)           | `esp_get_free_heap_size()`               |
| Missed deadlines                 | Custom counter per task                  |

### RTOS-11.3 — Debug build diagnostics task

In debug builds, a low-priority diagnostics task should periodically
print stack high-water marks and CPU load:

```cpp
#ifdef ARES_DEBUG
void diagnosticsTask(void* param)
{
    while (true)
    {
        static char buf[512];
        vTaskList(buf);
        ESP_LOGI("DIAG", "\n%s", buf);
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}
#endif
```

---

## RTOS-12 — Startup Synchronization

Tasks are created during `setup()`, but they may begin executing
before all subsystems are initialized. A barrier or synchronization
mechanism is required.

### RTOS-12.1 — Event group barrier pattern

Use an event group to signal that each subsystem has completed
initialization:

```cpp
// config.h
constexpr EventBits_t INIT_IMU    = (1 << 0);
constexpr EventBits_t INIT_BARO   = (1 << 1);
constexpr EventBits_t INIT_GPS    = (1 << 2);
constexpr EventBits_t INIT_RADIO  = (1 << 3);
constexpr EventBits_t INIT_ALL    = INIT_IMU | INIT_BARO
                                  | INIT_GPS | INIT_RADIO;

static StaticEventGroup_t initEventStorage;
EventGroupHandle_t initEvent = xEventGroupCreateStatic(&initEventStorage);
```

Each task signals its bit after successful init:

```cpp
void sensorTask(void* param)
{
    bool ok = imu.begin();
    ARES_ASSERT(ok);
    xEventGroupSetBits(initEvent, INIT_IMU);

    // Wait for ALL subsystems
    xEventGroupWaitBits(initEvent, INIT_ALL,
                        pdFALSE, pdTRUE,
                        pdMS_TO_TICKS(5000));
    // ... enter main loop
}
```

### RTOS-12.2 — Init timeout

If a subsystem fails to initialize within the timeout, the waiting
task should enter a safe/degraded state — never wait forever.

---

## RTOS-13 — Dual-Core Affinity

The ESP32-S3 has two cores. Tasks must have an explicit core affinity
policy.

### RTOS-13.1 — Core assignment philosophy

| Core   | Alias      | Preferred Tasks                              |
|--------|------------|----------------------------------------------|
| Core 0 | `PRO_CPU`  | Protocol/comms: radio, GPS, Wi-Fi, BT        |
| Core 1 | `APP_CPU`  | Application: sensors, FSM, control, pyros    |

Rationale: ESP-IDF protocol handlers (Wi-Fi, BT) run on Core 0
by default. Pinning flight-critical tasks to Core 1 isolates them
from protocol stack latency spikes.

### RTOS-13.2 — Task creation API

Always use `xTaskCreateStaticPinnedToCore()` (static allocation +
explicit core) instead of `xTaskCreate()` (dynamic + unpinned):

```cpp
// ✅ Correct: static, pinned, explicit core
TaskHandle_t handle = xTaskCreateStaticPinnedToCore(
    sensorTask,                   // function
    "sensor",                     // name
    sizeof(sensorStack),          // stack size (bytes)
    nullptr,                      // param
    ares::priority::SENSOR,       // priority
    sensorStack,                  // stack buffer
    &sensorTcb,                   // TCB buffer
    1                             // Core 1 (APP_CPU)
);
ARES_ASSERT(handle != nullptr);

// ❌ Forbidden: dynamic allocation, no core affinity
xTaskCreate(sensorTask, "sensor", 4096, nullptr, 5, nullptr);
```

### RTOS-13.3 — Unpinned tasks

`tskNO_AFFINITY` is allowed **only** for tasks that:

- Are stateless or communicate solely via queues.
- Do not use floating point (see RTOS-17).
- Are not time-critical.

Example: a low-priority LED heartbeat task.

### RTOS-13.4 — Core 0 caution

Do not pin high-frequency, time-critical tasks to Core 0 because:

- Core 0 handles the tick interrupt and global timekeeping.
- ESP-IDF system tasks (timer daemon, event loop) run on Core 0.
- Protocol stacks (Wi-Fi, BT) can cause latency spikes on Core 0.

If Core 0 is overloaded, the scheduler's tick count lags,
affecting all `vTaskDelayUntil()` timing system-wide.

---

## RTOS-14 — Critical Sections & Spinlocks

On ESP32-S3 SMP, disabling interrupts on one core does **not**
prevent the other core from accessing shared data. Critical sections
must use spinlocks.

### RTOS-14.1 — Spinlock API (ESP-IDF)

| API                                    | Context  | Purpose                         |
|----------------------------------------|----------|---------------------------------|
| `taskENTER_CRITICAL(&spinlock)`        | Task     | Enter critical section          |
| `taskEXIT_CRITICAL(&spinlock)`         | Task     | Exit critical section           |
| `taskENTER_CRITICAL_ISR(&spinlock)`    | ISR      | Enter critical section from ISR |
| `taskEXIT_CRITICAL_ISR(&spinlock)`     | ISR      | Exit critical section from ISR  |

Spinlocks are of type `portMUX_TYPE`:

```cpp
// Static initialization
static portMUX_TYPE mySpinlock = portMUX_INITIALIZER_UNLOCKED;

void updateSharedCounter()
{
    taskENTER_CRITICAL(&mySpinlock);
    sharedCounter++;            // safe on both cores
    taskEXIT_CRITICAL(&mySpinlock);
}
```

### RTOS-14.2 — Critical section rules

| Rule                                                  | Reason                               |
|-------------------------------------------------------|--------------------------------------|
| Keep critical sections **< 10 µs**                    | Interrupts are disabled inside       |
| No FreeRTOS API calls inside                          | May cause deadlock or crash          |
| No blocking or yielding inside                        | Interrupts are disabled              |
| No `printf`, logging, or I/O inside                   | Too slow, may block                  |
| One spinlock per shared resource                      | Avoid over-locking                   |
| Always pair ENTER/EXIT (same function)                | Mismatched calls corrupt IRQ state   |
| Beware deadlock with multiple spinlocks               | Always acquire in consistent order   |

### RTOS-14.3 — When to use which mechanism

| Mechanism                | Duration     | Cross-core | ISR-safe | Priority inheritance |
|--------------------------|--------------|------------|----------|---------------------|
| Spinlock (critical section) | < 10 µs   | Yes        | Yes      | No                  |
| Mutex                    | Milliseconds | Yes        | No       | Yes                 |
| Binary semaphore         | Signaling    | Yes        | Yes      | No                  |

- Use **spinlocks** for tiny read-modify-write operations
  (counters, flags, small struct copies).
- Use **mutexes** for longer operations (SPI transactions,
  multi-register I2C reads) where priority inheritance matters.
- Use **semaphores** for signaling, not mutual exclusion.

### RTOS-14.4 — `vTaskSuspendAll` is NOT mutual exclusion

On SMP, `vTaskSuspendAll()` only suspends the scheduler on the
**calling core**. The other core continues running. Never use
scheduler suspension for mutual exclusion:

```cpp
// ❌ NOT safe on SMP — other core still runs
vTaskSuspendAll();
sharedData++;
xTaskResumeAll();

// ✅ Safe on SMP
taskENTER_CRITICAL(&dataSpinlock);
sharedData++;
taskEXIT_CRITICAL(&dataSpinlock);
```

---

## RTOS-15 — Priority Inversion Prevention

Priority inversion occurs when a high-priority task is blocked
waiting for a resource held by a low-priority task, while a
medium-priority task preempts the low-priority task.

### RTOS-15.1 — Use mutexes, not binary semaphores

FreeRTOS **mutexes** implement priority inheritance: when a
high-priority task blocks on a mutex, the mutex-holding task's
priority is temporarily raised. Binary semaphores do **not** have
this mechanism.

| Scenario                           | Correct Primitive       |
|------------------------------------|-------------------------|
| Protecting a shared resource       | `xSemaphoreCreateMutex` |
| Signaling between task and ISR     | `xSemaphoreCreateBinary`|
| Counting events                    | `xSemaphoreCreateCounting` |

### RTOS-15.2 — Mutex discipline

- Acquire and release mutexes in the **same task**. Never give
  a mutex from a different task than the one that took it.
- Do not use mutexes from ISRs — mutexes are not ISR-safe.
- Keep mutex hold time as short as possible.
- If multiple mutexes are needed, always acquire them in the
  **same global order** to prevent deadlock.

```cpp
// ✅ Mutex for SPI bus protection
static SemaphoreHandle_t spiMutex;
static StaticSemaphore_t spiMutexStorage;

void init()
{
    spiMutex = xSemaphoreCreateMutexStatic(&spiMutexStorage);
}

bool spiTransfer(const uint8_t* tx, uint8_t* rx, size_t len)
{
    if (xSemaphoreTake(spiMutex, pdMS_TO_TICKS(50)) != pdTRUE)
    {
        return false;  // Timeout — SPI bus contention
    }

    spi_transaction(tx, rx, len);

    xSemaphoreGive(spiMutex);
    return true;
}
```

---

## RTOS-16 — Task Notifications

FreeRTOS task notifications are a lightweight alternative to
semaphores and event groups. They are faster (no kernel object
creation) and use less RAM.

### RTOS-16.1 — When to use task notifications

| Use Case                              | Task Notification API              |
|---------------------------------------|------------------------------------|
| Binary semaphore replacement          | `xTaskNotifyGive` / `ulTaskNotifyTake` |
| Counting semaphore replacement        | `xTaskNotifyGive` / `ulTaskNotifyTake` |
| Event flag group (32 bits)            | `xTaskNotify` / `xTaskNotifyWait`  |
| Mailbox (single 32-bit value)         | `xTaskNotify` with `eSetValueWithOverwrite` |

### RTOS-16.2 — Limitations

- A task notification can only have **one receiver** (the task
  that owns the notification). There is no broadcast.
- A task has a fixed array of notifications (default 1, can be
  increased via `configTASK_NOTIFICATION_ARRAY_ENTRIES`).
- Not suitable when multiple tasks must wait on the same event —
  use an event group for that.

### RTOS-16.3 — ISR-to-task lightweight signaling

```cpp
// ISR: lightweight signal to task
void IRAM_ATTR ppsIsr()
{
    BaseType_t woken = pdFALSE;
    vTaskNotifyGiveFromISR(gpsTaskHandle, &woken);
    portYIELD_FROM_ISR(woken);
}

// Task: wait for signal
void gpsTask(void* param)
{
    while (true)
    {
        // Block until ISR signals (acts like binary semaphore)
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(2000));
        processGpsPps();
    }
}
```

---

## RTOS-17 — FPU & Floating Point

The ESP32-S3 Xtensa LX7 has a single-precision FPU. Floating point
usage has RTOS implications.

### RTOS-17.1 — Auto-pinning

IDF FreeRTOS **automatically pins a task to its current core** the
first time the task uses a `float` operation. This is because FPU
registers are per-core and cannot be migrated.

Consequences:
- Any task that uses `float` will become pinned to whichever core
  it first runs on.
- This can cause **unexpected core affinity** for unpinned tasks.
- **Solution:** Explicitly pin all tasks that use `float` (which
  is most tasks in a flight computer). Do not rely on auto-pinning.

### RTOS-17.2 — No floating point in ISRs

FPU register state is tied to tasks. Using `float` in an ISR will
corrupt the interrupted task's FPU registers. This is **undefined
behavior**.

```cpp
// ❌ FORBIDDEN — float in ISR
void IRAM_ATTR imuIsr()
{
    float accel = readAccel();  // CORRUPTS FPU STATE
}

// ✅ Read raw integer, convert in task
void IRAM_ATTR imuIsr()
{
    int16_t raw = readAccelRaw();
    xQueueSendFromISR(rawQueue, &raw, &woken);
    portYIELD_FROM_ISR(woken);
}
```

### RTOS-17.3 — No `double`

The ESP32-S3 has **no double-precision FPU**. All `double` operations
are emulated in software, consuming ~10× more CPU cycles than `float`.
Use `float` exclusively (see MISRA-18).

---

## RTOS-18 — Task Lifecycle

### RTOS-18.1 — No runtime task creation

All tasks must be created during `setup()` / initialization. Creating
tasks at runtime risks heap fragmentation and makes the system's
timing behavior unpredictable.

### RTOS-18.2 — Task deletion policy

Task deletion (`vTaskDelete`) is **forbidden** in flight firmware:

| Concern                                    | Detail                                 |
|--------------------------------------------|----------------------------------------|
| Memory leak risk                           | Stack/TCB may not be freed immediately |
| Cross-core danger                          | Deleting a task running on another core can corrupt state |
| Mutex hazard                               | Deleted task may hold a mutex forever  |
| Non-deterministic                          | Idle task performs actual cleanup asynchronously |

If a task's work is complete, it should enter a permanent idle loop
with a long delay, or suspend itself:

```cpp
// ✅ Task no longer needed — self-suspend
vTaskSuspend(nullptr);
// Execution never reaches here unless resumed
```

### RTOS-18.3 — No task creation in ISRs

FreeRTOS does not support task creation from ISR context. This is
a hard API constraint.

---

## RTOS-19 — Anti-Patterns (Forbidden)

| Anti-Pattern                                          | Rule           |
|-------------------------------------------------------|----------------|
| `delay()` or `delayMicroseconds()` in tasks           | RTOS-1         |
| Busy-wait loops (`while (!flag)`)                     | RTOS-1         |
| Task loop without blocking call (CPU starvation)      | RTOS-1         |
| "God task" with multiple responsibilities             | RTOS-2         |
| Complex logic inside ISR                              | RTOS-3         |
| Non-`FromISR` API in ISR context                      | RTOS-3         |
| ISR handler without `IRAM_ATTR`                       | RTOS-3         |
| Floating point in ISR                                 | RTOS-17        |
| Mutable global without RTOS protection                | RTOS-4         |
| Binary semaphore for resource protection              | RTOS-15        |
| Blocking calls without timeout                        | RTOS-8         |
| `portMAX_DELAY` in flight-critical code               | RTOS-8         |
| Dynamic memory allocation after init                  | RTOS-7         |
| `xTaskCreate` (dynamic) instead of static             | RTOS-7, 13     |
| Task without explicit core affinity                   | RTOS-13        |
| `vTaskSuspendAll()` for mutual exclusion on SMP       | RTOS-14        |
| `volatile` as thread-safety mechanism                 | RTOS-10        |
| Priority inversion (semaphore instead of mutex)       | RTOS-15        |
| Multiple mutexes acquired in inconsistent order       | RTOS-15        |
| FreeRTOS API inside critical section                  | RTOS-14        |
| Task creation or deletion at runtime                  | RTOS-18        |
| `vTaskDelete` on task running on other core           | RTOS-18        |
| Tasks starting before all subsystems are initialized  | RTOS-12        |
| Ignoring watchdog subscription for critical tasks     | RTOS-9         |
| Using `double` instead of `float`                     | RTOS-17        |

---

## Cross-References

| RTOS Rule | Related ARES Rules                                         |
|-----------|------------------------------------------------------------|
| RTOS-1    | PO10-2 (bounded loops), DO-6 (determinism)                 |
| RTOS-2    | PO10-4 (short functions), DO-7 (modular design)            |
| RTOS-3    | CERT-1 (input validation), PO10-4 (short functions)        |
| RTOS-4    | PO10-6 (minimal scope), MISRA-4 (no shared mutable state) |
| RTOS-5    | DO-16 (partitioning), RTOS-13 (core affinity)              |
| RTOS-6    | PO10-2 (bounded loops)                                     |
| RTOS-7    | PO10-3 (no dynamic memory), MISRA-9 (static allocation)   |
| RTOS-8    | DO-6 (robustness), CERT-10 (timeout discipline)            |
| RTOS-9    | DO-6 (fail-safe), DO-9 (planning)                          |
| RTOS-10   | DO-7 (loose coupling)                                      |
| RTOS-11   | DO-5 (verification), PO10-10 (static analysis)             |
| RTOS-12   | DO-17 (transition criteria), RTOS-10 (no hidden coupling)  |
| RTOS-13   | DO-16 (partitioning), RTOS-17 (FPU pinning)                |
| RTOS-14   | MISRA-4 (thread safety), CERT-14 (concurrency)             |
| RTOS-15   | RTOS-4 (IPC primitives), RTOS-14 (spinlocks)               |
| RTOS-16   | RTOS-3 (ISR signaling), RTOS-4 (IPC)                       |
| RTOS-17   | MISRA-18 (float restrictions), PO10-3 (resource bounds)    |
| RTOS-18   | PO10-3 (no dynamic memory), RTOS-7 (static allocation)     |

---

## References

- ESP-IDF FreeRTOS (IDF) documentation:
  https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-reference/system/freertos_idf.html
- ESP-IDF FreeRTOS Supplemental Features:
  https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-reference/system/freertos_additions.html
- ESP-IDF Task Watchdog Timer:
  https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-reference/system/wdts.html
- FreeRTOS Kernel documentation:
  https://www.freertos.org/Documentation/02-Kernel/02-Kernel-features/00-Kernel-features
- Richard Barry, *Mastering the FreeRTOS Real Time Kernel*,
  FreeRTOS.org, 2016.

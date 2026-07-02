/**
 * @file  task.h
 * @brief Minimal FreeRTOS task-API stub for native and SITL builds.
 *
 * Provides only the symbols required by ARES firmware headers that include
 * <freertos/task.h>:
 *   - TaskHandle_t                   opaque pointer to a task control block
 *   - xTaskGetCurrentTaskHandle()    returns the "current task" handle
 *
 * The sim environment is inherently single-threaded, so there is exactly one
 * logical task.  xTaskGetCurrentTaskHandle() always returns the address of a
 * file-scope sentinel object — a stable, non-null, unique value — so that
 * same-task assertions (e.g. ARES_ASSERT(xTaskGetCurrentTaskHandle() == owner_))
 * always pass in SITL while still compiling and exercising the assertion path
 * in production debug builds.
 */
#pragma once

#include "FreeRTOS.h"

// ── Task handle type ─────────────────────────────────────────────────────────

/// Opaque task-control-block pointer.  void* matches the real FreeRTOS ABI.
using TaskHandle_t = void*;

// ── Current-task query ────────────────────────────────────────────────────────

/**
 * Return the handle of the currently running task.
 *
 * In the single-threaded sim there is one logical task.  The function returns
 * the address of a static sentinel so the value is:
 *   - non-null  (safe to compare against an uninitialised nullptr handle)
 *   - stable    (same address on every call within a process lifetime)
 *   - unique    (distinct from any unrelated pointer)
 *
 * All calls within the same process therefore return the same handle, which
 * means that ARES_ASSERT(xTaskGetCurrentTaskHandle() == ownerTask_) passes
 * whenever ownerTask_ was set by the same process — the intended behaviour
 * for single-threaded simulation.
 */
inline TaskHandle_t xTaskGetCurrentTaskHandle()
{
    static int kSimTaskSentinel = 0;
    return static_cast<TaskHandle_t>(&kSimTaskSentinel);
}

// ── Static task creation ──────────────────────────────────────────────────────

/**
 * Stub for xTaskCreateStaticPinnedToCore.
 *
 * begin() calls this but tests never call begin(), so the stub just
 * returns the sentinel handle to satisfy the linker without starting
 * an actual RTOS task.
 */
inline TaskHandle_t xTaskCreateStaticPinnedToCore(
    void (*/*pxTaskCode*/)(void*),
    const char* /*pcName*/,
    uint32_t    /*ulStackDepth*/,
    void*       /*pvParameters*/,
    UBaseType_t /*uxPriority*/,
    StackType_t*  /*puxStackBuffer*/,
    StaticTask_t* /*pxTaskBuffer*/,
    BaseType_t    /*xCoreID*/)
{
    return xTaskGetCurrentTaskHandle();
}

// ── Stack monitoring ──────────────────────────────────────────────────────────

/**
 * Return a plausible high-water mark so the stack-monitor log path in
 * run() compiles and exercises the log branch without triggering the
 * low-water warning.
 */
inline UBaseType_t uxTaskGetStackHighWaterMark(TaskHandle_t /*task*/)
{
    return static_cast<UBaseType_t>(8192U);
}

/**
 * Stub for ulTaskNotifyTake.
 *
 * In single-threaded sim there is no RTOS scheduler; return immediately as if
 * one notification token had been consumed so worker loops can progress.
 */
inline uint32_t ulTaskNotifyTake(BaseType_t /*xClearCountOnExit*/, TickType_t /*xTicksToWait*/)
{
    return 1U;
}

/**
 * Stub for xTaskNotifyGive.
 *
 * Notification delivery is a no-op in single-threaded sim; return pdTRUE to
 * match a successful post.
 */
inline BaseType_t xTaskNotifyGive(TaskHandle_t /*xTaskToNotify*/)
{
    return pdTRUE;
}

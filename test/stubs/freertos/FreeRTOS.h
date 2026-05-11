/**
 * @file  FreeRTOS.h
 * @brief Minimal FreeRTOS stub for native (host) test and SITL builds.
 *
 * Provides only the symbols required by ARES firmware when compiled
 * without the FreeRTOS kernel.  The sim environment is inherently
 * single-threaded, so mutex operations are no-ops that always succeed.
 *
 * Thread safety: N/A — host process is single-threaded.
 */
#pragma once

#include <cstdint>
#include <cstdlib>

// ── Primitive types ─────────────────────────────────────────────────────────

using BaseType_t  = int32_t;
using UBaseType_t = uint32_t;
using TickType_t  = uint32_t;

// ── Return codes ────────────────────────────────────────────────────────────

static constexpr BaseType_t pdTRUE  = 1;
static constexpr BaseType_t pdFALSE = 0;
static constexpr BaseType_t pdPASS  = pdTRUE;
static constexpr BaseType_t pdFAIL  = pdFALSE;

static constexpr TickType_t portMAX_DELAY = static_cast<TickType_t>(0xFFFFFFFFUL);

// ── Tick conversion ─────────────────────────────────────────────────────────

/** In the sim, 1 ms == 1 tick (no scaling). */
#define pdMS_TO_TICKS(ms) (static_cast<TickType_t>(ms))

// ── Semaphore types (opaque in real FreeRTOS; concrete here) ────────────────

struct StaticSemaphore_t
{
    bool locked = false;
};

using SemaphoreHandle_t = StaticSemaphore_t*;

// ── Task delay (no-op in single-threaded sim) ────────────────────────────────

inline void vTaskDelay(TickType_t /*xTicksToDelay*/) {}

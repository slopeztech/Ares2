/**
 * @file  semphr.h
 * @brief Minimal FreeRTOS semaphore/mutex stub for native SITL builds.
 *
 * In a single-threaded host process all mutex operations trivially succeed.
 * xSemaphoreCreateMutexStatic initialises the StaticSemaphore_t in-place
 * and returns a pointer to it.  Take/Give toggle the `locked` flag, which
 * allows ARES_ASSERT inside ScopedLock to detect double-take bugs even on
 * host.
 *
 * Thread safety: N/A — single-threaded sim.
 */
#pragma once

#include "FreeRTOS.h"

// ── Mutex creation ───────────────────────────────────────────────────────────

/**
 * Initialise @p pxStaticSemaphore and return a handle to it.
 * Mirrors xSemaphoreCreateMutexStatic() from FreeRTOS.
 */
inline SemaphoreHandle_t xSemaphoreCreateMutexStatic(StaticSemaphore_t* pxStaticSemaphore)
{
    if (pxStaticSemaphore == nullptr) { return nullptr; }
    pxStaticSemaphore->locked = false;
    return pxStaticSemaphore;
}

// ── Take / Give ──────────────────────────────────────────────────────────────

/**
 * Acquire the mutex.  Always succeeds in the single-threaded sim.
 * Returns pdTRUE if the semaphore was not already held; pdFALSE if it
 * was already locked (mirrors a "would block" scenario).
 */
inline BaseType_t xSemaphoreTake(SemaphoreHandle_t xSemaphore, TickType_t /*xBlockTime*/)
{
    if (xSemaphore == nullptr)        { return pdFALSE; }
    if (xSemaphore->locked)           { return pdFALSE; }
    xSemaphore->locked = true;
    return pdTRUE;
}

/**
 * Release the mutex.  Returns pdTRUE on success, pdFALSE if the
 * semaphore was not held (mirrors FreeRTOS behaviour).
 */
inline BaseType_t xSemaphoreGive(SemaphoreHandle_t xSemaphore)
{
    if (xSemaphore == nullptr)  { return pdFALSE; }
    if (!xSemaphore->locked)    { return pdFALSE; }
    xSemaphore->locked = false;
    return pdTRUE;
}

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
    pxStaticSemaphore->locked        = false;
    pxStaticSemaphore->takeFailCount = 0U;
    return pxStaticSemaphore;
}

// ── Take / Give ──────────────────────────────────────────────────────────────

/**
 * Acquire the mutex.
 *
 * **Sim limitation — timeout ignored**: @p xBlockTime is silently discarded.
 * In the real FreeRTOS kernel a contended Take would block until the mutex is
 * released or the timeout expires.  Here the call returns immediately:
 *
 *   - pdTRUE  — mutex was free; it is now held.
 *   - pdFALSE — mutex was already held (contention); @c takeFailCount is
 *               incremented so tests can assert that no unexpected contention
 *               occurred.  A production timeout-sensitive path would block
 *               here; the sim will not, masking that behaviour.
 *
 * Tests that need to verify locking discipline should check
 * @c xSemaphore->takeFailCount == 0 after exercising the code under test.
 * Use xSemaphoreResetFailCount() between test cases to clear the counter.
 */
inline BaseType_t xSemaphoreTake(SemaphoreHandle_t xSemaphore, TickType_t /*xBlockTime*/)
{
    if (xSemaphore == nullptr) { return pdFALSE; }
    if (xSemaphore->locked)
    {
        xSemaphore->takeFailCount++;
        return pdFALSE;
    }
    xSemaphore->locked = true;
    return pdTRUE;
}

// ── Sim-only helpers ─────────────────────────────────────────────────────────

/**
 * Reset the contention counter to zero.  Call between test cases so that
 * each case gets a clean baseline for `takeFailCount` assertions.
 */
inline void xSemaphoreResetFailCount(SemaphoreHandle_t xSemaphore)
{
    if (xSemaphore != nullptr) { xSemaphore->takeFailCount = 0U; }
}

// ── Give ──────────────────────────────────────────────────────────────────────

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

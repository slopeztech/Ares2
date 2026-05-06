/**
 * @file  rtos_guard.h
 * @brief RAII mutex guard for FreeRTOS SemaphoreHandle_t.
 *
 * Provides ScopedLock — a lightweight RAII wrapper that acquires a
 * FreeRTOS semaphore in the constructor and releases it in the destructor.
 * Prevents lock leaks on early-return paths (CERT-18.1, CERT-18.4, RTOS-4).
 *
 * This header is the canonical location for ScopedLock so it can be shared
 * across the API layer, hardware drivers, and any other FreeRTOS code
 * without creating inter-module coupling.
 */
#pragma once

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#include "ares_assert.h"

/**
 * RAII mutex guard with bounded timeout.
 *
 * Automatically releases the mutex on destruction, preventing
 * lock leaks on early-return paths (CERT-18.4).
 *
 * @code
 * ScopedLock lk(myMutex_, pdMS_TO_TICKS(100));
 * if (!lk.acquired()) { return false; }
 * // ... critical section ...
 * @endcode
 */
class ScopedLock
{
public:
    explicit ScopedLock(SemaphoreHandle_t m, TickType_t timeout)
        : mutex_(m), acquired_(false)
    {
        ARES_ASSERT(m != nullptr);
        acquired_ = (xSemaphoreTake(m, timeout) == pdTRUE);
    }

    ~ScopedLock()
    {
        if (acquired_)
        {
            // FreeRTOS macro internals trigger cppcheck dangerousTypeCast at call site.
            // cppcheck-suppress dangerousTypeCast
            const BaseType_t giveOk = static_cast<BaseType_t>(xSemaphoreGive(mutex_));
            ARES_ASSERT(giveOk == pdTRUE);
        }
    }

    bool acquired() const { return acquired_; }

    ScopedLock(const ScopedLock&)            = delete;
    ScopedLock& operator=(const ScopedLock&) = delete;

private:
    SemaphoreHandle_t mutex_;
    bool acquired_;
};

/**
 * @file  api_common.h
 * @brief Shared utilities for the ARES REST API layer.
 *
 * Provides the ScopedLock RAII guard, CORS headers, and HTTP
 * parsing constants used by the API server core and all
 * route-handler subdirectories.
 */
#pragma once

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#include "ares_assert.h"

// ── CORS headers (REST-7) ───────────────────────────────────
inline constexpr const char* CORS_HEADERS =
    "Access-Control-Allow-Origin: *\r\n"
    "Access-Control-Allow-Methods: GET, PUT, POST, DELETE, OPTIONS\r\n"
    "Access-Control-Allow-Headers: Content-Type\r\n";

// ── HTTP parsing constants (MISRA-7) ────────────────────────
static constexpr uint16_t HTTP_LINE_MAX       = 256;  ///< Max HTTP request line.
static constexpr uint16_t HTTP_HEADER_MAX     = 256;  ///< Max header line.
static constexpr uint8_t  HTTP_MAX_HEADERS    = 16;   ///< Max headers to parse (REST-3).
static constexpr uint16_t HTTP_METHOD_MAX     = 8;    ///< "OPTIONS" + null.
static constexpr uint8_t  HTTP_PATH_MAX       = 64;   ///< URL path buffer (REST-1.1).

// ── Scoped mutex lock (CERT-18.1, RTOS-4) ───────────────────

/**
 * RAII mutex guard with bounded timeout.
 *
 * Automatically releases the mutex on destruction, preventing
 * lock leaks on early-return paths (CERT-18.4).
 * Shared across API server core and all handler modules.
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

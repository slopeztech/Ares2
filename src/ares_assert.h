/**
 * @file  ares_assert.h
 * @brief ARES runtime assertion macro (PO10-5).
 *
 * Two modes controlled by `ARES_NDEBUG`:
 *   - **Debug** (default): evaluates the condition; on failure logs
 *     file/line/expression via LOG_E, flushes the UART so the
 *     message reaches the console, then calls abort().
 *   - **Release** (`-DARES_NDEBUG`): expands to a no-op so
 *     assertions have zero cost in flight firmware.
 *
 * @warning Do NOT use ARES_ASSERT on external/untrusted data
 *          (e.g. NMEA fields, radio packets).  Use explicit
 *          validation with error returns instead (CERT-21.3).
 */
#pragma once

#include "debug/ares_log.h"

#ifndef ARES_NDEBUG

/**
 * @def ARES_ASSERT(cond)
 * Abort with a log message if @p cond is false (debug builds only).
 *
 * Serial.flush() is called before abort() to guarantee the error
 * message is fully transmitted over USB-CDC, which is buffered and
 * would otherwise be lost when the CPU halts.
 */
#define ARES_ASSERT(cond)                                           \
    do {                                                            \
        if (!(cond)) {                                              \
            LOG_E("ASSERT", "%s:%d %s", __FILE__, __LINE__, #cond); \
            Serial.flush();                                         \
            ::abort();                                              \
        }                                                           \
    } while (0)

#else

#define ARES_ASSERT(cond) ((void)0)

#endif

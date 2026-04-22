/**
 * @file  ares_log.h
 * @brief Structured logging with compile-time and runtime severity filtering.
 *
 * Logging uses a two-stage gate:
 *   1. **Compile-time** (`ARES_LOG_LEVEL` macro) — the preprocessor
 *      strips out every macro below the chosen level, producing
 *      zero code for disabled severities.
 *   2. **Runtime** (`setLevel()`) — allows dynamic adjustment
 *      (e.g. enabling DEBUG over a radio command) without
 *      recompiling, as long as the compile-time gate includes it.
 *
 * Output format:  `[  uptime] L TAG: message\n`
 *   - uptime: millis(), right-aligned 7 digits.
 *   - L: single char severity (E/W/I/D).
 *   - TAG: caller-supplied subsystem identifier.
 *
 * Severity levels follow RFC 5424 (syslog), restricted to the
 * four levels relevant for embedded systems.
 *
 * Thread safety: emit() writes to a file-scope static buffer.
 *                It is NOT thread-safe.  In an RTOS build, callers
 *                from different tasks must serialise access or
 *                route through a dedicated logging task (CERT-13).
 */
#pragma once

#include <Arduino.h>

#include <cstdarg>

// ── Severity levels (RFC 5424 subset, relevant for embedded) ──
//
//   E  Error      — unrecoverable failure, system may abort
//   W  Warning    — anomaly, operation continues with degradation
//   I  Info       — significant events (phase transitions, init)
//   D  Debug      — development diagnostics, high-frequency data
//
// Compile-time filter: define ARES_LOG_LEVEL before including this
// header to suppress lower-priority messages entirely.
//   0 = none, 1 = E only, 2 = +W, 3 = +I (default), 4 = +D

#ifndef ARES_LOG_LEVEL
#define ARES_LOG_LEVEL 3
#endif

namespace ares
{
namespace log
{

/// Log severity levels (RFC 5424 subset).
enum Level : uint8_t
{
    NONE  = 0,   ///< All output suppressed.
    ERROR = 1,   ///< Unrecoverable failure — system may abort.
    WARN  = 2,   ///< Anomaly — operation continues with degradation.
    INFO  = 3,   ///< Significant event (init, phase transition).
    DEBUG = 4,   ///< High-frequency development diagnostics.
};

/// Set the runtime severity threshold.
/// Messages below @p level are silently discarded.
/// @param[in] level  Minimum severity to emit.
void     setLevel(Level level);

/// @return Current runtime severity threshold.
Level    getLevel();

/**
 * Core formatted-print function.
 *
 * Not intended for direct use — prefer the LOG_E / LOG_W /
 * LOG_I / LOG_D inline wrappers, which add compile-time gating.
 *
 * @param[in] levelChar  Single character identifying severity (E/W/I/D).
 * @param[in] tag        Subsystem identifier (e.g. "FSM", "GPS").
 * @param[in] fmt        printf-style format string.
 */
void     emit(char levelChar, const char* tag, const char* fmt, ...);

/**
 * Same as emit(), but receives a pre-built va_list.
 */
void     emitV(char levelChar, const char* tag, const char* fmt, va_list args);

} // namespace log
} // namespace ares

// ── Inline log wrappers (PO10-8: no variadic macros) ────────

inline void LOG_E(const char* tag, const char* fmt, ...)
{
#if ARES_LOG_LEVEL >= 1
    if (ares::log::getLevel() >= ares::log::ERROR)
    {
        va_list args;
        va_start(args, fmt);
        ares::log::emitV('E', tag, fmt, args);
        va_end(args);
    }
#else
    (void)tag;
    (void)fmt;
#endif
}

inline void LOG_W(const char* tag, const char* fmt, ...)
{
#if ARES_LOG_LEVEL >= 2
    if (ares::log::getLevel() >= ares::log::WARN)
    {
        va_list args;
        va_start(args, fmt);
        ares::log::emitV('W', tag, fmt, args);
        va_end(args);
    }
#else
    (void)tag;
    (void)fmt;
#endif
}

inline void LOG_I(const char* tag, const char* fmt, ...)
{
#if ARES_LOG_LEVEL >= 3
    if (ares::log::getLevel() >= ares::log::INFO)
    {
        va_list args;
        va_start(args, fmt);
        ares::log::emitV('I', tag, fmt, args);
        va_end(args);
    }
#else
    (void)tag;
    (void)fmt;
#endif
}

inline void LOG_D(const char* tag, const char* fmt, ...)
{
#if ARES_LOG_LEVEL >= 4
    if (ares::log::getLevel() >= ares::log::DEBUG)
    {
        va_list args;
        va_start(args, fmt);
        ares::log::emitV('D', tag, fmt, args);
        va_end(args);
    }
#else
    (void)tag;
    (void)fmt;
#endif
}

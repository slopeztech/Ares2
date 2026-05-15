/**
 * @file  Arduino.h
 * @brief Minimal Arduino stub for native (host) unit-test builds.
 *
 * Provides only the symbols that ARES firmware headers pull in from
 * <Arduino.h> when compiled without the Arduino framework:
 *   - millis()      used by ares_log.cpp for the timestamp header
 *   - Serial        used by ares_log.cpp (print) and ares_assert.h (flush)
 *
 * Nothing here is intended to be behaviourally correct — it exists only
 * to satisfy the linker when running `pio test -e native`.
 */
#pragma once

#include <cstdint>
#include <cstdio>

/// Returns the current simulation time so sensor-cache age checks inside the
/// AMS engine work correctly in SITL builds.  In other native test builds (no
/// sim clock available) returns 0 — the logger is the only caller there and
/// uses the value only for display, not for logic.
#ifdef ARES_SITL
#include "sim_clock.h"
inline uint32_t millis()   { return ares::sim::clock::nowMs(); }
inline uint64_t millis64() { return static_cast<uint64_t>(ares::sim::clock::nowMs()); }
#else
inline uint32_t millis()   { return 0U; }
inline uint64_t millis64() { return 0ULL; }
#endif

/// Minimal Serial stub: routes output to stdout so log lines are visible
/// during a test run.
struct HardwareSerialStub
{
    int print(const char* s)
    {
        if (s != nullptr) { (void)fputs(s, stdout); }
        return 0;
    }

    void flush() { (void)fflush(stdout); }
};

// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
static HardwareSerialStub Serial;

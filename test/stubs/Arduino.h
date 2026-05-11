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

/// Returns 0.  The logger uses it only for display, not for logic.
inline uint32_t millis() { return 0U; }

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

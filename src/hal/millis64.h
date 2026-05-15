/**
 * @file  millis64.h
 * @brief Platform-adaptive 64-bit millisecond counter.
 *
 * millis64() returns the number of milliseconds elapsed since boot as an
 * unsigned 64-bit integer.  Unlike the Arduino millis() which returns a
 * uint32_t (overflows at ~49.7 days), millis64() will not overflow for
 * approximately 585 million years.
 *
 * On ESP32 firmware builds (ARDUINO defined) this wraps esp_timer_get_time(),
 * which is the same source used by the Arduino millis() implementation.
 * On native (host) test builds the Arduino.h stub provides millis(), and
 * millis64() is a simple promotion of that 32-bit value.
 *
 * Usage: replace millis() with millis64() in any code path that stores
 * absolute timestamps in member variables that survive across ticks.
 *
 * @note The TelemetryPayload.timestampMs wire field remains uint32_t for
 *       on-air protocol compatibility; cast millis64() to uint32_t when
 *       populating that field.
 */
#pragma once

#include <cstdint>

#ifdef ARDUINO
// ── Firmware (ESP32 Arduino) ─────────────────────────────────────────────────
// esp_timer_get_time() returns int64_t microseconds since boot.
// Divide by 1000 to get milliseconds; reinterpret as uint64_t since the
// timer never returns a negative value in practice.
#  include <esp_timer.h>
inline uint64_t millis64()
{
    return static_cast<uint64_t>(esp_timer_get_time()) / 1000ULL;
}
#else
// ── Native / host test builds ────────────────────────────────────────────────
// millis64() is provided by test/stubs/Arduino.h (either sim clock or 0).
// Just include the stub; do NOT redefine the function here.
#  include <Arduino.h>
#endif

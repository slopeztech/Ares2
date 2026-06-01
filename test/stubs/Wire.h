/**
 * @file  Wire.h
 * @brief TwoWire (I2C) stub for native / sim test builds.
 *
 * Provides a virtual-method base class so test suites can derive concrete
 * scripted subclasses (e.g. for BMP280 driver tests) without changing
 * production code.
 *
 * All base-class methods are safe no-ops:
 *   - write() / endTransmission() return success codes.
 *   - requestFrom() / read() return 0 / -1 (bus empty).
 *
 * api_routing tests use a null TwoWire* that is never dereferenced, so the
 * change from non-virtual to virtual is backward-compatible.
 *
 * delay() is declared here (not in Arduino.h) because bmp280_driver.cpp
 * includes <Wire.h> (via bmp280_driver.h) and calls delay() from the
 * Arduino framework.  The stub provides a safe no-op.
 */
#pragma once

#include <cstdint>

// Arduino framework global — no-op in host builds.
inline void delay(uint32_t /*ms*/) {}

/**
 * @brief Minimal I2C bus stub.
 *
 * All methods are virtual so test suites can subclass and script I2C
 * responses without modifying production code.
 */
class TwoWire
{
public:
    virtual ~TwoWire() = default;

    virtual void    beginTransmission(uint8_t /*addr*/)            {}
    virtual int     write(uint8_t /*data*/)                        { return 1; }
    virtual uint8_t endTransmission()                              { return 1U; } ///< 1 = NACK (no device).
    virtual uint8_t endTransmission(bool /*stop*/)                 { return 0U; } ///< 0 = success (repeated start).
    virtual uint8_t requestFrom(uint8_t /*addr*/, uint8_t /*len*/) { return 0U; }
    virtual int     read()                                         { return -1; }
};

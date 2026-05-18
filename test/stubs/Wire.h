/**
 * @file  Wire.h
 * @brief Minimal TwoWire (I2C) stub for native api_routing test builds.
 *
 * bus_scan_handler.cpp includes <Wire.h> and calls TwoWire::beginTransmission()
 * and TwoWire::endTransmission() on non-null bus pointers.  In the routing
 * tests both i2c0_ and i2c1_ are nullptr so handleI2cScan() returns 500
 * before reaching those calls, but the type must be complete to compile.
 *
 * endTransmission() returns 1 (NACK / no device) so if a bus were ever
 * exercised no false-positive device detections would occur.
 */
#pragma once

#include <cstdint>

/**
 * @brief Minimal I2C bus stub.
 *
 * Only the two methods called by scanI2cBus() are provided.
 */
class TwoWire
{
public:
    void    beginTransmission(uint8_t /*addr*/) {}
    uint8_t endTransmission()  { return 1U; }  ///< 1 = no device (NACK).
};

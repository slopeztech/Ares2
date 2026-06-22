/**
 * @file  serial_interface.h
 * @brief Hardware-agnostic serial output interface (pure virtual).
 *
 * Provides a minimal byte-stream abstraction for AMS SERIAL.report output.
 * Implementations wrap the platform UART/CDC backend.
 */
#pragma once

#include <cstdint>

/**
 * Abstract serial output interface.
 *
 * Designed for bounded, non-blocking telemetry text output where callers need
 * to inspect TX capacity before writing.
 */
class SerialInterface
{
public:
    virtual ~SerialInterface() = default;

    /**
     * Query currently available TX buffer space.
     * @return Number of bytes that can be accepted without blocking.
     */
    virtual uint32_t availableForWrite() const = 0;

    /**
     * Write a raw byte block.
     * @param[in] data  Source buffer.
     * @param[in] len   Number of bytes to write.
     * @return Number of bytes accepted by the backend.
     */
    virtual uint32_t write(const uint8_t* data, uint32_t len) = 0;
};
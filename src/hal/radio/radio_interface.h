/**
 * @file  radio_interface.h
 * @brief Hardware-agnostic radio transceiver interface (pure virtual).
 *
 * Provides a transport-layer abstraction for packet radio modules.
 * Concrete drivers (e.g. DxLr03Driver) implement this interface.
 * Application code accesses the radio through a reference to this
 * interface, never through the concrete driver directly.
 *
 * Thread safety: Implementations MUST be callable from the comms
 *                task only.  Multi-task access requires external
 *                synchronisation (CERT-13).
 */
#pragma once

#include <cstdint>

/**
 * Status codes returned by RadioInterface operations.
 */
enum class RadioStatus : uint8_t
{
    OK         = 0,   ///< Operation succeeded.
    ERROR      = 1,   ///< Hardware or I/O error.
    NOT_READY  = 2,   ///< Module not initialised.
    BUSY       = 3,   ///< Module busy (TX in progress / AUX low).
    TIMEOUT    = 4,   ///< Operation timed out.
    OVERFLOW   = 5,   ///< Data exceeds module MTU.

    FIRST = OK,            // CERT-6.1 — range validation sentinels
    LAST  = OVERFLOW
};

/**
 * Abstract radio transceiver interface.
 *
 * Designed for UART-based packet radio modules (LoRa, FSK, etc.)
 * but generic enough for SPI-based radios.  The interface is
 * payload-agnostic — framing and CRC are handled by the protocol
 * layer above (see ares_radio_protocol.h, APUS-1/4).
 *
 * Usage pattern:
 *   1. begin()   — initialise hardware and enter RX mode.
 *   2. send()    — transmit a raw byte buffer (blocks until done).
 *   3. receive() — poll for incoming bytes (non-blocking).
 *   4. ready()   — check if the module is idle (AUX-based or SW).
 */
class RadioInterface
{
public:
    virtual ~RadioInterface() = default;

    /**
     * Initialise the radio hardware.
     * @pre  UART/SPI bus must be available.
     * @post Module is in normal receive mode on success.
     * @return true on success, false on hardware failure.
     */
    virtual bool begin() = 0;

    /**
     * Transmit a raw byte buffer.
     * @param[in] data  Buffer to transmit (caller-owned).
     * @param[in] len   Number of bytes to transmit.
     * @return Status code (see RadioStatus).
     *
     * @note Blocks until the module accepts all bytes or an
     *       error/timeout occurs.  The module may still be
     *       transmitting on-air when this returns OK.
     */
    virtual RadioStatus send(const uint8_t* data, uint16_t len) = 0;

    /**
     * Receive available bytes from the module.
     * @param[out] buf       Destination buffer (caller-owned).
     * @param[in]  bufSize   Size of @p buf in bytes.
     * @param[out] received  Number of bytes actually read (≤ bufSize).
     * @return Status code.  OK even if received == 0 (no data yet).
     *
     * @note Non-blocking — returns immediately with whatever bytes
     *       are available in the UART FIFO / internal buffer.
     */
    virtual RadioStatus receive(uint8_t* buf, uint16_t bufSize,
                                uint16_t& received) = 0;

    /**
     * Check whether the module is ready (idle, not transmitting).
     * @return true if the module can accept a new send().
     *
     * For modules with an AUX pin, this checks AUX == HIGH.
     * For software-only modules, returns true if TX is complete.
     */
    virtual bool ready() const = 0;

    /**
     * Maximum transmission unit — max bytes per single send().
     * @return Module-specific MTU in bytes.
     */
    virtual uint16_t mtu() const = 0;
};

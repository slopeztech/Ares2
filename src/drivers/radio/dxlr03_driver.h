/**
 * @file  dxlr03_driver.h
 * @brief DX-LR03-433T30D LoRa UART transceiver driver.
 *
 * Drives a DX-LR03 in NORMAL (transparent) mode via UART.
 * M0/M1 pins are not wired — the module defaults to NORMAL mode
 * (mode 00) via internal pull-down resistors.
 *
 * AUX pin handling:
 *   The AUX pin is *advisory*.  During begin() the driver probes
 *   AUX: if it goes HIGH within AUX_TIMEOUT_MS, AUX-based flow
 *   control is enabled; otherwise it is disabled and the driver
 *   sends unconditionally.  Some DX-LR03 units (or wiring configs)
 *   hold AUX permanently LOW yet transmit/receive correctly —
 *   confirmed during HW bring-up 2026-04-20.
 *
 * Thread safety: NOT thread-safe.  Must be accessed from the comms
 *                task only (CERT-13).
 */
#pragma once

#include <Arduino.h>
#include "config.h"

#include "hal/radio/radio_interface.h"

/**
 * Concrete RadioInterface for the DX-LR03-433T30D.
 *
 * Operates exclusively in NORMAL (transparent) mode:
 *   - Bytes written to UART are immediately transmitted over-air.
 *   - Received over-air bytes appear in the UART RX FIFO.
 *   - AUX goes LOW during TX and returns HIGH when complete.
 *
 * The module handles RF framing, FEC, and channel hopping
 * internally.  This driver provides raw byte-level access;
 * protocol framing is handled by ares_radio_protocol.h.
 *
 * Max single-packet size: 240 bytes (module limit).
 */
class DxLr03Driver : public RadioInterface
{
public:
    /**
     * Construct a DX-LR03 LoRa driver instance.
     * @param[in] serial  HardwareSerial port (UART2) — not owned.
     * @param[in] txPin   ESP32 GPIO for UART TX (→ module RX).
     * @param[in] rxPin   ESP32 GPIO for UART RX (← module TX).
     * @param[in] auxPin  ESP32 GPIO for AUX (input, HIGH = idle).
     * @param[in] baud    UART baud rate (default 9600).
     */
    DxLr03Driver(HardwareSerial& serial,
                 int8_t txPin, int8_t rxPin, int8_t auxPin,
                 uint32_t baud);

    // Non-copyable, non-movable (CERT-18.3)
    DxLr03Driver(const DxLr03Driver&)            = delete;
    DxLr03Driver& operator=(const DxLr03Driver&) = delete;

    /**
     * Initialise the UART connection to the LoRa module.
     * @return true on success; false on UART failure.
     * @post Module is in receive mode on success.
     */
    bool begin() override;

    /**
     * Transmit a raw byte buffer to the LoRa module.
     * @param[in] data  Buffer to transmit (caller-owned).
     * @param[in] len   Number of bytes to transmit.
     * @return Status code (OK, ERROR, BUSY, TIMEOUT, or OVERFLOW).
     * @note Blocks until TX is complete or timeout; may spin on AUX.
     */
    RadioStatus send(const uint8_t* data, uint16_t len) override;

    /**
     * Receive available bytes from the LoRa module's UART FIFO.
     * @param[out] buf       Destination buffer (caller-owned).
     * @param[in]  bufSize   Size of @p buf in bytes.
     * @param[out] received  Number of bytes actually read (≤ bufSize).
     * @return Status code; OK even if received == 0 (no data yet).
     * @note Non-blocking — returns immediately.
     */
    RadioStatus receive(uint8_t* buf, uint16_t bufSize,
                        uint16_t& received) override;

    /**
     * Check if the LoRa module is ready to transmit (AUX == HIGH).
     * @return true if the module is idle and not transmitting.
     */
    bool ready() const override;

    /**
     * Get the maximum transmission unit (MTU) for this module.
     * @return Module MTU in bytes (240 for DX-LR03).
     */
    uint16_t mtu() const override;
    const char* driverModel() const override { return "LORA"; }

private:
    /**
     * Block until AUX goes HIGH or timeout expires.
     * @param[in] timeoutMs  Max wait in milliseconds.
     * @return true if AUX went HIGH, false on timeout.
     */
    bool waitReady(uint32_t timeoutMs) const;

    HardwareSerial& serial_;    ///< UART port (injected, not owned).
    int8_t  txPin_;              ///< ESP32 GPIO → module RX.
    int8_t  rxPin_;              ///< ESP32 GPIO ← module TX.
    int8_t  auxPin_;             ///< AUX busy indicator (input).
    uint32_t baud_;              ///< UART baud rate.
    bool     ready_        = false;  ///< true after successful begin().
    bool     auxAvailable_ = false;  ///< true if AUX responded during init.

    /// Module max packet size (bytes over-air in a single burst).
    static constexpr uint16_t MODULE_MTU = 240;

    /// AUX wait timeout when starting a transmission (ms).
    static constexpr uint32_t AUX_TIMEOUT_MS = ares::LORA_AUX_TIMEOUT_MS;
};

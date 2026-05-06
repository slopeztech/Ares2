/**
 * @file  dxlr03_driver.cpp
 * @brief DX-LR03-433T30D driver implementation.
 *
 * Normal-mode (transparent) UART driver.  All RF configuration is
 * done via the module's on-board DIP switches or UART AT commands
 * in CONFIG mode (not supported here — M0/M1 not wired).
 *
 * AUX pin behaviour:
 *   On some DX-LR03 modules or wiring configurations the AUX pin
 *   remains permanently LOW even though the module transmits and
 *   receives correctly.  This was observed during HW bring-up
 *   (2026-04-20): AUX stayed LOW with INPUT_PULLUP, yet UART TX
 *   was received by a second LoRa node without issues.
 *
 *   Therefore the driver treats AUX as **advisory**: if AUX is
 *   HIGH the driver waits for idle before TX; if AUX stays LOW
 *   for longer than AUX_TIMEOUT_MS the driver proceeds with the
 *   UART write anyway.  This prevents the module from being
 *   blocked indefinitely while still honouring AUX when it works.
 */

#include "dxlr03_driver.h"
#include "debug/ares_log.h"

#include <freertos/task.h>
#include <cinttypes>

static constexpr const char* TAG = "LORA";

// ── Constructor ──────────────────────────────────────────
DxLr03Driver::DxLr03Driver(HardwareSerial& serial,
                           int8_t txPin, int8_t rxPin, int8_t auxPin,
                           uint32_t baud)
    : serial_(serial)
    , txPin_(txPin)
    , rxPin_(rxPin)
    , auxPin_(auxPin)
    , baud_(baud)
{
}

// ── begin() ──────────────────────────────────────────────
bool DxLr03Driver::begin()
{
    // INPUT_PULLUP: if AUX is not wired, the pull-up keeps the pin
    // HIGH (= idle).  If the module drives AUX, the weak pull-up
    // has no measurable effect.
    pinMode(static_cast<uint8_t>(auxPin_), INPUT_PULLUP);

    // Initialise UART with specified pins
    serial_.begin(baud_, SERIAL_8N1, rxPin_, txPin_);

    // Wait for module self-test (AUX LOW → HIGH, ~1 s).
    // If AUX never goes HIGH the module may still work — some
    // DX-LR03 units hold AUX LOW permanently (see file header).
    auxAvailable_ = waitReady(AUX_TIMEOUT_MS);
    if (!auxAvailable_)
    {
        LOG_W(TAG, "AUX stayed LOW — flow control disabled");
    }

    // Flush any stale data in RX buffer (PO10-2: bounded)
    constexpr uint16_t MAX_FLUSH_BYTES = 512;
    for (uint16_t i = 0; i < MAX_FLUSH_BYTES && serial_.available() > 0; ++i)
    {
        (void)serial_.read();
    }

    ready_ = true;
        LOG_I(TAG, "DX-LR03 ready (baud=%" PRIu32 ", AUX=GPIO%d, flow_ctrl=%s)",
              baud_, static_cast<int>(auxPin_),
          auxAvailable_ ? "on" : "off");
    return true;
}

// ── send() ───────────────────────────────────────────────
RadioStatus DxLr03Driver::send(const uint8_t* data, uint16_t len)
{
    if (!ready_)          { return RadioStatus::NOT_READY; }
    if (data == nullptr)  { return RadioStatus::ERROR; }
    if (len == 0)         { return RadioStatus::OK; }
    if (len > MODULE_MTU) { return RadioStatus::OVERFLOW; }

    // If AUX-based flow control is available, wait for the module
    // to signal idle.  Otherwise skip — the module works without
    // AUX on some units (see file header).
    if (auxAvailable_ && !waitReady(AUX_TIMEOUT_MS))
    {
        auxAvailable_ = false;
        LOG_W(TAG, "AUX timeout during TX — flow control disabled");
    }

    // Write payload to UART (module transmits over-air)
    const size_t written = serial_.write(data, len);
    if (written != len)
    {
        LOG_E(TAG, "TX short write: %u/%u", static_cast<unsigned>(written),
              static_cast<unsigned>(len));
        return RadioStatus::ERROR;
    }

    serial_.flush();  // Wait for UART TX FIFO to drain

    return RadioStatus::OK;
}

// ── receive() ────────────────────────────────────────────
RadioStatus DxLr03Driver::receive(uint8_t* buf, uint16_t bufSize,
                                  uint16_t& received)
{
    received = 0;

    if (!ready_)         { return RadioStatus::NOT_READY; }
    if (buf == nullptr)  { return RadioStatus::ERROR; }
    if (bufSize == 0)    { return RadioStatus::OK; }

    // Read whatever is available in the UART RX FIFO (non-blocking)
    while (serial_.available() > 0 && received < bufSize)
    {
        const int32_t rxByte = static_cast<int32_t>(serial_.read());
        if (rxByte < 0) { break; }   // should not happen, but guard
        buf[received] = static_cast<uint8_t>(rxByte);
        ++received;
    }

    return RadioStatus::OK;
}

// ── ready() ──────────────────────────────────────────────
bool DxLr03Driver::ready() const
{
    if (!ready_) { return false; }
    // When AUX is not usable, assume the module is always ready.
    if (!auxAvailable_) { return true; }
    return digitalRead(static_cast<uint8_t>(auxPin_)) == HIGH;
}

// ── mtu() ────────────────────────────────────────────────
uint16_t DxLr03Driver::mtu() const
{
    return MODULE_MTU;
}

// ── waitReady() ──────────────────────────────────────────
bool DxLr03Driver::waitReady(uint32_t timeoutMs) const
{
    static constexpr uint32_t POLL_PERIOD_MS = 1U;
    static constexpr uint16_t MAX_AUX_POLLS = 2000U;

    // POLL_PERIOD_MS is fixed to 1 ms for AUX polling.
    const uint32_t requestedPolls = timeoutMs;

    uint16_t pollsLeft = MAX_AUX_POLLS;
    if (requestedPolls < static_cast<uint32_t>(MAX_AUX_POLLS))
    {
        pollsLeft = static_cast<uint16_t>(requestedPolls);
    }

    while (pollsLeft > 0U)
    {
        if (digitalRead(static_cast<uint8_t>(auxPin_)) != LOW)
        {
            return true;
        }

        pollsLeft--;
        if (pollsLeft == 0U)
        {
            return false;
        }

        vTaskDelay(pdMS_TO_TICKS(POLL_PERIOD_MS));
    }

    return false;
}

/**
 * @file  neopixel_driver.cpp
 * @brief WS2812 NeoPixel driver implementation.
 *
 * Brightness scaling uses 16-bit intermediate arithmetic to
 * avoid overflow: max product is 255 × 255 = 65 025 (CERT-4).
 */

#include "sys/led/neopixel_driver.h"
#include "ares_assert.h"

#include <Arduino.h>

// ── Constructor ─────────────────────────────────────────────

NeopixelDriver::NeopixelDriver(uint8_t pin)
    : pin_(pin)
{
}

// ── Interface implementation ────────────────────────────────

bool NeopixelDriver::begin()
{
    // PO10-5: double-init guard
    ARES_ASSERT(!ready_);

    neopixelWrite(pin_, 0, 0, 0);  // init RMT + LED off
    ready_ = true;
    return true;
}

void NeopixelDriver::set(const RgbColor& color)
{
    if (!ready_) { return; }  // guard — not yet initialised

    // Apply brightness scaling (CERT-4: 16-bit intermediate)
    uint8_t r = 0U;
    r = static_cast<uint8_t>((static_cast<uint16_t>(color.r) * brightness_) / 255U);
    uint8_t g = 0U;
    g = static_cast<uint8_t>((static_cast<uint16_t>(color.g) * brightness_) / 255U);
    uint8_t b = 0U;
    b = static_cast<uint8_t>((static_cast<uint16_t>(color.b) * brightness_) / 255U);

    neopixelWrite(pin_, r, g, b);
}

void NeopixelDriver::off()
{
    if (!ready_) { return; }  // guard

    neopixelWrite(pin_, 0, 0, 0);
}

void NeopixelDriver::setBrightness(uint8_t brightness)
{
    brightness_ = brightness;
}

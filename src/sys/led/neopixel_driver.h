/**
 * @file  neopixel_driver.h
 * @brief WS2812 (NeoPixel) LED driver for ESP32-S3.
 *
 * Uses the Arduino ESP32 core built-in neopixelWrite() for
 * addressable LED control via the RMT peripheral.  No external
 * library required.
 *
 * Thread safety: NOT thread-safe.  The caller (status_led task)
 *                must be the sole user.
 */
#pragma once

#include "hal/led/led_interface.h"

/**
 * Concrete LedInterface for WS2812 / SK6812 addressable LEDs.
 *
 * The ESP32-S3 Zero Mini has a single WS2812 on GPIO 21.
 * neopixelWrite() uses the RMT peripheral for bit-banging
 * the 800 kHz protocol with precise timing.
 */
class NeopixelDriver : public LedInterface
{
public:
    /**
     * Construct a NeoPixel driver for a given GPIO pin.
     * @param[in] pin  GPIO number connected to the LED data line.
     */
    explicit NeopixelDriver(uint8_t pin);

    // Non-copyable, non-movable (CERT-18.3)
    NeopixelDriver(const NeopixelDriver&)            = delete;
    NeopixelDriver& operator=(const NeopixelDriver&) = delete;
    NeopixelDriver(NeopixelDriver&&)                 = delete;
    NeopixelDriver& operator=(NeopixelDriver&&)      = delete;

    /** Initialise the NeoPixel strip (must be called before set() or off()). @return true on success. */
    bool begin() override;
    /** Set all pixels to @p color. @param[in] color RGB value to apply. */
    void set(const RgbColor& color) override;
    /** Turn all pixels off. */
    void off() override;
    /** Set global brightness scaling factor (0=off, 255=full). @param[in] brightness 0–255 scale. */
    void setBrightness(uint8_t brightness) override;

private:
    uint8_t pin_        = 0;    ///< GPIO number.
    uint8_t brightness_ = 255;  ///< 0–255 scaling factor.
    bool    ready_      = false; ///< true after successful begin().
};

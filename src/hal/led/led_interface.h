/**
 * @file  led_interface.h
 * @brief Hardware-agnostic RGB LED interface (pure virtual).
 *
 * Provides a simple abstraction for controlling a single
 * addressable RGB LED.  Concrete implementations handle the
 * communication protocol (WS2812, APA102, etc.).
 *
 * Thread safety: Implementations are NOT required to be
 *                thread-safe.  The caller must serialise
 *                access when sharing a driver between tasks.
 */
#pragma once

#include <cstdint>

/**
 * 24-bit RGB colour value.
 * All fields default-initialised to zero (MISRA-4).
 */
struct RgbColor
{
    uint8_t r = 0;  ///< Red channel (0–255).
    uint8_t g = 0;  ///< Green channel (0–255).
    uint8_t b = 0;  ///< Blue channel (0–255).
};

/// Predefined colours for status LED patterns (MISRA-7).
namespace colors
{
    constexpr RgbColor OFF    = {  0,   0,   0};  ///< LED off.
    constexpr RgbColor RED    = {255,   0,   0};  ///< Error indication.
    constexpr RgbColor GREEN  = {  0, 255,   0};  ///< Idle / healthy.
    constexpr RgbColor BLUE   = {  0,   0, 255};  ///< Flight mode.
    constexpr RgbColor CYAN   = {  0, 255, 255};  ///< Test mode.
    constexpr RgbColor YELLOW = {255, 255,   0};  ///< Warning.
    constexpr RgbColor WHITE  = {255, 255, 255};  ///< Full white.
}

/**
 * Abstract interface for a single RGB LED.
 *
 * Implementations control the hardware protocol (WS2812, etc.).
 * Application code uses this interface exclusively — swapping
 * the LED type only requires changing the concrete driver in
 * main.cpp.
 */
class LedInterface
{
public:
    virtual ~LedInterface() = default;

    /**
     * Initialise the LED hardware.
     * @return true on success, false on failure.
     * @pre  GPIO must not be in use by another peripheral.
     * @post LED is off and ready to receive colour commands.
     */
    virtual bool begin() = 0;

    /**
     * Set the LED to a solid colour.
     * @param[in] color  RGB value to display.
     * @pre  begin() returned true.
     */
    virtual void set(const RgbColor& color) = 0;

    /**
     * Turn the LED off (equivalent to set({0,0,0})).
     * @pre  begin() returned true.
     */
    virtual void off() = 0;

    /**
     * Set global brightness scaling.
     * @param[in] brightness  0 (off) to 255 (full).
     * @pre  begin() returned true.
     */
    virtual void setBrightness(uint8_t brightness) = 0;
};

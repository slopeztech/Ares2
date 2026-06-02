/**
 * @file  passive_buzzer_driver.h
 * @brief Concrete driver for a passive buzzer (ESP32 LEDC PWM tone generation).
 *
 * A passive buzzer contains no internal oscillator — it requires an external
 * PWM signal at the desired audio frequency.  This driver uses the ESP32 LEDC
 * peripheral to generate a square wave at the requested frequency with a 50 %
 * duty cycle (half the LEDC resolution), which gives the loudest audible tone.
 *
 * If @p freqHz == 0 in beep(), the driver falls back to the compile-time
 * constant @c ares::BUZZER_DEFAULT_FREQ_HZ.
 *
 * The beep is non-blocking: beep() sets up the LEDC tone and starts a
 * FreeRTOS one-shot timer; the timer callback silences the output.
 * stop() cancels the timer and silences the output immediately.
 *
 * Platform: ESP32-S3 / Arduino (esp32-arduino HAL).
 */
#pragma once

#include <cstdint>
#include <freertos/FreeRTOS.h>
#include <freertos/timers.h>

#include "hal/buzzer/buzzer_interface.h"

namespace ares
{

/**
 * @brief Passive buzzer driver — ESP32 LEDC PWM tone generation.
 *
 * @par Usage
 * @code
 *   static PassiveBuzzerDriver buzzer(PIN_BUZZER_PASSIVE, BUZZER_LEDC_CHANNEL);
 *   buzzer.begin();
 *   buzzer.beep(500, 2000);  // 500 ms at 2 kHz
 *   buzzer.beep(300);        // 300 ms at BUZZER_DEFAULT_FREQ_HZ
 * @endcode
 */
class PassiveBuzzerDriver : public BuzzerInterface
{
public:
    /**
     * @param[in] pin          GPIO pin connected to the passive buzzer.
     * @param[in] ledcChannel  LEDC channel to use (0–15 on ESP32-S3).
     *                         Use the constant @c ares::BUZZER_LEDC_CHANNEL.
     */
    PassiveBuzzerDriver(uint8_t pin, uint8_t ledcChannel);
    ~PassiveBuzzerDriver() override;

    bool begin()                                              override;
    bool beep(uint32_t durationMs, uint32_t freqHz = 0U,
              uint8_t count = 1U)                            override;
    // cppcheck-suppress virtualCallInConstructor
    void stop()                                              override;
    bool isBusy() const                                      override;

private:
    static void timerCallback(TimerHandle_t xTimer);

    /// Silence the LEDC output without touching the timer.
    void silenceLocked();

    uint8_t       pin_;
    uint8_t       ledcChannel_;
    bool          initialised_    = false;
    bool          busy_           = false;
    TimerHandle_t timer_          = nullptr;
    uint8_t       pendingRepeats_ = 0U;    ///< Remaining beeps after the current one.
    uint32_t      repeatDurMs_    = 0U;    ///< Cached tone duration for repeat sequence.
    uint32_t      repeatFreqHz_   = 0U;    ///< Cached frequency for repeat sequence.
    bool          inGap_          = false; ///< True while waiting the silent gap between repeats.

    static constexpr uint8_t  kResolutionBits = 8U;   ///< LEDC resolution: 8-bit (values 0–255).
    static constexpr uint32_t kDuty50Pct      = 128U; ///< 50 % duty for maximum audible amplitude.
};

} // namespace ares

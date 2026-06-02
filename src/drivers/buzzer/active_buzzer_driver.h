/**
 * @file  active_buzzer_driver.h
 * @brief Concrete driver for an active buzzer (digital GPIO on/off).
 *
 * An active buzzer has an internal oscillator — it sounds whenever its
 * control pin is driven HIGH.  No frequency configuration is possible;
 * the @p freqHz argument to beep() is accepted but silently ignored.
 *
 * The beep is non-blocking: beep() asserts the GPIO and schedules a
 * FreeRTOS one-shot timer to de-assert it after @p durationMs.  stop()
 * cancels any pending timer and de-asserts the GPIO immediately.
 *
 * Platform: ESP32-S3 / Arduino + FreeRTOS.
 */
#pragma once

#include <cstdint>
#include <freertos/FreeRTOS.h>
#include <freertos/timers.h>

#include "hal/buzzer/buzzer_interface.h"

namespace ares
{

/**
 * @brief Active buzzer driver — digital GPIO, no frequency control.
 *
 * @par Usage
 * @code
 *   static ActiveBuzzerDriver buzzer(PIN_BUZZER_ACTIVE);
 *   buzzer.begin();
 *   buzzer.beep(300);        // 300 ms beep, freqHz ignored
 * @endcode
 */
class ActiveBuzzerDriver : public BuzzerInterface
{
public:
    /**
     * @param[in] pin  GPIO pin number connected to the buzzer control line.
     *                 A HIGH output activates the buzzer; LOW deactivates it.
     */
    explicit ActiveBuzzerDriver(uint8_t pin);
    ~ActiveBuzzerDriver() override;

    bool begin()                                              override;
    bool beep(uint32_t durationMs, uint32_t freqHz = 0U,
              uint8_t count = 1U)                            override;
    // cppcheck-suppress virtualCallInConstructor
    void stop()                                              override;
    bool isBusy() const                                      override;

private:
    static void timerCallback(TimerHandle_t xTimer);

    uint8_t      pin_;
    bool         initialised_    = false;
    bool         busy_           = false;
    TimerHandle_t timer_         = nullptr;
    uint8_t      pendingRepeats_ = 0U;    ///< Remaining beeps after the current one.
    uint32_t     repeatDurMs_    = 0U;    ///< Cached tone duration for repeat sequence.
    bool         inGap_          = false; ///< True while waiting the silent gap between repeats.
};

} // namespace ares

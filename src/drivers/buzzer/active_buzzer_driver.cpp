/**
 * @file  active_buzzer_driver.cpp
 * @brief Active buzzer driver implementation (GPIO + FreeRTOS timer).
 */

#include "drivers/buzzer/active_buzzer_driver.h"

#include "config.h"
#include "debug/ares_log.h"

#include <Arduino.h>

static constexpr const char* TAG = "BUZZ";

namespace ares
{

ActiveBuzzerDriver::ActiveBuzzerDriver(uint8_t pin)
    : pin_(pin)
{
}

ActiveBuzzerDriver::~ActiveBuzzerDriver()
{
    stop();
    if (timer_ != nullptr)
    {
        xTimerDelete(timer_, portMAX_DELAY);
        timer_ = nullptr;
    }
}

bool ActiveBuzzerDriver::begin()
{
    if (pin_ == 0xFFU)
    {
        // Silently report as initialised when no pin is wired.
        initialised_ = true;
        return true;
    }

    // Create a one-shot FreeRTOS timer.  pvTimerID = this pointer so
    // the static callback can reach back into the driver instance.
    timer_ = xTimerCreate(
        "buzzer_act",
        pdMS_TO_TICKS(1U),   // period set in beep(); dummy value here
        pdFALSE,              // one-shot
        static_cast<void*>(this),
        &ActiveBuzzerDriver::timerCallback);

    if (timer_ == nullptr)
    {
        LOG_E(TAG, "ActiveBuzzerDriver: xTimerCreate failed");
        return false;
    }

    pinMode(pin_, OUTPUT);
    digitalWrite(pin_, LOW);
    initialised_ = true;

    LOG_I(TAG, "ActiveBuzzerDriver: init OK (pin=%u)", static_cast<unsigned>(pin_));
    return true;
}

bool ActiveBuzzerDriver::beep(uint32_t durationMs, uint32_t /*freqHz*/, uint8_t count)
{
    if (!initialised_ || timer_ == nullptr) { return false; }
    if (durationMs == 0U)                  { return false; }
    if (pin_ == 0xFFU)                     { return true; }  // no-op, pin not wired

    // Clamp to configured limits.
    if (durationMs < ares::BUZZER_MIN_DURATION_MS)
    {
        durationMs = ares::BUZZER_MIN_DURATION_MS;
    }
    else if (durationMs > ares::BUZZER_MAX_DURATION_MS)
    {
        durationMs = ares::BUZZER_MAX_DURATION_MS;
    }

    // Cancel any in-progress beep.
    if (busy_)
    {
        (void)xTimerStop(timer_, 0U);
        digitalWrite(pin_, LOW);
    }

    // Clamp repeat count.
    if (count == 0U) { count = 1U; }
    if (count > ares::BUZZER_MAX_REPEAT_COUNT) { count = ares::BUZZER_MAX_REPEAT_COUNT; }

    // Cache parameters for the repeat sequence and start first tone.
    repeatDurMs_    = durationMs;
    pendingRepeats_ = static_cast<uint8_t>(count - 1U);
    inGap_          = false;

    // Set timer period and start.
    (void)xTimerChangePeriod(timer_, pdMS_TO_TICKS(durationMs), portMAX_DELAY);

    busy_ = true;
    digitalWrite(pin_, HIGH);

    LOG_D(TAG, "ActiveBuzzerDriver: beep %u ms x%u", static_cast<unsigned>(durationMs), static_cast<unsigned>(count));
    return true;
}

void ActiveBuzzerDriver::stop()
{
    if (!initialised_ || !busy_) { return; }

    if (timer_ != nullptr)
    {
        (void)xTimerStop(timer_, 0U);
    }
    if (pin_ != 0xFFU)
    {
        digitalWrite(pin_, LOW);
    }
    busy_ = false;

    LOG_D(TAG, "ActiveBuzzerDriver: stopped");
}

bool ActiveBuzzerDriver::isBusy() const
{
    return busy_;
}

// static
void ActiveBuzzerDriver::timerCallback(TimerHandle_t xTimer)
{
    auto* self = static_cast<ActiveBuzzerDriver*>(pvTimerGetTimerID(xTimer));
    if (self == nullptr) { return; }

    if (self->inGap_)
    {
        // Silent gap ended: start the next tone.
        self->inGap_ = false;
        if (self->pin_ != 0xFFU) { digitalWrite(self->pin_, HIGH); }
        (void)xTimerChangePeriod(self->timer_, pdMS_TO_TICKS(self->repeatDurMs_), portMAX_DELAY);
    }
    else
    {
        // Tone ended: silence the output.
        if (self->pin_ != 0xFFU) { digitalWrite(self->pin_, LOW); }

        if (self->pendingRepeats_ > 0U)
        {
            // Schedule a gap of the same duration before the next beep.
            self->pendingRepeats_--;
            self->inGap_ = true;
            (void)xTimerChangePeriod(self->timer_, pdMS_TO_TICKS(self->repeatDurMs_), portMAX_DELAY);
        }
        else
        {
            self->busy_ = false;
        }
    }
}

} // namespace ares

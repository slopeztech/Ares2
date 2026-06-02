/**
 * @file  passive_buzzer_driver.cpp
 * @brief Passive buzzer driver implementation (ESP32 LEDC + FreeRTOS timer).
 */

#include "drivers/buzzer/passive_buzzer_driver.h"

#include "config.h"
#include "debug/ares_log.h"

#include <Arduino.h>

static constexpr const char* TAG = "BUZZ";

namespace ares
{

PassiveBuzzerDriver::PassiveBuzzerDriver(uint8_t pin, uint8_t ledcChannel)
    : pin_(pin), ledcChannel_(ledcChannel)
{
}

PassiveBuzzerDriver::~PassiveBuzzerDriver()
{
    stop();
    if (timer_ != nullptr)
    {
        xTimerDelete(timer_, portMAX_DELAY);
        timer_ = nullptr;
    }
}

bool PassiveBuzzerDriver::begin()
{
    if (pin_ == 0xFFU)
    {
        initialised_ = true;
        return true;
    }

    timer_ = xTimerCreate(
        "buzzer_pas",
        pdMS_TO_TICKS(1U),
        pdFALSE,
        static_cast<void*>(this),
        &PassiveBuzzerDriver::timerCallback);

    if (timer_ == nullptr)
    {
        LOG_E(TAG, "PassiveBuzzerDriver: xTimerCreate failed");
        return false;
    }

    // Configure LEDC at default frequency, duty = 0 (silent).
    ledcSetup(ledcChannel_, ares::BUZZER_DEFAULT_FREQ_HZ, kResolutionBits);
    ledcAttachPin(pin_, ledcChannel_);
    ledcWrite(ledcChannel_, 0U);

    initialised_ = true;
    LOG_I(TAG, "PassiveBuzzerDriver: init OK (pin=%u ch=%u)",
          static_cast<unsigned>(pin_),
          static_cast<unsigned>(ledcChannel_));
    return true;
}

bool PassiveBuzzerDriver::beep(uint32_t durationMs, uint32_t freqHz, uint8_t count)
{
    if (!initialised_)             { return false; }
    if (durationMs == 0U)          { return false; }
    if (pin_ == 0xFFU)             { return true; }  // no-op
    if (timer_ == nullptr)         { return false; }

    // Clamp parameters to configured limits.
    if (durationMs < ares::BUZZER_MIN_DURATION_MS)
    {
        durationMs = ares::BUZZER_MIN_DURATION_MS;
    }
    else if (durationMs > ares::BUZZER_MAX_DURATION_MS)
    {
        durationMs = ares::BUZZER_MAX_DURATION_MS;
    }

    if (freqHz == 0U)
    {
        freqHz = ares::BUZZER_DEFAULT_FREQ_HZ;
    }
    else if (freqHz < ares::BUZZER_MIN_FREQ_HZ)
    {
        freqHz = ares::BUZZER_MIN_FREQ_HZ;
    }
    else if (freqHz > ares::BUZZER_MAX_FREQ_HZ)
    {
        freqHz = ares::BUZZER_MAX_FREQ_HZ;
    }

    // Cancel any in-progress beep.
    if (busy_)
    {
        (void)xTimerStop(timer_, 0U);
        silenceLocked();
    }

    // Clamp repeat count.
    if (count == 0U) { count = 1U; }
    if (count > ares::BUZZER_MAX_REPEAT_COUNT) { count = ares::BUZZER_MAX_REPEAT_COUNT; }

    // Cache parameters for the repeat sequence.
    repeatDurMs_    = durationMs;
    repeatFreqHz_   = freqHz;
    pendingRepeats_ = static_cast<uint8_t>(count - 1U);
    inGap_          = false;

    // Re-configure LEDC with the requested frequency.
    ledcSetup(ledcChannel_, freqHz, kResolutionBits);
    ledcAttachPin(pin_, ledcChannel_);
    ledcWrite(ledcChannel_, kDuty50Pct);

    (void)xTimerChangePeriod(timer_, pdMS_TO_TICKS(durationMs), portMAX_DELAY);

    busy_ = true;

    LOG_D(TAG, "PassiveBuzzerDriver: beep %u ms @ %u Hz x%u",
          static_cast<unsigned>(durationMs),
          static_cast<unsigned>(freqHz),
          static_cast<unsigned>(count));
    return true;
}

void PassiveBuzzerDriver::stop()
{
    if (!initialised_ || !busy_) { return; }

    if (timer_ != nullptr)
    {
        (void)xTimerStop(timer_, 0U);
    }
    silenceLocked();
    busy_ = false;

    LOG_D(TAG, "PassiveBuzzerDriver: stopped");
}

bool PassiveBuzzerDriver::isBusy() const
{
    return busy_;
}

void PassiveBuzzerDriver::silenceLocked()
{
    if (pin_ != 0xFFU)
    {
        ledcWrite(ledcChannel_, 0U);
    }
}

// static
void PassiveBuzzerDriver::timerCallback(TimerHandle_t xTimer)
{
    auto* self = static_cast<PassiveBuzzerDriver*>(pvTimerGetTimerID(xTimer));
    if (self == nullptr) { return; }

    if (self->inGap_)
    {
        // Silent gap ended: start the next tone.
        self->inGap_ = false;
        ledcSetup(self->ledcChannel_, self->repeatFreqHz_, kResolutionBits);
        ledcAttachPin(self->pin_, self->ledcChannel_);
        ledcWrite(self->ledcChannel_, kDuty50Pct);
        (void)xTimerChangePeriod(self->timer_, pdMS_TO_TICKS(self->repeatDurMs_), portMAX_DELAY);
    }
    else
    {
        // Tone ended: silence the output.
        self->silenceLocked();

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

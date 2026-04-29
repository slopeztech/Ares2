/**
 * @file  status_led.cpp
 * @brief Status LED service implementation.
 *
 * Each operating mode maps to a unique visual pattern so that
 * the rocket's state is identifiable at a glance from a distance.
 * All timing constants are named (MISRA-7) and the task loop
 * always blocks on vTaskDelay (RTOS-1.3).
 */

#include "sys/led/status_led.h"
#include "ares_assert.h"
#include "debug/ares_log.h"

// ── Log tag ─────────────────────────────────────────────────
static constexpr const char* TAG = "LED";

// ── Pattern timing constants (MISRA-7) ──────────────────────
static constexpr uint32_t LED_BOOT_ON_MS        = 300;  ///< BOOT mode on-time.
static constexpr uint32_t LED_BOOT_OFF_MS       = 300;  ///< BOOT mode off-time.
static constexpr uint32_t LED_TEST_ON_MS        = 500;  ///< TEST mode on-time.
static constexpr uint32_t LED_TEST_OFF_MS       = 500;  ///< TEST mode off-time.
static constexpr uint32_t LED_RECOVERY_ON_MS    = 300;  ///< RECOVERY mode on-time.
static constexpr uint32_t LED_RECOVERY_OFF_MS   = 700;  ///< RECOVERY mode off-time.
static constexpr uint32_t LED_ERROR_FLASH_MS    = 100;  ///< ERROR flash duration.
static constexpr uint32_t LED_ERROR_GAP_MS      = 100;  ///< ERROR inter-flash gap.
static constexpr uint32_t LED_ERROR_PAUSE_MS    = 400;  ///< ERROR post-burst pause.
static constexpr uint8_t  LED_ERROR_FLASH_COUNT = 3;    ///< ERROR flashes per burst.

using ares::OperatingMode;

static OperatingMode decodeOperatingMode(uint8_t rawMode)
{
    OperatingMode mode = OperatingMode::ERROR;

    if (rawMode <= static_cast<uint8_t>(OperatingMode::LAST))
    {
        mode = static_cast<OperatingMode>(rawMode);
    }

    return mode;
}

// ── Constructor ─────────────────────────────────────────────

StatusLed::StatusLed(LedInterface& led)
    : led_(led)
{
}

// ── Public API ──────────────────────────────────────────────

bool StatusLed::begin()
{
    TaskHandle_t handle = xTaskCreateStaticPinnedToCore(
        taskFn,
        "led",
        sizeof(stack_),             // RTOS-7.2: size in bytes
        this,                       // pass self as parameter
        ares::TASK_PRIORITY_LED,
        stack_,
        &tcb_,
        tskNO_AFFINITY);            // RTOS-13: LED is non-critical

    ARES_ASSERT(handle != nullptr);  // PO10-5: static create never fails

    LOG_I(TAG, "task started (stack=%u pri=%u)",
          static_cast<uint32_t>(sizeof(stack_)),
          static_cast<uint32_t>(ares::TASK_PRIORITY_LED));

    return handle != nullptr;
}

void StatusLed::setMode(OperatingMode mode)
{
    mode_.store(static_cast<uint8_t>(mode));
}

// ── RTOS task ───────────────────────────────────────────────

void StatusLed::taskFn(void* param)
{
    auto* self = static_cast<StatusLed*>(param);
    ARES_ASSERT(self != nullptr);  // PO10-5
    self->run();
}

void StatusLed::run()
{
    while (true)  // RTOS: task loop — blocks on vTaskDelay
    {
        uint8_t rawMode = 0U;
        rawMode = mode_.load();
        const OperatingMode mode = decodeOperatingMode(rawMode);

        switch (mode)
        {
        case OperatingMode::BOOT:
            patternBlink(colors::GREEN,
                         LED_BOOT_ON_MS, LED_BOOT_OFF_MS);
            break;

        case OperatingMode::IDLE:
            patternSolid(colors::GREEN);
            break;

        case OperatingMode::TEST:
            patternBlink(colors::CYAN,
                         LED_TEST_ON_MS, LED_TEST_OFF_MS);
            break;

        case OperatingMode::FLIGHT:
            patternSolid(colors::BLUE);
            break;

        case OperatingMode::RECOVERY:
            patternBlink(colors::GREEN,
                         LED_RECOVERY_ON_MS, LED_RECOVERY_OFF_MS);
            break;

        case OperatingMode::ERROR:
            patternTripleBlink();
            break;

        default:  // CERT-6.3: unknown value — safe fallback
            LOG_W(TAG, "unknown mode=%u",
                  static_cast<uint32_t>(rawMode));
            patternTripleBlink();
            break;
        }
    }
}

// ── Pattern helpers ─────────────────────────────────────────

// Solid colour with LED_RATE_MS delay (RTOS-6: non-critical timing).
void StatusLed::patternSolid(const RgbColor& color)
{
    led_.set(color);
    vTaskDelay(pdMS_TO_TICKS(ares::LED_RATE_MS));
}

// On/off blink with configurable duty cycle.
void StatusLed::patternBlink(const RgbColor& color,
                              uint32_t onMs, uint32_t offMs)
{
    led_.set(color);
    vTaskDelay(pdMS_TO_TICKS(onMs));
    led_.off();
    vTaskDelay(pdMS_TO_TICKS(offMs));
}

// Triple red flash then pause — distinctive error indicator.
void StatusLed::patternTripleBlink()
{
    for (uint8_t i = 0; i < LED_ERROR_FLASH_COUNT; i++)  // PO10-2: bounded
    {
        led_.set(colors::RED);
        vTaskDelay(pdMS_TO_TICKS(LED_ERROR_FLASH_MS));
        led_.off();
        vTaskDelay(pdMS_TO_TICKS(LED_ERROR_GAP_MS));
    }
    vTaskDelay(pdMS_TO_TICKS(LED_ERROR_PAUSE_MS));
}

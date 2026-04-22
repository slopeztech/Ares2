/**
 * @file  status_led.h
 * @brief Status LED service — maps operating mode to LED patterns.
 *
 * Runs a dedicated FreeRTOS task (RTOS-2: single responsibility)
 * that reads the current operating mode and drives the on-board
 * RGB LED with a distinctive visual pattern for each mode.
 *
 * LED behaviour per mode:
 *   BOOT     — fast green blink  (300 ms on / 300 ms off) — system initialising
 *   IDLE     — solid green
 *   TEST     — slow cyan blink   (500 ms on / 500 ms off)
 *   FLIGHT   — solid blue
 *   RECOVERY — slow green blink  (300 ms on / 700 ms off)
 *   ERROR    — fast red triple-blink (3× 100 ms on, 100 ms gap)
 *
 * Thread safety: setMode() is lock-free (std::atomic).
 *                All other methods run on the internal task only.
 */
#pragma once

#include <atomic>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "config.h"
#include "hal/led/led_interface.h"

/**
 * Status LED service — displays operating mode as LED patterns.
 *
 * Owns a dedicated FreeRTOS task (RTOS-2: single responsibility).
 * The task stack and TCB are statically allocated inside this
 * object (PO10-3, RTOS-7.1).  The object must be statically
 * allocated (not on a task stack) because it contains a ~2 KiB
 * task stack buffer.
 *
 * Concurrency model:
 *   - setMode() writes an atomic — lock-free, any task, any core.
 *   - The internal task reads the atomic each iteration and
 *     selects the corresponding LED pattern.
 *   - The LED driver is accessed exclusively from the internal
 *     task — no mutex required.
 */
class StatusLed
{
public:
    /**
     * Construct the status LED service.
     * @param[in] led  Reference to an initialised LedInterface.
     *                 Lifetime must exceed StatusLed.
     */
    explicit StatusLed(LedInterface& led);

    // Non-copyable, non-movable (CERT-18.3)
    StatusLed(const StatusLed&)            = delete;
    StatusLed& operator=(const StatusLed&) = delete;
    StatusLed(StatusLed&&)                 = delete;
    StatusLed& operator=(StatusLed&&)      = delete;

    /**
     * Create and start the LED RTOS task.
     * @return true on success, false on failure.
     * @pre  The referenced LedInterface::begin() has returned true.
     * @post LED task is running and displaying the current mode.
     */
    bool begin();

    /**
     * Set the current operating mode.
     * Thread-safe — can be called from any task or ISR context.
     * @param[in] mode  New operating mode to display.
     */
    void setMode(ares::OperatingMode mode);

private:
    /// RTOS task entry point (static, forwards to run()).
    static void taskFn(void* param);

    /// Main task loop — reads mode, selects pattern.  Never returns.
    void run();

    // Pattern helpers (called only from the internal task)
    void patternSolid(const RgbColor& color);
    void patternBlink(const RgbColor& color, uint32_t onMs, uint32_t offMs);
    void patternTripleBlink();

    LedInterface& led_;  ///< Borrowed LED driver reference.

    /// Current operating mode (lock-free atomic, CERT-13).
    std::atomic<uint8_t> mode_{
        static_cast<uint8_t>(ares::OperatingMode::BOOT)};

    /// Static task stack — no heap allocation (PO10-3, RTOS-7.1).
    StackType_t stack_[ares::TASK_STACK_SIZE_LED / sizeof(StackType_t)] = {};

    /// Static task control block.
    StaticTask_t tcb_ = {};
};

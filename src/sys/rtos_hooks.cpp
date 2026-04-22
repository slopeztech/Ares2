/**
 * @file  rtos_hooks.cpp
 * @brief FreeRTOS runtime safety hooks (stack overflow handling).
 */

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "debug/ares_log.h"

#include <esp_system.h>

extern "C" void vApplicationStackOverflowHook(TaskHandle_t xTask,
                                                char* pcTaskName)
{
    const char* taskName = (pcTaskName != nullptr) ? pcTaskName : "<unknown>";
    LOG_E("RTOS", "stack overflow: task=%s handle=%p", taskName, xTask);
    esp_restart();
}

/**
 * @file  esp_task_wdt.h
 * @brief Minimal ESP-IDF task watchdog stub for native api_routing builds.
 *
 * api_server.cpp calls esp_task_wdt_add(nullptr) and esp_task_wdt_reset()
 * inside run().  Tests never call run(), but the TU must compile and link.
 * Both functions are safe no-ops that always return ESP_OK.
 */
#pragma once

/// ESP-IDF error code type.
using esp_err_t = int;

/// Success code — matches the real ESP-IDF ESP_OK value.
static constexpr esp_err_t ESP_OK = 0;

/** Subscribe the calling task to the watchdog.  No-op in the sim. */
inline esp_err_t esp_task_wdt_add(void* /*handle*/) { return ESP_OK; }

/** Reset (pet) the watchdog for the calling task.  No-op in the sim. */
inline esp_err_t esp_task_wdt_reset() { return ESP_OK; }

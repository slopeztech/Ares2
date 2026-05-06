/**
 * @file  api_common.h
 * @brief Shared utilities for the ARES REST API layer.
 *
 * Provides CORS headers and HTTP parsing constants used by the API
 * server core and all route-handler subdirectories.
 * ScopedLock RAII guard is provided via rtos_guard.h (CERT-18.1, RTOS-4).
 */
#pragma once

#include "rtos_guard.h"  // ScopedLock — canonical shared RAII mutex guard

// ── CORS headers (REST-7) ───────────────────────────────────
inline constexpr const char* CORS_HEADERS =
    "Access-Control-Allow-Origin: *\r\n"
    "Access-Control-Allow-Methods: GET, PUT, POST, DELETE, OPTIONS\r\n"
    "Access-Control-Allow-Headers: Content-Type\r\n";

// ── HTTP parsing constants (MISRA-7) ────────────────────────
static constexpr uint16_t HTTP_LINE_MAX       = 256;  ///< Max HTTP request line.
static constexpr uint16_t HTTP_HEADER_MAX     = 256;  ///< Max header line.
static constexpr uint8_t  HTTP_MAX_HEADERS    = 16;   ///< Max headers to parse (REST-3).
static constexpr uint16_t HTTP_METHOD_MAX     = 8;    ///< "OPTIONS" + null.
static constexpr uint8_t  HTTP_PATH_MAX       = 64;   ///< URL path buffer (REST-1.1).

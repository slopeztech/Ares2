/**
 * @file  status_pure.h
 * @brief Pure conversion helpers for the status/bus-scan REST handlers.
 *
 * Extracted from bus_scan_handler.cpp so that host-native unit tests
 * can exercise the logic without pulling in Arduino / Wire / WiFiClient.
 */
#pragma once

#include "hal/gps/gps_interface.h"   // GpsStatus

namespace ares
{
namespace api
{

/**
 * @brief Convert a GpsStatus enum to a lowercase JSON-safe string.
 *
 * @param[in] st  Status value to convert.
 * @return Null-terminated constant string (never NULL).
 */
inline const char* gpsStatusToString(GpsStatus st)
{
    switch (st)
    {
    case GpsStatus::OK:        return "ok";
    case GpsStatus::NO_FIX:   return "no_fix";
    case GpsStatus::ERROR:     return "error";
    case GpsStatus::NOT_READY: return "not_ready";
    case GpsStatus::TIMEOUT:   return "timeout";
    default:                   return "invalid";
    }
}

} // namespace api
} // namespace ares

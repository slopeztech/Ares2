/**
 * @file  flight_pure.h
 * @brief Pure validation helpers for the flight REST handler.
 *
 * Extracted from flight_handler.cpp so that host-native unit tests
 * can exercise the logic without pulling in Arduino / WiFiClient.
 *
 * All functions are `inline` — include this header from both the
 * production .cpp and the test file.
 */
#pragma once

#include <cstring>

#include "config.h"   // ares::OperatingMode

namespace ares
{
namespace api
{

/**
 * @brief Validate a mode transition and return the target OperatingMode.
 *
 * Implements the REST-6.1 transition matrix:
 *   - "idle"   : allowed from TEST, RECOVERY, ERROR
 *   - "test"   : allowed from IDLE
 *   - "flight" : allowed from IDLE or TEST
 *
 * @param[in]  target   Requested mode string ("idle", "test", "flight").
 * @param[in]  current  Current operating mode.
 * @param[out] out      Set to the target mode when the transition is valid.
 * @return  0   — valid transition; @p out is set.
 * @return  400 — unrecognised mode string.
 * @return  409 — mode string is valid but the transition is not allowed.
 */
inline int32_t validateModeTransition(const char*         target,
                                      OperatingMode       current,
                                      OperatingMode&      out)
{
    if (strcmp(target, "idle") == 0)
    {
        if (current != OperatingMode::TEST
            && current != OperatingMode::RECOVERY
            && current != OperatingMode::ERROR)
        {
            return 409;
        }
        out = OperatingMode::IDLE;
    }
    else if (strcmp(target, "test") == 0)
    {
        if (current != OperatingMode::IDLE)
        {
            return 409;
        }
        out = OperatingMode::TEST;
    }
    else if (strcmp(target, "flight") == 0)
    {
        if (current != OperatingMode::IDLE
            && current != OperatingMode::TEST)
        {
            return 409;
        }
        out = OperatingMode::FLIGHT;
    }
    else
    {
        return 400;
    }
    return 0;
}

} // namespace api
} // namespace ares

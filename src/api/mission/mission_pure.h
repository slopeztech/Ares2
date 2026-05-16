/**
 * @file  mission_pure.h
 * @brief Pure validation helpers for the mission REST handler.
 *
 * Extracted from mission_handler.cpp so that host-native unit tests
 * can exercise the logic without pulling in Arduino / WiFiClient.
 */
#pragma once

#include <cstdint>
#include <cstdio>
#include <cstring>

#include "config.h"                      // ares::MISSION_DIR, MISSION_FILENAME_MAX
#include "ams/mission_script_engine.h"   // ares::ams::EngineStatus

namespace ares
{
namespace api
{

/**
 * @brief Validate that a mission filename contains no path-traversal characters.
 *
 * Allowed characters: alphanumeric, underscore, hyphen, dot.
 * Explicitly rejected: '/', '\\', ':', ".." component.
 * The name must not be empty or start with '.'.
 *
 * @param[in] name  Candidate filename (no directory component).
 * @return true if the filename is safe to use as a mission file name.
 */
inline bool isValidMissionFilename(const char* name)
{
    if (name == nullptr || name[0] == '\0' || name[0] == '.')
    {
        return false;
    }

    const uint32_t len = static_cast<uint32_t>(
        strnlen(name, static_cast<size_t>(MISSION_FILENAME_MAX) + 1U));
    if (len == 0U || len > MISSION_FILENAME_MAX)
    {
        return false;
    }

    for (uint32_t i = 0; i < len; i++)
    {
        const char c = name[i];
        const bool ok = (c >= 'a' && c <= 'z')
                     || (c >= 'A' && c <= 'Z')
                     || (c >= '0' && c <= '9')
                     || c == '_' || c == '-' || c == '.';
        if (!ok)
        {
            return false;
        }
    }

    return strstr(name, "..") == nullptr;
}

/**
 * @brief Build the full LittleFS path from a validated mission filename.
 *
 * @param[in]  filename  Validated filename (no path separators).
 * @param[out] out       Buffer to receive the full path.
 * @param[in]  outSize   Byte capacity of @p out.
 * @return true if the path fit in the buffer (no truncation).
 */
inline bool buildMissionPath(const char* filename, char* out, uint32_t outSize)
{
    const int32_t written = static_cast<int32_t>(
        snprintf(out, static_cast<size_t>(outSize), "%s/%s", MISSION_DIR, filename));
    return (written > 0
            && static_cast<uint32_t>(written) < outSize);
}

/**
 * @brief Convert an AMS EngineStatus enum to a lowercase JSON-safe string.
 *
 * @param[in] st  Engine status value.
 * @return Null-terminated constant string (never NULL).
 */
inline const char* toStatusText(ares::ams::EngineStatus st)
{
    switch (st)
    {
    case ares::ams::EngineStatus::IDLE:     return "idle";
    case ares::ams::EngineStatus::LOADED:   return "loaded";
    case ares::ams::EngineStatus::RUNNING:  return "running";
    case ares::ams::EngineStatus::COMPLETE: return "complete";
    case ares::ams::EngineStatus::ERROR:    return "error";
    default:                                return "unknown";
    }
}

} // namespace api
} // namespace ares

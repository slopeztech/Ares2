/**
 * @file  storage_pure.h
 * @brief Pure validation helpers for the storage REST handler.
 *
 * Extracted from storage_handler.cpp so that host-native unit tests
 * can exercise the logic without pulling in Arduino / WiFiClient.
 */
#pragma once

#include <cstdint>
#include <cstdio>
#include <cstring>

#include "config.h"   // ares::LOG_DIR, ares::LOG_FILENAME_MAX

namespace ares
{
namespace api
{

/**
 * @brief Validate that a log filename contains no path-traversal characters.
 *
 * Allowed characters: alphanumeric, underscore, hyphen, dot.
 * Explicitly rejected: '/', '\\', ':', ".." component.
 * The name must not be empty or start with '.'.
 *
 * @param[in] name  Candidate filename (no directory component).
 * @return true if the filename is safe to use as a log file name.
 */
inline bool isValidLogFilename(const char* name)
{
    if (name == nullptr || name[0] == '\0' || name[0] == '.')
    {
        return false;
    }

    const uint32_t len = static_cast<uint32_t>(
        strnlen(name, static_cast<size_t>(LOG_FILENAME_MAX) + 1U));

    if (len > LOG_FILENAME_MAX)
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
 * @brief Build the full LittleFS path from a validated log filename.
 *
 * @param[in]  filename  Validated filename (no path separators).
 * @param[out] out       Buffer to receive the full path.
 * @param[in]  outSize   Byte capacity of @p out.
 * @return true if the path fit in the buffer (no truncation).
 */
inline bool buildLogPath(const char* filename, char* out, uint32_t outSize)
{
    const int32_t written = static_cast<int32_t>(
        snprintf(out, static_cast<size_t>(outSize), "%s/%s", LOG_DIR, filename));
    return (written > 0
            && static_cast<uint32_t>(written) < outSize);
}

} // namespace api
} // namespace ares

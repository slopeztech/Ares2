/**
 * @file  mission_script_engine_utils.cpp
 * @brief AMS engine — pure string, path, and token-parse utilities.
 *
 * Contains filesystem-safe filename validation, mission-path construction,
 * in-place string trimming, and low-level token parsers (unsigned integer,
 * float, and TC command keyword).
 *
 * Thread safety: all functions are either pure (no shared state) or static
 * member helpers called while the engine mutex is already held by the caller.
 */

#include "ams/mission_script_engine.h"
#include "ams/mission_script_engine_helpers.h"

#include "ares_assert.h"
#include "config.h"

#include <cmath>
#include <cstdlib>
#include <cstring>

namespace ares
{
namespace ams
{

using detail::isOnlyTrailingWhitespace;

// ── isSafeFileName ───────────────────────────────────────────────────────────

/**
 * @brief Verify that a script filename is safe for LittleFS storage.
 *
 * Accepts only alphanumeric characters plus @c _ @c - and @c . and rejects
 * names that contain @c .. path-traversal sequences.
 *
 * @param[in] fileName  Candidate filename (NUL-terminated).
 * @return @c true if the name is non-null, non-empty, and contains only
 *         permitted characters without a leading dot or @c .. component.
 */
bool MissionScriptEngine::isSafeFileName(const char* fileName)
{
    if (fileName == nullptr || fileName[0] == '\0' || fileName[0] == '.')
    {
        return false;
    }

    const uint32_t len = static_cast<uint32_t>(
        strnlen(fileName, ares::MISSION_FILENAME_MAX + 1U));
    if (len == 0U || len > ares::MISSION_FILENAME_MAX)
    {
        return false;
    }

    for (uint32_t i = 0U; i < len; i++)
    {
        const char c = fileName[i];
        const bool ok = (c >= 'a' && c <= 'z')
                     || (c >= 'A' && c <= 'Z')
                     || (c >= '0' && c <= '9')
                     || c == '_' || c == '-' || c == '.';
        if (!ok) { return false; }
    }

    return strstr(fileName, "..") == nullptr;
}

// ── buildMissionPath ─────────────────────────────────────────────────────────

/**
 * @brief Construct the full LittleFS path for a mission script file.
 *
 * Prepends @c MISSION_DIR and appends @c .ams if the name lacks the extension.
 *
 * @param[in]  fileName  Script base name or filename (NUL-terminated).
 * @param[out] outPath   Buffer for the resulting absolute path.
 * @param[in]  outSize   Capacity of @p outPath in bytes.
 * @return @c true if the path was written within @p outSize without truncation.
 * @pre  @p fileName is a valid safe filename (pass through isSafeFileName first).
 */
bool MissionScriptEngine::buildMissionPath(const char* fileName,
                                           char*       outPath,
                                           uint32_t    outSize)
{
    const uint32_t len = static_cast<uint32_t>(strlen(fileName));
    const bool hasExt = (len >= 4U) && (strcmp(&fileName[len - 4U], ".ams") == 0);

    int32_t written = 0;
    if (hasExt)
    {
        written = static_cast<int32_t>(snprintf(outPath, outSize,
                           "%s/%s", ares::MISSION_DIR, fileName));
    }
    else
    {
        written = static_cast<int32_t>(snprintf(outPath, outSize,
                           "%s/%s.ams", ares::MISSION_DIR, fileName));
    }

    return (written > 0) && (static_cast<uint32_t>(written) < outSize);
}

// ── trimInPlace ──────────────────────────────────────────────────────────────

/**
 * @brief Strip leading and trailing whitespace from @p text in place.
 *
 * Handles spaces, tabs, and carriage returns.  The buffer is modified
 * directly; the result is always NUL-terminated.
 *
 * @param[in,out] text  NUL-terminated mutable string to trim.  No-op if
 *                      @p text is @c nullptr.
 */
void MissionScriptEngine::trimInPlace(char* text)
{
    if (text == nullptr) { return; }

    // CERT-3: strnlen() with bounds instead of strlen() (CERT-3.3).
    uint32_t len = static_cast<uint32_t>(
        strnlen(text, ares::AMS_MAX_LINE_LEN));

    // Strip trailing whitespace.
    while (len > 0U)
    {
        const char c = text[len - 1U];
        if (c == ' ' || c == '\t' || c == '\r')
        {
            text[len - 1U] = '\0';
            len--;
        }
        else { break; }
    }

    // Skip leading whitespace.
    uint32_t start = 0U;
    while (text[start] == ' ' || text[start] == '\t')
    {
        start++;
        if (start >= len) { return; }
    }

    if (start > 0U)
    {
        const uint32_t remainLen = static_cast<uint32_t>(
            strnlen(&text[start], len - start));
        memmove(text, &text[start], remainLen + 1U);  // +1 for NUL terminator
    }
}

// ── startsWith ───────────────────────────────────────────────────────────────

/**
 * @brief Test whether @p text begins with @p prefix.
 *
 * @param[in] text    NUL-terminated string to test.
 * @param[in] prefix  NUL-terminated prefix string.
 * @return @c true if @p text starts with @p prefix (case-sensitive).
 * @pre  Neither @p text nor @p prefix is @c nullptr.
 */
bool MissionScriptEngine::startsWith(const char* text, const char* prefix)
{
    if (text == nullptr || prefix == nullptr)
    {
        ARES_ASSERT(false && "null pointer in startsWith");
        return false;
    }

    // CERT-3: strnlen() with bounds for prefix length (CERT-3.3).
    const size_t preLen = strnlen(prefix, ares::AMS_MAX_LINE_LEN);
    if (preLen == 0U) { return false; }
    return strncmp(text, prefix, preLen) == 0;
}

// ── parseUint ────────────────────────────────────────────────────────────────

/**
 * @brief Parse an unsigned 32-bit integer from a NUL-terminated string.
 *
 * Skips leading whitespace.  Fails if any non-digit character (other than
 * trailing whitespace) is found, or if the value overflows @c uint32_t.
 *
 * @param[in]  text      Input string (NUL-terminated).
 * @param[out] outValue  Parsed value on success.
 * @return @c true if a valid unsigned integer was parsed.
 */
bool MissionScriptEngine::parseUint(const char* text, uint32_t& outValue)
{
    if (text == nullptr) { return false; }

    while (*text == ' ' || *text == '\t') { text++; }
    if (*text == '\0') { return false; }

    uint64_t acc = 0U;
    bool sawDigit = false;

    while (*text >= '0' && *text <= '9')
    {
        sawDigit = true;
        acc = (acc * 10U) + static_cast<uint64_t>(*text - '0');
        if (acc > UINT32_MAX) { return false; }
        text++;
    }

    if (!sawDigit)                       { return false; }
    if (!isOnlyTrailingWhitespace(text)) { return false; }

    outValue = static_cast<uint32_t>(acc);
    return true;
}

// ── parseFloatValue ──────────────────────────────────────────────────────────

/**
 * @brief Parse a finite floating-point value from a NUL-terminated string.
 *
 * Delegates to @c strtof.  Rejects NaN, ±infinity, and strings with
 * non-whitespace trailing characters.
 *
 * @param[in]  text      Input string (NUL-terminated).
 * @param[out] outValue  Parsed value on success.
 * @return @c true if a finite floating-point number was parsed.
 */
bool MissionScriptEngine::parseFloatValue(const char* text, float& outValue)
{
    if (text == nullptr) { return false; }

    char* endPtr = nullptr;
    const float val = strtof(text, &endPtr);
    if (endPtr == text)                         { return false; }
    if (!isOnlyTrailingWhitespace(endPtr))      { return false; }
    if (!std::isfinite(val))                    { return false; }

    outValue = val;
    return true;
}

// ── parseTcCommand ───────────────────────────────────────────────────────────

/**
 * @brief Map a TC command keyword to its @c TcCommand enumerator.
 *
 * Recognised keywords (case-sensitive): @c LAUNCH, @c ABORT, @c RESET.
 *
 * @param[in]  text  Keyword string (NUL-terminated).
 * @param[out] out   Populated with the corresponding @c TcCommand on success.
 * @return @c true if @p text matched a known TC command keyword.
 */
bool MissionScriptEngine::parseTcCommand(const char* text, TcCommand& out)
{
    if (text == nullptr) { return false; }

    if (strcmp(text, "LAUNCH") == 0) { out = TcCommand::LAUNCH; return true; }
    if (strcmp(text, "ABORT")  == 0) { out = TcCommand::ABORT;  return true; }
    if (strcmp(text, "RESET")  == 0) { out = TcCommand::RESET;  return true; }

    return false;
}

} // namespace ams
} // namespace ares

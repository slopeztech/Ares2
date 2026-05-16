/**
 * @file  mission_script_engine_parser.cpp
 * @brief AMS engine — script loading, parsing, and state-graph resolution.
 *
 * Reads a .ams file from LittleFS, splits it into lines, and populates the
 * @c Program data structure used by the runtime.  Parser functions return
 * @c false and call setErrorLocked() on any syntax or semantic error.
 *
 * Thread safety: all *Locked functions must be called while the engine
 * mutex is held by the calling task (Locked suffix convention).
 */

#include "ams/mission_script_engine.h"
#include "ams/mission_script_engine_helpers.h"

#include "ares_assert.h"
#include "ares_util.h"
#include "debug/ares_log.h"

#include <cstdio>
#include <cstring>

namespace ares
{
namespace ams
{
using detail::isOnlyTrailingWhitespace;

using ares::proto::NODE_BROADCAST;
using ares::proto::NODE_ROCKET;
using ares::proto::NODE_GROUND;
using ares::proto::NODE_PAYLOAD;

static constexpr const char* TAG = "AMS";

// ── loadFromStorageLocked ───────────────────────────────────────────────────

/**
 * @brief Load, parse, and activate an AMS script from storage.
 *
 * Validates the filename, reads the file into the static script buffer,
 * invokes parseScriptLocked(), and creates the mission log file.
 *
 * @param[in] fileName  Script filename inside /missions (e.g. "flight.ams").
 * @return @c true if the file was found, read, and parsed without error.
 * @pre  Caller holds the engine mutex.  @p fileName is non-null.
 * @post On success, @c activeFile_ and @c program_ are populated.
 *       On failure, the engine error is set via setErrorLocked().
 */
bool MissionScriptEngine::loadFromStorageLocked(const char* fileName)
{
    ARES_ASSERT(mutex_ != nullptr);

    if (!isSafeFileName(fileName))
    {
        setErrorLocked("invalid script filename");
        return false;
    }

    char fullPath[ares::STORAGE_MAX_PATH] = {};
    if (!buildMissionPath(fileName, fullPath, sizeof(fullPath)))
    {
        setErrorLocked("script path too long");
        return false;
    }

    bool exists = false;
    const StorageStatus stExists = storage_.exists(fullPath, exists);
    if (stExists != StorageStatus::OK || !exists)
    {
        setErrorLocked("script file not found");
        return false;
    }

    uint32_t bytesRead = 0;
    const StorageStatus stRead = storage_.readFile(
        fullPath,
        reinterpret_cast<uint8_t*>(scriptBuffer_),
        ares::AMS_MAX_SCRIPT_BYTES,
        bytesRead);

    if (stRead != StorageStatus::OK)
    {
        setErrorLocked("failed to read script file");
        return false;
    }

    if (bytesRead == 0U)
    {
        setErrorLocked("empty script file");
        return false;
    }

    scriptBuffer_[bytesRead] = '\0';

    // Record the source filename so that parse errors log "Parse Error (FILE, line N): ...".
    // When imports are added, push/pop this alongside parseLineNum_ (see header note).
    ares::util::copyZ(parseSourceFile_, fileName, sizeof(parseSourceFile_));
    parseSourceFile_[sizeof(parseSourceFile_) - 1U] = '\0';

    if (!parseScriptLocked(scriptBuffer_, bytesRead))
    {
        return false;
    }

    ares::util::copyZ(activeFile_, fileName, sizeof(activeFile_));
    activeFile_[sizeof(activeFile_) - 1U] = '\0';

    if (!ensureLogFileLocked(activeFile_))
    {
        setErrorLocked("failed to create mission log file");
        return false;
    }

    lastError_[0] = '\0';
    return true;
}

// ── initProgramParseStateLocked ──────────────────────────────────────────────

void MissionScriptEngine::initProgramParseStateLocked()
{
    program_ = {};
    program_.apid   = ares::AMS_DEFAULT_APID;
    program_.nodeId = ares::DEFAULT_NODE_ID;
    ares::util::copyZ(program_.hkAlias,    "HK",    sizeof(program_.hkAlias));
    ares::util::copyZ(program_.eventAlias, "EVENT", sizeof(program_.eventAlias));
    ares::util::copyZ(program_.tcAlias,    "TC",    sizeof(program_.tcAlias));

    parseCurrentTask_     = 0xFFU;
    parseCurrentTaskRule_ = 0xFFU;
    parseCurrentHkSlot_   = 0xFFU;
    parseCurrentLogSlot_  = 0xFFU;
    parseSeenState_       = false;
}

// ── finalizeScriptLocked ─────────────────────────────────────────────────────

bool MissionScriptEngine::finalizeScriptLocked()
{
    parseLineNum_ = 0U;            // clear: errors after this point are runtime, not parse
    parseSourceFile_[0] = '\0';    // clear: no longer parsing a named file

    if (program_.stateCount == 0U)
    {
        setErrorLocked("script has no states");
        return false;
    }

    if (!resolveTransitionsLocked()) { return false; }

    // AMS-11: resolve 'when in' state-filter names to indices.
    if (!resolveTasksLocked())       { return false; }

    // AMS-15: evaluate formal assertions (reachability, dead states, max depth).
    if (!validateAssertionsLocked()) { return false; }

    resolvePrimaryComLocked();

    // AMS-5.1: warn about obviously shadowed transitions (first-match-wins ordering).
    warnShadowedTransitionsLocked(0ULL);

    return true;
}

// ── parseScriptLocked ────────────────────────────────────────────────────────

/**
 * @brief Parse the entire AMS script text into the @c program_ structure.
 *
 * Resets @c program_ to defaults, then iterates over every line, calling
 * parseLineLocked().  After all lines are processed, calls
 * resolveTransitionsLocked() and resolves the primary COM interface.
 *
 * @param[in] script  NUL-terminated script text buffer.
 * @param[in] length  Number of valid bytes in @p script (excluding NUL).
 * @return @c true if the script contained at least one state and no errors.
 * @pre  Caller holds the engine mutex.  @p script != nullptr.
 * @post @c program_ is fully populated on success.
 */
bool MissionScriptEngine::parseScriptLocked(const char* script, uint32_t length)
{
    ARES_ASSERT(script != nullptr);
    ARES_ASSERT(length <= ares::AMS_MAX_SCRIPT_BYTES);

    initProgramParseStateLocked();

    BlockType blockType   = BlockType::NONE;
    uint8_t currentState  = ares::AMS_MAX_STATES;

    uint32_t offset = 0;
    parseLineNum_ = 0U;
    while (offset < length)
    {
        parseLineNum_++;
        char line[ares::AMS_MAX_LINE_LEN] = {};
        if (!readNextScriptLineLocked(script, length, offset,
                                      line, sizeof(line)))
        {
            return false;
        }
        // Invariant: sentinel byte must hold before any sscanf fires.
        // readNextScriptLineLocked writes line[lineSize-1]=NUL; this assert
        // catches any future refactor that might break that guarantee.
        ARES_ASSERT(line[sizeof(line) - 1U] == '\0');
        trimInPlace(line);

        if (!parseLineLocked(line, currentState, blockType))
        {
            return false;
        }
    }

    return finalizeScriptLocked();
}

void MissionScriptEngine::resolvePrimaryComLocked()
{
    // Resolve primary COM interface: use the first COM alias in the script,
    // or fall back to comDrivers_[0] if no COM include was specified.
    primaryCom_ = nullptr;
    for (uint8_t i = 0; i < program_.aliasCount; i++)
    {
        if (program_.aliases[i].kind == PeripheralKind::COM)
        {
            const uint8_t idx = program_.aliases[i].driverIdx;
            if (idx < comCount_ && comDrivers_[idx].iface != nullptr)
            {
                primaryCom_ = comDrivers_[idx].iface;
            }
            break;
        }
    }
    if (primaryCom_ == nullptr && comCount_ > 0U && comDrivers_[0].iface != nullptr)
    {
        primaryCom_ = comDrivers_[0].iface;  // fallback: first compiled-in COM
    }
}

// ── readNextScriptLineLocked ─────────────────────────────────────────────────

/**
 * @brief Extract the next newline-delimited line from the script buffer.
 *
 * Reads characters from @p script starting at @p offset until a newline or
 * end-of-buffer is reached.  Advances @p offset past the newline.
 *
 * @param[in]     script    Script text buffer (NUL-terminated).
 * @param[in]     length    Total valid bytes in @p script.
 * @param[in,out] offset    Read position; advanced past the consumed line.
 * @param[out]    line      Buffer to write the extracted line (NUL-terminated).
 * @param[in]     lineSize  Capacity of @p line in bytes.
 * @return @c true on success; @c false if the line exceeded @c AMS_MAX_LINE_LEN.
 * @pre  Caller holds the engine mutex.  @p script and @p line are non-null.
 */
bool MissionScriptEngine::readNextScriptLineLocked(const char* script,
                                                   uint32_t    length,
                                                   uint32_t&   offset,
                                                   char*       line,
                                                   uint32_t    lineSize)
{
    ARES_ASSERT(script   != nullptr);
    ARES_ASSERT(line     != nullptr);
    ARES_ASSERT(lineSize  > 1U);

    // AMS-8.5 / CERT-STR32: write the sentinel byte unconditionally so this
    // function is safe regardless of whether the caller initialised the buffer.
    // Any sscanf operating on 'line' after this call is guaranteed to find
    // a NUL before reading past the end of the array.
    line[lineSize - 1U] = '\0';

    uint32_t lineLen    = 0;
    uint32_t totalLen   = 0;  // all chars on this line, including overflow
    bool     lineTooLong = false;

    while (offset < length && script[offset] != '\n')
    {
        if (lineLen < (lineSize - 1U))
        {
            line[lineLen] = script[offset];
            lineLen++;
        }
        else
        {
            lineTooLong = true;
        }
        totalLen++;
        offset++;
    }

    if (offset < length && script[offset] == '\n')
    {
        offset++;
    }

    if (lineTooLong)
    {
        // AMS-8.5: lines that exceed the buffer capacity are rejected at parse
        // time with an explicit error so no directive is silently truncated.
        static char errBuf[ares::AMS_MAX_ERROR_TEXT] = {};
        snprintf(errBuf, sizeof(errBuf),
                 "line too long: %u chars (max %u)",
                 static_cast<unsigned>(totalLen),
                 static_cast<unsigned>(lineSize - 1U));
        setErrorLocked(errBuf);
        return false;
    }

    line[lineLen] = '\0';
    return true;
}

// ── parseLineLocked ──────────────────────────────────────────────────────────

/**
 * @brief Dispatch a single trimmed script line to the appropriate sub-parser.
 *
 * Handles blank lines, comments, closing braces, @c include, @c pus.*,
 * and @c state directives at the top level.  Delegates all state-scoped
 * content to parseStateScopedLineLocked().
 *
 * @param[in]     line          Trimmed, NUL-terminated line text.
 * @param[in,out] currentState  Index of the currently open state block.
 * @param[in,out] blockType     Active sub-block context within the state.
 * @return @c true on success; @c false with the engine error set on failure.
 * @pre  Caller holds the engine mutex.  @p line != nullptr.
 */
bool MissionScriptEngine::parseLineLocked(const char* line,
                                          uint8_t&    currentState,
                                          BlockType&  blockType)
{
    ARES_ASSERT(line != nullptr);

    if (line[0] == '\0' || startsWith(line, "//") || line[0] == '#')
    {
        return true;
    }

    bool handled = false;
    if (!parseTopLevelDirectiveLocked(line, currentState, blockType, handled))
    {
        return false;
    }
    if (handled) { return true; }

    if (!parseNonStateBlockLineLocked(line, blockType, handled))
    {
        return false;
    }
    if (handled) { return true; }

    if (currentState >= program_.stateCount)
    {
        setErrorLocked("statement outside state block");
        return false;
    }

    StateDef& st = program_.states[currentState];
    return parseStateScopedLineLocked(line, st, blockType);
}

bool MissionScriptEngine::parseTopLevelDirectiveLocked(const char* line,
                                                       uint8_t&    currentState,
                                                       BlockType&  blockType,
                                                       bool&       handled)
{
    handled = true;
    if (strcmp(line, "}") == 0)
    {
        blockType = BlockType::NONE;
        return true;
    }
    // Pre-state directives: include, radio.config, pus.*, var, const.
    bool preHandled = false;
    if (!parsePreStateDirectiveLocked(line, preHandled)) { return false; }
    if (preHandled) { return true; }

    if (startsWith(line, "state "))
    {
        blockType              = BlockType::NONE;
        parseCurrentTask_      = 0xFFU;
        parseCurrentTaskRule_  = 0xFFU;
        parseCurrentHkSlot_    = 0xFFU;
        parseCurrentLogSlot_   = 0xFFU;
        parseSeenState_        = true;
        return parseStateLineLocked(line, currentState);
    }
    if (startsWith(line, "task "))
    {
        blockType             = BlockType::TASK;
        parseCurrentTaskRule_ = 0xFFU;
        return parseTaskLineLocked(line);
    }
    if (strcmp(line, "assert:") == 0)
    {
        blockType             = BlockType::ASSERT;
        parseCurrentTask_     = 0xFFU;
        parseCurrentTaskRule_ = 0xFFU;
        return true;
    }
    handled = false;
    return true;
}

bool MissionScriptEngine::parseNonStateBlockLineLocked(const char* line,
                                                       BlockType&  blockType,
                                                       bool&       handled)
{
    handled = true;
    if (blockType == BlockType::TASK || blockType == BlockType::TASK_IF)
    {
        return parseTaskScopedLineLocked(line, blockType);
    }
    if (blockType == BlockType::ASSERT)
    {
        return parseAssertLineLocked(line);
    }
    handled = false;
    return true;
}


} // namespace ams
} // namespace ares
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

    if (!parseScriptLocked(scriptBuffer_, bytesRead))
    {
        return false;
    }

    strncpy(activeFile_, fileName, sizeof(activeFile_) - 1U);
    activeFile_[sizeof(activeFile_) - 1U] = '\0';

    if (!ensureLogFileLocked(activeFile_))
    {
        setErrorLocked("failed to create mission log file");
        return false;
    }

    lastError_[0] = '\0';
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
bool MissionScriptEngine::parseScriptLocked(const char* script, uint32_t length) // NOLINT(readability-function-size)
{
    ARES_ASSERT(script != nullptr);
    ARES_ASSERT(length <= ares::AMS_MAX_SCRIPT_BYTES);

    program_ = {};
    program_.apid   = ares::AMS_DEFAULT_APID;
    program_.nodeId = ares::DEFAULT_NODE_ID;
    strncpy(program_.hkAlias,    "HK",    sizeof(program_.hkAlias)    - 1U);
    strncpy(program_.eventAlias, "EVENT", sizeof(program_.eventAlias) - 1U);
    strncpy(program_.tcAlias,    "TC",    sizeof(program_.tcAlias)    - 1U);

    parseCurrentTask_     = 0xFFU;
    parseCurrentTaskRule_ = 0xFFU;
    parseCurrentHkSlot_   = 0xFFU;
    parseCurrentLogSlot_  = 0xFFU;

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
    parseLineNum_ = 0U;  // clear: errors after this point are runtime, not parse

    if (program_.stateCount == 0U)
    {
        setErrorLocked("script has no states");
        return false;
    }

    if (!resolveTransitionsLocked())
    {
        return false;
    }

    // AMS-11: resolve 'when in' state-filter names to indices.
    if (!resolveTasksLocked())
    {
        return false;
    }

    // AMS-15: evaluate formal assertions (reachability, dead states, max depth).
    if (!validateAssertionsLocked())
    {
        return false;
    }

    resolvePrimaryComLocked();

    return true;
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

    if (line[0] == '\0' || startsWith(line, "//"))
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

bool MissionScriptEngine::parsePusApidDirectiveLocked(const char* line)
{
    const char* eq = strchr(line, '=');
    if (eq == nullptr)
    {
        setErrorLocked("invalid pus.apid syntax");
        return false;
    }

    uint32_t apid = 0;
    if (!parseUint(eq + 1, apid) || apid > 2047U)
    {
        setErrorLocked("invalid pus.apid value");
        return false;
    }

    uint8_t nodeId = 0;
    if (!mapApidToNode(static_cast<uint16_t>(apid), nodeId))
    {
        setErrorLocked("unsupported APID for ARES node mapping");
        return false;
    }

    program_.apid   = static_cast<uint16_t>(apid);
    program_.nodeId = nodeId;
    return true;
}

// ── parsePusServiceDirectiveLocked ───────────────────────────────────────────

/**
 * @brief Parse a @c pus.service directive and assign a custom alias.
 *
 * Syntax: @c "pus.service N as ALIAS"
 *
 * Supported service numbers: 1 (TC verification), 3 (HK), 5 (event).
 * The reserved words TIME and LOG may not be used as aliases.
 *
 * @param[in] line  Script line starting with @c "pus.service ".
 * @return @c true if the directive was parsed and the alias stored.
 * @pre  Caller holds the engine mutex.
 */
bool MissionScriptEngine::parsePusServiceDirectiveLocked(const char* line)
{
    uint32_t svcNum = 0U;
    char     alias[16] = {};
    // cppcheck-suppress [cert-err34-c]
    const int32_t n = static_cast<int32_t>(sscanf(line, "pus.service %" SCNu32 " as %15s", &svcNum, alias)); // NOLINT(bugprone-unchecked-string-to-number-conversion)
    if (n != 2 || alias[0] == '\0')
    {
        setErrorLocked("invalid pus.service syntax (expected: pus.service N as ALIAS)");
        return false;
    }
    if (svcNum != 1U && svcNum != 3U && svcNum != 5U)
    {
        setErrorLocked("pus.service: unsupported service number (valid: 1, 3, 5)");
        return false;
    }
    if (strcmp(alias, "TIME") == 0 || strcmp(alias, "LOG") == 0)
    {
        setErrorLocked("pus.service: alias conflicts with reserved keyword");
        return false;
    }
    switch (svcNum)
    {
    case 1U:
        strncpy(program_.tcAlias, alias, sizeof(program_.tcAlias) - 1U);
        program_.tcAlias[sizeof(program_.tcAlias) - 1U] = '\0';
        break;
    case 3U:
        strncpy(program_.hkAlias, alias, sizeof(program_.hkAlias) - 1U);
        program_.hkAlias[sizeof(program_.hkAlias) - 1U] = '\0';
        break;
    case 5U:
        strncpy(program_.eventAlias, alias, sizeof(program_.eventAlias) - 1U);
        program_.eventAlias[sizeof(program_.eventAlias) - 1U] = '\0';
        break;
    default: break;
    }
    return true;
}

// ── parseRadioConfigLineLocked ───────────────────────────────────────────────

/**
 * @brief Parse a @c radio.config directive and store the override into
 *        @c program_.radioConfig (AMS-4.15).
 *
 * Syntax: @c "radio.config PARAM_NAME = VALUE"
 *
 * Supported @c PARAM_NAME tokens and their valid ranges:
 * | Token              | ConfigParamId         | Range (inclusive)          |
 * |--------------------|----------------------|----------------------------|
 * | telem_interval     | TELEM_INTERVAL_MS    | 100 – 60000 (ms)           |
 * | monitor.alt.high   | MONITOR_ALT_HIGH_M   |    0 – 15000 (m)           |
 * | monitor.alt.low    | MONITOR_ALT_LOW_M    | –500 – 1000  (m)           |
 * | monitor.accel.max  | MONITOR_ACCEL_HIGH   |    0 – 1000  (m/s²)        |
 * | monitor.temp.high  | MONITOR_TEMP_HIGH_C  |  –40 – 150   (°C)          |
 * | monitor.temp.low   | MONITOR_TEMP_LOW_C   | –100 – 50    (°C)          |
 *
 * @param[in] line  Script line starting with @c "radio.config ".
 * @return @c true if the directive was parsed and stored without error.
 * @pre  Caller holds the engine mutex.
 */
bool MissionScriptEngine::parseRadioConfigLineLocked(const char* line)
{
    // Parse:  radio.config PARAM_NAME = VALUE
    char  key[32] = {};
    float value   = 0.0F;
    // cppcheck-suppress [cert-err34-c]
    // MISRA-1: sscanf returns int (third-party API); cast to int32_t on capture.
    const int32_t n = static_cast<int32_t>(sscanf(line, "radio.config %31s = %f", key, &value)); // NOLINT(bugprone-unchecked-string-to-number-conversion)
    if (n != 2 || key[0] == '\0')
    {
        setErrorLocked("invalid radio.config syntax (expected: radio.config PARAM = VALUE)");
        return false;
    }

    // Resolve token → ConfigParamId + bounds.
    using Id = ares::proto::ConfigParamId;

    /** Maps an AMS keyword to its wire ConfigParamId and allowed value range. */
    struct ParamSpec
    {
        const char* token;   ///< AMS keyword.
        Id          id;      ///< Wire-protocol identifier.
        float       minVal;  ///< Inclusive lower bound.
        float       maxVal;  ///< Inclusive upper bound.
    };

    // MISRA-7: named constants from config.h — single authoritative source shared
    // with RadioDispatcher::configParams_[] (DRY, AMS-4.15).
    static const ParamSpec kSpecs[] = {
        { "telem_interval",    Id::TELEM_INTERVAL_MS,
          static_cast<double>(ares::TELEMETRY_INTERVAL_MIN),
          static_cast<double>(ares::TELEMETRY_INTERVAL_MAX) },
        { "monitor.alt.high",  Id::MONITOR_ALT_HIGH_M,  0.0f,  ares::MONITOR_ALT_HIGH_MAX_M  },
        { "monitor.alt.low",   Id::MONITOR_ALT_LOW_M,   ares::MONITOR_ALT_LOW_MIN_M,  ares::MONITOR_ALT_LOW_MAX_M  },
        { "monitor.accel.max", Id::MONITOR_ACCEL_HIGH,  0.0f,  ares::MONITOR_ACCEL_HIGH_MAX  },
        { "monitor.temp.high", Id::MONITOR_TEMP_HIGH_C, ares::MONITOR_TEMP_HIGH_MIN_C, ares::MONITOR_TEMP_HIGH_MAX_C },
        { "monitor.temp.low",  Id::MONITOR_TEMP_LOW_C,  ares::MONITOR_TEMP_LOW_MIN_C,  ares::MONITOR_TEMP_LOW_MAX_C  },
    };

    const ParamSpec* found = nullptr;
    for (uint8_t i = 0U; i < static_cast<uint8_t>(sizeof(kSpecs) / sizeof(kSpecs[0])); ++i)
    {
        if (strcmp(key, kSpecs[i].token) == 0)
        {
            found = &kSpecs[i];
            break;
        }
    }
    if (found == nullptr)
    {
        setErrorLocked("radio.config: unknown parameter name");
        return false;
    }
    if (value < found->minVal || value > found->maxVal)
    {
        setErrorLocked("radio.config: value out of allowed range");
        return false;
    }

    // Store override — index = (id - FIRST).
    // ARES-MISRA-DEV-002: outer cast suppresses integral promotion from uint8_t subtraction.
    const uint8_t idx = static_cast<uint8_t>(
        static_cast<uint8_t>(found->id) - static_cast<uint8_t>(Id::FIRST));
    program_.radioConfig[idx].id    = found->id;
    program_.radioConfig[idx].value = value;
    program_.radioConfig[idx].set   = true;

    return true;
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
    if (startsWith(line, "include ")) { return parseIncludeLineLocked(line); }
    if (startsWith(line, "radio.config ")) { return parseRadioConfigLineLocked(line); }
    if (startsWith(line, "pus.service ")) { return parsePusServiceDirectiveLocked(line); }
    if (startsWith(line, "pus.apid")) { return parsePusApidDirectiveLocked(line); }
    if (startsWith(line, "state "))
    {
        blockType              = BlockType::NONE;
        parseCurrentTask_      = 0xFFU;
        parseCurrentTaskRule_  = 0xFFU;
        parseCurrentHkSlot_    = 0xFFU;
        parseCurrentLogSlot_   = 0xFFU;
        return parseStateLineLocked(line, currentState);
    }
    if (startsWith(line, "var "))   { return parseVarLineLocked(line); }
    if (startsWith(line, "const ")) { return parseConstLineLocked(line); }
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

// ── parseStateScopedLineLocked ────────────────────────────────────────────────

/**
 * @brief Parse a line that belongs to the body of an open state block.
 *
 * Recognises block-header keywords (@c on_enter:, @c on_error:,
 * @c conditions:, @c every, @c log_every, @c priorities, @c HK.report,
 * @c LOG.report, @c transition) and block-content lines (field entries,
 * condition expressions, EVENT.* directives).
 *
 * @param[in]     line       Trimmed, NUL-terminated line.
 * @param[in,out] st         State being populated.
 * @param[in,out] blockType  Active sub-block context within the state.
 * @return @c true on success.
 * @pre  Caller holds the engine mutex.  @p line != nullptr.
 */
bool MissionScriptEngine::parseStateScopedLineLocked(const char* line,
                                                     StateDef&   st,
                                                     BlockType&  blockType)
{
    ARES_ASSERT(line != nullptr);
    ARES_ASSERT(st.hkFieldCount  <= ares::AMS_MAX_HK_FIELDS);
    ARES_ASSERT(st.logFieldCount <= ares::AMS_MAX_HK_FIELDS);

    bool handled = false;
    if (!parseStateBlockHeaderLocked(line, st, blockType, handled))
    {
        return false;
    }
    if (handled) { return true; }

    if (!parseStateBlockContentLocked(line, st, blockType, handled))
    {
        return false;
    }
    if (handled) { return true; }

    {
        char evtPrefix[20] = {};
        snprintf(evtPrefix, sizeof(evtPrefix), "%s.", program_.eventAlias);
        if (startsWith(line, evtPrefix)) { return parseEventLineLocked(line, st); }
    }

    setErrorLocked("unsupported statement");
    return false;
}

bool MissionScriptEngine::parseStateReportDirectivesLocked(const char* line,
                                                            const StateDef& st,
                                                            BlockType&  blockType,
                                                            bool&       matched)
{
    matched = false;
    {
        char hkReport[24] = {};
        snprintf(hkReport, sizeof(hkReport), "%s.report", program_.hkAlias);
        if (startsWith(line, hkReport))
        {
            if (!st.hasHkEvery)
            {
                setErrorLocked("HK.report requires every block");
                return false;
            }
            blockType = BlockType::HK;
            matched   = true;
            return true;
        }
    }
    if (startsWith(line, "LOG.report"))
    {
        if (!st.hasLogEvery)
        {
            setErrorLocked("LOG.report requires log_every block");
            return false;
        }
        blockType = BlockType::LOG;
        matched   = true;
        return true;
    }
    return true;
}

bool MissionScriptEngine::parseStateRateDirectivesLocked(const char* line,
                                                          StateDef&   st,
                                                          BlockType&  blockType,
                                                          bool&       matched)
{
    matched = false;
    if (startsWith(line, "every "))
    {
        blockType = BlockType::NONE;
        matched   = true;
        return parseEveryLineLocked(line, st);
    }
    if (startsWith(line, "log_every "))
    {
        blockType = BlockType::NONE;
        matched   = true;
        return parseLogEveryLineLocked(line, st);
    }
    if (startsWith(line, "priorities "))
    {
        blockType = BlockType::NONE;
        matched   = true;
        return parsePrioritiesLineLocked(line, st);
    }
    return true;
}

bool MissionScriptEngine::parseStateBlockHeaderLocked(const char* line,
                                                      StateDef&   st,
                                                      BlockType&  blockType,
                                                      bool&       handled)
{
    handled = true;
    if (startsWith(line, "on_enter:"))   { blockType = BlockType::ON_ENTER;   return true; }
    if (startsWith(line, "on_exit:"))    { blockType = BlockType::ON_EXIT;    return true; }
    if (startsWith(line, "on_error:"))   { blockType = BlockType::ON_ERROR;   return true; }
    if (startsWith(line, "on_timeout ")) { return parseOnTimeoutHeaderLocked(line, st, blockType); }
    if (startsWith(line, "conditions:")) { blockType = BlockType::CONDITIONS; return true; }

    bool rateMatched = false;
    if (!parseStateRateDirectivesLocked(line, st, blockType, rateMatched)) { return false; }
    if (rateMatched) { return true; }

    bool reportMatched = false;
    if (!parseStateReportDirectivesLocked(line, st, blockType, reportMatched)) { return false; }
    if (reportMatched) { return true; }
    if (startsWith(line, "transition to "))
    {
        // AMS-4.10.2: inside an on_error: block the 'transition to' directive
        // is an error-recovery transition, NOT a regular state transition.
        // Defer to parseStateBlockContentLocked which calls
        // parseOnErrorTransitionLineLocked in that case.
        if (blockType == BlockType::ON_ERROR)
        {
            handled = false;
            return true;
        }
        blockType = BlockType::NONE;
        return parseTransitionLineLocked(line, st);
    }
    if (startsWith(line, "fallback transition to "))
    {
        blockType = BlockType::NONE;
        return parseFallbackTransitionLineLocked(line, st);
    }

    handled = false;
    return true;
}

bool MissionScriptEngine::parseStateBlockContentLocked(const char* line, // NOLINT(readability-function-size)
                                                       StateDef&   st,
                                                       BlockType   blockType,
                                                       bool&       handled)
{
    handled = true;
    if (blockType == BlockType::CONDITIONS) { return parseConditionScopedLineLocked(line, st); }
    if (blockType == BlockType::ON_ERROR)
    {
        char evtPrefix[20] = {};
        snprintf(evtPrefix, sizeof(evtPrefix), "%s.", program_.eventAlias);
        if (startsWith(line, evtPrefix)) { return parseOnErrorEventLineLocked(line, st); }
        if (startsWith(line, "transition to ")) { return parseOnErrorTransitionLineLocked(line, st); }
        setErrorLocked("only EVENT.* or 'transition to' allowed inside on_error");
        return false;
    }
    if (blockType == BlockType::ON_ENTER)
    {
        char evtPrefix[20] = {};
        snprintf(evtPrefix, sizeof(evtPrefix), "%s.", program_.eventAlias);
        if (startsWith(line, evtPrefix)) { return parseEventLineLocked(line, st); }
        if (startsWith(line, "set ")) { return parseSetActionLineLocked(line, st); }
        if (startsWith(line, "PULSE.fire ")) { return parsePulseFireLineLocked(line, st); }  // AMS-4.17
        setErrorLocked("only EVENT.*, set and PULSE.fire are allowed inside on_enter");
        return false;
    }
    if (blockType == BlockType::ON_EXIT)
    {
        char evtPrefix[20] = {};
        snprintf(evtPrefix, sizeof(evtPrefix), "%s.", program_.eventAlias);
        if (startsWith(line, evtPrefix))
        {
            return parseOnExitEventLineLocked(line, st);
        }
        if (startsWith(line, "set "))
        {
            if (st.onExitSetCount >= ares::AMS_MAX_SET_ACTIONS)
            {
                setErrorLocked("too many set actions in on_exit block");
                return false;
            }
            if (!parseSetActionCoreLocked(line, st.onExitSetActions[st.onExitSetCount]))
            {
                return false;
            }
            st.onExitSetCount++;
            return true;
        }
        if (startsWith(line, "transition to "))
        {
            setErrorLocked("transition is not allowed inside on_exit");
            return false;
        }
        setErrorLocked("only EVENT.* and set are allowed inside on_exit");
        return false;
    }
    if (blockType == BlockType::ON_TIMEOUT)
    {
        char evtPrefix[20] = {};
        snprintf(evtPrefix, sizeof(evtPrefix), "%s.", program_.eventAlias);
        if (startsWith(line, evtPrefix)) { return parseOnTimeoutEventLineLocked(line, st); }
        if (startsWith(line, "transition to ")) { return parseOnTimeoutTransitionLineLocked(line, st); }
        setErrorLocked("only EVENT.* and 'transition to' allowed inside on_timeout");
        return false;
    }
    if (blockType == BlockType::HK)
    {
        // AMS-4.3.1: fill the active HK slot, and mirror into legacy slot-0 fields.
        if (parseCurrentHkSlot_ >= st.hkSlotCount)
        {
            setErrorLocked("HK field outside every block");
            return false;
        }
        HkSlot& slot = st.hkSlots[parseCurrentHkSlot_];
        if (!parseFieldLineLocked(line, slot.fields, slot.fieldCount, "HK")) { return false; }
        // Keep legacy fields in sync with slot-0 for single-slot code paths.
        if (parseCurrentHkSlot_ == 0U)
        {
            st.hkFieldCount = slot.fieldCount;
            if (slot.fieldCount > 0U)
            {
                st.hkFields[slot.fieldCount - 1U] = slot.fields[slot.fieldCount - 1U];
            }
        }
        return true;
    }
    if (blockType == BlockType::LOG)
    {
        // AMS-4.3.1: fill the active LOG slot, and mirror into legacy slot-0 fields.
        if (parseCurrentLogSlot_ >= st.logSlotCount)
        {
            setErrorLocked("LOG field outside log_every block");
            return false;
        }
        HkSlot& slot = st.logSlots[parseCurrentLogSlot_];
        if (!parseFieldLineLocked(line, slot.fields, slot.fieldCount, "LOG")) { return false; }
        // Keep legacy fields in sync with slot-0 for single-slot code paths.
        if (parseCurrentLogSlot_ == 0U)
        {
            st.logFieldCount = slot.fieldCount;
            if (slot.fieldCount > 0U)
            {
                st.logFields[slot.fieldCount - 1U] = slot.fields[slot.fieldCount - 1U];
            }
        }
        return true;
    }
    handled = false;
    return true;
}

// ── parseIncludeLineLocked ────────────────────────────────────────────────────

/**
 * @brief Parse an @c include directive and register the resulting alias.
 *
 * Syntax: @c "include \<MODEL\> as \<ALIAS\>"
 *
 * Looks up @p MODEL in each compiled-in driver registry (GPS, BARO, COM, IMU).
 * On success, adds an entry to @c program_.aliases[] so that subsequent
 * @c ALIAS.field expressions in the script are valid.
 *
 * @param[in] line  Script line starting with @c "include ".
 * @return @c true if the model was found and the alias registered.
 * @pre  Caller holds the engine mutex.
 * @post @c program_.aliasCount is incremented on success.
 */
bool MissionScriptEngine::lookupModelInDriversLocked(
    const char* model, PeripheralKind& kind, uint8_t& driverIdx) const
{
    driverIdx = 0xFFU;
    for (uint8_t i = 0; i < gpsCount_ && driverIdx == 0xFFU; i++)
    {
        if (gpsDrivers_[i].model != nullptr
            && strcmp(gpsDrivers_[i].model, model) == 0)
        { kind = PeripheralKind::GPS; driverIdx = i; }
    }
    for (uint8_t i = 0; i < baroCount_ && driverIdx == 0xFFU; i++)
    {
        if (baroDrivers_[i].model != nullptr
            && strcmp(baroDrivers_[i].model, model) == 0)
        { kind = PeripheralKind::BARO; driverIdx = i; }
    }
    for (uint8_t i = 0; i < comCount_ && driverIdx == 0xFFU; i++)
    {
        if (comDrivers_[i].model != nullptr
            && strcmp(comDrivers_[i].model, model) == 0)
        { kind = PeripheralKind::COM; driverIdx = i; }
    }
    for (uint8_t i = 0; i < imuCount_ && driverIdx == 0xFFU; i++)
    {
        if (imuDrivers_[i].model != nullptr
            && strcmp(imuDrivers_[i].model, model) == 0)
        { kind = PeripheralKind::IMU; driverIdx = i; }
    }
    return (driverIdx != 0xFFU);
}

bool MissionScriptEngine::parseIncludeOptionalsLocked(const char* line, AliasEntry& ae)
{
    const char* retryP = strstr(line, " retry=");
    if (retryP != nullptr)
    {
        uint32_t retries = 0U;
        if (sscanf(retryP + 7, "%" SCNu32, &retries) != 1  // NOLINT(bugprone-unchecked-string-to-number-conversion)
            || retries == 0U
            || retries > static_cast<uint32_t>(ares::AMS_MAX_SENSOR_RETRY))
        {
            setErrorLocked("include retry= value must be 1-AMS_MAX_SENSOR_RETRY");
            return false;
        }
        ae.retryCount = static_cast<uint8_t>(retries);
    }

    const char* toP = strstr(line, " timeout=");
    if (toP != nullptr)
    {
        const char* numStart = toP + 9;
        const char* msPtr    = strstr(numStart, "ms");
        if (msPtr == nullptr || msPtr == numStart)
        {
            setErrorLocked("include timeout= must be of the form Nms");
            return false;
        }
        const ptrdiff_t numLen = msPtr - numStart;
        if (numLen <= 0 || numLen >= 16)
        {
            setErrorLocked("include timeout= numeric value out of range");
            return false;
        }
    }
    return true;
}

bool MissionScriptEngine::parseIncludeLineLocked(const char* line)
{
    char model[16] = {};
    char alias[16] = {};
    // cppcheck-suppress [cert-err34-c]
    const int32_t parsed = static_cast<int32_t>(sscanf(line, "include %15s as %15s", model, alias));
    if (parsed != 2 || model[0] == '\0' || alias[0] == '\0')
    {
        setErrorLocked("invalid include syntax (expected: include <MODEL> as <ALIAS>)");
        return false;
    }

    // TIME is a built-in alias — it cannot be redefined.
    if (strcmp(alias, "TIME") == 0)
    {
        setErrorLocked("TIME is a reserved alias; do not use it with include");
        return false;
    }

    // Duplicate alias check.
    for (uint8_t i = 0; i < program_.aliasCount; i++)
    {
        if (strcmp(program_.aliases[i].alias, alias) == 0)
        {
            char msg[48] = {};
            snprintf(msg, sizeof(msg), "duplicate alias: %s", alias);
            setErrorLocked(msg);
            return false;
        }
    }

    if (program_.aliasCount >= ares::AMS_MAX_INCLUDES)
    {
        setErrorLocked("too many include directives (max AMS_MAX_INCLUDES)");
        return false;
    }

    // Look up MODEL in each driver kind registry.
    PeripheralKind kind      = PeripheralKind::GPS;
    uint8_t        driverIdx = 0xFFU;
    if (!lookupModelInDriversLocked(model, kind, driverIdx))
    {
        char msg[64] = {};
        snprintf(msg, sizeof(msg), "model '%s' not found in compiled-in drivers", model);
        setErrorLocked(msg);
        return false;
    }

    AliasEntry& ae = program_.aliases[program_.aliasCount++];
    strncpy(ae.alias, alias, sizeof(ae.alias) - 1U);
    ae.alias[sizeof(ae.alias) - 1U] = '\0';
    strncpy(ae.model, model, sizeof(ae.model) - 1U);
    ae.model[sizeof(ae.model) - 1U] = '\0';
    ae.kind      = kind;
    ae.driverIdx = driverIdx;
    ae.retryCount = 0U;

    if (!parseIncludeOptionalsLocked(line, ae)) { return false; }

    LOG_I(TAG, "include: alias=%s model=%s kind=%u idx=%u retry=%u",
          ae.alias, ae.model,
          static_cast<uint32_t>(ae.kind),
          static_cast<uint32_t>(ae.driverIdx),
          static_cast<uint32_t>(ae.retryCount));
    return true;
}

// ── parseStateLineLocked ─────────────────────────────────────────────────────

/**
 * @brief Parse a @c state declaration and open a new state block.
 *
 * Syntax: @c "state \<NAME\>:"
 *
 * @param[in]  line          Line starting with @c "state ".
 * @param[out] currentState  Set to the index of the newly opened state.
 * @return @c true if the state was created successfully.
 * @pre  Caller holds the engine mutex.
 * @post @c program_.stateCount is incremented on success.
 */
bool MissionScriptEngine::parseStateLineLocked(const char* line,
                                               uint8_t&    currentState)
{
    if (program_.stateCount >= ares::AMS_MAX_STATES)
    {
        setErrorLocked("too many states");
        return false;
    }

    char stateName[ares::AMS_MAX_STATE_NAME] = {};
    const int32_t n = static_cast<int32_t>(sscanf(line, "state %15[^:]:", stateName));
    if (n != 1)
    {
        setErrorLocked("invalid state syntax");
        return false;
    }

    currentState = program_.stateCount;
    StateDef& st = program_.states[currentState];
    strncpy(st.name, stateName, sizeof(st.name) - 1U);
    st.name[sizeof(st.name) - 1U] = '\0';
    program_.stateCount++;
    return true;
}

// ── parseEventLineLocked ─────────────────────────────────────────────────────

/**
 * @brief Parse an @c EVENT.* on_enter directive.
 *
 * Syntax: @c "EVENT.\<verb\> \"\<text\>\""
 *
 * Accepted verbs: @c info, @c warning, @c error.
 *
 * @param[in]  line  Script line starting with @c "EVENT.".
 * @param[out] st    State to populate with the on-enter event.
 * @return @c true if the event directive was parsed and stored.
 * @pre  Caller holds the engine mutex.
 */
bool MissionScriptEngine::parseEventLineLocked(const char* line, StateDef& st)
{
    char verb[8]  = {};
    char text[ares::AMS_MAX_EVENT_TEXT] = {};
    const char* dotPos = strchr(line, '.');
    if (dotPos == nullptr)
    {
        setErrorLocked("invalid EVENT syntax");
        return false;
    }
    const int32_t n = static_cast<int32_t>(sscanf(dotPos + 1, "%7[^ ] \"%63[^\"]\"", verb, text));
    if (n != 2)
    {
        setErrorLocked("invalid EVENT syntax");
        return false;
    }

    st.hasOnEnterEvent = true;
    strncpy(st.onEnterText, text, sizeof(st.onEnterText) - 1U);
    st.onEnterText[sizeof(st.onEnterText) - 1U] = '\0';

    if (strcmp(verb, "info")    == 0) { st.onEnterVerb = EventVerb::INFO;  return true; }
    if (strcmp(verb, "warning") == 0) { st.onEnterVerb = EventVerb::WARN;  return true; }
    if (strcmp(verb, "error")   == 0) { st.onEnterVerb = EventVerb::ERROR; return true; }

    setErrorLocked("unknown EVENT verb");
    return false;
}

// ── parseEveryLineLocked ─────────────────────────────────────────────────────

/**
 * @brief Parse an @c every directive that sets the HK telemetry interval.
 *
 * Syntax: @c "every \<N\>ms:"
 *
 * Enforces the minimum interval @c TELEMETRY_INTERVAL_MIN (APUS-19.3).
 *
 * @param[in]  line  Script line starting with @c "every ".
 * @param[out] st    State to update with the HK interval.
 * @return @c true if a valid interval was parsed.
 * @pre  Caller holds the engine mutex.
 */
bool MissionScriptEngine::parseEveryLineLocked(const char* line, StateDef& st)
{
    const char* const prefix    = "every ";
    const size_t      prefixLen = strlen(prefix);
    if (strncmp(line, prefix, prefixLen) != 0)
    {
        setErrorLocked("invalid every syntax");
        return false;
    }

    const char* valueStart = line + prefixLen;
    while (*valueStart == ' ' || *valueStart == '\t') { valueStart++; }

    const char* marker = strstr(valueStart, "ms:");
    if (marker == nullptr)
    {
        setErrorLocked("invalid every period");
        return false;
    }
    if (!isOnlyTrailingWhitespace(marker + 3))
    {
        setErrorLocked("invalid every period suffix");
        return false;
    }

    const ptrdiff_t rawLen = marker - valueStart;
    if (rawLen <= 0 || rawLen >= 16)
    {
        setErrorLocked("invalid every period length");
        return false;
    }

    char valueBuf[16] = {};
    memcpy(valueBuf, valueStart, static_cast<size_t>(rawLen));
    valueBuf[static_cast<size_t>(rawLen)] = '\0';
    trimInPlace(valueBuf);

    uint32_t everyMs = 0;
    if (!parseUint(valueBuf, everyMs) || everyMs < ares::TELEMETRY_INTERVAL_MIN)
    {
        setErrorLocked("HK interval must be >= 100 ms (APUS-19.3)");
        return false;
    }

    st.hasHkEvery   = true;
    st.hkEveryMs    = everyMs;
    st.hkFieldCount = 0;

    // AMS-4.3.1: allocate a new HK slot for this every directive.
    if (st.hkSlotCount >= ares::AMS_MAX_HK_SLOTS)
    {
        setErrorLocked("too many every blocks in state (max AMS_MAX_HK_SLOTS)");
        return false;
    }
    const uint8_t slotIdx = st.hkSlotCount;
    st.hkSlots[slotIdx].everyMs    = everyMs;
    st.hkSlots[slotIdx].fieldCount = 0U;
    st.hkSlotCount++;
    parseCurrentHkSlot_ = slotIdx;

    return true;
}

// ── parseLogEveryLineLocked ──────────────────────────────────────────────────

/**
 * @brief Parse a @c log_every directive that sets the CSV log interval.
 *
 * Syntax: @c "log_every \<N\>ms:"
 *
 * @param[in]  line  Script line starting with @c "log_every ".
 * @param[out] st    State to update with the log interval.
 * @return @c true if a valid non-zero interval was parsed.
 * @pre  Caller holds the engine mutex.
 */
bool MissionScriptEngine::parseLogEveryLineLocked(const char* line, StateDef& st)
{
    const char* const prefix    = "log_every ";
    const size_t      prefixLen = strlen(prefix);
    if (strncmp(line, prefix, prefixLen) != 0)
    {
        setErrorLocked("invalid log_every syntax");
        return false;
    }

    const char* valueStart = line + prefixLen;
    while (*valueStart == ' ' || *valueStart == '\t') { valueStart++; }

    const char* marker = strstr(valueStart, "ms:");
    if (marker == nullptr)
    {
        setErrorLocked("invalid log_every period");
        return false;
    }
    if (!isOnlyTrailingWhitespace(marker + 3))
    {
        setErrorLocked("invalid log_every period suffix");
        return false;
    }

    const ptrdiff_t rawLen = marker - valueStart;
    if (rawLen <= 0 || rawLen >= 16)
    {
        setErrorLocked("invalid log_every period length");
        return false;
    }

    char valueBuf[16] = {};
    memcpy(valueBuf, valueStart, static_cast<size_t>(rawLen));
    valueBuf[static_cast<size_t>(rawLen)] = '\0';
    trimInPlace(valueBuf);

    uint32_t everyMs = 0;
    if (!parseUint(valueBuf, everyMs) || everyMs == 0U)
    {
        setErrorLocked("invalid log_every period");
        return false;
    }

    st.hasLogEvery   = true;
    st.logEveryMs    = everyMs;
    st.logFieldCount = 0;

    // AMS-4.3.1: allocate a new LOG slot for this log_every directive.
    if (st.logSlotCount >= ares::AMS_MAX_HK_SLOTS)
    {
        setErrorLocked("too many log_every blocks in state (max AMS_MAX_HK_SLOTS)");
        return false;
    }
    const uint8_t slotIdx = st.logSlotCount;
    st.logSlots[slotIdx].everyMs    = everyMs;
    st.logSlots[slotIdx].fieldCount = 0U;
    st.logSlotCount++;
    parseCurrentLogSlot_ = slotIdx;

    return true;
}

// ── parsePrioritiesLineLocked ────────────────────────────────────────────────

/**
 * @brief Parse a @c priorities directive and update per-action scheduling.
 *
 * Syntax: @c "priorities [event=N] [hk=N] [log=N] [budget=N]"
 *
 * Valid priority values: 0–9.  Valid budget values: 1–3.
 * Delegates parsing variants to parsePrioritiesValuesLocked().
 *
 * @param[in]  line  Script line starting with @c "priorities ".
 * @param[out] st    State whose scheduling parameters are updated.
 * @return @c true if valid priority values were parsed.
 * @pre  Caller holds the engine mutex.
 */
bool MissionScriptEngine::parsePrioritiesLineLocked(const char* line,
                                                    StateDef&   st)
{
    ARES_ASSERT(line != nullptr);

    uint32_t event  = st.eventPriority;
    uint32_t hk     = 0U;
    uint32_t log    = 0U;
    uint32_t budget = st.actionBudget;

    if (!parsePrioritiesValuesLocked(line, event, hk, log, budget, st)
        || event > 9U || hk > 9U || log > 9U
        || budget < 1U || budget > 3U)
    {
        setErrorLocked("invalid priorities syntax");
        return false;
    }

    st.eventPriority = static_cast<uint8_t>(event);
    st.hkPriority    = static_cast<uint8_t>(hk);
    st.logPriority   = static_cast<uint8_t>(log);
    st.actionBudget  = static_cast<uint8_t>(budget);
    return true;
}

// ── parsePrioritiesValuesLocked ──────────────────────────────────────────────

/**
 * @brief Extract numeric values from a @c priorities directive using sscanf.
 *
 * Tries four variant formats in descending specificity (4-field, 3-field
 * with budget, 3-field without budget, 2-field).  Missing keys retain the
 * current state default.
 *
 * @param[in]     line    Script line starting with @c "priorities ".
 * @param[in,out] event   event= priority value (in: default, out: parsed).
 * @param[in,out] hk      hk= priority value.
 * @param[in,out] log     log= priority value.
 * @param[in,out] budget  budget= action budget.
 * @param[in]     st      Current state (used for default fallback values).
 * @return @c true if at least two values were parsed.
 * @pre  Caller holds the engine mutex.
 */
bool MissionScriptEngine::parsePrioritiesValuesLocked(const char* line,
                                                      uint32_t&   event,
                                                      uint32_t&   hk,
                                                      uint32_t&   log,
                                                      uint32_t&   budget,
                                                      const StateDef& st)
{
    int32_t n = 0;
    n = static_cast<int32_t>(sscanf(line,  // NOLINT(bugprone-unchecked-string-to-number-conversion)
                   "priorities event=%" SCNu32 " hk=%" SCNu32
                   " log=%" SCNu32 " budget=%" SCNu32,
                   &event, &hk, &log, &budget));
    if (n == 4) { return true; }

    n = static_cast<int32_t>(sscanf(line,  // NOLINT(bugprone-unchecked-string-to-number-conversion)
               "priorities event=%" SCNu32 " hk=%" SCNu32
               " log=%" SCNu32,
               &event, &hk, &log));
    if (n == 3) { budget = st.actionBudget; return true; }

    n = static_cast<int32_t>(sscanf(line,  // NOLINT(bugprone-unchecked-string-to-number-conversion)
               "priorities hk=%" SCNu32 " log=%" SCNu32
               " budget=%" SCNu32,
               &hk, &log, &budget));
    if (n == 3) { event = st.eventPriority; return true; }

    n = sscanf(line,  // NOLINT(bugprone-unchecked-string-to-number-conversion)
               "priorities hk=%" SCNu32 " log=%" SCNu32,
               &hk, &log);
    if (n == 2) { event = st.eventPriority; budget = st.actionBudget; return true; }

    return false;
}

// ── parseFieldLineLocked ─────────────────────────────────────────────────────

/**
 * @brief Parse one @c KEY: ALIAS.field entry into a @c HkField slot.
 *
 * Used for both @c HK.report and @c LOG.report field lists.
 *
 * @param[in]     line      Script line (e.g. @c "altitude: BARO.alt").
 * @param[out]    fields    Field array to append to.
 * @param[in,out] count     Current field count; incremented on success.
 * @param[in]     ctxName   Context name for error messages (@c "HK" or @c "LOG").
 * @return @c true if the field was valid and added.
 * @pre  Caller holds the engine mutex.
 */
bool MissionScriptEngine::parseFieldLineLocked(const char* line,
                                               HkField*    fields,
                                               uint8_t&    count,
                                               const char* ctxName)
{
    if (strcmp(line, "{") == 0) { return true; }

    char key[20]  = {};
    char expr[32] = {};
    const int32_t n = static_cast<int32_t>(sscanf(line, "%19[^:]: %31s", key, expr));
    if (n != 2)
    {
        char msg[48] = {};
        snprintf(msg, sizeof(msg), "invalid %s field syntax", ctxName);
        setErrorLocked(msg);
        return false;
    }

    char aliasStr[16] = {};
    char fieldStr[20] = {};
    if (!splitAliasDotField(expr,
                            aliasStr, sizeof(aliasStr),
                            fieldStr, sizeof(fieldStr)))
    {
        static char msg[96] = {};
        snprintf(msg, sizeof(msg),
                 "invalid %s expression '%s' (expected ALIAS.field)",
                 ctxName, expr);
        setErrorLocked(msg);
        return false;
    }

    const AliasEntry* ae = findAliasLocked(aliasStr);
    if (ae == nullptr)
    {
        char msg[56] = {};
        snprintf(msg, sizeof(msg), "unknown alias '%s'", aliasStr);
        setErrorLocked(msg);
        return false;
    }

    SensorField sf = SensorField::ALT;
    if (!parseSensorField(ae->kind, fieldStr, sf))
    {
        static char msg[80] = {};
        snprintf(msg, sizeof(msg),
                 "field '%s' not valid for alias '%s'", fieldStr, aliasStr);
        setErrorLocked(msg);
        return false;
    }

    if (count >= ares::AMS_MAX_HK_FIELDS)
    {
        char msg[48] = {};
        snprintf(msg, sizeof(msg), "too many %s fields", ctxName);
        setErrorLocked(msg);
        return false;
    }

    HkField& hf = fields[count++];
    strncpy(hf.label, key,      sizeof(hf.label) - 1U);
    hf.label[sizeof(hf.label) - 1U] = '\0';
    strncpy(hf.alias, aliasStr, sizeof(hf.alias) - 1U);
    hf.alias[sizeof(hf.alias) - 1U] = '\0';
    hf.field = sf;
    return true;
}

// ── parseOneConditionLocked ──────────────────────────────────────────────────

/**
 * @brief Parse a single condition sub-expression string into a @c CondExpr.
 *
 * Supported forms:
 *   - @c "ALIAS.field OP VALUE"   — standard sensor/time comparison
 *   - @c "TC.command == VALUE"    — TC one-shot token gate
 *   - @c "ALIAS.field delta OP VALUE" — inter-sample delta (AMS-4.6.2)
 *   - @c "ALIAS.field falling"   — syntactic sugar: delta < 0
 *   - @c "ALIAS.field rising"    — syntactic sugar: delta > 0
 *
 * @param[in]  condStr  Trimmed condition sub-expression (NUL-terminated).
 * @param[in]  allowTc  Whether TC.command is accepted.
 * @param[out] out      Populated @c CondExpr on success.
 * @return @c true if the expression was parsed without error.
 * @pre  Caller holds the engine mutex.
 */
bool MissionScriptEngine::parseDeltaCondLocked(
    const char* tok1, const char* tok3, const char* tok4, CondExpr& out)
{
    char aliasStr[16] = {};
    char fieldStr[20] = {};
    if (!splitAliasDotField(tok1, aliasStr, sizeof(aliasStr), fieldStr, sizeof(fieldStr)))
    {
        setErrorLocked("invalid delta LHS (expected ALIAS.field)");
        return false;
    }
    const AliasEntry* ae = findAliasLocked(aliasStr);
    if (ae == nullptr)
    {
        char msg[56] = {};
        snprintf(msg, sizeof(msg), "unknown alias '%s' in delta condition", aliasStr);
        setErrorLocked(msg);
        return false;
    }
    SensorField sf = SensorField::ALT;
    if (!parseSensorField(ae->kind, fieldStr, sf))
    {
        static char msg[80] = {};
        snprintf(msg, sizeof(msg), "field '%s' not valid for alias '%s'", fieldStr, aliasStr);
        setErrorLocked(msg);
        return false;
    }
    float thr = 0.0f;
    if (!parseFloatValue(tok4, thr))
    {
        setErrorLocked("invalid delta threshold value");
        return false;
    }
    if (strcmp(tok3, "<") == 0)      { out.kind = CondKind::SENSOR_DELTA_LT; }
    else if (strcmp(tok3, ">") == 0) { out.kind = CondKind::SENSOR_DELTA_GT; }
    else
    {
        setErrorLocked("delta condition only supports '<' or '>' operators");
        return false;
    }
    strncpy(out.alias, aliasStr, sizeof(out.alias) - 1U);
    out.alias[sizeof(out.alias) - 1U] = '\0';
    out.field     = sf;
    out.threshold = thr;
    return true;
}

bool MissionScriptEngine::parseFallingRisingCondLocked(
    const char* tok1, const char* tok2, CondExpr& out)
{
    char aliasStr[16] = {};
    char fieldStr[20] = {};
    if (!splitAliasDotField(tok1, aliasStr, sizeof(aliasStr), fieldStr, sizeof(fieldStr)))
    {
        setErrorLocked("invalid falling/rising LHS (expected ALIAS.field)");
        return false;
    }
    const AliasEntry* ae = findAliasLocked(aliasStr);
    if (ae == nullptr)
    {
        static char msg[72] = {};
        snprintf(msg, sizeof(msg), "unknown alias '%s' in falling/rising condition", aliasStr);
        setErrorLocked(msg);
        return false;
    }
    SensorField sf = SensorField::ALT;
    if (!parseSensorField(ae->kind, fieldStr, sf))
    {
        static char msg[80] = {};
        snprintf(msg, sizeof(msg), "field '%s' not valid for alias '%s'", fieldStr, aliasStr);
        setErrorLocked(msg);
        return false;
    }
    out.kind = (strcmp(tok2, "falling") == 0) ? CondKind::SENSOR_DELTA_LT
                                               : CondKind::SENSOR_DELTA_GT;
    strncpy(out.alias, aliasStr, sizeof(out.alias) - 1U);
    out.alias[sizeof(out.alias) - 1U] = '\0';
    out.field     = sf;
    out.threshold = 0.0f;
    return true;
}

bool MissionScriptEngine::parseTcDebounceCondLocked(
    bool allowTc, const char* d3, const char* d4, const char* d5, int32_t nd, CondExpr& out)
{
    if (!allowTc)
    {
        setErrorLocked("TC.command not valid in conditions block");
        return false;
    }
    TcCommand cmd = TcCommand::NONE;
    if (!parseTcCommand(d3, cmd) || cmd == TcCommand::NONE)
    {
        setErrorLocked("invalid TC.command value");
        return false;
    }
    out.kind    = CondKind::TC_EQ;
    out.tcValue = cmd;

    if (nd <= 3 || d4[0] == '\0')
    {
        out.tcDebounce = TcDebounceMode::ONESHOT;
        out.tcConfirmN = 1U;
    }
    else if (strcmp(d4, "once") == 0)
    {
        out.tcDebounce = TcDebounceMode::ONCE;
        out.tcConfirmN = 1U;
    }
    else if (strcmp(d4, "confirm") == 0)
    {
        if (nd < 5 || d5[0] == '\0')
        {
            setErrorLocked("TC confirm: missing count (expected: confirm N)");
            return false;
        }
        uint32_t n = 0U;
        if (!parseUint(d5, n) || n < 2U || n > 10U)
        {
            setErrorLocked("TC confirm N must be 2-10");
            return false;
        }
        out.tcDebounce = TcDebounceMode::CONFIRM;
        out.tcConfirmN = static_cast<uint8_t>(n);
    }
    else
    {
        char msg[64] = {};
        snprintf(msg, sizeof(msg), "unknown TC modifier '%s'", d4);
        setErrorLocked(msg);
        return false;
    }
    return true;
}

bool MissionScriptEngine::parseOneConditionLocked(const char* condStr,
                                                  bool        allowTc,
                                                  CondExpr&   out)
{
    ARES_ASSERT(condStr != nullptr);

    char tok1[20] = {};
    char tok2[20] = {};
    char tok3[4]  = {};
    char tok4[20] = {};

    // 4-token form: ALIAS.field delta OP VALUE
    const int32_t n4 = static_cast<int32_t>(sscanf(condStr, "%19s %19s %3s %19s", tok1, tok2, tok3, tok4));
    if (n4 == 4 && strcmp(tok2, "delta") == 0)
    {
        return parseDeltaCondLocked(tok1, tok3, tok4, out);
    }

    // 2-token form: ALIAS.field falling | rising
    const int32_t n2 = static_cast<int32_t>(sscanf(condStr, "%19s %19s", tok1, tok2));
    if (n2 == 2 && (strcmp(tok2, "falling") == 0 || strcmp(tok2, "rising") == 0))
    {
        return parseFallingRisingCondLocked(tok1, tok2, out);
    }

    // AMS-4.11: TC debounce forms (3-5 tokens)
    {
        char d1[20] = {};
        char d2[3]  = {};
        char d3[20] = {};
        char d4[8]  = {};
        char d5[8]  = {};
        // cppcheck-suppress [cert-err34-c]
        const int32_t nd = static_cast<int32_t>(sscanf(condStr, "%19s %2s %19s %7s %7s", d1, d2, d3, d4, d5));
        char tcCmdToken[24] = {};
        snprintf(tcCmdToken, sizeof(tcCmdToken), "%s.command", program_.tcAlias);
        if (nd >= 3 && strcmp(d1, tcCmdToken) == 0 && strcmp(d2, "==") == 0)
        {
            return parseTcDebounceCondLocked(allowTc, d3, d4, d5, nd, out);
        }
    }

    // 3-token form: standard LHS OP RHS (delegates to parseCondExprLocked)
    char lhs[20] = {};
    char op[3]   = {};
    char rhs[20] = {};
    const int32_t n3 = static_cast<int32_t>(sscanf(condStr, "%19s %2s %19s", lhs, op, rhs));
    if (n3 == 3)
    {
        return parseCondExprLocked(lhs, op, rhs, allowTc, out);
    }

    setErrorLocked("invalid condition sub-expression syntax");
    return false;
}

// ── parseTransitionLineLocked ────────────────────────────────────────────────

/**
 * @brief Parse a @c transition directive and store the resulting conditions.
 *
 * Extended syntax supports compound conditions connected by @c or / @c and
 * (AMS-4.6.2), delta operators, and the falling/rising shorthand:
 *
 * @code
 * transition to TARGET when COND [or|and COND ...] [for Nms]
 * @endcode
 *
 * Where each COND may be:
 *   - @c ALIAS.field OP VALUE
 *   - @c TC.command == VALUE
 *   - @c TIME.elapsed > VALUE
 *   - @c ALIAS.field delta OP VALUE
 *   - @c ALIAS.field falling  (delta < 0)
 *   - @c ALIAS.field rising   (delta > 0)
 *
 * Logic must be homogeneous: all @c or or all @c and within one transition.
 * Up to @c AMS_MAX_TRANSITION_CONDS sub-conditions are supported.
 *
 * @param[in]  line  Script line starting with @c "transition to ".
 * @param[out] st    State to store the transition on.
 * @return @c true if the transition was parsed and stored.
 * @pre  Caller holds the engine mutex.
 */
bool MissionScriptEngine::buildAndStoreTransitionLocked(
    const char* target, uint32_t holdMs, TransitionLogic logic,
    const char* const condParts[], uint8_t condCount, StateDef& st)
{
    if (condCount == 0U)
    {
        setErrorLocked("transition has no condition");
        return false;
    }

    // Guard against exceeding the per-state transition table (AMS-4.6).
    if (st.transitionCount >= ares::AMS_MAX_TRANSITIONS)
    {
        setErrorLocked("too many 'transition to' directives in state (AMS_MAX_TRANSITIONS exceeded)");
        return false;
    }

    Transition tr = {};
    tr.holdMs = holdMs;
    tr.logic  = logic;
    strncpy(tr.targetName, target, sizeof(tr.targetName) - 1U);
    tr.targetName[sizeof(tr.targetName) - 1U] = '\0';
    tr.targetResolved = false;

    for (uint8_t i = 0; i < condCount; i++)
    {
        if (!parseOneConditionLocked(condParts[i], /*allowTc=*/true, tr.conds[i]))
        {
            return false;
        }
    }
    tr.condCount = condCount;

    st.transitions[st.transitionCount] = tr;
    st.transitionCount++;
    return true;
}

bool MissionScriptEngine::parseTransitionLineLocked(const char* line,
                                                    StateDef&   st)
{
    ARES_ASSERT(line != nullptr);

    const char* const whenKw = strstr(line, " when ");
    if (whenKw == nullptr)
    {
        setErrorLocked("invalid transition syntax: missing 'when'");
        return false;
    }

    char target[ares::AMS_MAX_STATE_NAME] = {};
    const int32_t nTarget = static_cast<int32_t>(sscanf(line, "transition to %15s", target));
    if (nTarget != 1 || target[0] == '\0')
    {
        setErrorLocked("invalid transition syntax: missing target state");
        return false;
    }

    const char* body = whenKw + 6;
    while (*body == ' ') { body++; }

    char bodyBuf[ares::AMS_MAX_LINE_LEN] = {};
    strncpy(bodyBuf, body, sizeof(bodyBuf) - 1U);
    bodyBuf[sizeof(bodyBuf) - 1U] = '\0';

    uint32_t holdMs = 0U;
    if (!parseTransitionHoldClauseLocked(bodyBuf, holdMs)) { return false; }

    TransitionLogic logic = TransitionLogic::AND;
    const char* condParts[ares::AMS_MAX_TRANSITION_CONDS] = {};
    char condBufs[ares::AMS_MAX_TRANSITION_CONDS][ares::AMS_MAX_LINE_LEN] = {};
    uint8_t condCount = 0;
    if (!splitTransitionConditionsLocked(bodyBuf, condParts, condBufs, condCount, logic))
    {
        return false;
    }

    return buildAndStoreTransitionLocked(target, holdMs, logic, condParts, condCount, st);
}

bool MissionScriptEngine::parseTransitionHoldClauseLocked(char* bodyBuf,
                                                          uint32_t& holdMs)
{
    holdMs = 0U;
    char* forPtr = nullptr;
    char* search = bodyBuf;
    for (char* found = strstr(search, " for ");
         found != nullptr;
         found = strstr(search, " for "))
    {
        forPtr = found;
        search = found + 5;
    }
    if (forPtr == nullptr)
    {
        return true;
    }

    const char* forVal = forPtr + 5;
    const char* msSuffix = strstr(forVal, "ms");
    if (msSuffix == nullptr || msSuffix == forVal)
    {
        setErrorLocked("invalid 'for' clause: expected <N>ms");
        return false;
    }

    const ptrdiff_t numLen = msSuffix - forVal;
    if (numLen <= 0 || numLen >= 16)
    {
        setErrorLocked("invalid 'for' clause: numeric value too long");
        return false;
    }

    char numBuf[16] = {};
    memcpy(numBuf, forVal, static_cast<size_t>(numLen));
    numBuf[static_cast<size_t>(numLen)] = '\0';
    if (!parseUint(numBuf, holdMs) || holdMs == 0U)
    {
        setErrorLocked("invalid 'for' clause: holdMs must be > 0");
        return false;
    }

    *forPtr = '\0';
    uint32_t bLen = static_cast<uint32_t>(strnlen(bodyBuf, ares::AMS_MAX_LINE_LEN));
    while (bLen > 0U && (bodyBuf[bLen - 1U] == ' ' || bodyBuf[bLen - 1U] == '\t'))
    {
        bodyBuf[--bLen] = '\0';
    }
    return true;
}

bool MissionScriptEngine::splitTransitionConditionsLocked(
    char*            bodyBuf,
    const char*      condParts[],
    char             condBufs[][ares::AMS_MAX_LINE_LEN],
    uint8_t&         condCount,
    TransitionLogic& logic)
{
    condCount = 0U;
    bool logicDetected = false;
    logic = TransitionLogic::AND;

    char* remaining = bodyBuf;
    while (remaining != nullptr && remaining[0] != '\0')
    {
        char* orPtr  = strstr(remaining, " or ");
        char* andPtr = strstr(remaining, " and ");

        char* splitAt = nullptr;
        TransitionLogic splitLogic = TransitionLogic::AND;
        uint8_t skipLen = 0U;

        if (orPtr != nullptr && (andPtr == nullptr || orPtr < andPtr))
        {
            splitAt = orPtr;
            splitLogic = TransitionLogic::OR;
            skipLen = 4U;
        }
        else if (andPtr != nullptr)
        {
            splitAt = andPtr;
            splitLogic = TransitionLogic::AND;
            skipLen = 5U;
        }

        if (splitAt != nullptr)
        {
            if (!logicDetected)
            {
                logic = splitLogic;
                logicDetected = true;
            }
            else if (splitLogic != logic)
            {
                setErrorLocked("mixed 'or'/'and' in one transition is not supported");
                return false;
            }
        }

        const size_t partLen = (splitAt != nullptr)
                             ? static_cast<size_t>(splitAt - remaining)
                             : strnlen(remaining, ares::AMS_MAX_LINE_LEN);
        if (condCount >= ares::AMS_MAX_TRANSITION_CONDS)
        {
            setErrorLocked("too many sub-conditions in transition");
            return false;
        }

        strncpy(condBufs[condCount], remaining, partLen);
        condBufs[condCount][partLen] = '\0';
        trimInPlace(condBufs[condCount]);
        condParts[condCount] = condBufs[condCount];
        condCount++;

        remaining = (splitAt != nullptr) ? (splitAt + skipLen) : nullptr;
    }

    if (condCount == 0U)
    {
        setErrorLocked("transition has no condition");
        return false;
    }

    return true;
}

// ── parseConditionScopedLineLocked ────────────────────────────────────────────

/**
 * @brief Parse one line inside a @c conditions: block (AMS-4.7 guard).
 *
 * Syntax: @c "\<LHS\> \<OP\> \<RHS\>"
 *
 * TC.command is not allowed in guard conditions; use transitions instead.
 *
 * @param[in]  line  Trimmed script line (NUL-terminated).
 * @param[out] st    State to append the condition to.
 * @return @c true if the condition expression was parsed and stored.
 * @pre  Caller holds the engine mutex.
 */
bool MissionScriptEngine::parseConditionScopedLineLocked(const char* line,
                                                         StateDef&   st)
{
    ARES_ASSERT(line != nullptr);

    if (strcmp(line, "{") == 0) { return true; }  // optional block delimiter

    if (st.conditionCount >= ares::AMS_MAX_CONDITIONS)
    {
        setErrorLocked("too many conditions in state");
        return false;
    }

    char lhs[20] = {};
    char op[3]   = {};
    char rhs[20] = {};

    const int32_t n = static_cast<int32_t>(sscanf(line, "%19s %2s %19s", lhs, op, rhs));
    if (n != 3)
    {
        setErrorLocked("invalid condition syntax");
        return false;
    }

    CondExpr expr = {};
    if (!parseCondExprLocked(lhs, op, rhs, /*allowTc=*/false, expr))
    {
        return false;  // error already set
    }

    st.conditions[st.conditionCount++].expr = expr;
    return true;
}

// ── parseOnErrorEventLineLocked ──────────────────────────────────────────────

/**
 * @brief Parse an @c EVENT.* directive inside an @c on_error: block.
 *
 * Accepted verbs: @c info, @c warning, @c error.
 *
 * @param[in]  line  Script line starting with @c "EVENT.".
 * @param[out] st    State to store the on-error event configuration.
 * @return @c true if the event directive was parsed and stored.
 * @pre  Caller holds the engine mutex.
 */
bool MissionScriptEngine::parseOnErrorEventLineLocked(const char* line,
                                                      StateDef&   st)
{
    ARES_ASSERT(line != nullptr);

    char verb[8] = {};
    char text[ares::AMS_MAX_EVENT_TEXT] = {};
    const char* dotPos = strchr(line, '.');
    if (dotPos == nullptr)
    {
        setErrorLocked("invalid EVENT syntax in on_error");
        return false;
    }
    const int32_t n = static_cast<int32_t>(sscanf(dotPos + 1, "%7[^ ] \"%63[^\"]\"", verb, text));
    if (n != 2)
    {
        setErrorLocked("invalid EVENT syntax in on_error");
        return false;
    }

    st.hasOnErrorEvent = true;
    strncpy(st.onErrorText, text, sizeof(st.onErrorText) - 1U);
    st.onErrorText[sizeof(st.onErrorText) - 1U] = '\0';

    if (strcmp(verb, "info")    == 0) { st.onErrorVerb = EventVerb::INFO;  return true; }
    if (strcmp(verb, "warning") == 0) { st.onErrorVerb = EventVerb::WARN;  return true; }
    if (strcmp(verb, "error")   == 0) { st.onErrorVerb = EventVerb::ERROR; return true; }

    setErrorLocked("unknown EVENT verb in on_error");
    return false;
}

// ── parseOnExitEventLineLocked ───────────────────────────────────────────────

/**
 * @brief Parse an @c EVENT.* directive inside an @c on_exit: block (AMS-4.9).
 *
 * At most one EVENT.* is permitted per on_exit block.
 * Accepted verbs: @c info, @c warning, @c error.
 *
 * @param[in]  line  Script line starting with @c "EVENT.".
 * @param[out] st    State to store the on-exit event configuration.
 * @return @c true if the event directive was parsed and stored.
 * @pre  Caller holds the engine mutex.
 */
bool MissionScriptEngine::parseOnExitEventLineLocked(const char* line,
                                                     StateDef&   st)
{
    ARES_ASSERT(line != nullptr);

    if (st.hasOnExitEvent)
    {
        setErrorLocked("only one EVENT.* allowed inside on_exit");
        return false;
    }

    char verb[8] = {};
    char text[ares::AMS_MAX_EVENT_TEXT] = {};
    const char* dotPos = strchr(line, '.');
    if (dotPos == nullptr)
    {
        setErrorLocked("invalid EVENT syntax in on_exit");
        return false;
    }
    const int32_t n = static_cast<int32_t>(sscanf(dotPos + 1, "%7[^ ] \"%63[^\"]\"", verb, text));
    if (n != 2)
    {
        setErrorLocked("invalid EVENT syntax in on_exit");
        return false;
    }

    st.hasOnExitEvent = true;
    strncpy(st.onExitText, text, sizeof(st.onExitText) - 1U);
    st.onExitText[sizeof(st.onExitText) - 1U] = '\0';

    if (strcmp(verb, "info")    == 0) { st.onExitVerb = EventVerb::INFO;  return true; }
    if (strcmp(verb, "warning") == 0) { st.onExitVerb = EventVerb::WARN;  return true; }
    if (strcmp(verb, "error")   == 0) { st.onExitVerb = EventVerb::ERROR; return true; }

    setErrorLocked("unknown EVENT verb in on_exit");
    return false;
}

// ── parseOnTimeoutHeaderLocked ───────────────────────────────────────────────

/**
 * @brief Parse an @c on_timeout @c Nms: header and open the block (AMS-4.10.3).
 *
 * Syntax: @c "on_timeout Nms:"
 *
 * N must be a positive integer.  Only one @c on_timeout block per state is
 * allowed; a second declaration is a parse error.
 *
 * @param[in]     line       Script line starting with @c "on_timeout ".
 * @param[out]    st         State to attach the timeout to.
 * @param[in,out] blockType  Set to @c ON_TIMEOUT on success.
 * @return @c true if the header was parsed and stored.
 * @pre  Caller holds the engine mutex.
 */
bool MissionScriptEngine::parseOnTimeoutHeaderLocked(const char* line,
                                                     StateDef&   st,
                                                     BlockType&  blockType)
{
    ARES_ASSERT(line != nullptr);

    if (st.hasOnTimeout)
    {
        setErrorLocked("duplicate on_timeout block in state");
        return false;
    }

    // Parse "on_timeout Nms:"
    const char* numStart = line + sizeof("on_timeout ") - 1U;
    const char* msPtr    = strstr(numStart, "ms:");
    if (msPtr == nullptr)
    {
        setErrorLocked("on_timeout: expected 'on_timeout Nms:'");
        return false;
    }

    const ptrdiff_t numLen = msPtr - numStart;
    if (numLen <= 0 || numLen >= 16)
    {
        setErrorLocked("on_timeout: timeout value out of range");
        return false;
    }

    char numBuf[16] = {};
    memcpy(numBuf, numStart, static_cast<size_t>(numLen));
    numBuf[static_cast<size_t>(numLen)] = '\0';

    uint32_t ms = 0U;
    if (!parseUint(numBuf, ms) || ms == 0U)
    {
        setErrorLocked("on_timeout: timeout must be > 0 ms");
        return false;
    }

    if (!isOnlyTrailingWhitespace(msPtr + 3))
    {
        setErrorLocked("on_timeout: unexpected suffix after 'ms:'");
        return false;
    }

    st.hasOnTimeout = true;
    st.onTimeoutMs  = ms;
    blockType       = BlockType::ON_TIMEOUT;
    return true;
}

// ── parseOnTimeoutEventLineLocked ────────────────────────────────────────────

/**
 * @brief Parse an @c EVENT.* directive inside an @c on_timeout: block (AMS-4.10.3).
 *
 * At most one EVENT.* is permitted per on_timeout block.
 * Accepted verbs: @c info, @c warning, @c error.
 *
 * @param[in]  line  Script line starting with @c "EVENT.".
 * @param[out] st    State to store the on-timeout event configuration.
 * @return @c true if the event directive was parsed and stored.
 * @pre  Caller holds the engine mutex.
 */
bool MissionScriptEngine::parseOnTimeoutEventLineLocked(const char* line,
                                                        StateDef&   st)
{
    ARES_ASSERT(line != nullptr);

    if (st.hasOnTimeoutEvent)
    {
        setErrorLocked("only one EVENT.* allowed inside on_timeout");
        return false;
    }

    char verb[8] = {};
    char text[ares::AMS_MAX_EVENT_TEXT] = {};
    const char* dotPos = strchr(line, '.');
    if (dotPos == nullptr)
    {
        setErrorLocked("invalid EVENT syntax in on_timeout");
        return false;
    }
    const int32_t n = static_cast<int32_t>(sscanf(dotPos + 1, "%7[^ ] \"%63[^\"]\"", verb, text));
    if (n != 2)
    {
        setErrorLocked("invalid EVENT syntax in on_timeout");
        return false;
    }

    st.hasOnTimeoutEvent = true;
    strncpy(st.onTimeoutText, text, sizeof(st.onTimeoutText) - 1U);
    st.onTimeoutText[sizeof(st.onTimeoutText) - 1U] = '\0';

    if (strcmp(verb, "info")    == 0) { st.onTimeoutVerb = EventVerb::INFO;  return true; }
    if (strcmp(verb, "warning") == 0) { st.onTimeoutVerb = EventVerb::WARN;  return true; }
    if (strcmp(verb, "error")   == 0) { st.onTimeoutVerb = EventVerb::ERROR; return true; }

    setErrorLocked("unknown EVENT verb in on_timeout");
    return false;
}

// ── parseOnTimeoutTransitionLineLocked ───────────────────────────────────────

/**
 * @brief Parse a @c transition @c to directive inside an @c on_timeout: block (AMS-4.10.3).
 *
 * Syntax: @c "transition to \<STATE\>"
 *
 * Exactly one transition is required per on_timeout block; duplicates are rejected.
 *
 * @param[in]  line  Script line starting with @c "transition to ".
 * @param[out] st    State to attach the forced timeout transition to.
 * @return @c true if the transition target was parsed and stored.
 * @pre  Caller holds the engine mutex.
 */
bool MissionScriptEngine::parseOnTimeoutTransitionLineLocked(const char* line,
                                                             StateDef&   st)
{
    ARES_ASSERT(line != nullptr);

    if (st.onTimeoutTransitionTarget[0] != '\0')
    {
        setErrorLocked("duplicate 'transition to' in on_timeout block");
        return false;
    }

    char target[ares::AMS_MAX_STATE_NAME] = {};
    // cppcheck-suppress [cert-err34-c]
    const int32_t n = static_cast<int32_t>(sscanf(line, "transition to %15s", target));
    if (n != 1 || target[0] == '\0')
    {
        setErrorLocked("invalid on_timeout transition syntax: missing target state");
        return false;
    }

    strncpy(st.onTimeoutTransitionTarget, target,
            sizeof(st.onTimeoutTransitionTarget) - 1U);
    st.onTimeoutTransitionTarget[sizeof(st.onTimeoutTransitionTarget) - 1U] = '\0';
    st.onTimeoutTransitionResolved = false;

    LOG_I(TAG, "on_timeout transition: -> '%s'", target);
    return true;
}

// ── mapApidToNode ─────────────────────────────────────────────────────────────

/**
 * @brief Map a PUS APID value to the corresponding ARES node identifier.
 *
 * Supported mappings: 0 → BROADCAST, 1 → ROCKET, 2 → GROUND, 3 → PAYLOAD.
 *
 * @param[in]  apid    PUS APID value (0–3).
 * @param[out] nodeId  ARES protocol node identifier.
 * @return @c true if @p apid has a defined mapping.
 */
bool MissionScriptEngine::mapApidToNode(uint16_t apid, uint8_t& nodeId)
{
    switch (apid)
    {
    case 0U: nodeId = NODE_BROADCAST; return true;
    case 1U: nodeId = NODE_ROCKET;    return true;
    case 2U: nodeId = NODE_GROUND;    return true;
    case 3U: nodeId = NODE_PAYLOAD;   return true;
    default:                          return false;
    }
}

// ── resolveTransitionsLocked ─────────────────────────────────────────────────

/**
 * @brief Resolve transition target names to their numeric state indices.
 *
 * Called once after all states have been parsed.  For each state that has
 * a transition, looks up the target name in @c program_.states[].
 *
 * @return @c true if every transition target name resolved to a known state.
 * @pre  Caller holds the engine mutex.  All states have been parsed.
 * @post @c Transition::targetIndex and @c targetResolved are set on success.
 */
bool MissionScriptEngine::resolveTransitionsLocked()
{
    for (uint8_t i = 0; i < program_.stateCount; i++)
    {
        StateDef& st = program_.states[i];

        // Regular transition targets (AMS-4.6): one resolve pass per slot.
        for (uint8_t ti = 0U; ti < st.transitionCount; ti++)
        {
            const uint8_t idx = findStateByNameLocked(st.transitions[ti].targetName);
            if (idx >= program_.stateCount)
            {
                setErrorLocked("unknown transition target state");
                return false;
            }
            st.transitions[ti].targetIndex    = idx;
            st.transitions[ti].targetResolved = true;
        }

        // AMS-4.9.2: fallback transition target.
        if (st.hasFallback)
        {
            const uint8_t idx = findStateByNameLocked(st.fallbackTargetName);
            if (idx >= program_.stateCount)
            {
                setErrorLocked("unknown fallback transition target state");
                return false;
            }
            st.fallbackTargetIdx      = idx;
            st.fallbackTargetResolved = true;
        }

        // AMS-4.10.2: on_error recovery transition target.
        if (st.hasOnErrorTransition)
        {
            const uint8_t idx = findStateByNameLocked(st.onErrorTransitionTarget);
            if (idx >= program_.stateCount)
            {
                setErrorLocked("unknown on_error transition target state");
                return false;
            }
            st.onErrorTransitionIdx      = idx;
            st.onErrorTransitionResolved = true;
        }

        // AMS-4.10.3: on_timeout forced transition target.
        if (st.hasOnTimeout)
        {
            if (st.onTimeoutTransitionTarget[0] == '\0')
            {
                setErrorLocked("on_timeout block requires a 'transition to' directive");
                return false;
            }
            const uint8_t idx = findStateByNameLocked(st.onTimeoutTransitionTarget);
            if (idx >= program_.stateCount)
            {
                setErrorLocked("unknown on_timeout transition target state");
                return false;
            }
            st.onTimeoutTransitionIdx      = idx;
            st.onTimeoutTransitionResolved = true;
        }
    }

    return true;
}

// ── parseVarLineLocked ───────────────────────────────────────────────────────

/**
 * @brief Parse a global variable declaration from the script metadata section.
 *
 * Syntax: @c "var NAME = VALUE"
 *
 * Variables must be declared before any @c state block.  At most
 * @c AMS_MAX_VARS variables may be declared per script (AMS-4.8).
 * The initial @p VALUE is stored as a literal float and marks the
 * variable as @em invalid until a @c set action fires at runtime.
 *
 * @param[in] line  Script line starting with @c "var ".
 * @return @c true if the declaration was stored.
 * @pre  Caller holds the engine mutex.
 */
bool MissionScriptEngine::parseVarLineLocked(const char* line)
{
    ARES_ASSERT(line != nullptr);

    char name[ares::AMS_VAR_NAME_LEN] = {};
    char valBuf[24]                   = {};

    // cppcheck-suppress [cert-err34-c]
    const int32_t n = static_cast<int32_t>(sscanf(line, "var %15s = %23s", name, valBuf));
    if (n != 2 || name[0] == '\0')
    {
        setErrorLocked("invalid var syntax (expected: var NAME = VALUE)");
        return false;
    }

    // Validate name: must start with a letter/underscore; no dots.
    if (strchr(name, '.') != nullptr)
    {
        setErrorLocked("variable name must not contain '.'");
        return false;
    }

    // Reserved alias names must not shadow sensors.
    if (strcmp(name, "TIME") == 0 || strcmp(name, "TC") == 0)
    {
        setErrorLocked("variable name shadows reserved alias");
        return false;
    }

    // Duplicate check.
    for (uint8_t i = 0; i < program_.varCount; i++)
    {
        if (strcmp(program_.vars[i].name, name) == 0)
        {
            char msg[48] = {};
            snprintf(msg, sizeof(msg), "duplicate variable declaration: %s", name);
            setErrorLocked(msg);
            return false;
        }
    }

    if (program_.varCount >= ares::AMS_MAX_VARS)
    {
        setErrorLocked("too many variable declarations (AMS_MAX_VARS exceeded)");
        return false;
    }

    float initVal = 0.0f;
    if (!parseFloatValue(valBuf, initVal))
    {
        setErrorLocked("invalid var initial value");
        return false;
    }

    VarEntry& v = program_.vars[program_.varCount++];
    strncpy(v.name, name, sizeof(v.name) - 1U);
    v.name[sizeof(v.name) - 1U] = '\0';
    v.value = initVal;
    v.valid = false;  // marked invalid until a set action fires successfully
    return true;
}

// ── parseConstLineLocked ─────────────────────────────────────────────────────

/**
 * @brief Parse a named constant declaration from the script metadata section.
 *
 * Syntax: @c "const NAME = VALUE"
 *
 * Constants are immutable float literals resolved at parse time.  When a
 * constant name appears as a condition RHS, its value is inlined directly
 * into @c CondExpr::threshold (no runtime table lookup required).
 *
 * @param[in] line  Script line starting with @c "const ".
 * @return @c true if the constant was stored.
 * @pre  Caller holds the engine mutex.
 */
bool MissionScriptEngine::parseConstLineLocked(const char* line)
{
    ARES_ASSERT(line != nullptr);

    char name[ares::AMS_VAR_NAME_LEN] = {};
    char valBuf[24]                   = {};

    // cppcheck-suppress [cert-err34-c]
    const int32_t n = static_cast<int32_t>(sscanf(line, "const %15s = %23s", name, valBuf));
    if (n != 2 || name[0] == '\0')
    {
        setErrorLocked("invalid const syntax (expected: const NAME = VALUE)");
        return false;
    }

    if (!validateConstIdentifierLocked(name))
    {
        return false;
    }
    if (!ensureConstDoesNotConflictLocked(name))
    {
        return false;
    }

    if (program_.constCount >= ares::AMS_MAX_CONSTS)
    {
        setErrorLocked("too many const declarations (AMS_MAX_CONSTS exceeded)");
        return false;
    }

    float val = 0.0f;
    if (!parseFloatValue(valBuf, val))
    {
        setErrorLocked("const value must be a numeric literal");
        return false;
    }

    ConstEntry& ce = program_.consts[program_.constCount++];
    strncpy(ce.name, name, sizeof(ce.name) - 1U);
    ce.name[sizeof(ce.name) - 1U] = '\0';
    ce.value = val;

    LOG_I(TAG, "const '%s' = %.3f", name, static_cast<double>(val));
    return true;
}

bool MissionScriptEngine::validateConstIdentifierLocked(const char* name)
{
    ARES_ASSERT(name != nullptr);

    // Reject names with dots (reserved for ALIAS.field forms).
    if (strchr(name, '.') != nullptr)
    {
        setErrorLocked("const name must not contain '.'");
        return false;
    }

    // Reject reserved pseudo-aliases.
    if (strcmp(name, "TIME") == 0 || strcmp(name, "TC") == 0)
    {
        setErrorLocked("const name shadows reserved alias");
        return false;
    }

    return true;
}

bool MissionScriptEngine::ensureConstDoesNotConflictLocked(const char* name)
{
    ARES_ASSERT(name != nullptr);

    // Duplicate constant check.
    for (uint8_t i = 0; i < program_.constCount; i++)
    {
        if (strcmp(program_.consts[i].name, name) == 0)
        {
            char msg[64] = {};
            snprintf(msg, sizeof(msg), "duplicate const declaration: %s", name);
            setErrorLocked(msg);
            return false;
        }
    }

    // Name must not shadow an existing variable.
    for (uint8_t i = 0; i < program_.varCount; i++)
    {
        if (strcmp(program_.vars[i].name, name) == 0)
        {
            setErrorLocked("const name conflicts with an existing variable");
            return false;
        }
    }

    return true;
}

// ── findConstLocked ──────────────────────────────────────────────────────────

/**
 * @brief Look up a named constant by identifier.
 *
 * @param[in] name  Constant name to find.
 * @return Pointer to the @c ConstEntry, or @c nullptr if not found.
 * @pre  Caller holds the engine mutex.
 */
const MissionScriptEngine::ConstEntry*
MissionScriptEngine::findConstLocked(const char* name) const
{
    if (name == nullptr) { return nullptr; }
    for (uint8_t i = 0; i < program_.constCount; i++)
    {
        if (strcmp(program_.consts[i].name, name) == 0)
        {
            return &program_.consts[i];
        }
    }
    return nullptr;
}

// ── parseSetActionCoreLocked ─────────────────────────────────────────────────

/**
 * @brief Core parser for a @c set action expression (AMS-4.8).
 *
 * Shared by @c parseSetActionLineLocked (state on_enter blocks) and
 * @c parseTaskScopedLineLocked (task if-rules).
 *
 * Accepted forms — see parseSetActionLineLocked for details.
 *
 * @param[in]  line  Script line starting with @c "set ".
 * @param[out] out   SetAction struct to populate.
 * @return @c true if the action was parsed successfully.
 * @pre  Caller holds the engine mutex.
 */
bool MissionScriptEngine::parseSetActionCoreLocked(const char* line,
                                                   SetAction&  out)
{
    ARES_ASSERT(line != nullptr);

    char varName[ares::AMS_VAR_NAME_LEN] = {};
    char rhsBuf[64] = {};

    // cppcheck-suppress [cert-err34-c]
    const int32_t n = static_cast<int32_t>(sscanf(line, "set %15s = %63[^\n]", varName, rhsBuf));
    if (n != 2 || varName[0] == '\0' || rhsBuf[0] == '\0')
    {
        setErrorLocked("invalid set syntax (expected: set VARNAME = ALIAS.field)");
        return false;
    }

    if (!ensureSetVariableExistsLocked(varName))
    {
        return false;
    }

    strncpy(out.varName, varName, sizeof(out.varName) - 1U);
    out.varName[sizeof(out.varName) - 1U] = '\0';

    trimInPlace(rhsBuf);

    if (startsWith(rhsBuf, "CALIBRATE("))
    {
        return parseCalibrateSetActionLocked(rhsBuf, out);
    }
    if (startsWith(rhsBuf, "max(") || startsWith(rhsBuf, "min("))
    {
        return parseMinMaxSetActionLocked(rhsBuf, varName, out);
    }
    if (parseDeltaSetActionLocked(rhsBuf, out))
    {
        return true;
    }
    return parseSimpleSensorSetActionLocked(rhsBuf, out);
}

bool MissionScriptEngine::ensureSetVariableExistsLocked(const char* varName)
{
    for (uint8_t i = 0; i < program_.varCount; i++)
    {
        if (strcmp(program_.vars[i].name, varName) == 0)
        {
            return true;
        }
    }

    char msg[64] = {};
    snprintf(msg, sizeof(msg), "set action: undefined variable '%s'", varName);
    setErrorLocked(msg);
    return false;
}

bool MissionScriptEngine::parseCalibrateSetActionLocked(const char* rhsBuf,
                                                        SetAction&  out)
{
    char sensorExpr[32] = {};
    char nBuf[8] = {};
    const int32_t nc = static_cast<int32_t>(sscanf(rhsBuf, "CALIBRATE(%31[^,], %7[^)])", sensorExpr, nBuf));
    if (nc != 2)
    {
        setErrorLocked("invalid CALIBRATE syntax: expected CALIBRATE(ALIAS.field, N)");
        return false;
    }
    trimInPlace(sensorExpr);
    trimInPlace(nBuf);

    uint32_t samples = 0;
    if (!parseUint(nBuf, samples) || samples == 0U || samples > static_cast<uint32_t>(ares::AMS_CALIBRATE_MAX_SAMPLES))
    {
        static char msg[80] = {};
        snprintf(msg, sizeof(msg),
                 "CALIBRATE sample count must be 1-%u",
                 static_cast<unsigned>(ares::AMS_CALIBRATE_MAX_SAMPLES));
        setErrorLocked(msg);
        return false;
    }

    char aliasStr[16] = {};
    char fieldStr[20] = {};
    if (!splitAliasDotField(sensorExpr, aliasStr, sizeof(aliasStr), fieldStr, sizeof(fieldStr)))
    {
        setErrorLocked("CALIBRATE: invalid ALIAS.field expression");
        return false;
    }

    const AliasEntry* ae = findAliasLocked(aliasStr);
    if (ae == nullptr)
    {
        char msg[56] = {};
        snprintf(msg, sizeof(msg), "CALIBRATE: unknown alias '%s'", aliasStr);
        setErrorLocked(msg);
        return false;
    }

    SensorField sf = SensorField::ALT;
    if (!parseSensorField(ae->kind, fieldStr, sf))
    {
        static char msg[80] = {};
        snprintf(msg, sizeof(msg),
                 "CALIBRATE: field '%s' not valid for alias '%s'",
                 fieldStr, aliasStr);
        setErrorLocked(msg);
        return false;
    }

    strncpy(out.alias, aliasStr, sizeof(out.alias) - 1U);
    out.alias[sizeof(out.alias) - 1U] = '\0';
    out.field = sf;
    out.kind = SetActionKind::CALIBRATE;
    out.calibSamples = static_cast<uint8_t>(samples);
    return true;
}

bool MissionScriptEngine::parseMinMaxSetActionLocked(const char* rhsBuf,
                                                     const char* varName,
                                                     SetAction&  out)
{
    const bool isMax = (rhsBuf[0] == 'm' && rhsBuf[1] == 'a');
    char arg1[ares::AMS_VAR_NAME_LEN] = {};
    char arg2[32] = {};
    const int32_t nm = static_cast<int32_t>(sscanf(rhsBuf,
                          isMax ? "max(%15[^,], %31[^)])" : "min(%15[^,], %31[^)])",
                          arg1, arg2));
    if (nm != 2 || arg1[0] == '\0' || arg2[0] == '\0')
    {
        setErrorLocked("invalid max/min syntax: expected max(VARNAME, ALIAS.field)");
        return false;
    }
    trimInPlace(arg1);
    trimInPlace(arg2);
    if (strcmp(arg1, varName) != 0)
    {
        setErrorLocked("max/min first argument must match target variable name");
        return false;
    }

    char aliasStr[16] = {};
    char fieldStr[20] = {};
    if (!splitAliasDotField(arg2, aliasStr, sizeof(aliasStr), fieldStr, sizeof(fieldStr)))
    {
        setErrorLocked("max/min: invalid ALIAS.field expression");
        return false;
    }
    const AliasEntry* ae = findAliasLocked(aliasStr);
    if (ae == nullptr)
    {
        char msg[56] = {};
        snprintf(msg, sizeof(msg), "max/min: unknown alias '%s'", aliasStr);
        setErrorLocked(msg);
        return false;
    }
    SensorField sf = SensorField::ALT;
    if (!parseSensorField(ae->kind, fieldStr, sf))
    {
        static char msg[80] = {};
        snprintf(msg, sizeof(msg), "max/min: field '%s' not valid for alias '%s'",
                 fieldStr, aliasStr);
        setErrorLocked(msg);
        return false;
    }

    strncpy(out.alias, aliasStr, sizeof(out.alias) - 1U);
    out.alias[sizeof(out.alias) - 1U] = '\0';
    out.field = sf;
    out.kind = isMax ? SetActionKind::MAX_VAR : SetActionKind::MIN_VAR;
    return true;
}

bool MissionScriptEngine::parseDeltaSetActionLocked(const char* rhsBuf,
                                                    SetAction&  out)
{
    char exprBuf[32] = {};
    char kwBuf[8] = {};
    const int32_t nd = static_cast<int32_t>(sscanf(rhsBuf, "%31s %7s", exprBuf, kwBuf));
    if (nd != 2 || strcmp(kwBuf, "delta") != 0)
    {
        return false;
    }

    char aliasStr[16] = {};
    char fieldStr[20] = {};
    if (!splitAliasDotField(exprBuf, aliasStr, sizeof(aliasStr), fieldStr, sizeof(fieldStr)))
    {
        setErrorLocked("set delta: invalid ALIAS.field expression");
        return false;
    }

    const AliasEntry* ae = findAliasLocked(aliasStr);
    if (ae == nullptr)
    {
        char msg[56] = {};
        snprintf(msg, sizeof(msg), "set delta: unknown alias '%s'", aliasStr);
        setErrorLocked(msg);
        return false;
    }
    SensorField sf = SensorField::ALT;
    if (!parseSensorField(ae->kind, fieldStr, sf))
    {
        static char msg[80] = {};
        snprintf(msg, sizeof(msg), "set delta: field '%s' not valid for alias '%s'",
                 fieldStr, aliasStr);
        setErrorLocked(msg);
        return false;
    }

    strncpy(out.alias, aliasStr, sizeof(out.alias) - 1U);
    out.alias[sizeof(out.alias) - 1U] = '\0';
    out.field = sf;
    out.kind = SetActionKind::DELTA;
    out.deltaValid = false;
    return true;
}

bool MissionScriptEngine::parseSimpleSensorSetActionLocked(const char* rhsBuf,
                                                           SetAction&  out)
{
    char aliasStr[16] = {};
    char fieldStr[20] = {};
    if (!splitAliasDotField(rhsBuf, aliasStr, sizeof(aliasStr), fieldStr, sizeof(fieldStr)))
    {
        setErrorLocked("set action: invalid ALIAS.field expression");
        return false;
    }

    const AliasEntry* ae = findAliasLocked(aliasStr);
    if (ae == nullptr)
    {
        char msg[56] = {};
        snprintf(msg, sizeof(msg), "set action: unknown alias '%s'", aliasStr);
        setErrorLocked(msg);
        return false;
    }
    SensorField sf = SensorField::ALT;
    if (!parseSensorField(ae->kind, fieldStr, sf))
    {
        static char msg[80] = {};
        snprintf(msg, sizeof(msg), "set action: field '%s' not valid for alias '%s'",
                 fieldStr, aliasStr);
        setErrorLocked(msg);
        return false;
    }

    strncpy(out.alias, aliasStr, sizeof(out.alias) - 1U);
    out.alias[sizeof(out.alias) - 1U] = '\0';
    out.field = sf;
    out.kind = SetActionKind::SIMPLE;
    return true;
}

// ── parseSetActionLineLocked ─────────────────────────────────────────────────

/**
 * @brief Parse a @c set action inside an @c on_enter: block (AMS-4.8).
 *
 * Accepted forms:
 * @code
 *   set VARNAME = ALIAS.field                        -- snapshot (AMS-4.8.1)
 *   set VARNAME = CALIBRATE(ALIAS.field, N)          -- averaged snapshot (AMS-4.8.2)
 *   set VARNAME = ALIAS.field delta                  -- current minus previous (AMS-4.8.3)
 *   set VARNAME = max(VARNAME, ALIAS.field)          -- running maximum (AMS-4.8.4)
 *   set VARNAME = min(VARNAME, ALIAS.field)          -- running minimum (AMS-4.8.5)
 * @endcode
 *
 * @param[in]  line  Script line starting with @c "set ".
 * @param[out] st    State being populated.
 * @return @c true if the action was stored.
 * @pre  Caller holds the engine mutex.
 */
bool MissionScriptEngine::parseSetActionLineLocked(const char* line,
                                                   StateDef&   st)
{
    ARES_ASSERT(line != nullptr);

    if (st.setActionCount >= ares::AMS_MAX_SET_ACTIONS)
    {
        setErrorLocked("too many set actions in on_enter block");
        return false;
    }

    if (!parseSetActionCoreLocked(line, st.setActions[st.setActionCount]))
    {
        return false;
    }
    st.setActionCount++;
    return true;
}

// ── parsePulseFireLineLocked ──────────────────────────────────────────────────

/**
 * @brief Parse a @c PULSE.fire directive inside an @c on_enter: block (AMS-4.17).
 *
 * Syntax:
 * @code
 *   PULSE.fire A             -- fire channel A at default duration
 *   PULSE.fire B             -- fire channel B at default duration
 *   PULSE.fire A 500ms       -- fire channel A with 500 ms pulse override
 *   PULSE.fire B 1500ms      -- fire channel B with 1500 ms pulse override
 * @endcode
 *
 * The optional @c Nms duration overrides @c ares::FIRE_DURATION_MS at
 * execution time.  Storing 0 means "use the compile-time default".
 *
 * @param[in]  line  Script line starting with @c "PULSE.fire ".
 * @param[out] st    State definition to append the action to.
 * @return @c true if the directive was parsed and stored.
 * @pre  Caller holds the engine mutex.
 */
bool MissionScriptEngine::parsePulseFireLineLocked(const char* line,
                                                  StateDef&   st)
{
    ARES_ASSERT(line != nullptr);

    if (st.pulseActionCount >= ares::AMS_MAX_PULSE_ACTIONS)
    {
        setErrorLocked("too many PULSE.fire actions in on_enter block");
        return false;
    }

    // Skip "PULSE.fire " prefix (11 characters).
    static constexpr uint8_t kPrefixLen = 11U;
    const char* rest = line + kPrefixLen;

    // Determine channel.
    uint8_t channel = 0U;
    if (rest[0] == 'A')
    {
        channel = PulseChannel::CH_A;
    }
    else if (rest[0] == 'B')
    {
        channel = PulseChannel::CH_B;
    }
    else
    {
        setErrorLocked("PULSE.fire: channel must be A or B");
        return false;
    }

    uint32_t durationMs = 0U;
    if (!parsePulseDurationSuffixLocked(rest + 1U, durationMs))
    {
        return false;
    }

    st.pulseActions[st.pulseActionCount].channel    = channel;
    st.pulseActions[st.pulseActionCount].durationMs = durationMs;
    st.pulseActionCount++;
    return true;
}

/**
 * @brief Parse the optional " Nms" duration suffix of a PULSE.fire directive.
 *
 * @param[in]  afterCh  Pointer to the character immediately after the channel
 *                      letter ('A' or 'B') in the script line.
 * @param[out] out      Set to the parsed millisecond value, or 0 if absent
 *                      (meaning: use @c ares::FIRE_DURATION_MS at runtime).
 * @return @c true on success; @c false after calling @c setErrorLocked().
 * @pre  Caller holds the engine mutex.
 */
bool MissionScriptEngine::parsePulseDurationSuffixLocked(const char* afterCh,
                                                         uint32_t&   out)
{
    out = 0U;

    // No suffix at all — use compile-time default.
    if (*afterCh == '\0')
    {
        return true;
    }

    if (*afterCh != ' ')
    {
        setErrorLocked("PULSE.fire: unexpected characters after channel letter");
        return false;
    }

    const char* numStart = afterCh + 1U;

    if (*numStart == '\0')
    {
        setErrorLocked("PULSE.fire: expected Nms after channel letter");
        return false;
    }

    const char* msPtr = strstr(numStart, "ms");
    if (msPtr == nullptr || msPtr == numStart)
    {
        setErrorLocked("PULSE.fire: duration must be in the form Nms (e.g. 500ms)");
        return false;
    }

    const ptrdiff_t numLen = msPtr - numStart;
    if (numLen <= 0 || numLen >= 16)
    {
        setErrorLocked("PULSE.fire: duration value too long");
        return false;
    }

    char numBuf[16] = {};
    memcpy(numBuf, numStart, static_cast<size_t>(numLen));
    numBuf[static_cast<size_t>(numLen)] = '\0';

    if (!parseUint(numBuf, out) || out == 0U)
    {
        setErrorLocked("PULSE.fire: duration must be a positive integer");
        return false;
    }

    if (!isOnlyTrailingWhitespace(msPtr + 2U))
    {
        setErrorLocked("PULSE.fire: unexpected suffix after duration");
        return false;
    }

    return true;
}

/**
 * @brief Parse a @c fallback @c transition directive (AMS-4.9.2).
 *
 * Syntax: @c "fallback transition to \<STATE\> after \<N\>ms"
 *
 * If no regular transition fires within @p N milliseconds of entering the
 * state, the fallback transition fires unconditionally.  Intended to prevent
 * the mission from being permanently trapped in a state when sensors fail.
 *
 * @param[in]  line  Script line starting with @c "fallback transition to ".
 * @param[out] st    State to attach the fallback to.
 * @return @c true if the fallback was parsed and stored.
 * @pre  Caller holds the engine mutex.
 */
bool MissionScriptEngine::parseFallbackTransitionLineLocked(const char* line,
                                                            StateDef&   st)
{
    ARES_ASSERT(line != nullptr);

    if (st.hasFallback)
    {
        setErrorLocked("duplicate fallback transition in state");
        return false;
    }

    char target[ares::AMS_MAX_STATE_NAME] = {};
    uint32_t afterMs = 0U;

    // "fallback transition to TARGET after Nms"
    // Extract target state name.
    // cppcheck-suppress [cert-err34-c]
    const int32_t nt = static_cast<int32_t>(sscanf(line, "fallback transition to %15s", target));
    if (nt != 1 || target[0] == '\0')
    {
        setErrorLocked("invalid fallback transition syntax: missing target state");
        return false;
    }

    // Extract the "after Nms" clause.
    if (!parseFallbackTimeoutMsLocked(line, afterMs))
    {
        return false;
    }

    st.hasFallback     = true;
    st.fallbackAfterMs = afterMs;
    strncpy(st.fallbackTargetName, target, sizeof(st.fallbackTargetName) - 1U);
    st.fallbackTargetName[sizeof(st.fallbackTargetName) - 1U] = '\0';
    st.fallbackTargetResolved = false;

    LOG_I(TAG, "fallback: state will fall to '%s' after %" PRIu32 "ms",
          target, afterMs);
    return true;
}

bool MissionScriptEngine::parseFallbackTimeoutMsLocked(const char* line,
                                                       uint32_t&   afterMs)
{
    ARES_ASSERT(line != nullptr);

    const char* afterPtr = strstr(line, " after ");
    if (afterPtr == nullptr)
    {
        setErrorLocked("fallback transition requires 'after Nms' clause");
        return false;
    }

    const char* numStart = afterPtr + 7;  // skip " after "
    const char* msPtr    = strstr(numStart, "ms");
    if (msPtr == nullptr || msPtr == numStart)
    {
        setErrorLocked("fallback transition 'after' clause expects Nms");
        return false;
    }

    const ptrdiff_t numLen = msPtr - numStart;
    if (numLen <= 0 || numLen >= 16)
    {
        setErrorLocked("fallback transition: timeout value too long");
        return false;
    }

    char numBuf[16] = {};
    memcpy(numBuf, numStart, static_cast<size_t>(numLen));
    numBuf[static_cast<size_t>(numLen)] = '\0';

    if (!parseUint(numBuf, afterMs) || afterMs == 0U)
    {
        setErrorLocked("fallback transition: after= must be > 0 ms");
        return false;
    }

    if (!isOnlyTrailingWhitespace(msPtr + 2))
    {
        setErrorLocked("fallback transition: unexpected suffix after Nms");
        return false;
    }

    return true;
}

// ── parseOnErrorTransitionLineLocked ─────────────────────────────────────────

/**
 * @brief Parse a @c transition @c to directive inside an @c on_error: block (AMS-4.10.2).
 *
 * Syntax: @c "transition to \<STATE\>"
 *
 * When a guard condition is violated or a runtime sensor error occurs,
 * the engine enters this target state instead of halting in ERROR.
 * Multiple directives are rejected (only one recovery target per state).
 *
 * @param[in]  line  Script line starting with @c "transition to ".
 * @param[out] st    State to attach the error transition to.
 * @return @c true if the recovery transition was parsed and stored.
 * @pre  Caller holds the engine mutex.
 */
bool MissionScriptEngine::parseOnErrorTransitionLineLocked(const char* line,
                                                           StateDef&   st)
{
    ARES_ASSERT(line != nullptr);

    if (st.hasOnErrorTransition)
    {
        setErrorLocked("duplicate on_error transition in state");
        return false;
    }

    char target[ares::AMS_MAX_STATE_NAME] = {};
    // cppcheck-suppress [cert-err34-c]
    const int32_t n = static_cast<int32_t>(sscanf(line, "transition to %15s", target));
    if (n != 1 || target[0] == '\0')
    {
        setErrorLocked("invalid on_error transition syntax: missing target state");
        return false;
    }

    st.hasOnErrorTransition = true;
    strncpy(st.onErrorTransitionTarget, target,
            sizeof(st.onErrorTransitionTarget) - 1U);
    st.onErrorTransitionTarget[sizeof(st.onErrorTransitionTarget) - 1U] = '\0';
    st.onErrorTransitionResolved = false;

    LOG_I(TAG, "on_error recovery transition: -> '%s'", target);
    return true;
}

// ── parseTaskLineLocked ───────────────────────────────────────────────────────

/**
 * @brief Parse a task block header (AMS-11).
 *
 * Syntax:
 * @code
 *   task NAME:
 *   task NAME when in STATE1 STATE2 ...:
 * @endcode
 *
 * Creates a new @c TaskDef entry in @c program_.tasks[].  State-filter names
 * are stored as strings and resolved to indices by @c resolveTasksLocked().
 *
 * @param[in] line  Script line starting with @c "task ".
 * @return @c true if the task was registered.
 * @pre  Caller holds the engine mutex.
 */
bool MissionScriptEngine::parseTaskWhenInClauseLocked(
    const char* line, const char* whenIn, TaskDef& td)
{
    const ptrdiff_t nameLen = whenIn - (line + 5);
    if (nameLen <= 0 || nameLen >= static_cast<ptrdiff_t>(ares::AMS_MAX_STATE_NAME))
    {
        setErrorLocked("task: name too long or empty");
        return false;
    }
    memcpy(td.name, line + 5, static_cast<size_t>(nameLen));
    td.name[static_cast<size_t>(nameLen)] = '\0';
    trimInPlace(td.name);

    const char* stateList = whenIn + 9;
    char listBuf[ares::AMS_MAX_LINE_LEN] = {};
    strncpy(listBuf, stateList, sizeof(listBuf) - 1U);

    char* colon = strchr(listBuf, ':');
    if (colon == nullptr)
    {
        setErrorLocked("task 'when in' clause missing ':'");
        return false;
    }
    *colon = '\0';
    trimInPlace(listBuf);

    const char* p = listBuf;
    while (*p != '\0')
    {
        while (*p == ' ') { p++; }
        if (*p == '\0') { break; }

        const char* end = p;
        while (*end != ' ' && *end != '\0') { end++; }

        const ptrdiff_t tlen = end - p;
        if (tlen <= 0 || tlen >= static_cast<ptrdiff_t>(ares::AMS_MAX_STATE_NAME))
        {
            setErrorLocked("task 'when in': state name too long");
            return false;
        }
        if (td.activeStateCount >= ares::AMS_MAX_TASK_ACTIVE_STATES)
        {
            setErrorLocked("task 'when in': too many state names (AMS_MAX_TASK_ACTIVE_STATES exceeded)");
            return false;
        }
        memcpy(td.activeStateNames[td.activeStateCount], p, static_cast<size_t>(tlen));
        td.activeStateNames[td.activeStateCount][static_cast<size_t>(tlen)] = '\0';
        td.activeStateCount++;
        p = end;
    }

    if (td.activeStateCount == 0U)
    {
        setErrorLocked("task 'when in': no state names found");
        return false;
    }
    td.hasStateFilter = true;
    return true;
}

bool MissionScriptEngine::parseTaskLineLocked(const char* line)
{
    ARES_ASSERT(line != nullptr);

    if (program_.taskCount >= ares::AMS_MAX_TASKS)
    {
        setErrorLocked("too many task blocks (AMS_MAX_TASKS exceeded)");
        return false;
    }

    const uint8_t taskIdx = program_.taskCount;
    TaskDef& td = program_.tasks[taskIdx];

    const char* whenIn = strstr(line, " when in ");
    if (whenIn != nullptr)
    {
        if (!parseTaskWhenInClauseLocked(line, whenIn, td)) { return false; }
    }
    else
    {
        char name[ares::AMS_MAX_STATE_NAME] = {};
        // cppcheck-suppress [cert-err34-c]
        const int32_t n = static_cast<int32_t>(sscanf(line, "task %15[^:]:", name));
        if (n != 1 || name[0] == '\0')
        {
            setErrorLocked("invalid task syntax (expected: task NAME:)");
            return false;
        }
        strncpy(td.name, name, sizeof(td.name) - 1U);
        td.name[sizeof(td.name) - 1U] = '\0';
    }

    if (td.name[0] == '\0')
    {
        setErrorLocked("task name is empty");
        return false;
    }

    program_.taskCount++;
    parseCurrentTask_ = taskIdx;
    parseCurrentTaskRule_ = 0xFFU;

    LOG_I(TAG, "task '%s' registered (state-filter=%s)",
          td.name, td.hasStateFilter ? "yes" : "no");
    return true;
}

// ── parseTaskScopedLineLocked ─────────────────────────────────────────────────

/**
 * @brief Parse a line inside an open @c task: block (AMS-11).
 *
 * Handles:
 *  - @c "every Nms:"          — sets the task execution period.
 *  - @c "if COND:"            — opens a new conditional rule (no TC allowed).
 *  - @c "EVENT.verb \"text\"" — (inside if) emits an event on condition.
 *  - @c "set VARNAME = EXPR"  — (inside if) updates a global variable.
 *
 * @param[in]     line       Trimmed, NUL-terminated line.
 * @param[in,out] blockType  Current block context (TASK or TASK_IF).
 * @return @c true on success.
 * @pre  Caller holds the engine mutex.  parseCurrentTask_ is valid.
 */
bool MissionScriptEngine::parseTaskEveryPeriodLocked(const char* line, TaskDef& td)
{
    const char* msPtr = strstr(line, "ms:");
    if (msPtr == nullptr)
    {
        setErrorLocked("task: invalid 'every' period (expected: every Nms:)");
        return false;
    }
    const char* valueStart = line + 6;
    while (*valueStart == ' ') { valueStart++; }
    const ptrdiff_t rawLen = msPtr - valueStart;
    if (rawLen <= 0 || rawLen >= 16)
    {
        setErrorLocked("task: 'every' period value too long");
        return false;
    }
    char valueBuf[16] = {};
    memcpy(valueBuf, valueStart, static_cast<size_t>(rawLen));
    valueBuf[static_cast<size_t>(rawLen)] = '\0';
    trimInPlace(valueBuf);

    uint32_t evMs = 0;
    if (!parseUint(valueBuf, evMs) || evMs < ares::TELEMETRY_INTERVAL_MIN)
    {
        setErrorLocked("task: 'every' period must be >= 100ms");
        return false;
    }
    td.everyMs = evMs;
    return true;
}

bool MissionScriptEngine::parseTaskIfOpenLocked(const char* line, TaskDef& td,
                                                BlockType& blockType)
{
    if (td.ruleCount >= ares::AMS_MAX_TASK_RULES)
    {
        setErrorLocked("task: too many if-rules (AMS_MAX_TASK_RULES exceeded)");
        return false;
    }

    const char* condStart = line + 3;
    char condBuf[ares::AMS_MAX_LINE_LEN] = {};
    strncpy(condBuf, condStart, sizeof(condBuf) - 1U);

    char* colon = strrchr(condBuf, ':');
    if (colon == nullptr)
    {
        setErrorLocked("task: if-condition missing ':'");
        return false;
    }
    *colon = '\0';
    trimInPlace(condBuf);

    const uint8_t ruleIdx = td.ruleCount;
    TaskRule& rule = td.rules[ruleIdx];

    if (!parseOneConditionLocked(condBuf, /*allowTc=*/false, rule.cond)) { return false; }
    td.ruleCount++;
    parseCurrentTaskRule_ = ruleIdx;
    blockType = BlockType::TASK_IF;
    return true;
}

bool MissionScriptEngine::parseTaskIfBodyLocked(const char* line, TaskDef& td)
{
    if (parseCurrentTaskRule_ >= td.ruleCount) { return false; }
    TaskRule& rule = td.rules[parseCurrentTaskRule_];

    if (startsWith(line, "EVENT."))
    {
        char verb[8] = {};
        char text[ares::AMS_MAX_EVENT_TEXT] = {};
        // cppcheck-suppress [cert-err34-c]
        const int32_t n = static_cast<int32_t>(sscanf(line, "EVENT.%7[^ ] \"%63[^\"]\"", verb, text));
        if (n != 2)
        {
            setErrorLocked("task: invalid EVENT syntax (expected: EVENT.verb \"text\")");
            return false;
        }
        if      (strcmp(verb, "info")    == 0) { rule.eventVerb = EventVerb::INFO;  }
        else if (strcmp(verb, "warning") == 0) { rule.eventVerb = EventVerb::WARN;  }
        else if (strcmp(verb, "error")   == 0) { rule.eventVerb = EventVerb::ERROR; }
        else
        {
            setErrorLocked("task: unknown EVENT verb (expected: info, warning, error)");
            return false;
        }
        strncpy(rule.eventText, text, sizeof(rule.eventText) - 1U);
        rule.eventText[sizeof(rule.eventText) - 1U] = '\0';
        rule.hasEvent = true;
        return true;
    }

    if (startsWith(line, "set "))
    {
        if (rule.hasSet)
        {
            setErrorLocked("task: only one set action per if-rule is allowed");
            return false;
        }
        if (!parseSetActionCoreLocked(line, rule.setAction)) { return false; }
        rule.hasSet = true;
        return true;
    }

    setErrorLocked("unexpected line in task if-block");
    return false;
}

bool MissionScriptEngine::parseTaskScopedLineLocked(const char* line,
                                                    BlockType&  blockType)
{
    ARES_ASSERT(line != nullptr);

    if (line[0] == '\0') { return true; }

    if (parseCurrentTask_ >= program_.taskCount)
    {
        setErrorLocked("internal: task scope error");
        return false;
    }
    TaskDef& td = program_.tasks[parseCurrentTask_];

    if (startsWith(line, "every ")) { return parseTaskEveryPeriodLocked(line, td); }
    if (startsWith(line, "if "))    { return parseTaskIfOpenLocked(line, td, blockType); }

    if (blockType == BlockType::TASK_IF && parseCurrentTaskRule_ < td.ruleCount)
    {
        return parseTaskIfBodyLocked(line, td);
    }

    setErrorLocked("unexpected line in task block");
    return false;
}

// ── parseAssertLineLocked ─────────────────────────────────────────────────────

/**
 * @brief Parse one directive inside an @c assert: block (AMS-15).
 *
 * Supported directives:
 * @code
 *   reachable STATE_NAME          -- BFS reachability from initial state.
 *   no_dead_states                -- all states are reachable.
 *   max_transition_depth < N      -- longest simple path < N.
 * @endcode
 *
 * Assertions are evaluated after all states are resolved; parse errors are
 * recorded and prevent the script from activating.
 *
 * @param[in] line  Trimmed directive line.
 * @return @c true if the directive was stored.
 * @pre  Caller holds the engine mutex.
 */
bool MissionScriptEngine::parseAssertLineLocked(const char* line)
{
    ARES_ASSERT(line != nullptr);

    if (line[0] == '\0') { return true; }

    if (program_.assertCount >= ares::AMS_MAX_ASSERTS)
    {
        setErrorLocked("too many assert directives (AMS_MAX_ASSERTS exceeded)");
        return false;
    }

    AssertDef& ad = program_.asserts[program_.assertCount];

    // "reachable STATE_NAME"
    if (startsWith(line, "reachable "))
    {
        char name[ares::AMS_MAX_STATE_NAME] = {};
        // cppcheck-suppress [cert-err34-c]
        const int32_t n = static_cast<int32_t>(sscanf(line, "reachable %15s", name));
        if (n != 1 || name[0] == '\0')
        {
            setErrorLocked("assert: invalid 'reachable' syntax (expected: reachable STATE)");
            return false;
        }
        ad.kind = AssertKind::REACHABLE;
        strncpy(ad.targetName, name, sizeof(ad.targetName) - 1U);
        ad.targetName[sizeof(ad.targetName) - 1U] = '\0';
        program_.assertCount++;
        return true;
    }

    // "no_dead_states"
    if (strcmp(line, "no_dead_states") == 0)
    {
        ad.kind = AssertKind::NO_DEAD_STATES;
        program_.assertCount++;
        return true;
    }

    // "max_transition_depth < N"
    if (startsWith(line, "max_transition_depth"))
    {
        uint32_t limit = 0;
        // cppcheck-suppress [cert-err34-c]
        const int32_t nr = static_cast<int32_t>(sscanf(line, "max_transition_depth < %u", &limit));  // NOLINT(bugprone-unchecked-string-to-number-conversion)
        if (nr != 1 || limit == 0U || limit > 255U)
        {
            setErrorLocked("assert: invalid 'max_transition_depth' (expected: max_transition_depth < N, 1<=N<=255)");
            return false;
        }
        ad.kind       = AssertKind::MAX_DEPTH;
        ad.numericArg = static_cast<uint8_t>(limit);
        program_.assertCount++;
        return true;
    }

    setErrorLocked("assert: unknown directive");
    return false;
}

// ── resolveTasksLocked ────────────────────────────────────────────────────────

/**
 * @brief Resolve task 'when in' state-filter names to numeric indices (AMS-11).
 *
 * Called from @c parseScriptLocked() after @c resolveTransitionsLocked().
 * A task that references an unknown state name causes a parse error (fail-hard).
 *
 * @return @c true if all state names resolved successfully.
 * @pre  Caller holds the engine mutex.  All states have been parsed.
 */
bool MissionScriptEngine::resolveTasksLocked()
{
    for (uint8_t i = 0; i < program_.taskCount; i++)
    {
        TaskDef& td = program_.tasks[i];

        if (!td.hasStateFilter)
        {
            td.stateIndicesResolved = true;
            continue;
        }

        for (uint8_t j = 0; j < td.activeStateCount; j++)
        {
            const uint8_t idx = findStateByNameLocked(td.activeStateNames[j]);
            if (idx >= program_.stateCount)
            {
                static char msg[80] = {};
                snprintf(msg, sizeof(msg),
                         "task '%s': unknown state '%s' in 'when in' filter",
                         td.name, td.activeStateNames[j]);
                setErrorLocked(msg);
                return false;
            }
            td.activeStateIndices[j] = idx;
        }
        td.stateIndicesResolved = true;
    }
    return true;
}

// ── validateAssertionsLocked ──────────────────────────────────────────────────

/**
 * @brief Evaluate formal assertions against the fully-resolved transition graph (AMS-15).
 *
 * Performs graph analysis entirely on the stack (no heap).  Uses a
 * @c uint16_t bitmask for reachability (AMS_MAX_STATES ≤ 16).
 * Iterative DFS with path-tracking detects cycles and measures depth.
 *
 * Called from @c parseScriptLocked() as the final validation step.
 * Any failure sets the engine error and prevents activation.
 *
 * @return @c true if all assertions hold.
 * @pre  Caller holds the engine mutex.  resolveTransitionsLocked() has run.
 */
void MissionScriptEngine::computeBfsReachabilityLocked(uint16_t& reachable) const
{
    reachable = 0U;
    uint8_t queue[ares::AMS_MAX_STATES] = {};
    uint8_t head = 0U;
    uint8_t tail = 0U;

    queue[tail++] = 0U;
    reachable |= static_cast<uint16_t>(1U << 0U);

    while (head < tail)
    {
        const uint8_t s = queue[head++];
        const StateDef& sd = program_.states[s];

        auto visit = [&](uint8_t t)
        {
            if (t < program_.stateCount &&
                !(reachable & static_cast<uint16_t>(1U << t)))
            {
                reachable |= static_cast<uint16_t>(1U << t);
                queue[tail++] = t;
            }
        };

        // Regular transitions (AMS-4.6): all slots feed the BFS.
        for (uint8_t ti = 0U; ti < sd.transitionCount; ti++)
        {
            if (sd.transitions[ti].targetResolved)
            {
                visit(sd.transitions[ti].targetIndex);
            }
        }
        if (sd.hasFallback && sd.fallbackTargetResolved)
        {
            visit(sd.fallbackTargetIdx);
        }
        if (sd.hasOnErrorTransition && sd.onErrorTransitionResolved)
        {
            visit(sd.onErrorTransitionIdx);
        }
    }
}

void MissionScriptEngine::computeDfsMaxDepthLocked(uint8_t& maxDepth, bool& hasCycle) const
{
    /** Iterative DFS stack frame for cycle detection and max-depth computation. */
    struct DfsFrame
    {
        uint8_t  state;
        uint8_t  depth;
        uint16_t pathMask;
        uint8_t  child;
    };

    maxDepth = 0U;
    hasCycle = false;

    DfsFrame dfsStack[ares::AMS_MAX_STATES + 1U] = {};
    uint8_t  dfsTop = 0U;

    dfsStack[dfsTop++] = { 0U, 0U, static_cast<uint16_t>(1U << 0U), 0U };

    while (dfsTop > 0U)
    {
        DfsFrame& f = dfsStack[dfsTop - 1U];

        if (f.depth > maxDepth) { maxDepth = f.depth; }

        bool pushed = false;
        const StateDef& sd = program_.states[f.state];

        // Successor slots: [0..transitionCount-1] = regular transitions,
        //                    transitionCount         = fallback,
        //                    transitionCount+1       = on_error.
        const uint8_t totalSuccessors =
            static_cast<uint8_t>(sd.transitionCount + 2U);

        while (f.child < totalSuccessors && !pushed && !hasCycle)
        {
            uint8_t suc = ares::AMS_MAX_STATES;
            if (f.child < sd.transitionCount)
            {
                if (sd.transitions[f.child].targetResolved)
                {
                    suc = sd.transitions[f.child].targetIndex;
                }
            }
            else if (f.child == sd.transitionCount
                     && sd.hasFallback && sd.fallbackTargetResolved)
            {
                suc = sd.fallbackTargetIdx;
            }
            else if (f.child == static_cast<uint8_t>(sd.transitionCount + 1U)
                     && sd.hasOnErrorTransition && sd.onErrorTransitionResolved)
            {
                suc = sd.onErrorTransitionIdx;
            }
            f.child++;

            if (suc < program_.stateCount)
            {
                if (f.pathMask & static_cast<uint16_t>(1U << suc))
                {
                    hasCycle = true;
                    break;
                }
                if (dfsTop < static_cast<uint8_t>(ares::AMS_MAX_STATES + 1U))
                {
                    dfsStack[dfsTop++] = {
                        suc,
                        static_cast<uint8_t>(f.depth + 1U),
                        static_cast<uint16_t>(f.pathMask | static_cast<uint16_t>(1U << suc)),
                        0U
                    };
                    pushed = true;
                }
            }
        }

        if (!pushed && !hasCycle) { dfsTop--; }
    }
}

bool MissionScriptEngine::evaluateOneAssertionLocked( // NOLINT(readability-function-size)
    const AssertDef& ad, uint16_t reachable, uint8_t maxDepth, bool hasCycle)
{
    switch (ad.kind)
    {
    case AssertKind::REACHABLE:
    {
        const uint8_t idx = findStateByNameLocked(ad.targetName);
        if (idx >= program_.stateCount)
        {
            static char msg[80] = {};
            snprintf(msg, sizeof(msg), "assert reachable: unknown state '%s'", ad.targetName);
            setErrorLocked(msg);
            return false;
        }
        if (!(reachable & static_cast<uint16_t>(1U << idx)))
        {
            static char msg[80] = {};
            snprintf(msg, sizeof(msg),
                     "assert reachable: '%s' is not reachable from initial state",
                     ad.targetName);
            setErrorLocked(msg);
            return false;
        }
        LOG_I(TAG, "assert reachable '%s': PASS", ad.targetName);
        return true;
    }

    case AssertKind::NO_DEAD_STATES:
    {
        const uint16_t allMask = static_cast<uint16_t>(
            (static_cast<uint32_t>(1U) << program_.stateCount) - 1U);
        if ((reachable & allMask) != allMask)
        {
            for (uint8_t s = 0U; s < program_.stateCount; s++)
            {
                if (!(reachable & static_cast<uint16_t>(1U << s)))
                {
                    static char msg[80] = {};
                    snprintf(msg, sizeof(msg),
                             "assert no_dead_states: '%s' is unreachable",
                             program_.states[s].name);
                    setErrorLocked(msg);
                    return false;
                }
            }
        }
        LOG_I(TAG, "assert no_dead_states: PASS (all %u states reachable)",
              static_cast<unsigned>(program_.stateCount));
        return true;
    }

    case AssertKind::MAX_DEPTH:
    {
        if (hasCycle)
        {
            static char msg[96] = {};
            snprintf(msg, sizeof(msg),
                     "assert max_transition_depth < %u: cycle detected (unbounded depth)",
                     static_cast<unsigned>(ad.numericArg));
            setErrorLocked(msg);
            return false;
        }
        if (maxDepth >= static_cast<uint8_t>(ad.numericArg))
        {
            static char msg[96] = {};
            snprintf(msg, sizeof(msg),
                     "assert max_transition_depth < %u: actual depth is %u",
                     static_cast<unsigned>(ad.numericArg),
                     static_cast<unsigned>(maxDepth));
            setErrorLocked(msg);
            return false;
        }
        LOG_I(TAG, "assert max_transition_depth < %u: PASS (depth=%u)",
              static_cast<unsigned>(ad.numericArg),
              static_cast<unsigned>(maxDepth));
        return true;
    }

    default:
        return true;
    }
}

bool MissionScriptEngine::validateAssertionsLocked()
{
    if (program_.assertCount == 0U) { return true; }
    if (program_.stateCount  == 0U) { return true; }

    static_assert(ares::AMS_MAX_STATES <= 16U,
                  "validateAssertionsLocked: BFS bitmask requires AMS_MAX_STATES <= 16");

    uint16_t reachable = 0U;
    computeBfsReachabilityLocked(reachable);

    uint8_t maxDepth = 0U;
    bool    hasCycle = false;
    computeDfsMaxDepthLocked(maxDepth, hasCycle);

    for (uint8_t i = 0U; i < program_.assertCount; i++)
    {
        if (!evaluateOneAssertionLocked(program_.asserts[i], reachable, maxDepth, hasCycle))
        {
            return false;
        }
    }

    return true;
}

} // namespace ams
} // namespace ares

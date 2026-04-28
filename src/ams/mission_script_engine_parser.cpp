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

using ares::proto::NODE_BROADCAST;
using ares::proto::NODE_GROUND;
using ares::proto::NODE_PAYLOAD;
using ares::proto::NODE_ROCKET;

using detail::isOnlyTrailingWhitespace;

static constexpr const char* TAG = "AMS";

// ── loadFromStorageLocked ─────────────────────────────────────────────────────

/**
 * @brief Load, read, and parse a .ams script file from LittleFS.
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
bool MissionScriptEngine::parseScriptLocked(const char* script, uint32_t length)
{
    ARES_ASSERT(script != nullptr);
    ARES_ASSERT(length <= ares::AMS_MAX_SCRIPT_BYTES);

    program_ = {};
    program_.apid   = ares::AMS_DEFAULT_APID;
    program_.nodeId = ares::DEFAULT_NODE_ID;

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

    return true;
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

    uint32_t lineLen    = 0;
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
        offset++;
    }

    if (offset < length && script[offset] == '\n')
    {
        offset++;
    }

    if (lineTooLong)
    {
        setErrorLocked("script line exceeds AMS_MAX_LINE_LEN");
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

    if (strcmp(line, "}") == 0)
    {
        blockType = BlockType::NONE;
        return true;
    }

    if (startsWith(line, "include "))
    {
        return parseIncludeLineLocked(line);
    }

    if (startsWith(line, "pus.service "))
    {
        return true;  // informational; no runtime effect
    }

    if (startsWith(line, "pus.apid"))
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

    if (startsWith(line, "state "))
    {
        blockType = BlockType::NONE;
        return parseStateLineLocked(line, currentState);
    }

    // AMS-4.8: global variable declaration (valid before first state).
    if (startsWith(line, "var "))
    {
        return parseVarLineLocked(line);
    }

    if (currentState >= program_.stateCount)
    {
        setErrorLocked("statement outside state block");
        return false;
    }

    StateDef& st = program_.states[currentState];
    return parseStateScopedLineLocked(line, st, blockType);
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

    // ── Block headers always take priority — they reset context ────────
    if (startsWith(line, "on_enter:"))   { blockType = BlockType::ON_ENTER;   return true; }
    if (startsWith(line, "on_error:"))   { blockType = BlockType::ON_ERROR;   return true; }
    if (startsWith(line, "conditions:")) { blockType = BlockType::CONDITIONS; return true; }

    if (startsWith(line, "every "))      { blockType = BlockType::NONE; return parseEveryLineLocked(line, st); }
    if (startsWith(line, "log_every "))  { blockType = BlockType::NONE; return parseLogEveryLineLocked(line, st); }
    if (startsWith(line, "priorities ")) { blockType = BlockType::NONE; return parsePrioritiesLineLocked(line, st); }

    if (startsWith(line, "HK.report"))
    {
        if (!st.hasHkEvery)
        {
            setErrorLocked("HK.report requires every block");
            return false;
        }
        blockType = BlockType::HK;
        return true;
    }
    if (startsWith(line, "LOG.report"))
    {
        if (!st.hasLogEvery)
        {
            setErrorLocked("LOG.report requires log_every block");
            return false;
        }
        blockType = BlockType::LOG;
        return true;
    }
    if (startsWith(line, "transition to "))
    {
        blockType = BlockType::NONE;
        return parseTransitionLineLocked(line, st);
    }

    // ── Block content — dispatched by current blockType ─────────────────
    if (blockType == BlockType::CONDITIONS) { return parseConditionScopedLineLocked(line, st); }

    if (blockType == BlockType::ON_ERROR)
    {
        if (startsWith(line, "EVENT.")) { return parseOnErrorEventLineLocked(line, st); }
        setErrorLocked("only EVENT.* is allowed inside on_error");
        return false;
    }

    if (blockType == BlockType::ON_ENTER)
    {
        if (startsWith(line, "EVENT.")) { return parseEventLineLocked(line, st); }
        if (startsWith(line, "set "))   { return parseSetActionLineLocked(line, st); }
        setErrorLocked("only EVENT.* and set are allowed inside on_enter");
        return false;
    }

    if (blockType == BlockType::HK)  { return parseFieldLineLocked(line, st.hkFields,  st.hkFieldCount,  "HK");  }
    if (blockType == BlockType::LOG) { return parseFieldLineLocked(line, st.logFields, st.logFieldCount, "LOG"); }

    // ── NONE context: EVENT.* belongs to on_enter (legacy flat form) ────
    if (startsWith(line, "EVENT.")) { return parseEventLineLocked(line, st); }

    setErrorLocked("unsupported statement");
    return false;
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
bool MissionScriptEngine::parseIncludeLineLocked(const char* line)
{
    char model[16] = {};
    char alias[16] = {};
    // cppcheck-suppress [cert-err34-c]
    const int parsed = sscanf(line, "include %15s as %15s", model, alias);
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

    for (uint8_t i = 0; i < gpsCount_ && driverIdx == 0xFFU; i++)
    {
        if (gpsDrivers_[i].model != nullptr
            && strcmp(gpsDrivers_[i].model, model) == 0)
        {
            kind = PeripheralKind::GPS; driverIdx = i;
        }
    }
    for (uint8_t i = 0; i < baroCount_ && driverIdx == 0xFFU; i++)
    {
        if (baroDrivers_[i].model != nullptr
            && strcmp(baroDrivers_[i].model, model) == 0)
        {
            kind = PeripheralKind::BARO; driverIdx = i;
        }
    }
    for (uint8_t i = 0; i < comCount_ && driverIdx == 0xFFU; i++)
    {
        if (comDrivers_[i].model != nullptr
            && strcmp(comDrivers_[i].model, model) == 0)
        {
            kind = PeripheralKind::COM; driverIdx = i;
        }
    }
    for (uint8_t i = 0; i < imuCount_ && driverIdx == 0xFFU; i++)
    {
        if (imuDrivers_[i].model != nullptr
            && strcmp(imuDrivers_[i].model, model) == 0)
        {
            kind = PeripheralKind::IMU; driverIdx = i;
        }
    }

    if (driverIdx == 0xFFU)
    {
        char msg[64] = {};
        snprintf(msg, sizeof(msg),
                 "model '%s' not found in compiled-in drivers", model);
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

    LOG_I(TAG, "include: alias=%s model=%s kind=%u idx=%u",
          ae.alias, ae.model,
          static_cast<uint32_t>(ae.kind),
          static_cast<uint32_t>(ae.driverIdx));
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
    const int n = sscanf(line, "state %15[^:]:", stateName);
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
    const int n = sscanf(line, "EVENT.%7[^ ] \"%63[^\"]\"", verb, text);
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
    int n = sscanf(line,
                   "priorities event=%" SCNu32 " hk=%" SCNu32
                   " log=%" SCNu32 " budget=%" SCNu32,
                   &event, &hk, &log, &budget);
    if (n == 4) { return true; }

    n = sscanf(line,
               "priorities event=%" SCNu32 " hk=%" SCNu32
               " log=%" SCNu32,
               &event, &hk, &log);
    if (n == 3) { budget = st.actionBudget; return true; }

    n = sscanf(line,
               "priorities hk=%" SCNu32 " log=%" SCNu32
               " budget=%" SCNu32,
               &hk, &log, &budget);
    if (n == 3) { event = st.eventPriority; return true; }

    n = sscanf(line,
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
    const int n = sscanf(line, "%19[^:]: %31s", key, expr);
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
        char msg[96] = {};
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
        char msg[80] = {};
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
    const int n4 = sscanf(condStr, "%19s %19s %3s %19s", tok1, tok2, tok3, tok4);
    if (n4 == 4 && strcmp(tok2, "delta") == 0)
    {
        // tok1 = ALIAS.field, tok3 = op (</>), tok4 = threshold
        char aliasStr[16] = {};
        char fieldStr[20] = {};
        if (!splitAliasDotField(tok1, aliasStr, sizeof(aliasStr),
                                        fieldStr, sizeof(fieldStr)))
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
            char msg[80] = {};
            snprintf(msg, sizeof(msg),
                     "field '%s' not valid for alias '%s'", fieldStr, aliasStr);
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

    // 2-token form: ALIAS.field falling | rising
    const int n2 = sscanf(condStr, "%19s %19s", tok1, tok2);
    if (n2 == 2 && (strcmp(tok2, "falling") == 0 || strcmp(tok2, "rising") == 0))
    {
        char aliasStr[16] = {};
        char fieldStr[20] = {};
        if (!splitAliasDotField(tok1, aliasStr, sizeof(aliasStr),
                                        fieldStr, sizeof(fieldStr)))
        {
            setErrorLocked("invalid falling/rising LHS (expected ALIAS.field)");
            return false;
        }

        const AliasEntry* ae = findAliasLocked(aliasStr);
        if (ae == nullptr)
        {
            char msg[72] = {};
            snprintf(msg, sizeof(msg), "unknown alias '%s' in falling/rising condition",
                     aliasStr);
            setErrorLocked(msg);
            return false;
        }

        SensorField sf = SensorField::ALT;
        if (!parseSensorField(ae->kind, fieldStr, sf))
        {
            char msg[80] = {};
            snprintf(msg, sizeof(msg),
                     "field '%s' not valid for alias '%s'", fieldStr, aliasStr);
            setErrorLocked(msg);
            return false;
        }

        out.kind = (strcmp(tok2, "falling") == 0)
                   ? CondKind::SENSOR_DELTA_LT
                   : CondKind::SENSOR_DELTA_GT;
        strncpy(out.alias, aliasStr, sizeof(out.alias) - 1U);
        out.alias[sizeof(out.alias) - 1U] = '\0';
        out.field     = sf;
        out.threshold = 0.0f;  // falling = delta < 0; rising = delta > 0
        return true;
    }

    // 3-token form: standard LHS OP RHS (delegates to parseCondExprLocked)
    char lhs[20] = {};
    char op[3]   = {};
    char rhs[20] = {};
    const int n3 = sscanf(condStr, "%19s %2s %19s", lhs, op, rhs);
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
bool MissionScriptEngine::parseTransitionLineLocked(const char* line,
                                                    StateDef&   st)
{
    ARES_ASSERT(line != nullptr);

    // ── 1. Extract target name ────────────────────────────────────────────
    const char* const whenKw = strstr(line, " when ");
    if (whenKw == nullptr)
    {
        setErrorLocked("invalid transition syntax: missing 'when'");
        return false;
    }

    // "transition to TARGET when ..."
    char target[ares::AMS_MAX_STATE_NAME] = {};
    const int nTarget = sscanf(line, "transition to %15s", target);
    if (nTarget != 1 || target[0] == '\0')
    {
        setErrorLocked("invalid transition syntax: missing target state");
        return false;
    }

    // ── 2. Extract conditions body (after "when") ─────────────────────────
    const char* body = whenKw + 6;  // skip " when "
    while (*body == ' ') { body++; }

    // Copy body into a mutable buffer so we can NUL-terminate segments.
    char bodyBuf[ares::AMS_MAX_LINE_LEN] = {};
    strncpy(bodyBuf, body, sizeof(bodyBuf) - 1U);
    bodyBuf[sizeof(bodyBuf) - 1U] = '\0';

    // ── 3. Extract optional "for <N>ms" suffix ────────────────────────────
    uint32_t holdMs = 0U;
    // Search for " for " near the end of body.
    char* forPtr = nullptr;
    {
        char* search = bodyBuf;
        while ((search = strstr(search, " for ")) != nullptr)
        {
            forPtr  = search;  // keep last occurrence
            search += 5;
        }
    }
    if (forPtr != nullptr)
    {
        const char* forVal = forPtr + 5;  // skip " for "
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
        *forPtr = '\0';  // trim "for Nms" from body
        // Trim trailing space left by the removal.
        uint32_t bLen = static_cast<uint32_t>(strnlen(bodyBuf, sizeof(bodyBuf)));
        while (bLen > 0U && (bodyBuf[bLen - 1U] == ' ' || bodyBuf[bLen - 1U] == '\t'))
        {
            bodyBuf[--bLen] = '\0';
        }
    }

    // ── 4. Split by " or " / " and " — detect logic ───────────────────────
    // Scan for the first logic keyword and verify all separators are identical.
    TransitionLogic logic = TransitionLogic::AND;
    bool logicDetected = false;

    // Collect condition sub-strings into condParts[].
    const char* condParts[ares::AMS_MAX_TRANSITION_CONDS] = {};
    char        condBufs[ares::AMS_MAX_TRANSITION_CONDS][ares::AMS_MAX_LINE_LEN] = {};
    uint8_t     condCount = 0;

    // Work through bodyBuf, splitting on " or " / " and ".
    char* remaining = bodyBuf;
    while (remaining != nullptr && remaining[0] != '\0')
    {
        // Search for both keywords.
        char* orPtr  = strstr(remaining, " or ");
        char* andPtr = strstr(remaining, " and ");

        // Pick whichever appears first.
        char*           splitAt    = nullptr;
        TransitionLogic splitLogic = TransitionLogic::AND;
        uint8_t         skipLen    = 0;

        if (orPtr != nullptr && (andPtr == nullptr || orPtr < andPtr))
        {
            splitAt    = orPtr;
            splitLogic = TransitionLogic::OR;
            skipLen    = 4U;  // " or "
        }
        else if (andPtr != nullptr)
        {
            splitAt    = andPtr;
            splitLogic = TransitionLogic::AND;
            skipLen    = 5U;  // " and "
        }

        if (splitAt != nullptr)
        {
            // Validate homogeneous logic.
            if (!logicDetected)
            {
                logic         = splitLogic;
                logicDetected = true;
            }
            else if (splitLogic != logic)
            {
                setErrorLocked("mixed 'or'/'and' in one transition is not supported");
                return false;
            }
        }

        // Extract the condition part before the split point (or whole string).
        const size_t partLen = (splitAt != nullptr)
                               ? static_cast<size_t>(splitAt - remaining)
                               : strnlen(remaining, sizeof(bodyBuf));

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

    // ── 5. Parse each sub-condition ───────────────────────────────────────
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
            return false;  // error already set
        }
    }
    tr.condCount = condCount;

    st.hasTransition = true;
    st.transition    = tr;
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

    const int n = sscanf(line, "%19s %2s %19s", lhs, op, rhs);
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

    const int n = sscanf(line, "EVENT.%7[^ ] \"%63[^\"]\"", verb, text);
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
        if (!st.hasTransition) { continue; }

        const uint8_t idx = findStateByNameLocked(st.transition.targetName);
        if (idx >= program_.stateCount)
        {
            setErrorLocked("unknown transition target state");
            return false;
        }

        st.transition.targetIndex    = idx;
        st.transition.targetResolved = true;
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
    const int n = sscanf(line, "var %15s = %23s", name, valBuf);
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

// ── parseSetActionLineLocked ─────────────────────────────────────────────────

/**
 * @brief Parse a @c set action inside an @c on_enter: block (AMS-4.8).
 *
 * Two forms are accepted:
 *
 *   @code
 *   set VARNAME = ALIAS.field
 *   set VARNAME = CALIBRATE(ALIAS.field, N)
 *   @endcode
 *
 * The second form reads the sensor @p N times (1–10) and stores the
 * arithmetic mean of valid readings.  If all reads fail, the variable
 * retains its previous value and an EVENT.warning is emitted (NaN guard).
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

    char varName[ares::AMS_VAR_NAME_LEN] = {};
    char rhsBuf[64] = {};

    // cppcheck-suppress [cert-err34-c]
    const int n = sscanf(line, "set %15s = %63[^\n]", varName, rhsBuf);
    if (n != 2 || varName[0] == '\0' || rhsBuf[0] == '\0')
    {
        setErrorLocked("invalid set syntax (expected: set VARNAME = ALIAS.field)");
        return false;
    }

    // Validate that the variable was declared.
    bool varFound = false;
    for (uint8_t i = 0; i < program_.varCount; i++)
    {
        if (strcmp(program_.vars[i].name, varName) == 0) { varFound = true; break; }
    }
    if (!varFound)
    {
        char msg[64] = {};
        snprintf(msg, sizeof(msg), "set action: undefined variable '%s'", varName);
        setErrorLocked(msg);
        return false;
    }

    SetAction& act = st.setActions[st.setActionCount];
    strncpy(act.varName, varName, sizeof(act.varName) - 1U);
    act.varName[sizeof(act.varName) - 1U] = '\0';

    trimInPlace(rhsBuf);

    // ── CALIBRATE(ALIAS.field, N) form ────────────────────────────────
    if (startsWith(rhsBuf, "CALIBRATE("))
    {
        // Parse: CALIBRATE(ALIAS.field, N)
        char sensorExpr[32] = {};
        char nBuf[8]        = {};
        // cppcheck-suppress [cert-err34-c]
        const int nc = sscanf(rhsBuf, "CALIBRATE(%31[^,], %7[^)])", sensorExpr, nBuf);
        if (nc != 2)
        {
            setErrorLocked("invalid CALIBRATE syntax: expected CALIBRATE(ALIAS.field, N)");
            return false;
        }
        trimInPlace(sensorExpr);
        trimInPlace(nBuf);

        uint32_t samples = 0;
        if (!parseUint(nBuf, samples) || samples == 0U || samples > 10U)
        {
            setErrorLocked("CALIBRATE sample count must be 1-10");
            return false;
        }

        char aliasStr[16] = {};
        char fieldStr[20] = {};
        if (!splitAliasDotField(sensorExpr,
                                aliasStr, sizeof(aliasStr),
                                fieldStr, sizeof(fieldStr)))
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
            char msg[80] = {};
            snprintf(msg, sizeof(msg),
                     "CALIBRATE: field '%s' not valid for alias '%s'",
                     fieldStr, aliasStr);
            setErrorLocked(msg);
            return false;
        }

        strncpy(act.alias, aliasStr, sizeof(act.alias) - 1U);
        act.alias[sizeof(act.alias) - 1U] = '\0';
        act.field        = sf;
        act.calibrate    = true;
        act.calibSamples = static_cast<uint8_t>(samples);
        st.setActionCount++;
        return true;
    }

    // ── Simple form: ALIAS.field ─────────────────────────────────────
    char aliasStr[16] = {};
    char fieldStr[20] = {};
    if (!splitAliasDotField(rhsBuf,
                            aliasStr, sizeof(aliasStr),
                            fieldStr, sizeof(fieldStr)))
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
        char msg[80] = {};
        snprintf(msg, sizeof(msg),
                 "set action: field '%s' not valid for alias '%s'",
                 fieldStr, aliasStr);
        setErrorLocked(msg);
        return false;
    }

    strncpy(act.alias, aliasStr, sizeof(act.alias) - 1U);
    act.alias[sizeof(act.alias) - 1U] = '\0';
    act.field     = sf;
    act.calibrate = false;
    st.setActionCount++;
    return true;
}

} // namespace ams
} // namespace ares

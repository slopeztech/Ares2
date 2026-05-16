/**
 * @file  mission_script_engine_parser_state.cpp
 * @brief AMS engine â€” state block parsing (every, log_every, priorities, HK/LOG fields, events).
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
static constexpr const char* TAG = "AMS";
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
        //
        // AMS-4.10.3: inside an on_timeout: block the 'transition to'
        // directive is a timeout-forced transition (no 'when' clause).
        // Defer to parseStateBlockContentLocked which calls
        // parseOnTimeoutTransitionLineLocked in that case.
        if (blockType == BlockType::ON_ERROR || blockType == BlockType::ON_TIMEOUT)
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

bool MissionScriptEngine::parseOnErrorBlockLineLocked(const char* line, StateDef& st)
{
    char evtPrefix[20] = {};
    snprintf(evtPrefix, sizeof(evtPrefix), "%s.", program_.eventAlias);
    if (startsWith(line, evtPrefix))        { return parseOnErrorEventLineLocked(line, st); }
    if (startsWith(line, "transition to ")) { return parseOnErrorTransitionLineLocked(line, st); }
    setErrorLocked("only EVENT.* or 'transition to' allowed inside on_error");
    return false;
}

bool MissionScriptEngine::parseOnEnterBlockLineLocked(const char* line, StateDef& st)
{
    char evtPrefix[20] = {};
    snprintf(evtPrefix, sizeof(evtPrefix), "%s.", program_.eventAlias);
    if (startsWith(line, evtPrefix))       { return parseEventLineLocked(line, st); }
    if (startsWith(line, "set "))          { return parseSetActionLineLocked(line, st); }
    if (startsWith(line, "PULSE.fire "))   { return parsePulseFireLineLocked(line, st); }  // AMS-4.17
    setErrorLocked("only EVENT.*, set and PULSE.fire are allowed inside on_enter");
    return false;
}

bool MissionScriptEngine::parseOnExitBlockLineLocked(const char* line, StateDef& st)
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

bool MissionScriptEngine::parseOnTimeoutContentLineLocked(const char* line, StateDef& st)
{
    char evtPrefix[20] = {};
    snprintf(evtPrefix, sizeof(evtPrefix), "%s.", program_.eventAlias);
    if (startsWith(line, evtPrefix))        { return parseOnTimeoutEventLineLocked(line, st); }
    if (startsWith(line, "transition to ")) { return parseOnTimeoutTransitionLineLocked(line, st); }
    setErrorLocked("only EVENT.* and 'transition to' allowed inside on_timeout");
    return false;
}

bool MissionScriptEngine::parseHkSlotFieldLineLocked(const char* line, StateDef& st)
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

bool MissionScriptEngine::parseLogSlotFieldLineLocked(const char* line, StateDef& st)
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

bool MissionScriptEngine::parseStateBlockContentLocked(const char* line,
                                                       StateDef&   st,
                                                       BlockType   blockType,
                                                       bool&       handled)
{
    handled = true;
    if (blockType == BlockType::CONDITIONS) { return parseConditionScopedLineLocked(line, st); }
    if (blockType == BlockType::ON_ERROR)   { return parseOnErrorBlockLineLocked(line, st); }
    if (blockType == BlockType::ON_ENTER)   { return parseOnEnterBlockLineLocked(line, st); }
    if (blockType == BlockType::ON_EXIT)    { return parseOnExitBlockLineLocked(line, st); }
    if (blockType == BlockType::ON_TIMEOUT) { return parseOnTimeoutContentLineLocked(line, st); }
    if (blockType == BlockType::HK)         { return parseHkSlotFieldLineLocked(line, st); }
    if (blockType == BlockType::LOG)        { return parseLogSlotFieldLineLocked(line, st); }
    handled = false;
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
    ares::util::copyZ(st.onEnterText, text, sizeof(st.onEnterText));

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
    if (!detail::isOnlyTrailingWhitespace(marker + 3))
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
    if (!detail::isOnlyTrailingWhitespace(marker + 3))
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

    if (st.prioritiesSet)
    {
        setErrorLocked("duplicate priorities block in state");
        return false;
    }

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
    st.prioritiesSet = true;
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
    // Safe: all fields are SCNu32; no string buffers; input is a NUL-terminated script line;
    // return value is checked against the expected field count before any field is used.
    n = static_cast<int32_t>(sscanf(line,  // NOLINT(bugprone-unchecked-string-to-number-conversion)
                   "priorities event=%" SCNu32 " hk=%" SCNu32
                   " log=%" SCNu32 " budget=%" SCNu32,
                   &event, &hk, &log, &budget));
    if (n == 4) { return true; }

    // Safe: see comment on first sscanf above — same justification applies to all variants.
    n = static_cast<int32_t>(sscanf(line,  // NOLINT(bugprone-unchecked-string-to-number-conversion)
               "priorities event=%" SCNu32 " hk=%" SCNu32
               " log=%" SCNu32,
               &event, &hk, &log));
    if (n == 3) { budget = st.actionBudget; return true; }

    // Safe: see comment on first sscanf above — same justification applies to all variants.
    n = static_cast<int32_t>(sscanf(line,  // NOLINT(bugprone-unchecked-string-to-number-conversion)
               "priorities hk=%" SCNu32 " log=%" SCNu32
               " budget=%" SCNu32,
               &hk, &log, &budget));
    if (n == 3) { event = st.eventPriority; return true; }

    // Safe: see comment on first sscanf above — same justification applies to all variants.
    n = sscanf(line,  // NOLINT(bugprone-unchecked-string-to-number-conversion)
               "priorities hk=%" SCNu32 " log=%" SCNu32,
               &hk, &log);
    if (n == 2) { event = st.eventPriority; budget = st.actionBudget; return true; }

    return false;
}

// ── storeVarFieldLocked / storeAliasFieldLocked ───────────────────────────────

bool MissionScriptEngine::storeVarFieldLocked(const char* expr, const char* key,
                                               HkField*    fields, uint8_t& count,
                                               const char* ctxName)
{
    if (strlen(expr) >= sizeof(fields[0].alias))  // sizeof: compile-time, no eval
    {
        static char msg[64] = {};
        snprintf(msg, sizeof(msg), "variable name too long in %s field", ctxName);
        setErrorLocked(msg);
        return false;
    }
    const VarEntry* ve = findVarLocked(expr);
    if (ve == nullptr)
    {
        static char msg[128] = {};
        snprintf(msg, sizeof(msg),
                 "invalid %s expression '%s' (expected ALIAS.field or variable name)",
                 ctxName, expr);
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
    HkField& slot = fields[count++];
    ares::util::copyZ(slot.label, key,  sizeof(slot.label));
    ares::util::copyZ(slot.alias, expr, sizeof(slot.alias));
    slot.field = SensorField::VAR;
    return true;
}

bool MissionScriptEngine::storeAliasFieldLocked(const char* aliasStr,
                                                 const char* fieldStr,
                                                 const char* key,
                                                 HkField*    fields,
                                                 uint8_t&    count,
                                                 const char* ctxName)
{
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
    HkField& slot = fields[count++];
    ares::util::copyZ(slot.label, key,      sizeof(slot.label));
    ares::util::copyZ(slot.alias, aliasStr, sizeof(slot.alias));
    slot.field = sf;
    return true;
}

// ── parseFieldLineLocked ─────────────────────────────────────────────────────

/**
 * @brief Parse one @c KEY: ALIAS.field or @c KEY: varname entry into a @c HkField slot.
 *
 * Used for both @c HK.report and @c LOG.report field lists.
 * When the expression contains no @c '.' the parser checks whether it matches
 * a declared AMS variable (@c SensorField::VAR); the variable value is
 * included in LOG CSV reports.  For binary HK TM frames the variable has no
 * fixed @c TelemetryPayload slot and is omitted from the radio frame.
 *
 * @param[in]     line      Script line (e.g. @c "altitude: BARO.alt" or @c "ga: ground_alt").
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
        return storeVarFieldLocked(expr, key, fields, count, ctxName);
    }
    return storeAliasFieldLocked(aliasStr, fieldStr, key, fields, count, ctxName);
}

} // namespace ams
} // namespace ares
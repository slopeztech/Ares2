/**
 * @file  mission_script_engine_parser_actions.cpp
 * @brief AMS engine â€” action block parsing (on_error/exit/timeout handlers, set actions, PULSE.fire, on_error_transition).
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
    ares::util::copyZ(st.onErrorText, text, sizeof(st.onErrorText));

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
    ares::util::copyZ(st.onExitText, text, sizeof(st.onExitText));

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

    if (!detail::isOnlyTrailingWhitespace(msPtr + 3))
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
    ares::util::copyZ(st.onTimeoutText, text, sizeof(st.onTimeoutText));

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

    ares::util::copyZ(st.onTimeoutTransitionTarget, target,
            sizeof(st.onTimeoutTransitionTarget));
    st.onTimeoutTransitionResolved = false;

    LOG_I(TAG, "on_timeout transition: -> '%s'", target);
    return true;
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

    ares::util::copyZ(out.varName, varName, sizeof(out.varName));

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

    ares::util::copyZ(out.alias, aliasStr, sizeof(out.alias));
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

    ares::util::copyZ(out.alias, aliasStr, sizeof(out.alias));
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

    ares::util::copyZ(out.alias, aliasStr, sizeof(out.alias));
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

    ares::util::copyZ(out.alias, aliasStr, sizeof(out.alias));
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

    if (!detail::isOnlyTrailingWhitespace(msPtr + 2U))
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
    ares::util::copyZ(st.onErrorTransitionTarget, target,
            sizeof(st.onErrorTransitionTarget));
    st.onErrorTransitionResolved = false;

    LOG_I(TAG, "on_error recovery transition: -> '%s'", target);
    return true;
}

} // namespace ams
} // namespace ares
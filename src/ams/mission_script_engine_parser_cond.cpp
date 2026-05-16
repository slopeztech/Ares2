/**
 * @file  mission_script_engine_parser_cond.cpp
 * @brief AMS engine â€” condition and transition parsing (delta, falling/rising, TC debounce, transitions, fallback).
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
    ares::util::copyZ(out.alias, aliasStr, sizeof(out.alias));
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
    ares::util::copyZ(out.alias, aliasStr, sizeof(out.alias));
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
    ares::util::copyZ(tr.targetName, target, sizeof(tr.targetName));
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
    ares::util::copyZ(bodyBuf, body, sizeof(bodyBuf));

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
    ares::util::copyZ(st.fallbackTargetName, target, sizeof(st.fallbackTargetName));
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

    if (!detail::isOnlyTrailingWhitespace(msPtr + 2))
    {
        setErrorLocked("fallback transition: unexpected suffix after Nms");
        return false;
    }

    return true;
}

} // namespace ams
} // namespace ares
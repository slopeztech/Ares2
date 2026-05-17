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
    char rhsBuf[96] = {};

    // cppcheck-suppress [cert-err34-c]
    const int32_t n = static_cast<int32_t>(sscanf(line, "set %15s = %95[^\n]", varName, rhsBuf));
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
    if (parseExprSetActionLocked(rhsBuf, out))
    {
        return true;
    }
    // If the expression parser recognised the syntax but set a parse error
    // (e.g. unknown alias in a term), propagate the failure without falling
    // through to the SIMPLE parser.
    if (status_ == EngineStatus::ERROR)
    {
        return false;
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

// ── parseExprTermLocked ───────────────────────────────────────────────────────

/**
 * @brief Resolve a single expression token to an @c ExprOperand (AMS-4.8.8).
 *
 * Resolution order:
 *   1. Token contains @c '.'  → attempt SENSOR (ALIAS.field).
 *   2. Token parses as @c float → LITERAL.
 *   3. Token matches a declared variable → VARIABLE.
 *   4. None of the above → parse error.
 *
 * @param[in]  token  NUL-terminated whitespace-free token from expression RHS.
 * @param[out] out    ExprOperand struct to populate.
 * @return @c true on success; @c false with error set on failure.
 * @pre  Caller holds the engine mutex.
 */
bool MissionScriptEngine::parseExprTermLocked(const char* token, ExprOperand& out)
{
    ARES_ASSERT(token != nullptr);

    // SENSOR: token contains '.' — try ALIAS.field first.
    if (strchr(token, '.') != nullptr)
    {
        char aliasStr[16] = {};
        char fieldStr[20] = {};
        if (splitAliasDotField(token, aliasStr, sizeof(aliasStr), fieldStr, sizeof(fieldStr)))
        {
            const AliasEntry* ae = findAliasLocked(aliasStr);
            if (ae != nullptr)
            {
                SensorField sf = SensorField::ALT;
                if (parseSensorField(ae->kind, fieldStr, sf))
                {
                    out.kind  = ExprOperand::Kind::SENSOR;
                    ares::util::copyZ(out.name, aliasStr, sizeof(out.name));
                    out.field = sf;
                    return true;
                }
                char msg[80] = {};
                snprintf(msg, sizeof(msg),
                         "set expr: field '%s' not valid for alias '%s'",
                         fieldStr, aliasStr);
                setErrorLocked(msg);
                return false;
            }
        }
        // Dot present but not a recognised alias — fall through to float/var.
    }

    // LITERAL: token parses as a floating-point constant.
    float val = 0.0f;
    if (parseFloatValue(token, val))
    {
        out.kind    = ExprOperand::Kind::LITERAL;
        out.literal = val;
        return true;
    }

    // VARIABLE: token matches a declared variable name.
    const VarEntry* v = findVarLocked(token);
    if (v != nullptr)
    {
        out.kind = ExprOperand::Kind::VARIABLE;
        ares::util::copyZ(out.name, token, sizeof(out.name));
        return true;
    }

    char msg[72] = {};
    snprintf(msg, sizeof(msg),
             "set expr: '%s' is not a sensor, variable, or literal", token);
    setErrorLocked(msg);
    return false;
}

// ── stripAndTokenizeExpr (file-local helper) ────────────────────────────────

/**
 * @brief Strip parentheses from @p rhs and split on whitespace into tokens.
 *
 * @param[in]  rhs      NUL-terminated expression RHS string.
 * @param[out] tokens   2-D token buffer; each slot holds up to 31 characters + NUL.
 * @param[in]  maxToks  Capacity of the @p tokens array.
 * @return Actual token count, or @p maxToks + 1 as an overflow sentinel when
 *         the input contains more than @p maxToks tokens.
 */
static uint8_t stripAndTokenizeExpr(const char* rhs,
                                    char        tokens[][32],
                                    uint8_t     maxToks)
{
    // Strip parentheses into a working buffer.
    char     work[96] = {};
    uint32_t wIdx     = 0U;
    for (const char* p = rhs; *p != '\0' && wIdx < sizeof(work) - 1U; p++)
    {
        if (*p != '(' && *p != ')') { work[wIdx++] = *p; }
    }
    work[wIdx] = '\0';

    // Tokenize by whitespace.
    uint8_t     tokCount = 0U;
    const char* p        = work;
    while (*p != '\0' && tokCount < maxToks)
    {
        while (*p == ' ' || *p == '\t') { p++; }
        if (*p == '\0') { break; }
        uint32_t tLen = 0U;
        while (*p != '\0' && *p != ' ' && *p != '\t' && tLen < 31U)
        {
            tokens[tokCount][tLen++] = *p++;
        }
        tokens[tokCount][tLen] = '\0';
        // Token-length overflow: remainder of the token is not whitespace.
        // Skip to the next word boundary and return the overflow sentinel so
        // the caller rejects the expression rather than mis-tokenizing it.
        if (*p != '\0' && *p != ' ' && *p != '\t')
        {
            while (*p != '\0' && *p != ' ' && *p != '\t') { p++; }
            return static_cast<uint8_t>(maxToks + 1U);
        }
        tokCount++;
    }
    // Overflow sentinel: non-whitespace remains after maxToks tokens.
    while (*p == ' ' || *p == '\t') { p++; }
    return (*p != '\0') ? static_cast<uint8_t>(maxToks + 1U) : tokCount;
}

// ── buildExprOpsLocked ──────────────────────────────────────────────────────

/**
 * @brief Map operator tokens to ExprOp enum values; guard against literal ÷0.
 *
 * Returns @c false (no error set) when any operator token is not a recognised
 * single-character arithmetic operator — the caller treats the input as
 * "not an expression".  Returns @c false with an error set when a literal zero
 * divisor is detected.  Returns @c true on success.
 *
 * @param[in]  tokens     Full token array (operators are at odd indices).
 * @param[in]  termCount  Number of TERM operands; operators = termCount − 1.
 * @param[in]  terms      Parsed operand array (used to check divisor literals).
 * @param[out] ops        Caller-allocated operator array (termCount − 1 slots).
 * @pre  Caller holds the engine mutex.
 */
bool MissionScriptEngine::buildExprOpsLocked(const char        tokens[][32],
                                             uint8_t           termCount,
                                             const ExprOperand terms[],
                                             ExprOp            ops[])
{
    for (uint8_t i = 0U; i < termCount - 1U; i++)
    {
        const char* opTok = tokens[static_cast<size_t>(i) * 2U + 1U];
        if (opTok[1] != '\0') { return false; } // multi-char → not an expression
        switch (opTok[0])
        {
        case '+': ops[i] = ExprOp::ADD; break;
        case '-': ops[i] = ExprOp::SUB; break;
        case '*': ops[i] = ExprOp::MUL; break;
        case '/':
            ops[i] = ExprOp::DIV;
            if (terms[i + 1U].kind == ExprOperand::Kind::LITERAL &&
                terms[i + 1U].literal == 0.0f)
            {
                setErrorLocked("set expr: division by literal zero");
                return false;
            }
            break;
        default:
            return false; // Unrecognised operator → not an expression.
        }
    }
    return true;
}

// ── parseExprSetActionLocked ──────────────────────────────────────────────────

/**
 * @brief Attempt to parse the RHS of a @c set statement as an arithmetic
 *        expression (AMS-4.8.8).
 *
 * Accepted syntax (after stripping parentheses):
 * @code
 *   TERM op TERM [op TERM]
 * @endcode
 * where TERM is @c ALIAS.field, a declared variable name, or a float literal,
 * and @c op is one of @c + @c - @c * @c /.  Evaluation is strictly
 * left-to-right: <tt>((T0 op0 T1) op1 T2)</tt>.
 *
 * @param[in]  rhsBuf  Trimmed RHS string (everything after @c "= ").
 * @param[out] out     SetAction struct to populate on success.
 * @return @c true if the expression was recognised and parsed successfully.
 *         @c false without setting an error if the RHS does not match the
 *         expression pattern (caller may try other forms).
 *         @c false with an error set if the pattern matches but a term is
 *         invalid (caller must not fall through).
 * @pre  Caller holds the engine mutex.
 */
bool MissionScriptEngine::parseExprSetActionLocked(const char* rhsBuf,
                                                   SetAction&  out)
{
    ARES_ASSERT(rhsBuf != nullptr);

    // Strip parentheses and tokenize: T OP T [OP T] → up to 5 whitespace-delimited tokens.
    static constexpr uint8_t kMaxToks = 5U;
    char          tokens[kMaxToks][32] = {};
    const uint8_t tokCount = stripAndTokenizeExpr(rhsBuf, tokens, kMaxToks);

    // Require exactly 3 or 5 tokens (odd count in [3,5]).
    if (tokCount < 3U || tokCount > 5U || (tokCount % 2U) == 0U)
    {
        return false; // Not an expression — let caller try SIMPLE.
    }

    // Parse each TERM operand (tokens at even positions 0, 2, 4).
    const uint8_t termCount = static_cast<uint8_t>((tokCount + 1U) / 2U);
    ExprOperand terms[SetAction::kMaxExprTerms] = {};
    for (uint8_t i = 0U; i < termCount; i++)
    {
        if (!parseExprTermLocked(tokens[static_cast<size_t>(i) * 2U], terms[i]))
        {
            return false; // Error already set by parseExprTermLocked.
        }
    }

    // Map operator tokens → ExprOp[]; guard literal ÷0.
    ExprOp ops[SetAction::kMaxExprTerms - 1U] = {};
    if (!buildExprOpsLocked(tokens, termCount, terms, ops))
    {
        return false;
    }

    // Commit to out.
    out.kind          = SetActionKind::EXPR;
    out.exprTermCount = termCount;
    for (uint8_t i = 0U; i < termCount;         i++) { out.exprTerms[i] = terms[i]; }
    for (uint8_t i = 0U; i < termCount - 1U;    i++) { out.exprOps[i]   = ops[i];   }
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
 *   set VARNAME = TERM op TERM [op TERM]             -- arithmetic expression (AMS-4.8.8)
 * @endcode
 *
 * For AMS-4.8.8 each TERM is @c ALIAS.field, a declared variable name, or a
 * float literal; @c op is @c + @c - @c * or @c /.  Evaluation is strictly
 * left-to-right.  Parentheses are stripped before parsing and have no effect
 * on evaluation order.
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

// ── resolveChannelOrAliasLocked ───────────────────────────────────────────────

/**
 * @brief Resolve a channel token (letter or label) to a channel index.
 *
 * Checks the bare channel letters A/B/C/D first, then searches the declared
 * pulse channel labels in @c program_.pulseDecls for an alias match.
 *
 * @param[in] token  NUL-terminated token string.
 * @return Channel index (0–3), or @c 0xFF if not recognised.
 * @pre  Caller holds the engine mutex.
 */
uint8_t MissionScriptEngine::resolveChannelOrAliasLocked(const char* token) const
{
    ARES_ASSERT(token != nullptr);

    // Check bare channel letters (single character).
    if (token[0] != '\0' && token[1] == '\0')
    {
        if (token[0] == 'A') { return PulseChannel::CH_A; }
        if (token[0] == 'B') { return PulseChannel::CH_B; }
        if (token[0] == 'C') { return PulseChannel::CH_C; }
        if (token[0] == 'D') { return PulseChannel::CH_D; }
    }

    // Check declared labels.
    for (uint8_t i = 0U; i < PulseChannel::COUNT; i++)
    {
        if (program_.pulseDecls[i].declared
            && strcmp(program_.pulseDecls[i].label, token) == 0)
        {
            return i;
        }
    }

    return 0xFFU;  // not found
}

// ── parsePulseFireLineLocked ──────────────────────────────────────────────────

/**
 * @brief Parse a @c PULSE.fire directive inside an @c on_enter: block (AMS-4.17).
 *
 * Syntax:
 * @code
 *   PULSE.fire A             -- fire channel A at default duration
 *   PULSE.fire B             -- fire channel B at default duration
 *   PULSE.fire C             -- fire channel C at default duration
 *   PULSE.fire D             -- fire channel D at default duration
 *   PULSE.fire DROGUE 500ms  -- fire channel by declared label with 500 ms override
 * @endcode
 *
 * The channel token may be a bare letter (A/B/C/D) or any label assigned
 * by a prior @c pulse.channel X as LABEL directive (AMS-4.18).
 * The referenced channel @b must have been declared with @c pulse.channel;
 * an undeclared channel is a parse error.
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

    // Extract the channel token (runs until end-of-line or a space before
    // the optional duration suffix).
    char token[ares::AMS_PULSE_LABEL_LEN] = {};
    uint8_t tokLen = 0U;
    const char* afterToken = rest;
    while (*afterToken != '\0' && *afterToken != ' ')
    {
        if (tokLen >= ares::AMS_PULSE_LABEL_LEN - 1U)
        {
            setErrorLocked("PULSE.fire: channel token too long");
            return false;
        }
        token[tokLen++] = *afterToken++;
    }
    token[tokLen] = '\0';

    if (tokLen == 0U)
    {
        setErrorLocked("PULSE.fire: missing channel");
        return false;
    }

    // Resolve token -> channel index (letter or alias).
    const uint8_t channel = resolveChannelOrAliasLocked(token);
    if (channel == 0xFFU)
    {
        setErrorLocked("PULSE.fire: unknown channel or undeclared alias");
        return false;
    }

    // Enforce declaration requirement (AMS-4.18).
    if (!program_.pulseDecls[channel].declared)
    {
        setErrorLocked("PULSE.fire: channel used but not declared with pulse.channel");
        return false;
    }

    // Mark channel as used so finalizeScriptLocked can warn on declared-but-unused (AMS-4.18).
    pulseChannelUsed_[channel] = true;

    // Parse optional " Nms" duration suffix.
    uint32_t durationMs = 0U;
    if (!parsePulseDurationSuffixLocked(afterToken, durationMs))
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
 *                      token in the script line.
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

// ── parsePulseArmLineLocked ───────────────────────────────────────────────────

/**
 * @brief Parse a @c PULSE.arm directive inside an @c on_enter: block (AMS-4.19.1).
 *
 * Syntax: @c "PULSE.arm X"  where @p X is a channel letter (A-D) or declared alias.
 *
 * @c PULSE.arm marks a channel as armed so that a later @c PULSE.fire in another
 * state (or the same state) is permitted.  If @c pulse.arm_timeout is declared,
 * the arm flag expires automatically if @c PULSE.fire is not called in time.
 *
 * Calling @c PULSE.arm for a channel also sets @c pulseNeedsArm[channel] = true at
 * parse time, which activates the two-phase fire check (AMS-4.19.1) in
 * @c checkPulseSafetyLocked() for all subsequent @c PULSE.fire calls on that channel.
 *
 * @param[in]  line  Script line starting with @c "PULSE.arm ".
 * @param[out] st    State definition to append the arm action to.
 * @return @c true if parsed and stored.
 * @pre  Caller holds the engine mutex.
 */
bool MissionScriptEngine::parsePulseArmLineLocked(const char* line, StateDef& st)
{
    ARES_ASSERT(line != nullptr);

    if (st.pulseArmActionCount >= ares::AMS_MAX_PULSE_ACTIONS)
    {
        setErrorLocked("too many PULSE.arm actions in on_enter block");
        return false;
    }

    // Skip "PULSE.arm " prefix (10 characters).
    static constexpr uint8_t kPrefixLen = 10U;
    const char* rest = line + kPrefixLen;

    // Extract channel token.
    char token[ares::AMS_PULSE_LABEL_LEN] = {};
    uint8_t tokLen = 0U;
    while (rest[tokLen] != '\0' && rest[tokLen] != ' ')
    {
        if (tokLen >= ares::AMS_PULSE_LABEL_LEN - 1U)
        {
            setErrorLocked("PULSE.arm: channel token too long");
            return false;
        }
        token[tokLen] = rest[tokLen];
        tokLen++;
    }
    token[tokLen] = '\0';

    if (tokLen == 0U) { setErrorLocked("PULSE.arm: missing channel"); return false; }

    const uint8_t ch = resolveChannelOrAliasLocked(token);
    if (ch == 0xFFU) { setErrorLocked("PULSE.arm: unknown channel or undeclared alias"); return false; }
    if (!program_.pulseDecls[ch].declared) { setErrorLocked("PULSE.arm: channel not declared with pulse.channel"); return false; }

    // Mark at parse time: any PULSE.fire on this channel requires a prior arm.
    program_.pulseNeedsArm[ch] = true;

    st.pulseArmActions[st.pulseArmActionCount].channel = ch;
    st.pulseArmActionCount++;
    LOG_I(TAG, "PULSE.arm: ch=%u will require prior arm before PULSE.fire", static_cast<uint32_t>(ch));
    return true;
}

} // namespace ams
} // namespace ares
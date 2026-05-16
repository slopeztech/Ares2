/**
 * @file  mission_script_engine_parser_finalize.cpp
 * @brief AMS engine â€” post-parse finalization (shadow detection, transition resolve, tasks, assertions, BFS/DFS).
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
// ── emitShadowWarningLocked ───────────────────────────────────────────────────

void MissionScriptEngine::emitShadowWarningLocked(const StateDef& st,
                                                   uint8_t          i,
                                                   uint8_t          j,
                                                   const CondExpr&  ci,
                                                   const CondExpr&  cj,
                                                   uint64_t         nowMs)
{
    char thrI[12] = {};
    char thrJ[12] = {};
    (void)detail::formatScaledFloat(ci.threshold, 3U, thrI, sizeof(thrI));
    (void)detail::formatScaledFloat(cj.threshold, 3U, thrJ, sizeof(thrJ));

    // Use a local buffer larger than AMS_MAX_ERROR_TEXT so the
    // compiler's conservative estimate for %u (up to 10 digits)
    // does not trigger -Wformat-truncation.  This msg is only
    // forwarded to LOG_W / sendEventLocked, not stored in lastError_.
    char msg[160U] = {};
    if (ci.kind == CondKind::TIME_GT)
    {
        (void)snprintf(msg, sizeof(msg),
                       "tr[%u] in '%s' unreachable: "
                       "TIME.elapsed > %s (tr[%u] thr %s fires first)",
                       (unsigned)j, st.name, thrJ, (unsigned)i, thrI);
    }
    else
    {
        const char* opStr = (ci.kind == CondKind::SENSOR_LT) ? "<" : ">";
        (void)snprintf(msg, sizeof(msg),
                       "tr[%u] in '%s' unreachable: "
                       "%s.%s %s %s (tr[%u] thr %s fires first)",
                       (unsigned)j, st.name,
                       ci.alias, sensorFieldNameForLog(ci.field),
                       opStr, thrJ,
                       (unsigned)i, thrI);
    }
    msg[sizeof(msg) - 1U] = '\0';

    LOG_W(TAG, "%s", msg);
    sendEventLocked(EventVerb::INFO,
                    ares::proto::EventId::FPL_VIOLATION,
                    msg, nowMs);
}

// ── checkTransitionPairLocked ─────────────────────────────────────────────────

void MissionScriptEngine::checkTransitionPairLocked(const StateDef& st,
                                                     uint8_t         i,
                                                     const CondExpr& ci,
                                                     uint8_t         j,
                                                     uint64_t        nowMs)
{
    const Transition& tj = st.transitions[j];
    if (tj.condCount != 1U) { return; }
    const CondExpr& cj = tj.conds[0];
    if (cj.useVar)          { return; }
    if (cj.kind != ci.kind) { return; }

    // For sensor kinds, alias and field must also match.
    if (ci.kind != CondKind::TIME_GT)
    {
        if (strncmp(ci.alias, cj.alias, sizeof(ci.alias)) != 0) { return; }
        if (ci.field != cj.field)                                { return; }
    }

    // Determine if j is completely shadowed by i.
    bool shadowed = false;
    if (ci.kind == CondKind::SENSOR_GT || ci.kind == CondKind::TIME_GT)
    {
        // val > Ti fires first; val > Tj never reached when Ti <= Tj.
        shadowed = (ci.threshold <= cj.threshold);
    }
    else  // SENSOR_LT
    {
        // val < Ti fires first; val < Tj never reached when Ti >= Tj.
        shadowed = (ci.threshold >= cj.threshold);
    }

    if (!shadowed) { return; }

    emitShadowWarningLocked(st, i, j, ci, cj, nowMs);
}

// ── warnShadowedTransitionsLocked ─────────────────────────────────────────────

/**
 * @brief Emit LOG_W and EVENT.info for any obviously shadowed transitions (AMS-5.1).
 *
 * For each state, compares every pair of single-condition transitions (i, j)
 * (i declared before j, both with condCount == 1, no variable RHS).
 * If the condition of j is completely dominated by i — same LHS, same
 * operator, threshold of i is at least as permissive — transition j can
 * never fire and a diagnostic is emitted.
 *
 * Shadowing rules:
 *   - SENSOR_GT / TIME_GT:  i dominates j when threshold_i <= threshold_j.
 *   - SENSOR_LT:            i dominates j when threshold_i >= threshold_j.
 *
 * This is a lint-only pass: no error is set and the engine continues loading.
 *
 * @param[in] nowMs  Timestamp for EVENT frames (0 = load time, milliseconds).
 * @pre  Caller holds the engine mutex.  resolveTransitionsLocked() has run.
 */
void MissionScriptEngine::warnShadowedTransitionsLocked(uint64_t nowMs)
{
    for (uint8_t si = 0U; si < program_.stateCount; si++)  // PO10-2: bounded
    {
        const StateDef& st = program_.states[si];

        for (uint8_t i = 0U; i < st.transitionCount; i++)  // PO10-2: bounded
        {
            const Transition& ti = st.transitions[i];
            if (ti.condCount != 1U) { continue; }  // skip multi-condition
            const CondExpr& ci = ti.conds[0];
            if (ci.useVar) { continue; }            // variable RHS: skip

            if (ci.kind != CondKind::SENSOR_LT &&
                ci.kind != CondKind::SENSOR_GT &&
                ci.kind != CondKind::TIME_GT)
            {
                continue;  // TC / delta kinds: not statically comparable
            }

            for (uint8_t j = i + 1U; j < st.transitionCount; j++)  // PO10-2: bounded
            {
                checkTransitionPairLocked(st, i, ci, j, nowMs);
            }
        }
    }
}

// ── resolveTargetNameLocked ───────────────────────────────────────────────────

bool MissionScriptEngine::resolveTargetNameLocked(const char* name,
                                                   const char* kindStr,
                                                   uint8_t&    outIdx)
{
    const uint8_t idx = findStateByNameLocked(name);
    if (idx >= program_.stateCount)
    {
        char sug[40] = {};
        suggestStateNameLocked(name, sug, sizeof(sug));
        char msg[80] = {};
        snprintf(msg, sizeof(msg), "unknown %s target: '%s'%s", kindStr, name, sug);
        setErrorLocked(msg);
        return false;
    }
    outIdx = idx;
    return true;
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
            if (!resolveTargetNameLocked(st.transitions[ti].targetName,
                                          "transition",
                                          st.transitions[ti].targetIndex))
            {
                return false;
            }
            st.transitions[ti].targetResolved = true;
        }

        // AMS-4.9.2: fallback transition target.
        if (st.hasFallback)
        {
            if (!resolveTargetNameLocked(st.fallbackTargetName, "fallback",
                                          st.fallbackTargetIdx))
            {
                return false;
            }
            st.fallbackTargetResolved = true;
        }

        // AMS-4.10.2: on_error recovery transition target.
        if (st.hasOnErrorTransition)
        {
            if (!resolveTargetNameLocked(st.onErrorTransitionTarget, "on_error",
                                          st.onErrorTransitionIdx))
            {
                return false;
            }
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
            if (!resolveTargetNameLocked(st.onTimeoutTransitionTarget, "on_timeout",
                                          st.onTimeoutTransitionIdx))
            {
                return false;
            }
            st.onTimeoutTransitionResolved = true;
        }
    }

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
    ares::util::copyZ(listBuf, stateList, sizeof(listBuf));

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
        ares::util::copyZ(td.name, name, sizeof(td.name));
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
    ares::util::copyZ(condBuf, condStart, sizeof(condBuf));

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
        ares::util::copyZ(rule.eventText, text, sizeof(rule.eventText));
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
 *   no_silent_terminals           -- every state has ≥1 outgoing edge or periodic action.
 * @endcode
 *
 * Assertions are evaluated after all states are resolved; parse errors are
 * recorded and prevent the script from activating.
 *
 * @param[in] line  Trimmed directive line.
 * @return @c true if the directive was stored.
 * @pre  Caller holds the engine mutex.
 */
bool MissionScriptEngine::parseAssertLineLocked(const char* line)  // NOLINT(readability-function-size)
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
        ares::util::copyZ(ad.targetName, name, sizeof(ad.targetName));
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
        uint32_t limit = 0U;
        // cppcheck-suppress [cert-err34-c]
        const int32_t nr = static_cast<int32_t>(
            sscanf(line, "max_transition_depth < %u", &limit));  // NOLINT(bugprone-unchecked-string-to-number-conversion)
        if (nr != 1 || limit == 0U || limit > 255U)
        {
            setErrorLocked("assert: invalid 'max_transition_depth' "
                           "(expected: max_transition_depth < N, 1<=N<=255)");
            return false;
        }
        ad.kind       = AssertKind::MAX_DEPTH;
        ad.numericArg = static_cast<uint8_t>(limit);
        program_.assertCount++;
        return true;
    }

    // "no_silent_terminals"
    if (strcmp(line, "no_silent_terminals") == 0)
    {
        ad.kind = AssertKind::NO_SILENT_TERMINALS;
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
                char sug[40] = {};
                suggestStateNameLocked(td.activeStateNames[j], sug, sizeof(sug));
                char msg[96] = {};
                snprintf(msg, sizeof(msg),
                         "task '%s': unknown state '%s' in 'when in' filter%s",
                         td.name, td.activeStateNames[j], sug);
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

    while (dfsTop > 0U && !hasCycle)
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
            char sug[40] = {};
            suggestStateNameLocked(ad.targetName, sug, sizeof(sug));
            char msg[96] = {};
            snprintf(msg, sizeof(msg), "assert reachable: unknown state '%s'%s",
                     ad.targetName, sug);
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

    case AssertKind::NO_SILENT_TERMINALS:
    {
        for (uint8_t s = 0U; s < program_.stateCount; s++)
        {
            const StateDef& sd = program_.states[s];
            const bool hasExit = (sd.transitionCount > 0U) ||
                                  sd.hasFallback ||
                                  sd.hasOnErrorTransition ||
                                 (sd.hkSlotCount > 0U) ||
                                 (sd.logSlotCount > 0U);
            if (!hasExit)
            {
                static char msg[96] = {};
                snprintf(msg, sizeof(msg),
                         "assert no_silent_terminals: '%s' has no exit"
                         " (transition, fallback, on_error, hk, or log slot)",
                         sd.name);
                setErrorLocked(msg);
                return false;
            }
        }
        LOG_I(TAG, "assert no_silent_terminals: PASS (all %u states have exits)",
              static_cast<unsigned>(program_.stateCount));
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
/**
 * @file  mission_script_engine_actions.cpp
 * @brief AMS engine â€” variable resolution, set actions, tasks, and pulse.
 *
 * Contains findVarLocked, resolveVarThresholdLocked, executeSetActionsLocked,
 * calibration step, delta/min-max helpers, runTasksLocked, and pulse dispatch.
 *
 * Thread safety: *Locked helpers require the engine mutex held by caller.
 */

#include "ams/mission_script_engine.h"
#include "ams/mission_script_engine_helpers.h"

#include "api/api_common.h"
#include "ares_assert.h"
#include "ares_util.h"
#include "debug/ares_log.h"
#include "hal/millis64.h"

#include <Arduino.h>
#include <algorithm>
#include <cinttypes>
#include <iterator>
#include <cstdio>
#include <cstring>

namespace ares
{
namespace ams
{
static constexpr const char* TAG = "AMS";

/**
 * @brief Find a global variable by name (mutable version).
 * @param[in] name  Variable name to look up.
 * @return Pointer to the @c VarEntry, or @c nullptr if not found.
 * @pre  mutex_ is held by the caller.
 */
MissionScriptEngine::VarEntry* MissionScriptEngine::findVarLocked(const char* name)
{
    for (uint8_t i = 0; i < program_.varCount; i++)
    {
        if (strcmp(program_.vars[i].name, name) == 0)
        {
            return &program_.vars[i];
        }
    }
    return nullptr;
}

/**
 * @brief Find a global variable by name (const version).
 * @param[in] name  Variable name to look up.
 * @return Const pointer to the @c VarEntry, or @c nullptr if not found.
 * @pre  mutex_ is held by the caller.
 */
const MissionScriptEngine::VarEntry* MissionScriptEngine::findVarLocked(
    const char* name) const
{
    for (uint8_t i = 0; i < program_.varCount; i++)
    {
        if (strcmp(program_.vars[i].name, name) == 0)
        {
            return &program_.vars[i];
        }
    }
    return nullptr;
}

/**
 * @brief Resolve the effective RHS threshold for a condition expression.
 *
 * For conditions with @c useVar == true, looks up the named variable and
 * applies the optional additive offset.  Returns @c false if the variable
 * is not found or has not yet been set (NaN guard — AMS-4.8).
 * For literal-threshold conditions, copies @c cond.threshold directly.
 *
 * @param[in]  cond          Condition expression to resolve.
 * @param[out] outThreshold  Resolved numeric threshold.
 * @return @c true if the threshold was resolved; @c false on missing/invalid var.
 * @pre  mutex_ is held by the caller.
 */
bool MissionScriptEngine::resolveVarThresholdLocked(const CondExpr& cond,
                                                    float& outThreshold) const
{
    if (!cond.useVar)
    {
        outThreshold = cond.threshold;
        return true;
    }

    const VarEntry* v = findVarLocked(cond.varName);
    if (v == nullptr)
    {
        LOG_W(TAG, "condition references undefined variable '%s'", cond.varName);
        return false;
    }
    if (!v->valid)
    {
        // Variable declared but set action has not fired yet — treat as false.
        return false;
    }

    outThreshold = v->value + cond.varOffset;
    return true;
}

/**
 * @brief Execute all @c set actions declared in a state's @c on_enter block.
 *
 * Each action reads the configured sensor (or samples it @p calibSamples times
 * and averages the valid readings) and stores the result in the named global
 * variable.  If all sensor reads fail, the variable is left unchanged and an
 * EVENT.warning is queued (NaN guard — AMS-4.8).
 *
 * @param[in] st     Current state whose set actions are to be executed.
 * @param[in] nowMs  Current timestamp in milliseconds.
 * @pre  mutex_ is held by the caller.
 */
void MissionScriptEngine::executeSetActionsLocked(StateDef& st,
                                                  uint64_t nowMs)
{
    for (uint8_t i = 0; i < st.setActionCount; i++)
    {
        executeOneSetActionLocked(st.setActions[i], nowMs);
    }
}

/**
 * @brief Advance each in-progress CALIBRATE set action in @p st by one sample.
 *
 * Called at the start of every tick() before transition evaluation.  For each
 * set action whose @c calibInProgress flag is true, delegates one sensor read
 * to @c executeOneSetActionLocked, which internally calls
 * @c executeCalibrateSetActionLocked.  When all @c calibSamples have been
 * collected the accumulator is finalised and the target variable is updated.
 *
 * This distributes the N sensor reads of a CALIBRATE(ALIAS.field, N) form
 * across N ticks, bounding the per-tick mutex-hold time to a single I²C read
 * (~25 ms) regardless of the sample count (AMS-4.8.2).
 *
 * @param[in,out] st     Current state definition (set actions are mutable).
 * @param[in]     nowMs  Current timestamp for event framing.
 * @pre  mutex_ is held by the caller.
 */
void MissionScriptEngine::stepPendingCalibrationsLocked(StateDef& st, uint64_t nowMs)
{
    for (uint8_t i = 0U; i < st.setActionCount; i++)
    {
        SetAction& act = st.setActions[i];
        if (act.kind == SetActionKind::CALIBRATE && act.calibInProgress)
        {
            executeOneSetActionLocked(act, nowMs);
        }
    }
}

// ── executeOneSetActionLocked ─────────────────────────────────────────────────

/**
 * @brief Execute a single @c SetAction and update the target variable.
 *
 * Shared by the on_enter state path (via @c executeSetActionsLocked) and
 * the background task path (via @c runTasksLocked).
 *
 * @param[in,out] act    Set-action descriptor (deltaBaseline may be mutated).
 * @param[in]     nowMs  Current timestamp for event framing.
 * @pre  mutex_ is held by the caller.
 */
void MissionScriptEngine::executeCalibrateSetActionLocked(SetAction& act,
                                                          float&     result,
                                                          bool&      gotReading,
                                                          uint64_t   /*nowMs*/)
{
    // AMS-4.8.2 — Async CALIBRATE: one sensor read per tick.
    //
    // The first call (calibInProgress == false) initialises the accumulator;
    // each subsequent call (calibInProgress == true) advances by one sample
    // until calibSamples attempts have been made.  The variable is only
    // written once all samples have been collected (gotReading = true).
    // This keeps the per-tick mutex-hold time to a single I²C read (~25 ms)
    // regardless of the sample count N, eliminating the previous N×25 ms
    // blocking behaviour that violated AMS-8.3.

    if (!act.calibInProgress)
    {
        // Fresh start: initialise accumulator.
        act.calibSum       = 0.0f;
        act.calibValidN    = 0U;
        act.calibCollected = 0U;
        act.calibInProgress = true;
        LOG_I(TAG, "CALIBRATE '%s': starting async collection (%u samples)",
              act.varName, static_cast<unsigned>(act.calibSamples));
    }

    // Collect exactly one sample this tick.
    float sample = 0.0f;
    if (readSensorFloatLocked(act.alias, act.field, sample))
    {
        act.calibSum    += sample;
        act.calibValidN++;
    }
    act.calibCollected++;

    // Finalise when all requested sample attempts have been made.
    if (act.calibCollected >= act.calibSamples)
    {
        act.calibInProgress = false;

        if (act.calibValidN > 0U)
        {
            result     = act.calibSum / static_cast<float>(act.calibValidN);
            gotReading = true;
            LOG_I(TAG, "CALIBRATE '%s': complete — avg=%.3f (%u/%u valid, %u ticks)",
                  act.varName,
                  static_cast<double>(result),
                  static_cast<unsigned>(act.calibValidN),
                  static_cast<unsigned>(act.calibSamples),
                  static_cast<unsigned>(act.calibSamples));
        }
        // gotReading == false here: all reads failed; caller handles the failure.
    }
    // While calibCollected < calibSamples: gotReading stays false and
    // calibInProgress stays true — caller skips the variable update.
}

void MissionScriptEngine::executeDeltaSetActionLocked(SetAction& act,
                                                      float&     result,
                                                      bool&      gotReading)
{
    float curVal = 0.0f;
    if (readSensorFloatLocked(act.alias, act.field, curVal))
    {
        result            = act.deltaValid ? (curVal - act.deltaBaseline) : 0.0f;
        act.deltaBaseline = curVal;
        act.deltaValid    = true;
        gotReading        = true;
        LOG_I(TAG, "set delta '%s': delta=%.3f (cur=%.3f prev=%.3f)",
              act.varName,
              static_cast<double>(result),
              static_cast<double>(curVal),
              static_cast<double>(act.deltaBaseline - result));
    }
}

void MissionScriptEngine::executeMinMaxSetActionLocked(SetAction&    act,
                                                       const VarEntry* v,
                                                       float&         result,
                                                       bool&          gotReading)
{
    float curVal = 0.0f;
    if (!readSensorFloatLocked(act.alias, act.field, curVal)) { return; }

    const bool isMax = (act.kind == SetActionKind::MAX_VAR);
    if (isMax)
    {
        result = (v->valid && curVal < v->value) ? v->value : curVal;
        LOG_I(TAG, "set max '%s': %.3f (sensor=%.3f)",
              act.varName, static_cast<double>(result), static_cast<double>(curVal));
    }
    else
    {
        result = (v->valid && curVal > v->value) ? v->value : curVal;
        LOG_I(TAG, "set min '%s': %.3f (sensor=%.3f)",
              act.varName, static_cast<double>(result), static_cast<double>(curVal));
    }
    gotReading = true;
}

void MissionScriptEngine::executeOneSetActionLocked(SetAction& act, uint64_t nowMs)
{
    VarEntry* v = findVarLocked(act.varName);
    if (v == nullptr)
    {
        // Parser validates variable existence; this path is a safety guard only.
        LOG_W(TAG, "set: variable '%s' not found at runtime", act.varName);
        return;
    }

    float result     = 0.0f;
    bool  gotReading = false;

    switch (act.kind)
    {
    case SetActionKind::CALIBRATE:
        executeCalibrateSetActionLocked(act, result, gotReading, nowMs);
        break;

    case SetActionKind::DELTA:
        executeDeltaSetActionLocked(act, result, gotReading);
        break;

    case SetActionKind::MAX_VAR:
    case SetActionKind::MIN_VAR:
        executeMinMaxSetActionLocked(act, v, result, gotReading);
        break;

    case SetActionKind::SIMPLE:
    default:
        gotReading = readSensorFloatLocked(act.alias, act.field, result);
        if (gotReading)
        {
            LOG_I(TAG, "set '%s' = %.3f (from %s)",
                  act.varName, static_cast<double>(result), act.alias);
        }
        break;
    }

    if (gotReading)
    {
        v->value = result;
        v->valid = true;
        checkpointDirty_ = true; // var updated — checkpoint on next 1 s window
    }
    else if (act.kind == SetActionKind::CALIBRATE && act.calibInProgress)
    {
        // AMS-4.8.2: async calibration in progress — variable will be updated
        // on a future tick once all samples are gathered.  Not a failure.
    }
    else
    {
        char warnMsg[64] = {};
        snprintf(warnMsg, sizeof(warnMsg),
                 "set '%s': sensor read failed, value not updated", act.varName);
        LOG_W(TAG, "%s", warnMsg);
        sendEventLocked(EventVerb::WARN,
                        ares::proto::EventId::SENSOR_FAILURE,
                        warnMsg, nowMs);
    }
}

// ── runTasksLocked ────────────────────────────────────────────────────────────

/**
 * @brief Evaluate all background task blocks for the current tick (AMS-11).
 *
 * For each task whose period has elapsed and whose state filter (if any)
 * matches the current state, evaluates each @c if-rule in declaration order.
 * When a rule's condition is true the associated EVENT and/or set action fires.
 *
 * Called at the end of @c tick() after @c executeDueActionsLocked() so state
 * transitions are committed before tasks read variable values.
 *
 * @param[in] nowMs  Current system time in milliseconds.
 * @pre  mutex_ is held.  running_ is true.  currentState_ is valid.
 */
bool MissionScriptEngine::evaluateTaskRuleCondLocked(const TaskRule& rule,
                                                     uint64_t        nowMs,
                                                     bool&           condResult) const
{
    condResult = false;
    switch (rule.cond.kind)
    {
    case CondKind::SENSOR_LT:
    case CondKind::SENSOR_GT:
    {
        float thr = 0.0f;
        if (!resolveVarThresholdLocked(rule.cond, thr)) { return true; }
        float val = 0.0f;
        if (readSensorFloatLocked(rule.cond.alias, rule.cond.field, val))
        {
            condResult = (rule.cond.kind == CondKind::SENSOR_LT)
                         ? (val < thr) : (val > thr);
        }
        return true;
    }
    case CondKind::TIME_GT:
        condResult = (nowMs - stateEnterMs_) >
                     static_cast<uint64_t>(rule.cond.threshold);
        return true;
    default:
        // Delta and TC conditions are not supported in task if-rules.
        return true;
    }
}

void MissionScriptEngine::runTasksLocked(uint64_t nowMs)
{
    for (uint8_t i = 0U; i < program_.taskCount; i++)
    {
        TaskDef& td = program_.tasks[i];

        if (td.everyMs == 0U) { continue; }

        // AMS-11: apply optional state filter.
        if (td.hasStateFilter && td.stateIndicesResolved)
        {
            bool inActiveState = false;
            for (uint8_t j = 0U; j < td.activeStateCount; j++)
            {
                if (td.activeStateIndices[j] == currentState_)
                {
                    inActiveState = true;
                    break;
                }
            }
            if (!inActiveState)
            {
                // Reset the timer so the task fires promptly when we re-enter
                // an active state, rather than firing stale accumulated time.
                taskLastTickMs_[i] = nowMs;
                continue;
            }
        }

        if ((nowMs - taskLastTickMs_[i]) < td.everyMs) { continue; }
        taskLastTickMs_[i] = nowMs;

        // Evaluate each conditional rule in order.
        for (uint8_t r = 0U; r < td.ruleCount; r++)
        {
            TaskRule& rule = td.rules[r];
            bool condResult = false;
            (void)evaluateTaskRuleCondLocked(rule, nowMs, condResult);
            if (!condResult) { continue; }

            if (rule.hasEvent)
            {
                sendEventLocked(rule.eventVerb,
                                inferEventId(rule.eventVerb),
                                rule.eventText, nowMs);
            }
            if (rule.hasSet)
            {
                executeOneSetActionLocked(rule.setAction, nowMs);
            }
        }
    }
}

/**
 * @brief Queue the on-enter event of the current state for deferred dispatch.
 *
 * If the current state declares an on-enter event (hasOnEnterEvent == true),
 * populates the pending event fields so that the next tick() call dispatches
 * the event frame via sendEventLocked().  This avoids sending a radio frame
 * while the mutex is held inside enterStateLocked().
 *
 * @param[in] nowMs  Timestamp to embed in the event frame (milliseconds).
 * @pre  mutex_ is held by the caller.
 * @pre  currentState_ < program_.stateCount.
 */
void MissionScriptEngine::sendOnEnterEventLocked(uint64_t nowMs)
{
    if (currentState_ >= program_.stateCount)
    {
        return;
    }

    // AMS-4.8: execute set actions (non-const: DELTA/MAX/MIN update state per call).
    StateDef& st = program_.states[currentState_];

    // AMS-4.8: execute set actions synchronously on state entry.
    if (st.setActionCount > 0U)
    {
        executeSetActionsLocked(st, nowMs);
    }

    // AMS-4.17: fire pulse channels declared in on_enter: block.
    if (st.pulseActionCount > 0U)
    {
        executePulseActionsLocked(st);
    }

    if (!st.hasOnEnterEvent)
    {
        return;
    }

    pendingOnEnterEvent_ = true;
    pendingEventVerb_ = st.onEnterVerb;
    pendingEventTsMs_ = nowMs;
    ares::util::copyZ(pendingEventText_, st.onEnterText, sizeof(pendingEventText_));
}

/**
 * @brief Execute all PULSE.fire actions declared in a state's on_enter: block.
 *
 * Safety gate: only fires when the engine is RUNNING and execution is enabled.
 * If pulseIface_ is null (SITL / no pulse hardware) the call is silently skipped.
 *
 * @param[in] st  State definition containing pulseActions[].
 * @pre  mutex_ is held by the caller.
 */
void MissionScriptEngine::executePulseActionsLocked(const StateDef& st)
{
    if (pulseIface_ == nullptr
        || !executionEnabled_
        || status_ != EngineStatus::RUNNING)
    {
        return;
    }

    for (uint8_t i = 0U; i < st.pulseActionCount; i++)
    {
        const StateDef::PulseAction& act = st.pulseActions[i];
        const uint32_t dur = (act.durationMs == 0U)
                             ? static_cast<uint32_t>(ares::FIRE_DURATION_MS)
                             : act.durationMs;

        if (pulseIface_->fire(act.channel, dur))
        {
            // Set status bit directly — we already hold the mutex (Locked context).
            if (act.channel == 0U) { pulseAFired_ = true; }
            else if (act.channel == 1U) { pulseBFired_ = true; }
            LOG_I(TAG, "PULSE ch=%u fired dur=%" PRIu32 "ms",
                  static_cast<uint32_t>(act.channel), dur);
        }
        else
        {
            LOG_E(TAG, "PULSE ch=%u fire failed",
                  static_cast<uint32_t>(act.channel));
            setErrorLocked("PULSE.fire: driver rejected fire command");
        }
    }
}

} // namespace ams
} // namespace ares
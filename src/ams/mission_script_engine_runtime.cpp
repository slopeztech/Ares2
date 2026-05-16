/**
 * @file  mission_script_engine_runtime.cpp
 * @brief AMS engine â€” runtime state machine: tick, transitions, guards.
 *
 * Contains nextWakeupMs, tick, transition evaluation, guard conditions,
 * state entry/exit, and snapshot helpers.
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
static uint8_t clampActionBudget(uint8_t budget)
{
    if (budget == 0U) { return 1U; }
    if (budget > 3U)  { return 3U; }
    return budget;
}

/**
 * @brief Select the highest-priority action group that has pending work.
 *
 * Iterates over the three action groups (EVENT=0, HK=1, LOG=2) and returns
 * the index of the one with the highest priority among those with
 * @p due set to @c true.  Selecting a group dispatches the entire group
 * (all due HK slots or all due LOG slots) within one budget step (AMS-4.4).
 *
 * @param[in] priorities  Priority of each group (0-9; higher = earlier).
 * @param[in] due         Which groups have pending work.
 * @return Index of the winning group (0, 1, or 2), or -1 if none are due.
 */
static int8_t pickBestDueActionIndex(const uint8_t priorities[3],
                                     const bool    due[3])
{
    int8_t  best         = -1;
    uint8_t bestPriority = 0U;

    for (uint8_t i = 0; i < 3U; i++)
    {
        if (!due[i]) { continue; }
        if (best < 0 || priorities[i] > bestPriority)
        {
            best         = static_cast<int8_t>(i);
            bestPriority = priorities[i];
        }
    }

    return best;
}

void MissionScriptEngine::tick(uint64_t nowMs) // NOLINT(readability-function-size)
{
    // Reset staging buffers before every tick.  They are populated inside the
    // mutex scope below and flushed by flushPendingIoUnlocked() after the lock
    // is released, keeping LittleFS write latency out of the critical section
    // so that injectTcCommand("ABORT") and other API callers cannot time out
    // waiting for I/O (AMS-8.3).
    pendingCheckpoint_.pending = false;
    pendingAppendCount_        = 0U;

    {
        ScopedLock guard(mutex_, pdMS_TO_TICKS(ares::AMS_MUTEX_TIMEOUT_MS));
        if (!guard.acquired())
        {
            return;
        }

        if (!running_ || status_ != EngineStatus::RUNNING || !executionEnabled_)
        {
            return;
        }

        if (currentState_ >= program_.stateCount)
        {
            setErrorLocked("state index out of range");
            return;
        }

    StateDef& state = program_.states[currentState_];

    // AMS-4.8.2: advance any in-progress CALIBRATE actions by one sample before
    // evaluating transitions, so that calibrated variables are current when
    // conditions are checked this tick.
    stepPendingCalibrationsLocked(state, nowMs);

    if (evaluateTransitionAndMaybeEnterLocked(state, nowMs)) { return; }

    if (checkOnTimeoutLocked(state, nowMs)) { return; }

    if (state.hasFallback && state.fallbackTargetResolved)
    {
        const uint32_t elapsed = nowMs - stateEnterMs_;
        if (elapsed >= state.fallbackAfterMs)
        {
            LOG_I(TAG, "Fallback: '%s' -> '%s' (timeout %" PRIu32 "ms)",
                  state.name,
                  program_.states[state.fallbackTargetIdx].name,
                  elapsed);
            exitStateLocked(currentState_, nowMs);
            enterStateLocked(state.fallbackTargetIdx, nowMs);
            return;
        }
    }

    if (pendingTc_ == TcCommand::ABORT)
    {
        LOG_W(TAG, "ABORT TC not consumed by state=%s: force-deactivating",
              state.name);
        // Emit an auditable ERROR event so the ground station knows which state
        // was active when the unhandled ABORT triggered the force-deactivation.
        char abortMsg[80] = {};
        snprintf(abortMsg, sizeof(abortMsg),
                 "ABORT: forced in state=%s", state.name);
        sendEventLocked(EventVerb::ERROR,
                        ares::proto::EventId::FPL_VIOLATION,
                        abortMsg, nowMs);
        // Persist a final "abort" row in the mission log (survives deactivate
        // because queueAppendLocked copies the path before logPath_ is cleared).
        writeAbortMarkerLocked(state.name, nowMs);
        deactivateLocked();
        return;
    }

    // AMS-5.2: Evaluate guard conditions after transition, before actions.
    // If a condition is violated, on_error fires and engine halts.
    if (evaluateConditionsLocked(state, nowMs))
    {
        return;
    }

    executeDueActionsLocked(state, nowMs);

    // AMS-11: evaluate background tasks (run after main state actions).
    if (program_.taskCount > 0U)
    {
        runTasksLocked(nowMs);
    }

    (void)saveResumePointLocked(nowMs, false);

    const bool hasActiveHk = (state.hkSlotCount > 0U)
                          || (state.hasHkEvery
                              && state.hkEveryMs > 0U
                              && state.hkFieldCount > 0U);
    const bool hasActiveLog = (state.logSlotCount > 0U)
                           || (state.hasLogEvery
                               && state.logEveryMs > 0U
                               && state.logFieldCount > 0U);
    if (state.transitionCount == 0U
        && !state.hasFallback
        && !state.hasOnTimeout
        && !pendingOnEnterEvent_
        && !hasActiveHk
        && !hasActiveLog)
    {
        if (status_ == EngineStatus::RUNNING)
        {
            LOG_I(TAG, "mission complete: terminal state=%s", state.name);
            running_ = false;
            status_  = EngineStatus::COMPLETE;
            clearResumePointLocked();
        }
    }
    } // Mutex released; all staging complete.

    // Perform deferred LittleFS writes (log appends + checkpoint) outside the
    // critical section (AMS-8.3).
    flushPendingIoUnlocked();
}

/**
 * @brief Check the active state's transition conditions and enter the target
 *        state if they hold.
 *
 * Evaluates up to AMS_MAX_TRANSITION_CONDS sub-conditions combined by OR or
 * AND logic (AMS-4.6.2).  Delta conditions compare the current reading against
 * the previous reading stored in transitionPrevVal_[i].  The optional hold
 * window (AMS-4.6.1) is applied to the compound result before firing.
 *
 * TC tokens are consumed only when the overall compound condition evaluates
 * to true and the transition actually fires.
 *
 * @param[in,out] state  Current state definition.
 * @param[in]     nowMs  Current system time in milliseconds.
 * @return @c true if a transition was triggered, @c false otherwise.
 * @pre  mutex_ is held by the caller.
 * @pre  currentState_ < program_.stateCount.
 */
bool MissionScriptEngine::evaluateTransitionAndMaybeEnterLocked(StateDef& state,
                                                                uint64_t nowMs)
{
    ARES_ASSERT(currentState_ < program_.stateCount);

    if (state.transitionCount == 0U)
    {
        return false;
    }

    // Evaluate each transition independently; the first one whose compound
    // condition holds (and whose hold window has elapsed) fires.  Transitions
    // are tested in declaration order so script authors control priority.
    for (uint8_t ti = 0; ti < state.transitionCount; ti++)
    {
        const Transition& tr = state.transitions[ti];
        if (tr.condCount == 0U) { continue; }

        // Compute the flat base index for this transition's prev-value slots.
        const uint8_t prevBase =
            static_cast<uint8_t>(ti * static_cast<uint8_t>(ares::AMS_MAX_TRANSITION_CONDS));

        bool compound = (tr.logic == TransitionLogic::AND);
        bool tcPendingMatch = false;

        for (uint8_t i = 0; i < tr.condCount; i++)
        {
            const uint8_t flatIdx = static_cast<uint8_t>(prevBase + i);
            const bool condResult = evaluateOneTransitionConditionLocked(
                tr.conds[i], state, flatIdx, nowMs, tcPendingMatch);
            if (tr.logic == TransitionLogic::OR)
            {
                compound = compound || condResult;
            }
            else
            {
                compound = compound && condResult;
            }
        }

        if (!compound)
        {
            transitionCondHolding_[ti] = false;  // compound fell false — reset this transition's hold window
            continue;
        }

        if (!applyTransitionHoldLocked(tr, ti, nowMs))
        {
            continue;  // still within hold window — keep evaluating on future ticks
        }

        if (tcPendingMatch)
        {
            consumeMatchedTransitionTcLocked(tr);
            LOG_I(TAG, "TC condition matched in state=%s (transition %u)", state.name,
                  static_cast<unsigned>(ti));
        }

        return fireResolvedTransitionLocked(state, tr, nowMs);
    }

    return false;
}

bool MissionScriptEngine::evaluateOneTransitionConditionLocked(const CondExpr& cond,
                                                               StateDef&,
                                                               uint8_t         condIdx,
                                                               uint64_t        nowMs,
                                                               bool&           tcPendingMatch)
{
    bool condResult = false;
    switch (cond.kind)
    {
    case CondKind::TC_EQ:
        if (cond.tcDebounce == TcDebounceMode::CONFIRM)
        {
            const uint8_t idx = static_cast<uint8_t>(cond.tcValue);
            condResult = (idx < static_cast<uint8_t>(sizeof(tcConfirmCount_)))
                       && (tcConfirmCount_[idx] >= cond.tcConfirmN);
        }
        else
        {
            condResult = (pendingTc_ == cond.tcValue);
        }
        if (condResult) { tcPendingMatch = true; }
        break;

    case CondKind::TIME_GT:
        condResult = ((nowMs - stateEnterMs_) > static_cast<uint64_t>(cond.threshold));
        break;

    case CondKind::SENSOR_LT:
    case CondKind::SENSOR_GT:
    {
        float thr = 0.0f;
        if (!resolveVarThresholdLocked(cond, thr)) { break; }
        float val = 0.0f;
        if (readSensorFloatLocked(cond.alias, cond.field, val))
        {
            condResult = (cond.kind == CondKind::SENSOR_LT) ? (val < thr) : (val > thr);
        }
        break;
    }

    case CondKind::SENSOR_DELTA_LT:
    case CondKind::SENSOR_DELTA_GT:
    {
        float curVal = 0.0f;
        float thr = 0.0f;
        if (readSensorFloatLocked(cond.alias, cond.field, curVal)
            && resolveVarThresholdLocked(cond, thr))
        {
            if (transitionPrevValid_[condIdx])
            {
                const float delta = curVal - transitionPrevVal_[condIdx];
                condResult = (cond.kind == CondKind::SENSOR_DELTA_LT)
                           ? (delta < thr)
                           : (delta > thr);
            }
            transitionPrevVal_[condIdx] = curVal;
            transitionPrevValid_[condIdx] = true;
        }
        break;
    }

    default:
        break;
    }

    return condResult;
}

bool MissionScriptEngine::applyTransitionHoldLocked(const Transition& tr,
                                                    uint8_t           trIdx,
                                                    uint64_t          nowMs)
{
    ARES_ASSERT(trIdx < ares::AMS_MAX_TRANSITIONS);

    const bool isTcOnly = (tr.condCount == 1U && tr.conds[0].kind == CondKind::TC_EQ);
    if (tr.holdMs == 0U || isTcOnly)
    {
        return true;
    }

    if (!transitionCondHolding_[trIdx])
    {
        transitionCondHolding_[trIdx] = true;
        transitionCondMetMs_[trIdx]   = nowMs;
        return false;
    }

    if ((nowMs - transitionCondMetMs_[trIdx]) < tr.holdMs)
    {
        return false;
    }

    transitionCondHolding_[trIdx] = false;
    return true;
}

void MissionScriptEngine::consumeMatchedTransitionTcLocked(const Transition& tr)
{
    pendingTc_ = TcCommand::NONE;
    for (uint8_t ci = 0; ci < tr.condCount; ci++)
    {
        if (tr.conds[ci].kind == CondKind::TC_EQ
            && tr.conds[ci].tcDebounce == TcDebounceMode::CONFIRM)
        {
            const uint8_t idx = static_cast<uint8_t>(tr.conds[ci].tcValue);
            if (idx < static_cast<uint8_t>(sizeof(tcConfirmCount_)))
            {
                tcConfirmCount_[idx] = 0U;
            }
        }
    }
}

bool MissionScriptEngine::fireResolvedTransitionLocked(const StateDef&   state,
                                                       const Transition& tr,
                                                       uint64_t          nowMs)
{
    if (!tr.targetResolved || tr.targetIndex >= program_.stateCount)
    {
        setErrorLocked("invalid transition target");
        return true;
    }

    LOG_I(TAG, "Transition: '%s' -> '%s' at t=%" PRIu64 "ms",
          state.name,
          program_.states[tr.targetIndex].name,
          nowMs);
    exitStateLocked(currentState_, nowMs);
    enterStateLocked(tr.targetIndex, nowMs);
    return true;
}

/**
 * @brief Execute the highest-priority due action groups within the per-tick budget.
 *
 * On each call, up to `state.actionBudget` (clamped 1-3 by APUS-19.2) action
 * **groups** are dispatched in priority order: EVENT > HK > LOG (configurable
 * via `priorities` in the AMS script).  Within a selected group ALL due slots
 * fire before the next group is considered (AMS-4.4).  Timers for dispatched
 * slots are advanced by their configured cadence.
 *
 * @param[in] state  Current state definition (contains schedules and fields).
 * @param[in] nowMs  Current system time in milliseconds.
 * @pre  mutex_ is held by the caller.
 * @pre  currentState_ < program_.stateCount.
 */
void MissionScriptEngine::executeDueActionsLocked(const StateDef& state, uint64_t nowMs) // NOLINT(readability-function-size)
{
    ARES_ASSERT(currentState_ < program_.stateCount);

    // AMS-4.3.1: evaluate due status for every HK and LOG slot.
    // Slots beyond hkSlotCount/logSlotCount are never due.
    bool hkSlotDue[ares::AMS_MAX_HK_SLOTS]  = {};
    bool logSlotDue[ares::AMS_MAX_HK_SLOTS] = {};

    for (uint8_t s = 0; s < state.hkSlotCount; s++)
    {
        const HkSlot& sl = state.hkSlots[s];
        hkSlotDue[s] = (sl.everyMs > 0U)
                    && (sl.fieldCount > 0U)
                    && ((nowMs - lastHkSlotMs_[s]) >= sl.everyMs);
    }
    for (uint8_t s = 0; s < state.logSlotCount; s++)
    {
        const HkSlot& sl = state.logSlots[s];
        logSlotDue[s] = (sl.everyMs > 0U)
                     && (sl.fieldCount > 0U)
                     && ((nowMs - lastLogSlotMs_[s]) >= sl.everyMs);
    }

    // Build per-group due flags for pickBestDueActionIndex (AMS-4.4).
    // Each entry represents one action GROUP (EVENT=0, HK=1, LOG=2).
    // A group is "due" if at least one slot within it is due; selecting
    // the group will fire ALL due slots inside it in one budget step.
    bool anyHkDue  = false;
    bool anyLogDue = false;
    for (uint8_t s = 0; s < state.hkSlotCount;  s++) { if (hkSlotDue[s])  { anyHkDue  = true; break; } }
    for (uint8_t s = 0; s < state.logSlotCount; s++) { if (logSlotDue[s]) { anyLogDue = true; break; } }

    // Fall back to legacy single-slot logic when no multi-slot data is present.
    if (state.hkSlotCount == 0U)
    {
        anyHkDue = state.hasHkEvery
                && state.hkEveryMs > 0U
                && state.hkFieldCount > 0U
                && ((nowMs - lastHkMs_) >= state.hkEveryMs);
    }
    if (state.logSlotCount == 0U)
    {
        anyLogDue = state.hasLogEvery
                 && state.logEveryMs > 0U
                 && state.logFieldCount > 0U
                 && ((nowMs - lastLogMs_) >= state.logEveryMs);
    }

    const bool eventDue = pendingOnEnterEvent_;

    bool due[3] = {eventDue, anyHkDue, anyLogDue};
    const uint8_t priorities[3] = {
        state.eventPriority,
        state.hkPriority,
        state.logPriority,
    };
    const uint8_t budget = clampActionBudget(state.actionBudget);

    for (uint8_t step = 0; step < budget; step++)
    {
        const int8_t best = pickBestDueActionIndex(priorities, due);

        if (best < 0)
        {
            break;
        }

        if (best == 0)
        {
            sendEventLocked(pendingEventVerb_,
                            ares::proto::EventId::PHASE_CHANGE,
                            pendingEventText_,
                            pendingEventTsMs_);
            pendingOnEnterEvent_ = false;
            pendingEventText_[0] = '\0';
            due[0] = false;
        }
        else if (best == 1)
        {
            // AMS-4.3.1: fire all due HK slots within this budget step.
            if (state.hkSlotCount > 0U)
            {
                for (uint8_t s = 0; s < state.hkSlotCount; s++)
                {
                    if (hkSlotDue[s])
                    {
                        sendHkReportSlotLocked(nowMs, state.hkSlots[s]);
                        lastHkSlotMs_[s] += state.hkSlots[s].everyMs;
                        // Starvation guard (AMS-4.4): if still ≥1 period behind after
                        // the timer advance, skip to nowMs and emit one WARN so the
                        // radio bus is not flooded by N consecutive catch-up bursts.
                        if ((nowMs - lastHkSlotMs_[s]) >= state.hkSlots[s].everyMs)
                        {
                            const uint32_t skipped = static_cast<uint32_t>(
                                (nowMs - lastHkSlotMs_[s]) / state.hkSlots[s].everyMs);
                            char warnMsg[64] = {};
                            snprintf(warnMsg, sizeof(warnMsg),
                                     "HK slot %u skipped %" PRIu32 " samples",
                                     static_cast<uint32_t>(s), skipped);
                            sendEventLocked(EventVerb::WARN,
                                            ares::proto::EventId::FPL_VIOLATION,
                                            warnMsg, nowMs);
                            lastHkSlotMs_[s] = nowMs;
                        }
                        hkSlotDue[s] = false;
                    }
                }
            }
            else
            {
                // Legacy fallback.
                sendHkReportLocked(nowMs);
                lastHkMs_ += state.hkEveryMs;
                // Starvation guard (AMS-4.4).
                if ((nowMs - lastHkMs_) >= state.hkEveryMs)
                {
                    const uint32_t skipped = static_cast<uint32_t>(
                        (nowMs - lastHkMs_) / state.hkEveryMs);
                    char warnMsg[64] = {};
                    snprintf(warnMsg, sizeof(warnMsg),
                             "HK slot 0 skipped %" PRIu32 " samples", skipped);
                    sendEventLocked(EventVerb::WARN,
                                    ares::proto::EventId::FPL_VIOLATION,
                                    warnMsg, nowMs);
                    lastHkMs_ = nowMs;
                }
            }
            due[1] = false;
        }
        else
        {
            // AMS-4.3.1: fire all due LOG slots within this budget step.
            if (state.logSlotCount > 0U)
            {
                for (uint8_t s = 0; s < state.logSlotCount; s++)
                {
                    if (logSlotDue[s])
                    {
                        appendLogReportSlotLocked(nowMs, state.logSlots[s], s);
                        lastLogSlotMs_[s] += state.logSlots[s].everyMs;
                        // Starvation guard (AMS-4.4).
                        if ((nowMs - lastLogSlotMs_[s]) >= state.logSlots[s].everyMs)
                        {
                            const uint32_t skipped = static_cast<uint32_t>(
                                (nowMs - lastLogSlotMs_[s]) / state.logSlots[s].everyMs);
                            char warnMsg[64] = {};
                            snprintf(warnMsg, sizeof(warnMsg),
                                     "LOG slot %u skipped %" PRIu32 " samples",
                                     static_cast<uint32_t>(s), skipped);
                            sendEventLocked(EventVerb::WARN,
                                            ares::proto::EventId::FPL_VIOLATION,
                                            warnMsg, nowMs);
                            lastLogSlotMs_[s] = nowMs;
                        }
                        logSlotDue[s] = false;
                    }
                }
            }
            else
            {
                // Legacy fallback.
                appendLogReportLocked(nowMs);
                lastLogMs_ += state.logEveryMs;
                // Starvation guard (AMS-4.4).
                if ((nowMs - lastLogMs_) >= state.logEveryMs)
                {
                    const uint32_t skipped = static_cast<uint32_t>(
                        (nowMs - lastLogMs_) / state.logEveryMs);
                    char warnMsg[64] = {};
                    snprintf(warnMsg, sizeof(warnMsg),
                             "LOG slot 0 skipped %" PRIu32 " samples", skipped);
                    sendEventLocked(EventVerb::WARN,
                                    ares::proto::EventId::FPL_VIOLATION,
                                    warnMsg, nowMs);
                    lastLogMs_ = nowMs;
                }
            }
            due[2] = false;
        }
    }
}

void MissionScriptEngine::getSnapshot(EngineSnapshot& out) const
{
    ScopedLock guard(mutex_, pdMS_TO_TICKS(ares::AMS_MUTEX_TIMEOUT_MS));
    if (!guard.acquired())
    {
        return;
    }

    out.status = status_;
    ares::util::copyZ(out.activeFile, activeFile_, sizeof(out.activeFile));

    if (running_ && currentState_ < program_.stateCount)
    {
        ares::util::copyZ(out.stateName, program_.states[currentState_].name,
                sizeof(out.stateName));
    }
    else
    {
        ares::util::copyZ(out.stateName, "IDLE", sizeof(out.stateName));
    }

    ares::util::copyZ(out.lastError, lastError_, sizeof(out.lastError));
}

bool MissionScriptEngine::listScripts(FileEntry* entries,
                                      uint8_t maxEntries,
                                      uint8_t& count) const
{
    count = 0;
    if (entries == nullptr || maxEntries == 0U)
    {
        return false;
    }

    FileEntry tmp[ares::MAX_LOG_FILES] = {};
    uint8_t tmpCount = 0;

    const StorageStatus st = storage_.listFiles(
        ares::MISSION_DIR, tmp, ares::MAX_LOG_FILES, tmpCount);

    if (st == StorageStatus::NOT_FOUND)
    {
        return true;
    }
    if (st != StorageStatus::OK)
    {
        return false;
    }

    for (uint8_t i = 0; i < tmpCount && count < maxEntries; i++)
    {
        const char* name = tmp[i].name;
        const uint32_t len = static_cast<uint32_t>(
            strnlen(name, ares::STORAGE_MAX_PATH));

        if (len < 4U)
        {
            continue;
        }
        if (strcmp(&name[len - 4U], ".ams") != 0)
        {
            continue;
        }

        entries[count] = tmp[i];
        count++;
    }

    return true;
}

const char* MissionScriptEngine::sensorFieldNameForLog(SensorField f)
{
    switch (f)
    {
    case SensorField::LAT:       return "lat";
    case SensorField::LON:       return "lon";
    case SensorField::ALT:       return "alt";
    case SensorField::SPEED:     return "speed";
    case SensorField::TEMP:      return "temp";
    case SensorField::PRESSURE:  return "pressure";
    case SensorField::ACCEL_X:   return "accel_x";
    case SensorField::ACCEL_Y:   return "accel_y";
    case SensorField::ACCEL_Z:   return "accel_z";
    case SensorField::ACCEL_MAG: return "accel_mag";
    case SensorField::GYRO_X:    return "gyro_x";
    case SensorField::GYRO_Y:    return "gyro_y";
    case SensorField::GYRO_Z:    return "gyro_z";
    case SensorField::GYRO_MAG:  return "gyro_mag";
    case SensorField::VAR:       return "var";
    case SensorField::IMU_TEMP:  return "temp";
    default:                     return "?";
    }
}

bool MissionScriptEngine::evaluateGuardExprHoldsLocked(const CondExpr& expr,
                                                       const StateDef&,
                                                       uint64_t        nowMs,
                                                       float&          actualVal) const
{
    bool holds = true;
    switch (expr.kind)
    {
    case CondKind::TIME_GT:
        actualVal = static_cast<float>(nowMs - stateEnterMs_);
        holds = (static_cast<uint64_t>(actualVal)
                 <= static_cast<uint64_t>(expr.threshold));
        break;

    case CondKind::SENSOR_LT:
    case CondKind::SENSOR_GT:
        if (readSensorFloatLocked(expr.alias, expr.field, actualVal))
        {
            float thr = 0.0f;
            if (resolveVarThresholdLocked(expr, thr))
            {
                holds = (expr.kind == CondKind::SENSOR_LT)
                      ? (actualVal < thr)
                      : (actualVal > thr);
            }
            else if (expr.useVar)
            {
                // AMS-5.6: variable RHS not yet calibrated — guard skips
                // (holds stays true).  Log once per evaluation so the
                // operator can diagnose scripts that reference a variable
                // that has never been set by a set action.
                LOG_W(TAG, "guard skipped: variable '%s' not yet set",
                      expr.varName);
            }
        }
        break;

    default:
        break;
    }
    return holds;
}

bool MissionScriptEngine::handleGuardViolationLocked(const StateDef& state,
                                                     const CondExpr& expr,
                                                     float           actualVal,
                                                     uint64_t        nowMs)
{
    logGuardViolationLocked(state, expr, actualVal, nowMs);
    return applyGuardErrorLocked(state, nowMs, expr, actualVal);
}

void MissionScriptEngine::logGuardViolationLocked(const StateDef& state,
                                                  const CondExpr& expr,
                                                  float           actualVal,
                                                  uint64_t        nowMs)
{
    char valText[16] = {};
    char thrText[16] = {};
    if (!detail::formatScaledFloat(actualVal, 3U, valText, sizeof(valText)))
    {
        ares::util::copyZ(valText, "?", sizeof(valText));
    }
    if (!detail::formatScaledFloat(expr.threshold, 3U, thrText, sizeof(thrText)))
    {
        ares::util::copyZ(thrText, "?", sizeof(thrText));
    }

    if (expr.kind == CondKind::TIME_GT)
    {
        LOG_W(TAG,
              "Guard Violation: State '%s', Condition "
              "'TIME.elapsed > %s' failed (Value: %sms)",
              state.name, thrText, valText);
    }
    else
    {
        const char* opStr = (expr.kind == CondKind::SENSOR_LT) ? "<" : ">";
        LOG_W(TAG,
              "Guard Violation: State '%s', Condition "
              "'%s.%s %s %s' failed (Value: %s)",
              state.name, expr.alias,
              sensorFieldNameForLog(expr.field), opStr, thrText, valText);
    }

    if (state.hasOnErrorEvent)
    {
        sendEventLocked(state.onErrorVerb,
                        ares::proto::EventId::FPL_VIOLATION,
                        state.onErrorText,
                        nowMs);
    }
}

bool MissionScriptEngine::applyGuardErrorLocked(const StateDef& state,
                                                uint64_t        nowMs,
                                                const CondExpr& firstExpr,
                                                float           firstActualVal)
{
    if (state.hasOnErrorTransition && state.onErrorTransitionResolved)
    {
        LOG_I(TAG,
              "Error recovery: '%s' -> '%s' (guard violated)",
              state.name,
              program_.states[state.onErrorTransitionIdx].name);
        exitStateLocked(currentState_, nowMs);
        enterStateLocked(state.onErrorTransitionIdx, nowMs);
        return true;
    }

    // Build a detailed error string for lastError_ (visible via GET /api/mission).
    char valText[16] = {};
    char thrText[16] = {};
    char msg[ares::AMS_MAX_ERROR_TEXT] = {};

    if (!detail::formatScaledFloat(firstActualVal, 3U, valText, sizeof(valText)))
    {
        ares::util::copyZ(valText, "?", sizeof(valText));
    }
    if (!detail::formatScaledFloat(firstExpr.threshold, 3U, thrText, sizeof(thrText)))
    {
        ares::util::copyZ(thrText, "?", sizeof(thrText));
    }

    if (firstExpr.kind == CondKind::TIME_GT)
    {
        (void)snprintf(msg, sizeof(msg),
                       "guard TIME.elapsed > %s failed (val=%s)",
                       thrText, valText);
    }
    else
    {
        const char* opStr = (firstExpr.kind == CondKind::SENSOR_LT) ? "<" : ">";
        (void)snprintf(msg, sizeof(msg),
                       "guard %s.%s %s %s failed (val=%s)",
                       firstExpr.alias, sensorFieldNameForLog(firstExpr.field),
                       opStr, thrText, valText);
    }
    msg[sizeof(msg) - 1U] = '\0';

    setErrorLocked(msg);
    return true;
}

/**
 * @brief Evaluate all guard conditions for the current state (AMS-4.7).
 *
 * Iterates over every CondExpr in @p state.conditions.  If any condition is
 * violated the engine is put into ERROR via setErrorLocked() and the optional
 * on_error event is queued.  Time-based and sensor-based condition kinds are
 * supported; an unreadable sensor defaults to "holds" (no false positive).
 *
 * @param[in] state  State whose conditions are evaluated.
 * @param[in] nowMs  Current system time in milliseconds.
 * @return @c true if at least one condition was violated, @c false if all hold.
 * @pre  mutex_ is held by the caller.
 */
bool MissionScriptEngine::evaluateConditionsLocked(const StateDef& state,
                                                   uint64_t nowMs)
{
    if (state.conditionCount == 0U)
    {
        return false;
    }

    bool anyViolation = false;
    const CondExpr* firstViolated  = nullptr;
    float           firstActualVal = 0.0f;

    for (uint8_t i = 0; i < state.conditionCount; i++)  // PO10-2: bounded
    {
        const CondExpr& expr = state.conditions[i].expr;
        float actualVal = 0.0f;
        const bool holds = evaluateGuardExprHoldsLocked(expr, state, nowMs, actualVal);
        if (!holds)
        {
            logGuardViolationLocked(state, expr, actualVal, nowMs);
            if (firstViolated == nullptr)
            {
                firstViolated  = &state.conditions[i].expr;
                firstActualVal = actualVal;
            }
            anyViolation = true;
        }
    }

    if (anyViolation)
    {
        ARES_ASSERT(firstViolated != nullptr);
        return applyGuardErrorLocked(state, nowMs, *firstViolated, firstActualVal);
    }

    return false;
}

/**
 * @brief Find the index of a state by name.
 *
 * Performs a linear search over the compiled program state table.
 *
 * @param[in] name  Null-terminated state name to search for.
 * @return Index in program_.states[], or program_.stateCount if not found.
 * @pre  mutex_ is held by the caller.
 */
uint8_t MissionScriptEngine::findStateByNameLocked(const char* name) const
{
    for (uint8_t i = 0; i < program_.stateCount; i++)
    {
        if (strcmp(program_.states[i].name, name) == 0)
        {
            return i;
        }
    }
    return program_.stateCount;
}

/**
 * @brief Transition the engine to the ERROR state and record a diagnostic.
 *
 * Sets running_ to false, updates status_ to EngineStatus::ERROR, copies
 * @p reason into lastError_, and emits a LOG_E message.
 *
 * @param[in] reason  Null-terminated error description (truncated to
 *                    sizeof(lastError_) - 1 characters).
 * @pre  mutex_ is held by the caller.
 * @post status_ == EngineStatus::ERROR && running_ == false.
 */
void MissionScriptEngine::setErrorLocked(const char* reason)
{
    running_ = false;
    status_ = EngineStatus::ERROR;

    ares::util::copyZ(lastError_, reason, sizeof(lastError_));

    if (parseLineNum_ > 0U)
    {
        if (parseSourceFile_[0] != '\0')
        {
            LOG_E(TAG, "Parse Error (%s, line %" PRIu32 "): %s",
                  parseSourceFile_, parseLineNum_, lastError_);
        }
        else
        {
            LOG_E(TAG, "Parse Error (line %" PRIu32 "): %s", parseLineNum_, lastError_);
        }
        parseLineNum_ = 0U;           // consume parse context so runtime errors don't inherit it
        parseSourceFile_[0] = '\0';   // consume source file context
    }
    else
    {
        LOG_E(TAG, "error: %s", lastError_);
    }
    clearResumePointLocked();
}

/**
 * @brief Execute on_exit: handler of the state being left (AMS-4.9).
 *
 * Runs set actions then dispatches the optional EVENT synchronously.  Called
 * immediately before enterStateLocked() at every transition site so the
 * farewell event is visible on the bus before the new state's on_enter event.
 * Does NOT fire on deactivate() or when the engine enters ERROR without a
 * recovery transition.
 *
 * @param[in] stateIndex  Index of the departing state in program_.states[].
 * @param[in] nowMs       Current system time in milliseconds.
 * @pre  mutex_ is held by the caller.
 */
void MissionScriptEngine::exitStateLocked(uint8_t stateIndex, uint64_t nowMs)
{
    if (stateIndex >= program_.stateCount) { return; }
    StateDef& st = program_.states[stateIndex];

    // AMS-4.9: execute on_exit set actions synchronously before leaving state.
    for (uint8_t i = 0U; i < st.onExitSetCount; ++i)  // PO10-2: bounded
    {
        executeOneSetActionLocked(st.onExitSetActions[i], nowMs);
    }

    // AMS-4.9: dispatch on_exit EVENT immediately — deterministic ordering
    // guarantees it fires before on_enter of the incoming state.
    if (st.hasOnExitEvent)
    {
        sendEventLocked(st.onExitVerb,
                        inferEventId(st.onExitVerb),
                        st.onExitText, nowMs);
    }
}

/**
 * @brief Check whether the on_timeout deadline has elapsed and fire if so.
 *
 * Called every tick after regular transitions and before the fallback transition.
 * If the state has an on_timeout block and the elapsed time since state entry
 * equals or exceeds onTimeoutMs, the optional event is emitted and the engine
 * transitions to the configured target state.
 *
 * @param[in] state   Current state definition.
 * @param[in] nowMs   Current system time in milliseconds.
 * @return @c true if the timeout fired (caller must return immediately).
 * @pre  mutex_ is held by the caller.
 */
bool MissionScriptEngine::checkOnTimeoutLocked(StateDef& state, uint64_t nowMs)
{
    if (!state.hasOnTimeout || !state.onTimeoutTransitionResolved) { return false; }

    const uint64_t elapsed = nowMs - stateEnterMs_;
    if (elapsed < state.onTimeoutMs) { return false; }

    LOG_I(TAG, "on_timeout: '%s' -> '%s' (elapsed %" PRIu64 "ms)",
          state.name,
          program_.states[state.onTimeoutTransitionIdx].name,
          elapsed);

    if (state.hasOnTimeoutEvent)
    {
        sendEventLocked(state.onTimeoutVerb,
                        inferEventId(state.onTimeoutVerb),
                        state.onTimeoutText, nowMs);
    }

    exitStateLocked(currentState_, nowMs);
    enterStateLocked(state.onTimeoutTransitionIdx, nowMs);
    return true;
}

/**
 * @brief Enter a new state: update index, reset all timers, queue on-enter event.
 *
 * Updates currentState_, resets stateEnterMs_, lastHkMs_, and lastLogMs_ to
 * @p nowMs, logs the transition, and calls sendOnEnterEventLocked() to schedule
 * any on-enter telemetry event.
 *
 * @param[in] stateIndex  Index of the target state in program_.states[].
 * @param[in] nowMs       Current system time in milliseconds.
 * @pre  mutex_ is held by the caller.
 * @pre  stateIndex < program_.stateCount.
 */
void MissionScriptEngine::enterStateLocked(uint8_t stateIndex, uint64_t nowMs)
{
    if (stateIndex >= program_.stateCount)
    {
        setErrorLocked("state transition out of range");
        return;
    }

    currentState_ = stateIndex;
    stateEnterMs_ = nowMs;
    lastHkMs_ = nowMs;
    lastLogMs_ = nowMs;
    // AMS-4.3.1: reset per-slot HK/LOG timers so the new state's cadences start
    // from zero, not from the previous state's last-fire timestamps.
    for (uint8_t s = 0U; s < ares::AMS_MAX_HK_SLOTS; s++)
    {
        lastHkSlotMs_[s]  = nowMs;
        lastLogSlotMs_[s] = nowMs;
    }
    // AMS-4.6.1: reset all per-transition hold windows on every state entry.
    for (uint8_t ti = 0; ti < ares::AMS_MAX_TRANSITIONS; ti++)
    {
        transitionCondHolding_[ti] = false;
        transitionCondMetMs_[ti]   = 0U;
    }
    // AMS-4.11.2: reset TC CONFIRM counters on state entry (clean slate per state).
    memset(tcConfirmCount_, 0U, sizeof(tcConfirmCount_));
    // AMS-4.6.2: reset flat prev-value table so no delta baseline carries over between states.
    constexpr uint8_t kPrevSlots =
        static_cast<uint8_t>(ares::AMS_MAX_TRANSITIONS * ares::AMS_MAX_TRANSITION_CONDS);
    for (uint8_t pi = 0U; pi < kPrevSlots; pi++)
    {
        transitionPrevVal_[pi]   = 0.0f;
        transitionPrevValid_[pi] = false;
    }

    LOG_I(TAG, "state -> %s", program_.states[stateIndex].name);

    // AMS-4.8.2: reset async calibration progress for the entering state's set
    // actions so that CALIBRATE always starts fresh on every state entry (including
    // re-entries).  Also clear task-rule calibration progress so that a task firing
    // in the new state does not resume a stale accumulator from a prior state.
    StateDef& enteredState = program_.states[stateIndex];
    for (uint8_t i = 0U; i < enteredState.setActionCount; i++)
    {
        enteredState.setActions[i].calibInProgress = false;
        enteredState.setActions[i].calibCollected  = 0U;
    }
    for (uint8_t t = 0U; t < program_.taskCount; t++)
    {
        for (uint8_t r = 0U; r < program_.tasks[t].ruleCount; r++)
        {
            program_.tasks[t].rules[r].setAction.calibInProgress = false;
        }
    }

    sendOnEnterEventLocked(nowMs);
    checkpointDirty_ = true; // state change is material — write checkpoint soon
    (void)saveResumePointLocked(nowMs, true);
}
} // namespace ams
} // namespace ares
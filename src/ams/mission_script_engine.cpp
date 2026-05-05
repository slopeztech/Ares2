/**
 * @file  mission_script_engine.cpp
 * @brief AMS engine — lifecycle, public API, and runtime state machine.
 *
 * Contains engine lifecycle (begin/activate/deactivate), the public API
 * (tick, getSnapshot, listScripts, injectTcCommand, setExecutionEnabled),
 * and the core runtime (transition evaluation, action dispatch, guard
 * condition checks, state entry, and event scheduling).
 *
 * Thread safety: public methods acquire the engine mutex internally.
 * Private *Locked helpers are called while the mutex is held by the caller.
 */

#include "ams/mission_script_engine.h"
#include "ams/mission_script_engine_helpers.h"

#include "api/api_common.h"
#include "ares_assert.h"
#include "debug/ares_log.h"

#include <Arduino.h>
#include <cinttypes>
#include <cstdio>
#include <cstring>

namespace ares
{
namespace ams
{

using detail::formatScaledFloat;

static constexpr const char* TAG = "AMS";
static constexpr uint8_t AMS_RESUME_VERSION = 2U;

/**
 * @brief Clamp the per-tick action budget to the valid range [1, 3].
 *
 * A budget of 0 is treated as 1 (at least one action per tick).
 * Values above 3 are capped to 3 (APUS-19.2 maximum).
 *
 * @param[in] budget  Unclamped budget value.
 * @return Value clamped to [1, 3].
 */
static uint8_t clampActionBudget(uint8_t budget)
{
    if (budget == 0U) { return 1U; }
    if (budget > 3U)  { return 3U; }
    return budget;
}

/**
 * @brief Select the highest-priority action slot that has pending work.
 *
 * Iterates over the three action slots (EVENT=0, HK=1, LOG=2) and returns
 * the index of the one with the highest priority among those with
 * @p due set to @c true.
 *
 * @param[in] priorities  Priority of each slot (0-9; higher = earlier).
 * @param[in] due         Which slots have pending work.
 * @return Index of the winning slot (0, 1, or 2), or -1 if none are due.
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

MissionScriptEngine::MissionScriptEngine(StorageInterface&  storage,
                                         const GpsEntry*    gpsDrivers,  uint8_t gpsCount,
                                         const BaroEntry*   baroDrivers, uint8_t baroCount,
                                         const ComEntry*    comDrivers,  uint8_t comCount,
                                         const ImuEntry*    imuDrivers,  uint8_t imuCount)
    : storage_(storage),
      gpsDrivers_(gpsDrivers),   gpsCount_(gpsCount),
      baroDrivers_(baroDrivers), baroCount_(baroCount),
      comDrivers_(comDrivers),   comCount_(comCount),
      imuDrivers_(imuDrivers),   imuCount_(imuCount)
{
}

bool MissionScriptEngine::begin()
{
    mutex_ = xSemaphoreCreateMutexStatic(&mutexBuf_);
    ARES_ASSERT(mutex_ != nullptr);
    if (mutex_ == nullptr)
    {
        return false;
    }

    ScopedLock guard(mutex_, pdMS_TO_TICKS(ares::AMS_MUTEX_TIMEOUT_MS));
    if (!guard.acquired())
    {
        return false;
    }

    (void)tryRestoreResumePointLocked(millis());
    return true;
}

bool MissionScriptEngine::activate(const char* fileName)
{
    ScopedLock guard(mutex_, pdMS_TO_TICKS(ares::AMS_MUTEX_TIMEOUT_MS));
    if (!guard.acquired())
    {
        return false;
    }

    if (!loadFromStorageLocked(fileName))
    {
        return false;
    }

    // Note: ensureLogFileLocked is already called inside loadFromStorageLocked.

    status_ = EngineStatus::LOADED;  // armed via POST /api/arm, not yet executing
    running_ = true;
    executionEnabled_ = false;
    pendingTc_ = TcCommand::NONE;
    pendingOnEnterEvent_ = false;
    pendingEventText_[0] = '\0';
    pendingEventTsMs_ = 0;
    seq_ = 0;
    lastHkMs_ = 0;
    lastLogMs_ = 0;
    memset(lastHkSlotMs_,  0U, sizeof(lastHkSlotMs_));
    memset(lastLogSlotMs_, 0U, sizeof(lastLogSlotMs_));

    uint8_t startIdx = 0;
    const uint8_t waitIdx = findStateByNameLocked("WAIT");
    if (waitIdx < program_.stateCount)
    {
        startIdx = waitIdx;
    }

    enterStateLocked(startIdx, millis());
    LOG_I(TAG, "activated script=%s states=%u",
          activeFile_, static_cast<uint32_t>(program_.stateCount));
    return true;
}

void MissionScriptEngine::deactivateLocked()
{
    LOG_I(TAG, "deactivated");
    running_ = false;
    executionEnabled_ = false;
    status_ = EngineStatus::IDLE;
    program_ = {};
    primaryCom_ = nullptr;
    activeFile_[0] = '\0';
    logPath_[0] = '\0';
    lastError_[0] = '\0';
    pendingTc_ = TcCommand::NONE;
    pendingOnEnterEvent_ = false;
    pendingEventText_[0] = '\0';
    pendingEventTsMs_ = 0;
    transitionCondHolding_ = false;
    transitionCondMetMs_   = 0U;
    for (uint8_t i = 0; i < ares::AMS_MAX_TRANSITION_CONDS; i++)
    {
        transitionPrevVal_[i]   = 0.0f;
        transitionPrevValid_[i] = false;
    }
    memset(taskLastTickMs_, 0U, sizeof(taskLastTickMs_));
    parseCurrentTask_     = 0xFFU;
    parseCurrentTaskRule_ = 0xFFU;

    // Reset telemetry-derived state so a fresh activation starts clean.
    hasPrevAlt_     = false;
    prevAltM_       = 0.0f;
    prevAltMs_      = 0U;
    deltaBaseValid_ = false;
    deltaBaseAlt_   = 0.0f;
    deltaBasePress_ = 0.0f;
    hkTxCount_      = 0U;

    // Reset ST[12] monitoring state (APUS-12) — counters and FSM states only;
    // definitions (limits) are preserved across deactivations.
    for (uint8_t i = 0U; i < kMaxMonitorSlots; ++i)
    {
        monitorSlots_[i].consecutiveHit = 0U;
        if (monitorSlots_[i].state == ares::proto::MonitoringState::MON_ALARM)
        {
            monitorSlots_[i].state = ares::proto::MonitoringState::MON_ENABLED;
        }
    }

    clearResumePointLocked();
}

void MissionScriptEngine::deactivate()
{
    ScopedLock guard(mutex_, pdMS_TO_TICKS(ares::AMS_MUTEX_TIMEOUT_MS));
    if (!guard.acquired())
    {
        return;
    }

    deactivateLocked();
}

bool MissionScriptEngine::injectTcCommand(const char* commandText)
{
    if (commandText == nullptr)
    {
        return false;
    }

    TcCommand cmd = TcCommand::NONE;
    if (!parseTcCommand(commandText, cmd))
    {
        return false;
    }

    ScopedLock guard(mutex_, pdMS_TO_TICKS(ares::AMS_MUTEX_TIMEOUT_MS));
    if (!guard.acquired())
    {
        return false;
    }

    pendingTc_ = cmd;
    // AMS-4.11.2: increment per-TC injection counter (used by CONFIRM debounce mode).
    const uint8_t tcIdx = static_cast<uint8_t>(cmd);
    if (tcIdx < static_cast<uint8_t>(sizeof(tcConfirmCount_))
        && tcConfirmCount_[tcIdx] < 255U)
    {
        tcConfirmCount_[tcIdx]++;
    }
    LOG_I(TAG, "TC command queued: %s", commandText);
    return true;
}

bool MissionScriptEngine::arm()
{
    ScopedLock guard(mutex_, pdMS_TO_TICKS(ares::AMS_MUTEX_TIMEOUT_MS));
    if (!guard.acquired())
    {
        return false;
    }

    if (status_ != EngineStatus::LOADED)
    {
        return false;
    }

    executionEnabled_ = true;
    status_           = EngineStatus::RUNNING;
    pendingTc_        = TcCommand::LAUNCH;
    LOG_I(TAG, "arm: execution enabled, LAUNCH queued");

    (void)saveResumePointLocked(millis(), true);
    return true;
}

void MissionScriptEngine::setExecutionEnabled(bool enabled)
{
    ScopedLock guard(mutex_, pdMS_TO_TICKS(ares::AMS_MUTEX_TIMEOUT_MS));
    if (!guard.acquired())
    {
        return;
    }

    executionEnabled_ = enabled;
    // Transition status with execution enable/disable.
    if (enabled && status_ == EngineStatus::LOADED)
    {
        status_ = EngineStatus::RUNNING;
    }
    else if (!enabled && status_ == EngineStatus::RUNNING)
    {
        status_ = EngineStatus::LOADED;
    }
    LOG_I(TAG, "execution %s", enabled ? "enabled" : "disabled");

    (void)saveResumePointLocked(millis(), true);
}

bool MissionScriptEngine::requestTelemetry(uint32_t nowMs)
{
    ScopedLock guard(mutex_, pdMS_TO_TICKS(ares::AMS_MUTEX_TIMEOUT_MS));
    if (!guard.acquired()) { return false; }
    if (currentState_ >= program_.stateCount) { return false; }

    const StateDef& st = program_.states[currentState_];
    bool sent = false;

    // Transmit each active multi-slot HK entry (AMS-4.3.1).
    for (uint8_t i = 0U; i < st.hkSlotCount; ++i)
    {
        if (st.hkSlots[i].fieldCount > 0U)
        {
            sendHkReportSlotLocked(nowMs, st.hkSlots[i]);
            sent = true;
        }
    }

    // Legacy single-slot fallback — keeps backward compatibility with scripts
    // that use the original single 'every' block.
    if (!sent && st.hasHkEvery)
    {
        sendHkReportLocked(nowMs);
        sent = true;
    }

    return sent;
}

bool MissionScriptEngine::setTelemInterval(uint32_t intervalMs)
{
    ScopedLock guard(mutex_, pdMS_TO_TICKS(ares::AMS_MUTEX_TIMEOUT_MS));
    if (!guard.acquired()) { return false; }
    if (currentState_ >= program_.stateCount) { return false; }

    StateDef& st = program_.states[currentState_];
    bool updated = false;

    for (uint8_t i = 0U; i < st.hkSlotCount; ++i)
    {
        st.hkSlots[i].everyMs = intervalMs;
        updated = true;
    }

    // Keep legacy single-slot fields in sync.
    if (st.hasHkEvery)
    {
        st.hkEveryMs = intervalMs;
        updated = true;
    }

    if (updated)
    {
        LOG_I(TAG, "telem interval updated to %" PRIu32 " ms", intervalMs);
    }
    return updated;
}

// ── getStatusBits ─────────────────────────────────────────────────────────────

ares::proto::StatusBits MissionScriptEngine::getStatusBits() const
{
    ScopedLock guard(mutex_, pdMS_TO_TICKS(ares::AMS_MUTEX_TIMEOUT_MS));
    if (!guard.acquired())
    {
        ares::proto::StatusBits empty = {};
        return empty;
    }
    return buildStatusBitsLocked();
}

// ── notifyPyroFired ───────────────────────────────────────────────────────────

void MissionScriptEngine::notifyPyroFired(uint8_t channel)
{
    ScopedLock guard(mutex_, pdMS_TO_TICKS(ares::AMS_MUTEX_TIMEOUT_MS));
    if (!guard.acquired()) { return; }
    if (channel == 0U) { pyroAFired_ = true; }
    else if (channel == 1U) { pyroBFired_ = true; }
    else { /* invalid channel — ignore */ }
}

void MissionScriptEngine::tick(uint32_t nowMs)
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
    if (evaluateTransitionAndMaybeEnterLocked(state, nowMs)) { return; }

    if (state.hasFallback && state.fallbackTargetResolved)
    {
        const uint32_t elapsed = nowMs - stateEnterMs_;
        if (elapsed >= state.fallbackAfterMs)
        {
            LOG_I(TAG, "Fallback: '%s' -> '%s' (timeout %" PRIu32 "ms)",
                  state.name,
                  program_.states[state.fallbackTargetIdx].name,
                  elapsed);
            enterStateLocked(state.fallbackTargetIdx, nowMs);
            return;
        }
    }

    if (pendingTc_ == TcCommand::ABORT)
    {
        LOG_W(TAG, "ABORT TC not consumed by state=%s: force-deactivating",
              state.name);
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
    if (!state.hasTransition
        && !state.hasFallback
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
                                                                uint32_t nowMs)
{
    ARES_ASSERT(currentState_ < program_.stateCount);

    if (!state.hasTransition || state.transition.condCount == 0U)
    {
        return false;
    }

    const Transition& tr = state.transition;
    bool compound = (tr.logic == TransitionLogic::AND);
    bool tcPendingMatch = false;

    for (uint8_t i = 0; i < tr.condCount; i++)
    {
        const bool condResult = evaluateOneTransitionConditionLocked(
            tr.conds[i], state, i, nowMs, tcPendingMatch);
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
        transitionCondHolding_ = false;  // compound fell false — reset hold window
        return false;
    }

    if (!applyTransitionHoldLocked(tr, nowMs))
    {
        return false;
    }

    if (tcPendingMatch)
    {
        consumeMatchedTransitionTcLocked(tr);
        LOG_I(TAG, "TC condition matched in state=%s", state.name);
    }

    return fireResolvedTransitionLocked(state, tr, nowMs);
}

bool MissionScriptEngine::evaluateOneTransitionConditionLocked(const CondExpr& cond,
                                                               StateDef&,
                                                               uint8_t         condIdx,
                                                               uint32_t        nowMs,
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
        condResult = ((nowMs - stateEnterMs_) > static_cast<uint32_t>(cond.threshold));
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
                                                    uint32_t          nowMs)
{
    const bool isTcOnly = (tr.condCount == 1U && tr.conds[0].kind == CondKind::TC_EQ);
    if (tr.holdMs == 0U || isTcOnly)
    {
        return true;
    }

    if (!transitionCondHolding_)
    {
        transitionCondHolding_ = true;
        transitionCondMetMs_ = nowMs;
        return false;
    }

    if ((nowMs - transitionCondMetMs_) < tr.holdMs)
    {
        return false;
    }

    transitionCondHolding_ = false;
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
                                                       uint32_t          nowMs)
{
    if (!tr.targetResolved || tr.targetIndex >= program_.stateCount)
    {
        setErrorLocked("invalid transition target");
        return true;
    }

    LOG_I(TAG, "Transition: '%s' -> '%s' at t=%" PRIu32 "ms",
          state.name,
          program_.states[tr.targetIndex].name,
          nowMs);
    enterStateLocked(tr.targetIndex, nowMs);
    return true;
}

/**
 * @brief Execute the highest-priority due action(s) within the per-tick budget.
 *
 * On each call, up to `state.actionBudget` (clamped 1-3 by APUS-19.2) actions
 * are dispatched in priority order: EVENT > HK > LOG (configurable via
 * `priorities` in the AMS script).  Timers for dispatched actions are reset.
 *
 * @param[in] state  Current state definition (contains schedules and fields).
 * @param[in] nowMs  Current system time in milliseconds.
 * @pre  mutex_ is held by the caller.
 * @pre  currentState_ < program_.stateCount.
 */
void MissionScriptEngine::executeDueActionsLocked(const StateDef& state, uint32_t nowMs)
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

    // Legacy single-slot due flags (used by pickBestDueActionIndex).
    // A slot group is "due" if any slot within it is due.
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
                        hkSlotDue[s] = false;
                    }
                }
            }
            else
            {
                // Legacy fallback.
                sendHkReportLocked(nowMs);
                lastHkMs_ += state.hkEveryMs;
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
                        logSlotDue[s] = false;
                    }
                }
            }
            else
            {
                // Legacy fallback.
                appendLogReportLocked(nowMs);
                lastLogMs_ += state.logEveryMs;
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
    strncpy(out.activeFile, activeFile_, sizeof(out.activeFile) - 1U);
    out.activeFile[sizeof(out.activeFile) - 1U] = '\0';

    if (running_ && currentState_ < program_.stateCount)
    {
        strncpy(out.stateName, program_.states[currentState_].name,
                sizeof(out.stateName) - 1U);
        out.stateName[sizeof(out.stateName) - 1U] = '\0';
    }
    else
    {
        strncpy(out.stateName, "IDLE", sizeof(out.stateName) - 1U);
        out.stateName[sizeof(out.stateName) - 1U] = '\0';
    }

    strncpy(out.lastError, lastError_, sizeof(out.lastError) - 1U);
    out.lastError[sizeof(out.lastError) - 1U] = '\0';
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
    case SensorField::IMU_TEMP:  return "temp";
    default:                     return "?";
    }
}

bool MissionScriptEngine::evaluateGuardExprHoldsLocked(const CondExpr& expr,
                                                       const StateDef&,
                                                       uint32_t        nowMs,
                                                       float&          actualVal) const
{
    bool holds = true;
    switch (expr.kind)
    {
    case CondKind::TIME_GT:
        actualVal = static_cast<float>(nowMs - stateEnterMs_);
        holds = (static_cast<uint32_t>(actualVal)
                 <= static_cast<uint32_t>(expr.threshold));
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
                                                     uint32_t        nowMs)
{
    char valText[16] = {};
    char thrText[16] = {};
    if (!formatScaledFloat(actualVal, 3U, valText, sizeof(valText)))
    {
        strncpy(valText, "?", sizeof(valText) - 1U);
        valText[sizeof(valText) - 1U] = '\0';
    }
    if (!formatScaledFloat(expr.threshold, 3U, thrText, sizeof(thrText)))
    {
        strncpy(thrText, "?", sizeof(thrText) - 1U);
        thrText[sizeof(thrText) - 1U] = '\0';
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

    if (state.hasOnErrorTransition && state.onErrorTransitionResolved)
    {
        LOG_I(TAG,
              "Error recovery: '%s' -> '%s' (guard violated)",
              state.name,
              program_.states[state.onErrorTransitionIdx].name);
        enterStateLocked(state.onErrorTransitionIdx, nowMs);
        return true;
    }

    setErrorLocked("guard condition violated");
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
                                                   uint32_t nowMs)
{
    if (state.conditionCount == 0U)
    {
        return false;
    }

    for (uint8_t i = 0; i < state.conditionCount; i++)  // PO10-2: bounded
    {
        const CondExpr& expr = state.conditions[i].expr;
        float actualVal = 0.0f;
        const bool holds = evaluateGuardExprHoldsLocked(expr, state, nowMs, actualVal);
        if (!holds) { return handleGuardViolationLocked(state, expr, actualVal, nowMs); }
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

    strncpy(lastError_, reason, sizeof(lastError_) - 1U);
    lastError_[sizeof(lastError_) - 1U] = '\0';

    if (parseLineNum_ > 0U)
    {
        LOG_E(TAG, "Parse Error (line %" PRIu32 "): %s", parseLineNum_, lastError_);
        parseLineNum_ = 0U;  // consume parse context so runtime errors don't inherit it
    }
    else
    {
        LOG_E(TAG, "error: %s", lastError_);
    }
    clearResumePointLocked();
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
void MissionScriptEngine::enterStateLocked(uint8_t stateIndex, uint32_t nowMs)
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
    transitionCondHolding_ = false;  // reset hold window on every state entry (AMS-4.6.1)
    transitionCondMetMs_   = 0U;
    // AMS-4.11.2: reset TC CONFIRM counters on state entry (clean slate per state).
    memset(tcConfirmCount_, 0U, sizeof(tcConfirmCount_));
    // Reset delta prev-values so the first tick in the new state has no prior reading (AMS-4.6.2).
    for (uint8_t i = 0; i < ares::AMS_MAX_TRANSITION_CONDS; i++)
    {
        transitionPrevVal_[i]   = 0.0f;
        transitionPrevValid_[i] = false;
    }

    LOG_I(TAG, "state -> %s", program_.states[stateIndex].name);
    sendOnEnterEventLocked(nowMs);
    (void)saveResumePointLocked(nowMs, true);
}

bool MissionScriptEngine::buildCheckpointRecordLocked(uint32_t nowMs,
                                                      char*    record,
                                                      size_t   recSize,
                                                      int&     outWritten) const
{
    const uint32_t stateElapsed = nowMs - stateEnterMs_;
    const uint32_t hkElapsed    = nowMs - lastHkMs_;
    const uint32_t logElapsed   = nowMs - lastLogMs_;

    int written = snprintf(record, recSize,
                           "%" PRIu32 "|%s|%" PRIu32 "|%" PRIu32
                           "|%" PRIu32 "|%" PRIu32 "|%" PRIu32
                           "|%" PRIu32 "|%" PRIu32 "|%" PRIu32,
                           static_cast<uint32_t>(AMS_RESUME_VERSION),
                           activeFile_,
                           static_cast<uint32_t>(currentState_),
                           executionEnabled_ ? 1U : 0U,
                           running_ ? 1U : 0U,
                           static_cast<uint32_t>(status_),
                           static_cast<uint32_t>(seq_),
                           stateElapsed, hkElapsed, logElapsed);
    if (written <= 0 || static_cast<size_t>(written) >= recSize)
    {
        return false;
    }

    if (program_.varCount > 0U)
    {
        int w2 = snprintf(record + written, recSize - static_cast<size_t>(written),
                          "|%" PRIu32, static_cast<uint32_t>(program_.varCount));
        if (w2 > 0) { written += w2; }

        for (uint8_t vi = 0; vi < program_.varCount && written > 0; vi++)
        {
            const VarEntry& v = program_.vars[vi];
            char floatBuf[24] = {};
            (void)snprintf(floatBuf, sizeof(floatBuf), "%.6g",
                           static_cast<double>(v.value));
            const int w3 = snprintf(record + written,
                                    recSize - static_cast<size_t>(written),
                                    "|%s=%s=%u",
                                    v.name, floatBuf, v.valid ? 1U : 0U);
            if (w3 > 0 &&
                (static_cast<size_t>(written) + static_cast<size_t>(w3)) < recSize)
            {
                written += w3;
            }
        }
    }

    outWritten = written;
    return true;
}

bool MissionScriptEngine::saveResumePointLocked(uint32_t nowMs, bool force)
{
    if (!running_ || activeFile_[0] == '\0')
    {
        return false;
    }

    if (!force && ((nowMs - lastCheckpointMs_) < ares::AMS_CHECKPOINT_INTERVAL_MS))
    {
        return true;
    }

    if (currentState_ >= program_.stateCount)
    {
        return false;
    }

    // v2 format:
    //   VERSION|file|state|exec|running|status|seq|stateElap|hkElap|logElap
    //   |varCount|name1=value1=valid1|...|nameN=valueN=validN
    char record[400] = {};
    int written = 0;
    if (!buildCheckpointRecordLocked(nowMs, record, sizeof(record), written))
    {
        return false;
    }

    const StorageStatus st = storage_.writeFile(
        ares::AMS_RESUME_PATH,
        reinterpret_cast<const uint8_t*>(record),
        static_cast<uint32_t>(written));
    if (st != StorageStatus::OK)
    {
        LOG_W(TAG, "checkpoint write failed: %u", static_cast<uint32_t>(st));
        return false;
    }

    lastCheckpointMs_ = nowMs;
    return true;
}

void MissionScriptEngine::clearResumePointLocked()
{
    bool exists = false;
    const StorageStatus stExists = storage_.exists(ares::AMS_RESUME_PATH, exists);
    if (stExists != StorageStatus::OK || !exists)
    {
        return;
    }

    const StorageStatus stRemove = storage_.removeFile(ares::AMS_RESUME_PATH);
    if (stRemove != StorageStatus::OK && stRemove != StorageStatus::NOT_FOUND)
    {
        LOG_W(TAG, "checkpoint remove failed: %u", static_cast<uint32_t>(stRemove));
    }
}

bool MissionScriptEngine::parseCheckpointHeaderLocked(const char* buf,
                                                      uint32_t& version,
                                                      char*     fileName,
                                                      uint32_t& stateIdx,
                                                      uint32_t& execEnabled,
                                                      uint32_t& running,
                                                      uint32_t& status,
                                                      uint32_t& seq,
                                                      uint32_t& stateElapsed,
                                                      uint32_t& hkElapsed,
                                                      uint32_t& logElapsed) const
{
    const int parsed = sscanf(buf,  // NOLINT(bugprone-unchecked-string-to-number-conversion)
                              "%" SCNu32 "|%32[^|]|%" SCNu32 "|%" SCNu32
                              "|%" SCNu32 "|%" SCNu32 "|%" SCNu32
                              "|%" SCNu32 "|%" SCNu32 "|%" SCNu32,
                              &version, fileName,
                              &stateIdx, &execEnabled, &running,
                              &status, &seq,
                              &stateElapsed, &hkElapsed, &logElapsed);
    return parsed == 10;
}

void MissionScriptEngine::restoreCheckpointVarsLocked(const char* cursor)
{
    uint32_t vcStored = 0U;
    int vc = sscanf(cursor, "%" SCNu32, &vcStored);  // NOLINT(bugprone-unchecked-string-to-number-conversion)
    if (vc != 1 || vcStored == 0U) { return; }

    while (*cursor != '\0' && *cursor != '|') { cursor++; }

    for (uint32_t vi = 0; vi < vcStored && *cursor != '\0'; vi++)
    {
        if (*cursor == '|') { cursor++; }
        char vName[ares::AMS_VAR_NAME_LEN] = {};
        char vValStr[24] = {};
        uint32_t vValid  = 0U;
        // cppcheck-suppress [cert-err34-c]
        const int vp = sscanf(cursor, "%15[^=]=%23[^=]=%u",  // NOLINT(bugprone-unchecked-string-to-number-conversion)
                              vName, vValStr, &vValid);
        if (vp == 3)
        {
            VarEntry* v = findVarLocked(vName);
            if (v != nullptr)
            {
                float fval = 0.0f;
                if (parseFloatValue(vValStr, fval))
                {
                    v->value = fval;
                    v->valid = (vValid != 0U);
                }
            }
        }
        while (*cursor != '\0' && *cursor != '|') { cursor++; }
    }
}

bool MissionScriptEngine::applyCheckpointStateLocked(
    uint32_t nowMs, uint32_t stateIdx, uint32_t execEnabled,
    uint32_t running, uint32_t status, uint32_t seq,
    uint32_t stateElapsed, uint32_t hkElapsed, uint32_t logElapsed)
{
    currentState_ = static_cast<uint8_t>(stateIdx);
    stateEnterMs_ = nowMs - stateElapsed;
    lastHkMs_     = nowMs - hkElapsed;
    lastLogMs_    = nowMs - logElapsed;
    seq_          = static_cast<uint8_t>(seq & 0xFFU);

    running_          = (running != 0U);
    executionEnabled_ = (execEnabled != 0U);
    // CERT-1: validate enum range before cast to prevent undefined behaviour.
    if (status > static_cast<uint32_t>(EngineStatus::LAST))
    {
        LOG_W(TAG, "resume: invalid status field %" PRIu32 " — discarding checkpoint", status);
        clearResumePointLocked();
        return false;
    }
    status_ = static_cast<EngineStatus>(status);

    pendingTc_           = TcCommand::NONE;
    pendingOnEnterEvent_ = false;
    pendingEventText_[0] = '\0';
    pendingEventTsMs_    = 0U;
    lastError_[0]        = '\0';
    lastCheckpointMs_    = nowMs;

    if (!running_ || !executionEnabled_ || status_ != EngineStatus::RUNNING)
    {
        clearResumePointLocked();
        return false;
    }
    return true;
}

bool MissionScriptEngine::tryRestoreResumePointLocked(uint32_t nowMs)
{
    bool exists = false;
    const StorageStatus stExists = storage_.exists(ares::AMS_RESUME_PATH, exists);
    if (stExists != StorageStatus::OK || !exists)
    {
        return false;
    }

    char buf[400] = {};
    uint32_t bytesRead = 0;
    const StorageStatus stRead = storage_.readFile(
        ares::AMS_RESUME_PATH,
        reinterpret_cast<uint8_t*>(buf),
        sizeof(buf) - 1U,
        bytesRead);
    if (stRead != StorageStatus::OK || bytesRead == 0U)
    {
        clearResumePointLocked();
        return false;
    }
    buf[bytesRead] = '\0';

    uint32_t version = 0U;
    char fileName[ares::MISSION_FILENAME_MAX + 1U] = {};
    uint32_t stateIdx = 0U;
    uint32_t execEnabled = 0U;
    uint32_t running = 0U;
    uint32_t status = 0U;
    uint32_t seq = 0U;
    uint32_t stateElapsed = 0U;
    uint32_t hkElapsed = 0U;
    uint32_t logElapsed = 0U;

    const bool parsed = parseCheckpointHeaderLocked(buf, version, fileName,
                                                    stateIdx, execEnabled, running,
                                                    status, seq, stateElapsed,
                                                    hkElapsed, logElapsed);
    // Accept v1 (no vars) and v2 (with vars).  Reject everything else.
    if (!parsed
        || (version != 1U && version != static_cast<uint32_t>(AMS_RESUME_VERSION)))
    {
        clearResumePointLocked();
        return false;
    }

    if (!loadFromStorageLocked(fileName))
    {
        clearResumePointLocked();
        return false;
    }

    if (stateIdx >= program_.stateCount)
    {
        clearResumePointLocked();
        setErrorLocked("resume state out of range");
        return false;
    }

    if (!applyCheckpointStateLocked(nowMs, stateIdx, execEnabled, running, status, seq,
                                    stateElapsed, hkElapsed, logElapsed))
    {
        return false;
    }

    // ── v2: restore global variables ─────────────────────────────────
    if (version == static_cast<uint32_t>(AMS_RESUME_VERSION))
    {
        // Locate the 11th '|'-delimited token (index 10 = varCount field).
        const char* cursor = buf;
        uint8_t pipes = 0;
        while (*cursor != '\0' && pipes < 10U)
        {
            if (*cursor == '|') { pipes++; }
            cursor++;
        }
        if (*cursor != '\0')
        {
            restoreCheckpointVarsLocked(cursor);
        }
    }

    LOG_W(TAG, "resumed AMS from checkpoint: file=%s state=%s",
          activeFile_, program_.states[currentState_].name);

    // Persist immediately after restore to validate record freshness.
    (void)saveResumePointLocked(nowMs, true);
    return true;
}

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
                                                  uint32_t nowMs)
{
    for (uint8_t i = 0; i < st.setActionCount; i++)
    {
        executeOneSetActionLocked(st.setActions[i], nowMs);
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
                                                          uint32_t   /*nowMs*/)
{
    float sum      = 0.0f;
    uint8_t validN = 0;
    for (uint8_t s = 0; s < act.calibSamples; s++)
    {
        float sample = 0.0f;
        if (readSensorFloatLocked(act.alias, act.field, sample))
        {
            sum += sample;
            validN++;
        }
    }
    if (validN > 0U)
    {
        result     = sum / static_cast<float>(validN);
        gotReading = true;
        LOG_I(TAG, "CALIBRATE '%s': avg=%.3f (n=%u/%u)",
              act.varName, static_cast<double>(result),
              static_cast<unsigned>(validN),
              static_cast<unsigned>(act.calibSamples));
    }
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

void MissionScriptEngine::executeOneSetActionLocked(SetAction& act, uint32_t nowMs)
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
                                                     uint32_t        nowMs,
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
                     static_cast<uint32_t>(rule.cond.threshold);
        return true;
    default:
        // Delta and TC conditions are not supported in task if-rules.
        return true;
    }
}

void MissionScriptEngine::runTasksLocked(uint32_t nowMs)
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
void MissionScriptEngine::sendOnEnterEventLocked(uint32_t nowMs)
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

    if (!st.hasOnEnterEvent)
    {
        return;
    }

    pendingOnEnterEvent_ = true;
    pendingEventVerb_ = st.onEnterVerb;
    pendingEventTsMs_ = nowMs;
    strncpy(pendingEventText_, st.onEnterText, sizeof(pendingEventText_) - 1U);
    pendingEventText_[sizeof(pendingEventText_) - 1U] = '\0';
}

} // namespace ams
} // namespace ares

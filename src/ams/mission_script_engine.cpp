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
static constexpr uint8_t AMS_RESUME_VERSION = 1U;

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
    LOG_I(TAG, "TC command queued: %s", commandText);
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
    if (evaluateTransitionAndMaybeEnterLocked(state, nowMs))
    {
        return;
    }

    // AMS abort intercept: if ABORT TC is still pending after transition
    // evaluation (i.e. no transition in this state consumes it), the engine
    // must stop unconditionally.  Scripts that define an explicit ABORT
    // transition will consume the token inside evaluateTransitionAndMaybeEnterLocked,
    // so this path only fires when no ABORT transition is defined.
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

    (void)saveResumePointLocked(nowMs, false);

    // Terminal state detection: if the current state has no transition,
    // no pending on-enter event, and no periodic HK or LOG actions,
    // the mission is done. Mark COMPLETE so the log download unlocks.
    if (!state.hasTransition
        && !pendingOnEnterEvent_
        && !(state.hasHkEvery  && state.hkEveryMs  > 0U && state.hkFieldCount  > 0U)
        && !(state.hasLogEvery && state.logEveryMs > 0U && state.logFieldCount > 0U))
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
 * @brief Check the active state's transition condition and enter the target
 *        state if it holds.
 *
 * Reads the current time and sensor data, evaluates the guard expression, and
 * if the condition is satisfied calls enterStateLocked() on the target state.
 *
 * @param[in,out] state  Current state definition (read for transition metadata).
 * @param[in]     nowMs  Current system time in milliseconds.
 * @return @c true if a transition was triggered, @c false otherwise.
 * @pre  mutex_ is held by the caller.
 * @pre  currentState_ < program_.stateCount.
 */
bool MissionScriptEngine::evaluateTransitionAndMaybeEnterLocked(StateDef& state,
                                                                uint32_t nowMs)
{
    ARES_ASSERT(currentState_ < program_.stateCount);

    if (!state.hasTransition)
    {
        return false;
    }

    bool doTransition = false;
    const CondExpr& cond = state.transition.cond;

    switch (cond.kind)
    {
    case CondKind::TC_EQ:
        if (pendingTc_ == cond.tcValue)
        {
            LOG_I(TAG, "TC condition matched in state=%s", state.name);
            doTransition = true;
            pendingTc_ = TcCommand::NONE;
        }
        break;

    case CondKind::TIME_GT:
        doTransition = ((nowMs - stateEnterMs_) > static_cast<uint32_t>(cond.threshold));
        break;

    case CondKind::SENSOR_LT:
    case CondKind::SENSOR_GT:
    {
        float val = 0.0f;
        if (readSensorFloatLocked(cond.alias, cond.field, val))
        {
            doTransition = (cond.kind == CondKind::SENSOR_LT)
                         ? (val < cond.threshold)
                         : (val > cond.threshold);
            if (doTransition)
            {
                char thrText[16] = {};
                if (!formatScaledFloat(cond.threshold, 2U, thrText, sizeof(thrText)))
                {
                    strncpy(thrText, "nan", sizeof(thrText) - 1U);
                    thrText[sizeof(thrText) - 1U] = '\0';
                }
                LOG_I(TAG, "hw condition matched in state=%s thr=%s",
                      state.name, thrText);
            }
        }
        break;
    }

    default:
        break;
    }

    if (!doTransition)
    {
        return false;
    }

    if (!state.transition.targetResolved
        || state.transition.targetIndex >= program_.stateCount)
    {
        setErrorLocked("invalid transition target");
        return true;
    }

    LOG_I(TAG, "Transition: '%s' -> '%s' at t=%" PRIu32 "ms",
          state.name,
          program_.states[state.transition.targetIndex].name,
          nowMs);
    enterStateLocked(state.transition.targetIndex, nowMs);
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

    const bool hkDue = state.hasHkEvery
                    && state.hkEveryMs > 0U
                    && state.hkFieldCount > 0U
                    && ((nowMs - lastHkMs_) >= state.hkEveryMs);

    const bool logDue = state.hasLogEvery
                     && state.logEveryMs > 0U
                     && state.logFieldCount > 0U
                     && ((nowMs - lastLogMs_) >= state.logEveryMs);

    const bool eventDue = pendingOnEnterEvent_;

    bool due[3] = {eventDue, hkDue, logDue};
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
            sendEventLocked(pendingEventVerb_, pendingEventText_, pendingEventTsMs_);
            pendingOnEnterEvent_ = false;
            pendingEventText_[0] = '\0';
        }
        else if (best == 1)
        {
            sendHkReportLocked(nowMs);
            lastHkMs_ += state.hkEveryMs;
        }
        else
        {
            appendLogReportLocked(nowMs);
            lastLogMs_ += state.logEveryMs;
        }

        due[static_cast<uint8_t>(best)] = false;
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

/// Return a human-readable field name for use in guard violation log messages.
/// Defined as a static helper inside the member function to access the private
/// SensorField enum without changing the class interface.

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

    // Lambda: maps SensorField to a human-readable name for log messages.
    // Defined here to access the private SensorField enum without API changes.
    auto fieldName = [](SensorField f) -> const char*
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
    };

    for (uint8_t i = 0; i < state.conditionCount; i++)  // PO10-2: bounded
    {
        const CondExpr& expr = state.conditions[i].expr;
        bool holds = true;   // default: condition satisfied
        float actualVal = 0.0f;  // actual sensor / elapsed value (for logging)

        switch (expr.kind)
        {
        case CondKind::TIME_GT:
            // TIME.elapsed > N in a conditions: block is a watchdog.
            // The condition HOLDS (is safe) while elapsed <= threshold.
            // It FIRES (triggers on_error) once the deadline is exceeded.
            actualVal = static_cast<float>(nowMs - stateEnterMs_);
            holds = (static_cast<uint32_t>(actualVal)
                     <= static_cast<uint32_t>(expr.threshold));
            break;

        case CondKind::SENSOR_LT:
        case CondKind::SENSOR_GT:
            if (readSensorFloatLocked(expr.alias, expr.field, actualVal))
            {
                holds = (expr.kind == CondKind::SENSOR_LT)
                      ? (actualVal < expr.threshold)
                      : (actualVal > expr.threshold);
            }
            // Sensor not available: treat as "holds" (don't fire on_error
            // for missing optional sensors — AMS-5.6).
            break;

        default:
            break;
        }

        if (!holds)
        {
            // Emit structured guard violation log with condition text and value.
            char valText[16] = {};
            char thrText[16] = {};
            if (!formatScaledFloat(actualVal,      3U, valText, sizeof(valText)))
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
                      fieldName(expr.field), opStr, thrText, valText);
            }

            if (state.hasOnErrorEvent)
            {
                // Send immediately — setErrorLocked halts the engine next.
                sendEventLocked(state.onErrorVerb, state.onErrorText, nowMs);
            }
            setErrorLocked("guard condition violated");
            return true;
        }
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

    LOG_I(TAG, "state -> %s", program_.states[stateIndex].name);
    sendOnEnterEventLocked(nowMs);
    (void)saveResumePointLocked(nowMs, true);
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

    const uint32_t stateElapsed = nowMs - stateEnterMs_;
    const uint32_t hkElapsed = nowMs - lastHkMs_;
    const uint32_t logElapsed = nowMs - lastLogMs_;

    char record[192] = {};
    const int written = snprintf(record,
                                 sizeof(record),
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
                                 stateElapsed,
                                 hkElapsed,
                                 logElapsed);
    if (written <= 0 || static_cast<size_t>(written) >= sizeof(record))
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

bool MissionScriptEngine::tryRestoreResumePointLocked(uint32_t nowMs)
{
    bool exists = false;
    const StorageStatus stExists = storage_.exists(ares::AMS_RESUME_PATH, exists);
    if (stExists != StorageStatus::OK || !exists)
    {
        return false;
    }

    char buf[192] = {};
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

    const int parsed = sscanf(buf,
                              "%" SCNu32 "|%32[^|]|%" SCNu32 "|%" SCNu32
                              "|%" SCNu32 "|%" SCNu32 "|%" SCNu32
                              "|%" SCNu32 "|%" SCNu32 "|%" SCNu32,
                              &version,
                              fileName,
                              &stateIdx,
                              &execEnabled,
                              &running,
                              &status,
                              &seq,
                              &stateElapsed,
                              &hkElapsed,
                              &logElapsed);
    if (parsed != 10 || version != static_cast<uint32_t>(AMS_RESUME_VERSION))
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

    currentState_ = static_cast<uint8_t>(stateIdx);
    stateEnterMs_ = nowMs - stateElapsed;
    lastHkMs_ = nowMs - hkElapsed;
    lastLogMs_ = nowMs - logElapsed;
    seq_ = static_cast<uint8_t>(seq & 0xFFU);

    running_ = (running != 0U);
    executionEnabled_ = (execEnabled != 0U);
    // CERT-1: validate enum range before cast to prevent undefined behaviour.
    if (status > static_cast<uint32_t>(EngineStatus::LAST))
    {
        LOG_W(TAG, "resume: invalid status field %" PRIu32 " — discarding checkpoint", status);
        clearResumePointLocked();
        return false;
    }
    status_ = static_cast<EngineStatus>(status);

    pendingTc_ = TcCommand::NONE;
    pendingOnEnterEvent_ = false;
    pendingEventText_[0] = '\0';
    pendingEventTsMs_ = 0U;
    lastError_[0] = '\0';
    lastCheckpointMs_ = nowMs;

    if (!(running_ && executionEnabled_ && status_ == EngineStatus::RUNNING))
    {
        clearResumePointLocked();
        return false;
    }

    LOG_W(TAG, "resumed AMS from checkpoint: file=%s state=%s",
          activeFile_, program_.states[currentState_].name);

    // Persist immediately after restore to validate record freshness.
    (void)saveResumePointLocked(nowMs, true);
    return true;
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

    const StateDef& st = program_.states[currentState_];
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

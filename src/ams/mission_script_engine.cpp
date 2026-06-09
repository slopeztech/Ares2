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

using detail::formatScaledFloat;

static constexpr const char* TAG = "AMS";
MissionScriptEngine::MissionScriptEngine(StorageInterface&  storage,
                                         const GpsEntry*    gpsDrivers,  uint8_t gpsCount,
                                         const BaroEntry*   baroDrivers, uint8_t baroCount,
                                         const ComEntry*    comDrivers,  uint8_t comCount,
                                         const ImuEntry*    imuDrivers,  uint8_t imuCount,
                                         PulseInterface*    pulseIface,
                                         BuzzerInterface*   buzzerIface)
    : storage_(storage),
      gpsDrivers_(gpsDrivers),   gpsCount_(gpsCount),
      baroDrivers_(baroDrivers), baroCount_(baroCount),
      comDrivers_(comDrivers),   comCount_(comCount),
      imuDrivers_(imuDrivers),   imuCount_(imuCount),
      pulseIface_(pulseIface),
      buzzerIface_(buzzerIface)
{
}

static uint8_t sBeginCount = 0U;

MissionScriptEngine::~MissionScriptEngine()
{
    if (begun_)
    {
        sBeginCount--;
    }
}

bool MissionScriptEngine::begin()
{
    // Single-instance invariant: tryRestoreResumePointLocked uses a function-
    // local static buffer (static char buf[512]) that is shared across all
    // calls because it lives in .bss.  Two concurrent MissionScriptEngine
    // instances would race over that buffer during restore.  Enforce that
    // at most one engine may have begin() outstanding at any time.
    ARES_REQUIRE(sBeginCount == 0U);
    sBeginCount++;

    mutex_ = xSemaphoreCreateMutexStatic(&mutexBuf_);
    if (mutex_ == nullptr)
    {
        sBeginCount--;
        return false;
    }

    ScopedLock guard(mutex_, pdMS_TO_TICKS(ares::AMS_MUTEX_TIMEOUT_MS));
    if (!guard.acquired())
    {
        sBeginCount--;
        return false;
    }

    (void)tryRestoreResumePointLocked(millis64());
    begun_ = true;
    return true;
}

// ── validateAliasIfacesLocked ──────────────────────────────────────────────────
// Checks every alias registered by the script against the live driver
// registry.  Called from activate() after loadFromStorageLocked() so that
// setup() has already patched any late-bound interface pointers (e.g. the
// BMP280 BarometerInterface is nullptr at construction and patched in
// setup()).  Returns false and sets a descriptive error if any driver that
// a script alias references has iface == nullptr.
bool MissionScriptEngine::validateAliasIfacesLocked()
{
    for (uint8_t i = 0U; i < program_.aliasCount; i++)
    {
        const AliasEntry& ae = program_.aliases[i];
        bool ifaceNull = false;
        switch (ae.kind)
        {
        case PeripheralKind::GPS:
            ifaceNull = (ae.driverIdx < gpsCount_
                         && gpsDrivers_[ae.driverIdx].iface == nullptr);
            break;
        case PeripheralKind::BARO:
            ifaceNull = (ae.driverIdx < baroCount_
                         && baroDrivers_[ae.driverIdx].iface == nullptr);
            break;
        case PeripheralKind::COM:
            ifaceNull = (ae.driverIdx < comCount_
                         && comDrivers_[ae.driverIdx].iface == nullptr);
            break;
        case PeripheralKind::IMU:
            ifaceNull = (ae.driverIdx < imuCount_
                         && imuDrivers_[ae.driverIdx].iface == nullptr);
            break;
        default:
            break;
        }
        if (ifaceNull)
        {
            char msg[64] = {};
            snprintf(msg, sizeof(msg),
                     "driver for alias '%s' has no interface (iface=null)",
                     ae.alias);
            setErrorLocked(msg);
            return false;
        }
    }
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

    // check that every driver referenced by the script has a live
    // interface pointer.  This catches late-bound drivers whose iface is
    // patched after construction (e.g. BMP280 in setup()) but were not
    // ready in time, or misconfigured driver registries.
    if (!validateAliasIfacesLocked())
    {
        deactivateLocked();
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
    std::fill_n(lastHkSlotMs_,  ares::AMS_MAX_HK_SLOTS, uint64_t{0U});
    std::fill_n(lastLogSlotMs_, ares::AMS_MAX_HK_SLOTS, uint64_t{0U});

    uint8_t startIdx = 0;
    const uint8_t waitIdx = findStateByNameLocked("WAIT");
    if (waitIdx < program_.stateCount)
    {
        startIdx = waitIdx;
    }

    enterStateLocked(startIdx, millis64());
    LOG_I(TAG, "activated script=%s states=%u",
          activeFile_, static_cast<uint32_t>(program_.stateCount));
    return true;
}

void MissionScriptEngine::deactivateLocked()
{
    LOG_I(TAG, "deactivated");
    running_           = false;
    executionEnabled_  = false;
    checkpointDirty_   = false;
    status_            = EngineStatus::IDLE;
    program_ = {};
    primaryCom_ = nullptr;
    activeFile_[0] = '\0';
    logPath_[0] = '\0';
    lastError_[0] = '\0';
    pendingTc_ = TcCommand::NONE;
    pendingOnEnterEvent_ = false;
    pendingEventText_[0] = '\0';
    pendingEventTsMs_ = 0;
    for (uint8_t ti = 0U; ti < ares::AMS_MAX_TRANSITIONS; ti++)
    {
        transitionCondHolding_[ti] = false;
        transitionCondMetMs_[ti]   = 0U;
    }
    constexpr uint8_t kPrevSlots =
        static_cast<uint8_t>(ares::AMS_MAX_TRANSITIONS * ares::AMS_MAX_TRANSITION_CONDS);
    for (uint8_t pi = 0U; pi < kPrevSlots; pi++)
    {
        transitionPrevVal_[pi]   = 0.0f;
        transitionPrevValid_[pi] = false;
    }
    std::fill_n(taskLastTickMs_, ares::AMS_MAX_TASKS, uint64_t{0U});
    parseCurrentTask_     = 0xFFU;
    parseCurrentTaskRule_ = 0xFFU;

    resetSensorCachesLocked();
    resetTelemetryStateLocked();
    resetMonitorSlotsLocked();

    clearResumePointLocked();

    // AMS-4.19: reset pulse safety runtime state on every deactivation.
    for (uint8_t i = 0U; i < PulseChannel::COUNT; i++)
    {
        pulseArmed_[i]   = false;
        pulseArmedMs_[i] = 0U;
    }
    activationMs_ = 0U;

    // Reset per-channel fired status bits (Bug fix: these must clear between sessions
    // so that getStatusBits() does not report stale STATUS_PULSE_X_FIRED from a
    // previous mission script after deactivate + reactivate).
    pulseAFired_ = false;
    pulseBFired_ = false;
    pulseCFired_ = false;
    pulseDFired_ = false;
}

void MissionScriptEngine::resetSensorCachesLocked()
{
    std::fill(std::begin(baroCachedReadings_), std::end(baroCachedReadings_), BaroReading{});
    std::fill_n(baroCacheTsMs_,  ares::AMS_MAX_INCLUDES, uint64_t{0U});
    std::fill_n(baroCacheValid_, ares::AMS_MAX_INCLUDES, false);
    std::fill(std::begin(gpsCachedReadings_),  std::end(gpsCachedReadings_),  GpsReading{});
    std::fill_n(gpsCacheTsMs_,  ares::AMS_MAX_INCLUDES, uint64_t{0U});
    std::fill_n(gpsCacheValid_, ares::AMS_MAX_INCLUDES, false);
    std::fill(std::begin(imuCachedReadings_),  std::end(imuCachedReadings_),  ImuReading{});
    std::fill_n(imuCacheTsMs_,  ares::AMS_MAX_INCLUDES, uint64_t{0U});
    std::fill_n(imuCacheValid_, ares::AMS_MAX_INCLUDES, false);
}

void MissionScriptEngine::resetTelemetryStateLocked()
{
    hasPrevAlt_     = false;
    prevAltM_       = 0.0f;
    prevAltMs_      = 0U;
    deltaBaseValid_ = false;
    deltaBaseAlt_   = 0.0f;
    deltaBasePress_ = 0.0f;
    hkTxCount_      = 0U;
}

void MissionScriptEngine::resetMonitorSlotsLocked()
{
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
        // CONFIRM progress is material — write checkpoint soon so a reboot
        // preserves the accumulated count (AMS-8.5 v4 checkpoint).
        checkpointDirty_ = true;
    }
    LOG_I(TAG, "TC command queued: %s", commandText);
    return true;
}

bool MissionScriptEngine::arm()
{
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
        // AMS-4.19.5: start safe_delay timer.  Guard against millis64()==0 at
        // very early boot: if activationMs_ were 0 the safe_delay gate condition
        // (activationMs_ > 0U) would be false and the delay would be bypassed.
        //
        // Design note: activationMs_ is intentionally NOT included in the
        // checkpoint record (buildCheckpointRecordLocked does not serialise it)
        // and deactivateLocked() explicitly resets it to 0.  Consequently,
        // pulse.safe_delay is always measured from the current arm() call and
        // is NOT preserved across deactivation or power cycles.  Every mission
        // activation starts a fresh safe_delay window; this is the intended
        // semantics.
        const uint64_t nowMs = millis64();
        activationMs_ = (nowMs > 0U) ? nowMs : 1U;
        LOG_I(TAG, "arm: execution enabled, LAUNCH queued");

        // Pass the already-captured nowMs so the checkpoint elapsed-time fields
        // are consistent with activationMs_ (avoids a second millis64() call).
        (void)saveResumePointLocked(nowMs, true);
    } // Mutex released; checkpoint staged.

    flushPendingIoUnlocked(); // Write staged checkpoint outside the mutex (AMS-8.3).
    return true;
}

void MissionScriptEngine::setStateDirectiveCallback(StateDirectiveCallback callback)
{
    stateDirectiveCallback_ = callback;
}

void MissionScriptEngine::setExecutionEnabled(bool enabled)
{
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

            // Clear all per-transition hold windows so that a paused "for Nms"
            // window cannot fire spuriously on the first tick after re-enabling
            // (Bug #5: hold state must not accumulate during pause).
            for (uint8_t ti = 0U; ti < ares::AMS_MAX_TRANSITIONS; ti++)
            {
                transitionCondHolding_[ti] = false;
                transitionCondMetMs_[ti]   = 0U;
            }
        }
        LOG_I(TAG, "execution %s", enabled ? "enabled" : "disabled");

        (void)saveResumePointLocked(millis64(), true);
    } // Mutex released; checkpoint staged.

    flushPendingIoUnlocked(); // Write staged checkpoint outside the mutex (AMS-8.3).
}

bool MissionScriptEngine::requestTelemetry(uint64_t nowMs)
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

// ── notifyPulseFired ──────────────────────────────────────────────────────────

void MissionScriptEngine::notifyPulseFired(uint8_t channel)
{
    ScopedLock guard(mutex_, pdMS_TO_TICKS(ares::AMS_MUTEX_TIMEOUT_MS));
    if (!guard.acquired()) { return; }
    if (channel == 0U)      { pulseAFired_ = true; }
    else if (channel == 1U) { pulseBFired_ = true; }
    else if (channel == 2U) { pulseCFired_ = true; }
    else if (channel == 3U) { pulseDFired_ = true; }
    else { /* invalid channel — ignore */ }
}

// ── nextWakeupMs ─────────────────────────────────────────────────────────────

uint64_t MissionScriptEngine::nextWakeupMs(uint64_t nowMs) const
{
    // Fast-path: engine not running — keep full tick rate.
    if (!running_ || status_ != EngineStatus::RUNNING)
    {
        return nowMs + ares::SENSOR_RATE_MS;
    }

    // Pending on-enter event must be emitted promptly.
    if (pendingOnEnterEvent_)
    {
        return nowMs + ares::SENSOR_RATE_MS;
    }

    const StateDef& s = program_.states[currentState_];

    // Active transition or guard conditions require fresh sensor data every tick.
    if (s.transitionCount > 0U || s.conditionCount > 0U)
    {
        return nowMs + ares::SENSOR_RATE_MS;
    }

    // No conditions: compute the next time something actually needs to happen.
    // Start from the radio-cap so TC commands (LAUNCH/ABORT) are never delayed
    // more than kRadioMaxSleepMs even in pure reporting states.
    uint64_t next = nowMs + kRadioMaxSleepMs;

    next = nextDueFromHkSlots(s, next);
    next = nextDueFromLogSlots(s, next);
    next = nextDueFromTasks(next);

    if (s.hasOnTimeout)
    {
        const uint64_t due = stateEnterMs_ + s.onTimeoutMs;
        if (due < next) { next = due; }
    }
    if (s.hasFallback)
    {
        const uint64_t due = stateEnterMs_ + s.fallbackAfterMs;
        if (due < next) { next = due; }
    }

    return (next > nowMs) ? next : (nowMs + 1U);
}

uint64_t MissionScriptEngine::nextDueFromHkSlots(const StateDef& s, uint64_t cur) const
{
    for (uint8_t i = 0U; i < s.hkSlotCount; ++i)
    {
        if (s.hkSlots[i].everyMs > 0U)
        {
            const uint64_t due = lastHkSlotMs_[i] + s.hkSlots[i].everyMs;
            if (due < cur) { cur = due; }
        }
    }
    return cur;
}

uint64_t MissionScriptEngine::nextDueFromLogSlots(const StateDef& s, uint64_t cur) const
{
    for (uint8_t i = 0U; i < s.logSlotCount; ++i)
    {
        if (s.logSlots[i].everyMs > 0U)
        {
            const uint64_t due = lastLogSlotMs_[i] + s.logSlots[i].everyMs;
            if (due < cur) { cur = due; }
        }
    }
    return cur;
}

uint64_t MissionScriptEngine::nextDueFromTasks(uint64_t cur) const
{
    for (uint8_t t = 0U; t < program_.taskCount; ++t)
    {
        const TaskDef& td = program_.tasks[t];
        if (td.everyMs == 0U) { continue; }
        bool active = true;
        if (td.hasStateFilter && td.stateIndicesResolved)
        {
            active = false;
            for (uint8_t si = 0U; si < td.activeStateCount; ++si)
            {
                if (td.activeStateIndices[si] == currentState_) { active = true; break; }
            }
        }
        if (active)
        {
            const uint64_t due = taskLastTickMs_[t] + td.everyMs;
            if (due < cur) { cur = due; }
        }
    }
    return cur;
}


} // namespace ams
} // namespace ares
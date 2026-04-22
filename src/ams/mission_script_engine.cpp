/**
 * @file  mission_script_engine.cpp
 * @brief AMS parser + runtime implementation.
 */

#include "ams/mission_script_engine.h"

#include "api/api_common.h"
#include "ares_assert.h"
#include "debug/ares_log.h"
#include "hal/imu/imu_interface.h"

#include <Arduino.h>
#include <cmath>
#include <cstring>
#include <cstdlib>
#include <cstdio>
#include <cinttypes>

namespace ares
{
namespace ams
{

using ares::proto::EventHeader;
using ares::proto::EventId;
using ares::proto::EventSeverity;
using ares::proto::Frame;
using ares::proto::MAX_FRAME_LEN;
using ares::proto::MAX_PAYLOAD_LEN;
using ares::proto::MsgType;
using ares::proto::NODE_BROADCAST;
using ares::proto::NODE_GROUND;
using ares::proto::NODE_PAYLOAD;
using ares::proto::NODE_ROCKET;
using ares::proto::PROTOCOL_VERSION;
using ares::proto::TelemetryPayload;

static constexpr const char* TAG = "AMS";

static uint8_t clampActionBudget(uint8_t budget)
{
    if (budget == 0U)
    {
        return 1U;
    }
    if (budget > 3U)
    {
        return 3U;
    }
    return budget;
}

static int8_t pickBestDueActionIndex(const uint8_t priorities[3],
                                     const bool due[3])
{
    int8_t best = -1;
    uint8_t bestPriority = 0U;

    for (uint8_t i = 0; i < 3U; i++)
    {
        if (!due[i])
        {
            continue;
        }
        if (best < 0 || priorities[i] > bestPriority)
        {
            best = static_cast<int8_t>(i);
            bestPriority = priorities[i];
        }
    }

    return best;
}

static bool isOnlyTrailingWhitespace(const char* text)
{
    if (text == nullptr)
    {
        return false;
    }

    while (*text == ' ' || *text == '\t')
    {
        text++;
    }

    return *text == '\0';
}

static uint32_t pow10u(uint8_t digits)
{
    uint32_t value = 1U;
    for (uint8_t i = 0U; i < digits; i++)
    {
        value *= 10U;
    }
    return value;
}

static bool formatScaledFloat(float value,
                              uint8_t decimals,
                              char* out,
                              uint32_t outSize)
{
    if (out == nullptr || outSize == 0U)
    {
        return false;
    }

    const uint32_t scale = pow10u(decimals);
    const bool neg = (value < 0.0f);
    const float absValue = neg ? -value : value;
    const float scaledF = (absValue * static_cast<float>(scale)) + 0.5f;
    const uint64_t scaled = static_cast<uint64_t>(scaledF);
    const uint32_t whole = static_cast<uint32_t>(scaled / scale);
    const uint32_t frac = static_cast<uint32_t>(scaled % scale);

    int written = 0;
    if (decimals == 0U)
    {
        written = snprintf(out, outSize, "%s%" PRIu32,
                           neg ? "-" : "", whole);
    }
    else
    {
        written = snprintf(out, outSize, "%s%" PRIu32 ".%0*" PRIu32,
                           neg ? "-" : "", whole,
                           static_cast<int>(decimals), frac);
    }

    return (written > 0)
        && (static_cast<uint32_t>(written) < outSize);
}

MissionScriptEngine::MissionScriptEngine(StorageInterface& storage,
                                         GpsInterface& gps,
                                         BarometerInterface& baro,
                                         RadioInterface& radio,
                                         ImuInterface* imu)
    : storage_(storage), gps_(gps), baro_(baro), radio_(radio), imu_(imu)
{
}

bool MissionScriptEngine::begin()
{
    mutex_ = xSemaphoreCreateMutexStatic(&mutexBuf_);
    ARES_ASSERT(mutex_ != nullptr);
    return mutex_ != nullptr;
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

    if (!ensureLogFileLocked(activeFile_))
    {
        setErrorLocked("failed to create mission log file");
        return false;
    }

    status_ = EngineStatus::RUNNING;
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

void MissionScriptEngine::deactivate()
{
    ScopedLock guard(mutex_, pdMS_TO_TICKS(ares::AMS_MUTEX_TIMEOUT_MS));
    if (!guard.acquired())
    {
        return;
    }

    LOG_I(TAG, "deactivated");
    running_ = false;
    executionEnabled_ = false;
    status_ = EngineStatus::IDLE;
    program_ = {};
    activeFile_[0] = '\0';
    logPath_[0] = '\0';
    lastError_[0] = '\0';
    pendingTc_ = TcCommand::NONE;
    pendingOnEnterEvent_ = false;
    pendingEventText_[0] = '\0';
    pendingEventTsMs_ = 0;
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
    LOG_I(TAG, "execution %s", enabled ? "enabled" : "disabled");
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

    // AMS-5.2: Evaluate guard conditions after transition, before actions.
    // If a condition is violated, on_error fires and engine halts.
    if (evaluateConditionsLocked(state, nowMs))
    {
        return;
    }

    executeDueActionsLocked(state, nowMs);
}

bool MissionScriptEngine::evaluateTransitionAndMaybeEnterLocked(StateDef& state,
                                                                uint32_t nowMs)
{
    ARES_ASSERT(currentState_ < program_.stateCount);

    if (!state.hasTransition)
    {
        return false;
    }

    bool doTransition = false;

    if (state.transition.type == ConditionType::TC_COMMAND_EQ)
    {
        if (pendingTc_ == state.transition.tcValue)
        {
            LOG_I(TAG, "TC condition matched in state=%s", state.name);
            doTransition = true;
            pendingTc_ = TcCommand::NONE;
        }
    }
    else
    {
        // Hardware-sensor conditions: read sensors once per tick.
        BaroReading b = {};
        GpsReading  g = {};
        const bool baroOk = (baro_.read(b) == BaroStatus::OK);
        const bool gpsOk  = (gps_.read(g) == GpsStatus::OK);
        const float thr   = state.transition.threshold;

        switch (state.transition.type)
        {
        case ConditionType::BARO_ALT_LT:
            if (baroOk) { doTransition = (b.altitudeM < thr); }
            break;
        case ConditionType::BARO_ALT_GT:
            if (baroOk) { doTransition = (b.altitudeM > thr); }
            break;
        case ConditionType::BARO_TEMP_LT:
            if (baroOk) { doTransition = (b.temperatureC < thr); }
            break;
        case ConditionType::BARO_TEMP_GT:
            if (baroOk) { doTransition = (b.temperatureC > thr); }
            break;
        case ConditionType::BARO_PRESS_LT:
            if (baroOk) { doTransition = (b.pressurePa < thr); }
            break;
        case ConditionType::BARO_PRESS_GT:
            if (baroOk) { doTransition = (b.pressurePa > thr); }
            break;
        case ConditionType::GPS_ALT_LT:
            if (gpsOk) { doTransition = (g.altitudeM < thr); }
            break;
        case ConditionType::GPS_ALT_GT:
            if (gpsOk) { doTransition = (g.altitudeM > thr); }
            break;
        case ConditionType::GPS_SPEED_GT:
            if (gpsOk) { doTransition = (g.speedKmh > thr); }
            break;
        case ConditionType::GPS_SPEED_LT:
            if (gpsOk) { doTransition = (g.speedKmh < thr); }
            break;
        // Sensor-free: elapsed ms since state entry (APUS-13.4)
        case ConditionType::TIME_ELAPSED_GT:
            doTransition = ((nowMs - stateEnterMs_) > static_cast<uint32_t>(thr));
            break;
        // IMU accelerometer conditions
        case ConditionType::IMU_ACCEL_X_LT:
        case ConditionType::IMU_ACCEL_X_GT:
        case ConditionType::IMU_ACCEL_Y_LT:
        case ConditionType::IMU_ACCEL_Y_GT:
        case ConditionType::IMU_ACCEL_Z_LT:
        case ConditionType::IMU_ACCEL_Z_GT:
        case ConditionType::IMU_ACCEL_MAG_GT:
        case ConditionType::IMU_ACCEL_MAG_LT:
        {
            if (imu_ != nullptr)
            {
                ImuReading imuR = {};
                const bool imuOk = (imu_->read(imuR) == ImuStatus::OK);
                if (imuOk)
                {
                    switch (state.transition.type)
                    {
                    case ConditionType::IMU_ACCEL_X_LT:
                        doTransition = (imuR.accelX < thr); break;
                    case ConditionType::IMU_ACCEL_X_GT:
                        doTransition = (imuR.accelX > thr); break;
                    case ConditionType::IMU_ACCEL_Y_LT:
                        doTransition = (imuR.accelY < thr); break;
                    case ConditionType::IMU_ACCEL_Y_GT:
                        doTransition = (imuR.accelY > thr); break;
                    case ConditionType::IMU_ACCEL_Z_LT:
                        doTransition = (imuR.accelZ < thr); break;
                    case ConditionType::IMU_ACCEL_Z_GT:
                        doTransition = (imuR.accelZ > thr); break;
                    case ConditionType::IMU_ACCEL_MAG_GT:
                    {
                        const float mag = sqrtf(imuR.accelX * imuR.accelX
                                              + imuR.accelY * imuR.accelY
                                              + imuR.accelZ * imuR.accelZ);
                        doTransition = (mag > thr);
                        break;
                    }
                    case ConditionType::IMU_ACCEL_MAG_LT:
                    {
                        const float mag = sqrtf(imuR.accelX * imuR.accelX
                                              + imuR.accelY * imuR.accelY
                                              + imuR.accelZ * imuR.accelZ);
                        doTransition = (mag < thr);
                        break;
                    }
                    default:
                        break;
                    }
                }
            }
            break;
        }
        default:
            break;
        }

        if (doTransition)
        {
            char thrText[16] = {};
            if (!formatScaledFloat(thr, 2U, thrText, sizeof(thrText)))
            {
                strncpy(thrText, "nan", sizeof(thrText) - 1U);
                thrText[sizeof(thrText) - 1U] = '\0';
            }
            LOG_I(TAG, "hw condition matched in state=%s thr=%s",
                  state.name, thrText);
        }
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

    enterStateLocked(state.transition.targetIndex, nowMs);
    return true;
}

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
            lastHkMs_ = nowMs;
        }
        else
        {
            appendLogReportLocked(nowMs);
            lastLogMs_ = nowMs;
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

bool MissionScriptEngine::loadFromStorageLocked(const char* fileName)
{
    ARES_ASSERT(mutex_ != nullptr);

    if (!isSafeFileName(fileName))
    {
        setErrorLocked("invalid script filename");
        return false;
    }

    char fullPath[ares::STORAGE_MAX_PATH] = {};
    if (!buildMissionPath(fileName, fullPath, sizeof(fullPath)))
    {
        setErrorLocked("script path too long");
        return false;
    }

    bool exists = false;
    const StorageStatus stExists = storage_.exists(fullPath, exists);
    if (stExists != StorageStatus::OK || !exists)
    {
        setErrorLocked("script file not found");
        return false;
    }

    uint32_t bytesRead = 0;
    const StorageStatus stRead = storage_.readFile(
        fullPath,
        reinterpret_cast<uint8_t*>(scriptBuffer_),
        ares::AMS_MAX_SCRIPT_BYTES,
        bytesRead);

    if (stRead != StorageStatus::OK)
    {
        setErrorLocked("failed to read script file");
        return false;
    }

    if (bytesRead == 0U)
    {
        setErrorLocked("empty script file");
        return false;
    }

    scriptBuffer_[bytesRead] = '\0';

    if (!parseScriptLocked(scriptBuffer_, bytesRead))
    {
        return false;
    }

    strncpy(activeFile_, fileName, sizeof(activeFile_) - 1U);
    activeFile_[sizeof(activeFile_) - 1U] = '\0';

    if (!ensureLogFileLocked(activeFile_))
    {
        setErrorLocked("failed to create mission log file");
        return false;
    }

    lastError_[0] = '\0';
    return true;
}

bool MissionScriptEngine::parseScriptLocked(const char* script, uint32_t length)
{
    ARES_ASSERT(script != nullptr);
    ARES_ASSERT(length <= ares::AMS_MAX_SCRIPT_BYTES);

    program_ = {};
    program_.apid = ares::AMS_DEFAULT_APID;
    program_.nodeId = ares::DEFAULT_NODE_ID;

    BlockType blockType = BlockType::NONE;
    uint8_t currentState = ares::AMS_MAX_STATES;

    uint32_t offset = 0;
    while (offset < length)
    {
        char line[ares::AMS_MAX_LINE_LEN] = {};
        if (!readNextScriptLineLocked(script, length, offset,
                                      line, sizeof(line)))
        {
            return false;
        }
        trimInPlace(line);

        if (!parseLineLocked(line, currentState, blockType))
        {
            return false;
        }
    }

    if (program_.stateCount == 0U)
    {
        setErrorLocked("script has no states");
        return false;
    }

    if (!resolveTransitionsLocked())
    {
        return false;
    }

    return true;
}

bool MissionScriptEngine::readNextScriptLineLocked(const char* script,
                                                   uint32_t length,
                                                   uint32_t& offset,
                                                   char* line,
                                                   uint32_t lineSize)
{
    ARES_ASSERT(script != nullptr);
    ARES_ASSERT(line != nullptr);
    ARES_ASSERT(lineSize > 1U);

    uint32_t lineLen = 0;
    bool lineTooLong = false;

    while (offset < length && script[offset] != '\n')
    {
        if (lineLen < (lineSize - 1U))
        {
            line[lineLen] = script[offset];
            lineLen++;
        }
        else
        {
            lineTooLong = true;
        }
        offset++;
    }

    if (offset < length && script[offset] == '\n')
    {
        offset++;
    }

    if (lineTooLong)
    {
        setErrorLocked("script line exceeds AMS_MAX_LINE_LEN");
        return false;
    }

    line[lineLen] = '\0';
    return true;
}

bool MissionScriptEngine::parseLineLocked(const char* line,
                                          uint8_t& currentState,
                                          BlockType& blockType)
{
    ARES_ASSERT(line != nullptr);

    if (line[0] == '\0' || startsWith(line, "//"))
    {
        return true;
    }

    if (strcmp(line, "}") == 0)
    {
        blockType = BlockType::NONE;
        return true;
    }

    if (startsWith(line, "include ") || startsWith(line, "pus.service "))
    {
        return true;
    }

    if (startsWith(line, "pus.apid"))
    {
        const char* eq = strchr(line, '=');
        if (eq == nullptr)
        {
            setErrorLocked("invalid pus.apid syntax");
            return false;
        }

        uint32_t apid = 0;
        if (!parseUint(eq + 1, apid) || apid > 2047U)
        {
            setErrorLocked("invalid pus.apid value");
            return false;
        }

        uint8_t nodeId = 0;
        if (!mapApidToNode(static_cast<uint16_t>(apid), nodeId))
        {
            setErrorLocked("unsupported APID for ARES node mapping");
            return false;
        }

        program_.apid = static_cast<uint16_t>(apid);
        program_.nodeId = nodeId;
        return true;
    }

    if (startsWith(line, "state "))
    {
        blockType = BlockType::NONE;
        return parseStateLineLocked(line, currentState);
    }

    if (currentState >= program_.stateCount)
    {
        setErrorLocked("statement outside state block");
        return false;
    }

    StateDef& st = program_.states[currentState];

    return parseStateScopedLineLocked(line, st, blockType);
}

bool MissionScriptEngine::parseStateScopedLineLocked(const char* line,
                                                     StateDef& st,
                                                     BlockType& blockType)
{
    ARES_ASSERT(line != nullptr);
    ARES_ASSERT(st.hkFieldCount <= ares::AMS_MAX_HK_FIELDS);
    ARES_ASSERT(st.logFieldCount <= ares::AMS_MAX_HK_FIELDS);

    // ── Block headers always take priority — they reset context ────────
    if (startsWith(line, "on_enter:"))
    {
        blockType = BlockType::NONE;
        return true;
    }
    if (startsWith(line, "on_error:"))
    {
        blockType = BlockType::ON_ERROR;
        return true;
    }
    if (startsWith(line, "conditions:"))
    {
        blockType = BlockType::CONDITIONS;
        return true;
    }
    if (startsWith(line, "every "))
    {
        blockType = BlockType::NONE;
        return parseEveryLineLocked(line, st);
    }
    if (startsWith(line, "log_every "))
    {
        blockType = BlockType::NONE;
        return parseLogEveryLineLocked(line, st);
    }
    if (startsWith(line, "priorities "))
    {
        blockType = BlockType::NONE;
        return parsePrioritiesLineLocked(line, st);
    }
    if (startsWith(line, "HK.report"))
    {
        if (!st.hasHkEvery)
        {
            setErrorLocked("HK.report requires every block");
            return false;
        }
        blockType = BlockType::HK;
        return true;
    }
    if (startsWith(line, "LOG.report"))
    {
        if (!st.hasLogEvery)
        {
            setErrorLocked("LOG.report requires log_every block");
            return false;
        }
        blockType = BlockType::LOG;
        return true;
    }
    if (startsWith(line, "transition to "))
    {
        blockType = BlockType::NONE;
        return parseTransitionLineLocked(line, st);
    }

    // ── Block content — dispatched by current blockType ─────────────────
    if (blockType == BlockType::CONDITIONS)
    {
        return parseConditionScopedLineLocked(line, st);
    }
    if (blockType == BlockType::ON_ERROR)
    {
        if (startsWith(line, "EVENT."))
        {
            return parseOnErrorEventLineLocked(line, st);
        }
        setErrorLocked("only EVENT.* is allowed inside on_error");
        return false;
    }
    if (blockType == BlockType::HK)
    {
        return parseFieldLineLocked(line, st.hkFields, st.hkFieldCount, "HK");
    }
    if (blockType == BlockType::LOG)
    {
        return parseFieldLineLocked(line, st.logFields, st.logFieldCount, "LOG");
    }

    // ── NONE context: EVENT.* belongs to on_enter ───────────────────────
    if (startsWith(line, "EVENT."))
    {
        return parseEventLineLocked(line, st);
    }

    setErrorLocked("unsupported statement");
    return false;
}

bool MissionScriptEngine::parseStateLineLocked(const char* line,
                                               uint8_t& currentState)
{
    if (program_.stateCount >= ares::AMS_MAX_STATES)
    {
        setErrorLocked("too many states");
        return false;
    }

    char stateName[ares::AMS_MAX_STATE_NAME] = {};
    const int n = sscanf(line, "state %15[^:]:", stateName);
    if (n != 1)
    {
        setErrorLocked("invalid state syntax");
        return false;
    }

    currentState = program_.stateCount;
    StateDef& st = program_.states[currentState];
    strncpy(st.name, stateName, sizeof(st.name) - 1U);
    st.name[sizeof(st.name) - 1U] = '\0';
    program_.stateCount++;
    return true;
}

bool MissionScriptEngine::parseEventLineLocked(const char* line, StateDef& st)
{
    char verb[8] = {};
    char text[ares::AMS_MAX_EVENT_TEXT] = {};
    const int n = sscanf(line, "EVENT.%7[^ ] \"%63[^\"]\"", verb, text);
    if (n != 2)
    {
        setErrorLocked("invalid EVENT syntax");
        return false;
    }

    st.hasOnEnterEvent = true;
    strncpy(st.onEnterText, text, sizeof(st.onEnterText) - 1U);
    st.onEnterText[sizeof(st.onEnterText) - 1U] = '\0';

    if (strcmp(verb, "info") == 0)
    {
        st.onEnterVerb = EventVerb::INFO;
        return true;
    }
    if (strcmp(verb, "warning") == 0)
    {
        st.onEnterVerb = EventVerb::WARN;
        return true;
    }
    if (strcmp(verb, "error") == 0)
    {
        st.onEnterVerb = EventVerb::ERROR;
        return true;
    }

    setErrorLocked("unknown EVENT verb");
    return false;
}

bool MissionScriptEngine::parseEveryLineLocked(const char* line, StateDef& st)
{
    const char* prefix = "every ";
    const size_t prefixLen = strlen(prefix);
    if (strncmp(line, prefix, prefixLen) != 0)
    {
        setErrorLocked("invalid every syntax");
        return false;
    }

    const char* valueStart = line + prefixLen;
    while (*valueStart == ' ' || *valueStart == '\t')
    {
        valueStart++;
    }

    const char* marker = strstr(valueStart, "ms:");
    if (marker == nullptr)
    {
        setErrorLocked("invalid every period");
        return false;
    }

    if (!isOnlyTrailingWhitespace(marker + 3))
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

    st.hasHkEvery = true;
    st.hkEveryMs = everyMs;
    st.hkFieldCount = 0;
    return true;
}

bool MissionScriptEngine::parseLogEveryLineLocked(const char* line, StateDef& st)
{
    const char* prefix = "log_every ";
    const size_t prefixLen = strlen(prefix);
    if (strncmp(line, prefix, prefixLen) != 0)
    {
        setErrorLocked("invalid log_every syntax");
        return false;
    }

    const char* valueStart = line + prefixLen;
    while (*valueStart == ' ' || *valueStart == '\t')
    {
        valueStart++;
    }

    const char* marker = strstr(valueStart, "ms:");
    if (marker == nullptr)
    {
        setErrorLocked("invalid log_every period");
        return false;
    }

    if (!isOnlyTrailingWhitespace(marker + 3))
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

    st.hasLogEvery = true;
    st.logEveryMs = everyMs;
    st.logFieldCount = 0;
    return true;
}

bool MissionScriptEngine::parsePrioritiesLineLocked(const char* line,
                                                    StateDef& st)
{
    ARES_ASSERT(line != nullptr);

    uint32_t event = st.eventPriority;
    uint32_t hk = 0U;
    uint32_t log = 0U;
    uint32_t budget = st.actionBudget;

    if (!parsePrioritiesValuesLocked(line, event, hk, log, budget, st)
        || event > 9U || hk > 9U || log > 9U
        || budget < 1U || budget > 3U)
    {
        setErrorLocked("invalid priorities syntax");
        return false;
    }

    st.eventPriority = static_cast<uint8_t>(event);
    st.hkPriority = static_cast<uint8_t>(hk);
    st.logPriority = static_cast<uint8_t>(log);
    st.actionBudget = static_cast<uint8_t>(budget);
    return true;
}

bool MissionScriptEngine::parsePrioritiesValuesLocked(const char* line,
                                                      uint32_t& event,
                                                      uint32_t& hk,
                                                      uint32_t& log,
                                                      uint32_t& budget,
                                                      const StateDef& st)
{
    int n = sscanf(line,
                   "priorities event=%" SCNu32 " hk=%" SCNu32
                   " log=%" SCNu32 " budget=%" SCNu32,
                   &event, &hk, &log, &budget);
    if (n == 4)
    {
        return true;
    }

    n = sscanf(line,
               "priorities event=%" SCNu32 " hk=%" SCNu32
               " log=%" SCNu32,
               &event, &hk, &log);
    if (n == 3)
    {
        budget = st.actionBudget;
        return true;
    }

    n = sscanf(line,
               "priorities hk=%" SCNu32 " log=%" SCNu32
               " budget=%" SCNu32,
               &hk, &log, &budget);
    if (n == 3)
    {
        event = st.eventPriority;
        return true;
    }

    n = sscanf(line,
               "priorities hk=%" SCNu32 " log=%" SCNu32,
               &hk, &log);
    if (n == 2)
    {
        event = st.eventPriority;
        budget = st.actionBudget;
        return true;
    }

    return false;
}

bool MissionScriptEngine::parseFieldLineLocked(const char* line,
                                               HkField* fields,
                                               uint8_t& count,
                                               const char* ctxName)
{
    if (strcmp(line, "{") == 0)
    {
        return true;
    }

    char key[20] = {};
    char expr[20] = {};
    const int n = sscanf(line, "%19[^:]: %19s", key, expr);
    if (n != 2)
    {
        char msg[48] = {};
        snprintf(msg, sizeof(msg), "invalid %s field syntax", ctxName);
        setErrorLocked(msg);
        return false;
    }

    ExprKind kind = ExprKind::GPS_ALT;
    if (!parseExprKind(expr, kind))
    {
        char msg[56] = {};
        snprintf(msg, sizeof(msg), "unsupported %s expression", ctxName);
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

    fields[count].expr = kind;
    count++;
    return true;
}

bool MissionScriptEngine::parseTransitionLineLocked(const char* line,
                                                    StateDef& st)
{
    char target[ares::AMS_MAX_STATE_NAME] = {};
    char lhs[20] = {};
    char op[3] = {};
    char rhs[20] = {};

    const int n = sscanf(line, "transition to %15s when %19s %2s %19s",
                         target, lhs, op, rhs);
    if (n != 4)
    {
        setErrorLocked("invalid transition syntax");
        return false;
    }

    st.hasTransition = true;
    strncpy(st.transition.targetName, target,
            sizeof(st.transition.targetName) - 1U);
    st.transition.targetName[sizeof(st.transition.targetName) - 1U] = '\0';
    st.transition.targetResolved = false;

    if (strcmp(lhs, "TC.command") == 0 && strcmp(op, "==") == 0)
    {
        TcCommand cmd = TcCommand::NONE;
        if (!parseTcCommand(rhs, cmd) || cmd == TcCommand::NONE)
        {
            setErrorLocked("invalid TC.command transition");
            return false;
        }
        st.transition.type = ConditionType::TC_COMMAND_EQ;
        st.transition.tcValue = cmd;
        return true;
    }

    // ── Hardware sensor conditions ──────────────────────────────
    struct SensorCond
    {
        const char* lhsStr;
        const char* opStr;
        ConditionType ctype;
    };

    static const SensorCond kConds[] = {
        { "BARO.alt",      "<", ConditionType::BARO_ALT_LT    },
        { "BARO.alt",      ">", ConditionType::BARO_ALT_GT    },
        { "BARO.temp",     "<", ConditionType::BARO_TEMP_LT   },
        { "BARO.temp",     ">", ConditionType::BARO_TEMP_GT   },
        { "BARO.pressure", "<", ConditionType::BARO_PRESS_LT  },
        { "BARO.pressure", ">", ConditionType::BARO_PRESS_GT  },
        { "GPS.alt",       "<", ConditionType::GPS_ALT_LT     },
        { "GPS.alt",       ">", ConditionType::GPS_ALT_GT     },
        { "GPS.speed",     ">", ConditionType::GPS_SPEED_GT   },
        { "GPS.speed",     "<", ConditionType::GPS_SPEED_LT   },
        // Sensor-free time condition (APUS-13.4)
        { "TIME.elapsed",  ">", ConditionType::TIME_ELAPSED_GT },
        // IMU accelerometer conditions
        { "IMU.accel_x",   "<", ConditionType::IMU_ACCEL_X_LT   },
        { "IMU.accel_x",   ">", ConditionType::IMU_ACCEL_X_GT   },
        { "IMU.accel_y",   "<", ConditionType::IMU_ACCEL_Y_LT   },
        { "IMU.accel_y",   ">", ConditionType::IMU_ACCEL_Y_GT   },
        { "IMU.accel_z",   "<", ConditionType::IMU_ACCEL_Z_LT   },
        { "IMU.accel_z",   ">", ConditionType::IMU_ACCEL_Z_GT   },
        { "IMU.accel_mag", ">", ConditionType::IMU_ACCEL_MAG_GT },
        { "IMU.accel_mag", "<", ConditionType::IMU_ACCEL_MAG_LT },
    };

    for (uint8_t i = 0; i < sizeof(kConds) / sizeof(kConds[0]); i++)
    {
        if (strcmp(lhs, kConds[i].lhsStr) == 0
            && strcmp(op, kConds[i].opStr) == 0)
        {
            float threshold = 0.0f;
            if (!parseFloatValue(rhs, threshold))
            {
                setErrorLocked("invalid sensor threshold value");
                return false;
            }
            st.transition.type      = kConds[i].ctype;
            st.transition.threshold = threshold;
            return true;
        }
    }

    setErrorLocked("unsupported transition condition");
    return false;
}

// ── AMS-4.7 Guard conditions ─────────────────────────────────────────────────

bool MissionScriptEngine::parseConditionScopedLineLocked(const char* line,
                                                         StateDef& st)
{
    ARES_ASSERT(line != nullptr);

    // Optional block delimiter — accepted for style consistency with HK/LOG.
    if (strcmp(line, "{") == 0)
    {
        return true;
    }

    if (st.conditionCount >= ares::AMS_MAX_CONDITIONS)
    {
        setErrorLocked("too many conditions in state");
        return false;
    }

    char lhs[20] = {};
    char op[3]   = {};
    char rhs[20] = {};

    const int n = sscanf(line, "%19s %2s %19s", lhs, op, rhs);
    if (n != 3)
    {
        setErrorLocked("invalid condition syntax");
        return false;
    }

    // TC.command is a one-shot token — not valid as a persistent guard.
    if (strcmp(lhs, "TC.command") == 0)
    {
        setErrorLocked("TC.command not valid in conditions block");
        return false;
    }

    struct SensorCond
    {
        const char*   lhsStr;
        const char*   opStr;
        ConditionType ctype;
    };

    static const SensorCond kConds[] = {
        { "BARO.alt",      "<", ConditionType::BARO_ALT_LT    },
        { "BARO.alt",      ">", ConditionType::BARO_ALT_GT    },
        { "BARO.temp",     "<", ConditionType::BARO_TEMP_LT   },
        { "BARO.temp",     ">", ConditionType::BARO_TEMP_GT   },
        { "BARO.pressure", "<", ConditionType::BARO_PRESS_LT  },
        { "BARO.pressure", ">", ConditionType::BARO_PRESS_GT  },
        { "GPS.alt",       "<", ConditionType::GPS_ALT_LT     },
        { "GPS.alt",       ">", ConditionType::GPS_ALT_GT     },
        { "GPS.speed",     "<", ConditionType::GPS_SPEED_LT   },
        { "GPS.speed",     ">", ConditionType::GPS_SPEED_GT   },
        { "TIME.elapsed",  ">", ConditionType::TIME_ELAPSED_GT },
        // IMU accelerometer conditions
        { "IMU.accel_x",   "<", ConditionType::IMU_ACCEL_X_LT   },
        { "IMU.accel_x",   ">", ConditionType::IMU_ACCEL_X_GT   },
        { "IMU.accel_y",   "<", ConditionType::IMU_ACCEL_Y_LT   },
        { "IMU.accel_y",   ">", ConditionType::IMU_ACCEL_Y_GT   },
        { "IMU.accel_z",   "<", ConditionType::IMU_ACCEL_Z_LT   },
        { "IMU.accel_z",   ">", ConditionType::IMU_ACCEL_Z_GT   },
        { "IMU.accel_mag", ">", ConditionType::IMU_ACCEL_MAG_GT },
        { "IMU.accel_mag", "<", ConditionType::IMU_ACCEL_MAG_LT },
    };

    for (uint8_t i = 0; i < sizeof(kConds) / sizeof(kConds[0]); i++)
    {
        if (strcmp(lhs, kConds[i].lhsStr) == 0
            && strcmp(op, kConds[i].opStr) == 0)
        {
            float threshold = 0.0f;
            if (!parseFloatValue(rhs, threshold))
            {
                setErrorLocked("invalid condition threshold");
                return false;
            }
            st.conditions[st.conditionCount].type      = kConds[i].ctype;
            st.conditions[st.conditionCount].threshold = threshold;
            st.conditionCount++;
            return true;
        }
    }

    setErrorLocked("unsupported condition expression");
    return false;
}

bool MissionScriptEngine::parseOnErrorEventLineLocked(const char* line,
                                                      StateDef& st)
{
    ARES_ASSERT(line != nullptr);

    char verb[8] = {};
    char text[ares::AMS_MAX_EVENT_TEXT] = {};

    const int n = sscanf(line, "EVENT.%7[^ ] \"%63[^\"]\"", verb, text);
    if (n != 2)
    {
        setErrorLocked("invalid EVENT syntax in on_error");
        return false;
    }

    st.hasOnErrorEvent = true;
    strncpy(st.onErrorText, text, sizeof(st.onErrorText) - 1U);
    st.onErrorText[sizeof(st.onErrorText) - 1U] = '\0';

    if (strcmp(verb, "info") == 0)    { st.onErrorVerb = EventVerb::INFO;  return true; }
    if (strcmp(verb, "warning") == 0) { st.onErrorVerb = EventVerb::WARN;  return true; }
    if (strcmp(verb, "error") == 0)   { st.onErrorVerb = EventVerb::ERROR; return true; }

    setErrorLocked("unknown EVENT verb in on_error");
    return false;
}

bool MissionScriptEngine::evaluateConditionsLocked(const StateDef& state,
                                                   uint32_t nowMs)
{
    if (state.conditionCount == 0U)
    {
        return false;
    }

    BaroReading b   = {};
    GpsReading  g   = {};
    const BaroStatus baroSt = baro_.read(b);
    const GpsStatus  gpsSt  = gps_.read(g);

    for (uint8_t i = 0; i < state.conditionCount; i++)  // PO10-2: bounded
    {
        const Condition& cond = state.conditions[i];
        bool violated = false;
        const float thr = cond.threshold;

        switch (cond.type)
        {
        case ConditionType::BARO_ALT_LT:
            if (baroSt == BaroStatus::OK) { violated = !(b.altitudeM   < thr); }
            break;
        case ConditionType::BARO_ALT_GT:
            if (baroSt == BaroStatus::OK) { violated = !(b.altitudeM   > thr); }
            break;
        case ConditionType::BARO_TEMP_LT:
            if (baroSt == BaroStatus::OK) { violated = !(b.temperatureC < thr); }
            break;
        case ConditionType::BARO_TEMP_GT:
            if (baroSt == BaroStatus::OK) { violated = !(b.temperatureC > thr); }
            break;
        case ConditionType::BARO_PRESS_LT:
            if (baroSt == BaroStatus::OK) { violated = !(b.pressurePa  < thr); }
            break;
        case ConditionType::BARO_PRESS_GT:
            if (baroSt == BaroStatus::OK) { violated = !(b.pressurePa  > thr); }
            break;
        case ConditionType::GPS_ALT_LT:
            if (gpsSt == GpsStatus::OK) { violated = !(g.altitudeM  < thr); }
            break;
        case ConditionType::GPS_ALT_GT:
            if (gpsSt == GpsStatus::OK) { violated = !(g.altitudeM  > thr); }
            break;
        case ConditionType::GPS_SPEED_LT:
            if (gpsSt == GpsStatus::OK) { violated = !(g.speedKmh   < thr); }
            break;
        case ConditionType::GPS_SPEED_GT:
            if (gpsSt == GpsStatus::OK) { violated = !(g.speedKmh   > thr); }
            break;
        case ConditionType::TIME_ELAPSED_GT:
            violated = !((nowMs - stateEnterMs_) > static_cast<uint32_t>(thr));
            break;
        // IMU accelerometer guard conditions
        case ConditionType::IMU_ACCEL_X_LT:
        case ConditionType::IMU_ACCEL_X_GT:
        case ConditionType::IMU_ACCEL_Y_LT:
        case ConditionType::IMU_ACCEL_Y_GT:
        case ConditionType::IMU_ACCEL_Z_LT:
        case ConditionType::IMU_ACCEL_Z_GT:
        case ConditionType::IMU_ACCEL_MAG_GT:
        case ConditionType::IMU_ACCEL_MAG_LT:
        {
            if (imu_ != nullptr)
            {
                ImuReading imuR = {};
                const bool imuOk = (imu_->read(imuR) == ImuStatus::OK);
                if (imuOk)
                {
                    bool holds = false;
                    switch (cond.type)
                    {
                    case ConditionType::IMU_ACCEL_X_LT:
                        holds = (imuR.accelX < thr); break;
                    case ConditionType::IMU_ACCEL_X_GT:
                        holds = (imuR.accelX > thr); break;
                    case ConditionType::IMU_ACCEL_Y_LT:
                        holds = (imuR.accelY < thr); break;
                    case ConditionType::IMU_ACCEL_Y_GT:
                        holds = (imuR.accelY > thr); break;
                    case ConditionType::IMU_ACCEL_Z_LT:
                        holds = (imuR.accelZ < thr); break;
                    case ConditionType::IMU_ACCEL_Z_GT:
                        holds = (imuR.accelZ > thr); break;
                    case ConditionType::IMU_ACCEL_MAG_GT:
                    {
                        const float mag = sqrtf(imuR.accelX * imuR.accelX
                                              + imuR.accelY * imuR.accelY
                                              + imuR.accelZ * imuR.accelZ);
                        holds = (mag > thr);
                        break;
                    }
                    case ConditionType::IMU_ACCEL_MAG_LT:
                    {
                        const float mag = sqrtf(imuR.accelX * imuR.accelX
                                              + imuR.accelY * imuR.accelY
                                              + imuR.accelZ * imuR.accelZ);
                        holds = (mag < thr);
                        break;
                    }
                    default:
                        break;
                    }
                    violated = !holds;
                }
            }
            break;
        }
        default:
            break;
        }

        if (violated)
        {
            LOG_W(TAG, "condition[%u] violated in state=%s",
                  static_cast<uint32_t>(i), state.name);
            if (state.hasOnErrorEvent)
            {
                // Send immediately — setErrorLocked halts the engine next.
                sendEventLocked(state.onErrorVerb, state.onErrorText, nowMs);
            }
            setErrorLocked("condition violated");
            return true;
        }
    }

    return false;
}

bool MissionScriptEngine::mapApidToNode(uint16_t apid, uint8_t& nodeId)
{
    switch (apid)
    {
    case 0U:
        nodeId = NODE_BROADCAST;
        return true;
    case 1U:
        nodeId = NODE_ROCKET;
        return true;
    case 2U:
        nodeId = NODE_GROUND;
        return true;
    case 3U:
        nodeId = NODE_PAYLOAD;
        return true;
    default:
        return false;
    }
}

bool MissionScriptEngine::resolveTransitionsLocked()
{
    for (uint8_t i = 0; i < program_.stateCount; i++)
    {
        StateDef& st = program_.states[i];
        if (!st.hasTransition)
        {
            continue;
        }

        const uint8_t idx = findStateByNameLocked(st.transition.targetName);
        if (idx >= program_.stateCount)
        {
            setErrorLocked("unknown transition target state");
            return false;
        }

        st.transition.targetIndex = idx;
        st.transition.targetResolved = true;
    }

    return true;
}

bool MissionScriptEngine::isSafeFileName(const char* fileName)
{
    if (fileName == nullptr || fileName[0] == '\0' || fileName[0] == '.')
    {
        return false;
    }

    const uint32_t len = static_cast<uint32_t>(
        strnlen(fileName, ares::MISSION_FILENAME_MAX + 1U));
    if (len == 0U || len > ares::MISSION_FILENAME_MAX)
    {
        return false;
    }

    for (uint32_t i = 0; i < len; i++)
    {
        const char c = fileName[i];
        const bool ok = (c >= 'a' && c <= 'z')
                     || (c >= 'A' && c <= 'Z')
                     || (c >= '0' && c <= '9')
                     || c == '_' || c == '-' || c == '.';
        if (!ok)
        {
            return false;
        }
    }

    if (strstr(fileName, "..") != nullptr)
    {
        return false;
    }

    return true;
}

bool MissionScriptEngine::buildMissionPath(const char* fileName,
                                           char* outPath,
                                           uint32_t outSize)
{
    const uint32_t len = static_cast<uint32_t>(strlen(fileName));
    const bool hasExt = (len >= 4U) && (strcmp(&fileName[len - 4U], ".ams") == 0);

    int written = 0;
    if (hasExt)
    {
        written = snprintf(outPath, outSize, "%s/%s",
                           ares::MISSION_DIR, fileName);
    }
    else
    {
        written = snprintf(outPath, outSize, "%s/%s.ams",
                           ares::MISSION_DIR, fileName);
    }

    return (written > 0) && (static_cast<uint32_t>(written) < outSize);
}

void MissionScriptEngine::trimInPlace(char* text)
{
    if (text == nullptr)
    {
        return;
    }

    // CERT-3: Use strnlen() with bounds instead of strlen() (CERT-3.3)
    uint32_t len = static_cast<uint32_t>(
        strnlen(text, ares::AMS_MAX_LINE_LEN));
    while (len > 0U)
    {
        const char c = text[len - 1U];
        if (c == ' ' || c == '\t' || c == '\r')
        {
            text[len - 1U] = '\0';
            len--;
        }
        else
        {
            break;
        }
    }

    uint32_t start = 0;
    while (text[start] == ' ' || text[start] == '\t')
    {
        start++;
        // CERT-2: Bounds check on start to prevent overflow
        if (start >= len)
        {
            return;
        }
    }

    if (start > 0U)
    {
        // CERT-3: Use strnlen() instead of strlen() for memmove length (CERT-3.3)
        const uint32_t remainLen = static_cast<uint32_t>(
            strnlen(&text[start], len - start));
        memmove(text, &text[start], remainLen + 1U);  // +1 for null terminator
    }
}

bool MissionScriptEngine::startsWith(const char* text, const char* prefix)
{
    // CERT-22.1: Explicit nullptr checks at function entry
    if (text == nullptr || prefix == nullptr)
    {
        ARES_ASSERT(false && "null pointer in startsWith");
        return false;
    }

    // CERT-3: Use strnlen() with bounds for string length measurement
    const size_t preLen = strnlen(prefix, ares::AMS_MAX_LINE_LEN);
    if (preLen == 0U)
    {
        return false;
    }
    return strncmp(text, prefix, preLen) == 0;
}

bool MissionScriptEngine::parseUint(const char* text, uint32_t& outValue)
{
    if (text == nullptr)
    {
        return false;
    }

    while (*text == ' ' || *text == '\t')
    {
        text++;
    }

    if (*text == '\0')
    {
        return false;
    }

    uint64_t acc = 0U;
    bool sawDigit = false;
    while (*text >= '0' && *text <= '9')
    {
        sawDigit = true;
        acc = (acc * 10U) + static_cast<uint64_t>(*text - '0');
        if (acc > UINT32_MAX)
        {
            return false;
        }
        text++;
    }

    if (!sawDigit)
    {
        return false;
    }

    if (!isOnlyTrailingWhitespace(text))
    {
        return false;
    }

    outValue = static_cast<uint32_t>(acc);
    return true;
}

bool MissionScriptEngine::parseFloatValue(const char* text, float& outValue)
{
    if (text == nullptr)
    {
        return false;
    }

    char* endPtr = nullptr;
    const float val = strtof(text, &endPtr);
    if (endPtr == text)
    {
        return false;
    }

    if (!isOnlyTrailingWhitespace(endPtr))
    {
        return false;
    }

    if (!std::isfinite(val))
    {
        return false;
    }

    outValue = val;
    return true;
}

bool MissionScriptEngine::parseTcCommand(const char* text, TcCommand& out)
{
    if (text == nullptr)
    {
        return false;
    }

    if (strcmp(text, "LAUNCH") == 0)
    {
        out = TcCommand::LAUNCH;
        return true;
    }
    if (strcmp(text, "ABORT") == 0)
    {
        out = TcCommand::ABORT;
        return true;
    }
    if (strcmp(text, "RESET") == 0)
    {
        out = TcCommand::RESET;
        return true;
    }

    return false;
}

bool MissionScriptEngine::parseExprKind(const char* text, ExprKind& out)
{
    if (text == nullptr)
    {
        return false;
    }

    if (strcmp(text, "GPS.lat") == 0)
    {
        out = ExprKind::GPS_LAT;
        return true;
    }
    if (strcmp(text, "GPS.lon") == 0)
    {
        out = ExprKind::GPS_LON;
        return true;
    }
    if (strcmp(text, "GPS.alt") == 0)
    {
        out = ExprKind::GPS_ALT;
        return true;
    }
    if (strcmp(text, "BARO.alt") == 0)
    {
        out = ExprKind::BARO_ALT;
        return true;
    }
    if (strcmp(text, "BARO.temp") == 0)
    {
        out = ExprKind::BARO_TEMP;
        return true;
    }
    if (strcmp(text, "BARO.pressure") == 0)
    {
        out = ExprKind::BARO_PRESS;
        return true;
    }
    if (strcmp(text, "IMU.accel_x") == 0)
    {
        out = ExprKind::IMU_ACCEL_X;
        return true;
    }
    if (strcmp(text, "IMU.accel_y") == 0)
    {
        out = ExprKind::IMU_ACCEL_Y;
        return true;
    }
    if (strcmp(text, "IMU.accel_z") == 0)
    {
        out = ExprKind::IMU_ACCEL_Z;
        return true;
    }
    if (strcmp(text, "IMU.accel_mag") == 0)
    {
        out = ExprKind::IMU_ACCEL_MAG;
        return true;
    }
    if (strcmp(text, "IMU.gyro_x") == 0)
    {
        out = ExprKind::IMU_GYRO_X;
        return true;
    }
    if (strcmp(text, "IMU.gyro_y") == 0)
    {
        out = ExprKind::IMU_GYRO_Y;
        return true;
    }
    if (strcmp(text, "IMU.gyro_z") == 0)
    {
        out = ExprKind::IMU_GYRO_Z;
        return true;
    }
    if (strcmp(text, "IMU.temp") == 0)
    {
        out = ExprKind::IMU_TEMP;
        return true;
    }

    return false;
}

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

void MissionScriptEngine::setErrorLocked(const char* reason)
{
    running_ = false;
    status_ = EngineStatus::ERROR;

    strncpy(lastError_, reason, sizeof(lastError_) - 1U);
    lastError_[sizeof(lastError_) - 1U] = '\0';

    LOG_E(TAG, "error: %s", lastError_);
}

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
}

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

void MissionScriptEngine::sendHkReportLocked(uint32_t nowMs)
{
    if (currentState_ >= program_.stateCount)
    {
        return;
    }

    const StateDef& st = program_.states[currentState_];
    ARES_ASSERT(st.hkFieldCount <= ares::AMS_MAX_HK_FIELDS);
    if (!st.hasHkEvery || st.hkFieldCount == 0U)
    {
        return;
    }

    TelemetryPayload tm = {};
    tm.timestampMs = nowMs;

    GpsReading g = {};
    BaroReading b = {};
    ImuReading  imuR = {};
    const GpsStatus  gpsSt  = gps_.read(g);
    const BaroStatus baroSt = baro_.read(b);
    const ImuStatus  imuSt  = (imu_ != nullptr)
                            ? imu_->read(imuR)
                            : ImuStatus::NOT_READY;

    tm.statusBits = {};
    tm.statusBits.gpsValid = (gpsSt == GpsStatus::OK) ? 1U : 0U;

    for (uint8_t i = 0; i < st.hkFieldCount; i++)
    {
        applyHkExprToPayloadLocked(st.hkFields[i].expr, g, gpsSt, b, baroSt, imuR, imuSt, tm);
    }

    Frame frame = {};
    frame.ver = PROTOCOL_VERSION;
    frame.flags = 0;
    frame.node = program_.nodeId;
    frame.type = MsgType::TELEMETRY;
    frame.seq = seq_++;
    frame.len = static_cast<uint8_t>(sizeof(TelemetryPayload));
    memcpy(frame.payload, &tm, sizeof(tm));

    if (!sendFrameLocked(frame))
    {
        LOG_W(TAG, "HK frame send failed");
    }
}

void MissionScriptEngine::applyHkExprToPayloadLocked(ExprKind expr,
                                                     const GpsReading& g,
                                                     GpsStatus gpsSt,
                                                     const BaroReading& b,
                                                     BaroStatus baroSt,
                                                     const ImuReading& imuR,
                                                     ImuStatus imuSt,
                                                     ares::proto::TelemetryPayload& tm)
{
    switch (expr)
    {
    case ExprKind::GPS_LAT:
        if (gpsSt == GpsStatus::OK)
        {
            tm.latitudeE7 = static_cast<int32_t>(g.latitude * 10000000.0f);
        }
        break;
    case ExprKind::GPS_LON:
        if (gpsSt == GpsStatus::OK)
        {
            tm.longitudeE7 = static_cast<int32_t>(g.longitude * 10000000.0f);
        }
        break;
    case ExprKind::GPS_ALT:
        if (gpsSt == GpsStatus::OK)
        {
            const float dm = g.altitudeM * 10.0f;
            if (dm > 0.0f)
            {
                tm.gpsAltDm = (dm > static_cast<float>(UINT16_MAX))
                            ? UINT16_MAX
                            : static_cast<uint16_t>(dm);
            }
        }
        break;
    case ExprKind::BARO_ALT:
        if (baroSt == BaroStatus::OK)
        {
            tm.altitudeAglM = b.altitudeM;
        }
        break;
    case ExprKind::BARO_TEMP:
        if (baroSt == BaroStatus::OK)
        {
            tm.temperatureC = b.temperatureC;
        }
        break;
    case ExprKind::BARO_PRESS:
        if (baroSt == BaroStatus::OK)
        {
            tm.pressurePa = b.pressurePa;
        }
        break;
    case ExprKind::IMU_ACCEL_MAG:
        if (imuSt == ImuStatus::OK)
        {
            tm.accelMag = sqrtf(imuR.accelX * imuR.accelX
                              + imuR.accelY * imuR.accelY
                              + imuR.accelZ * imuR.accelZ);
        }
        break;
    // Individual accel/gyro axes and IMU temp have no dedicated field in
    // TelemetryPayload (APUS-3.6 fixed layout).  They are available in LOG
    // reports via formatExprValueLocked.  No-op here intentionally.
    case ExprKind::IMU_ACCEL_X:
    case ExprKind::IMU_ACCEL_Y:
    case ExprKind::IMU_ACCEL_Z:
    case ExprKind::IMU_GYRO_X:
    case ExprKind::IMU_GYRO_Y:
    case ExprKind::IMU_GYRO_Z:
    case ExprKind::IMU_TEMP:
        break;
    default:
        break;
    }
}

void MissionScriptEngine::appendLogReportLocked(uint32_t nowMs)
{
    if (currentState_ >= program_.stateCount)
    {
        return;
    }

    const StateDef& st = program_.states[currentState_];
    ARES_ASSERT(st.logFieldCount <= ares::AMS_MAX_HK_FIELDS);
    if (!st.hasLogEvery || st.logFieldCount == 0U || logPath_[0] == '\0')
    {
        return;
    }

    GpsReading g = {};
    BaroReading b = {};
    ImuReading  imuR = {};
    const GpsStatus  gpsSt  = gps_.read(g);
    const BaroStatus baroSt = baro_.read(b);
    const ImuStatus  imuSt  = (imu_ != nullptr)
                            ? imu_->read(imuR)
                            : ImuStatus::NOT_READY;

    char line[256] = {};

    // Helper lambda-equivalent: map ExprKind to column name string.
    // Defined as an inline switch used twice (header and data row).
    auto exprToKey = [](ExprKind e) -> const char* {
        switch (e)
        {
        case ExprKind::GPS_LAT:       return "gps_lat";
        case ExprKind::GPS_LON:       return "gps_lon";
        case ExprKind::GPS_ALT:       return "gps_alt";
        case ExprKind::BARO_ALT:      return "baro_alt";
        case ExprKind::BARO_TEMP:     return "baro_temp";
        case ExprKind::BARO_PRESS:    return "baro_pressure";
        case ExprKind::IMU_ACCEL_X:   return "imu_accel_x";
        case ExprKind::IMU_ACCEL_Y:   return "imu_accel_y";
        case ExprKind::IMU_ACCEL_Z:   return "imu_accel_z";
        case ExprKind::IMU_ACCEL_MAG: return "imu_accel_mag";
        case ExprKind::IMU_GYRO_X:    return "imu_gyro_x";
        case ExprKind::IMU_GYRO_Y:    return "imu_gyro_y";
        case ExprKind::IMU_GYRO_Z:    return "imu_gyro_z";
        case ExprKind::IMU_TEMP:      return "imu_temp";
        default:                      return "expr";
        }
    };

    // Write the CSV header row once per log file (first log write).
    if (!logHeaderWritten_)
    {
        int hLen = snprintf(line, sizeof(line), "t_ms,state");
        if (hLen > 0)
        {
            uint32_t hPos = static_cast<uint32_t>(hLen);
            for (uint8_t i = 0; i < st.logFieldCount; i++)
            {
                const int n = snprintf(&line[hPos], sizeof(line) - hPos,
                                       ",%s", exprToKey(st.logFields[i].expr));
                if (n <= 0) { break; }
                hPos += static_cast<uint32_t>(n);
                if (hPos >= sizeof(line) - 2U) { break; }
            }
            const int nl = snprintf(&line[hPos], sizeof(line) - hPos, "\n");
            if (nl > 0)
            {
                const uint32_t hTot = hPos + static_cast<uint32_t>(nl);
                storage_.appendFile(logPath_,
                                    reinterpret_cast<const uint8_t*>(line),
                                    hTot);
                logHeaderWritten_ = true;
            }
        }
    }

    // Data row: t_ms,state,val1,val2,...
    int headLen = snprintf(line, sizeof(line),
                           "%" PRIu32 ",%s",
                           nowMs,
                           st.name);
    if (headLen <= 0)
    {
        return;
    }

    uint32_t pos = static_cast<uint32_t>(headLen);
    for (uint8_t i = 0; i < st.logFieldCount; i++)
    {
        char value[32] = {};
        if (!formatExprValueLocked(st.logFields[i].expr,
                                   g, gpsSt, b, baroSt, imuR, imuSt,
                                   value, sizeof(value)))
        {
            // Write empty field to keep column alignment.
            const int n = snprintf(&line[pos], sizeof(line) - pos, ",");
            if (n <= 0) { break; }
            pos += static_cast<uint32_t>(n);
            continue;
        }

        const int n = snprintf(&line[pos], sizeof(line) - pos,
                               ",%s", value);
        if (n <= 0)
        {
            break;
        }

        pos += static_cast<uint32_t>(n);
        if (pos >= sizeof(line) - 2U)
        {
            break;
        }
    }

    const int tail = snprintf(&line[pos], sizeof(line) - pos, "\n");
    if (tail <= 0)
    {
        return;
    }

    const uint32_t len = pos + static_cast<uint32_t>(tail);
    const StorageStatus stAppend = storage_.appendFile(
        logPath_, reinterpret_cast<const uint8_t*>(line), len);
    if (stAppend != StorageStatus::OK)
    {
        LOG_W(TAG, "append mission log failed: %u", static_cast<uint32_t>(stAppend));
    }
}

bool MissionScriptEngine::ensureLogFileLocked(const char* fileName)
{
    if (fileName == nullptr || fileName[0] == '\0')
    {
        return false;
    }

    char safeName[ares::MISSION_FILENAME_MAX + 1U] = {};
    const uint32_t inLen = static_cast<uint32_t>(
        strnlen(fileName, ares::MISSION_FILENAME_MAX));
    for (uint32_t i = 0; i < inLen; i++)
    {
        const char c = fileName[i];
        const bool alphaNum = (c >= 'a' && c <= 'z')
                           || (c >= 'A' && c <= 'Z')
                           || (c >= '0' && c <= '9');
        safeName[i] = (alphaNum || c == '_' || c == '-') ? c : '_';
    }

    const int written = snprintf(logPath_, sizeof(logPath_),
                                 "%s/mission_%s.csv",
                                 ares::LOG_DIR,
                                 safeName);
    if (written <= 0 || static_cast<uint32_t>(written) >= sizeof(logPath_))
    {
        return false;
    }

    // Create an empty file; the CSV header row is written on the first log write
    // so it reflects the actual column names used by the first logging state.
    logHeaderWritten_ = false;
    const StorageStatus stWrite = storage_.writeFile(
        logPath_, reinterpret_cast<const uint8_t*>(""), 0U);
    return stWrite == StorageStatus::OK;
}

bool MissionScriptEngine::formatExprValueLocked(ExprKind expr,
                                                const GpsReading& g,
                                                GpsStatus gpsSt,
                                                const BaroReading& b,
                                                BaroStatus baroSt,
                                                const ImuReading& imuR,
                                                ImuStatus imuSt,
                                                char* out,
                                                uint32_t outSize)
{
    if (out == nullptr || outSize == 0U)
    {
        return false;
    }

    switch (expr)
    {
    case ExprKind::GPS_LAT:
        if (gpsSt == GpsStatus::OK)
        {
            return formatScaledFloat(g.latitude, 7U, out, outSize);
        }
        break;
    case ExprKind::GPS_LON:
        if (gpsSt == GpsStatus::OK)
        {
            return formatScaledFloat(g.longitude, 7U, out, outSize);
        }
        break;
    case ExprKind::GPS_ALT:
        if (gpsSt == GpsStatus::OK)
        {
            return formatScaledFloat(g.altitudeM, 2U, out, outSize);
        }
        break;
    case ExprKind::BARO_ALT:
        if (baroSt == BaroStatus::OK)
        {
            return formatScaledFloat(b.altitudeM, 2U, out, outSize);
        }
        break;
    case ExprKind::BARO_TEMP:
        if (baroSt == BaroStatus::OK)
        {
            return formatScaledFloat(b.temperatureC, 2U, out, outSize);
        }
        break;
    case ExprKind::BARO_PRESS:
        if (baroSt == BaroStatus::OK)
        {
            return formatScaledFloat(b.pressurePa, 2U, out, outSize);
        }
        break;
    case ExprKind::IMU_ACCEL_X:
        if (imuSt == ImuStatus::OK)
        {
            return formatScaledFloat(imuR.accelX, 3U, out, outSize);
        }
        break;
    case ExprKind::IMU_ACCEL_Y:
        if (imuSt == ImuStatus::OK)
        {
            return formatScaledFloat(imuR.accelY, 3U, out, outSize);
        }
        break;
    case ExprKind::IMU_ACCEL_Z:
        if (imuSt == ImuStatus::OK)
        {
            return formatScaledFloat(imuR.accelZ, 3U, out, outSize);
        }
        break;
    case ExprKind::IMU_ACCEL_MAG:
        if (imuSt == ImuStatus::OK)
        {
            const float mag = sqrtf(imuR.accelX * imuR.accelX
                                  + imuR.accelY * imuR.accelY
                                  + imuR.accelZ * imuR.accelZ);
            return formatScaledFloat(mag, 3U, out, outSize);
        }
        break;
    case ExprKind::IMU_GYRO_X:
        if (imuSt == ImuStatus::OK)
        {
            return formatScaledFloat(imuR.gyroX, 2U, out, outSize);
        }
        break;
    case ExprKind::IMU_GYRO_Y:
        if (imuSt == ImuStatus::OK)
        {
            return formatScaledFloat(imuR.gyroY, 2U, out, outSize);
        }
        break;
    case ExprKind::IMU_GYRO_Z:
        if (imuSt == ImuStatus::OK)
        {
            return formatScaledFloat(imuR.gyroZ, 2U, out, outSize);
        }
        break;
    case ExprKind::IMU_TEMP:
        if (imuSt == ImuStatus::OK)
        {
            return formatScaledFloat(imuR.tempC, 2U, out, outSize);
        }
        break;
    default:
        break;
    }

    strncpy(out, "nan", outSize - 1U);
    out[outSize - 1U] = '\0';
    return true;
}

void MissionScriptEngine::sendEventLocked(EventVerb verb,
                                          const char* text,
                                          uint32_t nowMs)
{
    ARES_ASSERT(text != nullptr);

    EventHeader header = {};
    header.timestampMs = nowMs;
    // APUS-8: AMS events represent flight-phase transitions → PHASE_CHANGE (0x02)
    header.eventId = static_cast<uint8_t>(EventId::PHASE_CHANGE);

    if (verb == EventVerb::INFO)
    {
        header.severity = static_cast<uint8_t>(EventSeverity::INFO);
    }
    else if (verb == EventVerb::WARN)
    {
        header.severity = static_cast<uint8_t>(EventSeverity::WARN);
    }
    else
    {
        header.severity = static_cast<uint8_t>(EventSeverity::ERR);
    }

    uint8_t payload[MAX_PAYLOAD_LEN] = {};
    memcpy(payload, &header, sizeof(header));

    const uint8_t maxText = static_cast<uint8_t>(MAX_PAYLOAD_LEN - sizeof(header));
    uint8_t textLen = static_cast<uint8_t>(strnlen(text, maxText));
    if (textLen > 0U)
    {
        memcpy(&payload[sizeof(header)], text, textLen);
    }

    Frame frame = {};
    frame.ver = PROTOCOL_VERSION;
    frame.flags = 0;
    frame.node = program_.nodeId;
    frame.type = MsgType::EVENT;
    frame.seq = seq_++;
    frame.len = static_cast<uint8_t>(sizeof(header) + textLen);
    memcpy(frame.payload, payload, frame.len);

    if (!sendFrameLocked(frame))
    {
        LOG_W(TAG, "EVENT frame send failed");
    }
}

bool MissionScriptEngine::sendFrameLocked(const Frame& frame)
{
    ARES_ASSERT(frame.len <= MAX_PAYLOAD_LEN);

    uint8_t wire[MAX_FRAME_LEN] = {};
    const uint16_t wireLen = ares::proto::encode(frame, wire,
                                                  sizeof(wire));
    if (wireLen == 0U)
    {
        return false;
    }

    const RadioStatus rs = radio_.send(wire, wireLen);
    return rs == RadioStatus::OK;
}

} // namespace ams
} // namespace ares

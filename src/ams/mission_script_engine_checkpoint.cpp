/**
 * @file  mission_script_engine_checkpoint.cpp
 * @brief AMS engine â€” checkpoint, resume, and abort-marker persistence.
 *
 * Handles saveResumePointLocked, tryRestoreResumePointLocked,
 * clearResumePointLocked, writeAbortMarkerLocked, and all CRC-guarded
 * record helpers.
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
#if defined(ARDUINO_ARCH_ESP32)
#include <esp_timer.h>
#endif

namespace ares
{
namespace ams
{
static constexpr const char* TAG = "AMS";
static constexpr uint8_t AMS_RESUME_VERSION = 4U;

static inline uint64_t monotonicMicros()
{
#if defined(ARDUINO_ARCH_ESP32)
    return static_cast<uint64_t>(esp_timer_get_time());
#else
    // On host/sim builds, Arduino's micros() provides the needed short-span
    // timing for profiling deferred I/O flush sections.
    return static_cast<uint64_t>(micros());
#endif
}

static uint32_t crc32Compute(const char* data, size_t len)
{
    uint32_t crc = 0xFFFFFFFFU;
    for (size_t i = 0U; i < len; i++)
    {
        crc ^= static_cast<uint8_t>(data[i]);
        for (uint8_t b = 0U; b < 8U; b++)
        {
            crc = (crc >> 1U) ^ ((crc & 1U) ? 0xEDB88320U : 0U);
        }
    }
    return crc ^ 0xFFFFFFFFU;
}

/**
 * @brief Clamp the per-tick action budget to the valid range [1, 3].
 *
 * @c budget counts **action groups** (EVENT=0, HK=1, LOG=2) dispatched per
 * tick, not individual HK/LOG slots.  When the HK group is selected, all due
 * HK slots fire within that single budget step (AMS-4.4).
 *
 * A budget of 0 is treated as 1 (at least one group per tick).
 * Values above 3 are capped to 3 (APUS-19.2 maximum).
 *
 * @param[in] budget  Unclamped budget value.
 * @return Value clamped to [1, 3].
 */

bool MissionScriptEngine::buildCheckpointRecordLocked(uint64_t nowMs,
                                                      char*    record,
                                                      size_t   recSize,
                                                      int32_t& outWritten) const
{
    const uint32_t stateElapsed = static_cast<uint32_t>(nowMs - stateEnterMs_);
    const uint32_t hkElapsed    = static_cast<uint32_t>(nowMs - lastHkMs_);
    const uint32_t logElapsed   = static_cast<uint32_t>(nowMs - lastLogMs_);

    int32_t written = static_cast<int32_t>(snprintf(record, recSize,
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
                           stateElapsed, hkElapsed, logElapsed));
    if (written <= 0 || static_cast<size_t>(written) >= recSize)
    {
        return false;
    }

    if (!appendVarsSectionLocked(record, recSize, written))
    {
        LOG_W(TAG, "checkpoint: vars section truncated, aborting");
        return false;
    }
    if (!appendSlotTimersSectionLocked(record, recSize, written, nowMs))
    {
        LOG_W(TAG, "checkpoint: slot timers section truncated, aborting");
        return false;
    }
    if (!appendConfirmHoldSectionLocked(record, recSize, written, nowMs))
    {
        LOG_W(TAG, "checkpoint: confirm-hold section truncated, aborting");
        return false;
    }

    // Append CRC32 over all bytes written so far (AMS-8.5).
    // The CRC is computed AFTER all data fields so it covers the complete record.
    if (written > 0 && static_cast<size_t>(written) < recSize)
    {
        const uint32_t crc = crc32Compute(record, static_cast<size_t>(written));
        const int32_t wCrc = static_cast<int32_t>(snprintf(
            record + written,
            recSize - static_cast<size_t>(written),
            "|%08" PRIX32, crc));
        if (wCrc > 0 &&
            (static_cast<size_t>(written) + static_cast<size_t>(wCrc)) < recSize)
        {
            written += wCrc;
        }
    }

    outWritten = written;
    return true;
}

bool MissionScriptEngine::appendConfirmHoldSectionLocked(char*    record,
                                                         size_t   recSize,
                                                         int32_t& written,
                                                         uint64_t nowMs) const
{
    if (written <= 0) { return false; }

    // tcConfirmCount_[0..3] (AMS-4.11.2 v4 checkpoint).
    const size_t  rem1 = recSize - static_cast<size_t>(written);
    const int32_t wCc = static_cast<int32_t>(snprintf(
        record + written, rem1,
        "|%" PRIu32 "|%" PRIu32 "|%" PRIu32 "|%" PRIu32,
        static_cast<uint32_t>(tcConfirmCount_[0]),
        static_cast<uint32_t>(tcConfirmCount_[1]),
        static_cast<uint32_t>(tcConfirmCount_[2]),
        static_cast<uint32_t>(tcConfirmCount_[3])));
    if (wCc <= 0 || static_cast<size_t>(wCc) >= rem1) { return false; }
    written += wCc;

    // transitionCondHolding_[i] and elapsed-since-condMet for each slot (AMS-4.6.1).
    for (uint8_t i = 0U; i < ares::AMS_MAX_TRANSITIONS; i++)  // PO10-2
    {
        const uint32_t condElap = transitionCondHolding_[i]
            ? static_cast<uint32_t>(nowMs - transitionCondMetMs_[i])
            : 0U;
        const size_t  remH = recSize - static_cast<size_t>(written);
        const int32_t wh = static_cast<int32_t>(snprintf(
            record + written, remH,
            "|%" PRIu32 "|%" PRIu32,
            transitionCondHolding_[i] ? 1U : 0U,
            condElap));
        if (wh <= 0 || static_cast<size_t>(wh) >= remH) { return false; }
        written += wh;
    }
    return true;
}

bool MissionScriptEngine::appendVarsSectionLocked(char*    record,
                                                  size_t   recSize,
                                                  int32_t& written) const
{
    if (program_.varCount == 0U) { return true; }
    if (written <= 0) { return false; }

    const size_t  rem0 = recSize - static_cast<size_t>(written);
    int32_t w2 = static_cast<int32_t>(snprintf(record + written, rem0,
                     "|%" PRIu32, static_cast<uint32_t>(program_.varCount)));
    if (w2 <= 0 || static_cast<size_t>(w2) >= rem0) { return false; }
    written += w2;

    for (uint8_t vi = 0; vi < program_.varCount; vi++)  // PO10-2
    {
        const VarEntry& v = program_.vars[vi];
        char floatBuf[24] = {};
        (void)snprintf(floatBuf, sizeof(floatBuf), "%.6g",
                       static_cast<double>(v.value));
        const size_t  remV = recSize - static_cast<size_t>(written);
        const int32_t w3 = static_cast<int32_t>(snprintf(record + written, remV,
                                "|%s=%s=%u",
                                v.name, floatBuf, v.valid ? 1U : 0U));
        if (w3 <= 0 || static_cast<size_t>(w3) >= remV) { return false; }
        written += w3;
    }
    return true;
}

bool MissionScriptEngine::appendSlotTimersSectionLocked(char*    record,
                                                        size_t   recSize,
                                                        int32_t& written,
                                                        uint64_t nowMs) const
{
    if (written <= 0) { return false; }
    if (currentState_ >= program_.stateCount) { return true; }
    const StateDef& curSt = program_.states[currentState_];

    // HK slots: count then per-slot elapsed (AMS-4.3.1 v3 checkpoint).
    const size_t  remH = recSize - static_cast<size_t>(written);
    const int32_t wHk = static_cast<int32_t>(snprintf(
        record + written, remH,
        "|%" PRIu32, static_cast<uint32_t>(curSt.hkSlotCount)));
    if (wHk <= 0 || static_cast<size_t>(wHk) >= remH) { return false; }
    written += wHk;

    for (uint8_t s = 0U; s < curSt.hkSlotCount; s++)  // PO10-2
    {
        const uint32_t slotElap = static_cast<uint32_t>(nowMs - lastHkSlotMs_[s]);
        const size_t  remS = recSize - static_cast<size_t>(written);
        const int32_t ws = static_cast<int32_t>(snprintf(
            record + written, remS,
            "|%" PRIu32, slotElap));
        if (ws <= 0 || static_cast<size_t>(ws) >= remS) { return false; }
        written += ws;
    }

    // LOG slots: count then per-slot elapsed.
    const size_t  remL = recSize - static_cast<size_t>(written);
    const int32_t wLog = static_cast<int32_t>(snprintf(
        record + written, remL,
        "|%" PRIu32, static_cast<uint32_t>(curSt.logSlotCount)));
    if (wLog <= 0 || static_cast<size_t>(wLog) >= remL) { return false; }
    written += wLog;

    for (uint8_t s = 0U; s < curSt.logSlotCount; s++)  // PO10-2
    {
        const uint32_t slotElap = static_cast<uint32_t>(nowMs - lastLogSlotMs_[s]);
        const size_t  remS = recSize - static_cast<size_t>(written);
        const int32_t ws = static_cast<int32_t>(snprintf(
            record + written, remS,
            "|%" PRIu32, slotElap));
        if (ws <= 0 || static_cast<size_t>(ws) >= remS) { return false; }
        written += ws;
    }
    return true;
}

bool MissionScriptEngine::saveResumePointLocked(uint64_t nowMs, bool force)
{
    if (!running_ || activeFile_[0] == '\0')
    {
        return false;
    }

    if (!force && ((nowMs - lastCheckpointMs_) < ares::AMS_CHECKPOINT_INTERVAL_MS))
    {
        return true;
    }

    if (!force && !checkpointDirty_
        && ((nowMs - lastCheckpointMs_) < ares::AMS_CHECKPOINT_STABLE_INTERVAL_MS))
    {
        // No material state change since last write; defer until the stable
        // cadence elapses to reduce flash sector wear (AMS-8.4).
        return true;
    }

    if (currentState_ >= program_.stateCount)
    {
        return false;
    }

    // v3 format:
    //   VERSION|file|state|exec|running|status|seq|stateElap|hkElap|logElap
    //   [|varCount|name1=value1=valid1|...|nameN=valueN=validN]
    //   |hkSlotCount|hkSlotElap0|...|logSlotCount|logSlotElap0|...
    //
    // Stage the record in pendingCheckpoint_.buf; the actual writeFile call is
    // deferred to flushPendingCheckpointUnlocked() after the mutex is released (AMS-8.3).
    int32_t written = 0;
    if (!buildCheckpointRecordLocked(nowMs, pendingCheckpoint_.buf,
                                     sizeof(pendingCheckpoint_.buf), written))
    {
        return false;
    }

    pendingCheckpoint_.len     = written;
    pendingCheckpoint_.pending = true;
    pendingCheckpoint_.nowMs   = nowMs;   // committed to lastCheckpointMs_ only on successful flush
    return true;
}

void MissionScriptEngine::clearResumePointLocked()
{
    // MUTEX INVARIANT — NO RACE WITH saveResumePointLocked / flushPendingCheckpointUnlocked
    //
    // The `Locked` suffix on this function (and on saveResumePointLocked) is a
    // firm contract: the caller MUST hold mutex_ before entering.  FreeRTOS
    // mutexes are non-reentrant and grant exclusive ownership, so these two
    // functions can never execute concurrently on any task.
    //
    // The two-phase write design (stage under mutex, flush after release) adds a
    // subtlety: flushPendingCheckpointUnlocked() runs *without* the mutex and calls
    // writeFile() when pendingCheckpoint_.pending == true.  The potential
    // scenario is:
    //
    //   1. Task A: saveResumePointLocked() sets pending = true, releases mutex.
    //   2. Task B: acquires mutex, calls clearResumePointLocked().
    //   3. Task A: flushPendingCheckpointUnlocked() — would it write a stale checkpoint?
    //
    // This is prevented by the line below.  Setting pending = false here
    // (while still holding mutex_) happens-before flushPendingCheckpointUnlocked()
    // inspects the flag, because in the current single-engine, single-ticker
    // architecture only one task drives the engine: it holds the mutex for the
    // entire tick (or API call), then flushes.  A second task that calls clear
    // via an API endpoint also holds the mutex, and its flush in turn runs
    // serially after this function returns.  There is no path where flush from
    // task A and clear from task B overlap.
    //
    // TL;DR: pending = false here atomically cancels any staged write before
    // the removeFile below, making the clear/save ordering safe by construction.
    pendingCheckpoint_.pending = false;

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

void MissionScriptEngine::writeAbortMarkerLocked(const char* stateName, uint64_t nowMs)
{
    // Appends one final CSV row to the mission log so the abort event is
    // traceable in post-flight analysis (AMS ABORT audit trail).
    // queueAppendLocked copies logPath_ into a local buffer, so this row is
    // written even after deactivateLocked() clears logPath_.
    if (logPath_[0] == '\0') { return; }

    // Build row body without newline so CRC8 covers only the data content.
    // kCrcSuffix: "," + 2 hex digits + "\n" + NUL = 5 bytes (",xx\n\0").
    static constexpr size_t kCrcSuffix = 5U;
    static char line[100] = {};
    const int32_t bodyLen = static_cast<int32_t>(
        snprintf(line, sizeof(line) - kCrcSuffix,
                 "%" PRIu64 ",ABORT,aborted_in_state=%s",
                 nowMs, stateName));
    if (bodyLen > 0 && static_cast<uint32_t>(bodyLen) < sizeof(line) - kCrcSuffix)
    {
        const uint32_t pos = static_cast<uint32_t>(bodyLen);
        const uint8_t  crc = detail::crc8Smbus(line, pos);
        const int32_t tail = static_cast<int32_t>(
            snprintf(&line[pos], sizeof(line) - pos, ",%02x\n",
                     static_cast<unsigned>(crc)));
        if (tail > 0)
        {
            queueAppendLocked(logPath_,
                              reinterpret_cast<const uint8_t*>(line),
                              pos + static_cast<uint32_t>(tail));
        }
    }
}

// ── queueAppendLocked ─────────────────────────────────────────────────────────

/**
 * @brief Stage a LittleFS append operation for deferred execution outside the mutex.
 *
 * Copies @p data into the next free @c pendingAppends_ slot so the actual
 * @c appendFile call can be issued by @c flushPendingAppendsUnlocked() after the
 * mutex has been released (AMS-8.3).  Drops the entry with a warning if the
 * queue is full or the payload exceeds the slot buffer.
 *
 * @param[in] path  Destination file path (null-terminated, ≤ STORAGE_MAX_PATH).
 * @param[in] data  Raw bytes to append.
 * @param[in] len   Byte count.  Must satisfy 0 < len ≤ sizeof(PendingAppend::data).
 * @pre  Caller holds the engine mutex.
 */
void MissionScriptEngine::queueAppendLocked(const char*    path,
                                            const uint8_t* data,
                                            uint32_t       len,
                                            bool*          onSuccessFlag)
{
    const bool notifyWorker = (pendingAppendCount_ == 0U);

    // Header rows carry a non-null success flag pointer. While that header is
    // pending flush, avoid enqueueing duplicates for the same file/flag pair.
    // This prevents repeated CSV headers when producer cadence is faster than
    // deferred I/O flush cadence.
    if (onSuccessFlag != nullptr)
    {
        for (uint8_t i = 0U; i < pendingAppendCount_; i++)
        {
            const PendingAppend& queued = pendingAppends_[i];
            if (queued.onSuccessFlag == onSuccessFlag
                && strcmp(queued.path, path) == 0)
            {
                return;
            }
        }
    }

    if (pendingAppendCount_ >= kMaxPendingAppends)
    {
        LOG_W(TAG, "append queue full — row dropped (path=%s)", path);
        return;
    }
    if (len == 0U || len > static_cast<uint32_t>(sizeof(PendingAppend::data)))
    {
        LOG_W(TAG, "queueAppend: invalid len %" PRIu32, len);
        return;
    }

    PendingAppend& e = pendingAppends_[pendingAppendCount_];
    ares::util::copyZ(e.path, path, sizeof(e.path));
    memcpy(e.data, data, len);
    e.len           = len;
    e.onSuccessFlag = onSuccessFlag; // null for data rows; set for header entries
    pendingAppendCount_++;

    if (notifyWorker)
    {
        notifyDeferredIoWorker();
    }
}

// ── flushPendingAppendsUnlocked ──────────────────────────────────────────────

bool MissionScriptEngine::stageAppendBurstLocked(uint8_t* burst,
                                                 uint32_t burstCap,
                                                 uint32_t& burstLen,
                                                 uint8_t& burstRows,
                                                 char* burstPath,
                                                 size_t burstPathSize,
                                                 bool** successFlags,
                                                 uint8_t& successFlagCount)
{
    ScopedLock guard(mutex_, pdMS_TO_TICKS(ares::AMS_MUTEX_TIMEOUT_MS));
    if (!guard.acquired())
    {
        LOG_W(TAG, "deferred append drain: mutex timeout");
        return false;
    }

    if (pendingAppendCount_ == 0U)
    {
        return false;
    }

    ares::util::copyZ(burstPath, pendingAppends_[0U].path, burstPathSize);

    while (pendingAppendCount_ > 0U)
    {
        const PendingAppend& head = pendingAppends_[0U];
        if ((strcmp(head.path, burstPath) != 0) && (burstRows > 0U))
        {
            break;
        }

        const uint32_t freeBytes = burstCap - burstLen;
        if ((head.len > freeBytes) && (burstRows > 0U))
        {
            break;
        }

        memcpy(&burst[burstLen], head.data, head.len);
        burstLen += head.len;
        burstRows++;

        if ((head.onSuccessFlag != nullptr) && (successFlagCount < kMaxPendingAppends))
        {
            successFlags[successFlagCount] = head.onSuccessFlag;
            successFlagCount++;
        }

        for (uint8_t i = 1U; i < pendingAppendCount_; i++)
        {
            pendingAppends_[i - 1U] = pendingAppends_[i];
        }
        pendingAppendCount_--;

        if (burstLen >= burstCap)
        {
            break;
        }
    }

    return (burstRows > 0U) && (burstLen > 0U);
}

void MissionScriptEngine::markAppendSuccessFlags(bool* const* successFlags,
                                                 uint8_t      successFlagCount)
{
    for (uint8_t i = 0U; i < successFlagCount; i++)
    {
        if (successFlags[i] != nullptr)
        {
            *successFlags[i] = true;
        }
    }
}

/**
 * @brief Drain staged CSV appends from the deferred I/O queue.
 *
 * Copies the bounded append queue while holding the engine mutex, then writes
 * the copied rows to LittleFS after the lock is released.  This keeps the
 * per-row flash append path off the mission tick while preserving bounded RAM
 * staging and retry semantics.
 */
void MissionScriptEngine::flushPendingAppendsUnlocked()
{
    const uint64_t flushStartUs = monotonicMicros();
    uint8_t drainedCount = 0U;

    static_assert(ares::AMS_IO_APPEND_BURST_BYTES >= 256U,
                  "AMS_IO_APPEND_BURST_BYTES must fit one max CSV row");

    while (true)
    {
        uint8_t  burst[ares::AMS_IO_APPEND_BURST_BYTES] = {};
        uint32_t burstLen = 0U;
        uint8_t  burstRows = 0U;
        char     burstPath[ares::STORAGE_MAX_PATH] = {};
        bool*    successFlags[kMaxPendingAppends] = {};
        uint8_t  successFlagCount = 0U;

        if (!stageAppendBurstLocked(burst,
                                    static_cast<uint32_t>(sizeof(burst)),
                                    burstLen,
                                    burstRows,
                                    burstPath,
                                    sizeof(burstPath),
                                    successFlags,
                                    successFlagCount))
        {
            break;
        }

        drainedCount = static_cast<uint8_t>(drainedCount + burstRows);

        const StorageStatus st = storage_.appendFile(burstPath, burst, burstLen);
        if (st == StorageStatus::OK)
        {
            markAppendSuccessFlags(successFlags, successFlagCount);
        }
        else
        {
            LOG_W(TAG, "deferred append batch failed: rows=%u bytes=%u path='%s' st=%u",
                  static_cast<unsigned>(burstRows),
                  static_cast<unsigned>(burstLen),
                  burstPath,
                  static_cast<uint32_t>(st));
        }
    }

    if (drainedCount == 0U)
    {
        return;
    }

    if (ares::LOOP_TIMING_PROFILE_ENABLED)
    {
        const uint32_t flushElapsedUs = static_cast<uint32_t>(
            monotonicMicros() - flushStartUs);
        LOG_I(TAG, "append drain: %" PRIu32 " us", flushElapsedUs);
    }
}

// ── flushPendingCheckpointUnlocked ───────────────────────────────────────────

/**
 * @brief Execute the staged checkpoint write outside the mutex.
 *
 * Must be called AFTER the engine mutex has been released.  Issues the
 * checkpoint @c writeFile operation deferred by @c saveResumePointLocked,
 * keeping that flash write outside the critical section (AMS-8.3).
 *
 * @post @c pendingCheckpoint_.pending == false.
 */
void MissionScriptEngine::flushPendingCheckpointUnlocked()
{
    const uint64_t flushStartUs = monotonicMicros();

    // Flush staged checkpoint write.
    if (pendingCheckpoint_.pending && pendingCheckpoint_.len > 0)
    {
        const StorageStatus st = storage_.writeFile(
            ares::AMS_RESUME_PATH,
            reinterpret_cast<const uint8_t*>(pendingCheckpoint_.buf),
            static_cast<uint32_t>(pendingCheckpoint_.len));
        if (st == StorageStatus::OK)
        {
            // Advance timing markers only after a confirmed write.  Pre-marking
            // them in saveResumePointLocked() would cause a failed flush to be
            // treated as successful, deferring the next retry by a full
            // AMS_CHECKPOINT_INTERVAL_MS even though no data reached disk.
            ScopedLock guard(mutex_, pdMS_TO_TICKS(ares::AMS_MUTEX_TIMEOUT_MS));
            if (guard.acquired())
            {
                lastCheckpointMs_ = pendingCheckpoint_.nowMs;
                checkpointDirty_  = false;
            }
            else
            {
                LOG_W(TAG, "checkpoint commit: mutex timeout — markers not advanced, will retry");
            }
        }
        else
        {
            LOG_W(TAG, "deferred checkpoint write failed: %u — re-arming for retry",
                  static_cast<uint32_t>(st));
            // Re-arm the dirty flag so that the next tick retries after
            // AMS_CHECKPOINT_INTERVAL_MS rather than the longer stable cadence.
            // lastCheckpointMs_ is intentionally left unchanged so the retry
            // window is measured from the last *successful* write.
            ScopedLock guard(mutex_, pdMS_TO_TICKS(ares::AMS_MUTEX_TIMEOUT_MS));
            if (guard.acquired())
            {
                checkpointDirty_ = true;
            }
        }
        pendingCheckpoint_.pending = false;
    }

    if (ares::LOOP_TIMING_PROFILE_ENABLED)
    {
        const uint32_t flushElapsedUs = static_cast<uint32_t>(monotonicMicros() - flushStartUs);
        LOG_I(TAG, "checkpoint flush: %" PRIu32 " us", flushElapsedUs);
    }
}

void MissionScriptEngine::deferredIoTaskFn(void* param)
{
    auto* self = static_cast<MissionScriptEngine*>(param);
    ARES_ASSERT(self != nullptr);
    self->deferredIoTaskLoop();
}

void MissionScriptEngine::deferredIoTaskLoop()
{
    while (true)
    {
        (void)ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        flushPendingAppendsUnlocked();
    }
}

void MissionScriptEngine::notifyDeferredIoWorker()
{
    if (deferredIoTaskHandle_ != nullptr)
    {
        (void)xTaskNotifyGive(deferredIoTaskHandle_);
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
                                                      uint32_t& logElapsed)
{
    // Safe: buf is a NUL-terminated fixed-size checkpoint buffer; string field uses
    // %32[^|] width limiter (fits fileName[33]); all other fields are SCNu32; return checked == 10.
    const int32_t parsed = static_cast<int32_t>(sscanf(buf,  // NOLINT(bugprone-unchecked-string-to-number-conversion)
                              "%" SCNu32 "|%32[^|]|%" SCNu32 "|%" SCNu32
                              "|%" SCNu32 "|%" SCNu32 "|%" SCNu32
                              "|%" SCNu32 "|%" SCNu32 "|%" SCNu32,
                              &version, fileName,
                              &stateIdx, &execEnabled, &running,
                              &status, &seq,
                              &stateElapsed, &hkElapsed, &logElapsed));
    return parsed == 10;
}

const char* MissionScriptEngine::restoreCheckpointVarsLocked(const char* cursor)
{
    uint32_t vcStored = 0U;
    // Safe: cursor points into a NUL-terminated checkpoint buffer; reads a single SCNu32
    // (variable count); return value checked — vc != 1 short-circuits restore.
    int32_t vc = static_cast<int32_t>(sscanf(cursor, "%" SCNu32, &vcStored));  // NOLINT(bugprone-unchecked-string-to-number-conversion)

    // Always advance past the varCount token so the caller can locate the next section.
    while (*cursor != '\0' && *cursor != '|') { cursor++; }

    if (vc != 1 || vcStored == 0U) { return cursor; }

    for (uint32_t vi = 0; vi < vcStored && *cursor != '\0'; vi++)
    {
        if (*cursor == '|') { cursor++; }
        char vName[ares::AMS_VAR_NAME_LEN] = {};
        char vValStr[24] = {};
        uint32_t vValid  = 0U;
        // Safe: vName[AMS_VAR_NAME_LEN] fits %15[^=] (≤15 chars + NUL), vValStr[24] fits
        // %23[^=] (≤23 chars + NUL); return value checked == 3 before any field is used.
        // cppcheck-suppress [cert-err34-c]
        const int32_t vp = static_cast<int32_t>(sscanf(cursor, "%15[^=]=%23[^=]=%u",  // NOLINT(bugprone-unchecked-string-to-number-conversion)
                              vName, vValStr, &vValid));
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
    return cursor;
}

const char* MissionScriptEngine::restoreCheckpointSlotsLocked(const char* cursor, uint64_t nowMs)
{
    // cursor points to the first character of the hkSlotCount value.
    uint32_t hkCount = 0U;
    // Safe: cursor points into a NUL-terminated checkpoint buffer; reads a single SCNu32
    // (hkSlotCount); return checked != 1; value clamped to AMS_MAX_HK_SLOTS before loop use.
    if (static_cast<int32_t>(sscanf(cursor, "%" SCNu32, &hkCount)) != 1)  // NOLINT(bugprone-unchecked-string-to-number-conversion)
    {
        return cursor;
    }
    const uint8_t hkSlots = static_cast<uint8_t>(
        hkCount < ares::AMS_MAX_HK_SLOTS ? hkCount : ares::AMS_MAX_HK_SLOTS);

    while (*cursor != '\0' && *cursor != '|') { cursor++; }  // advance past hkCount

    for (uint8_t s = 0U; s < hkSlots && *cursor != '\0'; s++)
    {
        if (*cursor == '|') { cursor++; }
        uint32_t elap = 0U;
        // Safe: cursor points into a NUL-terminated checkpoint buffer; reads a single SCNu32
        // (elapsed ms); result used only as a relative timestamp offset; == 1 check guards use.
        if (static_cast<int32_t>(sscanf(cursor, "%" SCNu32, &elap)) == 1)  // NOLINT(bugprone-unchecked-string-to-number-conversion)
        {
            lastHkSlotMs_[s] = nowMs - elap;
        }
        while (*cursor != '\0' && *cursor != '|') { cursor++; }
    }

    // logSlotCount field.
    if (*cursor == '|') { cursor++; }
    uint32_t logCount = 0U;
    // Safe: cursor points into a NUL-terminated checkpoint buffer; reads a single SCNu32
    // (logSlotCount); return checked != 1; value clamped to AMS_MAX_HK_SLOTS before loop use.
    if (static_cast<int32_t>(sscanf(cursor, "%" SCNu32, &logCount)) != 1)  // NOLINT(bugprone-unchecked-string-to-number-conversion)
    {
        return cursor;
    }
    const uint8_t logSlots = static_cast<uint8_t>(
        logCount < ares::AMS_MAX_HK_SLOTS ? logCount : ares::AMS_MAX_HK_SLOTS);

    while (*cursor != '\0' && *cursor != '|') { cursor++; }  // advance past logCount

    for (uint8_t s = 0U; s < logSlots && *cursor != '\0'; s++)
    {
        if (*cursor == '|') { cursor++; }
        uint32_t elap = 0U;
        // Safe: cursor points into a NUL-terminated checkpoint buffer; reads a single SCNu32
        // (elapsed ms); result used only as a relative timestamp offset; == 1 check guards use.
        if (static_cast<int32_t>(sscanf(cursor, "%" SCNu32, &elap)) == 1)  // NOLINT(bugprone-unchecked-string-to-number-conversion)
        {
            lastLogSlotMs_[s] = nowMs - elap;
        }
        while (*cursor != '\0' && *cursor != '|') { cursor++; }
    }
    return cursor;
}

// ── restoreCheckpointV4StrictLocked ──────────────────────────────────────────
//
// Strict, atomic restore of all v4-specific sections.
//
// Called by tryRestoreResumePointLocked ONLY for version == 4 records whose
// CRC has already been validated.  Parses every section (vars, HK/LOG slots,
// confirm counters, hold windows) into temporary staging buffers first.  If
// any required field is missing or a count is inconsistent with the currently-
// loaded program, the function logs a warning and returns false WITHOUT
// modifying any live engine state.  All changes are committed atomically only
// when every check passes.
//
// This prevents the partial-restore hazard that the lenient v2/v3 path
// tolerates for backward compat: a v4 record with a valid CRC but a wrong
// field count would previously corrupt tcConfirmCount_, hold windows, or slot
// timers.
//
// @param cursor  Points to the first character of the 11th pipe-delimited
//                token in the checkpoint buffer, i.e., the first field after
//                the 10-field header.  The caller has already advanced past
//                the 10th pipe.
// @param nowMs   Current monotonic clock value, used to reconstruct absolute
//                timestamps from stored elapsed values.
// @return true   All fields parsed and consistent; live state updated.
// @return false  Structural mismatch; caller should discard the checkpoint.
bool MissionScriptEngine::restoreCheckpointV4StrictLocked(const char* cursor, uint64_t nowMs) // NOLINT(readability-function-size)
{
    // ── Temporary staging buffers ──────────────────────────────────────────
    // All reads go here first; live state is not touched until all checks pass.
    float    tmpVarVal[ares::AMS_MAX_VARS]          = {};
    bool     tmpVarValid[ares::AMS_MAX_VARS]        = {};
    bool     tmpVarSet[ares::AMS_MAX_VARS]          = {};  // which slots were populated
    uint32_t tmpHkElap[ares::AMS_MAX_HK_SLOTS]     = {};
    uint32_t tmpLogElap[ares::AMS_MAX_HK_SLOTS]    = {};
    uint8_t  tmpCc[4]                              = {};
    bool     tmpHolding[ares::AMS_MAX_TRANSITIONS] = {};
    uint32_t tmpCondElap[ares::AMS_MAX_TRANSITIONS]= {};

    const StateDef& curSt = program_.states[currentState_];

    // ── 1. Vars section (only written when program_.varCount > 0) ─────────
    // When varCount == 0 the checkpoint has no varCount token; cursor is
    // already pointing at the hkSlotCount digit.
    if (program_.varCount > 0U)
    {
        uint32_t vcStored = 0U;
        // Safe: NUL-terminated checkpoint buffer; single SCNu32; return checked.
        if (static_cast<int32_t>(sscanf(cursor, "%" SCNu32, &vcStored)) != 1)  // NOLINT(bugprone-unchecked-string-to-number-conversion)
        {
            return false;
        }
        if (vcStored != static_cast<uint32_t>(program_.varCount))
        {
            LOG_W(TAG, "resume v4: varCount mismatch (stored=%u loaded=%u) — discarding",
                  static_cast<unsigned>(vcStored),
                  static_cast<unsigned>(program_.varCount));
            return false;
        }
        while (*cursor != '\0' && *cursor != '|') { cursor++; }

        for (uint32_t vi = 0U; vi < vcStored && *cursor != '\0'; vi++)
        {
            if (*cursor != '|') { return false; }
            cursor++;
            char vName[ares::AMS_VAR_NAME_LEN] = {};
            char vValStr[24] = {};
            uint32_t vValid = 0U;
            // Safe: NUL-terminated buffer; widths limit both strings; return checked == 3.
            // cppcheck-suppress [cert-err34-c]
            if (static_cast<int32_t>(sscanf(cursor, "%15[^=]=%23[^=]=%u",  // NOLINT(bugprone-unchecked-string-to-number-conversion)
                                            vName, vValStr, &vValid)) != 3)
            {
                return false;
            }
            const VarEntry* v = findVarLocked(vName);
            if (v != nullptr)
            {
                const uint32_t vidx = static_cast<uint32_t>(v - program_.vars);
                if (vidx < static_cast<uint32_t>(ares::AMS_MAX_VARS))
                {
                    float fval = 0.0f;
                    if (parseFloatValue(vValStr, fval))
                    {
                        tmpVarVal[vidx]   = fval;
                        tmpVarValid[vidx] = (vValid != 0U);
                        tmpVarSet[vidx]   = true;
                    }
                }
            }
            while (*cursor != '\0' && *cursor != '|') { cursor++; }
        }
        // cursor is now at '|' before hkSlotCount (or '\0').
    }
    // else: cursor already at hkSlotCount digit — no varCount token present.

    // ── 2. HK slot count (must match currently-loaded state) ──────────────
    // When varCount > 0 the vars section leaves cursor at '|hkSlotCount';
    // when varCount == 0 cursor is already at the hkSlotCount digit.
    if (program_.varCount > 0U)
    {
        if (*cursor != '|') { return false; }
        cursor++;
    }
    uint32_t hkCount = 0U;
    // Safe: NUL-terminated checkpoint buffer; single SCNu32; return checked.
    if (static_cast<int32_t>(sscanf(cursor, "%" SCNu32, &hkCount)) != 1)  // NOLINT(bugprone-unchecked-string-to-number-conversion)
    {
        return false;
    }
    if (hkCount != static_cast<uint32_t>(curSt.hkSlotCount))
    {
        LOG_W(TAG, "resume v4: hkSlotCount mismatch (stored=%u state=%u) — discarding",
              static_cast<unsigned>(hkCount),
              static_cast<unsigned>(curSt.hkSlotCount));
        return false;
    }
    while (*cursor != '\0' && *cursor != '|') { cursor++; }

    for (uint8_t s = 0U; s < static_cast<uint8_t>(hkCount); s++)
    {
        if (*cursor != '|') { return false; }
        cursor++;
        // Safe: NUL-terminated checkpoint buffer; single SCNu32; return checked.
        if (static_cast<int32_t>(sscanf(cursor, "%" SCNu32, &tmpHkElap[s])) != 1)  // NOLINT(bugprone-unchecked-string-to-number-conversion)
        {
            return false;
        }
        while (*cursor != '\0' && *cursor != '|') { cursor++; }
    }

    // ── 3. LOG slot count (must match currently-loaded state) ─────────────
    if (*cursor != '|') { return false; }
    cursor++;
    uint32_t logCount = 0U;
    // Safe: NUL-terminated checkpoint buffer; single SCNu32; return checked.
    if (static_cast<int32_t>(sscanf(cursor, "%" SCNu32, &logCount)) != 1)  // NOLINT(bugprone-unchecked-string-to-number-conversion)
    {
        return false;
    }
    if (logCount != static_cast<uint32_t>(curSt.logSlotCount))
    {
        LOG_W(TAG, "resume v4: logSlotCount mismatch (stored=%u state=%u) — discarding",
              static_cast<unsigned>(logCount),
              static_cast<unsigned>(curSt.logSlotCount));
        return false;
    }
    while (*cursor != '\0' && *cursor != '|') { cursor++; }

    for (uint8_t s = 0U; s < static_cast<uint8_t>(logCount); s++)
    {
        if (*cursor != '|') { return false; }
        cursor++;
        // Safe: NUL-terminated checkpoint buffer; single SCNu32; return checked.
        if (static_cast<int32_t>(sscanf(cursor, "%" SCNu32, &tmpLogElap[s])) != 1)  // NOLINT(bugprone-unchecked-string-to-number-conversion)
        {
            return false;
        }
        while (*cursor != '\0' && *cursor != '|') { cursor++; }
    }

    // ── 4. Confirm counters (exactly 4 required) ──────────────────────────
    for (uint8_t i = 0U; i < 4U; i++)  // PO10-2
    {
        if (*cursor != '|')
        {
            LOG_W(TAG, "resume v4: confirm counter[%u] missing — discarding",
                  static_cast<unsigned>(i));
            return false;
        }
        cursor++;
        uint32_t cc = 0U;
        // Safe: NUL-terminated checkpoint buffer; single SCNu32; return checked.
        if (static_cast<int32_t>(sscanf(cursor, "%" SCNu32, &cc)) != 1)  // NOLINT(bugprone-unchecked-string-to-number-conversion)
        {
            LOG_W(TAG, "resume v4: confirm counter[%u] unparseable — discarding",
                  static_cast<unsigned>(i));
            return false;
        }
        tmpCc[i] = static_cast<uint8_t>(cc < 255U ? cc : 255U);
        while (*cursor != '\0' && *cursor != '|') { cursor++; }
    }

    // ── 5. Transition hold windows (2 × AMS_MAX_TRANSITIONS fields) ───────
    for (uint8_t i = 0U; i < ares::AMS_MAX_TRANSITIONS; i++)  // PO10-2
    {
        if (*cursor != '|')
        {
            LOG_W(TAG, "resume v4: hold flag[%u] missing — discarding",
                  static_cast<unsigned>(i));
            return false;
        }
        cursor++;
        uint32_t holding = 0U;
        // Safe: NUL-terminated checkpoint buffer; single SCNu32; return checked.
        if (static_cast<int32_t>(sscanf(cursor, "%" SCNu32, &holding)) != 1)  // NOLINT(bugprone-unchecked-string-to-number-conversion)
        {
            LOG_W(TAG, "resume v4: hold flag[%u] unparseable — discarding",
                  static_cast<unsigned>(i));
            return false;
        }
        while (*cursor != '\0' && *cursor != '|') { cursor++; }

        if (*cursor != '|')
        {
            LOG_W(TAG, "resume v4: hold elapsed[%u] missing — discarding",
                  static_cast<unsigned>(i));
            return false;
        }
        cursor++;
        uint32_t elap = 0U;
        // Safe: NUL-terminated checkpoint buffer; single SCNu32; return checked.
        if (static_cast<int32_t>(sscanf(cursor, "%" SCNu32, &elap)) != 1)  // NOLINT(bugprone-unchecked-string-to-number-conversion)
        {
            LOG_W(TAG, "resume v4: hold elapsed[%u] unparseable — discarding",
                  static_cast<unsigned>(i));
            return false;
        }
        while (*cursor != '\0' && *cursor != '|') { cursor++; }

        tmpHolding[i]  = (holding != 0U);
        tmpCondElap[i] = elap;
    }

    // ── 6. All checks passed — commit to live state ────────────────────────
    for (uint8_t vi = 0U; vi < program_.varCount; vi++)
    {
        if (tmpVarSet[vi])
        {
            program_.vars[vi].value = tmpVarVal[vi];
            program_.vars[vi].valid = tmpVarValid[vi];
        }
    }
    for (uint8_t s = 0U; s < static_cast<uint8_t>(hkCount); s++)
    {
        lastHkSlotMs_[s] = nowMs - tmpHkElap[s];
    }
    for (uint8_t s = 0U; s < static_cast<uint8_t>(logCount); s++)
    {
        lastLogSlotMs_[s] = nowMs - tmpLogElap[s];
    }
    for (uint8_t i = 0U; i < 4U; i++)
    {
        tcConfirmCount_[i] = tmpCc[i];
    }
    for (uint8_t i = 0U; i < ares::AMS_MAX_TRANSITIONS; i++)
    {
        transitionCondHolding_[i] = tmpHolding[i];
        if (tmpHolding[i])
        {
            transitionCondMetMs_[i] = nowMs - tmpCondElap[i];
        }
    }
    return true;
}

void MissionScriptEngine::restoreCheckpointV4FieldsLocked(const char* cursor, uint64_t nowMs)
{
    // cursor points to the first character of cc[0] (tcConfirmCount_[0]).
    // Format: {cc0}|{cc1}|{cc2}|{cc3}|{hold0}|{condElap0}|...|{hold3}|{condElap3}|{CRC32HEX}
    // The CRC field is the last token — stop parsing when the remaining token
    // is 8 hex chars followed by NUL (it has already been validated above).

    // Restore tcConfirmCount_[0..3].
    for (uint8_t i = 0U; i < 4U; i++)  // PO10-2
    {
        uint32_t cc = 0U;
        // Safe: cursor points into a NUL-terminated checkpoint buffer; reads a single SCNu32
        // (confirm count); return checked != 1; value clamped to [0, 255] on assignment.
        if (static_cast<int32_t>(sscanf(cursor, "%" SCNu32, &cc)) != 1)  // NOLINT(bugprone-unchecked-string-to-number-conversion)
        {
            return;
        }
        tcConfirmCount_[i] = static_cast<uint8_t>(cc < 255U ? cc : 255U);
        while (*cursor != '\0' && *cursor != '|') { cursor++; }
        if (*cursor == '|') { cursor++; }
    }

    // Restore transitionCondHolding_ and transitionCondMetMs_[0..3].
    for (uint8_t i = 0U; i < ares::AMS_MAX_TRANSITIONS; i++)  // PO10-2
    {
        uint32_t holding = 0U;
        uint32_t elap    = 0U;
        // Safe: cursor points into a NUL-terminated checkpoint buffer; reads a single SCNu32
        // (holding flag); return checked != 1; result used only as a boolean.
        if (static_cast<int32_t>(sscanf(cursor, "%" SCNu32, &holding)) != 1)  // NOLINT(bugprone-unchecked-string-to-number-conversion)
        {
            return;
        }
        while (*cursor != '\0' && *cursor != '|') { cursor++; }
        if (*cursor == '|') { cursor++; }

        // Safe: cursor points into a NUL-terminated checkpoint buffer; reads a single SCNu32
        // (elapsed ms); return checked != 1; result used only as a relative timestamp offset.
        if (static_cast<int32_t>(sscanf(cursor, "%" SCNu32, &elap)) != 1)  // NOLINT(bugprone-unchecked-string-to-number-conversion)
        {
            return;
        }
        while (*cursor != '\0' && *cursor != '|') { cursor++; }
        if (*cursor == '|') { cursor++; }

        transitionCondHolding_[i] = (holding != 0U);
        if (transitionCondHolding_[i])
        {
            transitionCondMetMs_[i] = nowMs - elap;
        }
    }
}

bool MissionScriptEngine::applyCheckpointStateLocked(
    uint64_t nowMs, uint32_t stateIdx, uint32_t execEnabled,
    uint32_t running, uint32_t status, uint32_t seq,
    uint32_t stateElapsed, uint32_t hkElapsed, uint32_t logElapsed)
{
    // ── Validate ALL inputs before touching any member variable ───────────────
    // Any early-return below must not leave the engine in a partially-mutated
    // state.  We therefore perform all checks on the raw uint32_t arguments and
    // only commit to member variables once every check has passed.

    // CERT-1: validate enum range before cast to prevent undefined behaviour.
    if (status > static_cast<uint32_t>(EngineStatus::LAST))
    {
        LOG_W(TAG, "resume: invalid status field %" PRIu32 " — discarding checkpoint", status);
        clearResumePointLocked();
        return false;
    }
    const EngineStatus resolvedStatus = static_cast<EngineStatus>(status);

    // Only a checkpoint where the engine was actively running is worth
    // restoring.  A paused (running=0) or execution-disabled (exec=0)
    // checkpoint is discarded so the caller falls through to a clean boot.
    if (running == 0U || execEnabled == 0U || resolvedStatus != EngineStatus::RUNNING)
    {
        LOG_W(TAG, "resume: non-running checkpoint (running=%" PRIu32
              " exec=%" PRIu32 " status=%" PRIu32 ") — discarding",
              running, execEnabled, status);
        clearResumePointLocked();
        return false;
    }

    // ── Commit: all inputs are valid — apply to member variables ─────────────
    currentState_ = static_cast<uint8_t>(stateIdx);
    stateEnterMs_ = nowMs - stateElapsed;
    lastHkMs_     = nowMs - hkElapsed;
    lastLogMs_    = nowMs - logElapsed;
    seq_          = static_cast<uint8_t>(seq & 0xFFU);

    // AMS-4.3.1: initialize slot timers to nowMs as a conservative fallback.
    // A v3 checkpoint will override these with precise values immediately after.
    for (uint8_t s = 0U; s < ares::AMS_MAX_HK_SLOTS; s++)
    {
        lastHkSlotMs_[s]  = nowMs;
        lastLogSlotMs_[s] = nowMs;
    }

    running_          = true;
    executionEnabled_ = true;
    status_           = resolvedStatus;

    pendingTc_           = TcCommand::NONE;
    pendingOnEnterEvent_ = false;
    pendingEventText_[0] = '\0';
    pendingEventTsMs_    = 0U;
    lastError_[0]        = '\0';
    lastCheckpointMs_    = nowMs;

    return true;
}

bool MissionScriptEngine::tryRestoreResumePointLocked(uint64_t nowMs) // NOLINT(readability-function-size)
{
    bool exists = false;
    const StorageStatus stExists = storage_.exists(ares::AMS_RESUME_PATH, exists);
    if (stExists != StorageStatus::OK || !exists)
    {
        return false;
    }

    // SINGLE-INSTANCE ASSUMPTION: this buffer is intentionally static to avoid
    // placing 512 bytes on the FreeRTOS task stack (stack space is scarce on
    // ESP32-S3).  This is safe only because the firmware runs exactly one
    // MissionScriptEngine instance — enforced by the ARES_REQUIRE in begin()
    // (ARES_REQUIRE is NOT elided with ARES_NDEBUG, unlike ARES_ASSERT).
    // If a second engine is ever added, convert this to a member array or
    // allocate from a dedicated scratch region.
    static char buf[512] = {};
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

    // Truncation guard: if readFile filled the entire usable buffer the file is
    // longer than expected (corruption or format change).  A truncated record
    // must not be parsed — the CRC check below only covers v4 records, so v1–v3
    // records would silently reach the parser with incomplete data otherwise.
    if (bytesRead >= sizeof(buf) - 1U)
    {
        LOG_W(TAG, "checkpoint: file exceeds buffer (%u B) — discarding",
              static_cast<unsigned>(sizeof(buf) - 1U));
        clearResumePointLocked();
        return false;
    }

    // ── v4: CRC32 integrity check ─────────────────────────────────────────────
    // Quick-parse the version field before trusting any field value.
    // For v4 records, the last pipe-delimited token is an 8-char uppercase-hex
    // CRC32 that covers all preceding bytes (AMS-8.5).
    {
        uint32_t quickVer = 0U;
        // Safe: buf is a NUL-terminated fixed-size checkpoint buffer; reads only the leading
        // SCNu32 version field; result used solely to select the parse path, not as an
        // array index or allocation size; return value is intentionally discarded.
        (void)sscanf(buf, "%" SCNu32, &quickVer);  // NOLINT(bugprone-unchecked-string-to-number-conversion)
        if (quickVer == 4U)
        {
            const char* lastPipe = strrchr(buf, '|');
            if (lastPipe == nullptr)
            {
                LOG_W(TAG, "checkpoint: v4 record missing CRC field — discarding");
                clearResumePointLocked();
                return false;
            }
            const char* crcStr = lastPipe + 1U;
            // Validate: exactly 8 hex chars followed by NUL.
            bool hexOk = (strnlen(crcStr, 9U) == 8U);
            for (uint8_t ci = 0U; hexOk && ci < 8U; ci++)
            {
                const char c = crcStr[ci];
                hexOk = (c >= '0' && c <= '9')
                     || (c >= 'A' && c <= 'F')
                     || (c >= 'a' && c <= 'f');
            }
            if (!hexOk)
            {
                LOG_W(TAG, "checkpoint: v4 CRC field malformed — discarding");
                clearResumePointLocked();
                return false;
            }
            const uint32_t computed = crc32Compute(buf,
                static_cast<size_t>(lastPipe - buf));
            uint32_t stored = 0U;
            // Safe: crcStr was validated above to be exactly 8 hex chars + NUL (strnlen == 8
            // and hex-char loop); SCNx32 is therefore guaranteed to match; result discarded
            // with (void) since format correctness was pre-checked.
            (void)sscanf(crcStr, "%" SCNx32, &stored);  // NOLINT(bugprone-unchecked-string-to-number-conversion)
            if (computed != stored)
            {
                LOG_W(TAG, "checkpoint: CRC mismatch (stored=0x%08" PRIX32
                      " computed=0x%08" PRIX32 ") — discarding", stored, computed);
                clearResumePointLocked();
                return false;
            }
        }
    }

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
    // Accept v1 (no vars), v2 (with vars), v3 (with vars + per-slot timers),
    // v4 (v3 + confirm/hold + CRC32).
    if (!parsed
        || (version != 1U && version != 2U && version != 3U && version != 4U))
    {
        clearResumePointLocked();
        return false;
    }

    if (!isSafeFileName(fileName))
    {
        LOG_W(TAG, "checkpoint: unsafe fileName rejected — discarding");
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

    // ── v2+: restore global variables ────────────────────────────────────────
    if (version >= 2U)
    {
        // Locate the 11th '|'-delimited token (field after the 10-field header).
        const char* cursor = buf;
        uint8_t pipes = 0U;
        while (*cursor != '\0' && pipes < 10U)
        {
            if (*cursor == '|') { pipes++; }
            cursor++;
        }

        if (version == 4U)
        {
            // v4: strict atomic restore — all sections must be complete and
            // coherent with the loaded script.  A partial record (wrong field
            // counts, truncated sections) is discarded outright (P2-3).
            if (*cursor == '\0' || !restoreCheckpointV4StrictLocked(cursor, nowMs))
            {
                LOG_W(TAG, "resume: v4 record structurally incomplete — discarding");
                deactivateLocked();        // undo applyCheckpointStateLocked
                clearResumePointLocked();
                return false;
            }
        }
        else
        {
            // v2/v3: lenient restore for backward compatibility with records
            // written by older firmware that may omit optional sections.
            if (*cursor != '\0')
            {
                cursor = restoreCheckpointVarsLocked(cursor);
            }
            if (version >= 3U && *cursor == '|')
            {
                (void)restoreCheckpointSlotsLocked(cursor + 1U, nowMs);
            }
        }
    }

    LOG_W(TAG, "resumed AMS from checkpoint: file=%s state=%s",
          activeFile_, program_.states[currentState_].name);

    // Persist immediately after restore to validate record freshness.
    (void)saveResumePointLocked(nowMs, true);
    return true;
}

} // namespace ams
} // namespace ares
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

namespace ares
{
namespace ams
{
static constexpr const char* TAG = "AMS";
static constexpr uint8_t AMS_RESUME_VERSION = 4U;
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

    appendVarsSectionLocked(record, recSize, written);
    appendSlotTimersSectionLocked(record, recSize, written, nowMs);
    appendConfirmHoldSectionLocked(record, recSize, written, nowMs);

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

void MissionScriptEngine::appendConfirmHoldSectionLocked(char*    record,
                                                         size_t   recSize,
                                                         int32_t& written,
                                                         uint64_t nowMs) const
{
    if (written <= 0) { return; }

    // tcConfirmCount_[0..3] (AMS-4.11.2 v4 checkpoint).
    const int32_t wCc = static_cast<int32_t>(snprintf(
        record + written, recSize - static_cast<size_t>(written),
        "|%" PRIu32 "|%" PRIu32 "|%" PRIu32 "|%" PRIu32,
        static_cast<uint32_t>(tcConfirmCount_[0]),
        static_cast<uint32_t>(tcConfirmCount_[1]),
        static_cast<uint32_t>(tcConfirmCount_[2]),
        static_cast<uint32_t>(tcConfirmCount_[3])));
    if (wCc > 0 && (static_cast<size_t>(written) + static_cast<size_t>(wCc)) < recSize)
    {
        written += wCc;
    }

    // transitionCondHolding_[i] and elapsed-since-condMet for each slot (AMS-4.6.1).
    for (uint8_t i = 0U; i < ares::AMS_MAX_TRANSITIONS; i++)  // PO10-2
    {
        const uint32_t condElap = transitionCondHolding_[i]
            ? static_cast<uint32_t>(nowMs - transitionCondMetMs_[i])
            : 0U;
        const int32_t wh = static_cast<int32_t>(snprintf(
            record + written,
            recSize - static_cast<size_t>(written),
            "|%" PRIu32 "|%" PRIu32,
            transitionCondHolding_[i] ? 1U : 0U,
            condElap));
        if (wh > 0 && (static_cast<size_t>(written) + static_cast<size_t>(wh)) < recSize)
        {
            written += wh;
        }
    }
}

void MissionScriptEngine::appendVarsSectionLocked(char*   record,
                                                  size_t  recSize,
                                                  int32_t& written) const
{
    if (program_.varCount == 0U || written <= 0) { return; }

    int32_t w2 = static_cast<int32_t>(snprintf(record + written,
                     recSize - static_cast<size_t>(written),
                     "|%" PRIu32, static_cast<uint32_t>(program_.varCount)));
    if (w2 > 0) { written += w2; }

    for (uint8_t vi = 0; vi < program_.varCount && written > 0; vi++)  // PO10-2
    {
        const VarEntry& v = program_.vars[vi];
        char floatBuf[24] = {};
        (void)snprintf(floatBuf, sizeof(floatBuf), "%.6g",
                       static_cast<double>(v.value));
        const int32_t w3 = static_cast<int32_t>(snprintf(record + written,
                                recSize - static_cast<size_t>(written),
                                "|%s=%s=%u",
                                v.name, floatBuf, v.valid ? 1U : 0U));
        if (w3 > 0 &&
            (static_cast<size_t>(written) + static_cast<size_t>(w3)) < recSize)
        {
            written += w3;
        }
    }
}

void MissionScriptEngine::appendSlotTimersSectionLocked(char*    record,
                                                        size_t   recSize,
                                                        int32_t& written,
                                                        uint64_t nowMs) const
{
    if (written <= 0 || currentState_ >= program_.stateCount) { return; }
    const StateDef& curSt = program_.states[currentState_];

    // HK slots: count then per-slot elapsed (AMS-4.3.1 v3 checkpoint).
    const int32_t wHk = static_cast<int32_t>(snprintf(
        record + written, recSize - static_cast<size_t>(written),
        "|%" PRIu32, static_cast<uint32_t>(curSt.hkSlotCount)));
    if (wHk > 0 && (static_cast<size_t>(written) + static_cast<size_t>(wHk)) < recSize)
    {
        written += wHk;
    }
    for (uint8_t s = 0U; s < curSt.hkSlotCount && written > 0; s++)  // PO10-2
    {
        const uint32_t slotElap = static_cast<uint32_t>(nowMs - lastHkSlotMs_[s]);
        const int32_t ws = static_cast<int32_t>(snprintf(
            record + written, recSize - static_cast<size_t>(written),
            "|%" PRIu32, slotElap));
        if (ws > 0 && (static_cast<size_t>(written) + static_cast<size_t>(ws)) < recSize)
        {
            written += ws;
        }
    }

    // LOG slots: count then per-slot elapsed.
    const int32_t wLog = static_cast<int32_t>(snprintf(
        record + written, recSize - static_cast<size_t>(written),
        "|%" PRIu32, static_cast<uint32_t>(curSt.logSlotCount)));
    if (wLog > 0 && (static_cast<size_t>(written) + static_cast<size_t>(wLog)) < recSize)
    {
        written += wLog;
    }
    for (uint8_t s = 0U; s < curSt.logSlotCount && written > 0; s++)  // PO10-2
    {
        const uint32_t slotElap = static_cast<uint32_t>(nowMs - lastLogSlotMs_[s]);
        const int32_t ws = static_cast<int32_t>(snprintf(
            record + written, recSize - static_cast<size_t>(written),
            "|%" PRIu32, slotElap));
        if (ws > 0 && (static_cast<size_t>(written) + static_cast<size_t>(ws)) < recSize)
        {
            written += ws;
        }
    }
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
    // deferred to flushPendingIoUnlocked() after the mutex is released (AMS-8.3).
    int32_t written = 0;
    if (!buildCheckpointRecordLocked(nowMs, pendingCheckpoint_.buf,
                                     sizeof(pendingCheckpoint_.buf), written))
    {
        return false;
    }

    pendingCheckpoint_.len     = written;
    pendingCheckpoint_.pending = true;
    lastCheckpointMs_          = nowMs;
    checkpointDirty_           = false;
    return true;
}

void MissionScriptEngine::clearResumePointLocked()
{
    // MUTEX INVARIANT — NO RACE WITH saveResumePointLocked / flushPendingIoUnlocked
    //
    // The `Locked` suffix on this function (and on saveResumePointLocked) is a
    // firm contract: the caller MUST hold mutex_ before entering.  FreeRTOS
    // mutexes are non-reentrant and grant exclusive ownership, so these two
    // functions can never execute concurrently on any task.
    //
    // The two-phase write design (stage under mutex, flush after release) adds a
    // subtlety: flushPendingIoUnlocked() runs *without* the mutex and calls
    // writeFile() when pendingCheckpoint_.pending == true.  The potential
    // scenario is:
    //
    //   1. Task A: saveResumePointLocked() sets pending = true, releases mutex.
    //   2. Task B: acquires mutex, calls clearResumePointLocked().
    //   3. Task A: flushPendingIoUnlocked() — would it write a stale checkpoint?
    //
    // This is prevented by the line below.  Setting pending = false here
    // (while still holding mutex_) happens-before flushPendingIoUnlocked()
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
    char line[100] = {};
    const int32_t bodyLen = static_cast<int32_t>(
        snprintf(line, sizeof(line) - 5U,  // reserve room for ",xx\n\0"
                 "%" PRIu64 ",ABORT,aborted_in_state=%s",
                 nowMs, stateName));
    if (bodyLen > 0 && static_cast<uint32_t>(bodyLen) < sizeof(line) - 5U)
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
 * @c appendFile call can be issued by @c flushPendingIoUnlocked() after the
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
}

// ── flushPendingIoUnlocked ────────────────────────────────────────────────────

/**
 * @brief Execute all file I/O staged during the previous tick or API call.
 *
 * Must be called AFTER the engine mutex has been released.  Issues all
 * @c appendFile and @c writeFile operations that were deferred by
 * @c queueAppendLocked and @c saveResumePointLocked, keeping those operations
 * outside the critical section (AMS-8.3).
 *
 * @post @c pendingAppendCount_ == 0 and @c pendingCheckpoint_.pending == false.
 */
void MissionScriptEngine::flushPendingIoUnlocked()
{
    // Flush staged log appends first (data rows before checkpoint update).
    for (uint8_t i = 0U; i < pendingAppendCount_; i++)
    {
        const PendingAppend& e = pendingAppends_[i];
        const StorageStatus st = storage_.appendFile(e.path, e.data, e.len);
        if (st == StorageStatus::OK)
        {
            // Confirm header-written flag only after a successful flush so that
            // a NO_SPACE failure leaves the flag false and allows a retry on the
            // next tick (AMS-4.3.2).
            if (e.onSuccessFlag != nullptr)
            {
                *e.onSuccessFlag = true;
            }
        }
        else
        {
            LOG_W(TAG, "deferred append [%u] to '%s' failed: %u",
                  static_cast<unsigned>(i), e.path,
                  static_cast<uint32_t>(st));
        }
    }
    pendingAppendCount_ = 0U;

    // Flush staged checkpoint write.
    if (pendingCheckpoint_.pending && pendingCheckpoint_.len > 0)
    {
        const StorageStatus st = storage_.writeFile(
            ares::AMS_RESUME_PATH,
            reinterpret_cast<const uint8_t*>(pendingCheckpoint_.buf),
            static_cast<uint32_t>(pendingCheckpoint_.len));
        if (st != StorageStatus::OK)
        {
            LOG_W(TAG, "deferred checkpoint write failed: %u",
                  static_cast<uint32_t>(st));
        }
        pendingCheckpoint_.pending = false;
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
    // MissionScriptEngine instance — enforced by the ARES_ASSERT in begin().
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
        // Locate the 11th '|'-delimited token (index 10 = varCount field).
        const char* cursor = buf;
        uint8_t pipes = 0U;
        while (*cursor != '\0' && pipes < 10U)
        {
            if (*cursor == '|') { pipes++; }
            cursor++;
        }
        if (*cursor != '\0')
        {
            cursor = restoreCheckpointVarsLocked(cursor);
        }

        // ── v3+: restore per-slot HK/LOG elapsed timers (AMS-4.3.1) ──────────
        if (version >= 3U && *cursor == '|')
        {
            cursor = restoreCheckpointSlotsLocked(cursor + 1U, nowMs);
        }

        // ── v4: restore tcConfirmCount_ and transition hold windows (AMS-8.5) ─
        if (version == 4U && *cursor == '|')
        {
            restoreCheckpointV4FieldsLocked(cursor + 1U, nowMs);
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
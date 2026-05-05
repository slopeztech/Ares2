/**
 * @file  mission_script_engine_telemetry.cpp
 * @brief AMS engine — PUS frame generation, CSV logging, and frame dispatch.
 *
 * Assembles and transmits APUS-compatible frames over the primary COM link:
 *   - HK.report   → TM Service 3  (@c MsgType::TELEMETRY, @c TelemetryPayload)
 *   - EVENT.*     → TM Service 5  (@c MsgType::EVENT,     @c EventHeader + text)
 *
 * Also writes periodic CSV log rows to LittleFS via the storage interface.
 *
 * Thread safety: all *Locked functions must be called while the engine
 * mutex is held by the calling task (Locked suffix convention).
 */

#include "ams/mission_script_engine.h"
#include "ams/mission_script_engine_helpers.h"

#include "ares_assert.h"
#include "debug/ares_log.h"

#include <cinttypes>
#include <cstdio>
#include <cstring>

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
using ares::proto::PROTOCOL_VERSION;
using ares::proto::TelemetryPayload;
using ares::proto::StatusBits;

using detail::formatScaledFloat;

static constexpr const char* TAG = "AMS";

// ── sendHkReportLocked ───────────────────────────────────────────────────────

/**
 * @brief Build and transmit a PUS TM Service 3 housekeeping frame.
 *
 * Iterates over all HK fields defined in the current state, populates a
 * @c TelemetryPayload, wraps it in an APUS @c Frame, and sends it via
 * the primary COM interface.
 *
 * @param[in] nowMs  Current millis() timestamp (placed in the payload).
 * @pre  Caller holds the engine mutex.  @c currentState_ is valid.
 * @post A TELEMETRY frame is transmitted; @c seq_ is advanced.
 */
void MissionScriptEngine::sendHkReportLocked(uint32_t nowMs)
{
    if (currentState_ >= program_.stateCount) { return; }

    const StateDef& st = program_.states[currentState_];
    ARES_ASSERT(st.hkFieldCount <= ares::AMS_MAX_HK_FIELDS);
    if (!st.hasHkEvery || st.hkFieldCount == 0U) { return; }

    TelemetryPayload tm = {};
    tm.timestampMs = nowMs;
    tm.flightPhase = static_cast<uint8_t>(currentState_);
    tm.statusBits  = buildStatusBitsLocked();

    for (uint8_t i = 0; i < st.hkFieldCount; i++)
    {
        applyHkFieldToPayloadLocked(st.hkFields[i], tm);
    }

    // ── GPS satellite count (APUS-6, gap #10) ──────────────────────────────
    tm.gpsSats = readGpsSatsLocked();

    // ── Vertical velocity (APUS-6, gap #9) ────────────────────────────────
    // Derived from consecutive barometric altitude samples.  Suppressed on
    // the first sample to avoid a spurious spike from a zero baseline.
    if (hasPrevAlt_ && (nowMs != prevAltMs_))
    {
        const float dtS      = static_cast<float>(nowMs - prevAltMs_) * 0.001f;
        const float dAlt     = tm.altitudeAglM - prevAltM_;
        float       velMs    = dAlt / dtS;
        if (velMs >  500.0f) { velMs =  500.0f; }  // Saturate ± 500 m/s
        if (velMs < -500.0f) { velMs = -500.0f; }
        tm.verticalVelMs = velMs;
    }
    prevAltM_   = tm.altitudeAglM;
    prevAltMs_  = nowMs;
    hasPrevAlt_ = true;

    // ── ST[12] monitoring evaluation (APUS-12.4) ──────────────────────────────
    // Evaluate against the raw (pre-delta) values so limits are in real units.
    evaluateMonitoringLocked(tm, nowMs);

    // ── Delta encoding (APUS-3.3, gap #12) ────────────────────────────────
    // Every kDeltaResyncInterval frames (or on first frame) transmit absolute
    // values and update the baseline.  All other frames transmit deltas with
    // StatusBits.deltaFrame = 1.  Force re-sync on saturation overflow.
    const bool forceResync = !deltaBaseValid_ ||
                             ((hkTxCount_ % kDeltaResyncInterval) == 0U);
    if (!forceResync)
    {
        const float dAlt   = tm.altitudeAglM - deltaBaseAlt_;
        const float dPress = tm.pressurePa   - deltaBasePress_;
        const bool  overflow = (dAlt   >  kMaxDeltaAltM)  || (dAlt   < -kMaxDeltaAltM) ||
                               (dPress >  kMaxDeltaPressPa)|| (dPress < -kMaxDeltaPressPa);
        if (!overflow)
        {
            tm.altitudeAglM          = dAlt;
            tm.pressurePa            = dPress;
            tm.statusBits.deltaFrame = 1U;
        }
        else
        {
            // Overflow: fall through to absolute; reset baseline below.
            LOG_D(TAG, "HK delta overflow dAlt=%.1f dP=%.1f \u2014 forced resync",
                  static_cast<double>(dAlt), static_cast<double>(dPress));
        }
    }
    if (forceResync || tm.statusBits.deltaFrame == 0U)
    {
        deltaBaseAlt_   = tm.altitudeAglM;
        deltaBasePress_ = tm.pressurePa;
        deltaBaseValid_ = true;
    }
    hkTxCount_++;

    Frame frame = {};
    frame.ver   = PROTOCOL_VERSION;
    frame.flags = 0;
    frame.node  = program_.nodeId;
    frame.type  = MsgType::TELEMETRY;
    frame.seq   = seq_++;
    frame.len   = static_cast<uint8_t>(sizeof(TelemetryPayload));
    memcpy(frame.payload, &tm, sizeof(tm));

    if (!sendFrameLocked(frame))
    {
        LOG_W(TAG, "HK frame send failed");
    }
    else
    {
        LOG_D(TAG, "HK frame sent seq=%u delta=%u len=%u",
              frame.seq, static_cast<unsigned>(tm.statusBits.deltaFrame), frame.len);
    }
}

// ── sendHkReportSlotLocked ───────────────────────────────────────────────────

/**
 * @brief Build and transmit a PUS TM Service 3 housekeeping frame for one slot.
 *
 * AMS-4.3.1: each @c every block creates an independent @c HkSlot.
 * This function builds a @c TelemetryPayload from the slot's field list
 * and transmits it, independent of the legacy single-slot path.
 *
 * @param[in] nowMs  Current millis() timestamp.
 * @param[in] slot   The HK slot to transmit.
 * @pre  Caller holds the engine mutex.
 */
void MissionScriptEngine::sendHkReportSlotLocked(uint32_t nowMs, const HkSlot& slot)
{
    ARES_ASSERT(slot.fieldCount <= ares::AMS_MAX_HK_FIELDS);
    if (slot.everyMs == 0U || slot.fieldCount == 0U) { return; }

    TelemetryPayload tm = {};
    tm.timestampMs = nowMs;
    tm.flightPhase = static_cast<uint8_t>(currentState_);
    tm.statusBits  = buildStatusBitsLocked();

    for (uint8_t i = 0; i < slot.fieldCount; i++)
    {
        applyHkFieldToPayloadLocked(slot.fields[i], tm);
    }

    // ── GPS satellite count (APUS-6, gap #10) ──────────────────────────────
    tm.gpsSats = readGpsSatsLocked();

    // ── Vertical velocity (APUS-6, gap #9) ────────────────────────────────
    if (hasPrevAlt_ && (nowMs != prevAltMs_))
    {
        const float dtS      = static_cast<float>(nowMs - prevAltMs_) * 0.001f;
        const float dAlt     = tm.altitudeAglM - prevAltM_;
        float       velMs    = dAlt / dtS;
        if (velMs >  500.0f) { velMs =  500.0f; }
        if (velMs < -500.0f) { velMs = -500.0f; }
        tm.verticalVelMs = velMs;
    }
    prevAltM_   = tm.altitudeAglM;
    prevAltMs_  = nowMs;
    hasPrevAlt_ = true;
    // ── ST[12] monitoring evaluation (APUS-12.4) ──────────────────────────
    // Evaluate against the raw (pre-delta) values so limits are in real units.
    evaluateMonitoringLocked(tm, nowMs);
    // ── Delta encoding (APUS-3.3, gap #12) ────────────────────────────────
    const bool forceResync = !deltaBaseValid_ ||
                             ((hkTxCount_ % kDeltaResyncInterval) == 0U);
    if (!forceResync)
    {
        const float dAlt   = tm.altitudeAglM - deltaBaseAlt_;
        const float dPress = tm.pressurePa   - deltaBasePress_;
        const bool  overflow = (dAlt   >  kMaxDeltaAltM)  || (dAlt   < -kMaxDeltaAltM) ||
                               (dPress >  kMaxDeltaPressPa)|| (dPress < -kMaxDeltaPressPa);
        if (!overflow)
        {
            tm.altitudeAglM          = dAlt;
            tm.pressurePa            = dPress;
            tm.statusBits.deltaFrame = 1U;
        }
        else
        {
            LOG_D(TAG, "HK slot delta overflow dAlt=%.1f dP=%.1f \u2014 forced resync",
                  static_cast<double>(dAlt), static_cast<double>(dPress));
        }
    }
    if (forceResync || tm.statusBits.deltaFrame == 0U)
    {
        deltaBaseAlt_   = tm.altitudeAglM;
        deltaBasePress_ = tm.pressurePa;
        deltaBaseValid_ = true;
    }
    hkTxCount_++;

    Frame frame = {};
    frame.ver   = PROTOCOL_VERSION;
    frame.flags = 0;
    frame.node  = program_.nodeId;
    frame.type  = MsgType::TELEMETRY;
    frame.seq   = seq_++;
    frame.len   = static_cast<uint8_t>(sizeof(TelemetryPayload));
    memcpy(frame.payload, &tm, sizeof(tm));

    if (!sendFrameLocked(frame))
    {
        LOG_W(TAG, "HK slot frame send failed");
    }
    else
    {
        LOG_D(TAG, "HK slot frame sent seq=%u delta=%u len=%u",
              frame.seq, static_cast<unsigned>(tm.statusBits.deltaFrame), frame.len);
    }
}

// ── applyHkFieldToPayloadLocked ──────────────────────────────────────────────

// ── buildStatusBitsLocked ────────────────────────────────────────────────────

/**
 * @brief Derive the wire-protocol StatusBits from the engine's current runtime
 *        state.
 *
 * Called from every send path that produces a @c TelemetryPayload so that the
 * GCS always receives accurate status flags.  Complies with APUS-6.
 *
 * - @c armed      true when the engine is in the RUNNING state.
 * - @c fcsActive  true when execution is enabled (not paused).
 * - @c gpsValid   true when at least one registered GPS driver reports hasFix().
 * - @c pyroAFired / @c pyroBFired set via notifyPyroFired().
 *
 * @pre  Caller holds mutex_.
 * @return StatusBits struct with reserved bits cleared.
 */
ares::proto::StatusBits MissionScriptEngine::buildStatusBitsLocked() const
{
    StatusBits bits = {};
    bits.armed    = (status_ == EngineStatus::RUNNING) ? 1U : 0U;
    bits.fcsActive = executionEnabled_ ? 1U : 0U;

    // gpsValid: true if any registered GPS driver has a current fix.
    bits.gpsValid = 0U;
    for (uint8_t i = 0U; i < gpsCount_; ++i)
    {
        if ((gpsDrivers_[i].iface != nullptr) && gpsDrivers_[i].iface->hasFix())
        {
            bits.gpsValid = 1U;
            break;
        }
    }

    bits.pyroAFired = pyroAFired_ ? 1U : 0U;
    bits.pyroBFired = pyroBFired_ ? 1U : 0U;
    bits.reserved   = 0U;
    return bits;
}

// ── inferEventId ────────────────────────────────────────────────────────────

/**
 * @brief Infer the APUS-8 EventId from an AMS EventVerb.
 *
 * Used when the exact EventId cannot be determined at the call site,
 * for example in user-defined task rule events (APUS-8).
 *
 * @param[in] verb  AMS event verb (INFO, WARN, or ERROR).
 * @return Corresponding APUS-8 EventId.
 */
/*static*/ ares::proto::EventId MissionScriptEngine::inferEventId(EventVerb verb)
{
    if (verb == EventVerb::INFO)  { return ares::proto::EventId::PHASE_CHANGE;   }
    if (verb == EventVerb::WARN)  { return ares::proto::EventId::SENSOR_FAILURE; }
    return ares::proto::EventId::FPL_VIOLATION;
}

// ── readGpsSatsLocked ────────────────────────────────────────────────────────

/**
 * @brief Return the satellite count from the first GPS driver that has a fix.
 *
 * Iterates registered GPS drivers in registration order and returns the
 * @c GpsReading::satellites of the first driver whose @c read() succeeds.
 * Returns 0 if no GPS driver is registered or none has a valid fix (APUS-6).
 *
 * @pre  Caller holds the engine mutex.
 * @return Number of satellites in use, or 0 if unavailable.
 */
uint8_t MissionScriptEngine::readGpsSatsLocked() const
{
    for (uint8_t i = 0U; i < gpsCount_; ++i)
    {
        if (gpsDrivers_[i].iface == nullptr) { continue; }
        GpsReading r = {};
        if (gpsDrivers_[i].iface->read(r) == GpsStatus::OK)
        {
            return r.satellites;
        }
    }
    return 0U;
}

// ── applyHkFieldToPayloadLocked ──────────────────────────────────────────────

/**
 * @brief Write a single HK field value into the appropriate @c TelemetryPayload
 *        member.
 *
 * GPS altitude is mapped to @c gpsAltDm (decimetres); BARO altitude to
 * @c altitudeAglM.  Individual IMU axes and speed have no dedicated payload
 * field (APUS-3.6 fixed layout) and are silently skipped for HK reports
 * (they appear in CSV LOG reports via appendLogReportLocked()).
 *
 * @param[in]  f   HK field descriptor (alias + field enum).
 * @param[out] tm  Telemetry payload to update.
 * @pre  Caller holds the engine mutex.
 */
void MissionScriptEngine::applyHkFieldToPayloadLocked(
    const HkField&               f,
    ares::proto::TelemetryPayload& tm) const
{
    const AliasEntry* ae = findAliasLocked(f.alias);
    float val = 0.0f;
    if (!readSensorFloatLocked(f.alias, f.field, val)) { return; }

    // GPS altitude → gpsAltDm;  BARO altitude → altitudeAglM.
    if (f.field == SensorField::ALT && ae != nullptr)
    {
        if (ae->kind == PeripheralKind::GPS)
        {
            const float dm = val * 10.0f;
            if (dm > 0.0f)
            {
                tm.gpsAltDm = (dm > static_cast<float>(UINT16_MAX))
                            ? UINT16_MAX
                            : static_cast<uint16_t>(dm);
            }
        }
        else
        {
            tm.altitudeAglM = val;
        }
        return;
    }

    switch (f.field)
    {
    case SensorField::LAT:      tm.latitudeE7  = static_cast<int32_t>(val * 10000000.0f); break;
    case SensorField::LON:      tm.longitudeE7 = static_cast<int32_t>(val * 10000000.0f); break;
    case SensorField::TEMP:     tm.temperatureC = val;  break;
    case SensorField::PRESSURE: tm.pressurePa   = val;  break;
    case SensorField::ACCEL_MAG:tm.accelMag     = val;  break;
    // Individual accel/gyro axes and IMU temp have no dedicated field in
    // TelemetryPayload (APUS-3.6 fixed layout); available in LOG reports.
    case SensorField::ACCEL_X:
    case SensorField::ACCEL_Y:
    case SensorField::ACCEL_Z:
    case SensorField::GYRO_X:
    case SensorField::GYRO_Y:
    case SensorField::GYRO_Z:
    case SensorField::IMU_TEMP:
    case SensorField::SPEED:
    default:
        break;
    }
}

// ── appendLogReportLocked ────────────────────────────────────────────────────

/**
 * @brief Append a CSV data row to the mission log file.
 *
 * Writes a header row on the first call (column names from @c logFields[].label).
 * Each subsequent call writes one data row with the current timestamp,
 * state name, and sensor values for all configured LOG fields.
 *
 * @param[in] nowMs  Current millis() timestamp (first CSV column).
 * @pre  Caller holds the engine mutex.  @c logPath_ is set.
 * @post One or two lines are appended to @c logPath_ on the storage device.
 */
void MissionScriptEngine::appendLogReportLocked(uint32_t nowMs)
{
    if (currentState_ >= program_.stateCount) { return; }

    const StateDef& st = program_.states[currentState_];
    ARES_ASSERT(st.logFieldCount <= ares::AMS_MAX_HK_FIELDS);
    if (!st.hasLogEvery || st.logFieldCount == 0U || logPath_[0] == '\0') { return; }

    if (!writeLogHeaderIfNeededLocked(st))
    {
        return;
    }

    char line[256] = {};
    uint32_t len = 0U;
    if (!buildLogDataRowLocked(st, nowMs, line, sizeof(line), len))
    {
        return;
    }

    const StorageStatus stAppend = storage_.appendFile(
        logPath_, reinterpret_cast<const uint8_t*>(line), len);
    if (stAppend != StorageStatus::OK)
    {
        LOG_W(TAG, "append mission log failed: %u",
              static_cast<uint32_t>(stAppend));
    }
}

// ── appendLogReportSlotLocked ────────────────────────────────────────────────

/**
 * @brief Append a CSV data row for one log_every slot (AMS-4.3.1).
 *
 * Each @c log_every slot has an independent cadence and field list.
 * A per-slot CSV header is written on the first call for that slot,
 * then data rows are appended on subsequent calls.
 *
 * @param[in] nowMs    Current millis() timestamp.
 * @param[in] slot     The LOG slot descriptor (fields + interval).
 * @param[in] slotIdx  Zero-based slot index (used to track header state).
 * @pre  Caller holds the engine mutex.  @c logPath_ is set.
 */
void MissionScriptEngine::appendLogReportSlotLocked(uint32_t      nowMs,
                                                    const HkSlot& slot,
                                                    uint8_t       slotIdx)
{
    if (currentState_ >= program_.stateCount) { return; }
    ARES_ASSERT(slot.fieldCount <= ares::AMS_MAX_HK_FIELDS);
    if (slot.everyMs == 0U || slot.fieldCount == 0U || logPath_[0] == '\0') { return; }
    ARES_ASSERT(slotIdx < ares::AMS_MAX_HK_SLOTS);

    const StateDef& st = program_.states[currentState_];

    // Write per-slot header if this is the first row for this slot.
    if (!logSlotHeaderWritten_[slotIdx])
    {
        char hLine[256] = {};
        const int hLen = snprintf(hLine, sizeof(hLine), "t_ms,state,slot");
        if (hLen > 0)
        {
            uint32_t hPos = static_cast<uint32_t>(hLen);
            for (uint8_t i = 0; i < slot.fieldCount && hPos < sizeof(hLine) - 2U; i++)
            {
                const int n = snprintf(&hLine[hPos], sizeof(hLine) - hPos,
                                       ",%s", slot.fields[i].label);
                if (n <= 0) { break; }
                hPos += static_cast<uint32_t>(n);
            }
            const int nl = snprintf(&hLine[hPos], sizeof(hLine) - hPos, "\n");
            if (nl > 0)
            {
                const uint32_t hTot = hPos + static_cast<uint32_t>(nl);
                storage_.appendFile(logPath_,
                                    reinterpret_cast<const uint8_t*>(hLine), hTot);
                logSlotHeaderWritten_[slotIdx] = true;
            }
        }
    }

    // Build data row: t_ms, state_name, slot_index, field values.
    char line[256] = {};
    int head = snprintf(line, sizeof(line), "%" PRIu32 ",%s,%u",
                        nowMs, st.name, static_cast<uint32_t>(slotIdx));
    if (head <= 0) { return; }

    uint32_t pos = static_cast<uint32_t>(head);
    for (uint8_t i = 0; i < slot.fieldCount && pos < sizeof(line) - 2U; i++)
    {
        char value[32] = {};
        const bool hasValue = formatHkFieldValueLocked(slot.fields[i], value, sizeof(value));
        const int n = hasValue
                    ? snprintf(&line[pos], sizeof(line) - pos, ",%s", value)
                    : snprintf(&line[pos], sizeof(line) - pos, ",");
        if (n <= 0) { break; }
        pos += static_cast<uint32_t>(n);
    }

    const int tail = snprintf(&line[pos], sizeof(line) - pos, "\n");
    if (tail <= 0) { return; }
    const uint32_t totalLen = pos + static_cast<uint32_t>(tail);

    const StorageStatus stAppend = storage_.appendFile(
        logPath_, reinterpret_cast<const uint8_t*>(line), totalLen);
    if (stAppend != StorageStatus::OK)
    {
        LOG_W(TAG, "append log slot %u failed: %u",
              static_cast<uint32_t>(slotIdx),
              static_cast<uint32_t>(stAppend));
    }
}

bool MissionScriptEngine::writeLogHeaderIfNeededLocked(const StateDef& st)
{
    if (logHeaderWritten_)
    {
        return true;
    }

    char line[256] = {};
    const int hLen = snprintf(line, sizeof(line), "t_ms,state");
    if (hLen <= 0)
    {
        return false;
    }

    uint32_t hPos = static_cast<uint32_t>(hLen);
    for (uint8_t i = 0; i < st.logFieldCount; i++)
    {
        const int n = snprintf(&line[hPos], sizeof(line) - hPos, ",%s", st.logFields[i].label);
        if (n <= 0) { break; }
        hPos += static_cast<uint32_t>(n);
        if (hPos >= sizeof(line) - 2U) { break; }
    }

    const int nl = snprintf(&line[hPos], sizeof(line) - hPos, "\n");
    if (nl <= 0)
    {
        return false;
    }

    const uint32_t hTot = hPos + static_cast<uint32_t>(nl);
    storage_.appendFile(logPath_, reinterpret_cast<const uint8_t*>(line), hTot);
    logHeaderWritten_ = true;
    return true;
}

bool MissionScriptEngine::buildLogDataRowLocked(const StateDef& st,
                                                uint32_t        nowMs,
                                                char*           outLine,
                                                uint32_t        outSize,
                                                uint32_t&       outLen) const
{
    int headLen = snprintf(outLine, outSize, "%" PRIu32 ",%s", nowMs, st.name);
    if (headLen <= 0)
    {
        return false;
    }

    uint32_t pos = static_cast<uint32_t>(headLen);
    for (uint8_t i = 0; i < st.logFieldCount; i++)
    {
        char value[32] = {};
        const bool hasValue = formatHkFieldValueLocked(st.logFields[i], value, sizeof(value));
        const int n = hasValue
                    ? snprintf(&outLine[pos], outSize - pos, ",%s", value)
                    : snprintf(&outLine[pos], outSize - pos, ",");
        if (n <= 0)
        {
            return false;
        }
        pos += static_cast<uint32_t>(n);
        if (pos >= outSize - 2U)
        {
            break;
        }
    }

    const int tail = snprintf(&outLine[pos], outSize - pos, "\n");
    if (tail <= 0)
    {
        return false;
    }
    outLen = pos + static_cast<uint32_t>(tail);
    return true;
}

// ── ensureLogFileLocked ──────────────────────────────────────────────────────

/**
 * @brief Create (or truncate) the mission CSV log file for a script.
 *
 * Derives the log filename from @p fileName by sanitising non-alphanumeric
 * characters, then creates an empty file at @c LOG_DIR/mission_<name>.csv.
 * The CSV header row is deferred to the first appendLogReportLocked() call
 * so the columns match the actual logging state.
 *
 * @param[in] fileName  Active script filename (used as the log name base).
 * @return @c true if the log file was created successfully.
 * @pre  Caller holds the engine mutex.  @p fileName is non-null.
 * @post @c logPath_ and @c logHeaderWritten_ are set.
 */
bool MissionScriptEngine::ensureLogFileLocked(const char* fileName)
{
    if (fileName == nullptr || fileName[0] == '\0') { return false; }

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
                                 ares::LOG_DIR, safeName);
    if (written <= 0 || static_cast<uint32_t>(written) >= sizeof(logPath_))
    {
        return false;
    }

    logHeaderWritten_ = false;
    memset(logSlotHeaderWritten_, 0, sizeof(logSlotHeaderWritten_));
    const StorageStatus stWrite = storage_.writeFile(
        logPath_, reinterpret_cast<const uint8_t*>(""), 0U);
    return stWrite == StorageStatus::OK;
}

// ── formatHkFieldValueLocked ─────────────────────────────────────────────────

/**
 * @brief Format a single HK/LOG field value as a fixed-point decimal string.
 *
 * Selects the decimal precision based on the field type:
 *   - LAT/LON: 7 decimal places
 *   - ACCEL_* / GYRO_*: 3 decimal places
 *   - all others: 2 decimal places
 *
 * Returns @c "nan" (still writes @c true) when the driver read fails, to
 * preserve CSV column alignment.
 *
 * @param[in]  f        Field descriptor.
 * @param[out] out      Output buffer (NUL-terminated).
 * @param[in]  outSize  Capacity of @p out in bytes.
 * @return @c true on success (including the nan fallback case).
 * @pre  Caller holds the engine mutex.
 */
bool MissionScriptEngine::formatHkFieldValueLocked(const HkField& f,
                                                   char*          out,
                                                   uint32_t       outSize) const
{
    if (out == nullptr || outSize == 0U) { return false; }

    float val = 0.0f;
    if (!readSensorFloatLocked(f.alias, f.field, val))
    {
        strncpy(out, "nan", outSize - 1U);
        out[outSize - 1U] = '\0';
        return true;  // write placeholder so CSV columns stay aligned
    }

    uint8_t decimals = 2U;
    switch (f.field)
    {
    case SensorField::LAT:
    case SensorField::LON:
        decimals = 7U; break;
    case SensorField::ACCEL_X:
    case SensorField::ACCEL_Y:
    case SensorField::ACCEL_Z:
    case SensorField::ACCEL_MAG:
    case SensorField::GYRO_X:
    case SensorField::GYRO_Y:
    case SensorField::GYRO_Z:
        decimals = 3U; break;
    default:
        decimals = 2U; break;
    }

    return formatScaledFloat(val, decimals, out, outSize);
}

// ── sendEventLocked ──────────────────────────────────────────────────────────

/**
 * @brief Build and transmit a PUS TM Service 5 event frame.
 *
 * Constructs an @c EventHeader with the @c PHASE_CHANGE event ID (APUS-8)
 * and appends the event text string to the payload.
 *
 * @param[in] verb   Severity of the event (@c INFO, @c WARN, or @c ERROR).
 * @param[in] text   NUL-terminated event description text.
 * @param[in] nowMs  Current millis() timestamp (placed in the header).
 * @pre  Caller holds the engine mutex.  @p text != nullptr.
 * @post An EVENT frame is transmitted; @c seq_ is advanced.
 */
void MissionScriptEngine::sendEventLocked(EventVerb         verb,
                                          ares::proto::EventId id,
                                          const char*       text,
                                          uint32_t          nowMs)
{
    ARES_ASSERT(text != nullptr);

    EventHeader header = {};
    header.timestampMs = nowMs;
    // APUS-8: use the caller-supplied EventId so each event class is distinct.
    header.eventId = static_cast<uint8_t>(id);

    if      (verb == EventVerb::INFO) { header.severity = static_cast<uint8_t>(EventSeverity::INFO); }
    else if (verb == EventVerb::WARN) { header.severity = static_cast<uint8_t>(EventSeverity::WARN); }
    else                              { header.severity = static_cast<uint8_t>(EventSeverity::ERR);  }

    uint8_t payload[MAX_PAYLOAD_LEN] = {};
    memcpy(payload, &header, sizeof(header));

    const uint8_t maxText = static_cast<uint8_t>(MAX_PAYLOAD_LEN - sizeof(header));
    const uint8_t textLen = static_cast<uint8_t>(strnlen(text, maxText));
    if (textLen > 0U)
    {
        memcpy(&payload[sizeof(header)], text, textLen);
    }

    Frame frame = {};
    frame.ver   = PROTOCOL_VERSION;
    frame.flags = 0;
    frame.node  = program_.nodeId;
    frame.type  = MsgType::EVENT;
    frame.seq   = seq_++;
    frame.len   = static_cast<uint8_t>(sizeof(header) + textLen);
    memcpy(frame.payload, payload, frame.len);

    if (!sendFrameLocked(frame))
    {
        LOG_W(TAG, "EVENT frame send failed");
    }
    else
    {
        LOG_D(TAG, "EVENT frame sent seq=%u len=%u", frame.seq, frame.len);
    }
}

// ── ST[12] on-board monitoring ────────────────────────────────────────────────

/*static*/ float MissionScriptEngine::extractMonitorParam(
    ares::proto::MonitorParamId          id,
    const ares::proto::TelemetryPayload& tm)
{
    switch (id)
    {
    case ares::proto::MonitorParamId::ALTITUDE_AGL_M:  return tm.altitudeAglM;
    case ares::proto::MonitorParamId::VERTICAL_VEL_MS: return tm.verticalVelMs;
    case ares::proto::MonitorParamId::ACCEL_MAG:       return tm.accelMag;
    case ares::proto::MonitorParamId::PRESSURE_PA:     return tm.pressurePa;
    case ares::proto::MonitorParamId::TEMPERATURE_C:   return tm.temperatureC;
    default:                                           return 0.0f;
    }
}

/**
 * @brief Evaluate all enabled monitoring slots against the current TM payload.
 *
 * Implements the APUS-12 state machine:
 *   ENABLED → consecutiveHit increments on violation; resets on normal.
 *   When consecutiveHit >= def.consecutiveRequired → ALARM + EVENT frame.
 *   ALARM   → self-heals to ENABLED when value returns within limits.
 *
 * Disabled slots are skipped without touching any other state (APUS-12.5).
 *
 * @pre  Caller holds the engine mutex.
 */
void MissionScriptEngine::evaluateMonitoringLocked(
    const ares::proto::TelemetryPayload& tm,
    uint32_t                             nowMs)
{
    for (uint8_t i = 0U; i < kMaxMonitorSlots; ++i)
    {
        MonitoringSlot& slot = monitorSlots_[i];

        // APUS-12.5: skip disabled slots entirely.
        if (!slot.def.enabled ||
            slot.state == ares::proto::MonitoringState::MON_DISABLED)
        {
            continue;
        }

        const float value = extractMonitorParam(slot.def.parameterId, tm);
        const bool  inViolation = (value < slot.def.lowLimit) ||
                                  (value > slot.def.highLimit);

        if (inViolation)
        {
            // Increment consecutive counter — saturate at consecutiveRequired
            // to avoid rollover (CERT-4: no silent wrap).
            if (slot.consecutiveHit < slot.def.consecutiveRequired)
            {
                slot.consecutiveHit++;
            }
            // Transition ENABLED → ALARM after N consecutive violations (APUS-12.2).
            if (slot.state == ares::proto::MonitoringState::MON_ENABLED &&
                slot.consecutiveHit >= slot.def.consecutiveRequired)
            {
                slot.state = ares::proto::MonitoringState::MON_ALARM;
                LOG_W(TAG,
                      "MON ALARM slot=%u paramId=0x%02X value=%.2f "
                      "[%.2f..%.2f] consec=%u",
                      static_cast<unsigned>(i),
                      static_cast<unsigned>(
                          static_cast<uint8_t>(slot.def.parameterId)),
                      static_cast<double>(value),
                      static_cast<double>(slot.def.lowLimit),
                      static_cast<double>(slot.def.highLimit),
                      static_cast<unsigned>(slot.consecutiveHit));

                // APUS-8: emit FPL_VIOLATION event for limit breaches (APUS-12).
                sendEventLocked(EventVerb::WARN,
                                ares::proto::EventId::FPL_VIOLATION,
                                "MON:ALARM",
                                nowMs);
            }
        }
        else
        {
            // Value within limits: reset counter.
            slot.consecutiveHit = 0U;
            // Self-heal ALARM → ENABLED when limits are satisfied again.
            if (slot.state == ares::proto::MonitoringState::MON_ALARM)
            {
                slot.state = ares::proto::MonitoringState::MON_ENABLED;
                LOG_I(TAG,
                      "MON RECOVERED slot=%u paramId=0x%02X value=%.2f",
                      static_cast<unsigned>(i),
                      static_cast<unsigned>(
                          static_cast<uint8_t>(slot.def.parameterId)),
                      static_cast<double>(value));
            }
        }
    }
}

/**
 * @brief Map a ST[20] ConfigParamId threshold to the appropriate
 *        MonitoringSlot and update its limit (APUS-12.1, APUS-16 bridge).
 *
 * @param[in] id     Config parameter identifier (monitoring params only).
 * @param[in] value  New threshold value.
 * @return true if the parameter was recognised and applied.
 */
bool MissionScriptEngine::configureMonitorFromParam(ares::proto::ConfigParamId id,
                                                    float                      value)
{
    // Slot 0 = Altitude, Slot 1 = Accel, Slot 2 = Temperature.
    switch (id)
    {
    case ares::proto::ConfigParamId::MONITOR_ALT_HIGH_M:
        monitorSlots_[0].def.highLimit = value;
        monitorSlots_[0].consecutiveHit = 0U;  // reset on definition change
        return true;
    case ares::proto::ConfigParamId::MONITOR_ALT_LOW_M:
        monitorSlots_[0].def.lowLimit = value;
        monitorSlots_[0].consecutiveHit = 0U;
        return true;
    case ares::proto::ConfigParamId::MONITOR_ACCEL_HIGH:
        monitorSlots_[1].def.highLimit = value;
        monitorSlots_[1].consecutiveHit = 0U;
        return true;
    case ares::proto::ConfigParamId::MONITOR_TEMP_HIGH_C:
        monitorSlots_[2].def.highLimit = value;
        monitorSlots_[2].consecutiveHit = 0U;
        return true;
    case ares::proto::ConfigParamId::MONITOR_TEMP_LOW_C:
        monitorSlots_[2].def.lowLimit = value;
        monitorSlots_[2].consecutiveHit = 0U;
        return true;
    default:
        return false;
    }
}

// ── getScriptRadioConfig ─────────────────────────────────────────────────────

/**
 * @brief Return the script-declared override for @p id, if any (AMS-4.13).
 *
 * @param[in]  id     Parameter identifier.
 * @param[out] value  Populated with the script value when @c true is returned.
 * @return @c true if the currently loaded script declared an override for @p id.
 * @pre  The engine may be locked or unlocked; the @c program_ struct is
 *       immutable while the script is armed.
 */
bool MissionScriptEngine::getScriptRadioConfig(ares::proto::ConfigParamId id,
                                               float&                     value) const
{
    using Id = ares::proto::ConfigParamId;
    const uint8_t idx = static_cast<uint8_t>(id) -
                        static_cast<uint8_t>(Id::FIRST);
    if (idx >= Program::kRadioConfigCount)
    {
        return false;
    }
    if (!program_.radioConfig[idx].set)
    {
        return false;
    }
    value = program_.radioConfig[idx].value;
    return true;
}

// ── sendFrameLocked ──────────────────────────────────────────────────────────

/**
 * @brief Encode an APUS @c Frame and transmit it via the primary COM interface.
 *
 * @param[in] frame  Fully populated @c Frame to send.
 * @return @c true if @c RadioInterface::send() returned @c RadioStatus::OK.
 * @pre  Caller holds the engine mutex.  @c frame.len <= MAX_PAYLOAD_LEN.
 */
bool MissionScriptEngine::sendFrameLocked(const Frame& frame)
{
    ARES_ASSERT(frame.len <= MAX_PAYLOAD_LEN);

    if (primaryCom_ == nullptr) { return false; }

    uint8_t wire[MAX_FRAME_LEN] = {};
    const uint16_t wireLen = ares::proto::encode(frame, wire, sizeof(wire));
    if (wireLen == 0U) { return false; }

    const RadioStatus rs = primaryCom_->send(wire, wireLen);
    return rs == RadioStatus::OK;
}

} // namespace ams
} // namespace ares

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
    tm.statusBits  = {};

    for (uint8_t i = 0; i < st.hkFieldCount; i++)
    {
        applyHkFieldToPayloadLocked(st.hkFields[i], tm);
    }

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
        LOG_D(TAG, "HK frame sent seq=%u len=%u", frame.seq, frame.len);
    }
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

    char line[256] = {};

    // Write the CSV header row once per log file (on the first log write).
    if (!logHeaderWritten_)
    {
        int hLen = snprintf(line, sizeof(line), "t_ms,state");
        if (hLen > 0)
        {
            uint32_t hPos = static_cast<uint32_t>(hLen);
            for (uint8_t i = 0; i < st.logFieldCount; i++)
            {
                const int n = snprintf(&line[hPos], sizeof(line) - hPos,
                                       ",%s", st.logFields[i].label);
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
                           "%" PRIu32 ",%s", nowMs, st.name);
    if (headLen <= 0) { return; }

    uint32_t pos = static_cast<uint32_t>(headLen);

    for (uint8_t i = 0; i < st.logFieldCount; i++)
    {
        char value[32] = {};
        if (!formatHkFieldValueLocked(st.logFields[i], value, sizeof(value)))
        {
            // Write an empty field to preserve column alignment.
            const int n = snprintf(&line[pos], sizeof(line) - pos, ",");
            if (n <= 0) { break; }
            pos += static_cast<uint32_t>(n);
            continue;
        }

        const int n = snprintf(&line[pos], sizeof(line) - pos, ",%s", value);
        if (n <= 0) { break; }

        pos += static_cast<uint32_t>(n);
        if (pos >= sizeof(line) - 2U) { break; }
    }

    const int tail = snprintf(&line[pos], sizeof(line) - pos, "\n");
    if (tail <= 0) { return; }

    const uint32_t len = pos + static_cast<uint32_t>(tail);
    const StorageStatus stAppend = storage_.appendFile(
        logPath_, reinterpret_cast<const uint8_t*>(line), len);
    if (stAppend != StorageStatus::OK)
    {
        LOG_W(TAG, "append mission log failed: %u",
              static_cast<uint32_t>(stAppend));
    }
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
void MissionScriptEngine::sendEventLocked(EventVerb   verb,
                                          const char* text,
                                          uint32_t    nowMs)
{
    ARES_ASSERT(text != nullptr);

    EventHeader header = {};
    header.timestampMs = nowMs;
    // APUS-8: AMS events represent flight-phase transitions → PHASE_CHANGE (0x02)
    header.eventId = static_cast<uint8_t>(EventId::PHASE_CHANGE);

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

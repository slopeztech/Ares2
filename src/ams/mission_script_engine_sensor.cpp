/**
 * @file  mission_script_engine_sensor.cpp
 * @brief AMS engine — alias resolution and peripheral sensor-field readers.
 *
 * Provides the runtime bridge between AMS script aliases (e.g. "GPS",
 * "BARO1") and the concrete HAL driver instances registered at engine
 * construction time.  All sensor reads are non-blocking; a driver failure
 * causes the operation to return @c false without modifying output state.
 *
 * Thread safety: all *Locked functions must be called while the engine
 * mutex is held by the calling task (Locked suffix convention).
 */

#include "ams/mission_script_engine.h"

#include "ares_assert.h"
#include "debug/ares_log.h"

#include <Arduino.h>
#include <cmath>
#include <cstdio>
#include <cstring>

namespace ares
{
namespace ams
{

static constexpr const char* TAG = "AMS";

// ── splitAliasDotField ───────────────────────────────────────────────────────

/**
 * @brief Split an @c ALIAS.field expression into its two components.
 *
 * Finds the first @c '.' in @p expr and copies the substring before it into
 * @p aliasOut and the substring after it into @p fieldOut.
 *
 * @param[in]  expr       Source expression (NUL-terminated, e.g. "GPS.alt").
 * @param[out] aliasOut   Buffer for the alias part.
 * @param[in]  aliasSize  Capacity of @p aliasOut in bytes.
 * @param[out] fieldOut   Buffer for the field name part.
 * @param[in]  fieldSize  Capacity of @p fieldOut in bytes.
 * @return @c true if a non-empty alias and non-empty field were extracted.
 */
bool MissionScriptEngine::splitAliasDotField(const char* expr,
                                             char*       aliasOut,
                                             uint8_t     aliasSize,
                                             char*       fieldOut,
                                             uint8_t     fieldSize)
{
    if (expr == nullptr || aliasOut == nullptr || fieldOut == nullptr)
    {
        return false;
    }

    const char* dot = strchr(expr, '.');
    if (dot == nullptr) { return false; }

    const uint32_t aliasLen = static_cast<uint32_t>(dot - expr);
    if (aliasLen == 0U || aliasLen >= static_cast<uint32_t>(aliasSize))
    {
        return false;
    }

    strncpy(aliasOut, expr, aliasLen);
    aliasOut[aliasLen] = '\0';

    strncpy(fieldOut, dot + 1, static_cast<size_t>(fieldSize) - 1U);
    fieldOut[fieldSize - 1U] = '\0';

    return fieldOut[0] != '\0';
}

// ── parseSensorField ─────────────────────────────────────────────────────────

/**
 * @brief Map a field name string to its @c SensorField enumerator.
 *
 * Valid field names depend on @p kind:
 *   - GPS:  @c lat, @c lon, @c alt, @c speed, @c sats, @c hdop
 *   - BARO: @c alt, @c temp, @c pressure
 *   - IMU:  @c accel_x/y/z, @c accel_mag, @c gyro_x/y/z, @c temp
 *   - COM:  (no readable float fields — always returns @c false)
 *
 * @param[in]  kind      Peripheral kind of the owning alias.
 * @param[in]  fieldStr  Field name string (NUL-terminated).
 * @param[out] out       Populated with the matching @c SensorField on success.
 * @return @c true if @p fieldStr is a valid field for @p kind.
 */
bool MissionScriptEngine::parseSensorField(PeripheralKind kind,
                                           const char*    fieldStr,
                                           SensorField&   out)
{
    if (fieldStr == nullptr) { return false; }
    switch (kind)
    {
    case PeripheralKind::GPS:  return parseGpsSensorField(fieldStr, out);
    case PeripheralKind::BARO: return parseBaroSensorField(fieldStr, out);
    case PeripheralKind::IMU:  return parseImuSensorField(fieldStr, out);
    case PeripheralKind::COM:
    default:
        return false;
    }
}

bool MissionScriptEngine::parseGpsSensorField(const char* fieldStr, SensorField& out)
{
    if (strcmp(fieldStr, "lat") == 0)   { out = SensorField::LAT;   return true; }
    if (strcmp(fieldStr, "lon") == 0)   { out = SensorField::LON;   return true; }
    if (strcmp(fieldStr, "alt") == 0)   { out = SensorField::ALT;   return true; }
    if (strcmp(fieldStr, "speed") == 0) { out = SensorField::SPEED; return true; }
    if (strcmp(fieldStr, "sats") == 0)  { out = SensorField::SATS;  return true; }
    if (strcmp(fieldStr, "hdop") == 0)  { out = SensorField::HDOP;  return true; }
    return false;
}

bool MissionScriptEngine::parseBaroSensorField(const char* fieldStr, SensorField& out)
{
    if (strcmp(fieldStr, "alt") == 0)      { out = SensorField::ALT;      return true; }
    if (strcmp(fieldStr, "temp") == 0)     { out = SensorField::TEMP;     return true; }
    if (strcmp(fieldStr, "pressure") == 0) { out = SensorField::PRESSURE; return true; }
    return false;
}

bool MissionScriptEngine::parseImuSensorField(const char* fieldStr, SensorField& out)
{
    if (strcmp(fieldStr, "accel_x") == 0)   { out = SensorField::ACCEL_X;   return true; }
    if (strcmp(fieldStr, "accel_y") == 0)   { out = SensorField::ACCEL_Y;   return true; }
    if (strcmp(fieldStr, "accel_z") == 0)   { out = SensorField::ACCEL_Z;   return true; }
    if (strcmp(fieldStr, "accel_mag") == 0) { out = SensorField::ACCEL_MAG; return true; }
    if (strcmp(fieldStr, "gyro_x") == 0)    { out = SensorField::GYRO_X;    return true; }
    if (strcmp(fieldStr, "gyro_y") == 0)    { out = SensorField::GYRO_Y;    return true; }
    if (strcmp(fieldStr, "gyro_z") == 0)    { out = SensorField::GYRO_Z;    return true; }
    if (strcmp(fieldStr, "temp") == 0)      { out = SensorField::IMU_TEMP;  return true; }
    return false;
}

// ── findAliasLocked ──────────────────────────────────────────────────────────

/**
 * @brief Look up a peripheral alias by name in the active script's alias table.
 *
 * @param[in] alias  Alias name string (NUL-terminated, e.g. "GPS", "GPS1").
 * @return Pointer to the matching @c AliasEntry, or @c nullptr if not found.
 * @note Caller must hold the engine mutex (Locked suffix convention).
 */
const MissionScriptEngine::AliasEntry*
MissionScriptEngine::findAliasLocked(const char* alias) const
{
    if (alias == nullptr || alias[0] == '\0') { return nullptr; }

    for (uint8_t i = 0; i < program_.aliasCount; i++)
    {
        if (strcmp(program_.aliases[i].alias, alias) == 0)
        {
            return &program_.aliases[i];
        }
    }

    return nullptr;
}

// ── readSensorFloatLocked ────────────────────────────────────────────────────

/**
 * @brief Read a single float value from a named sensor alias and field.
 *
 * Resolves the alias to a concrete HAL driver via the driver registry,
 * performs a blocking HAL read, and returns the requested field.
 * Returns @c false without modifying @p outVal on any failure (unknown
 * alias, out-of-range driver index, null interface, or driver error).
 *
 * @param[in]  alias   Peripheral alias name (e.g. "GPS", "BARO").
 * @param[in]  field   Sensor field to retrieve.
 * @param[out] outVal  Set to the read value on success.
 * @return @c true if the value was successfully read from the driver.
 * @note Caller must hold the engine mutex (Locked suffix convention).
 */
bool MissionScriptEngine::readSensorFloatLocked(const char*  alias,
                                                SensorField  field,
                                                float&       outVal) const
{
    const AliasEntry* ae = findAliasLocked(alias);
    if (ae == nullptr || ae->driverIdx == 0xFFU) { return false; }

    switch (ae->kind)
    {
    case PeripheralKind::GPS:  return readGpsFieldLocked(*ae, field, outVal);
    case PeripheralKind::BARO: return readBaroFieldLocked(*ae, field, outVal);
    case PeripheralKind::IMU:  return readImuFieldLocked(*ae, field, outVal);
    case PeripheralKind::COM:
    default:
        return false;
    }
}

bool MissionScriptEngine::readGpsFieldLocked(const AliasEntry& ae,
                                             SensorField       field,
                                             float&            outVal) const
{
    if (ae.driverIdx >= gpsCount_) { return false; }
    GpsInterface* gps = gpsDrivers_[ae.driverIdx].iface;
    if (gps == nullptr) { return false; }

    const uint8_t  aliasIdx = static_cast<uint8_t>(&ae - &program_.aliases[0]);
    const uint32_t nowMs    = static_cast<uint32_t>(millis());

    if (!gpsCacheValid_[aliasIdx] || (nowMs - gpsCacheTsMs_[aliasIdx]) >= GPS_CACHE_MAX_AGE_MS)
    {
        const uint8_t maxAttempts = static_cast<uint8_t>(ae.retryCount + 1U);
        GpsReading r = {};
        bool ok = false;
        for (uint8_t attempt = 0U; attempt < maxAttempts && !ok; attempt++)
        {
            ok = (gps->read(r) == GpsStatus::OK);
        }
        gpsCacheTsMs_[aliasIdx]  = static_cast<uint32_t>(millis());
        gpsCacheValid_[aliasIdx] = ok;
        if (ok) { gpsCachedReadings_[aliasIdx] = r; }
        else    { return false; }
    }

    if (!gpsCacheValid_[aliasIdx]) { return false; }
    const GpsReading& r = gpsCachedReadings_[aliasIdx];
    switch (field)
    {
    case SensorField::LAT:   outVal = r.latitude;                        return true;
    case SensorField::LON:   outVal = r.longitude;                       return true;
    case SensorField::ALT:   outVal = r.altitudeM;                       return true;
    case SensorField::SPEED: outVal = r.speedKmh;                        return true;
    case SensorField::SATS:  outVal = static_cast<float>(r.satellites);  return true;
    case SensorField::HDOP:  outVal = r.hdop;                            return true;
    default:                                                              return false;
    }
}

bool MissionScriptEngine::readBaroFieldLocked(const AliasEntry& ae,
                                              SensorField       field,
                                              float&            outVal) const
{
    if (ae.driverIdx >= baroCount_) { return false; }
    BarometerInterface* baro = baroDrivers_[ae.driverIdx].iface;
    if (baro == nullptr) { return false; }

    const uint8_t  aliasIdx = static_cast<uint8_t>(&ae - &program_.aliases[0]);
    const uint32_t nowMs    = static_cast<uint32_t>(millis());

    if (!baroCacheValid_[aliasIdx] || (nowMs - baroCacheTsMs_[aliasIdx]) >= BARO_CACHE_MAX_AGE_MS)
    {
        const uint8_t maxAttempts = static_cast<uint8_t>(ae.retryCount + 1U);
        BaroReading r = {};
        bool ok = false;
        for (uint8_t attempt = 0U; attempt < maxAttempts && !ok; attempt++)
        {
            ok = (baro->read(r) == BaroStatus::OK);
        }
        baroCacheTsMs_[aliasIdx]  = static_cast<uint32_t>(millis());
        baroCacheValid_[aliasIdx] = ok;
        if (ok) { baroCachedReadings_[aliasIdx] = r; }
        else    { return false; }
    }

    if (!baroCacheValid_[aliasIdx]) { return false; }
    const BaroReading& r = baroCachedReadings_[aliasIdx];
    switch (field)
    {
    case SensorField::ALT:      outVal = r.altitudeM;    return true;
    case SensorField::TEMP:     outVal = r.temperatureC; return true;
    case SensorField::PRESSURE: outVal = r.pressurePa;   return true;
    default:                                              return false;
    }
}

bool MissionScriptEngine::refreshImuCacheLocked(ImuInterface* imu,
                                                uint8_t       maxAttempts) const
{
    const uint32_t nowMs = static_cast<uint32_t>(millis());
    if ((nowMs - imuCacheTsMs_) < IMU_CACHE_MAX_AGE_MS) { return true; }

    bool readOk = false;
    ImuReading r = {};
    for (uint8_t attempt = 0U; attempt < maxAttempts; attempt++)
    {
        if (imu->read(r) == ImuStatus::OK)
        {
            readOk = true;
            break;
        }
    }
    imuCacheTsMs_ = static_cast<uint32_t>(millis());
    if (!readOk)
    {
        imuCacheValid_ = false;
        return false;
    }
    imuCachedReading_ = r;
    imuCacheValid_    = true;
    return true;
}

bool MissionScriptEngine::readImuFieldLocked(const AliasEntry& ae,
                                             SensorField       field,
                                             float&            outVal) const
{
    if (ae.driverIdx >= imuCount_) { return false; }
    ImuInterface* imu = imuDrivers_[ae.driverIdx].iface;
    if (imu == nullptr) { return false; }

    const uint8_t maxAttempts = static_cast<uint8_t>(ae.retryCount + 1U);
    if (!refreshImuCacheLocked(imu, maxAttempts)) { return false; }

    if (!imuCacheValid_) { return false; }
    const ImuReading& r = imuCachedReading_;
    switch (field)
    {
    case SensorField::ACCEL_X:   outVal = r.accelX; return true;
    case SensorField::ACCEL_Y:   outVal = r.accelY; return true;
    case SensorField::ACCEL_Z:   outVal = r.accelZ; return true;
    case SensorField::ACCEL_MAG:
        outVal = sqrtf(r.accelX * r.accelX
                     + r.accelY * r.accelY
                     + r.accelZ * r.accelZ);
        return true;
    case SensorField::GYRO_X:    outVal = r.gyroX;  return true;
    case SensorField::GYRO_Y:    outVal = r.gyroY;  return true;
    case SensorField::GYRO_Z:    outVal = r.gyroZ;  return true;
    case SensorField::IMU_TEMP:  outVal = r.tempC;  return true;
    default:
        return false;
    }
}

// ── parseCondExprLocked ──────────────────────────────────────────────────────

/**
 * @brief Parse a condition expression token triple (lhs op rhs) into a
 *        @c CondExpr value.
 *
 * Recognised LHS forms:
 *   - @c TC.command  → @c CondKind::TC_EQ  (only if @p allowTc is @c true)
 *   - @c TIME.elapsed → @c CondKind::TIME_GT (operator must be @c >)
 *   - @c ALIAS.field  → @c CondKind::SENSOR_LT or @c SENSOR_GT
 *
 * On failure the engine error is set via setErrorLocked().
 *
 * @param[in]  lhs      Left-hand side token (e.g. "GPS.alt", "TC.command").
 * @param[in]  op       Operator token (@c ==, @c <, or @c >).
 * @param[in]  rhs      Right-hand side token (value or TC keyword).
 * @param[in]  allowTc  Whether @c TC.command is accepted as an LHS.
 * @param[out] out      Populated @c CondExpr on success.
 * @return @c true if the expression was parsed without error.
 * @note Caller must hold the engine mutex (Locked suffix convention).
 */
bool MissionScriptEngine::parseCondExprLocked(const char* lhs,
                                              const char* op,
                                              const char* rhs,
                                              bool        allowTc,
                                              CondExpr&   out)
{
    if (lhs == nullptr || op == nullptr || rhs == nullptr)
    {
        setErrorLocked("condition expression has null token");
        return false;
    }

    if (strcmp(lhs, "TIME.elapsed") == 0)
    {
        return parseTimeCondExprLocked(op, rhs, out);
    }
    {
        char tcCmdToken[24] = {};
        snprintf(tcCmdToken, sizeof(tcCmdToken), "%s.command", program_.tcAlias);
        if (strcmp(lhs, tcCmdToken) == 0)
        {
            return parseTcCondExprLocked(op, rhs, allowTc, out);
        }
    }
    return parseSensorCondExprLocked(lhs, op, rhs, out);
}

bool MissionScriptEngine::parseTcCondExprLocked(const char* op,
                                                const char* rhs,
                                                bool        allowTc,
                                                CondExpr&   out)
{
    if (!allowTc)
    {
        setErrorLocked("TC.command not valid in conditions block");
        return false;
    }
    if (strcmp(op, "==") != 0)
    {
        setErrorLocked("TC.command only supports '==' operator");
        return false;
    }
    TcCommand cmd = TcCommand::NONE;
    if (!parseTcCommand(rhs, cmd) || cmd == TcCommand::NONE)
    {
        setErrorLocked("invalid TC.command value");
        return false;
    }
    out.kind = CondKind::TC_EQ;
    out.tcValue = cmd;
    return true;
}

bool MissionScriptEngine::parseTimeCondExprLocked(const char* op,
                                                  const char* rhs,
                                                  CondExpr&   out)
{
    if (strcmp(op, ">") != 0)
    {
        setErrorLocked("TIME.elapsed only supports '>' operator");
        return false;
    }
    float thr = 0.0f;
    if (!parseFloatValue(rhs, thr))
    {
        setErrorLocked("invalid TIME.elapsed threshold");
        return false;
    }
    out.kind = CondKind::TIME_GT;
    out.threshold = thr;
    return true;
}

bool MissionScriptEngine::parseSensorCondExprLocked(const char* lhs,
                                                    const char* op,
                                                    const char* rhs,
                                                    CondExpr&   out)
{
    char aliasStr[16] = {};
    char fieldStr[20] = {};
    if (!splitAliasDotField(lhs,
                            aliasStr, sizeof(aliasStr),
                            fieldStr, sizeof(fieldStr)))
    {
        setErrorLocked("invalid condition LHS (expected ALIAS.field)");
        return false;
    }

    const AliasEntry* ae = findAliasLocked(aliasStr);
    if (ae == nullptr)
    {
        char msg[56] = {};
        snprintf(msg, sizeof(msg), "unknown alias '%s' in condition", aliasStr);
        setErrorLocked(msg);
        return false;
    }

    SensorField sf = SensorField::ALT;
    if (!parseSensorField(ae->kind, fieldStr, sf))
    {
        static char msg[80] = {};
        snprintf(msg, sizeof(msg),
                 "field '%s' not valid for alias '%s'", fieldStr, aliasStr);
        setErrorLocked(msg);
        return false;
    }

    if (strcmp(op, "<") == 0)
    {
        out.kind = CondKind::SENSOR_LT;
    }
    else if (strcmp(op, ">") == 0)
    {
        out.kind = CondKind::SENSOR_GT;
    }
    else
    {
        setErrorLocked("sensor condition only supports '<' or '>' operators");
        return false;
    }

    strncpy(out.alias, aliasStr, sizeof(out.alias) - 1U);
    out.alias[sizeof(out.alias) - 1U] = '\0';
    out.field = sf;

    return parseConditionRhsThresholdLocked(rhs, out);
}

bool MissionScriptEngine::parseRhsVarNameOffsetLocked(
    const char* varExpr, char* varName, uint8_t varNameSz,
    float& offset, bool& hasOffset)
{
    const char* plus  = strstr(varExpr, " + ");
    const char* minus = strstr(varExpr, " - ");

    if (plus != nullptr || minus != nullptr)
    {
        const char* sep   = (minus != nullptr && (plus == nullptr || minus < plus)) ? minus : plus;
        const bool  isNeg = (sep == minus);
        const ptrdiff_t nameLen = sep - varExpr;
        if (nameLen <= 0 || nameLen >= static_cast<ptrdiff_t>(varNameSz))
        {
            setErrorLocked("condition RHS: variable name too long");
            return false;
        }
        memcpy(varName, varExpr, static_cast<size_t>(nameLen));
        varName[static_cast<size_t>(nameLen)] = '\0';
        trimInPlace(varName);

        const char* offStr = sep + 3;  // skip " + " or " - "
        if (!parseFloatValue(offStr, offset))
        {
            setErrorLocked("condition RHS: invalid offset value");
            return false;
        }
        if (isNeg) { offset = -offset; }
        hasOffset = true;
    }
    else
    {
        strncpy(varName, varExpr, static_cast<size_t>(varNameSz) - 1U);
        varName[static_cast<size_t>(varNameSz) - 1U] = '\0';
        trimInPlace(varName);
    }

    if (varName[0] == '\0')
    {
        setErrorLocked("condition RHS: empty variable name");
        return false;
    }
    return true;
}

bool MissionScriptEngine::parseConditionRhsThresholdLocked(const char* rhs, // NOLINT(readability-function-size)
                                                           CondExpr&   out)
{
    float thr = 0.0f;
    if (parseFloatValue(rhs, thr))
    {
        out.threshold = thr;
        return true;
    }

    const char* varExpr = rhs;
    char stripped[ares::AMS_MAX_LINE_LEN] = {};
    if (rhs[0] == '(')
    {
        const char* close = strchr(rhs, ')');
        if (close == nullptr)
        {
            setErrorLocked("condition RHS: unclosed '(' in variable expression");
            return false;
        }
        const ptrdiff_t innerLen = close - (rhs + 1);
        if (innerLen <= 0 || innerLen >= static_cast<ptrdiff_t>(sizeof(stripped)))
        {
            setErrorLocked("condition RHS: variable expression too long");
            return false;
        }
        memcpy(stripped, rhs + 1, static_cast<size_t>(innerLen));
        stripped[static_cast<size_t>(innerLen)] = '\0';
        trimInPlace(stripped);
        varExpr = stripped;
    }

    char varName[ares::AMS_VAR_NAME_LEN] = {};
    float offset = 0.0f;
    bool hasOffset = false;
    if (!parseRhsVarNameOffsetLocked(varExpr, varName, static_cast<uint8_t>(sizeof(varName)),
                                     offset, hasOffset))
    {
        return false;
    }

    bool found = false;
    for (uint8_t i = 0; i < program_.varCount; i++)
    {
        if (strcmp(program_.vars[i].name, varName) == 0) { found = true; break; }
    }
    if (!found)
    {
        for (uint8_t ci = 0; ci < program_.constCount; ci++)
        {
            if (strcmp(program_.consts[ci].name, varName) == 0)
            {
                out.threshold = program_.consts[ci].value + (hasOffset ? offset : 0.0f);
                return true;
            }
        }
        char msg[64] = {};
        snprintf(msg, sizeof(msg),
                 "condition RHS: undefined variable or constant '%s'", varName);
        setErrorLocked(msg);
        return false;
    }

    out.useVar = true;
    out.varOffset = hasOffset ? offset : 0.0f;
    strncpy(out.varName, varName, sizeof(out.varName) - 1U);
    out.varName[sizeof(out.varName) - 1U] = '\0';
    return true;
}

} // namespace ams
} // namespace ares

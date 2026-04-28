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
 *   - GPS:  @c lat, @c lon, @c alt, @c speed
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

    if (kind == PeripheralKind::GPS)
    {
        if (strcmp(fieldStr, "lat")   == 0) { out = SensorField::LAT;   return true; }
        if (strcmp(fieldStr, "lon")   == 0) { out = SensorField::LON;   return true; }
        if (strcmp(fieldStr, "alt")   == 0) { out = SensorField::ALT;   return true; }
        if (strcmp(fieldStr, "speed") == 0) { out = SensorField::SPEED; return true; }
        return false;
    }

    if (kind == PeripheralKind::BARO)
    {
        if (strcmp(fieldStr, "alt")      == 0) { out = SensorField::ALT;      return true; }
        if (strcmp(fieldStr, "temp")     == 0) { out = SensorField::TEMP;     return true; }
        if (strcmp(fieldStr, "pressure") == 0) { out = SensorField::PRESSURE; return true; }
        return false;
    }

    if (kind == PeripheralKind::IMU)
    {
        if (strcmp(fieldStr, "accel_x")   == 0) { out = SensorField::ACCEL_X;   return true; }
        if (strcmp(fieldStr, "accel_y")   == 0) { out = SensorField::ACCEL_Y;   return true; }
        if (strcmp(fieldStr, "accel_z")   == 0) { out = SensorField::ACCEL_Z;   return true; }
        if (strcmp(fieldStr, "accel_mag") == 0) { out = SensorField::ACCEL_MAG; return true; }
        if (strcmp(fieldStr, "gyro_x")    == 0) { out = SensorField::GYRO_X;    return true; }
        if (strcmp(fieldStr, "gyro_y")    == 0) { out = SensorField::GYRO_Y;    return true; }
        if (strcmp(fieldStr, "gyro_z")    == 0) { out = SensorField::GYRO_Z;    return true; }
        if (strcmp(fieldStr, "temp")      == 0) { out = SensorField::IMU_TEMP;  return true; }
        return false;
    }

    return false;  // COM has no readable float fields
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
    case PeripheralKind::GPS:
    {
        if (ae->driverIdx >= gpsCount_)    { return false; }
        GpsInterface* gps = gpsDrivers_[ae->driverIdx].iface;
        if (gps == nullptr)                { return false; }
        GpsReading r = {};
        if (gps->read(r) != GpsStatus::OK) { return false; }
        switch (field)
        {
        case SensorField::LAT:   outVal = r.latitude;  return true;
        case SensorField::LON:   outVal = r.longitude; return true;
        case SensorField::ALT:   outVal = r.altitudeM; return true;
        case SensorField::SPEED: outVal = r.speedKmh;  return true;
        default:                                        return false;
        }
    }

    case PeripheralKind::BARO:
    {
        if (ae->driverIdx >= baroCount_)          { return false; }
        BarometerInterface* baro = baroDrivers_[ae->driverIdx].iface;
        if (baro == nullptr)                      { return false; }
        BaroReading r = {};
        if (baro->read(r) != BaroStatus::OK)      { return false; }
        switch (field)
        {
        case SensorField::ALT:      outVal = r.altitudeM;    return true;
        case SensorField::TEMP:     outVal = r.temperatureC; return true;
        case SensorField::PRESSURE: outVal = r.pressurePa;   return true;
        default:                                              return false;
        }
    }

    case PeripheralKind::IMU:
    {
        if (ae->driverIdx >= imuCount_)      { return false; }
        ImuInterface* imu = imuDrivers_[ae->driverIdx].iface;
        if (imu == nullptr)                  { return false; }

        // Re-use the cached burst if it was filled within IMU_CACHE_MAX_AGE_MS.
        // This ensures all fields in a single LOG/HK report row share one I2C
        // read, eliminating the partial-nan scatter caused by each field failing
        // independently on a flaky connection.
        const uint32_t nowMs = static_cast<uint32_t>(millis());
        if ((nowMs - imuCacheTsMs_) >= IMU_CACHE_MAX_AGE_MS)
        {
            ImuReading r  = {};
            const bool readOk = (imu->read(r) == ImuStatus::OK);
            // Timestamp set AFTER the read so the 5 ms window starts from
            // completion, not before a potentially-long I2C timeout.
            // All subsequent fields in the same LOG row then see a fresh
            // cache and skip the re-read entirely.
            imuCacheTsMs_ = static_cast<uint32_t>(millis());
            if (!readOk)
            {
                imuCacheValid_ = false;
                return false;
            }
            imuCachedReading_ = r;
            imuCacheValid_    = true;
        }

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
        default:                                         return false;
        }
    }

    case PeripheralKind::COM:
    default:
        return false;  // COM has no readable float fields
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

    // TC.command == LAUNCH | ABORT | RESET
    if (strcmp(lhs, "TC.command") == 0)
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
        out.kind    = CondKind::TC_EQ;
        out.tcValue = cmd;
        return true;
    }

    // TIME.elapsed > VALUE  (built-in pseudo-alias; no include needed)
    if (strcmp(lhs, "TIME.elapsed") == 0)
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
        out.kind      = CondKind::TIME_GT;
        out.threshold = thr;
        return true;
    }

    // ALIAS.field < VALUE  or  ALIAS.field > VALUE
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
        char msg[80] = {};
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

    // AMS-4.8: RHS can be a literal float OR a variable reference:
    //   ALIAS.field OP varname
    //   ALIAS.field OP (varname + offset)
    //   ALIAS.field OP (varname - offset)
    float thr = 0.0f;
    if (parseFloatValue(rhs, thr))
    {
        // Plain numeric literal — existing behaviour.
        out.threshold = thr;
        return true;
    }

    // Try variable reference forms.
    // Strip optional parentheses.
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

    // Attempt "varname +/- offset" or bare "varname".
    char varName[ares::AMS_VAR_NAME_LEN] = {};
    float offset = 0.0f;
    bool hasOffset = false;

    const char* plus  = strstr(varExpr, " + ");
    const char* minus = strstr(varExpr, " - ");

    if (plus != nullptr || minus != nullptr)
    {
        // Pick whichever separator appears first.
        const char* sep   = (minus != nullptr && (plus == nullptr || minus < plus)) ? minus : plus;
        const bool  isNeg = (sep == minus);
        const ptrdiff_t nameLen = sep - varExpr;
        if (nameLen <= 0 || nameLen >= static_cast<ptrdiff_t>(ares::AMS_VAR_NAME_LEN))
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
        // Bare variable name.
        strncpy(varName, varExpr, sizeof(varName) - 1U);
        varName[sizeof(varName) - 1U] = '\0';
        trimInPlace(varName);
    }

    if (varName[0] == '\0')
    {
        setErrorLocked("condition RHS: empty variable name");
        return false;
    }

    // Validate that the variable exists in the program at parse time.
    bool found = false;
    for (uint8_t i = 0; i < program_.varCount; i++)
    {
        if (strcmp(program_.vars[i].name, varName) == 0) { found = true; break; }
    }
    if (!found)
    {
        char msg[64] = {};
        snprintf(msg, sizeof(msg), "condition RHS: undefined variable '%s'", varName);
        setErrorLocked(msg);
        return false;
    }

    out.useVar    = true;
    out.varOffset = hasOffset ? offset : 0.0f;
    strncpy(out.varName, varName, sizeof(out.varName) - 1U);
    out.varName[sizeof(out.varName) - 1U] = '\0';
    return true;
}

} // namespace ams
} // namespace ares

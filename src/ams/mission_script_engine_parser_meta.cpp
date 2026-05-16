/**
 * @file  mission_script_engine_parser_meta.cpp
 * @brief AMS engine â€” pre-state metadata parsing (pus.*, radio.config, include, var, const).
 *
 * Thread safety: all *Locked functions must be called while the engine
 * mutex is held by the calling task (Locked suffix convention).
 */

#include "ams/mission_script_engine.h"
#include "ams/mission_script_engine_helpers.h"

#include "ares_assert.h"
#include "ares_util.h"
#include "debug/ares_log.h"

#include <cstdio>
#include <cstring>

namespace ares
{
namespace ams
{
using ares::proto::NODE_BROADCAST;
using ares::proto::NODE_ROCKET;
using ares::proto::NODE_GROUND;
using ares::proto::NODE_PAYLOAD;
static constexpr const char* TAG = "AMS";
bool MissionScriptEngine::parsePusApidDirectiveLocked(const char* line)
{
    const char* eq = strchr(line, '=');
    if (eq == nullptr)
    {
        setErrorLocked("invalid pus.apid syntax");
        return false;
    }

    uint32_t apid = 0;
    if (!parseUint(eq + 1, apid) || apid > 3U)
    {
        setErrorLocked("pus.apid out of range: valid values are 0..3");
        return false;
    }

    uint8_t nodeId = 0;
    if (!mapApidToNode(static_cast<uint16_t>(apid), nodeId))
    {
        ARES_ASSERT(false && "unreachable: apid validated to 0..3 above");
        return false;
    }

    program_.apid   = static_cast<uint16_t>(apid);
    program_.nodeId = nodeId;
    return true;
}

// ── parsePusServiceDirectiveLocked ───────────────────────────────────────────

/**
 * @brief Parse a @c pus.service directive and assign a custom alias.
 *
 * Syntax: @c "pus.service N as ALIAS"
 *
 * Supported service numbers: 1 (TC verification), 3 (HK), 5 (event).
 * The reserved words TIME and LOG may not be used as aliases.
 *
 * @param[in] line  Script line starting with @c "pus.service ".
 * @return @c true if the directive was parsed and the alias stored.
 * @pre  Caller holds the engine mutex.
 */
bool MissionScriptEngine::parsePusServiceDirectiveLocked(const char* line)
{
    uint32_t svcNum = 0U;
    char     alias[16] = {};
    // cppcheck-suppress [cert-err34-c]
    // Safe: alias[16] is a local buffer; format %15s limits the scan to 15 chars (sizeof-1);
    // return value checked == 2 before any field is used.
    const int32_t n = static_cast<int32_t>(sscanf(line, "pus.service %" SCNu32 " as %15s", &svcNum, alias)); // NOLINT(bugprone-unchecked-string-to-number-conversion)
    if (n != 2 || alias[0] == '\0')
    {
        setErrorLocked("invalid pus.service syntax (expected: pus.service N as ALIAS)");
        return false;
    }
    if (svcNum != 1U && svcNum != 3U && svcNum != 5U)
    {
        setErrorLocked("pus.service: unsupported service number (valid: 1, 3, 5)");
        return false;
    }
    if (strcmp(alias, "TIME") == 0 || strcmp(alias, "LOG") == 0)
    {
        setErrorLocked("pus.service: alias conflicts with reserved keyword");
        return false;
    }
    switch (svcNum)
    {
    case 1U:
        ares::util::copyZ(program_.tcAlias, alias, sizeof(program_.tcAlias));
        program_.tcAlias[sizeof(program_.tcAlias) - 1U] = '\0';
        break;
    case 3U:
        ares::util::copyZ(program_.hkAlias, alias, sizeof(program_.hkAlias));
        program_.hkAlias[sizeof(program_.hkAlias) - 1U] = '\0';
        break;
    case 5U:
        ares::util::copyZ(program_.eventAlias, alias, sizeof(program_.eventAlias));
        program_.eventAlias[sizeof(program_.eventAlias) - 1U] = '\0';
        break;
    default: break;
    }
    return true;
}

// ── parseRadioConfigLineLocked ───────────────────────────────────────────────

/**
 * @brief Parse a @c radio.config directive and store the override into
 *        @c program_.radioConfig (AMS-4.15).
 *
 * Syntax: @c "radio.config PARAM_NAME = VALUE"
 *
 * Supported @c PARAM_NAME tokens and their valid ranges:
 * | Token              | ConfigParamId         | Range (inclusive)          |
 * |--------------------|----------------------|----------------------------|
 * | telem_interval     | TELEM_INTERVAL_MS    | 100 – 60000 (ms)           |
 * | monitor.alt.high   | MONITOR_ALT_HIGH_M   |    0 – 15000 (m)           |
 * | monitor.alt.low    | MONITOR_ALT_LOW_M    | –500 – 1000  (m)           |
 * | monitor.accel.max  | MONITOR_ACCEL_HIGH   |    0 – 1000  (m/s²)        |
 * | monitor.temp.high  | MONITOR_TEMP_HIGH_C  |  –40 – 150   (°C)          |
 * | monitor.temp.low   | MONITOR_TEMP_LOW_C   | –100 – 50    (°C)          |
 *
 * @param[in] line  Script line starting with @c "radio.config ".
 * @return @c true if the directive was parsed and stored without error.
 * @pre  Caller holds the engine mutex.
 */
bool MissionScriptEngine::parseRadioConfigLineLocked(const char* line)
{
    // Parse:  radio.config PARAM_NAME = VALUE
    char  key[32] = {};
    float value   = 0.0F;
    // cppcheck-suppress [cert-err34-c]
    // MISRA-1: sscanf returns int (third-party API); cast to int32_t on capture.
    // Safe: key[32] is a local buffer; format %31s limits the scan to 31 chars (sizeof-1);
    // return value checked == 2 before any field is used.
    const int32_t n = static_cast<int32_t>(sscanf(line, "radio.config %31s = %f", key, &value)); // NOLINT(bugprone-unchecked-string-to-number-conversion)
    if (n != 2 || key[0] == '\0')
    {
        setErrorLocked("invalid radio.config syntax (expected: radio.config PARAM = VALUE)");
        return false;
    }

    // Resolve token → ConfigParamId + bounds.
    using Id = ares::proto::ConfigParamId;

    /** Maps an AMS keyword to its wire ConfigParamId and allowed value range. */
    struct ParamSpec
    {
        const char* token;   ///< AMS keyword.
        Id          id;      ///< Wire-protocol identifier.
        float       minVal;  ///< Inclusive lower bound.
        float       maxVal;  ///< Inclusive upper bound.
    };

    // MISRA-7: named constants from config.h — single authoritative source shared
    // with RadioDispatcher::configParams_[] (DRY, AMS-4.15).
    static const ParamSpec kSpecs[] = {
        { "telem_interval",    Id::TELEM_INTERVAL_MS,
          static_cast<double>(ares::TELEMETRY_INTERVAL_MIN),
          static_cast<double>(ares::TELEMETRY_INTERVAL_MAX) },
        { "monitor.alt.high",  Id::MONITOR_ALT_HIGH_M,  0.0f,  ares::MONITOR_ALT_HIGH_MAX_M  },
        { "monitor.alt.low",   Id::MONITOR_ALT_LOW_M,   ares::MONITOR_ALT_LOW_MIN_M,  ares::MONITOR_ALT_LOW_MAX_M  },
        { "monitor.accel.max", Id::MONITOR_ACCEL_HIGH,  0.0f,  ares::MONITOR_ACCEL_HIGH_MAX  },
        { "monitor.temp.high", Id::MONITOR_TEMP_HIGH_C, ares::MONITOR_TEMP_HIGH_MIN_C, ares::MONITOR_TEMP_HIGH_MAX_C },
        { "monitor.temp.low",  Id::MONITOR_TEMP_LOW_C,  ares::MONITOR_TEMP_LOW_MIN_C,  ares::MONITOR_TEMP_LOW_MAX_C  },
    };

    const ParamSpec* found = nullptr;
    for (uint8_t i = 0U; i < static_cast<uint8_t>(sizeof(kSpecs) / sizeof(kSpecs[0])); ++i)
    {
        if (strcmp(key, kSpecs[i].token) == 0)
        {
            found = &kSpecs[i];
            break;
        }
    }
    if (found == nullptr)
    {
        setErrorLocked("radio.config: unknown parameter name");
        return false;
    }
    if (value < found->minVal || value > found->maxVal)
    {
        setErrorLocked("radio.config: value out of allowed range");
        return false;
    }

    // Store override — index = (id - FIRST).
    // ARES-MISRA-DEV-002: outer cast suppresses integral promotion from uint8_t subtraction.
    const uint8_t idx = static_cast<uint8_t>(
        static_cast<uint8_t>(found->id) - static_cast<uint8_t>(Id::FIRST));
    program_.radioConfig[idx].id    = found->id;
    program_.radioConfig[idx].value = value;
    program_.radioConfig[idx].set   = true;

    return true;
}

bool MissionScriptEngine::parsePreStateDirectiveLocked(const char* line, bool& handled)
{
    handled = true;
    if (startsWith(line, "include "))
    {
        if (parseSeenState_) { setErrorLocked("'include' must appear before any state block"); return false; }
        return parseIncludeLineLocked(line);
    }
    if (startsWith(line, "radio.config "))
    {
        if (parseSeenState_) { setErrorLocked("'radio.config' must appear before any state block"); return false; }
        return parseRadioConfigLineLocked(line);
    }
    if (startsWith(line, "pus.service "))
    {
        if (parseSeenState_) { setErrorLocked("'pus.service' must appear before any state block"); return false; }
        return parsePusServiceDirectiveLocked(line);
    }
    if (startsWith(line, "pus.apid"))
    {
        if (parseSeenState_) { setErrorLocked("'pus.apid' must appear before any state block"); return false; }
        return parsePusApidDirectiveLocked(line);
    }
    if (startsWith(line, "var "))
    {
        if (parseSeenState_) { setErrorLocked("'var' must appear before any state block"); return false; }
        return parseVarLineLocked(line);
    }
    if (startsWith(line, "const "))
    {
        if (parseSeenState_) { setErrorLocked("'const' must appear before any state block"); return false; }
        return parseConstLineLocked(line);
    }
    handled = false;
    return true;
}

// ── parseIncludeLineLocked ────────────────────────────────────────────────────

/**
 * @brief Parse an @c include directive and register the resulting alias.
 *
 * Syntax: @c "include \<MODEL\> as \<ALIAS\>"
 *
 * Looks up @p MODEL in each compiled-in driver registry (GPS, BARO, COM, IMU).
 * On success, adds an entry to @c program_.aliases[] so that subsequent
 * @c ALIAS.field expressions in the script are valid.
 *
 * @param[in] line  Script line starting with @c "include ".
 * @return @c true if the model was found and the alias registered.
 * @pre  Caller holds the engine mutex.
 * @post @c program_.aliasCount is incremented on success.
 */
bool MissionScriptEngine::lookupModelInDriversLocked(
    const char* model, PeripheralKind& kind, uint8_t& driverIdx) const
{
    driverIdx = 0xFFU;
    for (uint8_t i = 0; i < gpsCount_ && driverIdx == 0xFFU; i++)
    {
        if (gpsDrivers_[i].model != nullptr
            && strcmp(gpsDrivers_[i].model, model) == 0)
        { kind = PeripheralKind::GPS; driverIdx = i; }
    }
    for (uint8_t i = 0; i < baroCount_ && driverIdx == 0xFFU; i++)
    {
        if (baroDrivers_[i].model != nullptr
            && strcmp(baroDrivers_[i].model, model) == 0)
        { kind = PeripheralKind::BARO; driverIdx = i; }
    }
    for (uint8_t i = 0; i < comCount_ && driverIdx == 0xFFU; i++)
    {
        if (comDrivers_[i].model != nullptr
            && strcmp(comDrivers_[i].model, model) == 0)
        { kind = PeripheralKind::COM; driverIdx = i; }
    }
    for (uint8_t i = 0; i < imuCount_ && driverIdx == 0xFFU; i++)
    {
        if (imuDrivers_[i].model != nullptr
            && strcmp(imuDrivers_[i].model, model) == 0)
        { kind = PeripheralKind::IMU; driverIdx = i; }
    }
    return (driverIdx != 0xFFU);
}

bool MissionScriptEngine::parseIncludeOptionalsLocked(const char* line, AliasEntry& ae)
{
    const char* retryP = strstr(line, " retry=");
    if (retryP != nullptr)
    {
        uint32_t retries = 0U;
        // Safe: retryP+7 points into a NUL-terminated script line; reads a single SCNu32;
        // return checked != 1; value validated in range [1, AMS_MAX_SENSOR_RETRY] before use.
        if (sscanf(retryP + 7, "%" SCNu32, &retries) != 1  // NOLINT(bugprone-unchecked-string-to-number-conversion)
            || retries == 0U
            || retries > static_cast<uint32_t>(ares::AMS_MAX_SENSOR_RETRY))
        {
            setErrorLocked("include retry= value must be 1-AMS_MAX_SENSOR_RETRY");
            return false;
        }
        ae.retryCount = static_cast<uint8_t>(retries);
    }

    const char* toP = strstr(line, " timeout=");
    if (toP != nullptr)
    {
        const char* numStart = toP + 9;
        const char* msPtr    = strstr(numStart, "ms");
        if (msPtr == nullptr || msPtr == numStart)
        {
            setErrorLocked("include timeout= must be of the form Nms");
            return false;
        }
        const ptrdiff_t numLen = msPtr - numStart;
        if (numLen <= 0 || numLen >= 16)
        {
            setErrorLocked("include timeout= numeric value out of range");
            return false;
        }
    }
    return true;
}

bool MissionScriptEngine::parseIncludeLineLocked(const char* line)
{
    char model[16] = {};
    char alias[16] = {};
    // cppcheck-suppress [cert-err34-c]
    const int32_t parsed = static_cast<int32_t>(sscanf(line, "include %15s as %15s", model, alias));
    if (parsed != 2 || model[0] == '\0' || alias[0] == '\0')
    {
        setErrorLocked("invalid include syntax (expected: include <MODEL> as <ALIAS>)");
        return false;
    }

    // TIME is a built-in alias — it cannot be redefined.
    if (strcmp(alias, "TIME") == 0)
    {
        setErrorLocked("TIME is a reserved alias; do not use it with include");
        return false;
    }

    // Duplicate alias check.
    for (uint8_t i = 0; i < program_.aliasCount; i++)
    {
        if (strcmp(program_.aliases[i].alias, alias) == 0)
        {
            char msg[48] = {};
            snprintf(msg, sizeof(msg), "duplicate alias: %s", alias);
            setErrorLocked(msg);
            return false;
        }
    }

    if (program_.aliasCount >= ares::AMS_MAX_INCLUDES)
    {
        setErrorLocked("too many include directives (max AMS_MAX_INCLUDES)");
        return false;
    }

    // Look up MODEL in each driver kind registry.
    PeripheralKind kind      = PeripheralKind::GPS;
    uint8_t        driverIdx = 0xFFU;
    if (!lookupModelInDriversLocked(model, kind, driverIdx))
    {
        char msg[64] = {};
        snprintf(msg, sizeof(msg), "model '%s' not found in compiled-in drivers", model);
        setErrorLocked(msg);
        return false;
    }

    AliasEntry& ae = program_.aliases[program_.aliasCount++];
    ares::util::copyZ(ae.alias, alias, sizeof(ae.alias));
    ares::util::copyZ(ae.model, model, sizeof(ae.model));
    ae.kind      = kind;
    ae.driverIdx = driverIdx;
    ae.retryCount = 0U;

    if (!parseIncludeOptionalsLocked(line, ae)) { return false; }

    LOG_I(TAG, "include: alias=%s model=%s kind=%u idx=%u retry=%u",
          ae.alias, ae.model,
          static_cast<uint32_t>(ae.kind),
          static_cast<uint32_t>(ae.driverIdx),
          static_cast<uint32_t>(ae.retryCount));
    return true;
}

// ── parseStateLineLocked ─────────────────────────────────────────────────────

/**
 * @brief Parse a @c state declaration and open a new state block.
 *
 * Syntax: @c "state \<NAME\>:"
 *
 * @param[in]  line          Line starting with @c "state ".
 * @param[out] currentState  Set to the index of the newly opened state.
 * @return @c true if the state was created successfully.
 * @pre  Caller holds the engine mutex.
 * @post @c program_.stateCount is incremented on success.
 */
bool MissionScriptEngine::parseStateLineLocked(const char* line,
                                               uint8_t&    currentState)
{
    if (program_.stateCount >= ares::AMS_MAX_STATES)
    {
        setErrorLocked("too many states");
        return false;
    }

    char stateName[ares::AMS_MAX_STATE_NAME] = {};
    const int32_t n = static_cast<int32_t>(sscanf(line, "state %15[^:]:", stateName));
    if (n != 1)
    {
        setErrorLocked("invalid state syntax");
        return false;
    }

    // AMS-4.1: state names must be valid identifiers ([A-Za-z0-9_]+).
    for (uint8_t ci = 0U; stateName[ci] != '\0'; ci++)
    {
        const char c = stateName[ci];
        const bool ok = (c >= 'a' && c <= 'z')
                     || (c >= 'A' && c <= 'Z')
                     || (c >= '0' && c <= '9')
                     || c == '_';
        if (!ok)
        {
            setErrorLocked("state name contains invalid characters (A-Z a-z 0-9 _ only)");
            return false;
        }
    }

    // Duplicate state name check.
    for (uint8_t si = 0U; si < program_.stateCount; si++)
    {
        if (strncmp(program_.states[si].name, stateName, ares::AMS_MAX_STATE_NAME) == 0)
        {
            char msg[ares::AMS_MAX_ERROR_TEXT] = {};
            (void)snprintf(msg, sizeof(msg), "duplicate state name: %s", stateName);
            setErrorLocked(msg);
            return false;
        }
    }

    currentState = program_.stateCount;
    StateDef& st = program_.states[currentState];
    ares::util::copyZ(st.name, stateName, sizeof(st.name));
    program_.stateCount++;
    return true;
}

// ── mapApidToNode ─────────────────────────────────────────────────────────────

/**
 * @brief Map a PUS APID value to the corresponding ARES node identifier.
 *
 * Supported mappings: 0 → BROADCAST, 1 → ROCKET, 2 → GROUND, 3 → PAYLOAD.
 *
 * @param[in]  apid    PUS APID value (0–3).
 * @param[out] nodeId  ARES protocol node identifier.
 * @return @c true if @p apid has a defined mapping.
 */
bool MissionScriptEngine::mapApidToNode(uint16_t apid, uint8_t& nodeId)
{
    switch (apid)
    {
    case 0U: nodeId = NODE_BROADCAST; return true;
    case 1U: nodeId = NODE_ROCKET;    return true;
    case 2U: nodeId = NODE_GROUND;    return true;
    case 3U: nodeId = NODE_PAYLOAD;   return true;
    default:                          return false;
    }
}

// ── parseVarLineLocked ───────────────────────────────────────────────────────

/**
 * @brief Parse a global variable declaration from the script metadata section.
 *
 * Syntax: @c "var NAME = VALUE"
 *
 * Variables must be declared before any @c state block.  At most
 * @c AMS_MAX_VARS variables may be declared per script (AMS-4.8).
 * The initial @p VALUE is stored as a literal float and marks the
 * variable as @em invalid until a @c set action fires at runtime.
 *
 * @param[in] line  Script line starting with @c "var ".
 * @return @c true if the declaration was stored.
 * @pre  Caller holds the engine mutex.
 */
bool MissionScriptEngine::parseVarLineLocked(const char* line)
{
    ARES_ASSERT(line != nullptr);

    char name[ares::AMS_VAR_NAME_LEN] = {};
    char valBuf[24]                   = {};

    // cppcheck-suppress [cert-err34-c]
    const int32_t n = static_cast<int32_t>(sscanf(line, "var %15s = %23s", name, valBuf));
    if (n != 2 || name[0] == '\0')
    {
        setErrorLocked("invalid var syntax (expected: var NAME = VALUE)");
        return false;
    }

    // Validate name: must start with a letter/underscore; no dots.
    if (strchr(name, '.') != nullptr)
    {
        setErrorLocked("variable name must not contain '.'");
        return false;
    }

    // Reserved alias names must not shadow sensors.
    if (strcmp(name, "TIME") == 0 || strcmp(name, "TC") == 0)
    {
        setErrorLocked("variable name shadows reserved alias");
        return false;
    }

    // Duplicate check.
    for (uint8_t i = 0; i < program_.varCount; i++)
    {
        if (strcmp(program_.vars[i].name, name) == 0)
        {
            char msg[48] = {};
            snprintf(msg, sizeof(msg), "duplicate variable declaration: %s", name);
            setErrorLocked(msg);
            return false;
        }
    }

    if (program_.varCount >= ares::AMS_MAX_VARS)
    {
        setErrorLocked("too many variable declarations (AMS_MAX_VARS exceeded)");
        return false;
    }

    float initVal = 0.0f;
    if (!parseFloatValue(valBuf, initVal))
    {
        setErrorLocked("invalid var initial value");
        return false;
    }

    VarEntry& v = program_.vars[program_.varCount++];
    ares::util::copyZ(v.name, name, sizeof(v.name));
    v.value = initVal;
    v.valid = false;  // marked invalid until a set action fires successfully
    return true;
}

// ── parseConstLineLocked ─────────────────────────────────────────────────────

/**
 * @brief Parse a named constant declaration from the script metadata section.
 *
 * Syntax: @c "const NAME = VALUE"
 *
 * Constants are immutable float literals resolved at parse time.  When a
 * constant name appears as a condition RHS, its value is inlined directly
 * into @c CondExpr::threshold (no runtime table lookup required).
 *
 * @param[in] line  Script line starting with @c "const ".
 * @return @c true if the constant was stored.
 * @pre  Caller holds the engine mutex.
 */
bool MissionScriptEngine::parseConstLineLocked(const char* line)
{
    ARES_ASSERT(line != nullptr);

    char name[ares::AMS_VAR_NAME_LEN] = {};
    char valBuf[24]                   = {};

    // cppcheck-suppress [cert-err34-c]
    const int32_t n = static_cast<int32_t>(sscanf(line, "const %15s = %23s", name, valBuf));
    if (n != 2 || name[0] == '\0')
    {
        setErrorLocked("invalid const syntax (expected: const NAME = VALUE)");
        return false;
    }

    if (!validateConstIdentifierLocked(name))
    {
        return false;
    }
    if (!ensureConstDoesNotConflictLocked(name))
    {
        return false;
    }

    if (program_.constCount >= ares::AMS_MAX_CONSTS)
    {
        setErrorLocked("too many const declarations (AMS_MAX_CONSTS exceeded)");
        return false;
    }

    float val = 0.0f;
    if (!parseFloatValue(valBuf, val))
    {
        setErrorLocked("const value must be a numeric literal");
        return false;
    }

    ConstEntry& ce = program_.consts[program_.constCount++];
    ares::util::copyZ(ce.name, name, sizeof(ce.name));
    ce.value = val;

    LOG_I(TAG, "const '%s' = %.3f", name, static_cast<double>(val));
    return true;
}

bool MissionScriptEngine::validateConstIdentifierLocked(const char* name)
{
    ARES_ASSERT(name != nullptr);

    // Reject names with dots (reserved for ALIAS.field forms).
    if (strchr(name, '.') != nullptr)
    {
        setErrorLocked("const name must not contain '.'");
        return false;
    }

    // Reject reserved pseudo-aliases.
    if (strcmp(name, "TIME") == 0 || strcmp(name, "TC") == 0)
    {
        setErrorLocked("const name shadows reserved alias");
        return false;
    }

    return true;
}

bool MissionScriptEngine::ensureConstDoesNotConflictLocked(const char* name)
{
    ARES_ASSERT(name != nullptr);

    // Duplicate constant check.
    for (uint8_t i = 0; i < program_.constCount; i++)
    {
        if (strcmp(program_.consts[i].name, name) == 0)
        {
            char msg[64] = {};
            snprintf(msg, sizeof(msg), "duplicate const declaration: %s", name);
            setErrorLocked(msg);
            return false;
        }
    }

    // Name must not shadow an existing variable.
    for (uint8_t i = 0; i < program_.varCount; i++)
    {
        if (strcmp(program_.vars[i].name, name) == 0)
        {
            setErrorLocked("const name conflicts with an existing variable");
            return false;
        }
    }

    return true;
}

// ── findConstLocked ──────────────────────────────────────────────────────────

/**
 * @brief Look up a named constant by identifier.
 *
 * @param[in] name  Constant name to find.
 * @return Pointer to the @c ConstEntry, or @c nullptr if not found.
 * @pre  Caller holds the engine mutex.
 */
const MissionScriptEngine::ConstEntry*
MissionScriptEngine::findConstLocked(const char* name) const
{
    if (name == nullptr) { return nullptr; }
    for (uint8_t i = 0; i < program_.constCount; i++)
    {
        if (strcmp(program_.consts[i].name, name) == 0)
        {
            return &program_.consts[i];
        }
    }
    return nullptr;
}

} // namespace ams
} // namespace ares
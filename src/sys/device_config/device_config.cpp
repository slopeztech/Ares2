/**
 * @file  device_config.cpp
 * @brief DeviceConfig — load / save / validate / serialise.
 *
 * JSON schema stored in LittleFS (/ares_device.json):
 * @code
 * {
 *   "_schema":      1,
 *   "wifi_password": "ares1234",
 *   "api_token":     "",
 *   "cors_origin":   "*"
 * }
 * @endcode
 *
 * Only the fields the operator provides are updated (patch semantics).
 * "_schema" is written on save and read for future migration but not
 * exposed through the public API yet.
 */

#include "sys/device_config/device_config.h"
#include "debug/ares_log.h"

#include <ArduinoJson.h>
#include <cstring>
#include <cstdio>

static constexpr const char* TAG = "DEVCFG";

/// JSON schema version — increment when the field set changes in a way
/// that requires migration logic in load().
static constexpr uint8_t SCHEMA_VERSION = 1U;

/// Maximum raw JSON file size accepted from storage.
static constexpr uint32_t JSON_READ_MAX = 512U;

// ── Construction ──────────────────────────────────────────────

DeviceConfig::DeviceConfig()
{
    setDefaults();
}

void DeviceConfig::setDefaults()
{
    // WiFi: use the compile-time factory password.
    (void)strncpy(wifiPassword_, ares::WIFI_AP_PASSWORD,
                  sizeof(wifiPassword_) - 1U);
    wifiPassword_[sizeof(wifiPassword_) - 1U] = '\0';

    // Token: empty → auth disabled.
    apiToken_[0] = '\0';

    // CORS: open by default (legacy behaviour).
    (void)strncpy(corsOrigin_, "*", sizeof(corsOrigin_) - 1U);
    corsOrigin_[sizeof(corsOrigin_) - 1U] = '\0';
}

// ── Persistence ───────────────────────────────────────────────

bool DeviceConfig::load(StorageInterface* storage)
{
    setDefaults();

    if (storage == nullptr)
    {
        LOG_I(TAG, "no storage — using defaults");
        return false;
    }

    bool exists = false;
    if (storage->exists(ares::DEVICE_CONFIG_PATH, exists) != StorageStatus::OK
        || !exists)
    {
        LOG_I(TAG, "config absent — using defaults (open mode)");
        return false;
    }

    // Read raw bytes into a local buffer.
    uint8_t rawBuf[JSON_READ_MAX + 1U] = {};
    uint32_t bytesRead = 0U;
    if (storage->readFile(ares::DEVICE_CONFIG_PATH,
                          rawBuf, JSON_READ_MAX,
                          bytesRead) != StorageStatus::OK)
    {
        LOG_W(TAG, "read error — using defaults");
        return false;
    }
    rawBuf[bytesRead] = '\0';

    char errBuf[96] = {};
    if (!applyJson(reinterpret_cast<const char*>(rawBuf), bytesRead,
                   errBuf, static_cast<uint8_t>(sizeof(errBuf))))
    {
        LOG_W(TAG, "parse/validation error (%s) — using defaults", errBuf);
        setDefaults();
        return false;
    }

    LOG_I(TAG, "loaded (auth=%s cors=%s)",
          isAuthEnabled() ? "ON" : "OFF",
          corsOrigin_);
    return true;
}

bool DeviceConfig::save(StorageInterface* storage) const
{
    if (storage == nullptr)
    {
        return false;
    }

    JsonDocument doc;
    doc["_schema"]      = SCHEMA_VERSION;
    doc["wifi_password"] = wifiPassword_;
    doc["api_token"]     = apiToken_;
    doc["cors_origin"]   = corsOrigin_;

    char buf[JSON_READ_MAX] = {};
    const size_t len = serializeJson(doc, buf, sizeof(buf) - 1U);
    buf[len] = '\0';

    const StorageStatus st =
        storage->writeFile(ares::DEVICE_CONFIG_PATH,
                           reinterpret_cast<const uint8_t*>(buf),
                           static_cast<uint32_t>(len));
    if (st != StorageStatus::OK)
    {
        LOG_E(TAG, "save failed: status=%d", static_cast<int>(st));
        return false;
    }

    LOG_I(TAG, "saved to %s", ares::DEVICE_CONFIG_PATH);
    return true;
}

// ── Validation helpers ─────────────────────────────────────────

/** Returns true if every byte in [s, s+len) is printable ASCII (0x20-0x7E). */
static bool isPrintableAscii(const char* s, size_t len)
{
    for (size_t i = 0U; i < len; i++)
    {
        const unsigned char c = static_cast<unsigned char>(s[i]);
        if (c < 0x20U || c > 0x7EU)
        {
            return false;
        }
    }
    return true;
}

// ── applyJson field helpers ────────────────────────────────────

/** Write @p msg into @p errOut (bounded, NUL-safe) and return false. */
static bool setFieldErr(char* errOut, uint8_t errLen, const char* msg)
{
    if (errOut != nullptr && errLen > 0U)
    {
        (void)strncpy(errOut, msg, errLen - 1U);
        errOut[errLen - 1U] = '\0';
    }
    return false;
}

/**
 * Validate and copy the wifi_password field from a JSON variant.
 * Absent field is a no-op (returns true without setting @p changed).
 */
static bool validateWifiPass(JsonVariant v,
                              char* out, size_t outSize, bool& changed,
                              char* errOut, uint8_t errLen)
{
    if (v.isNull())           { return true; }
    if (!v.is<const char*>()) { return setFieldErr(errOut, errLen, "wifi_password must be a string"); }

    const char* val = nullptr;
    val = v.as<const char*>();
    if (val == nullptr) { val = ""; }

    const size_t vlen = strlen(val);
    if (vlen != 0U && (vlen < 8U || vlen > 63U))
    {
        return setFieldErr(errOut, errLen,
            "wifi_password must be 8-63 chars or empty (factory default)");
    }
    if (!isPrintableAscii(val, vlen))
    {
        return setFieldErr(errOut, errLen, "wifi_password: non-printable characters");
    }
    const char* src = (vlen == 0U) ? ares::WIFI_AP_PASSWORD : val;
    (void)strncpy(out, src, outSize - 1U);
    out[outSize - 1U] = '\0';
    changed = true;
    return true;
}

/**
 * Validate and copy the api_token field from a JSON variant.
 * Empty string is accepted (disables auth).
 */
static bool validateToken(JsonVariant v,
                           char* out, size_t outSize, bool& changed,
                           char* errOut, uint8_t errLen)
{
    if (v.isNull())           { return true; }
    if (!v.is<const char*>()) { return setFieldErr(errOut, errLen, "api_token must be a string"); }

    const char* val = nullptr;
    val = v.as<const char*>();
    if (val == nullptr) { val = ""; }

    const size_t vlen = strlen(val);
    if (vlen != 0U && (vlen < 8U || vlen > 64U))
    {
        return setFieldErr(errOut, errLen,
            "api_token must be 8-64 chars or empty (auth disabled)");
    }
    if (!isPrintableAscii(val, vlen))
    {
        return setFieldErr(errOut, errLen, "api_token: non-printable characters");
    }
    (void)strncpy(out, val, outSize - 1U);
    out[outSize - 1U] = '\0';
    changed = true;
    return true;
}

/** Validate and copy the cors_origin field from a JSON variant. */
static bool validateCors(JsonVariant v,
                          char* out, size_t outSize, bool& changed,
                          char* errOut, uint8_t errLen)
{
    if (v.isNull())           { return true; }
    if (!v.is<const char*>()) { return setFieldErr(errOut, errLen, "cors_origin must be a string"); }

    const char* val = nullptr;
    val = v.as<const char*>();
    if (val == nullptr) { val = ""; }

    const size_t vlen = strlen(val);
    if (vlen == 0U || vlen >= outSize)
    {
        char tmp[80] = {};
        (void)snprintf(tmp, sizeof(tmp), "cors_origin must be 1-%u chars",
                       static_cast<unsigned>(outSize - 1U));
        return setFieldErr(errOut, errLen, tmp);
    }
    (void)strncpy(out, val, outSize - 1U);
    out[outSize - 1U] = '\0';
    changed = true;
    return true;
}

// ── applyJson ─────────────────────────────────────────────────

bool DeviceConfig::applyJson(const char* json, uint32_t len,
                              char* errOut, uint8_t errLen)
{
    if (json == nullptr || len == 0U)
    {
        return setFieldErr(errOut, errLen, "empty body");
    }

    JsonDocument doc;
    const DeserializationError err = deserializeJson(doc, json, len);
    if (err)
    {
        char tmp[80] = {};
        (void)snprintf(tmp, sizeof(tmp), "JSON: %s", err.c_str());
        return setFieldErr(errOut, errLen, tmp);
    }

    // ── Phase 1: validate all present fields into local temporaries ──────
    char newWifiPass[ares::DEVICE_WIFI_PASS_MAX]  = {};
    char newToken   [ares::DEVICE_TOKEN_MAX]       = {};
    char newCors    [ares::DEVICE_CORS_ORIGIN_MAX] = {};
    bool wifiPassChanged = false;
    bool tokenChanged    = false;
    bool corsChanged     = false;

    if (!validateWifiPass(doc["wifi_password"],
                          newWifiPass, sizeof(newWifiPass),
                          wifiPassChanged, errOut, errLen)) { return false; }
    if (!validateToken(doc["api_token"],
                       newToken, sizeof(newToken),
                       tokenChanged, errOut, errLen))       { return false; }
    if (!validateCors(doc["cors_origin"],
                      newCors, sizeof(newCors),
                      corsChanged, errOut, errLen))         { return false; }

    // ── Phase 2: commit all validated fields atomically ──────────────────
    if (wifiPassChanged) { (void)memcpy(wifiPassword_, newWifiPass, sizeof(wifiPassword_)); }
    if (tokenChanged)    { (void)memcpy(apiToken_,     newToken,    sizeof(apiToken_));     }
    if (corsChanged)     { (void)memcpy(corsOrigin_,   newCors,     sizeof(corsOrigin_));   }

    return true;
}

// ── toPublicJson ───────────────────────────────────────────────

uint32_t DeviceConfig::toPublicJson(char* buf, uint32_t bufSize) const
{
    JsonDocument doc;
    doc["_schema"]      = SCHEMA_VERSION;
    doc["wifi_password"] = wifiPassword_;
    // api_token intentionally omitted — write-only through this interface.
    doc["cors_origin"]  = corsOrigin_;
    doc["auth_enabled"] = isAuthEnabled();

    const size_t len = serializeJson(doc, buf, bufSize - 1U);
    buf[len] = '\0';
    return static_cast<uint32_t>(len);
}

// ── Field accessors ────────────────────────────────────────────

const char* DeviceConfig::wifiPassword() const
{
    return wifiPassword_;
}

const char* DeviceConfig::corsOrigin() const
{
    return corsOrigin_;
}

bool DeviceConfig::isAuthEnabled() const
{
    return apiToken_[0] != '\0';
}

bool DeviceConfig::checkToken(const char* provided) const
{
    if (provided == nullptr || !isAuthEnabled())
    {
        return false;
    }

    // Constant-time comparison over the full stored-token length.
    // The loop always runs storedLen iterations regardless of where a
    // mismatch occurs, preventing timing side-channels (OWASP A07:2021).
    const size_t storedLen   = strlen(apiToken_);
    const size_t providedLen = strlen(provided);

    // Different length → guaranteed fail, but we still run the full loop
    // to avoid leaking the stored-token length through timing.
    uint8_t diff = (storedLen == providedLen) ? 0U : 1U;

    for (size_t i = 0U; i < storedLen; i++)
    {
        const char c = (i < providedLen) ? provided[i] : '\0';
        diff |= static_cast<uint8_t>(static_cast<uint8_t>(apiToken_[i])
                                   ^ static_cast<uint8_t>(c));
    }

    return diff == 0U;
}

// ── CORS header builder ────────────────────────────────────────

void DeviceConfig::buildCorsHeader(char* out, size_t outSize) const
{
    (void)snprintf(out, outSize,
                   "Access-Control-Allow-Origin: %s\r\n"
                   "Access-Control-Allow-Methods: GET, PUT, POST, DELETE, OPTIONS\r\n"
                   "Access-Control-Allow-Headers: Content-Type, X-ARES-Token\r\n",
                   corsOrigin_);
}

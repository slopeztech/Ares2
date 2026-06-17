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
#include "sys/hardware/installed_driver_registry.h"

#include <ArduinoJson.h>
#include <cstring>
#include <cstdio>

static constexpr const char* TAG = "DEVCFG";

/// JSON schema version — increment when the field set changes in a way
/// that requires migration logic in load().
static constexpr uint8_t SCHEMA_VERSION = 3U;

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

    // Radio MAC key: empty → open mode (no MAC required).
    radioKeyHex_[0] = '\0';

    (void)strncpy(defaultGpsDriver_, ares::hardware::defaultGpsDriverModel(),
                  sizeof(defaultGpsDriver_) - 1U);
    defaultGpsDriver_[sizeof(defaultGpsDriver_) - 1U] = '\0';

    (void)strncpy(defaultBaroDriver_, ares::hardware::defaultBaroDriverModel(),
                  sizeof(defaultBaroDriver_) - 1U);
    defaultBaroDriver_[sizeof(defaultBaroDriver_) - 1U] = '\0';

    (void)strncpy(defaultComDriver_, ares::hardware::defaultComDriverModel(),
                  sizeof(defaultComDriver_) - 1U);
    defaultComDriver_[sizeof(defaultComDriver_) - 1U] = '\0';

    // Default IMU model for API status/imu endpoints.
    (void)strncpy(defaultImuDriver_, ares::hardware::defaultImuDriverModel(),
                  sizeof(defaultImuDriver_) - 1U);
    defaultImuDriver_[sizeof(defaultImuDriver_) - 1U] = '\0';
}

// ── Persistence ───────────────────────────────────────────────

bool DeviceConfig::load(StorageInterface* storage)
{
    firstBootProvisioned_ = false;
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

    static char errBuf[96] = {};
    if (!applyJson(reinterpret_cast<const char*>(rawBuf), bytesRead,
                   errBuf, static_cast<uint8_t>(sizeof(errBuf))))
    {
        LOG_W(TAG, "parse/validation error (%s) — using defaults", errBuf);
        setDefaults();
        return false;
    }

    LOG_I(TAG, "loaded (auth=%s cors=%s mac=%s)",
          isAuthEnabled() ? "ON" : "OFF",
          corsOrigin_,
          radioKeyHex_[0] != '\0' ? "ON" : "OFF");
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
    doc["radio_key"]     = radioKeyHex_;
    doc["default_gps_driver"] = defaultGpsDriver_;
    doc["default_baro_driver"] = defaultBaroDriver_;
    doc["default_com_driver"] = defaultComDriver_;
    doc["default_imu_driver"] = defaultImuDriver_;

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

// ── First-boot provisioning ────────────────────────────────────

// Platform-independent random-word helper.
// On ESP32 Arduino builds: uses the hardware TRNG (esp_random()).
// On native host builds (pio test -e native): falls back to rand() — only
// used in tests, never in production firmware.
#ifdef ARDUINO
#  include <esp_random.h>
static uint32_t platformRandom32() { return esp_random(); }
#else
#  include <cstdlib>
static uint32_t platformRandom32() { return static_cast<uint32_t>(rand()); }
#endif

/** Fill @p out with @p count lowercase hex characters, then NUL-terminate. */
static void fillHexRandom(char* out, size_t count)
{
    static constexpr char kHex[] = "0123456789abcdef";
    size_t pos = 0U;
    while (pos < count)
    {
        const uint32_t word = platformRandom32();
        for (uint8_t b = 0U; b < 4U && pos < count; b++)
        {
            const uint8_t byte = static_cast<uint8_t>((word >> (b * 8U)) & 0xFFU);
            out[pos++] = kHex[byte >> 4U];
            if (pos < count) { out[pos++] = kHex[byte & 0x0FU]; }
        }
    }
    out[pos] = '\0';
}

/** Fill @p out with @p count alphanumeric characters, then NUL-terminate. */
static void fillAlnumRandom(char* out, size_t count)
{
    static constexpr const char kAlnum[] =
        "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
        "abcdefghijklmnopqrstuvwxyz"
        "0123456789";
    static constexpr size_t kAlnumLen = sizeof(kAlnum) - 1U;
    size_t pos = 0U;
    while (pos < count)
    {
        const uint32_t word = platformRandom32();
        for (uint8_t b = 0U; b < 4U && pos < count; b++)
        {
            const uint8_t byte = static_cast<uint8_t>((word >> (b * 8U)) & 0xFFU);
            out[pos++] = kAlnum[byte % kAlnumLen];
        }
    }
    out[pos] = '\0';
}

bool DeviceConfig::provisionIfFirstBoot(StorageInterface* storage)
{
    firstBootProvisioned_ = false;
    if (storage == nullptr) { return false; }

    // Generate a 12-char alphanumeric WPA2-PSK password (≥ 8 chars, well
    // within 63-char limit), a 32-char hex API bearer token (128-bit
    // entropy), and a 32-char hex radio MAC key (128-bit entropy).
    char newPass [13U] = {};  // 12 chars + NUL
    char newToken[33U] = {};  // 32 hex chars + NUL
    char newKey  [33U] = {};  // 32 hex chars + NUL
    fillAlnumRandom(newPass,  12U);
    fillHexRandom  (newToken, 32U);
    fillHexRandom  (newKey,   32U);

    (void)strncpy(wifiPassword_, newPass,  sizeof(wifiPassword_) - 1U);
    wifiPassword_[sizeof(wifiPassword_) - 1U] = '\0';
    (void)strncpy(apiToken_,     newToken, sizeof(apiToken_) - 1U);
    apiToken_    [sizeof(apiToken_) - 1U]     = '\0';
    (void)strncpy(radioKeyHex_,  newKey,   sizeof(radioKeyHex_) - 1U);
    radioKeyHex_ [sizeof(radioKeyHex_) - 1U] = '\0';

    if (!save(storage))
    {
        // Persist failed — roll back to open defaults so the device still
        // boots normally (missing config → setDefaults() path).
        setDefaults();
        LOG_E(TAG, "first-boot: credential save failed — reverting to defaults");
        return false;
    }

    // Print bootstrap credentials to USB-CDC.  The operator must read them
    // at first power-on; they are not shown again after the first reboot.
    LOG_I(TAG, "═══════════════════════════════════════════════════════");
    LOG_I(TAG, "FIRST BOOT — ARES security credentials generated");
    LOG_I(TAG, "  WiFi AP password : %s", wifiPassword_);
    LOG_I(TAG, "  API bearer token : %s", apiToken_);
    LOG_I(TAG, "  Radio MAC key    : %s", radioKeyHex_);
    LOG_I(TAG, "  Record these now. They will NOT be shown again.");
    LOG_I(TAG, "═══════════════════════════════════════════════════════");

    firstBootProvisioned_ = true;
    return true;
}

bool DeviceConfig::wasFirstBootProvisioned() const
{
    return firstBootProvisioned_;
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
static bool validateWifiPass(JsonVariantConst v,
                              char* out, size_t outSize, bool& changed,
                              char* errOut, uint8_t errLen)
{
    if (v.isNull())           { return true; }
    if (!v.is<const char*>()) { return setFieldErr(errOut, errLen, "wifi_password must be a string"); }

    const char* val = nullptr;
    val = v.as<const char*>();
    if (val == nullptr) { val = ""; }

    const size_t vlen = strlen(val);
    if (vlen == 0U)
    {
        // Empty field — preserve the currently stored password instead of
        // falling back to the compile-time factory default (M10).
        return true;
    }
    if (vlen < 8U || vlen > 63U)
    {
        return setFieldErr(errOut, errLen,
            "wifi_password must be 8-63 chars or empty (keep current)");
    }
    if (!isPrintableAscii(val, vlen))
    {
        return setFieldErr(errOut, errLen, "wifi_password: non-printable characters");
    }
    (void)strncpy(out, val, outSize - 1U);
    out[outSize - 1U] = '\0';
    changed = true;
    return true;
}

/**
 * Validate and copy the api_token field from a JSON variant.
 * Empty string is accepted (disables auth).
 */
static bool validateToken(JsonVariantConst v,
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
static bool validateCors(JsonVariantConst v,
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
        static char tmp[80] = {};
        (void)snprintf(tmp, sizeof(tmp), "cors_origin must be 1-%u chars",
                       static_cast<unsigned>(outSize - 1U));
        return setFieldErr(errOut, errLen, tmp);
    }
    (void)strncpy(out, val, outSize - 1U);
    out[outSize - 1U] = '\0';
    changed = true;
    return true;
}

/**
 * Validate and copy the radio_key field from a JSON variant.
 * Must be empty (disables MAC) or exactly 32 lowercase hex characters.
 */
static bool validateRadioKey(JsonVariantConst v,
                              char* out, size_t outSize, bool& changed,
                              char* errOut, uint8_t errLen)
{
    if (v.isNull())           { return true; }
    if (!v.is<const char*>()) { return setFieldErr(errOut, errLen, "radio_key must be a string"); }

    const char* val = nullptr;
    val = v.as<const char*>();
    if (val == nullptr) { val = ""; }

    const size_t vlen = strlen(val);
    // Accept empty (disables MAC) or exactly 32 lowercase hex chars.
    if (vlen != 0U && vlen != 32U)
    {
        return setFieldErr(errOut, errLen,
            "radio_key must be empty (disable MAC) or exactly 32 lowercase hex chars");
    }
    for (size_t i = 0U; i < vlen; i++)
    {
        const char c = val[i];
        const bool isHex = (c >= '0' && c <= '9') || (c >= 'a' && c <= 'f');
        if (!isHex)
        {
            return setFieldErr(errOut, errLen,
                "radio_key must contain only lowercase hex digits (0-9, a-f)");
        }
    }
    (void)strncpy(out, val, outSize - 1U);
    out[outSize - 1U] = '\0';
    changed = true;
    return true;
}

static bool validateDefaultGpsDriver(JsonVariantConst v,
                                     char* out, size_t outSize, bool& changed,
                                     char* errOut, uint8_t errLen)
{
    if (v.isNull())           { return true; }
    if (!v.is<const char*>()) { return setFieldErr(errOut, errLen, "default_gps_driver must be a string"); }

    const char* val = v.as<const char*>();
    if (val == nullptr) { val = ""; }

    const size_t vlen = strlen(val);
    if (vlen == 0U || vlen >= outSize)
    {
        return setFieldErr(errOut, errLen,
            "default_gps_driver must be a non-empty supported model");
    }

    const bool supported = ares::hardware::isSupportedModel(ares::hardware::DriverKind::GPS,
                                                            val);
    if (!supported)
    {
        return setFieldErr(errOut, errLen,
            "default_gps_driver must name an installed GPS driver");
    }

    (void)strncpy(out, val, outSize - 1U);
    out[outSize - 1U] = '\0';
    changed = true;
    return true;
}

static bool validateDefaultBaroDriver(JsonVariantConst v,
                                      char* out, size_t outSize, bool& changed,
                                      char* errOut, uint8_t errLen)
{
    if (v.isNull())           { return true; }
    if (!v.is<const char*>()) { return setFieldErr(errOut, errLen, "default_baro_driver must be a string"); }

    const char* val = v.as<const char*>();
    if (val == nullptr) { val = ""; }

    const size_t vlen = strlen(val);
    if (vlen == 0U || vlen >= outSize)
    {
        return setFieldErr(errOut, errLen,
            "default_baro_driver must be a non-empty supported model");
    }

    const bool supported = ares::hardware::isSupportedModel(ares::hardware::DriverKind::BARO,
                                                            val);
    if (!supported)
    {
        return setFieldErr(errOut, errLen,
            "default_baro_driver must name an installed BARO driver");
    }

    (void)strncpy(out, val, outSize - 1U);
    out[outSize - 1U] = '\0';
    changed = true;
    return true;
}

static bool validateDefaultComDriver(JsonVariantConst v,
                                     char* out, size_t outSize, bool& changed,
                                     char* errOut, uint8_t errLen)
{
    if (v.isNull())           { return true; }
    if (!v.is<const char*>()) { return setFieldErr(errOut, errLen, "default_com_driver must be a string"); }

    const char* val = v.as<const char*>();
    if (val == nullptr) { val = ""; }

    const size_t vlen = strlen(val);
    if (vlen == 0U || vlen >= outSize)
    {
        return setFieldErr(errOut, errLen,
            "default_com_driver must be a non-empty supported model");
    }

    const bool supported = ares::hardware::isSupportedModel(ares::hardware::DriverKind::COM,
                                                            val);
    if (!supported)
    {
        return setFieldErr(errOut, errLen,
            "default_com_driver must name an installed COM driver");
    }

    (void)strncpy(out, val, outSize - 1U);
    out[outSize - 1U] = '\0';
    changed = true;
    return true;
}

/** Validate and copy the default_imu_driver field from a JSON variant. */
static bool validateDefaultImuDriver(JsonVariantConst v,
                                     char* out, size_t outSize, bool& changed,
                                     char* errOut, uint8_t errLen)
{
    if (v.isNull())           { return true; }
    if (!v.is<const char*>()) { return setFieldErr(errOut, errLen, "default_imu_driver must be a string"); }

    const char* val = nullptr;
    val = v.as<const char*>();
    if (val == nullptr) { val = ""; }

    const size_t vlen = strlen(val);
    if (vlen == 0U || vlen >= outSize)
    {
        return setFieldErr(errOut, errLen,
            "default_imu_driver must be a non-empty supported model");
    }

    const bool supported = ares::hardware::isSupportedModel(ares::hardware::DriverKind::IMU,
                                                            val);
    if (!supported)
    {
        return setFieldErr(errOut, errLen,
            "default_imu_driver must name an installed IMU driver");
    }

    (void)strncpy(out, val, outSize - 1U);
    out[outSize - 1U] = '\0';
    changed = true;
    return true;
}

struct PendingDeviceConfigPatch
{
    char newWifiPass[ares::DEVICE_WIFI_PASS_MAX]     = {};
    char newToken[ares::DEVICE_TOKEN_MAX]            = {};
    char newCors[ares::DEVICE_CORS_ORIGIN_MAX]       = {};
    char newRadioKey[ares::DEVICE_RADIO_KEY_HEX_MAX] = {};
    char newDefaultGps[ares::DEVICE_IMU_DRIVER_MAX]  = {};
    char newDefaultBaro[ares::DEVICE_IMU_DRIVER_MAX] = {};
    char newDefaultCom[ares::DEVICE_IMU_DRIVER_MAX]  = {};
    char newDefaultImu[ares::DEVICE_IMU_DRIVER_MAX]  = {};

    bool wifiPassChanged   = false;
    bool tokenChanged      = false;
    bool corsChanged       = false;
    bool radioKeyChanged   = false;
    bool defaultGpsChanged = false;
    bool defaultBaroChanged = false;
    bool defaultComChanged = false;
    bool defaultImuChanged = false;
};

static bool validatePatchDocument(const JsonDocument& doc,
                                  PendingDeviceConfigPatch& patch,
                                  char* errOut,
                                  uint8_t errLen)
{
    if (!validateWifiPass(doc["wifi_password"],
                          patch.newWifiPass, sizeof(patch.newWifiPass),
                          patch.wifiPassChanged, errOut, errLen)) { return false; }
    if (!validateToken(doc["api_token"],
                       patch.newToken, sizeof(patch.newToken),
                       patch.tokenChanged, errOut, errLen)) { return false; }
    if (!validateCors(doc["cors_origin"],
                      patch.newCors, sizeof(patch.newCors),
                      patch.corsChanged, errOut, errLen)) { return false; }
    if (!validateRadioKey(doc["radio_key"],
                          patch.newRadioKey, sizeof(patch.newRadioKey),
                          patch.radioKeyChanged, errOut, errLen)) { return false; }
    if (!validateDefaultGpsDriver(doc["default_gps_driver"],
                                  patch.newDefaultGps, sizeof(patch.newDefaultGps),
                                  patch.defaultGpsChanged, errOut, errLen)) { return false; }
    if (!validateDefaultBaroDriver(doc["default_baro_driver"],
                                   patch.newDefaultBaro, sizeof(patch.newDefaultBaro),
                                   patch.defaultBaroChanged, errOut, errLen)) { return false; }
    if (!validateDefaultComDriver(doc["default_com_driver"],
                                  patch.newDefaultCom, sizeof(patch.newDefaultCom),
                                  patch.defaultComChanged, errOut, errLen)) { return false; }
    if (!validateDefaultImuDriver(doc["default_imu_driver"],
                                  patch.newDefaultImu, sizeof(patch.newDefaultImu),
                                  patch.defaultImuChanged, errOut, errLen)) { return false; }

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
        static char tmp[80] = {};
        (void)snprintf(tmp, sizeof(tmp), "JSON: %s", err.c_str());
        return setFieldErr(errOut, errLen, tmp);
    }

    PendingDeviceConfigPatch patch = {};
    if (!validatePatchDocument(doc, patch, errOut, errLen))
    {
        return false;
    }

    if (patch.wifiPassChanged)
    {
        (void)memcpy(wifiPassword_, patch.newWifiPass, sizeof(wifiPassword_));
    }
    if (patch.tokenChanged)
    {
        (void)memcpy(apiToken_, patch.newToken, sizeof(apiToken_));
    }
    if (patch.corsChanged)
    {
        (void)memcpy(corsOrigin_, patch.newCors, sizeof(corsOrigin_));
    }
    if (patch.radioKeyChanged)
    {
        (void)memcpy(radioKeyHex_, patch.newRadioKey, sizeof(radioKeyHex_));
    }
    if (patch.defaultGpsChanged)
    {
        (void)memcpy(defaultGpsDriver_, patch.newDefaultGps, sizeof(defaultGpsDriver_));
    }
    if (patch.defaultBaroChanged)
    {
        (void)memcpy(defaultBaroDriver_, patch.newDefaultBaro, sizeof(defaultBaroDriver_));
    }
    if (patch.defaultComChanged)
    {
        (void)memcpy(defaultComDriver_, patch.newDefaultCom, sizeof(defaultComDriver_));
    }
    if (patch.defaultImuChanged)
    {
        (void)memcpy(defaultImuDriver_, patch.newDefaultImu, sizeof(defaultImuDriver_));
    }

    return true;
}

// ── toPublicJson ───────────────────────────────────────────────

uint32_t DeviceConfig::toPublicJson(char* buf, uint32_t bufSize) const
{
    // Guard: reject buffers too small to hold a minimal valid JSON response.
    // Avoids writing buf[len] = '\0' out-of-bounds when bufSize is 0 (M7).
    if (bufSize < 16U) { return 0U; }
    JsonDocument doc;
    doc["_schema"]      = SCHEMA_VERSION;
    doc["wifi_password"] = wifiPassword_;
    // api_token intentionally omitted — write-only through this interface.
    doc["cors_origin"]  = corsOrigin_;
    doc["auth_enabled"] = isAuthEnabled();
    doc["default_gps_driver"] = defaultGpsDriver_;
    doc["default_baro_driver"] = defaultBaroDriver_;
    doc["default_com_driver"] = defaultComDriver_;
    doc["default_imu_driver"] = defaultImuDriver_;
    // radio_key intentionally omitted — secret, write-only (same policy as api_token).

    const size_t len = serializeJson(doc, buf, bufSize - 1U);
    buf[len] = '\0';
    return static_cast<uint32_t>(len);
}

// ── Field accessors ────────────────────────────────────────────

const char* DeviceConfig::wifiPassword() const
{
    return wifiPassword_;
}

const char* DeviceConfig::apiToken() const
{
    return apiToken_;
}

const char* DeviceConfig::corsOrigin() const
{
    return corsOrigin_;
}

bool DeviceConfig::isAuthEnabled() const
{
    return apiToken_[0] != '\0';
}

const char* DeviceConfig::defaultImuDriver() const
{
    return defaultImuDriver_;
}

const char* DeviceConfig::defaultGpsDriver() const
{
    return defaultGpsDriver_;
}

const char* DeviceConfig::defaultBaroDriver() const
{
    return defaultBaroDriver_;
}

const char* DeviceConfig::defaultComDriver() const
{
    return defaultComDriver_;
}

bool DeviceConfig::checkToken(const char* provided) const
{
    if (provided == nullptr || !isAuthEnabled())
    {
        return false;
    }

    // Constant-time comparison over the full length of both tokens.
    // The loop runs max(storedLen, providedLen) iterations so that:
    //   1. Different lengths always fail (diff already set to 1).
    //   2. Extra bytes from the attacker beyond storedLen are still XOR'd,
    //      preventing a timing oracle on the stored-token length (OWASP A07).
    const size_t storedLen   = strlen(apiToken_);
    const size_t providedLen = strlen(provided);
    const size_t maxLen      = (storedLen > providedLen) ? storedLen : providedLen;

    // Different length → guaranteed fail, but we still run the full loop
    // to avoid leaking the stored-token length through timing.
    uint8_t diff = (storedLen == providedLen) ? 0U : 1U;

    for (size_t i = 0U; i < maxLen; i++)
    {
        const char stored_c   = (i < storedLen)   ? apiToken_[i] : '\0';
        const char provided_c = (i < providedLen) ? provided[i]  : '\0';
        diff |= static_cast<uint8_t>(static_cast<uint8_t>(stored_c)
                                   ^ static_cast<uint8_t>(provided_c));
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

// ── Radio MAC key accessor ─────────────────────────────────────

bool DeviceConfig::radioKey(uint8_t* out, uint8_t outLen) const
{
    // Key length in binary bytes derived from the hex string constant.
    // DEVICE_RADIO_KEY_HEX_MAX = 33 (32 hex chars + NUL) → 16 binary bytes.
    static constexpr uint8_t kKeyBytes =
        static_cast<uint8_t>((ares::DEVICE_RADIO_KEY_HEX_MAX - 1U) / 2U);

    if (out == nullptr || outLen < kKeyBytes)
    {
        return false;
    }
    if (radioKeyHex_[0] == '\0')
    {
        return false;  // No key provisioned — open mode.
    }

    // Decode the 32-char hex string into 16 binary bytes.
    for (uint8_t i = 0U; i < kKeyBytes; i++)
    {
        const char hi = radioKeyHex_[static_cast<uint8_t>(i * 2U)];
        const char lo = radioKeyHex_[static_cast<uint8_t>(i * 2U + 1U)];

        const uint8_t hiNibble = (hi >= '0' && hi <= '9')
            ? static_cast<uint8_t>(hi - '0')
            : static_cast<uint8_t>(hi - 'a' + 10U);
        const uint8_t loNibble = (lo >= '0' && lo <= '9')
            ? static_cast<uint8_t>(lo - '0')
            : static_cast<uint8_t>(lo - 'a' + 10U);

        out[i] = static_cast<uint8_t>((hiNibble << 4U) | loNibble);
    }
    return true;
}

/**
 * @file  device_config.h
 * @brief Persistent device security configuration for ARES.
 *
 * Stores security-sensitive runtime parameters in LittleFS as a JSON
 * document at @c /ares_device.json.  If the file is absent or cannot be
 * parsed, all fields fall back to compile-time defaults from config.h
 * (backward-compatible open mode — no auth, factory WiFi password).
 *
 * Configurable parameters (schema v3):
 * | Key            | Type   | Default       | Description                            |
 * |----------------|--------|---------------|----------------------------------------|
 * | wifi_password  | string | "ares1234"    | WPA2-PSK for the soft-AP.              |
 * |                |        |               | Effective after reboot.  8-63 chars.   |
 * | api_token      | string | ""            | Shared bearer token (X-ARES-Token).    |
 * |                |        |               | Empty = auth disabled.  8-64 chars.    |
 * | cors_origin    | string | "*"           | Access-Control-Allow-Origin value.     |
 * |                |        |               | Use "*", "null", or a specific origin. |
 * | default_imu_driver | string | "MPU6050" | IMU model used by status/imu endpoints |
 * |                |        |               | and other default API sensor reads.    |
 * | default_gps_driver | string | "BN220" | GPS model used by status endpoints      |
 * | default_baro_driver | string | "BMP280" | BARO model used by status endpoints   |
 * | default_com_driver | string | "DXLR03" | COM/radio model used by runtime I/O    |
 *
 * Auth behaviour:
 *   - When api_token is empty: all endpoints are open (legacy behaviour).
 *   - When api_token is set: GET /api/status, /api/imu and sub-paths remain public;
 *     every other endpoint requires the header  X-ARES-Token: <token>.
 *     Missing or wrong token → HTTP 401.
 *
 * Extending the schema:
 *   1. Add a private field with its default in setDefaults().
 *   2. Parse it in applyJson() — unknown keys are silently ignored,
 *      so older firmware continues to load newer config files.
 *   3. Expose it in toPublicJson() and in an accessor method.
 *   4. Bump SCHEMA_VERSION in device_config.cpp.
 *
 * Thread safety:
 *   DeviceConfig is loaded once in setup() before any RTOS task starts.
 *   After that it is mutated exclusively from the API task via applyJson()
 *   (PUT /api/device/config).  No internal mutex is required.
 */
#pragma once

#include "config.h"
#include "hal/storage/storage_interface.h"

#include <cstddef>
#include <cstdint>

/**
 * @brief Persistent device security configuration.
 *
 * Statically allocated — no heap (PO10-3).
 */
class DeviceConfig
{
public:
    /** @brief Construct with compile-time factory defaults. */
    DeviceConfig();

    // Non-copyable, non-movable (CERT-18.3)
    DeviceConfig(const DeviceConfig&)            = delete;
    DeviceConfig& operator=(const DeviceConfig&) = delete;
    DeviceConfig(DeviceConfig&&)                 = delete;
    DeviceConfig& operator=(DeviceConfig&&)      = delete;

    // ── Persistence ──────────────────────────────────────────

    /**
     * Load configuration from LittleFS.
     *
     * If the file does not exist, or if parsing fails, all fields
     * are set to their compile-time defaults.
     *
     * @param[in] storage  Storage backend.  Passing nullptr uses defaults.
     * @return true if a valid config file was found and applied,
     *         false if defaults were used.
     */
    bool load(StorageInterface* storage);

    /**
     * Generate random WiFi password and API token on first boot (no config
     * file present) and persist them immediately.
     *
     * Must be called right after load() returns false and the storage
     * backend is valid.  On success the generated credentials are logged
     * to USB-CDC via LOG_I so the operator can read them at first
     * power-on.  Subsequent boots load the persisted values normally.
     *
     * @param[in] storage  Storage backend used to write the config file.
     *                     Must not be nullptr.
     * @return true if credentials were generated and persisted successfully.
     */
    bool provisionIfFirstBoot(StorageInterface* storage);

    /**
     * @return true if provisionIfFirstBoot() ran and saved this session.
     *         Reset to false on the next load() call.
     */
    bool wasFirstBootProvisioned() const;

    /**
     * Persist the current configuration (including api_token) to LittleFS.
     *
     * @param[in] storage  Storage backend.  Must not be nullptr.
     * @return true on success.
     */
    bool save(StorageInterface* storage) const;

    // ── Runtime update ────────────────────────────────────────

    /**
     * Validate and apply a JSON patch document.
     *
     * Implements REST-5 (validate-all before mutating any field):
     * if any present field fails validation the entire update is rejected
     * and the object's state is unchanged.
     *
     * Unknown JSON keys are silently ignored (forward-compatibility).
     *
     * @param json    Pointer to JSON text (not necessarily NUL-terminated;
     *                only the first @p len bytes are examined).
     * @param len     Length of the JSON body in bytes.
     * @param errOut  Buffer for a human-readable error message on failure.
     *                May be nullptr if the caller does not need the message.
     * @param errLen  Capacity of @p errOut.
     * @return true on success, false if any field failed validation.
     */
    bool applyJson(const char* json, uint32_t len,
                   char* errOut, uint8_t errLen);

    /**
     * Serialise the current configuration to JSON, omitting api_token.
     *
     * The token is write-only through this interface to prevent accidental
     * leakage in GET responses.
     *
     * @param[out] buf      Destination buffer.
     * @param[in]  bufSize  Capacity of @p buf (must be > 0).
     * @return Number of bytes written (excluding the NUL terminator).
     */
    uint32_t toPublicJson(char* buf, uint32_t bufSize) const;

    // ── Field accessors ───────────────────────────────────────

    /**
     * @return WPA2-PSK password (always non-empty; falls back to factory default).
     */
    const char* wifiPassword() const;

    /**
     * @return Current X-ARES API token (empty when auth is disabled).
     */
    const char* apiToken() const;

    /**
     * @return CORS Access-Control-Allow-Origin value (e.g. "*").
     */
    const char* corsOrigin() const;

    /**
     * @return true if an api_token is configured and HTTP auth is required.
     */
    bool isAuthEnabled() const;

    /**
     * @return Model name used as default IMU for API sensor endpoints.
     */
    const char* defaultImuDriver() const;

    /** @return Model name used as default GPS for API endpoints. */
    const char* defaultGpsDriver() const;

    /** @return Model name used as default BARO for API endpoints. */
    const char* defaultBaroDriver() const;

    /** @return Model name used as default COM/radio driver. */
    const char* defaultComDriver() const;

    /**
     * Constant-time token comparison (prevents timing side-channels).
     *
     * A byte-XOR accumulator is used so the comparison always runs the
     * full length of the stored token regardless of where a mismatch occurs.
     *
     * @param[in] provided  NUL-terminated candidate token from the HTTP header.
     *                  May be nullptr (treated as empty).
     * @return true if @p provided exactly matches the stored token.
     *         Always false when auth is disabled or when @p provided is nullptr.
     */
    bool checkToken(const char* provided) const;

    /**
     * Write the CORS response-header block into @p out.
     *
     * Output format (three header lines, each ending with CRLF):
     * @code
     *   Access-Control-Allow-Origin: <origin>\r\n
     *   Access-Control-Allow-Methods: GET, PUT, POST, DELETE, OPTIONS\r\n
     *   Access-Control-Allow-Headers: Content-Type, X-ARES-Token\r\n
     * @endcode
     *
     * @param[out] out      Destination buffer.
     * @param[in]  outSize  Capacity of @p out.
     */
    void buildCorsHeader(char* out, size_t outSize) const;

    /**
     * Decode the stored radio MAC key from hex and write it to @p out.
     *
     * Returns false (and writes nothing) when no key is provisioned (open mode)
     * or when @p outLen is less than proto::HMAC_KEY_LEN (16).
     *
     * @param[out] out     Buffer to receive the 16-byte binary key.
     * @param[in]  outLen  Must be >= proto::HMAC_KEY_LEN.
     * @return true if a key was decoded and written to @p out.
     */
    bool radioKey(uint8_t* out, uint8_t outLen) const;

private:
    /** Reset all fields to compile-time defaults. */
    void setDefaults();

    char wifiPassword_[ares::DEVICE_WIFI_PASS_MAX]  = {};  ///< WPA2-PSK password.
    char apiToken_    [ares::DEVICE_TOKEN_MAX]       = {};  ///< Bearer token (empty = disabled).
    char corsOrigin_  [ares::DEVICE_CORS_ORIGIN_MAX] = {};  ///< CORS origin value.
    char radioKeyHex_ [ares::DEVICE_RADIO_KEY_HEX_MAX] = {}; ///< Radio MAC key as hex (empty = open mode).
    char defaultGpsDriver_[ares::DEVICE_IMU_DRIVER_MAX]  = {}; ///< Default API GPS model name.
    char defaultBaroDriver_[ares::DEVICE_IMU_DRIVER_MAX] = {}; ///< Default API BARO model name.
    char defaultComDriver_[ares::DEVICE_IMU_DRIVER_MAX]  = {}; ///< Default COM/radio model name.
    char defaultImuDriver_[ares::DEVICE_IMU_DRIVER_MAX] = {}; ///< Default API IMU model name.

    /** Set by provisionIfFirstBoot() when new credentials were generated this session. */
    bool firstBootProvisioned_ = false;
};

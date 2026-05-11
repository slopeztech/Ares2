/**
 * @file  api_server.h
 * @brief HTTP REST API server for ground configuration over WiFi.
 *
 * Implements the ARES REST API Standard (REST-1 … REST-14).
 * Runs as a dedicated FreeRTOS task (REST-13, RTOS-2) with
 * static stack allocation (PO10-3, RTOS-7).
 *
 * Endpoints:
 *   GET    /api/status      — system status and sensor data
 *   GET    /api/imu         — IMU snapshot
 *   GET    /api/imu/health  — IMU health only
 *   GET    /api/config      — current runtime configuration
 *   PUT    /api/config      — update runtime configuration
 *   POST   /api/mode        — change operating mode
 *   POST   /api/arm         — arm flight FSM
 *   POST   /api/abort       — abort current flight
 *   POST   /api/scans/i2c   — scan configured I2C buses
 *   POST   /api/scans/uart  — scan configured UART ports
 *   GET    /api/logs        — list log files (REST-12)
 *   GET    /api/logs/:id    — download log file
 *   DELETE /api/logs        — delete all logs
 *   DELETE /api/logs/:id    — delete single log
 *
 * Thread safety:
 *   - Operating mode is accessed via std::atomic (lock-free).
 *   - Runtime config is protected by a mutex with 50 ms
 *     timeout (RTOS-4, CERT-10).
 *   - Sensor readings are snapshot-copied under mutex.
 */
#pragma once

#include <atomic>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

#include "ams/mission_script_engine.h"
#include "config.h"
#include "hal/baro/barometer_interface.h"
#include "hal/gps/gps_interface.h"
#include "hal/imu/imu_interface.h"
#include "hal/pulse/pulse_interface.h"
#include "hal/radio/radio_interface.h"
#include "hal/storage/storage_interface.h"
#include "sys/wifi/wifi_ap.h"

class StatusLed;  ///< Forward declaration — avoids circular headers.
class TwoWire;
class HardwareSerial;

/**
 * Runtime configuration struct.
 * Protected by ApiServer's config mutex (REST-8).
 */
struct RuntimeConfig
{
    uint32_t telemetryIntervalMs = ares::TELEMETRY_INTERVAL_MS;
    uint8_t  nodeId              = ares::DEFAULT_NODE_ID;
    uint8_t  ledBrightness       = ares::DEFAULT_LED_BRIGHTNESS;
};

/**
 * HTTP REST API server.
 *
 * Owns a FreeRTOS task that polls a WiFiServer for incoming
 * HTTP connections.  Each connection is parsed, routed, and
 * responded to before being closed (REST-10.3: no keep-alive).
 *
 * All JSON serialisation uses JsonDocument (ArduinoJson v7) with
 * bounded output buffers (REST-4.1, PO10-3).
 */
class ApiServer
{
public:
    /**
     * Construct the API server.
     * @param[in] wifi       Reference to the WiFi AP service.
     * @param[in] baro       Reference to the barometer interface.
     * @param[in] gps        Reference to the GPS interface.
     * @param[in] imu        Reference to the IMU interface.
     * @param[in] storage    Pointer to storage interface (nullable).
     * @param[in] mission    Pointer to AMS runtime (nullable).
     * @param[in] statusLed  Pointer to status LED service (nullable).
     * @param[in] i2c0       Pointer to I2C0 bus (nullable).
     * @param[in] i2c1       Pointer to I2C1 bus (nullable).
     * @param[in] gpsUart    Pointer to GPS UART port (nullable).
     * @param[in] loraUart   Pointer to LoRa UART port (nullable).
     * @param[in] radio      Pointer to radio interface (nullable).
     * @param[in] pulse      Pointer to pulse channel interface (nullable).
     */
    ApiServer(WifiAp& wifi, BarometerInterface& baro,
              GpsInterface& gps, ImuInterface& imu,
              StorageInterface* storage = nullptr,
              ares::ams::MissionScriptEngine* mission = nullptr,
              StatusLed* statusLed = nullptr,
              TwoWire* i2c0 = nullptr,
              TwoWire* i2c1 = nullptr,
              HardwareSerial* gpsUart = nullptr,
              HardwareSerial* loraUart = nullptr,
              RadioInterface* radio = nullptr,
              PulseInterface* pulse = nullptr);

    // Non-copyable, non-movable (CERT-18.3)
    ApiServer(const ApiServer&)            = delete;
    ApiServer& operator=(const ApiServer&) = delete;
    ApiServer(ApiServer&&)                 = delete;
    ApiServer& operator=(ApiServer&&)      = delete;

    /**
     * Create the TCP server and start the RTOS task.
     * @return true on success.
     * @pre  WifiAp::begin() returned true.
     */
    bool begin();

    /**
     * Set the operating mode (lock-free, any task).
     * @param[in] mode  New mode to apply.
     */
    void setMode(ares::OperatingMode mode);

    /**
     * @return Current operating mode.
     */
    ares::OperatingMode getMode() const;

    /**
     * @brief  Notify that the active AMS mission has reached COMPLETE or ERROR.
     *
     * Called by the main loop when the AMS engine terminates normally or faults.
     * Resets armed flag and transitions operating mode back to IDLE.
     * Safe to call from any task (lock-free).
     *
     * @post   `isArmed()` returns false; operating mode is IDLE.
     */
    void notifyMissionComplete();

    /**
     * @brief  Notify that a persisted AMS mission has been auto-resumed after reboot.
     *
     * Called when AMS is auto-restored and already RUNNING.
     * Sets armed flag and FLIGHT mode so API state matches mission runtime.
     *
     * @post   `isArmed()` returns true; operating mode is FLIGHT.
     */
    void notifyMissionResumed();

    /**
     * @return Read-only reference to current config.
     * @warning Not thread-safe -- use getConfigCopy() from other tasks.
     */
    const RuntimeConfig& config() const;

    /**
     * Thread-safe config snapshot.
     * @param[out] out  Populated with a copy of the current config.
     * @return true if mutex acquired, false on timeout.
     */
    bool getConfigCopy(RuntimeConfig& out) const;

private:
    /// RTOS task entry point.
    static void taskFn(void* param);

    /// Main task loop.
    void run();

    /// Parse one HTTP request from a connected client.
    void handleClient(class WiFiClient& client);

    // ── Route handlers (implemented in subdirectories) ─────
    // status/
    void handleStatus(class WiFiClient& client);
    void handleImuGet(class WiFiClient& client);
    void handleImuHealth(class WiFiClient& client);
    // config/
    void handleConfigGet(class WiFiClient& client);
    void handleConfigPut(class WiFiClient& client,
                         const char* body, uint32_t bodyLen);
    // flight/
    void handleMode(class WiFiClient& client,
                    const char* body, uint32_t bodyLen);
    void handleArm(class WiFiClient& client);
    void handleAbort(class WiFiClient& client);
    void handleI2cScan(class WiFiClient& client);
    void handleUartScan(class WiFiClient& client);
    /// Execute the arm sequence; send error and return false on failure.
    bool executeArm(class WiFiClient& client);
    // storage/
    void handleLogsList(class WiFiClient& client);
    void handleLogDownload(class WiFiClient& client,
                           const char* filename);
    void handleLogDelete(class WiFiClient& client,
                         const char* filename);
    void handleLogDeleteAll(class WiFiClient& client);
    void handleStorageHealth(class WiFiClient& client);
    // mission/
    void handleMissionStatus(class WiFiClient& client);
    void handleMissionList(class WiFiClient& client);
    void handleMissionDownload(class WiFiClient& client,
                               const char* filename);
    void handleMissionUpload(class WiFiClient& client,
                             const char* filename,
                             const char* body,
                             uint32_t bodyLen);
    void handleMissionDelete(class WiFiClient& client,
                             const char* filename);
    void handleMissionActivate(class WiFiClient& client,
                               const char* body, uint32_t bodyLen);
    void handleMissionDeactivate(class WiFiClient& client);
    void handleMissionCommand(class WiFiClient& client,
                              const char* body, uint32_t bodyLen);
    // pulse/
    void handlePulseStatus(class WiFiClient& client);  ///< GET /api/pulse/status (AMS-4.17)

    // ── Request routing (PO10-4.1: decomposed from handleClient) ─
    void routeRequest(class WiFiClient& client,
                      const char* method, const char* path,
                      const char* body, uint32_t bodyLen);
    bool routeStatusAndConfigRequest(class WiFiClient& client,
                                     const char* method,
                                     const char* path,
                                     const char* body,
                                     uint32_t bodyLen);
    bool routeFlightRequest(class WiFiClient& client,
                            const char* method,
                            const char* path,
                            const char* body,
                            uint32_t bodyLen);
    bool routeMissionTopLevelRequest(class WiFiClient& client,
                                     const char* method,
                                     const char* path,
                                     const char* body,
                                     uint32_t bodyLen);
    void routeLogRequest(class WiFiClient& client,
                         const char* method, const char* sub);
    void routeMissionRequest(class WiFiClient& client,
                             const char* method, const char* sub,
                             const char* body, uint32_t bodyLen);

    // ── Response helpers ────────────────────────────────────
    static void sendJson(class WiFiClient& client, uint16_t code,
                         const char* json, uint32_t len);
    static void sendError(class WiFiClient& client, uint16_t code,
                          const char* message);
    static void sendNoContent(class WiFiClient& client, uint16_t code);

    /**
     * @brief Send a file from storage as a chunked HTTP 200 response.
     *
     * Writes the HTTP headers (Content-Type, Content-Length,
     * Content-Disposition, CORS) then streams the file in
     * @c DOWNLOAD_CHUNK_SIZE blocks until all bytes are sent or the
     * client disconnects.
     *
     * @param client        Connected WiFi client.
     * @param path          Absolute storage path to read from.
     * @param filename      Bare filename used in Content-Disposition.
     * @param contentType   MIME type string (e.g. "application/octet-stream").
     * @param fileSize      Total byte length of the file (pre-queried by caller).
     */
    void sendFileChunked(class WiFiClient& client,
                         const char* path,
                         const char* filename,
                         const char* contentType,
                         uint32_t    fileSize);

    /// Check if current mode forbids write operations (REST-6.2).
    bool isFlightLocked() const;

    /// Convert OperatingMode to string.
    static const char* modeToString(ares::OperatingMode mode);

    // ── References ──────────────────────────────────────────
    WifiAp&              wifi_;
    BarometerInterface&  baro_;
    GpsInterface&        gps_;
    ImuInterface&        imu_;
    StorageInterface*    storage_;  ///< Nullable — logs disabled if null.
    ares::ams::MissionScriptEngine* mission_; ///< Nullable — AMS disabled if null.
    StatusLed*           statusLed_; ///< Nullable — LED updates disabled if null.
    TwoWire*             i2c0_;      ///< Nullable — board I2C0 bus.
    TwoWire*             i2c1_;      ///< Nullable — board I2C1 bus.
    HardwareSerial*      gpsUart_;   ///< Nullable — UART1 monitor handle.
    HardwareSerial*      loraUart_;  ///< Nullable — UART2 monitor handle.
    RadioInterface*      radio_;     ///< Nullable — radio health probe.
    PulseInterface*      pulse_;     ///< Nullable — pulse status disabled if null.

    // ── Shared state ────────────────────────────────────────
    RuntimeConfig config_ = {};

    /// Config mutex — static buffer (PO10-3, RTOS-7).
    StaticSemaphore_t cfgMtxBuf_ = {};
    SemaphoreHandle_t cfgMtx_    = nullptr;

    /// Operating mode — lock-free (REST-8.1).
    std::atomic<uint8_t> mode_{
        static_cast<uint8_t>(ares::OperatingMode::IDLE)};

    /// Armed flag.
    std::atomic<bool> armed_{false};

    // ── RTOS task (static allocation) ───────────────────────
    StackType_t stack_[ares::TASK_STACK_SIZE_API / sizeof(StackType_t)] = {};
    StaticTask_t tcb_ = {};
};

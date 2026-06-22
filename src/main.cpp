/**
 * @file  main.cpp
 * @brief ARES runtime bootstrap (quiet mode).
 *
 * Initialises subsystems (sensors, radio, storage, WiFi, API, AMS)
 * and then remains idle until external commands arrive through the
 * REST API / mission runtime.
 *
 * All objects are statically allocated (no `new`) so heap
 * usage stays at zero (PO10-3).
 */

#include "config.h"
#include "hal/baro/barometer_interface.h"
#include "hal/gps/gps_interface.h"
#include "hal/imu/imu_interface.h"
#include "hal/led/led_interface.h"
#include "hal/radio/radio_interface.h"
#include "drivers/baro/bmp280_driver.h"
#include "drivers/gps/bn220_driver.h"
#include "drivers/imu/mpu6050_driver.h"
#include "drivers/imu/adxl375_driver.h"
#include "drivers/radio/dxlr03_driver.h"
#include "drivers/pulse/pulse_driver.h"
#include "drivers/serial/arduino_serial_interface.h"
#include "sys/baro/baro_selector.h"
#include "sys/com/com_selector.h"
#include "sys/gps/gps_selector.h"
#include "sys/imu/imu_selector.h"
#include "sys/hardware/installed_driver_registry.h"
#include "sys/led/neopixel_driver.h"
#include "sys/led/status_led.h"
#include "sys/wifi/wifi_ap.h"
#include "sys/storage/littlefs_storage.h"
#include "sys/device_config/device_config.h"
#include "api/api_server.h"
#include "ams/mission_script_engine.h"
#include "hal/millis64.h"
#include "comms/ares_radio_protocol.h"
#include "comms/radio_dispatcher.h"

#include <Arduino.h>
#include <freertos/task.h>
#include <Wire.h>
#include <new>
#include <esp_system.h>
#include <esp_task_wdt.h>

#include "debug/ares_log.h"

// ── Peripherals (static allocation, no heap) ───────────────
// Concrete drivers are created here and exposed below as
// interface references.  This is the only place in the
// codebase that knows which sensor hardware is installed.
static HardwareSerial gpsSerial(ares::GPS_UART_PORT);
static HardwareSerial loraSerial(ares::LORA_UART_PORT);
static TwoWire        imuWire(1);
// baro: deferred to setup() — constructor references Wire (another TU).
// Placement new in setup() eliminates the cross-TU static-init-order
// dependency (SIOF, P1-3).  Static storage keeps PO10-3 (no heap).
alignas(Bmp280Driver) static uint8_t  s_baroBuf[sizeof(Bmp280Driver)];
static Bmp280Driver*                  pBaro = nullptr;
static Bn220Driver    gps(gpsSerial, ares::PIN_GPS_RX,
                          ares::PIN_GPS_TX, ares::GPS_BAUD);
static DxLr03Driver   radio(loraSerial, ares::PIN_LORA_TX,
                             ares::PIN_LORA_RX, ares::PIN_LORA_AUX,
                             ares::LORA_UART_BAUD);
static Mpu6050Driver  imu(imuWire, ares::MPU6050_I2C_ADDR);
static Adxl375Driver  imu2(imuWire, ares::ADXL375_I2C_ADDR);

static void bindInstalledDrivers(BarometerInterface* barometer,
                                 SerialInterface* serialOut)
{
    const ares::hardware::InstalledHardwareRefs refs = {
        barometer,
        &gps,
        &radio,
        &imu,
        &imu2,
        serialOut
    };
    ares::hardware::bindInstalledHardware(refs);
}

struct InstalledDriversBootstrap
{
    InstalledDriversBootstrap()
    {
        bindInstalledDrivers(nullptr, nullptr);
    }
};

static InstalledDriversBootstrap s_installedDriversBootstrap;

static BaroSelector apiBaroSelector(ares::hardware::baroDrivers().entries,
                                    ares::hardware::baroDrivers().count,
                                    0U);
static BaroSelectorControl apiBaroSelectorControl(apiBaroSelector);

static GpsSelector apiGpsSelector(ares::hardware::gpsDrivers().entries,
                                  ares::hardware::gpsDrivers().count,
                                  0U);
static GpsSelectorControl apiGpsSelectorControl(apiGpsSelector);

static ComSelector apiComSelector(ares::hardware::comDrivers().entries,
                                  ares::hardware::comDrivers().count,
                                  0U);
static ComSelectorControl apiComSelectorControl(apiComSelector);

static ImuSelector apiImuSelector(ares::hardware::imuDrivers().entries,
                                  ares::hardware::imuDrivers().count,
                                  0U);
static ImuSelectorControl apiImuSelectorControl(apiImuSelector);
static NeopixelDriver led(ares::PIN_LED_RGB);
static StatusLed      statusLed(led);
// Pulse channel configuration.  Fire and continuity-sense GPIOs come from
// config.h.  Set PIN_PULSE_x = PIN_NO_FIRE to disable a fire channel; its GPIO
// can then be assigned to PIN_PULSE_y_CONT for another channel's continuity
// sense without any change here (see config.h for the 2-fire + 2-cont example).
static const PulseDriver::ChannelConfig kPulseChannels[PulseChannel::COUNT] = {
    { ares::PIN_PULSE_A, ares::PIN_PULSE_A_CONT },  // CH_A
    { ares::PIN_PULSE_B, ares::PIN_PULSE_B_CONT },  // CH_B
    { ares::PIN_PULSE_C, ares::PIN_PULSE_C_CONT },  // CH_C
    { ares::PIN_PULSE_D, ares::PIN_PULSE_D_CONT },  // CH_D
};
static PulseDriver pulse(kPulseChannels);
static ArduinoSerialInterface serialOut(Serial);

static SerialInterface* resolveSerialOutput()
{
    const ares::hardware::DriverList<ares::hardware::SerialDriverEntry>& serials =
        ares::hardware::serialDrivers();
    if (serials.count == 0U || serials.entries == nullptr)
    {
        return nullptr;
    }
    return serials.entries[0U].iface;
}

// WiFi Access Point (sys layer) — ground configuration link.
static WifiAp wifiAp;

// Persistent device security configuration (WiFi password, API token, CORS).
// Loaded from LittleFS (/ares_device.json) in setup() before wifiAp.begin().
static DeviceConfig deviceConfig;

// On-board flash filesystem for flight logs.
static LittleFsStorage storage;

// Interface references — all downstream code uses these.
static LedInterface&       ledIf     = led;
static StorageInterface&   storageIf = storage;
static RadioInterface&     radioIf   = apiComSelector;

// REST API server — receives references to interfaces.
static ares::ams::MissionScriptEngine missionEngine(
    storageIf,
    ares::hardware::gpsDrivers().entries,  ares::hardware::gpsDrivers().count,
    ares::hardware::baroDrivers().entries, ares::hardware::baroDrivers().count,
    ares::hardware::comDrivers().entries,  ares::hardware::comDrivers().count,
    ares::hardware::imuDrivers().entries,  ares::hardware::imuDrivers().count,
    &pulse);
// Radio dispatcher — polls the LoRa receive FIFO and dispatches inbound APUS
// frames (APUS-4.4).  Sends acceptance ACK / NACK for every COMMAND (APUS-9).
static ares::RadioDispatcher radioDispatcher(radioIf, missionEngine, &pulse);

// apiServer: deferred to setup() — takes *s_baroIfaces[0] which requires baro
// to be live first, and baro is deferred (P1-3).  Static storage (PO10-3).
alignas(ApiServer) static uint8_t  s_apiBuf[sizeof(ApiServer)];
static ApiServer*                  pApiServer = nullptr;

// ═══════════════════════════════════════════════════════════
/// @brief Inspect the reset cause and apply an AMS boot checkpoint if the
///        mission was running when the board restarted.  Called once from setup().
static void applyBootCheckpoint(ares::ams::MissionScriptEngine& engine,
                                 ApiServer& api,
                                 StatusLed& led)
{
    // Classify the reset cause before acting on the restored checkpoint.
    // Abnormal causes (panic, WDT, brownout) while in-flight trigger TC.RESET_ABNORMAL
    // so the user's AMS script can decide how to respond (e.g. deploy parachute).
    const esp_reset_reason_t resetCause = esp_reset_reason();
    const bool abnormalReset =
        (resetCause == ESP_RST_PANIC    ||
         resetCause == ESP_RST_INT_WDT  ||
         resetCause == ESP_RST_TASK_WDT ||
         resetCause == ESP_RST_WDT      ||
         resetCause == ESP_RST_BROWNOUT);

    // If AMS restored an in-flight checkpoint after reboot, reflect it in API mode.
    ares::ams::EngineSnapshot bootSnap = {};
    engine.getSnapshot(bootSnap);
    if (bootSnap.status == ares::ams::EngineStatus::RUNNING)
    {
        if (abnormalReset)
        {
            // In-flight + abnormal reset: inject TC so the script handles recovery.
            // The TC is consumed on the first tick — zero latency penalty.
            LOG_W("BOOT", "Abnormal reset during flight (cause=%d) — injecting TC.RESET_ABNORMAL",
                  static_cast<int>(resetCause));
            (void)engine.injectTcCommand("RESET_ABNORMAL");
        }
        api.notifyMissionResumed();
    }
    else
    {
        // All subsystems ready — exit boot blink, go solid green (IDLE)
        led.setMode(ares::OperatingMode::IDLE);
    }
}

// ═══════════════════════════════════════════════════════════
/**
 * @brief RAII guard: ensures the status LED transitions out of BOOT on every
 *        exit path from setup() — normal completion or early return.
 *        Destructor forces ERROR when the mode is still BOOT at scope exit,
 *        making any bypassed applyBootCheckpoint() immediately visible (BUG-18).
 */
struct LedBootGuard
{
    explicit LedBootGuard(StatusLed& l) : led_(l) {}
    ~LedBootGuard()
    {
        if (led_.getMode() == ares::OperatingMode::BOOT)
        {
            led_.setMode(ares::OperatingMode::ERROR);
        }
    }
    LedBootGuard(const LedBootGuard&)            = delete;  // CERT-18.3
    LedBootGuard& operator=(const LedBootGuard&) = delete;
    LedBootGuard(LedBootGuard&&)                 = delete;
    LedBootGuard& operator=(LedBootGuard&&)      = delete;
private:
    StatusLed& led_;
};

// ═══════════════════════════════════════════════════════════
/// @brief Load the provisioned radio MAC key into the dispatcher (APUS-17).
///        No-op when no key has been provisioned yet (open / dev mode).
static void applyRadioMacKey(const DeviceConfig& cfg, ares::RadioDispatcher& dispatcher)
{
    uint8_t radioKeyBuf[ares::proto::HMAC_KEY_LEN] = {};
    if (cfg.radioKey(radioKeyBuf, static_cast<uint8_t>(sizeof(radioKeyBuf))))
    {
        dispatcher.setMacKey(radioKeyBuf, static_cast<uint8_t>(sizeof(radioKeyBuf)));
    }
}

// ═════════════════════════════════════════════════════════
/// Apply state-level AMS directives to the live WiFi/API runtime.
/// Each directive is applied independently so AMS can disable the AP, the
/// REST listener, or both for a given mission state.
static void applyMissionStateDirective(bool wifiEnabled, bool apiEnabled)
{
    if (pApiServer != nullptr)
    {
        pApiServer->setWifiEnabled(wifiEnabled);
        pApiServer->setApiEnabled(apiEnabled);
    }
}

static bool startApiServer()
{
    ApiServer* p = nullptr;
    p = new (s_apiBuf) ApiServer(
        wifiAp,
        apiBaroSelector,
        apiGpsSelector,
        apiImuSelector,
        deviceConfig,
        &storageIf, &missionEngine,
        &statusLed,
        &Wire, &imuWire,
        &gpsSerial, &loraSerial,
        &radioIf,
        &pulse,
        &radioDispatcher,
        &apiImuSelectorControl,
        &apiGpsSelectorControl,
        &apiBaroSelectorControl,
        &apiComSelectorControl);
    if (!p->begin()) { return false; }
    pApiServer = p;
    return true;
}

static void logRuntimeSecrets(uint32_t now,
                              uint32_t& lastWifiPasswordPrintMs,
                              uint32_t& lastApiTokenPrintMs)
{
    if (ares::WIFI_PASSWORD_SERIAL_LOG_ENABLED
        && (now - lastWifiPasswordPrintMs) >= ares::WIFI_PASSWORD_SERIAL_LOG_INTERVAL_MS)
    {
        Serial.printf("WiFi AP password: %s\n", deviceConfig.wifiPassword());
        lastWifiPasswordPrintMs = now;
    }

    if (ares::API_TOKEN_SERIAL_LOG_ENABLED
        && deviceConfig.isAuthEnabled()
        && (now - lastApiTokenPrintMs) >= ares::API_TOKEN_SERIAL_LOG_INTERVAL_MS)
    {
        Serial.printf("X-ARES token: %s\n", deviceConfig.apiToken());
        lastApiTokenPrintMs = now;
    }
}

static void updateMissionCompletionStatus(ApiServer& api,
                                          const ares::ams::MissionScriptEngine& engine,
                                          ares::ams::EngineStatus& lastNotifiedStatus)
{
    if (api.getMode() != ares::OperatingMode::FLIGHT)
    {
        lastNotifiedStatus = ares::ams::EngineStatus::IDLE;
        return;
    }

    ares::ams::EngineSnapshot snap = {};
    engine.getSnapshot(snap);
    if ((snap.status == ares::ams::EngineStatus::COMPLETE
         || snap.status == ares::ams::EngineStatus::ERROR)
        && snap.status != lastNotifiedStatus)
    {
        api.notifyMissionComplete();
        lastNotifiedStatus = snap.status;
    }
}

static uint32_t computeSleepMs(uint64_t nowAms,
                               const ares::ams::MissionScriptEngine& engine)
{
    const uint64_t wakeupMs  = engine.nextWakeupMs(nowAms);
    const uint64_t sleepMs64 = (wakeupMs > nowAms) ? (wakeupMs - nowAms) : 1ULL;
    return static_cast<uint32_t>(sleepMs64 > static_cast<uint64_t>(ares::LOOP_SLEEP_MAX_MS)
                                 ? ares::LOOP_SLEEP_MAX_MS
                                 : sleepMs64);
}

// ═════════════════════════════════════════════════════════
void setup() // NOLINT(readability-function-size)
{
    // Keep USB serial available for on-demand diagnostics, but do not
    // emit boot banners or periodic traces in normal operation.
    Serial.begin(ares::SERIAL_BAUD);

    // I2C buses — initialise before any I2C driver.
    // I2C0 (Wire): shared board sensors (BMP280 on GPIO 1/2).
    Wire.begin(ares::PIN_I2C_SDA, ares::PIN_I2C_SCL, ares::I2C_FREQ);
    Wire.setTimeOut(ares::I2C_TIMEOUT_MS);
    // [P1-3] Construct baro here — Wire is now initialised (SIOF fix).
    pBaro = new (s_baroBuf) Bmp280Driver(Wire, ares::BMP280_I2C_ADDR);
    bindInstalledDrivers(pBaro, &serialOut);
    // I2C1: dedicated IMU bus (MPU6050 on GPIO 12/13).
    // Uses 100 kHz standard mode — GY-521 pull-ups (10 kΩ) are not reliable at 400 kHz.
    imuWire.begin(ares::PIN_IMU_SDA, ares::PIN_IMU_SCL, ares::I2C_FREQ_IMU);
    imuWire.setTimeOut(ares::I2C_TIMEOUT_MS);

    const ares::hardware::DriverList<ares::hardware::BaroDriverEntry>& baroDrivers =
        ares::hardware::baroDrivers();
    for (uint8_t i = 0U; i < baroDrivers.count; i++)
    {
        BarometerInterface* const iface = baroDrivers.entries[i].iface;
        if (iface == nullptr) { continue; }
        if (!iface->begin()) { LOG_W("BOOT", "baro begin() failed"); }
    }

    const ares::hardware::DriverList<ares::hardware::ImuDriverEntry>& imuDrivers =
        ares::hardware::imuDrivers();
    for (uint8_t i = 0U; i < imuDrivers.count; i++)
    {
        ImuInterface* const iface = imuDrivers.entries[i].iface;
        if (iface == nullptr) { continue; }
        if (!iface->begin()) { LOG_W("BOOT", "imu begin() failed"); }
    }

    const ares::hardware::DriverList<ares::hardware::GpsDriverEntry>& gpsDrivers =
        ares::hardware::gpsDrivers();
    for (uint8_t i = 0U; i < gpsDrivers.count; i++)
    {
        GpsInterface* const iface = gpsDrivers.entries[i].iface;
        if (iface == nullptr) { continue; }
        if (!iface->begin()) { LOG_W("BOOT", "gps begin() failed"); }
    }
    if (!pulse.begin()) { LOG_W("BOOT", "pulse begin() failed"); }

    // Status LED — NeoPixel on GPIO 21
    (void)ledIf.begin();
    ledIf.setBrightness(ares::DEFAULT_LED_BRIGHTNESS);
    statusLed.begin();  // starts RTOS task — fast green blink (BOOT)
    LedBootGuard bootGuard{statusLed};  // BUG-18: force ERROR on any early exit

    // On-board flash storage (LittleFS)
    if (!storageIf.begin())
    {
        LOG_E("BOOT", "storage init failed");
        return;
    }

    // Device security config — load before WiFi AP; generate fresh random
    // credentials on first boot via TRNG (provisionIfFirstBoot prints them).
    if (!deviceConfig.load(&storageIf))
    {
        // Config absent (first boot) or corrupted — generate fresh credentials.
        (void)deviceConfig.provisionIfFirstBoot(&storageIf);
    }

    (void)apiBaroSelector.selectDriverByName(deviceConfig.defaultBaroDriver());
    (void)apiGpsSelector.selectDriverByName(deviceConfig.defaultGpsDriver());
    (void)apiComSelector.selectDriverByName(deviceConfig.defaultComDriver());
    (void)apiImuSelector.selectDriverByName(deviceConfig.defaultImuDriver());

    // Wire radio MAC key into the dispatcher (APUS-17).
    applyRadioMacKey(deviceConfig, radioDispatcher);

    // COM transceiver(s)
    (void)radioIf.begin();

    // WiFi AP — must be up before API server.
    // Password comes from deviceConfig (factory default when no file present).
    (void)wifiAp.begin(deviceConfig);

    // AMS runtime (IDLE by default, waits for API activation)
    if (!missionEngine.begin())
    {
        LOG_E("BOOT", "AMS engine init failed");
        return;
    }

    missionEngine.setSerialInterface(resolveSerialOutput());

    missionEngine.setStateDirectiveCallback(applyMissionStateDirective);

    // REST API server — construct + start after AMS is ready to avoid init race
    if (!startApiServer())
    {
        LOG_E("BOOT", "API server init failed");
        return;
    }

    // Classify reset cause, restore in-flight checkpoint if needed,
    // and set the initial LED mode.
    applyBootCheckpoint(missionEngine, *pApiServer, statusLed);

    // Subscribe loop() to the TWDT — registered last to avoid false trips
    // during the subsystem init sequence above.
    (void)esp_task_wdt_add(nullptr);
}

// ═══════════════════════════════════════════════════════════
void loop()
{
    if (pApiServer == nullptr)
    {
        vTaskDelay(pdMS_TO_TICKS(ares::LOOP_SLEEP_MAX_MS));
        return;
    }

    esp_task_wdt_reset();

    static uint32_t lastWifiPasswordPrintMs = 0U;
    static uint32_t lastApiTokenPrintMs = 0U;
    static ares::ams::EngineStatus lastNotifiedStatus = ares::ams::EngineStatus::IDLE;

    const uint32_t now      = static_cast<uint32_t>(millis());
    const uint64_t nowAms   = millis64();
    const uint64_t tickStart = nowAms;

    const ares::hardware::DriverList<ares::hardware::GpsDriverEntry>& gpsDrivers =
        ares::hardware::gpsDrivers();
    for (uint8_t i = 0U; i < gpsDrivers.count; i++)
    {
        GpsInterface* const iface = gpsDrivers.entries[i].iface;
        if (iface != nullptr) { iface->update(); }
    }
    radioDispatcher.poll(now);
    logRuntimeSecrets(now, lastWifiPasswordPrintMs, lastApiTokenPrintMs);

    missionEngine.tick(nowAms);
    updateMissionCompletionStatus(*pApiServer, missionEngine, lastNotifiedStatus);

    const uint32_t tickMs = static_cast<uint32_t>(millis64() - tickStart);
    if (tickMs > ares::LOOP_TICK_WARN_MS)
    {
        LOG_W("LOOP", "tick overrun: %" PRIu32 " ms (budget=%" PRIu32 " ms)",
              tickMs, ares::LOOP_TICK_WARN_MS);
    }

    const uint32_t sleepMs = computeSleepMs(nowAms, missionEngine);
    vTaskDelay(pdMS_TO_TICKS(sleepMs));
}

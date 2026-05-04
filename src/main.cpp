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
#include "sys/led/neopixel_driver.h"
#include "sys/led/status_led.h"
#include "sys/wifi/wifi_ap.h"
#include "sys/storage/littlefs_storage.h"
#include "api/api_server.h"
#include "ams/mission_script_engine.h"
#include "comms/ares_radio_protocol.h"

#include <Arduino.h>
#include <freertos/task.h>
#include <Wire.h>

// ── Peripherals (static allocation, no heap) ───────────────
// Concrete drivers are created here and exposed below as
// interface references.  This is the only place in the
// codebase that knows which sensor hardware is installed.
static HardwareSerial gpsSerial(ares::GPS_UART_PORT);
static HardwareSerial loraSerial(ares::LORA_UART_PORT);
static TwoWire        imuWire(1);
static Bmp280Driver   baro(Wire, ares::BMP280_I2C_ADDR);
static Bn220Driver    gps(gpsSerial, ares::PIN_GPS_RX,
                          ares::PIN_GPS_TX, ares::GPS_BAUD);
static DxLr03Driver   radio(loraSerial, ares::PIN_LORA_TX,
                             ares::PIN_LORA_RX, ares::PIN_LORA_AUX,
                             ares::LORA_UART_BAUD);
static Mpu6050Driver  imu(imuWire, ares::MPU6050_I2C_ADDR);
static Adxl375Driver  imu2(imuWire, ares::ADXL375_I2C_ADDR);
static NeopixelDriver led(ares::PIN_LED_RGB);
static StatusLed      statusLed(led);

// WiFi Access Point (sys layer) — ground configuration link.
static WifiAp wifiAp;

// On-board flash filesystem for flight logs.
static LittleFsStorage storage;

// Interface references — all downstream code uses these.
static LedInterface&       ledIf     = led;
static StorageInterface&   storageIf = storage;
static RadioInterface&     radioIf   = radio;
// Driver registries — enumerate every physical peripheral available to AMS scripts.
// Scripts select which driver to use via: include MODEL as ALIAS
static BarometerInterface* const  kBaroIfaces[]   = { &baro };
static GpsInterface* const        kGpsIfaces[]    = { &gps  };
static const ares::ams::GpsEntry  kGpsDrivers[]   = { { "BN220",  kGpsIfaces[0]  } };
static const ares::ams::BaroEntry kBaroDrivers[]  = { { "BMP280", kBaroIfaces[0] } };
static const ares::ams::ComEntry  kComDrivers[]   = { { "DXLR03", &radioIf } };
static ImuInterface* const        kImuIfaces[]    = { &imu, &imu2 };
static const ares::ams::ImuEntry  kImuDrivers[]   = { { "MPU6050", kImuIfaces[0] },
                                                       { "ADXL375", kImuIfaces[1] } };

// REST API server — receives references to interfaces.
static ares::ams::MissionScriptEngine missionEngine(
    storageIf,
    kGpsDrivers,  static_cast<uint8_t>(1),
    kBaroDrivers, static_cast<uint8_t>(1),
    kComDrivers,  static_cast<uint8_t>(1),
    kImuDrivers,  static_cast<uint8_t>(2));
static ApiServer apiServer(wifiAp, *kBaroIfaces[0], *kGpsIfaces[0], *kImuIfaces[0],
                           &storageIf, &missionEngine,
                           &statusLed,
                           &Wire, &imuWire,
                           &gpsSerial, &loraSerial,
                           &radioIf);

// ═══════════════════════════════════════════════════════════
void setup()
{
    // Keep USB serial available for on-demand diagnostics, but do not
    // emit boot banners or periodic traces in normal operation.
    Serial.begin(ares::SERIAL_BAUD);

    // I2C buses — initialise before any I2C driver.
    // I2C0 (Wire): shared board sensors (BMP280 on GPIO 1/2).
    Wire.begin(ares::PIN_I2C_SDA, ares::PIN_I2C_SCL, ares::I2C_FREQ);
    Wire.setTimeOut(ares::I2C_TIMEOUT_MS);
    // I2C1: dedicated IMU bus (MPU6050 on GPIO 12/13).
    // Uses 100 kHz standard mode — GY-521 pull-ups (10 kΩ) are not reliable at 400 kHz.
    imuWire.begin(ares::PIN_IMU_SDA, ares::PIN_IMU_SCL, ares::I2C_FREQ_IMU);
    imuWire.setTimeOut(ares::I2C_TIMEOUT_MS);

    for (BarometerInterface* iface : kBaroIfaces) { (void)iface->begin(); }
    for (ImuInterface*        iface : kImuIfaces)  { (void)iface->begin(); }
    for (GpsInterface*        iface : kGpsIfaces)  { (void)iface->begin(); }

    // Status LED — NeoPixel on GPIO 21
    (void)ledIf.begin();
    ledIf.setBrightness(ares::DEFAULT_LED_BRIGHTNESS);
    statusLed.begin();  // starts RTOS task — solid green (IDLE)

    // On-board flash storage (LittleFS)
    (void)storageIf.begin();

    // LoRa radio transceiver (UART2)
    (void)radioIf.begin();

    // WiFi AP — must be up before API server
    (void)wifiAp.begin();

    // AMS runtime (IDLE by default, waits for API activation)
    (void)missionEngine.begin();

    // REST API server — start after AMS is ready to avoid init race
    (void)apiServer.begin();

    // If AMS restored an in-flight checkpoint after reboot, reflect it in API mode.
    ares::ams::EngineSnapshot bootSnap = {};
    missionEngine.getSnapshot(bootSnap);
    if (bootSnap.status == ares::ams::EngineStatus::RUNNING)
    {
        apiServer.notifyMissionResumed();
    }
    else
    {
        // All subsystems ready — exit boot blink, go solid green (IDLE)
        statusLed.setMode(ares::OperatingMode::IDLE);
    }
}

// ═══════════════════════════════════════════════════════════
void loop()
{
    const uint32_t now = millis();

    // GPS bytes must be consumed every iteration to keep
    // the UART FIFO from overflowing (72-byte HW FIFO).
    for (GpsInterface* iface : kGpsIfaces) { iface->update(); }

    // AMS script runtime tick (state machine + PUS emission).
    missionEngine.tick(now);

    // Auto-return to IDLE when the AMS mission finishes or faults.
    // Only act once per transition: check that we are currently in FLIGHT.
    if (apiServer.getMode() == ares::OperatingMode::FLIGHT)
    {
        ares::ams::EngineSnapshot snap = {};
        missionEngine.getSnapshot(snap);
        if (snap.status == ares::ams::EngineStatus::COMPLETE
            || snap.status == ares::ams::EngineStatus::ERROR)
        {
            apiServer.notifyMissionComplete();
        }
    }

    // Yield via RTOS delay (avoid Arduino delay() in RTOS task context).
    vTaskDelay(pdMS_TO_TICKS(ares::SENSOR_RATE_MS));
}

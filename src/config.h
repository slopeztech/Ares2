/**
 * @file  config.h
 * @brief ARES global configuration (ESP32-S3 Zero Mini).
 *
 * All hardware-dependent and tunable parameters live here so that
 * porting to a different board only requires editing this file.
 * Constants use `constexpr` to be evaluated at compile time and
 * occupy no RAM (MISRA-7.2).
 *
 * The file is split in two sections:
 *   1. **Board-specific** — pin assignments that change per PCB.
 *   2. **Shared**         — protocol, RTOS, and flight parameters
 *      that stay constant across board revisions.
 */
#pragma once

#include <cstdint>
#include <freertos/FreeRTOS.h>

// ── Version ─────────────────────────────────────────────────
/// Semantic version string embedded in the firmware binary.
/// Printed at boot and included in telemetry headers.
#define ARES_VERSION_STRING "0.1.0"

namespace ares
{

// ═══════════════════════════════════════════════════════════
// ESP32-S3 Zero Mini — pin configuration
// ═══════════════════════════════════════════════════════════

constexpr const char* BOARD_NAME     = "ESP32-S3 Zero Mini"; ///< Board identifier.
constexpr uint8_t     PIN_LED_RGB    = 21;   ///< On-board RGB LED GPIO.
constexpr uint8_t     PIN_I2C_SDA    = 1;    ///< I2C0 data GPIO (barometer bus).
constexpr uint8_t     PIN_I2C_SCL    = 2;    ///< I2C0 clock GPIO (barometer bus).
constexpr uint8_t     PIN_IMU_SDA    = 12;   ///< I2C1 data GPIO (dedicated IMU bus).
constexpr uint8_t     PIN_IMU_SCL    = 13;   ///< I2C1 clock GPIO (dedicated IMU bus).
constexpr uint8_t     PIN_GPS_RX     = 5;    ///< UART1 RX — GPS TX.
constexpr uint8_t     PIN_GPS_TX     = 6;    ///< UART1 TX — GPS RX.
constexpr uint8_t     GPS_UART_PORT  = 1;    ///< HardwareSerial port index.
constexpr uint8_t     PIN_LORA_TX    = 7;    ///< UART2 TX → module RX.
constexpr uint8_t     PIN_LORA_RX    = 8;    ///< UART2 RX ← module TX.
constexpr uint8_t     PIN_LORA_AUX   = 9;    ///< AUX pin (HIGH = idle).
// M0/M1 not wired — module internal pull-downs default to NORMAL.
// Config is written via UART commands. Pins reserved for future PCB:
constexpr uint8_t     PIN_LORA_M0    = 14;   ///< Mode select bit 0 (not wired).
constexpr uint8_t     PIN_LORA_M1    = 3;    ///< Mode select bit 1 (not wired).
constexpr uint8_t     LORA_UART_PORT = 2;    ///< HardwareSerial port index.
constexpr uint8_t     PIN_DROGUE     = 4;    ///< Drogue pyro channel GPIO.
constexpr uint8_t     PIN_MAIN       = 15;   ///< Main chute pyro channel GPIO.

// ═══════════════════════════════════════════════════════════
// Common configuration (shared across all boards)
// ═══════════════════════════════════════════════════════════

// ── Board ───────────────────────────────────────────────────
constexpr uint32_t    FLASH_SIZE_KB  = 4096;    ///< Total flash in KiB.
constexpr uint32_t    SERIAL_BAUD    = 115200;   ///< USB-CDC baud rate.

// ── I2C ─────────────────────────────────────────────────────
constexpr uint32_t I2C_FREQ     = 400000;  ///< I2C0 bus speed in Hz (BMP280, 400 kHz fast mode).
constexpr uint32_t I2C_FREQ_IMU =  50000;  ///< I2C1 bus speed in Hz (MPU-6050, 50 kHz).
                                            ///< GY-521 modules use 10 kΩ pull-ups; t_rise ~0.85 µs is
                                            ///< marginal at 100 kHz (spec: 1 µs max).  50 kHz doubles
                                            ///< the allowed rise time to 2 µs, eliminating NACK storms.
constexpr uint16_t I2C_TIMEOUT_MS = 5;     ///< Max I2C transaction stall before fail-fast recovery.

// ── Barometer (I2C BMP280) ──────────────────────────────────
constexpr uint8_t BMP280_I2C_ADDR = 0x77;  ///< SDO → VCC (AHT20+BMP280 default).

// ── IMU (I2C MPU-6050) ──────────────────────────────────────
constexpr uint8_t MPU6050_I2C_ADDR = 0x68; ///< AD0 → GND (default address).

// ── IMU (I2C ADXL375) ───────────────────────────────────────
constexpr uint8_t ADXL375_I2C_ADDR  = 0x53U; ///< ALT_ADDRESS/CS → GND (default address).
constexpr uint8_t ADXL375_SETTLE_MS = 5U;    ///< Wait after POWER_CTL → Measure before first read.

// ── IMU (SPI ADXL375) ───────────────────────────────────────
/// Chip-select GPIO for the SPI ADXL375 variant.  Assign per schematic.
constexpr uint8_t  ADXL375_SPI_CS_PIN   = 10U;
/// SPI clock frequency — 5 MHz max per ADXL375 datasheet §Serial Comm.
constexpr uint32_t ADXL375_SPI_FREQ_HZ  = 5000000UL;
/// SPI bus index passed to SPIClass (HSPI=1).  Override as needed.
constexpr uint8_t  ADXL375_SPI_BUS      = 1U;

// ── UART (GPS) ──────────────────────────────────────────────
constexpr uint32_t GPS_BAUD      = 9600;   ///< Factory default baud rate.

// ── Actuators (parachute) ──────────────────────────────────
constexpr uint32_t FIRE_DURATION_MS = 1000;  ///< Pyro channel fire pulse duration.

// ── LoRa (UART-based modules) ───────────────────────────────
constexpr float    LORA_FREQ_MHZ          = 433.125f; ///< CH=23: 410.125+23.
constexpr int8_t   LORA_TX_POWER          = 20;       ///< Transmit power in dBm.
constexpr uint32_t LORA_UART_BAUD         = 9600;     ///< Module serial baud rate.
constexpr uint32_t LORA_AUX_TIMEOUT_MS    = 50;       ///< Max AUX wait before TX proceeds in degraded mode.
constexpr uint32_t TELEMETRY_INTERVAL_MS  = 500;      ///< Downlink interval.

// ── WiFi AP (ground configuration) ─────────────────────────
constexpr const char* WIFI_AP_PASSWORD    = "ares1234"; ///< AP WPA2 password.
constexpr uint16_t    WIFI_API_PORT       = 80;         ///< HTTP REST API port.
constexpr uint8_t     WIFI_AP_CHANNEL     = 1;          ///< WiFi radio channel.
constexpr uint8_t     WIFI_AP_MAX_CLIENTS = 4;          ///< Max simultaneous clients.
constexpr int8_t      WIFI_TX_POWER_DBM   = 8;          ///< ~6 mW — enough for ground ops.
constexpr bool        WIFI_DISABLE_IN_FLIGHT = true;    ///< Kill AP on launch (REST-13.3).
constexpr uint16_t    WIFI_CONN_TIMEOUT_MS   = 5000;    ///< Stale connection timeout (REST-10).

// ── REST API limits (REST-3) ────────────────────────────────
constexpr uint16_t    API_MAX_REQUEST_BODY   = 1024;    ///< Max JSON request body bytes.
constexpr uint16_t    API_MAX_RESPONSE_BODY  = 2048;    ///< Max JSON response buffer bytes.
constexpr uint16_t    API_CFG_MUTEX_TIMEOUT_MS = 50;    ///< Config mutex wait (REST-8.2).

// ── Storage / log download (REST-12) ────────────────────────
constexpr const char* LOG_DIR             = "/logs";    ///< LittleFS log directory.
constexpr uint8_t     MAX_LOG_FILES       = 16;         ///< Max files returned by listing.
constexpr uint16_t    DOWNLOAD_CHUNK_SIZE = 512;        ///< Chunk size for file download (REST-12.2).
constexpr uint8_t     LOG_FILENAME_MAX    = 32;         ///< Max log filename length (no path).

// ── AMS (Ares Mission Script) ───────────────────────────────
constexpr const char* MISSION_DIR          = "/missions"; ///< LittleFS directory for .ams files.
constexpr uint8_t     MISSION_FILENAME_MAX = 32;           ///< Max .ams filename length.
constexpr uint16_t    AMS_MAX_SCRIPT_BYTES = 4096;         ///< Max bytes read from one script.
constexpr uint8_t     AMS_MAX_STATES       = 10;           ///< Max states per mission script.
constexpr uint8_t     AMS_MAX_STATE_NAME   = 16;           ///< Max state name length.
constexpr uint8_t     AMS_MAX_EVENT_TEXT   = 64;           ///< Max EVENT message text.
constexpr uint8_t     AMS_MAX_HK_FIELDS    = 16;           ///< Max HK fields inside report block.
constexpr uint8_t     AMS_MAX_HK_SLOTS     = 4U;           ///< Max every/log_every blocks per state (AMS-4.3.1).
constexpr uint8_t     AMS_MAX_CONDITIONS        = 4;  ///< Max guard conditions per state.
constexpr uint8_t     AMS_MAX_TRANSITION_CONDS = 4;  ///< Max sub-conditions in one transition (AMS-4.6.2).
constexpr uint8_t     AMS_MAX_VARS             = 8U; ///< Max global variables per script (AMS-4.8).
constexpr uint8_t     AMS_MAX_CONSTS           = 8U;  ///< Max named constants per script (AMS-4.12).
constexpr uint8_t     AMS_MAX_TASKS              = 4U;  ///< Max background task blocks per script (AMS-11).
constexpr uint8_t     AMS_MAX_TASK_RULES         = 4U;  ///< Max if-rules per task block (AMS-11).
constexpr uint8_t     AMS_MAX_TASK_ACTIVE_STATES = 6U;  ///< Max states in a task 'when in' filter (AMS-11).
constexpr uint8_t     AMS_MAX_ASSERTS            = 8U;  ///< Max assert directives per script (AMS-15).
constexpr uint8_t     AMS_VAR_NAME_LEN         = 16U;///< Max variable name length including NUL (AMS-4.8).
constexpr uint8_t     AMS_MAX_SET_ACTIONS      = 4U; ///< Max set actions per on_enter block (AMS-4.8).
constexpr uint8_t     AMS_MAX_SENSOR_RETRY     = 5U; ///< Max sensor read retries declared in include (AMS-4.9.1).
constexpr uint8_t     AMS_MAX_ERROR_TEXT   = 96;           ///< Max internal parser/runtime error text.
constexpr uint16_t    AMS_MAX_LINE_LEN     = 128;          ///< Parser line buffer size.
constexpr uint8_t     AMS_MAX_INCLUDES     = 8U;           ///< Max 'include' alias registrations per script.
constexpr uint16_t    AMS_DEFAULT_APID     = 0x01U;        ///< Default APID for mission runtime (APUS-10: rocket = 0x01).
constexpr uint16_t    AMS_MUTEX_TIMEOUT_MS = 250;          ///< Mutex timeout for API/runtime sync (must exceed LittleFS write latency ~100 ms).
constexpr const char* AMS_RESUME_PATH      = "/missions/.ams_resume.chk"; ///< Persistent AMS checkpoint file.
constexpr uint32_t    AMS_CHECKPOINT_INTERVAL_MS = 1000;   ///< Periodic checkpoint cadence while RUNNING.

// ── Config field validation bounds (REST-5.4) ───────────────
constexpr uint32_t    TELEMETRY_INTERVAL_MIN = 100;     ///< Min telemetry interval ms.
constexpr uint32_t    TELEMETRY_INTERVAL_MAX = 60000;   ///< Max telemetry interval ms.
constexpr uint8_t     NODE_ID_MIN            = 1;       ///< Min telemetry node ID.
constexpr uint8_t     NODE_ID_MAX            = 253;     ///< Max telemetry node ID.

// ── Monitoring parameter bounds (APUS-16.2, AMS-4.15) ────────────────────
// Used by both configParams_[] (radio_dispatcher.h) and kSpecs[] (parser).
// MISRA-7: named constants replace magic float literals in both sites.
constexpr float MONITOR_ALT_HIGH_MAX_M   = 15000.0f;  ///< Max altitude high-alarm threshold (m).
constexpr float MONITOR_ALT_LOW_MIN_M    =  -500.0f;  ///< Min altitude low-alarm threshold (m).
constexpr float MONITOR_ALT_LOW_MAX_M    =  1000.0f;  ///< Max altitude low-alarm threshold (m).
constexpr float MONITOR_ACCEL_HIGH_MAX   =  1000.0f;  ///< Max accel high-alarm threshold (m/s²).
constexpr float MONITOR_TEMP_HIGH_MIN_C  =   -40.0f;  ///< Min temperature high-alarm threshold (°C).
constexpr float MONITOR_TEMP_HIGH_MAX_C  =   150.0f;  ///< Max temperature high-alarm threshold (°C).
constexpr float MONITOR_TEMP_LOW_MIN_C   =  -100.0f;  ///< Min temperature low-alarm threshold (°C).
constexpr float MONITOR_TEMP_LOW_MAX_C   =    50.0f;  ///< Max temperature low-alarm threshold (°C).

// ── RTOS Tasks ──────────────────────────────────────────────
// Stack sizes are in bytes.  API task needs extra headroom for
// ArduinoJson serialisation buffers and LittleFS I/O.
constexpr uint32_t TASK_STACK_SIZE       = 4096;  ///< Default task stack in bytes.
constexpr uint32_t TASK_STACK_SIZE_API   = 12288; ///< API task stack in bytes (JSON/FS + safety headroom).
constexpr uint32_t TASK_STACK_SIZE_LED   = 2048;  ///< LED task stack in bytes (minimal I/O).

// Task priorities (RTOS-5: documented, explicit)
// Higher number = higher priority on FreeRTOS.
constexpr UBaseType_t TASK_PRIORITY_COMMS = 2;  ///< Medium — telemetry TX.
constexpr UBaseType_t TASK_PRIORITY_API   = 1;  ///< Low    — REST API polling.
constexpr UBaseType_t TASK_PRIORITY_LED   = 1;  ///< Low    — status heartbeat.
constexpr uint32_t SENSOR_RATE_MS    = 10;   ///< 100 Hz.
constexpr uint32_t FLIGHT_RATE_MS    = 20;   ///<  50 Hz.
constexpr uint32_t LOG_RATE_MS       = 100;  ///<  10 Hz.
constexpr uint32_t TELEMETRY_RATE_MS = 2000; ///< 0.5 Hz — must exceed air TX time (~1.85 s @ 2.4 kbps).
constexpr uint32_t API_RATE_MS       = 50;   ///<  20 Hz.
constexpr uint32_t LED_RATE_MS       = 1000; ///<   1 Hz.

// ── Flight thresholds ──────────────────────────────────────
// These define the state-machine transitions for the flight
// controller.  Values are conservative defaults; fine-tune
// per motor and airframe before flight.

// ── Operating modes ─────────────────────────────────────────
/**
 * System operating modes.
 * Determines LED patterns, telemetry rate, and safety interlocks.
 */
enum class OperatingMode : uint8_t
{
    IDLE     = 0,   ///< Ground idle — awaiting commands.
    TEST     = 1,   ///< Sensor / system checkout.
    FLIGHT   = 2,   ///< Active flight (launch to landing).
    RECOVERY = 3,   ///< Post-landing recovery beacon.
    ERROR    = 4,   ///< System error — safe mode.
    BOOT     = 5,   ///< Initialising — green blink until ready.

    FIRST = IDLE,   // CERT-6.1 — range validation sentinels
    LAST  = BOOT
};

// ── Flight thresholds (values) ─────────────────────────────
constexpr float LAUNCH_ACCEL_THRESHOLD  = 30.0f;  ///< m/s² — detect motor ignition.
constexpr float BURNOUT_ACCEL_THRESHOLD = 0.5f;   ///< m/s² — detect motor burnout.
constexpr float APOGEE_VEL_THRESHOLD    = 2.0f;   ///< m/s  — vertical speed ≈ 0 → apogee.
constexpr float MAIN_DEPLOY_ALT_M       = 300.0f; ///< m    — main chute deployment altitude.
constexpr float LANDED_VEL_THRESHOLD    = 1.0f;   ///< m/s  — near-zero → on ground.
constexpr uint32_t LANDED_STABLE_MS     = 5000;   ///< ms   — velocity must stay low this long.

// ── Init delays (MISRA-7) ──────────────────────────────────
// Named constants for every delay() call in the codebase.
constexpr uint8_t  BMP280_RESET_DELAY_MS   = 10;    ///< Wait for BMP280 soft-reset completion.
constexpr uint8_t  BMP280_POLL_DELAY_MS    = 2;     ///< Status register poll interval during NVM copy.
constexpr uint8_t  MPU6050_WAKE_DELAY_MS   = 10;    ///< Wait after waking MPU-6050 from sleep.
constexpr uint32_t IMU_REINIT_INTERVAL_MS  = 5000;  ///< Min interval between lazy re-init attempts in read().
constexpr uint8_t  IMU_MAX_CONSECUTIVE_ERRORS = 3U; ///< Read failures before marking device not-ready and silencing Wire log spam.
constexpr uint32_t IMU_LOCK_TIMEOUT_MS        = 300U; ///< Max wait for IMU driver mutex (CERT-13). Must exceed begin() reinit time (~110 ms).
constexpr uint8_t  SERIAL_WAIT_DELAY_MS    = 10;    ///< USB-CDC host poll interval in setup().
constexpr uint16_t SERIAL_WAIT_TIMEOUT_MS  = 3000;  ///< Max wait for USB-CDC host before continuing.
constexpr uint16_t TEST_LOOP_DELAY_MS      = 1000;  ///< Test sketch main-loop print interval.
constexpr uint16_t STORAGE_MUTEX_TIMEOUT_MS = 1000;  ///< Max wait for storage lock (RTOS-8).

// ── Loop bounds (PO10-2) ───────────────────────────────────
// Hard upper limits for every unbounded loop in the codebase.
// Prevents runaway iteration if a peripheral misbehaves.
constexpr uint16_t MAX_SERIAL_READ   = 512;   ///< Max bytes consumed per update() call.
constexpr uint8_t  MAX_SENSORS       = 8;     ///< Max registered sensor drivers.
constexpr uint16_t MAX_LOG_ENTRIES   = 8192;  ///< Flash log ring-buffer capacity.
constexpr uint8_t  MAX_PACKET_SIZE   = 64;    ///< Max radio payload bytes.
constexpr uint8_t  MAX_I2C_TRANSFER  = 32;    ///< Max bytes in a single I2C burst.
constexpr uint8_t  STORAGE_MAX_PATH  = 64;    ///< Max file path length (including null).
constexpr uint32_t STORAGE_MAX_FILE  = 65536; ///< Max single file size (64 KiB).

// ── Defaults (MISRA-7: no magic numbers) ───────────────────
constexpr uint8_t  DEFAULT_NODE_ID        = 0x01;  ///< Telemetry source node.
constexpr uint8_t  DEFAULT_LED_BRIGHTNESS = 80;    ///< 0–255 initial brightness.

} // namespace ares

/**
 * @file  mission_script_engine.h
 * @brief ARES Mission Script (AMS) runtime and parser.
 *
 * AMS executes mission states from plain-text `.ams` files stored
 * in LittleFS and emits PUS-compatible frames through the radio link:
 *   - HK.report   -> TM Service 3 (MsgType::TELEMETRY)
 *   - EVENT.*     -> TM Service 5 (MsgType::EVENT)
 *   - TC.command  -> TC Service 1 trigger (injected via API)
 *
 * Design constraints:
 *   - No dynamic allocation (PO10-3)
 *   - Bounded parser/runtime loops (PO10-2)
 *   - Thread-safe control methods for API task access (RTOS-4)
 */
#pragma once

#include <cstdint>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#include "ams/ams_driver_registry.h"
#include "comms/ares_radio_protocol.h"
#include "config.h"
#include "hal/baro/barometer_interface.h"
#include "hal/gps/gps_interface.h"
#include "hal/imu/imu_interface.h"
#include "hal/radio/radio_interface.h"
#include "hal/storage/storage_interface.h"

namespace ares
{
namespace ams
{

/**
 * AMS engine execution states.
 */
enum class EngineStatus : uint8_t
{
    IDLE     = 0,   ///< No script loaded; engine is inactive.
    RUNNING  = 1,   ///< Script is executing and advancing states.
    ERROR    = 2,   ///< Parser or runtime error; check lastError.
    COMPLETE = 3,   ///< Terminal state reached; mission finished normally.

    FIRST = IDLE,     // CERT-6.1 — range validation sentinels
    LAST  = COMPLETE,
};

/**
 * Telecommand tokens that can be injected via injectTcCommand().
 * Consumed by state-machine transition conditions (TC_COMMAND_EQ).
 */
enum class TcCommand : uint8_t
{
    NONE   = 0,   ///< No pending command.
    LAUNCH = 1,   ///< Launch command — triggers LAUNCH transitions.
    ABORT  = 2,   ///< Abort command — triggers ABORT transitions.
    RESET  = 3,   ///< Reset command — triggers RESET transitions.

    FIRST = NONE,  // CERT-6.1 — range validation sentinels
    LAST  = RESET,
};

/**
 * Immutable snapshot of the engine's public state.
 * Obtained via MissionScriptEngine::getSnapshot() under mutex.
 */
struct EngineSnapshot
{
    EngineStatus status = EngineStatus::IDLE;            ///< Current engine status.
    char activeFile[ares::MISSION_FILENAME_MAX] = {};    ///< Loaded script filename (empty if IDLE).
    char stateName[ares::AMS_MAX_STATE_NAME] = {};       ///< Active state name (empty if IDLE).
    char lastError[ares::AMS_MAX_ERROR_TEXT] = {};       ///< Last parser/runtime error message.
};

/**
 * ARES Mission Script (AMS) runtime.
 *
 * Loads, parses, and executes `.ams` mission scripts stored
 * in LittleFS.  Emits PUS-compatible APUS frames over the radio
 * link on HK and EVENT actions.
 *
 * Design constraints:
 *   - No dynamic allocation (PO10-3).
 *   - Bounded loops (PO10-2).
 *   - All state is mutex-protected for API-task access (RTOS-4).
 *
 * @pre  begin() must be called before any other method.
 */
class MissionScriptEngine
{
public:
    /**
     * Construct the mission script engine.
     *
     * @param[in] storage     Storage interface for reading .ams files.
     * @param[in] gpsDrivers  Array of compiled-in GPS drivers.
     * @param[in] gpsCount    Number of entries in @p gpsDrivers.
     * @param[in] baroDrivers Array of compiled-in barometer drivers.
     * @param[in] baroCount   Number of entries in @p baroDrivers.
     * @param[in] comDrivers  Array of compiled-in COM/radio drivers.
     * @param[in] comCount    Number of entries in @p comDrivers.
     * @param[in] imuDrivers  Array of compiled-in IMU drivers.
     * @param[in] imuCount    Number of entries in @p imuDrivers.
     */
    MissionScriptEngine(StorageInterface&  storage,
                        const GpsEntry*    gpsDrivers,  uint8_t gpsCount,
                        const BaroEntry*   baroDrivers, uint8_t baroCount,
                        const ComEntry*    comDrivers,  uint8_t comCount,
                        const ImuEntry*    imuDrivers,  uint8_t imuCount);

    MissionScriptEngine(const MissionScriptEngine&)            = delete;
    MissionScriptEngine& operator=(const MissionScriptEngine&) = delete;
    MissionScriptEngine(MissionScriptEngine&&)                 = delete;
    MissionScriptEngine& operator=(MissionScriptEngine&&)      = delete;

    /**
     * Initialise the engine and create the internal mutex.
     * @return true on success, false on mutex creation failure.
     * @pre  Storage interface is mounted and ready.
     */
    bool begin();

    /**
     * Load and start a mission script from LittleFS.
     * @param[in] fileName  Script filename inside /missions (e.g. "flight.ams").
     * @return true if the script was parsed and execution started.
     * @pre  begin() returned true.
     * @post Engine status is RUNNING on success, ERROR on parse failure.
     */
    bool activate(const char* fileName);

    /**
     * Stop execution and return the engine to IDLE.
     * @post Engine status is IDLE; activeFile and stateName are cleared.
     */
    void deactivate();

    /**
     * Inject a telecommand token for consumption by transition conditions.
     * @param[in] commandText  Text token (e.g. "LAUNCH", "ABORT", "RESET").
     * @return true if the token was recognised and stored.
     * @note Only one TC token is buffered at a time; a second call
     *       before the engine consumes the first overwrites it.
     */
    bool injectTcCommand(const char* commandText);

    /**
     * Execute one engine tick: evaluate transitions and run due actions.
     * Must be called from the mission task at the configured rate.
     * @param[in] nowMs  Current millis() timestamp.
     * @pre  begin() returned true.
     */
    void tick(uint32_t nowMs);

    /**
     * Enable or disable runtime progression (state transitions and periodic actions).
     * @param[in] enabled  true to enable; false to pause without deactivating.
     */
    void setExecutionEnabled(bool enabled);

    /**
     * Thread-safe snapshot of the engine's current state.
     * @param[out] out  Populated with a copy of the engine's public state.
     */
    void getSnapshot(EngineSnapshot& out) const;

    /**
     * List available .ams script files in the /missions directory.
     * @param[out] entries     Caller-provided array of FileEntry.
     * @param[in]  maxEntries  Capacity of @p entries.
     * @param[out] count       Number of entries written (≤ maxEntries).
     * @return true on success (zero files is not an error).
     */
    bool listScripts(FileEntry* entries, uint8_t maxEntries,
                     uint8_t& count) const;

private:
    enum class BlockType : uint8_t
    {
        NONE       = 0,
        HK         = 1,
        LOG        = 2,
        CONDITIONS = 3,  ///< Inside a conditions: block.
        ON_ERROR   = 4,  ///< Inside an on_error: block.
    };

    // ── Peripheral kind ──────────────────────────────────────────────────────
    enum class PeripheralKind : uint8_t
    {
        GPS  = 0,
        BARO = 1,
        COM  = 2,
        IMU  = 3,
    };

    // ── Sensor field within a peripheral ─────────────────────────────────────
    enum class SensorField : uint8_t
    {
        LAT       = 0,  ///< GPS: latitude  (deg)
        LON       = 1,  ///< GPS: longitude (deg)
        ALT       = 2,  ///< GPS/BARO: altitude (m)
        SPEED     = 3,  ///< GPS: speed (km/h)
        TEMP      = 4,  ///< BARO/IMU: temperature (°C)
        PRESSURE  = 5,  ///< BARO: pressure (Pa)
        ACCEL_X   = 6,  ///< IMU: accel X (m/s²)
        ACCEL_Y   = 7,  ///< IMU: accel Y (m/s²)
        ACCEL_Z   = 8,  ///< IMU: accel Z (m/s²)
        ACCEL_MAG = 9,  ///< IMU: ||accel|| (m/s²)
        GYRO_X    = 10, ///< IMU: gyro X (deg/s)
        GYRO_Y    = 11, ///< IMU: gyro Y (deg/s)
        GYRO_Z    = 12, ///< IMU: gyro Z (deg/s)
        IMU_TEMP  = 13, ///< IMU: die temperature (°C)
    };

    // ── Unified condition kind ─────────────────────────────────────────────
    enum class CondKind : uint8_t
    {
        NONE      = 0,
        SENSOR_LT = 1,  ///< alias.field < threshold
        SENSOR_GT = 2,  ///< alias.field > threshold
        TC_EQ     = 3,  ///< TC.command == value
        TIME_GT   = 4,  ///< TIME.elapsed > threshold (ms since state entry)
    };

    // ── Unified condition expression ───────────────────────────────────────
    struct CondExpr
    {
        CondKind    kind      = CondKind::NONE;
        char        alias[16] = {};              ///< Peripheral alias (e.g. "GPS", "GPS1").
        SensorField field     = SensorField::ALT;
        float       threshold = 0.0f;
        TcCommand   tcValue   = TcCommand::NONE;
    };

    enum class EventVerb : uint8_t
    {
        INFO  = 0,
        WARN  = 1,
        ERROR = 2,
    };

    /**
     * One HK/LOG field: maps a user label to a peripheral alias + sensor field.
     * The label is used as the CSV column name in log reports.
     */
    struct HkField
    {
        char        label[20] = {};              ///< User-provided key (CSV column name).
        char        alias[16] = {};              ///< Peripheral alias (e.g. "GPS", "GPS1").
        SensorField field     = SensorField::ALT;
    };

    /**
     * A single guard condition (evaluated every tick).
     * TC.command is intentionally excluded — use transitions for TC gates.
     */
    struct Condition
    {
        CondExpr expr = {};
    };

    struct Transition
    {
        CondExpr cond         = {};
        char     targetName[16] = {};
        uint8_t  targetIndex  = 0;
        bool     targetResolved = false;
    };

    /**
     * One entry in the per-script alias table.
     * Created by 'include <MODEL> as <ALIAS>'.
     */
    struct AliasEntry
    {
        char           alias[16]   = {};    ///< e.g. "GPS", "GPS1", "BARO"
        char           model[16]   = {};    ///< e.g. "BN220", "BMP280"
        PeripheralKind kind        = PeripheralKind::GPS;
        uint8_t        driverIdx   = 0xFFU; ///< Index in registry; 0xFF = not resolved
    };

    struct StateDef
    {
        char name[ares::AMS_MAX_STATE_NAME] = {};

        bool hasOnEnterEvent = false;
        EventVerb onEnterVerb = EventVerb::INFO;
        char onEnterText[ares::AMS_MAX_EVENT_TEXT] = {};

        bool hasHkEvery = false;
        uint32_t hkEveryMs = 0;
        uint8_t hkFieldCount = 0;
        HkField hkFields[ares::AMS_MAX_HK_FIELDS] = {};

        bool hasLogEvery = false;
        uint32_t logEveryMs = 0;
        uint8_t logFieldCount = 0;
        HkField logFields[ares::AMS_MAX_HK_FIELDS] = {};

        uint8_t hkPriority    = 2;  ///< Higher value = higher priority.
        uint8_t logPriority   = 1;  ///< Keep local log below PUS HK by default.
        uint8_t eventPriority = 4;  ///< PUS event reporting has highest default priority.
        uint8_t actionBudget  = 2;  ///< Max actions per tick (EVENT/HK/LOG arbitration).

        bool hasTransition = false;
        Transition transition = {};

        // ── Guard conditions (AMS-4.7) ─────────────────────
        uint8_t   conditionCount = 0;
        Condition conditions[ares::AMS_MAX_CONDITIONS] = {};

        // ── on_error handler ────────────────────────────────
        bool      hasOnErrorEvent = false;
        EventVerb onErrorVerb     = EventVerb::ERROR;
        char      onErrorText[ares::AMS_MAX_EVENT_TEXT] = {};
    };

    struct Program
    {
        uint16_t   apid       = 0x01U;  ///< Active APID (APUS-10: rocket = 0x01).
        uint8_t    nodeId     = ares::DEFAULT_NODE_ID;
        uint8_t    stateCount = 0;
        StateDef   states[ares::AMS_MAX_STATES] = {};
        uint8_t    aliasCount = 0;
        AliasEntry aliases[ares::AMS_MAX_INCLUDES] = {};
    };

    bool loadFromStorageLocked(const char* fileName);
    bool parseScriptLocked(const char* script, uint32_t length);
    bool readNextScriptLineLocked(const char* script,
                                  uint32_t length,
                                  uint32_t& offset,
                                  char* line,
                                  uint32_t lineSize);
    bool parseLineLocked(const char* line,
                         uint8_t& currentState,
                         BlockType& blockType);
    bool parseIncludeLineLocked(const char* line);
    bool parseStateScopedLineLocked(const char* line,
                                    StateDef& st,
                                    BlockType& blockType);
    bool parseStateLineLocked(const char* line, uint8_t& currentState);
    bool parseEventLineLocked(const char* line, StateDef& st);
    bool parseEveryLineLocked(const char* line, StateDef& st);
    bool parseLogEveryLineLocked(const char* line, StateDef& st);
    static bool parsePrioritiesValuesLocked(const char* line,
                                            uint32_t& event,
                                            uint32_t& hk,
                                            uint32_t& log,
                                            uint32_t& budget,
                                            const StateDef& st);
    bool parsePrioritiesLineLocked(const char* line, StateDef& st);
    bool parseFieldLineLocked(const char* line,
                              HkField* fields,
                              uint8_t& count,
                              const char* ctxName);
    bool parseTransitionLineLocked(const char* line, StateDef& st);
    bool parseConditionScopedLineLocked(const char* line, StateDef& st);
    bool parseOnErrorEventLineLocked(const char* line, StateDef& st);
    static bool mapApidToNode(uint16_t apid, uint8_t& nodeId);
    bool resolveTransitionsLocked();

    static bool isSafeFileName(const char* fileName);
    static bool buildMissionPath(const char* fileName,
                                 char* outPath,
                                 uint32_t outSize);

    static void trimInPlace(char* text);
    static bool startsWith(const char* text, const char* prefix);
    static bool parseUint(const char* text, uint32_t& outValue);
    static bool parseFloatValue(const char* text, float& outValue);
    static bool parseTcCommand(const char* text, TcCommand& out);
    static bool parseSensorField(PeripheralKind kind,
                                 const char*    fieldStr,
                                 SensorField&   out);
    static bool splitAliasDotField(const char* expr,
                                   char*       aliasOut, uint8_t aliasSize,
                                   char*       fieldOut,  uint8_t fieldSize);

    const AliasEntry* findAliasLocked(const char* alias) const;
    bool readSensorFloatLocked(const char*  alias,
                               SensorField  field,
                               float&       outVal) const;
    bool parseCondExprLocked(const char* lhs,
                             const char* op,
                             const char* rhs,
                             bool        allowTc,
                             CondExpr&   out);

    uint8_t findStateByNameLocked(const char* name) const;

    void setErrorLocked(const char* reason);
    void enterStateLocked(uint8_t stateIndex, uint32_t nowMs);
    bool evaluateTransitionAndMaybeEnterLocked(StateDef& state, uint32_t nowMs);
    void executeDueActionsLocked(const StateDef& state, uint32_t nowMs);

    void sendOnEnterEventLocked(uint32_t nowMs);
    bool evaluateConditionsLocked(const StateDef& state, uint32_t nowMs);
    void sendHkReportLocked(uint32_t nowMs);
    void appendLogReportLocked(uint32_t nowMs);
    void sendEventLocked(EventVerb verb, const char* text, uint32_t nowMs);
    bool ensureLogFileLocked(const char* fileName);
    void applyHkFieldToPayloadLocked(const HkField&               f,
                                     ares::proto::TelemetryPayload& tm) const;
    bool formatHkFieldValueLocked(const HkField& f,
                                  char*          out,
                                  uint32_t       outSize) const;

    bool sendFrameLocked(const ares::proto::Frame& frame);

    StorageInterface&    storage_;
    const GpsEntry*      gpsDrivers_;
    uint8_t              gpsCount_;
    const BaroEntry*     baroDrivers_;
    uint8_t              baroCount_;
    const ComEntry*      comDrivers_;
    uint8_t              comCount_;
    const ImuEntry*      imuDrivers_;
    uint8_t              imuCount_;
    RadioInterface*      primaryCom_ = nullptr; ///< Active COM driver for frame TX.

    StaticSemaphore_t mutexBuf_ = {};
    SemaphoreHandle_t mutex_ = nullptr;

    Program program_ = {};

    EngineStatus status_ = EngineStatus::IDLE;
    char activeFile_[ares::MISSION_FILENAME_MAX] = {};
    char lastError_[ares::AMS_MAX_ERROR_TEXT] = {};

    bool running_ = false;
    bool executionEnabled_ = false;
    uint8_t currentState_ = 0;
    uint32_t stateEnterMs_ = 0;
    uint32_t lastHkMs_ = 0;
    uint32_t lastLogMs_ = 0;
    bool pendingOnEnterEvent_ = false;
    EventVerb pendingEventVerb_ = EventVerb::INFO;
    char pendingEventText_[ares::AMS_MAX_EVENT_TEXT] = {};
    uint32_t pendingEventTsMs_ = 0;

    TcCommand pendingTc_ = TcCommand::NONE;
    uint8_t seq_ = 0;
    char logPath_[ares::STORAGE_MAX_PATH] = {};
    bool logHeaderWritten_ = false;

    char scriptBuffer_[ares::AMS_MAX_SCRIPT_BYTES + 1U] = {};
};

} // namespace ams
} // namespace ares

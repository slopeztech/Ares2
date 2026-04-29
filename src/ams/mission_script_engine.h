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
    LOADED   = 4,   ///< Script parsed and ready; waiting for arm to start execution.

    FIRST = IDLE,     // CERT-6.1 — range validation sentinels
    LAST  = LOADED,
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
     * Atomically enable execution and inject the LAUNCH TC in a single
     * mutex acquisition.  Equivalent to setExecutionEnabled(true) followed
     * by injectTcCommand("LAUNCH") but without a race window between them.
     * @return true if the engine was in LOADED state and LAUNCH was queued.
     * @note  Called by POST /api/arm after validating the mission state.
     */
    bool arm();

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
        ON_ENTER   = 5,  ///< Inside an on_enter: block (EVENT.* and set actions).
        TASK       = 6,  ///< Inside a task: block header (before first if-rule) (AMS-11).
        TASK_IF    = 7,  ///< Inside an if COND: block within a task (AMS-11).
        ASSERT     = 8,  ///< Inside an assert: block (AMS-15).
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
        NONE           = 0,
        SENSOR_LT      = 1,  ///< alias.field < threshold
        SENSOR_GT      = 2,  ///< alias.field > threshold
        TC_EQ          = 3,  ///< TC.command == value
        TIME_GT        = 4,  ///< TIME.elapsed > threshold (ms since state entry)
        SENSOR_DELTA_LT = 5, ///< (alias.field[n] - alias.field[n-1]) < threshold (AMS-4.6.2)
        SENSOR_DELTA_GT = 6, ///< (alias.field[n] - alias.field[n-1]) > threshold (AMS-4.6.2)
    };

    // ── TC debounce mode (AMS-4.11) ────────────────────────────────────────
    enum class TcDebounceMode : uint8_t
    {
        ONESHOT  = 0U, ///< Fire on first TC match — one-shot, existing behavior (AMS-4.11.1).
        ONCE     = 1U, ///< Explicit alias for ONESHOT — self-documenting keyword.
        CONFIRM  = 2U, ///< Require N successive inject calls before firing (AMS-4.11.2).
    };

    // ── Logical operator for compound transitions (AMS-4.6.2) ─────────────
    enum class TransitionLogic : uint8_t
    {
        AND = 0,  ///< All sub-conditions must be true.
        OR  = 1,  ///< Any sub-condition being true is sufficient.
    };

    // ── Unified condition expression ───────────────────────────────────────
    struct CondExpr
    {
        CondKind    kind      = CondKind::NONE;
        char        alias[16] = {};              ///< Peripheral alias (e.g. "GPS", "GPS1").
        SensorField field     = SensorField::ALT;
        float       threshold = 0.0f;            ///< Literal RHS value (used when !useVar).
        TcCommand   tcValue   = TcCommand::NONE;
        TcDebounceMode tcDebounce = TcDebounceMode::ONESHOT; ///< Debounce mode for TC_EQ (AMS-4.11).
        uint8_t        tcConfirmN = 1U;                      ///< Injections required in CONFIRM mode.

        // AMS-4.8: variable reference in RHS.
        bool  useVar  = false;                   ///< true = compare against (varValue + varOffset).
        char  varName[ares::AMS_VAR_NAME_LEN] = {}; ///< Variable name (used when useVar).
        float varOffset = 0.0f;                  ///< Additive offset: compare against var+offset.
    };

    /**
     * Global scalar variable declared in the script metadata section (AMS-4.8).
     * Memory is reserved at parse time; no dynamic allocation.
     */
    struct VarEntry
    {
        char  name[ares::AMS_VAR_NAME_LEN] = {};  ///< Variable name.
        float value = 0.0f;                        ///< Current value (init or last set).
        bool  valid = false;                       ///< false until first successful set fires.
    };

    /**
     * Named constant declared in the script metadata section (AMS-4.12).
     * Value is resolved and inlined into CondExpr::threshold at parse time.
     */
    struct ConstEntry
    {
        char  name[ares::AMS_VAR_NAME_LEN] = {};  ///< Constant identifier (up to 15 chars).
        float value = 0.0f;                        ///< Immutable float value set at parse time.
    };

    // ── AMS-15: Formal validation assertions ──────────────────────────────────

    /**
     * Kind of a formal assertion evaluated at load time (AMS-15).
     */
    enum class AssertKind : uint8_t
    {
        REACHABLE      = 0U, ///< assert reachable STATE — BFS reachability from initial state.
        NO_DEAD_STATES = 1U, ///< assert no_dead_states — all states are reachable from initial.
        MAX_DEPTH      = 2U, ///< assert max_transition_depth < N — longest simple path < N.
    };

    /**
     * One formal assertion stored from an @c assert: block (AMS-15).
     */
    struct AssertDef
    {
        AssertKind kind       = AssertKind::REACHABLE;
        char       targetName[ares::AMS_MAX_STATE_NAME] = {}; ///< REACHABLE: target state name.
        uint8_t    numericArg = 0U;                            ///< MAX_DEPTH: depth limit.
    };

    /**
     * Computation kind for a @c set action (AMS-4.8).
     */
    enum class SetActionKind : uint8_t
    {
        SIMPLE    = 0U, ///< var = ALIAS.field                       (AMS-4.8.1)
        CALIBRATE = 1U, ///< var = CALIBRATE(ALIAS.field, N)         (AMS-4.8.2)
        DELTA     = 2U, ///< var = ALIAS.field delta (curr - prev)   (AMS-4.8.3)
        MAX_VAR   = 3U, ///< var = max(var, ALIAS.field)             (AMS-4.8.4)
        MIN_VAR   = 4U, ///< var = min(var, ALIAS.field)             (AMS-4.8.5)
    };

    /**
     * A single @c set action inside an @c on_enter: block (AMS-4.8).
     * Captures a sensor reading (or derived value) into a global variable.
     */
    struct SetAction
    {
        char          varName[ares::AMS_VAR_NAME_LEN] = {};  ///< Target variable name.
        char          alias[16]      = {};                    ///< Sensor peripheral alias.
        SensorField   field          = SensorField::ALT;      ///< Sensor field to read.
        SetActionKind kind           = SetActionKind::SIMPLE; ///< Computation kind.
        uint8_t       calibSamples   = 1U;    ///< Samples to average (CALIBRATE form, 1-10).
        float         deltaBaseline  = 0.0f;  ///< Previous sensor value (DELTA form).
        bool          deltaValid     = false; ///< Whether deltaBaseline has been set.
    };

    enum class EventVerb : uint8_t
    {
        INFO  = 0,
        WARN  = 1,
        ERROR = 2,
    };

    // ── AMS-11: Parallel background tasks ────────────────────────────────────

    /**
     * One conditional rule inside a @c task: block (AMS-11).
     *
     * At runtime, when the task fires (every N ms), each rule's condition
     * is evaluated; if true, the associated event and/or set action fires.
     */
    struct TaskRule
    {
        CondExpr   cond      = {};                             ///< Trigger condition (no TC allowed).
        bool       hasEvent  = false;                          ///< true if this rule emits an EVENT.
        EventVerb  eventVerb = EventVerb::INFO;                ///< Verb for the emitted event.
        char       eventText[ares::AMS_MAX_EVENT_TEXT] = {};   ///< Event message text.
        bool       hasSet    = false;                          ///< true if this rule executes a set action.
        SetAction  setAction = {};                             ///< Variable update action.
    };

    /**
     * A background task definition (AMS-11).
     *
     * Tasks run independently of the state machine on a fixed period.
     * An optional state filter (@c when @c in) restricts execution to
     * specific active states.  Each task holds up to @c AMS_MAX_TASK_RULES
     * conditional rules.
     */
    struct TaskDef
    {
        char     name[ares::AMS_MAX_STATE_NAME] = {};          ///< Task identifier (debug).
        uint32_t everyMs            = 0U;                      ///< Execution period (ms).
        uint8_t  ruleCount          = 0U;                      ///< Number of populated rules.
        TaskRule rules[ares::AMS_MAX_TASK_RULES] = {};         ///< Rule table.
        bool     hasStateFilter     = false;                   ///< true = only run in listed states.
        uint8_t  activeStateCount   = 0U;                      ///< Entries in activeStateNames[].
        char     activeStateNames[ares::AMS_MAX_TASK_ACTIVE_STATES][ares::AMS_MAX_STATE_NAME] = {}; ///< Unresolved names.
        uint8_t  activeStateIndices[ares::AMS_MAX_TASK_ACTIVE_STATES] = {}; ///< Resolved indices.
        bool     stateIndicesResolved = false;                 ///< true after resolveTasksLocked().
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
        uint8_t         condCount      = 0;
        TransitionLogic logic          = TransitionLogic::AND;
        CondExpr        conds[ares::AMS_MAX_TRANSITION_CONDS] = {};
        char            targetName[16] = {};
        uint8_t         targetIndex    = 0;
        bool            targetResolved = false;
        uint32_t        holdMs         = 0;  ///< Persistence window in ms (0 = immediate, AMS-4.6.1).
    };

    /**
     * One entry in the per-script alias table.
     * Created by 'include <MODEL> as <ALIAS> [retry=N] [timeout=Nms]' (AMS-4.9.1).
     */
    struct AliasEntry
    {
        char           alias[16]   = {};    ///< e.g. "GPS", "GPS1", "BARO"
        char           model[16]   = {};    ///< e.g. "BN220", "BMP280"
        PeripheralKind kind        = PeripheralKind::GPS;
        uint8_t        driverIdx   = 0xFFU; ///< Index in registry; 0xFF = not resolved
        uint8_t        retryCount  = 0U;    ///< Extra read attempts on failure (0 = no retry, max AMS_MAX_SENSOR_RETRY).
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

        /// If set, a guard violation or runtime error transitions to this
        /// state instead of halting the engine (AMS-4.10.2).
        bool    hasOnErrorTransition      = false;
        char    onErrorTransitionTarget[ares::AMS_MAX_STATE_NAME] = {};
        uint8_t onErrorTransitionIdx      = 0U;
        bool    onErrorTransitionResolved = false;

        // ── Fallback transition (AMS-4.9.2) ──────────────────
        /// If no regular transition fires within fallbackAfterMs,
        /// the fallback transition fires unconditionally.
        bool     hasFallback              = false;
        char     fallbackTargetName[ares::AMS_MAX_STATE_NAME] = {};
        uint8_t  fallbackTargetIdx        = 0U;
        bool     fallbackTargetResolved   = false;
        uint32_t fallbackAfterMs          = 0U;

        // ── on_enter set actions (AMS-4.8) ──────────────────
        uint8_t   setActionCount = 0;
        SetAction setActions[ares::AMS_MAX_SET_ACTIONS] = {};
    };

    struct Program
    {
        uint16_t   apid       = 0x01U;  ///< Active APID (APUS-10: rocket = 0x01).
        uint8_t    nodeId     = ares::DEFAULT_NODE_ID;
        uint8_t    stateCount = 0;
        StateDef   states[ares::AMS_MAX_STATES] = {};
        uint8_t    aliasCount = 0;
        AliasEntry aliases[ares::AMS_MAX_INCLUDES] = {};
        uint8_t    varCount   = 0;
        VarEntry   vars[ares::AMS_MAX_VARS] = {};   ///< Global variables (AMS-4.8).
        uint8_t    constCount = 0;
        ConstEntry consts[ares::AMS_MAX_CONSTS] = {}; ///< Named constants (AMS-4.12).
        uint8_t    taskCount  = 0;
        TaskDef    tasks[ares::AMS_MAX_TASKS] = {};   ///< Background tasks (AMS-11).
        uint8_t    assertCount = 0;
        AssertDef  asserts[ares::AMS_MAX_ASSERTS] = {}; ///< Formal assertions (AMS-15).
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
    bool parseOneConditionLocked(const char* condStr, bool allowTc, CondExpr& out);
    bool parseConditionScopedLineLocked(const char* line, StateDef& st);
    bool parseOnErrorEventLineLocked(const char* line, StateDef& st);
    bool parseVarLineLocked(const char* line);
    bool parseConstLineLocked(const char* line);
    bool parseSetActionLineLocked(const char* line, StateDef& st);
    bool parseSetActionCoreLocked(const char* line, SetAction& out);  ///< Shared core; called by state + task parsers.
    bool parseTaskLineLocked(const char* line);                        ///< AMS-11: parse task NAME[:when in…]: header.
    bool parseTaskScopedLineLocked(const char* line, BlockType& blockType); ///< AMS-11: parse lines inside a task block.
    bool parseAssertLineLocked(const char* line);                      ///< AMS-15: parse one directive inside assert:.
    bool parseFallbackTransitionLineLocked(const char* line, StateDef& st);
    bool parseOnErrorTransitionLineLocked(const char* line, StateDef& st);
    static bool mapApidToNode(uint16_t apid, uint8_t& nodeId);
    bool resolveTransitionsLocked();
    bool resolveTasksLocked();        ///< AMS-11: resolve 'when in' state names to indices after parsing.
    bool validateAssertionsLocked();  ///< AMS-15: run BFS/DFS graph checks; call from parseScriptLocked.

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
    void executeSetActionsLocked(StateDef& st, uint32_t nowMs);
    void executeOneSetActionLocked(SetAction& act, uint32_t nowMs); ///< Execute a single set action (shared by state + task paths).
    void runTasksLocked(uint32_t nowMs);                            ///< AMS-11: evaluate all background tasks.
    bool resolveVarThresholdLocked(const CondExpr& cond, float& outThreshold) const;
    VarEntry*       findVarLocked(const char* name);
    const VarEntry* findVarLocked(const char* name) const;
    const ConstEntry* findConstLocked(const char* name) const;
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

    bool saveResumePointLocked(uint32_t nowMs, bool force);
    bool tryRestoreResumePointLocked(uint32_t nowMs);
    void clearResumePointLocked();

    /**
     * Internal deactivation — must be called with mutex_ already held.
     * Resets all engine state to IDLE without acquiring the mutex.
     * @pre  mutex_ is held by the caller.
     */
    void deactivateLocked();

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
    uint8_t   tcConfirmCount_[4] = {}; ///< Per-TC injection counter for CONFIRM mode (AMS-4.11.2).
    uint8_t seq_ = 0;
    char logPath_[ares::STORAGE_MAX_PATH] = {};

    // ── Transition hold (debounce) state (AMS-4.6.1) ─────────────────────────
    bool     transitionCondHolding_ = false;  ///< true while condition is being timed.
    uint32_t transitionCondMetMs_   = 0;      ///< millis() when condition first became true.

    // ── Transition delta tracking (AMS-4.6.2) ────────────────────────────────
    // Previous sensor readings indexed by condition slot [0..AMS_MAX_TRANSITION_CONDS-1].
    // Reset on every state entry so delta is always relative to the state-entry baseline.
    float transitionPrevVal_[ares::AMS_MAX_TRANSITION_CONDS]   = {};
    bool  transitionPrevValid_[ares::AMS_MAX_TRANSITION_CONDS] = {};

    bool logHeaderWritten_ = false;
    uint32_t lastCheckpointMs_ = 0;

    // ── IMU read cache ────────────────────────────────────────────────────────
    // When a LOG.report or HK.report contains multiple IMU fields, each field
    // would otherwise trigger a separate I2C burst.  These mutable members
    // cache the last successful burst so all fields in one report share a
    // single read, reducing I2C traffic and eliminating the partial-nan pattern
    // caused by each field independently succeeding or failing.
    mutable ImuReading imuCachedReading_ = {};  ///< Last successful IMU burst.
    mutable uint32_t   imuCacheTsMs_     = 0;   ///< millis() when last attempt was made.
    mutable bool       imuCacheValid_    = false; ///< true iff imuCachedReading_ holds good data.
    // Max age (ms) before a new read attempt is made.  Updated on both success
    // and failure so all fields in one report share a single attempt (no 8x timeout).
    static constexpr uint32_t IMU_CACHE_MAX_AGE_MS = 5U;

    char scriptBuffer_[ares::AMS_MAX_SCRIPT_BYTES + 1U] = {};

    // Current line number during script parsing (1-based).
    // Zero when the engine is not actively parsing.
    // Used by setErrorLocked() to emit "Parse Error (line N): ..." messages.
    uint32_t parseLineNum_ = 0U;

    // ── AMS-11: per-task last-fire timestamps ─────────────────────────────────
    // Indexed by task slot in program_.tasks[].  Reset on deactivate().
    uint32_t taskLastTickMs_[ares::AMS_MAX_TASKS] = {};

    // ── Parser context for task blocks (valid only during parseScriptLocked) ──
    uint8_t  parseCurrentTask_     = 0xFFU; ///< Index of task being parsed; 0xFF = none.
    uint8_t  parseCurrentTaskRule_ = 0xFFU; ///< Index of current if-rule within task; 0xFF = none.
};

} // namespace ams
} // namespace ares

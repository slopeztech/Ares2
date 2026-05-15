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
#include "hal/pulse/pulse_interface.h"
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
    NONE             = 0,   ///< No pending command.
    LAUNCH           = 1,   ///< Launch command — triggers LAUNCH transitions.
    ABORT            = 2,   ///< Abort command — triggers ABORT transitions.
    RESET            = 3,   ///< Reset command — triggers RESET transitions.
    RESET_ABNORMAL   = 4,   ///< Injected on boot after abnormal reset (panic/WDT/brownout) while in-flight.

    FIRST = NONE,  // CERT-6.1 — range validation sentinels
    LAST  = RESET_ABNORMAL,
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
     * @param[in] pulseIface  Optional electric-pulse channel interface.
     *                        Pass @c nullptr to disable PULSE.fire execution
     *                        (e.g. in SITL tests without a pulse driver).
     */
    MissionScriptEngine(StorageInterface&  storage,
                        const GpsEntry*    gpsDrivers,  uint8_t gpsCount,
                        const BaroEntry*   baroDrivers, uint8_t baroCount,
                        const ComEntry*    comDrivers,  uint8_t comCount,
                        const ImuEntry*    imuDrivers,  uint8_t imuCount,
                        PulseInterface*    pulseIface = nullptr);

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
    void tick(uint64_t nowMs);

    /**
     * Compute the earliest timestamp at which the next engine event is due.
     *
     * For states with active transition or guard conditions, returns
     * @p nowMs + SENSOR_RATE_MS so the main loop keeps ticking at full rate.
     * For states with no conditions (e.g. pure HK-reporting or terminal states),
     * returns the time of the next scheduled HK/LOG slot, background task, or
     * timeout — capped to @p nowMs + kRadioMaxSleepMs for TC responsiveness.
     *
     * Intended use: the main loop calls this immediately after @c tick() to
     * determine how long to sleep before the next @c tick() call.
     *
     * @note  Lock-free by design — this is a scheduling hint, not a correctness
     *        path.  Reads of uint8_t/uint32_t members are atomic on Xtensa LX7.
     * @param[in] nowMs  Current millis() timestamp.
     * @return Absolute millis() timestamp for the next required wakeup.
     */
    uint64_t nextWakeupMs(uint64_t nowMs) const;

    /**
     * Enable or disable runtime progression (state transitions and periodic actions).
     * @param[in] enabled  true to enable; false to pause without deactivating.
     */
    void setExecutionEnabled(bool enabled);

    /**
     * Immediately transmit all active HK telemetry slots for the current state.
     *
     * Acquires the engine mutex, builds a @c TelemetryPayload for every
     * active HK slot in the current state, and sends each as a TELEMETRY frame
     * via the primary COM link. Falls back to the legacy single-slot path if
     * no multi-slot data is present.
     *
     * Called by @c RadioDispatcher on reception of a @c REQUEST_TELEMETRY
     * command (APUS-7, @c CommandId::REQUEST_TELEMETRY).
     *
     * @param[in] nowMs  Current millis() timestamp (written into the frame).
     * @return true if at least one frame was transmitted.
     */
    bool requestTelemetry(uint64_t nowMs);

    /**
     * Override the HK report interval for all active telemetry slots in the
     * current state.  The change is in-RAM only — it does not persist across
     * state transitions or script reloads.
     *
     * Called by @c RadioDispatcher on reception of a @c SET_TELEM_INTERVAL
     * command (APUS-7, @c CommandId::SET_TELEM_INTERVAL).
     *
     * @param[in] intervalMs  New interval in milliseconds.
     * @return true if at least one slot was updated.
     */
    bool setTelemInterval(uint32_t intervalMs);

    /**
     * Thread-safe read of the current wire-protocol StatusBits
     * (armed, fcsActive, gpsValid, pulseAFired, pulseBFired).
     *
     * Used by @c RadioDispatcher when building an immediate status
     * response to a @c REQUEST_STATUS command (APUS-6).
     *
     * @return StatusBits populated from the engine's current runtime state.
     */
    ares::proto::StatusBits getStatusBits() const;

    /**
     * Record that a pulse channel was successfully actuated.
     *
     * Sets the corresponding bit in StatusBits so the next telemetry
     * frame reflects the actuation.  Must be called by the layer that
     * drives the pulse GPIO after a confirmed FIRE_PULSE_A / FIRE_PULSE_B
     * command.
     *
     * @param[in] channel  0 = pulse A (drogue), 1 = pulse B (main).
     */
    void notifyPulseFired(uint8_t channel);

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

    /**
     * @brief Bridge from ST[20] parameter management to ST[12] monitoring
     *        thresholds (APUS-16.3, APUS-12.1).
     *
     * When the ground sends SET_CONFIG_PARAM with a monitoring-threshold
     * ConfigParamId, RadioDispatcher calls this to propagate the new value
     * into the corresponding MonitoringSlot at runtime.
     *
     * @param[in] id     Config parameter identifier (monitoring params only).
     * @param[in] value  New threshold value (in the same unit as the payload field).
     * @return true if @p id mapped to a monitoring slot and was updated.
     */
    bool configureMonitorFromParam(ares::proto::ConfigParamId id, float value);

    /**
     * @brief Query whether the loaded script declares a @c radio.config override
     *        for @p id (AMS-4.15).
     *
     * Called by RadioDispatcher after a successful @c arm() to apply
     * script-declared default parameter values into the config table.
     * Ground can still override these values later via @c SET_CONFIG_PARAM.
     *
     * @param[in]  id     Parameter identifier.
     * @param[out] value  Set to the script-declared value when @c true is returned.
     * @return @c true if the script declared an override for @p id.
     */
    bool getScriptRadioConfig(ares::proto::ConfigParamId id, float& value) const;

private:
    /** Parser state: identifies which top-level directive block is currently open. */
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
        ON_EXIT    = 9,  ///< Inside an on_exit: block (EVENT.* and set actions).
        ON_TIMEOUT = 10, ///< Inside an on_timeout: block (transition + optional EVENT).
    };

    // ── Peripheral kind ──────────────────────────────────────────────────────
    /** Sensor peripheral category for alias resolution in AMS scripts. */
    enum class PeripheralKind : uint8_t
    {
        GPS  = 0,
        BARO = 1,
        COM  = 2,
        IMU  = 3,
    };

    // ── Sensor field within a peripheral ─────────────────────────────────────
    /** Physical measurement field within a peripheral alias (AMS-4.6). */
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
        SATS      = 14, ///< GPS: satellites in use (integer, compared as float)
        HDOP      = 15, ///< GPS: horizontal dilution of precision (dimensionless)
    };

    // ── Unified condition kind ─────────────────────────────────────────────
    /** Discriminant for a unified CondExpr — sensor comparison, TC match, or elapsed time. */
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
    /** TC debounce policy for TC_EQ conditions (AMS-4.11). */
    enum class TcDebounceMode : uint8_t
    {
        ONESHOT  = 0U, ///< Fire on first TC match — one-shot, existing behavior (AMS-4.11.1).
        ONCE     = 1U, ///< Explicit alias for ONESHOT — self-documenting keyword.
        CONFIRM  = 2U, ///< Require N successive inject calls before firing (AMS-4.11.2).
    };

    // ── Logical operator for compound transitions (AMS-4.6.2) ─────────────
    /** Boolean combinator for multi-condition transitions (AMS-4.6.2). */
    enum class TransitionLogic : uint8_t
    {
        AND = 0,  ///< All sub-conditions must be true.
        OR  = 1,  ///< Any sub-condition being true is sufficient.
    };

    // ── Unified condition expression ───────────────────────────────────────
    /**
     * A single parsed AMS condition (sensor comparison, TC match, or elapsed time).
     * Used in transitions, task rules, and assert blocks.
     */
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
        uint8_t       calibSamples   = 1U;    ///< Samples to average (CALIBRATE form, 1–AMS_CALIBRATE_MAX_SAMPLES).
        float         deltaBaseline  = 0.0f;  ///< Previous sensor value (DELTA form).
        bool          deltaValid     = false; ///< Whether deltaBaseline has been set.

        // ── Async CALIBRATE progress (AMS-4.8.2) ─────────────────────────────
        // Reset by enterStateLocked() on every state entry; advanced one sample
        // per tick by stepPendingCalibrationsLocked() until all calibSamples are
        // collected.  Non-CALIBRATE set actions leave these fields unused.
        float   calibSum        = 0.0f;   ///< Accumulated sum of valid samples.
        uint8_t calibValidN     = 0U;     ///< Valid (non-failed) samples collected.
        uint8_t calibCollected  = 0U;     ///< Total sample attempts so far.
        bool    calibInProgress = false;  ///< True while async calibration is running.
    };

    /** Severity level for AMS on_enter and task rule events (AMS-4.7). */
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
     * One scheduled HK (telemetry) or LOG (CSV) report slot (AMS-4.3.1).
     *
     * A state may have up to @c AMS_MAX_HK_SLOTS of these for each of
     * the @c every (HK) and @c log_every (LOG) directives, allowing
     * independent cadences (e.g. IMU at 10 ms and GPS at 1 s).
     */
    struct HkSlot
    {
        uint32_t everyMs    = 0U;                            ///< Report interval (ms); 0 = unused.
        uint8_t  fieldCount = 0U;                            ///< Number of populated fields.
        HkField  fields[ares::AMS_MAX_HK_FIELDS] = {};      ///< Field descriptors.
    };

    /**
     * A single guard condition (evaluated every tick).
     * TC.command is intentionally excluded — use transitions for TC gates.
     */
    struct Condition
    {
        CondExpr expr = {};
    };

    /**
     * A parsed state-machine transition: conditions, logic operator, target state,
     * and optional persistence hold window (AMS-4.6).
     */
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

    /**
     * A single parsed AMS state definition: name, on_enter event, scheduled
     * reports, transitions, set actions, and fallback configuration.
     */
    struct StateDef
    {
        char name[ares::AMS_MAX_STATE_NAME] = {};

        bool hasOnEnterEvent = false;
        EventVerb onEnterVerb = EventVerb::INFO;
        char onEnterText[ares::AMS_MAX_EVENT_TEXT] = {};

        // ── HK (every) and LOG (log_every) slots (AMS-4.3.1) ────────────────
        // Up to AMS_MAX_HK_SLOTS independent cadences per state.
        uint8_t hkSlotCount  = 0U;                            ///< Active every slots.
        HkSlot  hkSlots[ares::AMS_MAX_HK_SLOTS]  = {};       ///< Scheduled HK reports.
        uint8_t logSlotCount = 0U;                            ///< Active log_every slots.
        HkSlot  logSlots[ares::AMS_MAX_HK_SLOTS] = {};       ///< Scheduled LOG reports.

        // ── Legacy single-slot accessors (kept for telemetry helpers) ────────
        // Mirrors hkSlots[0] / logSlots[0]; kept so existing helper code
        // that references hasHkEvery / hkFields etc. still compiles.
        bool     hasHkEvery   = false;
        uint32_t hkEveryMs    = 0U;
        uint8_t  hkFieldCount = 0U;
        HkField  hkFields[ares::AMS_MAX_HK_FIELDS] = {};

        bool     hasLogEvery   = false;
        uint32_t logEveryMs    = 0U;
        uint8_t  logFieldCount = 0U;
        HkField  logFields[ares::AMS_MAX_HK_FIELDS] = {};

        uint8_t hkPriority    = 2;  ///< Higher value = higher priority.
        uint8_t logPriority   = 1;  ///< Keep local log below PUS HK by default.
        uint8_t eventPriority = 4;  ///< PUS event reporting has highest default priority.
        uint8_t actionBudget  = 2;  ///< Max actions per tick (EVENT/HK/LOG arbitration).

        // ── Regular transitions (AMS-4.6) ──────────────────────────────────
        // Up to AMS_MAX_TRANSITIONS independent "transition to" directives per state.
        // The first condition that evaluates true (respecting its own hold window)
        // fires; subsequent transitions in the list are not evaluated that tick.
        uint8_t    transitionCount = 0U;
        Transition transitions[ares::AMS_MAX_TRANSITIONS] = {};

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

        // ── on_enter pulse fire actions (AMS-4.17) ───────────
        /// A single PULSE.fire command parsed from an on_enter: block.
        struct PulseAction
        {
            uint8_t  channel;      ///< PulseChannel::CH_A (0) or CH_B (1).
            uint32_t durationMs;   ///< Pulse duration; 0 means use config default.
        };
        uint8_t      pulseActionCount = 0U;
        PulseAction  pulseActions[ares::AMS_MAX_PULSE_ACTIONS] = {};

        // ── on_exit handler (AMS-4.9) ────────────────────────
        // Executed synchronously when leaving this state via any transition
        // (normal, fallback, or error-recovery). Does NOT fire on deactivate()
        // or when entering ERROR without a recovery transition.
        bool      hasOnExitEvent = false;
        EventVerb onExitVerb     = EventVerb::INFO;
        char      onExitText[ares::AMS_MAX_EVENT_TEXT] = {};
        uint8_t   onExitSetCount = 0;
        SetAction onExitSetActions[ares::AMS_MAX_SET_ACTIONS] = {};

        // ── on_timeout handler (AMS-4.10.3) ─────────────────────────────────
        // Forces a transition after onTimeoutMs if no regular transition fired.
        // An optional EVENT is emitted before the forced transition.
        bool      hasOnTimeout                = false;
        uint32_t  onTimeoutMs                 = 0U;
        bool      hasOnTimeoutEvent           = false;
        EventVerb onTimeoutVerb               = EventVerb::WARN;
        char      onTimeoutText[ares::AMS_MAX_EVENT_TEXT]       = {};
        char      onTimeoutTransitionTarget[ares::AMS_MAX_STATE_NAME] = {};
        uint8_t   onTimeoutTransitionIdx      = 0U;
        bool      onTimeoutTransitionResolved = false;
    };

    /**
     * Complete parsed AMS script: APID, node ID, state table, alias table,
     * variables, constants, tasks, and formal assertions.
     */
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
        char       hkAlias[16]    = {};  ///< PUS ST[3] service alias (default: "HK").
        char       eventAlias[16] = {};  ///< PUS ST[5] service alias (default: "EVENT").
        char       tcAlias[16]    = {};  ///< PUS ST[1] service alias (default: "TC").

        // ── AMS-4.15: script-declared radio config overrides ─────────────────
        /**
         * One @c radio.config override declared in the script metadata section.
         *
         * When @c set is true, @c value replaces the compile-time default in
         * RadioDispatcher::configParams_ at arm time.  Ground can still send
         * SET_CONFIG_PARAM afterwards to further adjust the value (APUS-16.1).
         */
        struct RadioConfigParam
        {
            ares::proto::ConfigParamId id;    ///< Parameter identifier.
            float                      value; ///< Script-declared default value.
            bool                       set;   ///< true when this slot has been populated.
        };
        // ARES-MISRA-DEV-002: outer static_cast<uint8_t> suppresses integral promotion
        // from uint8_t arithmetic; result is [0,6] and always fits in uint8_t.
        static constexpr uint8_t kRadioConfigCount =
            static_cast<uint8_t>(
                static_cast<uint8_t>(ares::proto::ConfigParamId::LAST) -
                static_cast<uint8_t>(ares::proto::ConfigParamId::FIRST) + 1U);
        RadioConfigParam radioConfig[kRadioConfigCount] = {};  ///< Script radio.config overrides (AMS-4.15), indexed by (ConfigParamId - FIRST).
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
    bool parseTopLevelDirectiveLocked(const char* line,
                                      uint8_t& currentState,
                                      BlockType& blockType,
                                      bool& handled);
    bool parsePusApidDirectiveLocked(const char* line);
    bool parsePusServiceDirectiveLocked(const char* line);
    bool parseRadioConfigLineLocked(const char* line);  ///< AMS-4.15: radio.config PARAM = VALUE.
    bool parseNonStateBlockLineLocked(const char* line,
                                      BlockType& blockType,
                                      bool& handled);
    bool parseIncludeLineLocked(const char* line);
    bool lookupModelInDriversLocked(const char* model, PeripheralKind& kind,
                                     uint8_t& driverIdx) const;
    bool parseIncludeOptionalsLocked(const char* line, AliasEntry& ae);
    bool parseStateScopedLineLocked(const char* line,
                                    StateDef& st,
                                    BlockType& blockType);
    bool parseStateBlockHeaderLocked(const char* line,
                                     StateDef& st,
                                     BlockType& blockType,
                                     bool& handled);
    bool parseStateRateDirectivesLocked(const char* line, StateDef& st,
                                        BlockType& blockType, bool& matched);
    bool parseStateReportDirectivesLocked(const char* line, const StateDef& st,
                                          BlockType& blockType, bool& matched);
    bool parseStateBlockContentLocked(const char* line,
                                      StateDef& st,
                                      BlockType blockType,
                                      bool& handled);
    bool parseOnErrorBlockLineLocked(const char* line, StateDef& st);
    bool parseOnEnterBlockLineLocked(const char* line, StateDef& st);
    bool parseOnExitBlockLineLocked(const char* line, StateDef& st);
    bool parseOnTimeoutContentLineLocked(const char* line, StateDef& st);
    bool parseHkSlotFieldLineLocked(const char* line, StateDef& st);
    bool parseLogSlotFieldLineLocked(const char* line, StateDef& st);
    bool parseStateLineLocked(const char* line, uint8_t& currentState);
    bool parseEventLineLocked(const char* line, StateDef& st);
    bool parseEveryLineLocked(const char* line, StateDef& st);
    bool parseLogEveryLineLocked(const char* line, StateDef& st);    static bool parsePrioritiesValuesLocked(const char* line,
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
    bool buildAndStoreTransitionLocked(const char* target, uint32_t holdMs,
                                        TransitionLogic logic,
                                        const char* const condParts[],
                                        uint8_t condCount, StateDef& st);
    bool parseTransitionHoldClauseLocked(char* bodyBuf, uint32_t& holdMs);
    bool splitTransitionConditionsLocked(char*           bodyBuf,
                                         const char*    condParts[],
                                         char           condBufs[][ares::AMS_MAX_LINE_LEN],
                                         uint8_t&       condCount,
                                         TransitionLogic& logic);
    bool parseOneConditionLocked(const char* condStr, bool allowTc, CondExpr& out);
    bool parseDeltaCondLocked(const char* tok1, const char* tok3, const char* tok4,
                               CondExpr& out);
    bool parseFallingRisingCondLocked(const char* tok1, const char* tok2, CondExpr& out);
    bool parseTcDebounceCondLocked(bool allowTc, const char* d3, const char* d4,
                                    const char* d5, int32_t nd, CondExpr& out);
    bool parseConditionScopedLineLocked(const char* line, StateDef& st);
    bool parseOnErrorEventLineLocked(const char* line, StateDef& st);
    bool parseOnExitEventLineLocked(const char* line, StateDef& st);
    bool parseOnTimeoutHeaderLocked(const char* line, StateDef& st, BlockType& blockType);
    bool parseOnTimeoutEventLineLocked(const char* line, StateDef& st);
    bool parseOnTimeoutTransitionLineLocked(const char* line, StateDef& st);
    bool parseVarLineLocked(const char* line);
    bool parseConstLineLocked(const char* line);
    bool validateConstIdentifierLocked(const char* name);
    bool ensureConstDoesNotConflictLocked(const char* name);
    bool parseSetActionLineLocked(const char* line, StateDef& st);
    bool parsePulseFireLineLocked(const char* line, StateDef& st);            ///< AMS-4.17: parse "PULSE.fire A[/B] [Nms]".
    bool parsePulseDurationSuffixLocked(const char* afterCh, uint32_t& out); ///< Parse optional " Nms" suffix; called by parsePulseFireLineLocked.
    bool parseSetActionCoreLocked(const char* line, SetAction& out);  ///< Shared core; called by state + task parsers.
    bool ensureSetVariableExistsLocked(const char* varName);
    bool parseCalibrateSetActionLocked(const char* rhsBuf, SetAction& out);
    bool parseMinMaxSetActionLocked(const char* rhsBuf,
                                    const char* varName,
                                    SetAction&  out);
    bool parseDeltaSetActionLocked(const char* rhsBuf, SetAction& out);
    bool parseSimpleSensorSetActionLocked(const char* rhsBuf, SetAction& out);
    bool parseTaskLineLocked(const char* line);                        ///< AMS-11: parse task NAME[:when in…]: header.
    bool parseTaskWhenInClauseLocked(const char* line, const char* whenIn, TaskDef& td);
    bool parseTaskScopedLineLocked(const char* line, BlockType& blockType); ///< AMS-11: parse lines inside a task block.
    bool parseTaskEveryPeriodLocked(const char* line, TaskDef& td);
    bool parseTaskIfOpenLocked(const char* line, TaskDef& td, BlockType& blockType);
    bool parseTaskIfBodyLocked(const char* line, TaskDef& td);
    bool parseAssertLineLocked(const char* line);                      ///< AMS-15: parse one directive inside assert:.
    bool parseFallbackTransitionLineLocked(const char* line, StateDef& st);
    bool parseFallbackTimeoutMsLocked(const char* line, uint32_t& afterMs);
    bool parseOnErrorTransitionLineLocked(const char* line, StateDef& st);
    static bool mapApidToNode(uint16_t apid, uint8_t& nodeId);
    bool resolveTransitionsLocked();
    bool resolveTasksLocked();        ///< AMS-11: resolve 'when in' state names to indices after parsing.
    bool validateAssertionsLocked();  ///< AMS-15: run BFS/DFS graph checks; call from parseScriptLocked.
    void computeBfsReachabilityLocked(uint16_t& reachable) const;
    void computeDfsMaxDepthLocked(uint8_t& maxDepth, bool& hasCycle) const;
    bool evaluateOneAssertionLocked(const AssertDef& ad, uint16_t reachable,
                                     uint8_t maxDepth, bool hasCycle);
    void resolvePrimaryComLocked();

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
    static bool parseGpsSensorField(const char* fieldStr, SensorField& out);
    static bool parseBaroSensorField(const char* fieldStr, SensorField& out);
    static bool parseImuSensorField(const char* fieldStr, SensorField& out);
    static bool splitAliasDotField(const char* expr,
                                   char*       aliasOut, uint8_t aliasSize,
                                   char*       fieldOut,  uint8_t fieldSize);

    const AliasEntry* findAliasLocked(const char* alias) const;
    bool readSensorFloatLocked(const char*  alias,
                               SensorField  field,
                               float&       outVal) const;
    bool readGpsFieldLocked(const AliasEntry& ae,
                            SensorField       field,
                            float&            outVal) const;
    bool readBaroFieldLocked(const AliasEntry& ae,
                             SensorField       field,
                             float&            outVal) const;
    bool readImuFieldLocked(const AliasEntry& ae,
                            SensorField       field,
                            float&            outVal) const;
    bool refreshImuCacheLocked(ImuInterface* imu, uint8_t maxAttempts) const;
    bool parseCondExprLocked(const char* lhs,
                             const char* op,
                             const char* rhs,
                             bool        allowTc,
                             CondExpr&   out);
    bool parseTcCondExprLocked(const char* op,
                               const char* rhs,
                               bool        allowTc,
                               CondExpr&   out);
    bool parseTimeCondExprLocked(const char* op,
                                 const char* rhs,
                                 CondExpr&   out);
    bool parseSensorCondExprLocked(const char* lhs,
                                   const char* op,
                                   const char* rhs,
                                   CondExpr&   out);
    bool parseConditionRhsThresholdLocked(const char* rhs,
                                          CondExpr&   out);

    bool parseRhsVarNameOffsetLocked(const char* varExpr, char* varName, uint8_t varNameSz,
                                      float& offset, bool& hasOffset);
    uint8_t findStateByNameLocked(const char* name) const;
    void    suggestStateNameLocked(const char* typo, char* buf, uint8_t bufSize) const;

    static uint8_t levenshteinDist(const char* a, const char* b);

    void setErrorLocked(const char* reason);
    void exitStateLocked(uint8_t stateIndex, uint64_t nowMs);
    void enterStateLocked(uint8_t stateIndex, uint64_t nowMs);
    bool checkOnTimeoutLocked(StateDef& state, uint64_t nowMs);
    bool evaluateTransitionAndMaybeEnterLocked(StateDef& state, uint64_t nowMs);
    bool evaluateOneTransitionConditionLocked(const CondExpr& cond,
                                              StateDef&,
                                              uint8_t         condIdx,
                                              uint64_t        nowMs,
                                              bool&           tcPendingMatch);
    bool applyTransitionHoldLocked(const Transition& tr, uint8_t trIdx, uint64_t nowMs);
    void consumeMatchedTransitionTcLocked(const Transition& tr);
    bool fireResolvedTransitionLocked(const StateDef&   state,
                                      const Transition& tr,
                                      uint64_t          nowMs);
    void executeDueActionsLocked(const StateDef& state, uint64_t nowMs);

    void sendOnEnterEventLocked(uint64_t nowMs);
    void executeSetActionsLocked(StateDef& st, uint64_t nowMs);
    void executePulseActionsLocked(const StateDef& st);  ///< AMS-4.17: fire all PULSE.fire actions for state entry.
    void executeOneSetActionLocked(SetAction& act, uint64_t nowMs); ///< Execute a single set action (shared by state + task paths).
    void stepPendingCalibrationsLocked(StateDef& st, uint64_t nowMs); ///< AMS-4.8.2: advance each in-progress CALIBRATE by one sample.
    void executeCalibrateSetActionLocked(SetAction& act, float& result, bool& gotReading, uint64_t nowMs);
    void executeDeltaSetActionLocked(SetAction& act, float& result, bool& gotReading);
    void executeMinMaxSetActionLocked(SetAction& act, const VarEntry* v, float& result, bool& gotReading);
    void runTasksLocked(uint64_t nowMs);                            ///< AMS-11: evaluate all background tasks.
    bool evaluateTaskRuleCondLocked(const TaskRule& rule, uint64_t nowMs, bool& condResult) const;
    bool resolveVarThresholdLocked(const CondExpr& cond, float& outThreshold) const;
    VarEntry*       findVarLocked(const char* name);
    const VarEntry* findVarLocked(const char* name) const;
    const ConstEntry* findConstLocked(const char* name) const;
    bool evaluateConditionsLocked(const StateDef& state, uint64_t nowMs);
    static const char* sensorFieldNameForLog(SensorField f);
    bool evaluateGuardExprHoldsLocked(const CondExpr& expr,
                                      const StateDef& state,
                                      uint64_t        nowMs,
                                      float&          actualVal) const;
    bool handleGuardViolationLocked(const StateDef& state,
                                    const CondExpr& expr,
                                    float           actualVal,
                                    uint64_t        nowMs);
    void logGuardViolationLocked(const StateDef& state,
                                 const CondExpr& expr,
                                 float           actualVal,
                                 uint64_t        nowMs);
    bool applyGuardErrorLocked(const StateDef& state, uint64_t nowMs);
    void sendHkReportLocked(uint64_t nowMs);
    void sendHkReportSlotLocked(uint64_t nowMs, const HkSlot& slot);       ///< AMS-4.3.1: single slot variant.
    void appendLogReportLocked(uint64_t nowMs);
    void appendLogReportSlotLocked(uint64_t nowMs,
                                   const HkSlot& slot,
                                   uint8_t slotIdx);                       ///< AMS-4.3.1: single slot variant.
    bool writeLogHeaderIfNeededLocked(const StateDef& st);
    bool buildLogDataRowLocked(const StateDef& st,
                               uint64_t        nowMs,
                               char*           outLine,
                               uint32_t        outSize,
                               uint32_t&       outLen) const;
    void sendEventLocked(EventVerb verb, ares::proto::EventId id, const char* text, uint64_t nowMs);

    /**
     * @brief Infer the APUS-8 EventId from an AMS EventVerb.
     *
     * Used when the exact EventId cannot be determined at the call site
     * (e.g. user-defined task rule events).  Mapping:
     *   - INFO  → PHASE_CHANGE  (informational state update)
     *   - WARN  → SENSOR_FAILURE (anomaly detected)
     *   - ERROR → FPL_VIOLATION  (flight parameter limit violated)
     *
     * @param[in] verb  AMS event verb.
     * @return Corresponding APUS-8 EventId.
     */
    static ares::proto::EventId inferEventId(EventVerb verb);
    bool ensureLogFileLocked(const char* fileName);
    void applyHkFieldToPayloadLocked(const HkField&               f,
                                     ares::proto::TelemetryPayload& tm) const;
    /**
     * Build the StatusBits word from current engine state and GPS health.
     * @pre  Caller holds mutex_.
     * @return Populated StatusBits struct (all reserved bits cleared).
     */
    ares::proto::StatusBits buildStatusBitsLocked() const;
    /**
     * @brief Return the satellite count from the first GPS driver that has a
     *        valid fix.  Returns 0 if no GPS driver is configured or has a fix.
     * @pre  Caller holds mutex_.
     */
    uint8_t readGpsSatsLocked() const;

    /**
     * @brief Evaluate all enabled monitoring slots against @p tm (APUS-12.4).
     *
     * Implements the APUS-12 state machine: after @c consecutiveRequired
     * consecutive violations, transitions ENABLED → ALARM and sends an
     * APUS-8 EVENT frame.  Recovers ALARM → ENABLED when within limits
     * (APUS-12.2).  Disabled slots are skipped without CPU overhead (APUS-12.5).
     *
     * @pre  Caller holds mutex_.
     * @param[in] tm     The TelemetryPayload just built (raw values, before delta).
     * @param[in] nowMs  Current millis() for the event timestamp.
     */
    void evaluateMonitoringLocked(const ares::proto::TelemetryPayload& tm,
                                  uint64_t nowMs);

    /**
     * @brief Extract the value of @p id from @p tm.
     *
     * @param[in] id  Monitored parameter identifier.
     * @param[in] tm  Telemetry payload from the current HK frame.
     * @return The corresponding field value, or 0.0f for unknown IDs.
     */
    static float extractMonitorParam(ares::proto::MonitorParamId id,
                                     const ares::proto::TelemetryPayload& tm);
    bool formatHkFieldValueLocked(const HkField& f,
                                  char*          out,
                                  uint32_t       outSize) const;

    bool sendFrameLocked(const ares::proto::Frame& frame);

    /// Stage a log-file append for deferred execution outside the mutex (AMS-8.3).
    void queueAppendLocked(const char* path, const uint8_t* data, uint32_t len);
    /// Flush all staged file I/O that was built under the mutex. Must be called
    /// after the mutex is released. Idempotent if nothing is pending.
    void flushPendingIoUnlocked();

    bool saveResumePointLocked(uint64_t nowMs, bool force);
    bool buildCheckpointRecordLocked(uint64_t nowMs, char* record, size_t recSize, int32_t& outWritten) const;
    void appendVarsSectionLocked(char* record, size_t recSize, int32_t& written) const;
    void appendSlotTimersSectionLocked(char* record, size_t recSize, int32_t& written, uint64_t nowMs) const;
    bool tryRestoreResumePointLocked(uint64_t nowMs);
    bool applyCheckpointStateLocked(uint64_t nowMs, uint32_t stateIdx, uint32_t execEnabled,
                                     uint32_t running, uint32_t status, uint32_t seq,
                                     uint32_t stateElapsed, uint32_t hkElapsed,
                                     uint32_t logElapsed);
    static bool parseCheckpointHeaderLocked(const char* buf, uint32_t& version, char* fileName, uint32_t& stateIdx, uint32_t& execEnabled, uint32_t& running, uint32_t& status, uint32_t& seq, uint32_t& stateElapsed, uint32_t& hkElapsed, uint32_t& logElapsed);
    const char* restoreCheckpointVarsLocked(const char* cursor);
    void restoreCheckpointSlotsLocked(const char* cursor, uint64_t nowMs);
    void clearResumePointLocked();
    void writeAbortMarkerLocked(const char* stateName, uint64_t nowMs);

    /**
     * Internal deactivation — must be called with mutex_ already held.
     * Resets all engine state to IDLE without acquiring the mutex.
     * @pre  mutex_ is held by the caller.
     */
    void deactivateLocked();

    /** Reset per-alias baro/GPS/IMU sensor read caches. @pre mutex_ held. */
    void resetSensorCachesLocked();
    /** Reset telemetry-derived state (altitude delta, HK counter, etc.). @pre mutex_ held. */
    void resetTelemetryStateLocked();
    /** Reset ST[12] monitoring counters and FSM states. @pre mutex_ held. */
    void resetMonitorSlotsLocked();

    /** Compute earliest due time across active HK slots; returns updated @p cur. */
    uint64_t nextDueFromHkSlots(const StateDef& s, uint64_t cur) const;
    /** Compute earliest due time across active LOG slots; returns updated @p cur. */
    uint64_t nextDueFromLogSlots(const StateDef& s, uint64_t cur) const;
    /** Compute earliest due time across background tasks; returns updated @p cur. */
    uint64_t nextDueFromTasks(uint64_t cur) const;

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
    PulseInterface*      pulseIface_ = nullptr;  ///< Nullable — pulse disabled if null (AMS-4.17).

    StaticSemaphore_t mutexBuf_ = {};
    SemaphoreHandle_t mutex_ = nullptr;

    Program program_ = {};

    EngineStatus status_ = EngineStatus::IDLE;
    char activeFile_[ares::MISSION_FILENAME_MAX] = {};
    char lastError_[ares::AMS_MAX_ERROR_TEXT] = {};

    bool running_ = false;
    bool executionEnabled_ = false;
    uint8_t currentState_ = 0;
    uint64_t stateEnterMs_ = 0;
    uint64_t lastHkMs_ = 0;
    uint64_t lastLogMs_ = 0;
    /// Per-slot last-fire timestamps (AMS-4.3.1). Index matches hkSlots[]/logSlots[].
    uint64_t lastHkSlotMs_[ares::AMS_MAX_HK_SLOTS]  = {};
    uint64_t lastLogSlotMs_[ares::AMS_MAX_HK_SLOTS] = {};
    bool pendingOnEnterEvent_ = false;
    EventVerb pendingEventVerb_ = EventVerb::INFO;
    char pendingEventText_[ares::AMS_MAX_EVENT_TEXT] = {};
    uint64_t pendingEventTsMs_ = 0;

    TcCommand pendingTc_ = TcCommand::NONE;
    uint8_t   tcConfirmCount_[4] = {}; ///< Per-TC injection counter for CONFIRM mode (AMS-4.11.2).
    bool      pulseAFired_ = false;    ///< True after FIRE_PULSE_A was successfully executed (APUS-6).
    bool      pulseBFired_ = false;    ///< True after FIRE_PULSE_B was successfully executed (APUS-6).
    uint8_t seq_ = 0;
    char logPath_[ares::STORAGE_MAX_PATH] = {};

    // ── Transition hold (debounce) state (AMS-4.6.1) ─────────────────────────
    // One hold-window entry per transition slot [0..AMS_MAX_TRANSITIONS-1].
    bool     transitionCondHolding_[ares::AMS_MAX_TRANSITIONS] = {};  ///< true while that transition's condition is being timed.
    uint64_t transitionCondMetMs_[ares::AMS_MAX_TRANSITIONS]   = {};  ///< millis64() when that transition's condition first became true.

    // ── Transition delta tracking (AMS-4.6.2) ────────────────────────────────
    // Flat 2D array indexed as [trIdx * AMS_MAX_TRANSITION_CONDS + condIdx].
    // Reset on every state entry so delta is always relative to the state-entry baseline.
    float transitionPrevVal_[ares::AMS_MAX_TRANSITIONS * ares::AMS_MAX_TRANSITION_CONDS]   = {};
    bool  transitionPrevValid_[ares::AMS_MAX_TRANSITIONS * ares::AMS_MAX_TRANSITION_CONDS] = {};

    bool logHeaderWritten_ = false;
    bool logSlotHeaderWritten_[ares::AMS_MAX_HK_SLOTS] = {}; ///< Per-slot CSV header written flags (AMS-4.3.1).
    uint64_t lastCheckpointMs_ = 0;
    bool     checkpointDirty_  = false; ///< True when a material field changed since the last checkpoint write.

    // ── Deferred I/O staging (AMS-8.3) ───────────────────────────────────────
    // Checkpoint writes and CSV log appends are assembled inside the mutex but
    // executed by flushPendingIoUnlocked() after the lock is released.  This
    // keeps blocking LittleFS latency out of the critical section so that
    // injectTcCommand("ABORT") and other API callers cannot timeout on I/O.
    //
    // Worst-case pending appends per tick:
    //   1 header + 1 data row per LOG slot = 2 × AMS_MAX_HK_SLOTS = 8,
    //   plus 1 legacy header + 1 legacy data row = 10 entries total.
    static constexpr uint8_t kMaxPendingAppends =
        static_cast<uint8_t>(ares::AMS_MAX_HK_SLOTS * 2U + 2U);

    struct PendingCheckpoint
    {
        char    buf[512];  ///< Serialised v3 checkpoint record.
        int32_t len;       ///< Valid bytes in buf[]; 0 = nothing staged.
        bool    pending;   ///< True iff buf[] holds an unflushed record.
    };

    struct PendingAppend
    {
        char     path[ares::STORAGE_MAX_PATH]; ///< Target LittleFS path.
        uint8_t  data[256];                    ///< Serialised row bytes.
        uint32_t len;                          ///< Valid bytes in data[].
    };

    PendingCheckpoint pendingCheckpoint_              = {};
    PendingAppend     pendingAppends_[kMaxPendingAppends] = {};
    uint8_t           pendingAppendCount_             = 0U;

    // ── Per-alias sensor read caches ─────────────────────────────────────────
    // When a HK/LOG slot contains multiple fields from the same peripheral,
    // each field would otherwise trigger a separate I2C/UART transaction.
    // The caches below amortise that cost: the first read in a tick stores
    // the full sensor reading; subsequent field requests within the same tick
    // return the cached value with zero hardware I/O.
    //
    // IMU: original per-engine cache (unchanged).
    mutable ImuReading imuCachedReading_ = {};   ///< Last successful IMU burst.
    mutable uint64_t   imuCacheTsMs_     = 0U;   ///< millis64() when last attempt was made.
    mutable bool       imuCacheValid_    = false; ///< true iff imuCachedReading_ holds good data.
    static constexpr uint32_t IMU_CACHE_MAX_AGE_MS  = 5U; ///< Max cache age — one tick period.

    // BARO: per-alias cache (one entry per 'include' alias, indexed by alias slot).
    mutable BaroReading baroCachedReadings_[ares::AMS_MAX_INCLUDES] = {};
    mutable uint64_t    baroCacheTsMs_[ares::AMS_MAX_INCLUDES]      = {};
    mutable bool        baroCacheValid_[ares::AMS_MAX_INCLUDES]      = {};
    static constexpr uint32_t BARO_CACHE_MAX_AGE_MS = 5U;

    // GPS: per-alias cache.  GPS sentences update at 1–10 Hz; a 5 ms TTL
    // amortises redundant UART reads for multi-field HK slots.  The TTL is
    // shorter than SENSOR_RATE_MS so the cache expires between ticks; at
    // normal tick rate, transition conditions therefore read fresh data on
    // their first field access each tick.
    mutable GpsReading  gpsCachedReadings_[ares::AMS_MAX_INCLUDES] = {};
    mutable uint64_t    gpsCacheTsMs_[ares::AMS_MAX_INCLUDES]      = {};
    mutable bool        gpsCacheValid_[ares::AMS_MAX_INCLUDES]      = {};
    static constexpr uint32_t GPS_CACHE_MAX_AGE_MS  = 5U;

    // ── Adaptive tick scheduling ──────────────────────────────────────────────
    /// Maximum time the main loop may sleep even when no sensor condition is
    /// active.  Keeps TC command latency (LAUNCH/ABORT) within 50 ms.
    static constexpr uint32_t kRadioMaxSleepMs = 50U;

    // ── Vertical velocity tracking (APUS-6) ───────────────────────────────────
    // Computed from consecutive baro altitude samples in every HK frame.
    float    prevAltM_       = 0.0f;  ///< Altitude at the previous HK send (m AGL).
    uint64_t prevAltMs_      = 0U;    ///< Timestamp of the previous HK send.
    bool     hasPrevAlt_     = false; ///< True after the first HK altitude sample.

    // ── Telemetry delta-encoding state (APUS-3.3) ─────────────────────────────
    // altitudeAglM and pressurePa are transmitted as deltas from the baseline
    // value.  An absolute re-sync frame is sent at least every
    // kDeltaResyncInterval HK frames, or whenever the delta would overflow the
    // saturation limits.
    static constexpr uint8_t  kDeltaResyncInterval = 10U; ///< Max frames between absolute re-syncs.
    static constexpr float    kMaxDeltaAltM        = 10000.0f; ///< ±10 km saturation limit.
    static constexpr float    kMaxDeltaPressPa     = 100000.0f; ///< ±100 kPa saturation limit.

    uint16_t hkTxCount_      = 0U;    ///< Total HK frames transmitted (modulo counter for delta).
    float    deltaBaseAlt_   = 0.0f;  ///< Altitude of the last absolute/re-sync frame (m AGL).
    float    deltaBasePress_ = 0.0f;  ///< Pressure of the last absolute/re-sync frame (Pa).
    bool     deltaBaseValid_ = false; ///< True once the first absolute frame has been sent.

    // ── ST[12] on-board parameter monitoring (APUS-12) ───────────────────────
    /**
     * Compile-time monitoring definition for one parameter (APUS-12.1).
     * Defines the parameter to monitor, its limit bounds, and the consecutive-
     * violation count required before transitioning to ALARM (APUS-12.2).
     */
    struct MonitoringDefinition
    {
        ares::proto::MonitorParamId parameterId;   ///< Field of TelemetryPayload to check.
        float   lowLimit;                          ///< Below this → violation.
        float   highLimit;                         ///< Above this → violation.
        uint8_t consecutiveRequired;               ///< N violations before ALARM (APUS-12.2).
        bool    enabled;                           ///< false = DISABLED (APUS-12.5).
    };

    /**
     * Runtime state for one monitoring slot: definition + FSM state + counter.
     */
    struct MonitoringSlot
    {
        MonitoringDefinition         def;            ///< Immutable definition (modifiable via API).
        ares::proto::MonitoringState state;          ///< Current FSM state (DISABLED/ENABLED/ALARM).
        uint8_t                      consecutiveHit; ///< Current consecutive violation count.
    };

    static constexpr uint8_t kMaxMonitorSlots = 4U;  ///< Maximum concurrent ST[12] monitoring slots (APUS-12.3).

    /// Default monitoring table (APUS-12.1: declared at compile time).
    /// Ground can override limits via SET_CONFIG_PARAM (APUS-16).
    MonitoringSlot monitorSlots_[kMaxMonitorSlots] = {
        // Slot 0: Altitude AGL — alarm if above 3000 m or below −100 m for 3 samples.
        { { ares::proto::MonitorParamId::ALTITUDE_AGL_M, -100.0f, 3000.0f, 3U, true },
          ares::proto::MonitoringState::MON_ENABLED, 0U },
        // Slot 1: Accel magnitude — alarm if above 150 m/s² for 3 samples.
        { { ares::proto::MonitorParamId::ACCEL_MAG, 0.0f, 150.0f, 3U, true },
          ares::proto::MonitoringState::MON_ENABLED, 0U },
        // Slot 2: Temperature — alarm if above 85 °C or below −40 °C for 5 samples.
        { { ares::proto::MonitorParamId::TEMPERATURE_C, -40.0f, 85.0f, 5U, true },
          ares::proto::MonitoringState::MON_ENABLED, 0U },
        // Slot 3: Vertical velocity — disabled by default (noisy on ground).
        { { ares::proto::MonitorParamId::VERTICAL_VEL_MS, -300.0f, 300.0f, 2U, false },
          ares::proto::MonitoringState::MON_DISABLED, 0U },
    };

    char scriptBuffer_[ares::AMS_MAX_SCRIPT_BYTES + 1U] = {};

    // Current line number during script parsing (1-based).
    // Zero when the engine is not actively parsing.
    // Used by setErrorLocked() to emit "Parse Error (line N): ..." messages.
    uint32_t parseLineNum_ = 0U;

    // ── AMS-11: per-task last-fire timestamps ─────────────────────────────────
    // Indexed by task slot in program_.tasks[].  Reset on deactivate().
    uint64_t taskLastTickMs_[ares::AMS_MAX_TASKS] = {};

    // ── Parser context for task blocks (valid only during parseScriptLocked) ──
    uint8_t  parseCurrentTask_     = 0xFFU; ///< Index of task being parsed; 0xFF = none.
    uint8_t  parseCurrentTaskRule_ = 0xFFU; ///< Index of current if-rule within task; 0xFF = none.

    // ── Parser context for multi-slot every/log_every (AMS-4.3.1) ────────────
    uint8_t  parseCurrentHkSlot_  = 0xFFU; ///< Index of HK slot being parsed; 0xFF = none.
    uint8_t  parseCurrentLogSlot_ = 0xFFU; ///< Index of LOG slot being parsed; 0xFF = none.

    // ── Metadata-order guard (AMS-4.2) ───────────────────────────────────────
    // Set to true the moment the first state: keyword is parsed.  Any metadata
    // directive (include, var, const, pus.*, radio.config) seen after this
    // point is a parse error.
    bool     parseSeenState_      = false;
};

} // namespace ams
} // namespace ares

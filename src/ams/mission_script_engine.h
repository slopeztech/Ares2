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

#include "ams/mission_script_engine_internal.h"
#include "ams/mission_script_engine_types.h"

namespace ares
{
namespace ams
{


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
    ~MissionScriptEngine();

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
     * @param[in] channel  Channel index 0–3 (PulseChannel::CH_A – CH_D).
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

    bool loadFromStorageLocked(const char* fileName);
    bool parseScriptLocked(const char* script, uint32_t length);
    void initProgramParseStateLocked();                               ///< Reset program_ and parse state variables; called from parseScriptLocked.
    bool finalizeScriptLocked();                                      ///< Post-parse resolve + validate + warn; called from parseScriptLocked.
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
    bool parsePreStateDirectiveLocked(const char* line, bool& handled);  ///< Handle include/radio.config/pus.*/var/const before any state block.
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
    bool commitEverySlotLocked(StateDef&   st,
                               const char* valueStart,
                               const char* marker,
                               const char* comAlias);
    bool resolveEveryMarkerLocked(const char* valueStart,
                                  const char** outMarker,
                                  char*        outAlias,
                                  size_t       aliasSize);
    bool parseEveryViaAliasLocked(const char* rawAliasStart,
                                  char*       outAlias,
                                  size_t      aliasSize);
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
    bool storeVarFieldLocked(const char* expr, const char* key,
                             HkField* fields, uint8_t& count,
                             const char* ctxName);                    ///< Store a variable-reference HK/LOG field entry (AMS-4.8).
    bool storeAliasFieldLocked(const char* aliasStr, const char* fieldStr,
                               const char* key, HkField* fields,
                               uint8_t& count, const char* ctxName); ///< Store an ALIAS.field HK/LOG field entry.
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
    bool parsePulseFireLineLocked(const char* line, StateDef& st);            ///< AMS-4.17: parse "PULSE.fire A/B/C/D/ALIAS [Nms]".
    bool parsePulseDurationSuffixLocked(const char* afterCh, uint32_t& out); ///< Parse optional " Nms" suffix; called by parsePulseFireLineLocked.
    bool parsePulseChannelLineLocked(const char* line);                       ///< AMS-4.18: parse "pulse.channel A/B/C/D [as LABEL]".
    bool parsePulseArmLineLocked(const char* line, StateDef& st);                   ///< AMS-4.19.1: parse "PULSE.arm A/B/C/D/ALIAS".
    bool parsePulseRequireContinuityLineLocked(const char* line);                   ///< AMS-4.19.4: parse "pulse.require_continuity A/B/C/D".
    bool parsePulseMinAltLineLocked(const char* line);                              ///< AMS-4.19.2: parse "pulse.min_altitude N".
    bool parsePulseSafeDelayLineLocked(const char* line);                           ///< AMS-4.19.5: parse "pulse.safe_delay N".
    bool parsePulseArmTimeoutLineLocked(const char* line);                          ///< AMS-4.19.3: parse "pulse.arm_timeout N".
    bool parsePulseNoBaroPolicyLineLocked(const char* line);                        ///< AMS-4.19.6: parse "pulse.no_baro_policy allow|block".
    uint8_t resolveChannelOrAliasLocked(const char* token) const;             ///< Resolve channel letter or label -> 0-3; returns 0xFF on failure.
    bool parseSetActionCoreLocked(const char* line, SetAction& out);  ///< Shared core; called by state + task parsers.
    bool ensureSetVariableExistsLocked(const char* varName);
    bool parseCalibrateSetActionLocked(const char* rhsBuf, SetAction& out);
    bool parseMinMaxSetActionLocked(const char* rhsBuf,
                                    const char* varName,
                                    SetAction&  out);
    bool parseDeltaSetActionLocked(const char* rhsBuf, SetAction& out);
    bool parseExprSetActionLocked(const char* rhsBuf, SetAction& out);      ///< AMS-4.8.8: parse arithmetic expression RHS.
    bool parseExprTermLocked(const char* token, ExprOperand& out);          ///< Resolve one expression token to an ExprOperand.
    bool buildExprOpsLocked(const char tokens[][32], uint8_t termCount, const ExprOperand terms[], ExprOp ops[]); ///< AMS-4.8.8: map operator tokens → ExprOp[]; guard literal ÷0.
    bool parseSimpleSensorSetActionLocked(const char* rhsBuf, SetAction& out);
    bool evaluateExprOperandLocked(const ExprOperand& op, float& value) const; ///< AMS-4.8.8: resolve one operand to float at runtime.
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
    bool resolveTargetNameLocked(const char* name, const char* kindStr,
                                  uint8_t& outIdx);                   ///< Find state by name, emit error with suggestion on failure.
    bool resolveTasksLocked();        ///< AMS-11: resolve 'when in' state names to indices after parsing.
    bool validateAssertionsLocked();  ///< AMS-15: run BFS/DFS graph checks; call from parseScriptLocked.
    void warnShadowedTransitionsLocked(uint64_t nowMs); ///< AMS-5.1: LOG_W + EVENT.info for unreachable transitions.
    void checkTransitionPairLocked(const StateDef& st, uint8_t i, const CondExpr& ci,
                                    uint8_t j, uint64_t nowMs);       ///< Test one (i,j) pair for shadowing; emits warning if shadowed.
    void emitShadowWarningLocked(const StateDef& st, uint8_t i, uint8_t j,
                                  const CondExpr& ci, const CondExpr& cj,
                                  uint64_t nowMs);                    ///< Emit LOG_W + EVENT for one shadowed transition pair.
    void computeBfsReachabilityLocked(uint32_t& reachable) const;
    void computeDfsMaxDepthLocked(uint8_t& maxDepth, bool& hasCycle) const;
    bool evaluateOneAssertionLocked(const AssertDef& ad, uint32_t reachable,
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
    bool refreshImuCacheLocked(ImuInterface* imu, uint8_t aliasIdx, uint8_t maxAttempts) const;
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
                                              size_t          condIdx,
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
    void executePulseActionsLocked(const StateDef& st, uint64_t nowMs);    ///< AMS-4.17 + AMS-4.19: fire all PULSE.fire actions, safety-gated.
    void executePulseArmActionsLocked(const StateDef& st, uint64_t nowMs); ///< AMS-4.19.1: set arm flags for PULSE.arm actions.
    void checkPulseArmTimeoutsLocked(uint64_t nowMs);                       ///< AMS-4.19.3: disarm channels whose arm_timeout has expired.
    bool checkPulseSafetyLocked(uint8_t channel, uint64_t nowMs);           ///< AMS-4.19: evaluate all safety gates; returns true if fire is allowed.
    bool checkPulseAltGateLocked(uint8_t channel);                          ///< AMS-4.19.2/4.19.6: evaluate altitude gate; extracted for size.
    bool parsePulsePreStateDirectiveLocked(const char* line, bool& handled); ///< Pulse-specific sub-dispatch; extracted from parsePreStateDirectiveLocked.
    bool parsePulseChannelLabelLocked(const char* rest, char* labelOut);    ///< Parse/validate optional 'as LABEL' suffix; extracted from parsePulseChannelLineLocked.
    void executeOneSetActionLocked(SetAction& act, uint64_t nowMs); ///< Execute a single set action (shared by state + task paths).
    void stepPendingCalibrationsLocked(StateDef& st, uint64_t nowMs); ///< AMS-4.8.2: advance each in-progress CALIBRATE by one sample.
    void executeCalibrateSetActionLocked(SetAction& act, float& result, bool& gotReading, uint64_t nowMs);
    void executeDeltaSetActionLocked(SetAction& act, float& result, bool& gotReading);
    void executeMinMaxSetActionLocked(SetAction& act, const VarEntry* v, float& result, bool& gotReading);
    void executeExprSetActionLocked(SetAction& act, float& result, bool& gotReading, uint64_t nowMs); ///< AMS-4.8.8: evaluate arithmetic expression.
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
    bool applyGuardErrorLocked(const StateDef& state, uint64_t nowMs,
                               const CondExpr& firstExpr, float firstActualVal);
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
    void    finaliseHkPayloadLocked(ares::proto::TelemetryPayload& tm, uint64_t nowMs);
    void    transmitHkFrameLocked(const ares::proto::TelemetryPayload& tm, const char* logLabel,
                                  RadioInterface* com = nullptr);

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

    bool sendFrameLocked(const ares::proto::Frame& frame, RadioInterface* com = nullptr);

    /// Stage a log-file append for deferred execution outside the mutex (AMS-8.3).
    void queueAppendLocked(const char* path, const uint8_t* data, uint32_t len,
                             bool* onSuccessFlag = nullptr);
    /// Flush all staged file I/O that was built under the mutex. Must be called
    /// after the mutex is released. Idempotent if nothing is pending.
    void flushPendingIoUnlocked();

    bool saveResumePointLocked(uint64_t nowMs, bool force);
    bool buildCheckpointRecordLocked(uint64_t nowMs, char* record, size_t recSize, int32_t& outWritten) const;
    bool appendVarsSectionLocked(char* record, size_t recSize, int32_t& written) const;
    bool appendSlotTimersSectionLocked(char* record, size_t recSize, int32_t& written, uint64_t nowMs) const;
    bool appendConfirmHoldSectionLocked(char* record, size_t recSize, int32_t& written, uint64_t nowMs) const;
    bool tryRestoreResumePointLocked(uint64_t nowMs);
    bool applyCheckpointStateLocked(uint64_t nowMs, uint32_t stateIdx, uint32_t execEnabled,
                                     uint32_t running, uint32_t status, uint32_t seq,
                                     uint32_t stateElapsed, uint32_t hkElapsed,
                                     uint32_t logElapsed);
    static bool parseCheckpointHeaderLocked(const char* buf, uint32_t& version, char* fileName, uint32_t& stateIdx, uint32_t& execEnabled, uint32_t& running, uint32_t& status, uint32_t& seq, uint32_t& stateElapsed, uint32_t& hkElapsed, uint32_t& logElapsed);
    const char* restoreCheckpointVarsLocked(const char* cursor);
    const char* restoreCheckpointSlotsLocked(const char* cursor, uint64_t nowMs);
    bool        restoreCheckpointV4StrictLocked(const char* cursor, uint64_t nowMs);
    void        restoreCheckpointV4FieldsLocked(const char* cursor, uint64_t nowMs);
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
    bool      pulseCFired_ = false;    ///< True after FIRE_PULSE_C was successfully executed (APUS-6).
    bool      pulseDFired_ = false;    ///< True after FIRE_PULSE_D was successfully executed (APUS-6).
    bool      pulseChannelUsed_[PulseChannel::COUNT] = {}; ///< Tracks which declared channels had at least one PULSE.fire (AMS-4.18).

    // ── AMS-4.19: Pulse safety runtime state ─────────────────────────────────
    bool     pulseArmed_[PulseChannel::COUNT]   = {};  ///< AMS-4.19.1: per-channel arm flag (set by PULSE.arm, cleared on fire or timeout).
    uint64_t pulseArmedMs_[PulseChannel::COUNT] = {};  ///< AMS-4.19.3: millis64() when each channel was last armed.
    uint64_t activationMs_                      = 0U;  ///< AMS-4.19.5: millis64() when arm() put the engine into RUNNING state.
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
    // See mission_script_engine_internal.h for PendingCheckpoint, PendingAppend,
    // and kMaxPendingAppends.
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
    // IMU: per-alias cache (one entry per 'include' alias, indexed by alias
    // slot).  A single shared slot would otherwise cross-contaminate readings
    // when a script declares two IMUs (e.g. a standard MPU6050 plus a high-g
    // ADXL375): the second alias would receive the first sensor's data while
    // the shared slot was still within its TTL.
    mutable ImuReading imuCachedReadings_[ares::AMS_MAX_INCLUDES] = {};   ///< Last successful IMU burst, per alias.
    mutable uint64_t   imuCacheTsMs_[ares::AMS_MAX_INCLUDES]      = {};   ///< millis64() of last attempt, per alias.
    mutable bool       imuCacheValid_[ares::AMS_MAX_INCLUDES]     = {};   ///< true iff imuCachedReadings_[i] holds good data.
    /// Cache age limit: reuse reading if age < AMS_SENSOR_CACHE_TTL_MS (config.h).
    static constexpr uint32_t IMU_CACHE_MAX_AGE_MS  = ares::AMS_SENSOR_CACHE_TTL_MS;

    // BARO: per-alias cache (one entry per 'include' alias, indexed by alias slot).
    mutable BaroReading baroCachedReadings_[ares::AMS_MAX_INCLUDES] = {};
    mutable uint64_t    baroCacheTsMs_[ares::AMS_MAX_INCLUDES]      = {};
    mutable bool        baroCacheValid_[ares::AMS_MAX_INCLUDES]      = {};
    static constexpr uint32_t BARO_CACHE_MAX_AGE_MS = ares::AMS_SENSOR_CACHE_TTL_MS; ///< Cache TTL for BARO readings; reuse if age < this value (ms).

    // GPS: per-alias cache.  GPS sentences update at 1–10 Hz; a short TTL
    // (AMS_SENSOR_CACHE_TTL_MS, config.h) amortises redundant UART reads for
    // multi-field HK slots.  The TTL is shorter than SENSOR_RATE_MS so the
    // cache expires between ticks; at normal tick rate, transition conditions
    // therefore read fresh data on their first field access each tick.
    mutable GpsReading  gpsCachedReadings_[ares::AMS_MAX_INCLUDES] = {};
    mutable uint64_t    gpsCacheTsMs_[ares::AMS_MAX_INCLUDES]      = {};
    mutable bool        gpsCacheValid_[ares::AMS_MAX_INCLUDES]      = {};
    static constexpr uint32_t GPS_CACHE_MAX_AGE_MS  = ares::AMS_SENSOR_CACHE_TTL_MS;  ///< Cache TTL for GPS readings; reuse if age < this value (ms).

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
    // See mission_script_engine_internal.h for MonitoringDefinition,
    // MonitoringSlot, and kMaxMonitorSlots.

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
    // Used by setErrorLocked() to emit "Parse Error (FILE, line N): ..." messages.
    uint32_t parseLineNum_ = 0U;

    // Basename of the file currently being parsed (e.g. "flight.ams").
    // Empty string when the engine is not actively parsing a named source.
    // Set in loadFromStorageLocked() before parseScriptLocked() is called.
    // Cleared by setErrorLocked() (error path) and by parseScriptLocked()
    // at the point where parseLineNum_ is cleared (success path).
    //
    // NOTE FOR FUTURE IMPORTS: when "include <file>" is added, push/pop
    // parseSourceFile_ and parseLineNum_ onto a small stack (depth-limited
    // by AMS_MAX_IMPORT_DEPTH) so nested files report their own filename
    // and line number correctly.  The public error format must remain
    // "Parse Error (FILE, line N): reason" — only FILE and N change.
    char parseSourceFile_[ares::MISSION_FILENAME_MAX] = {};

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
    bool     begun_               = false;
};

} // namespace ams
} // namespace ares

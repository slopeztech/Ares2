/**
 * @file  mission_script_engine_types.h
 * @brief AMS parser and data-model type definitions.
 *
 * Contains all enum and struct types used by the AMS script engine parser
 * and data model: EngineStatus, TcCommand, EngineSnapshot (public API types),
 * and all internal types (BlockType, CondExpr, StateDef, Program, etc.).
 *
 * These types were formerly either public types or private nested types of
 * @c MissionScriptEngine; they now reside at the @c ares::ams namespace level
 * so that the engine's multiple translation units can include this header
 * independently of the full class declaration.
 *
 * Design constraints:
 *   - No dynamic allocation (PO10-3)
 *   - All array sizes bounded by @c ares::AMS_* compile-time constants (PO10-2)
 */
#pragma once

#include <cstdint>

#include "comms/ares_radio_protocol.h"
#include "config.h"
#include "hal/pulse/pulse_interface.h"

namespace ares
{
namespace ams
{

// ─────────────────────────────────────────────────────────────────────────────
// Public API types (formerly at namespace level in mission_script_engine.h)
// ─────────────────────────────────────────────────────────────────────────────

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
// Sentinel consistency guards — fail at compile time if a new enumerator is
// inserted before IDLE or after LOADED without updating FIRST / LAST.
static_assert(static_cast<uint8_t>(EngineStatus::FIRST) == 0U,
              "EngineStatus::FIRST must alias the lowest enumerator (IDLE = 0)");
static_assert(static_cast<uint8_t>(EngineStatus::LAST)  == 4U,
              "EngineStatus::LAST must alias the highest enumerator (LOADED = 4)");

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

// ─────────────────────────────────────────────────────────────────────────────
// Internal parser / data-model types (formerly private nested in the class)
// ─────────────────────────────────────────────────────────────────────────────

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

// ── Peripheral kind ──────────────────────────────────────────────────────────
/** Sensor peripheral category for alias resolution in AMS scripts. */
enum class PeripheralKind : uint8_t
{
    GPS  = 0,
    BARO = 1,
    COM  = 2,
    IMU  = 3,
};

// ── Sensor field within a peripheral ─────────────────────────────────────────
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
    GYRO_MAG  = 16, ///< IMU: ||gyro|| (deg/s) — angular rate magnitude √(gx²+gy²+gz²)
    VAR       = 17, ///< AMS variable — alias holds the variable name (AMS-4.8)
};

// ── Unified condition kind ─────────────────────────────────────────────
/** Discriminant for a unified CondExpr — sensor comparison, TC match, or elapsed time. */
enum class CondKind : uint8_t
{
    NONE            = 0,
    SENSOR_LT       = 1,  ///< alias.field < threshold
    SENSOR_GT       = 2,  ///< alias.field > threshold
    TC_EQ           = 3,  ///< TC.command == value
    TIME_GT         = 4,  ///< TIME.elapsed > threshold (ms since state entry)
    SENSOR_DELTA_LT = 5,  ///< (alias.field[n] - alias.field[n-1]) < threshold (AMS-4.6.2)
    SENSOR_DELTA_GT = 6,  ///< (alias.field[n] - alias.field[n-1]) > threshold (AMS-4.6.2)
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
    bool  useVar  = false;                      ///< true = compare against (varValue + varOffset).
    char  varName[ares::AMS_VAR_NAME_LEN] = {}; ///< Variable name (used when useVar).
    float varOffset = 0.0f;                     ///< Additive offset: compare against var+offset.
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

// ── AMS-15: Formal validation assertions ──────────────────────────────────────

/**
 * Kind of a formal assertion evaluated at load time (AMS-15).
 */
enum class AssertKind : uint8_t
{
    REACHABLE           = 0U, ///< assert reachable STATE — BFS reachability from initial state.
    NO_DEAD_STATES      = 1U, ///< assert no_dead_states — all states are reachable from initial.
    MAX_DEPTH           = 2U, ///< assert max_transition_depth < N — longest simple path < N.
    NO_SILENT_TERMINALS = 3U, ///< assert no_silent_terminals — every state has ≥1 exit.
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
    EXPR      = 5U, ///< var = TERM op TERM [op TERM]            (AMS-4.8.8)
};

/** Binary operator for an arithmetic set-expression (AMS-4.8.8). */
enum class ExprOp : uint8_t
{
    ADD = 0U, ///< Addition (+).
    SUB = 1U, ///< Subtraction (-).
    MUL = 2U, ///< Multiplication (*).
    DIV = 3U, ///< Division (/).
};

/**
 * @brief One value-producing operand in an AMS-4.8.8 arithmetic expression.
 *
 * The @c name field doubles as the peripheral alias (SENSOR kind) or the
 * declared variable name (VARIABLE kind); @c literal is used only for LITERAL.
 */
struct ExprOperand
{
    /** Source kind of this operand. */
    enum class Kind : uint8_t
    {
        SENSOR   = 0U, ///< Read ALIAS.field from a peripheral at execute time.
        VARIABLE = 1U, ///< Read a declared global variable at execute time.
        LITERAL  = 2U, ///< Fixed @c float constant resolved at parse time.
    };

    Kind        kind    = Kind::LITERAL;             ///< Operand source.
    char        name[ares::AMS_VAR_NAME_LEN] = {};  ///< SENSOR: alias; VARIABLE: variable name.
    SensorField field   = SensorField::ALT;          ///< SENSOR only: field selector.
    float       literal = 0.0f;                      ///< LITERAL only: constant value.
};

/**
 * A single @c set action inside an @c on_enter: block (AMS-4.8).
 * Captures a sensor reading (or derived value) into a global variable.
 */
struct SetAction
{
    char          varName[ares::AMS_VAR_NAME_LEN] = {};  ///< Target variable name.
    char          alias[16]      = {};                    ///< Sensor peripheral alias (SIMPLE/CALIBRATE/DELTA/MAX_VAR/MIN_VAR).
    SensorField   field          = SensorField::ALT;      ///< Sensor field to read.
    SetActionKind kind           = SetActionKind::SIMPLE; ///< Computation kind.
    uint8_t       calibSamples   = 1U;    ///< Samples to average (CALIBRATE form, 1–AMS_CALIBRATE_MAX_SAMPLES).
    float         deltaBaseline  = 0.0f;  ///< Previous sensor value (DELTA form).
    bool          deltaValid     = false; ///< Whether deltaBaseline has been set.

    // ── Async CALIBRATE progress (AMS-4.8.2) ─────────────────────────────────
    // Reset by enterStateLocked() on every state entry; advanced one sample
    // per tick by stepPendingCalibrationsLocked() until all calibSamples are
    // collected.  Non-CALIBRATE set actions leave these fields unused.
    float   calibSum        = 0.0f;   ///< Accumulated sum of valid samples.
    uint8_t calibValidN     = 0U;     ///< Valid (non-failed) samples collected.
    uint8_t calibCollected  = 0U;     ///< Total sample attempts so far.
    bool    calibInProgress = false;  ///< True while async calibration is running.

    // ── AMS-4.8.8: arithmetic expression ─────────────────────────────────────
    // Used when kind == EXPR.  Up to kMaxExprTerms value-producing operands
    // connected by (kMaxExprTerms - 1) binary operators, evaluated strictly
    // left-to-right:  result = ((terms[0] op[0] terms[1]) op[1] terms[2]).
    static constexpr uint8_t kMaxExprTerms = 3U;         ///< Supports up to (A op B) op C.
    ExprOperand exprTerms[kMaxExprTerms]     = {};        ///< Operands (first exprTermCount entries are valid).
    ExprOp      exprOps[kMaxExprTerms - 1U]  = {};        ///< Operators between consecutive terms.
    uint8_t     exprTermCount                = 0U;        ///< Actual number of terms (2 or 3).
};

/** Severity level for AMS on_enter and task rule events (AMS-4.7). */
enum class EventVerb : uint8_t
{
    INFO  = 0,
    WARN  = 1,
    ERROR = 2,
};

// ── AMS-11: Parallel background tasks ────────────────────────────────────────

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
    char        label[20] = {};  ///< User-provided key (CSV column name).
                                 ///< Must not contain ',' or '"'; any such characters
                                 ///< are replaced with '_' when the CSV header is written.
    char        alias[16] = {};  ///< Peripheral alias (e.g. "GPS") or variable name
                                 ///< when @c field == SensorField::VAR (AMS-4.8).
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
    uint32_t everyMs      = 0U;                            ///< Report interval (ms); 0 = unused.
    uint8_t  fieldCount   = 0U;                            ///< Number of populated fields.
    char     comAlias[16] = {};                            ///< Target COM alias; empty = use primaryCom_.
    HkField  fields[ares::AMS_MAX_HK_FIELDS] = {};        ///< Field descriptors.
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
 * Created by 'include <MODEL> as <ALIAS> [retry=N]' (AMS-4.9.1).
 */
struct AliasEntry
{
    char           alias[16]   = {};    ///< e.g. "GPS", "GPS1", "BARO"
    char           model[16]   = {};    ///< e.g. "BN220", "BMP280"
    PeripheralKind kind        = PeripheralKind::GPS;
    uint8_t        driverIdx   = 0xFFU; ///< Index in registry; 0xFF = not resolved
    uint8_t        retryCount  = 0U;    ///< Extra read attempts on failure (0 = no retry, max AMS_MAX_SENSOR_RETRY).
};

/// A single PULSE.fire command parsed from an on_enter: block (AMS-4.17).
struct PulseAction
{
    uint8_t  channel;      ///< PulseChannel::CH_A (0), CH_B (1), CH_C (2), or CH_D (3).
    uint32_t durationMs;   ///< Pulse duration; 0 means use config default.
};

/// Arms a channel so that a subsequent PULSE.fire in another state is
/// permitted (two-phase firing: arm then fire) (AMS-4.19.1).
struct PulseArmAction
{
    uint8_t channel;  ///< PulseChannel::CH_A (0) … CH_D (3).
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
    bool    prioritiesSet = false; ///< True once a priorities directive has been parsed.

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
    uint8_t      pulseActionCount = 0U;
    PulseAction  pulseActions[ares::AMS_MAX_PULSE_ACTIONS] = {};

    // ── on_enter PULSE.arm actions (AMS-4.19.1) ──────────
    uint8_t        pulseArmActionCount = 0U;
    PulseArmAction pulseArmActions[ares::AMS_MAX_PULSE_ACTIONS] = {};

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

// ── AMS-4.18: pulse channel declarations ──────────────────────────────────────

/**
 * Declaration record for one pulse channel (AMS-4.18).
 *
 * Set by @c pulse.channel X [as LABEL] in the script metadata section.
 * Until @c declared is @c true, any @c PULSE.fire targeting this channel
 * (by letter or alias) is rejected at parse time.
 */
struct PulseChannelDecl
{
    bool declared = false;                        ///< true once 'pulse.channel X' is parsed.
    char label[ares::AMS_PULSE_LABEL_LEN] = {};  ///< User label ("A".."D" if no 'as' clause).
};

// ── AMS-4.15: script-declared radio config overrides ─────────────────────────

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

    // ── AMS-4.18: pulse channel declarations ──────────────────────────────────
    PulseChannelDecl pulseDecls[PulseChannel::COUNT] = {}; ///< Declarations indexed by channel (0=A, 1=B, 2=C, 3=D).

    // ── AMS-4.19: Pulse safety directives ────────────────────────────────────
    uint32_t pulseSafeDelayMs                            = 0U;    ///< AMS-4.19.5: ms from arm() before any PULSE.fire is allowed (0 = disabled).
    bool     pulseHasMinAlt                              = false;  ///< AMS-4.19.2: true when a pulse.min_altitude guard is active.
    uint32_t pulseMinAltitudeM                           = 0U;    ///< AMS-4.19.2: minimum baro altitude (m MSL) required for PULSE.fire.
    bool     pulseNoBaroPolicyAllow                      = false; ///< AMS-4.19.6: when true, altitude gate is bypassed if no baro driver registered.
    bool     pulseHasNoBaroPolicy                        = false; ///< AMS-4.19.6: true once "pulse.no_baro_policy" has been parsed (duplicate guard).
    bool     pulseRequireContinuity[PulseChannel::COUNT] = {};    ///< AMS-4.19.4: channel requires intact bridgewire before fire.
    bool     pulseNeedsArm[PulseChannel::COUNT]          = {};    ///< AMS-4.19.1: channel requires prior PULSE.arm in on_enter.
    uint32_t pulseArmTimeoutMs                           = 0U;    ///< AMS-4.19.3: ms after PULSE.arm before auto-disarm (0 = no timeout).

    // ── AMS-4.15: script-declared radio config overrides ─────────────────────
    // ARES-MISRA-DEV-002: outer static_cast<uint8_t> suppresses integral promotion
    // from uint8_t arithmetic; result is [0,6] and always fits in uint8_t.
    static constexpr uint8_t kRadioConfigCount =
        static_cast<uint8_t>(
            static_cast<uint8_t>(ares::proto::ConfigParamId::LAST) -
            static_cast<uint8_t>(ares::proto::ConfigParamId::FIRST) + 1U);
    RadioConfigParam radioConfig[kRadioConfigCount] = {};  ///< Script radio.config overrides (AMS-4.15), indexed by (ConfigParamId - FIRST).
};

} // namespace ams
} // namespace ares

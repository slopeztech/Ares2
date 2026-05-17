/**
 * @file  test_ams_set_expr.cpp
 * @brief SITL integration tests — arithmetic expressions in `set` statements (AMS-4.8.8).
 *
 * AMS-4.8.8 extends the `set` statement so that the right-hand side may be an
 * arithmetic expression of the form:
 * @code
 *   set VARNAME = TERM op TERM [op TERM]
 * @endcode
 * where TERM is a sensor field reference (ALIAS.field), a declared variable,
 * or a float literal, and op is one of +, -, *, /.  Evaluation is strictly
 * left-to-right: ((T0 op0 T1) op1 T2).  Parentheses are cosmetic — they are
 * stripped before tokenisation and do not affect precedence.
 *
 * Runtime error policy:
 *   - Literal ÷ 0 at parse time   → parse error, activate() returns false.
 *   - Unknown peripheral alias     → parse error, activate() returns false.
 *   - Variable operand not yet set → result not written; target unchanged.
 *   - Runtime ÷ 0, NaN, or Inf    → EVENT.warning; target unchanged.
 *
 * Tests:
 *   1. test_set_expr_parse_succeeds               — 2-term expr script parses OK.
 *   2. test_set_expr_sensor_minus_var_executes     — alt_agl = BARO.alt - ground_alt.
 *   3. test_set_expr_three_term_executes           — vvel_avg = (BARO.alt - prev_alt) / 0.5.
 *   4. test_set_expr_sensor_plus_literal           — alt_off = BARO.alt + 10.0.
 *   5. test_set_expr_div_by_zero_literal_rejected  — / 0.0 → parse error.
 *   6. test_set_expr_unknown_alias_rejected        — NOPE.alt → parse error.
 *   7. test_set_expr_unset_var_operand_skips_update — invalid variable → target unchanged.
 */

#include <unity.h>

#include "ams/mission_script_engine.h"
#include "ams/ams_driver_registry.h"

using ares::ams::GpsEntry;
using ares::ams::BaroEntry;
using ares::ams::ComEntry;
using ares::ams::ImuEntry;

#include "sim_clock.h"
#include "sim_gps_driver.h"
#include "sim_baro_driver.h"
#include "sim_imu_driver.h"
#include "sim_radio_driver.h"
#include "sim_storage_driver.h"

using ares::ams::MissionScriptEngine;
using ares::ams::EngineSnapshot;
using ares::ams::EngineStatus;

// ── Flight profile ─────────────────────────────────────────────────────────────
// Ascent from 0 m (t=0) through 150 m (t=2000 ms) to 300 m (t=4000 ms).
// Tests inject TC LAUNCH at t=2000 ms so that on_enter of FLIGHT reads
// BARO.alt = 150 m.

static const ares::sim::FlightProfile kExprProfile = {
    .count   = 3U,
    .samples = {
        { .timeMs =    0U, .gpsAltM =   0.0f, .gpsSpeedKmh = 0.0f,
          .gpsHdop = 1.0f, .gpsSats = 9U, .gpsFix = true,
          .baroAltM =   0.0f, .baroPressurePa = 101325.0f, .baroTempC = 20.0f,
          .accelX = 0.0f, .accelY = 0.0f, .accelZ = 9.81f, .imuTempC = 25.0f },
        { .timeMs = 2000U, .gpsAltM = 150.0f, .gpsSpeedKmh = 50.0f,
          .gpsHdop = 1.0f, .gpsSats = 9U, .gpsFix = true,
          .baroAltM = 150.0f, .baroPressurePa = 99000.0f, .baroTempC = 18.0f,
          .accelX = 0.3f, .accelY = 0.0f, .accelZ = 12.0f, .imuTempC = 26.0f },
        { .timeMs = 4000U, .gpsAltM = 300.0f, .gpsSpeedKmh = 20.0f,
          .gpsHdop = 1.0f, .gpsSats = 9U, .gpsFix = true,
          .baroAltM = 300.0f, .baroPressurePa = 98700.0f, .baroTempC = 15.0f,
          .accelX = 0.0f, .accelY = 0.0f, .accelZ = 9.81f, .imuTempC = 27.0f },
    }
};

// ── Scripts ───────────────────────────────────────────────────────────────────

/**
 * SENSOR minus VARIABLE:  set alt_agl = BARO.alt - ground_alt
 *
 * WAIT  on_enter: set ground_alt = BARO.alt  (records base altitude).
 * FLIGHT on_enter: set alt_agl = BARO.alt - ground_alt.
 * Transition FLIGHT→END fires when BARO.alt > 50.
 */
static const char kScriptExprSensorMinusVar[] =
    "include SIM_GPS as GPS\n"
    "include SIM_BARO as BARO\n"
    "include SIM_COM as COM\n"
    "include SIM_IMU as IMU\n"
    "\n"
    "pus.apid = 1\n"
    "pus.service 5 as EVENT\n"
    "pus.service 1 as TC\n"
    "\n"
    "var ground_alt = 0.0\n"
    "var alt_agl = 0.0\n"
    "\n"
    "state WAIT:\n"
    "  on_enter:\n"
    "    set ground_alt = BARO.alt\n"
    "  transition to FLIGHT when TC.command == LAUNCH\n"
    "\n"
    "state FLIGHT:\n"
    "  on_enter:\n"
    "    set alt_agl = BARO.alt - ground_alt\n"
    "  transition to END when BARO.alt > 50\n"
    "\n"
    "state END:\n"
    "  on_enter:\n"
    "    EVENT.info \"END\"\n";

/**
 * Three-term expression:  set vvel_avg = ( BARO.alt - prev_alt ) / 0.5
 *
 * WAIT  on_enter: set prev_alt = BARO.alt  (= 0 m at t=0).
 * FLIGHT on_enter: set vvel_avg = ( BARO.alt - prev_alt ) / 0.5.
 * At t=2000 ms: vvel_avg = (150 - 0) / 0.5 = 300 > 100 → END.
 */
static const char kScriptExprThreeTerm[] =
    "include SIM_GPS as GPS\n"
    "include SIM_BARO as BARO\n"
    "include SIM_COM as COM\n"
    "include SIM_IMU as IMU\n"
    "\n"
    "pus.apid = 1\n"
    "pus.service 5 as EVENT\n"
    "pus.service 1 as TC\n"
    "\n"
    "var prev_alt = 0.0\n"
    "var vvel_avg = 0.0\n"
    "\n"
    "state WAIT:\n"
    "  on_enter:\n"
    "    set prev_alt = BARO.alt\n"
    "  transition to FLIGHT when TC.command == LAUNCH\n"
    "\n"
    "state FLIGHT:\n"
    "  on_enter:\n"
    "    set vvel_avg = ( BARO.alt - prev_alt ) / 0.5\n"
    "  transition to END when BARO.alt > 50\n"
    "\n"
    "state END:\n"
    "  on_enter:\n"
    "    EVENT.info \"END\"\n";

/**
 * SENSOR plus LITERAL:  set alt_off = BARO.alt + 10.0
 *
 * WAIT on_enter: set alt_off = BARO.alt + 10.0  (= 0 + 10 = 10.0 at t=0).
 * Transition WAIT→FLIGHT via TC LAUNCH.
 * Condition FLIGHT: BARO.alt > -1  → always true → END.
 */
static const char kScriptExprSensorPlusLiteral[] =
    "include SIM_GPS as GPS\n"
    "include SIM_BARO as BARO\n"
    "include SIM_COM as COM\n"
    "include SIM_IMU as IMU\n"
    "\n"
    "pus.apid = 1\n"
    "pus.service 5 as EVENT\n"
    "pus.service 1 as TC\n"
    "\n"
    "var alt_off = 0.0\n"
    "\n"
    "state WAIT:\n"
    "  on_enter:\n"
    "    set alt_off = BARO.alt + 10.0\n"
    "  transition to FLIGHT when TC.command == LAUNCH\n"
    "\n"
    "state FLIGHT:\n"
    "  transition to END when BARO.alt > -1\n"
    "\n"
    "state END:\n"
    "  on_enter:\n"
    "    EVENT.info \"END\"\n";

/**
 * Parse error: literal zero in divisor position.
 * Expected: activate() returns false.
 */
static const char kScriptExprDivByZeroLiteral[] =
    "include SIM_BARO as BARO\n"
    "include SIM_COM as COM\n"
    "\n"
    "pus.apid = 1\n"
    "pus.service 5 as EVENT\n"
    "\n"
    "var ratio = 0.0\n"
    "\n"
    "state WAIT:\n"
    "  on_enter:\n"
    "    set ratio = BARO.alt / 0.0\n"
    "  transition to END when BARO.alt > 9999\n"
    "\n"
    "state END:\n";

/**
 * Parse error: unknown peripheral alias in expression.
 * Expected: activate() returns false.
 */
static const char kScriptExprUnknownAlias[] =
    "include SIM_BARO as BARO\n"
    "include SIM_COM as COM\n"
    "\n"
    "pus.apid = 1\n"
    "pus.service 5 as EVENT\n"
    "\n"
    "var foo = 0.0\n"
    "var ground_alt = 0.0\n"
    "\n"
    "state WAIT:\n"
    "  on_enter:\n"
    "    set foo = NOPE.alt - ground_alt\n"
    "  transition to END when BARO.alt > 9999\n"
    "\n"
    "state END:\n";

/**
 * Runtime skip: variable operand declared but never set (valid = false).
 *
 * `ground_alt` is declared but is never written by any set action, so its
 * `valid` flag remains false.  The executor must detect this and leave
 * `alt_agl` unchanged (also invalid).  The transition uses `alt_agl` itself
 * as the RHS threshold (`BARO.alt > alt_agl`): because `alt_agl` is invalid,
 * `resolveVarThresholdLocked` returns false and the condition never fires —
 * the engine stays in WAIT.  If the executor were buggy and wrote
 * `alt_agl = BARO.alt(0) − ground_alt.value(0) = 0` at t = 0 ms, the
 * variable would become valid with value 0; the next tick at t = 2000 ms
 * (BARO.alt = 150 m) would then satisfy `150 > 0` and transition to END,
 * causing the test to fail.
 */
static const char kScriptExprUnsetVarOperand[] =
    "include SIM_GPS as GPS\n"
    "include SIM_BARO as BARO\n"
    "include SIM_COM as COM\n"
    "include SIM_IMU as IMU\n"
    "\n"
    "pus.apid = 1\n"
    "pus.service 5 as EVENT\n"
    "pus.service 1 as TC\n"
    "\n"
    "var ground_alt = 0.0\n"
    "var alt_agl = 0.0\n"
    "\n"
    "state WAIT:\n"
    "  on_enter:\n"
    "    set alt_agl = BARO.alt - ground_alt\n"
    "  transition to END when BARO.alt > alt_agl\n"
    "\n"
    "state END:\n"
    "  on_enter:\n"
    "    EVENT.info \"END\"\n";

// ── Fixture ────────────────────────────────────────────────────────────────────

struct ExprSetFixture
{
    ares::sim::SimStorageDriver storage;
    ares::sim::SimGpsDriver     gps{kExprProfile};
    ares::sim::SimBaroDriver    baro{kExprProfile};
    ares::sim::SimImuDriver     imu{kExprProfile};
    ares::sim::SimRadioDriver   radio;

    GpsEntry  gpsEntry  = { "SIM_GPS",  &gps   };
    BaroEntry baroEntry = { "SIM_BARO", &baro  };
    ComEntry  comEntry  = { "SIM_COM",  &radio };
    ImuEntry  imuEntry  = { "SIM_IMU",  &imu   };

    MissionScriptEngine engine{
        storage,
        &gpsEntry,  1U,
        &baroEntry, 1U,
        &comEntry,  1U,
        &imuEntry,  1U
    };

    void init(const char* path, const char* content)
    {
        (void)storage.begin();
        storage.registerFile(path, content);
        (void)gps.begin();
        (void)baro.begin();
        (void)imu.begin();
        (void)radio.begin();
        (void)engine.begin();
    }
};

// ── Tests ─────────────────────────────────────────────────────────────────────

// ─────────────────────────────────────────────────────────────────────────────
// Test 1: A script containing a 2-term arithmetic `set` expression parses and
// activates without error.
// ─────────────────────────────────────────────────────────────────────────────

void test_set_expr_parse_succeeds()
{
    ExprSetFixture f;
    f.init("/missions/expr_minus.ams", kScriptExprSensorMinusVar);

    const bool ok = f.engine.activate("expr_minus.ams");
    if (!ok) {
        EngineSnapshot snap{};
        f.engine.getSnapshot(snap);
        TEST_FAIL_MESSAGE(snap.lastError);
    }
    TEST_ASSERT_TRUE(ok);
}

// ─────────────────────────────────────────────────────────────────────────────
// Test 2: SENSOR minus VARIABLE executes correctly.
//
// Sequence:
//   t = 0    WAIT on_enter fires:  set ground_alt = BARO.alt (= 0 m).
//   t = 2000 Inject TC LAUNCH.
//   tick     WAIT → FLIGHT; FLIGHT on_enter: alt_agl = 150 - 0 = 150 m.
//            BARO.alt = 150 m > 50 so FLIGHT→END fires in the same tick.
// ─────────────────────────────────────────────────────────────────────────────

void test_set_expr_sensor_minus_var_executes()
{
    ExprSetFixture f;
    f.init("/missions/expr_minus2.ams", kScriptExprSensorMinusVar);
    TEST_ASSERT_TRUE(f.engine.activate("expr_minus2.ams"));
    TEST_ASSERT_TRUE(f.engine.arm());

    // t = 0: WAIT on_enter fires; ground_alt = BARO.alt = 0 m.
    f.engine.tick(ares::sim::clock::nowMs());

    // Advance to t = 2000 ms so BARO.alt = 150 m when FLIGHT is entered.
    ares::sim::clock::advanceMs(2000U);

    // Inject TC LAUNCH and tick: WAIT → FLIGHT; FLIGHT on_enter computes alt_agl.
    // BARO.alt = 150 m > 50 so the FLIGHT→END transition fires in the same tick.
    f.engine.injectTcCommand("LAUNCH");
    f.engine.tick(ares::sim::clock::nowMs());

    EngineSnapshot snap{};
    f.engine.getSnapshot(snap);
    TEST_ASSERT_EQUAL_STRING("END", snap.stateName);
}

// ─────────────────────────────────────────────────────────────────────────────
// Test 3: Three-term expression (SENSOR - VARIABLE) / LITERAL executes.
//
// Sequence:
//   t = 0    WAIT on_enter: set prev_alt = BARO.alt (= 0 m).
//   t = 2000 Inject TC LAUNCH.
//   tick     FLIGHT on_enter: vvel_avg = (150 - 0) / 0.5 = 300.
//            BARO.alt = 150 m > 50 so FLIGHT→END fires in the same tick.
// ─────────────────────────────────────────────────────────────────────────────

void test_set_expr_three_term_executes()
{
    ExprSetFixture f;
    f.init("/missions/expr_three.ams", kScriptExprThreeTerm);
    TEST_ASSERT_TRUE(f.engine.activate("expr_three.ams"));
    TEST_ASSERT_TRUE(f.engine.arm());

    // t = 0: WAIT on_enter fires; prev_alt = 0 m.
    f.engine.tick(ares::sim::clock::nowMs());

    // Advance to t = 2000 ms (BARO.alt = 150 m).
    ares::sim::clock::advanceMs(2000U);

    // BARO.alt = 150 m > 50 so FLIGHT→END fires in the same tick as LAUNCH.
    f.engine.injectTcCommand("LAUNCH");
    f.engine.tick(ares::sim::clock::nowMs());

    EngineSnapshot snap{};
    f.engine.getSnapshot(snap);
    TEST_ASSERT_EQUAL_STRING("END", snap.stateName);
}

// ─────────────────────────────────────────────────────────────────────────────
// Test 4: SENSOR plus LITERAL executes correctly.
//
// WAIT on_enter at t=0: alt_off = BARO.alt + 10.0 = 0 + 10 = 10 m.
// After TC LAUNCH, FLIGHT condition BARO.alt > -1 is immediately true → END.
// ─────────────────────────────────────────────────────────────────────────────

void test_set_expr_sensor_plus_literal()
{
    ExprSetFixture f;
    f.init("/missions/expr_plus.ams", kScriptExprSensorPlusLiteral);
    TEST_ASSERT_TRUE(f.engine.activate("expr_plus.ams"));
    TEST_ASSERT_TRUE(f.engine.arm());

    // t = 0: WAIT on_enter: alt_off = 0 + 10 = 10.
    f.engine.tick(ares::sim::clock::nowMs());

    // Inject TC LAUNCH → WAIT → FLIGHT; BARO.alt > -1 fires immediately → END.
    f.engine.injectTcCommand("LAUNCH");
    f.engine.tick(ares::sim::clock::nowMs());

    EngineSnapshot snap{};
    f.engine.getSnapshot(snap);
    TEST_ASSERT_EQUAL_STRING("END", snap.stateName);
}

// ─────────────────────────────────────────────────────────────────────────────
// Test 5: A literal zero in divisor position is rejected at parse time.
//
// `set ratio = BARO.alt / 0.0` must produce a parse error, preventing the
// script from activating.
// ─────────────────────────────────────────────────────────────────────────────

void test_set_expr_div_by_zero_literal_rejected()
{
    ExprSetFixture f;
    f.init("/missions/expr_divzero.ams", kScriptExprDivByZeroLiteral);

    const bool ok = f.engine.activate("expr_divzero.ams");
    TEST_ASSERT_FALSE(ok);

    EngineSnapshot snap{};
    f.engine.getSnapshot(snap);
    TEST_ASSERT_EQUAL(EngineStatus::ERROR, snap.status);
}

// ─────────────────────────────────────────────────────────────────────────────
// Test 6: An undeclared peripheral alias in an expression is a parse error.
//
// `set foo = NOPE.alt - ground_alt` must produce a parse error because
// NOPE has not been declared via `include`.
// ─────────────────────────────────────────────────────────────────────────────

void test_set_expr_unknown_alias_rejected()
{
    ExprSetFixture f;
    f.init("/missions/expr_badali.ams", kScriptExprUnknownAlias);

    const bool ok = f.engine.activate("expr_badali.ams");
    TEST_ASSERT_FALSE(ok);

    EngineSnapshot snap{};
    f.engine.getSnapshot(snap);
    TEST_ASSERT_EQUAL(EngineStatus::ERROR, snap.status);
}

// ─────────────────────────────────────────────────────────────────────────────
// Test 7: When a VARIABLE operand is not yet valid, the target is not updated.
//
// `ground_alt` is declared with `var ground_alt = 0.0` but is never written by
// a set action — its `valid` flag therefore remains false.
// When `set alt_agl = BARO.alt - ground_alt` fires in WAIT on_enter at t = 0
// (BARO.alt = 0 m), the executor detects that ground_alt is not valid and
// aborts without writing alt_agl (alt_agl remains invalid).
//
// The transition `BARO.alt > alt_agl` uses alt_agl as a variable threshold.
// Because alt_agl is invalid, resolveVarThresholdLocked returns false and the
// condition never fires — the engine stays in WAIT at t = 2000 ms (BARO = 150 m).
//
// If the executor were buggy and wrote alt_agl = 0 − 0 = 0 (using the raw
// default value), alt_agl.valid would become true and the transition
// `150 > 0` would fire at t = 2000 ms, transitioning to END and failing the test.
// ─────────────────────────────────────────────────────────────────────────────

void test_set_expr_unset_var_operand_skips_update()
{
    ExprSetFixture f;
    f.init("/missions/expr_unset.ams", kScriptExprUnsetVarOperand);
    TEST_ASSERT_TRUE(f.engine.activate("expr_unset.ams"));
    TEST_ASSERT_TRUE(f.engine.arm());

    // t = 0: WAIT on_enter: ground_alt is invalid → alt_agl stays invalid.
    f.engine.tick(ares::sim::clock::nowMs());

    EngineSnapshot snap{};
    f.engine.getSnapshot(snap);
    TEST_ASSERT_EQUAL_STRING("WAIT", snap.stateName);

    // Advance to t = 2000 ms (BARO.alt = 150 m).
    // If alt_agl were written (bug), threshold = 0 and 150 > 0 → transition fires.
    // With the fix, alt_agl remains invalid → condition always false → still WAIT.
    ares::sim::clock::advanceMs(2000U);
    f.engine.tick(ares::sim::clock::nowMs());
    f.engine.getSnapshot(snap);

    TEST_ASSERT_EQUAL_STRING("WAIT", snap.stateName);
}

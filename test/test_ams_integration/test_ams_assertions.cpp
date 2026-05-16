/**
 * @file  test_ams_assertions.cpp
 * @brief SITL integration tests — AMS-15 formal assertion checks (AMS-4.14).
 *
 * Covers all four assert kinds defined in AMS-4.14.2/3:
 *   - reachable STATE          : BFS reachability from state[0]
 *   - no_dead_states           : every declared state is reachable from state[0]
 *   - max_transition_depth < N : longest acyclic path depth < N; cycle detection
 *   - no_silent_terminals      : every state has ≥1 exit (transition/fallback/on_error/hk/log)
 *
 * Assertions are evaluated at parse time inside activate().  No arm() or
 * tick() calls are needed — the fixture only calls init() + activate().
 *
 * Test count: 14 (4 pass cases, 7 fail cases, 1 no-assert baseline, 2 no_silent_terminals)
 */

#include <unity.h>
#include <cstdio>
#include <cstring>

#include "ams/mission_script_engine.h"
#include "ams/ams_driver_registry.h"
#include "hal/storage/storage_interface.h"

using ares::ams::BaroEntry;
using ares::ams::ComEntry;
using ares::ams::EngineSnapshot;
using ares::ams::EngineStatus;
using ares::ams::GpsEntry;
using ares::ams::ImuEntry;
using ares::ams::MissionScriptEngine;

#include "sim_clock.h"
#include "sim_baro_driver.h"
#include "sim_gps_driver.h"
#include "sim_imu_driver.h"
#include "sim_radio_driver.h"
#include "sim_storage_driver.h"

// ── Minimal quiescent flight profile (sensors are never ticked in these tests) ─

static const ares::sim::FlightProfile kAssertProfile = {
    .count   = 1U,
    .samples = {
        { .timeMs        = 0U,
          .gpsLatDeg     = 0.0f,      .gpsLonDeg  = 0.0f,      .gpsAltM      = 0.0f,
          .gpsSpeedKmh   = 0.0f,      .gpsHdop    = 1.0f,
          .gpsSats       = 6U,        .gpsFix     = true,
          .baroAltM      = 0.0f,      .baroPressurePa = 101325.0f, .baroTempC = 20.0f,
          .accelX        = 0.0f,      .accelY     = 0.0f,      .accelZ      = 9.81f,
          .imuTempC      = 25.0f },
    }
};

// ── Test fixture ──────────────────────────────────────────────────────────────

struct AssertFixture
{
    ares::sim::SimStorageDriver storage;
    ares::sim::SimGpsDriver     gps  { kAssertProfile };
    ares::sim::SimBaroDriver    baro { kAssertProfile };
    ares::sim::SimImuDriver     imu  { kAssertProfile };
    ares::sim::SimRadioDriver   radio;

    GpsEntry  gpsEntry  = { "SIM_GPS",  &gps   };
    BaroEntry baroEntry = { "SIM_BARO", &baro  };
    ComEntry  comEntry  = { "SIM_COM",  &radio };
    ImuEntry  imuEntry  = { "SIM_IMU",  &imu   };

    MissionScriptEngine engine {
        storage,
        &gpsEntry,  1U,
        &baroEntry, 1U,
        &comEntry,  1U,
        &imuEntry,  1U
    };

    /** Register @p content at "/missions/<name>" and run the full begin chain. */
    bool load(const char* name, const char* content)
    {
        char path[64] = {};
        (void)snprintf(path, sizeof(path), "/missions/%s", name);

        (void)storage.begin();
        storage.registerFile(path, content);
        (void)gps.begin();
        (void)baro.begin();
        (void)imu.begin();
        (void)radio.begin();
        (void)engine.begin();

        return engine.activate(name);
    }
};

// ── Tests: assert reachable STATE ────────────────────────────────────────────

/**
 * PASS — assert reachable END: END is reachable from WAIT via TC.LAUNCH.
 * activate() must return true and status must be LOADED.
 */
void test_assert_reachable_pass()
{
    static const char kScript[] =
        "pus.apid = 1\n"
        "pus.service 1 as TC\n"
        "pus.service 5 as EVENT\n"
        "\n"
        "state WAIT:\n"
        "  transition to END when TC.command == LAUNCH\n"
        "\n"
        "state END:\n"
        "  on_enter:\n"
        "    EVENT.info \"DONE\"\n"
        "\n"
        "assert:\n"
        "  reachable END\n";

    AssertFixture f;
    const bool ok = f.load("reach_pass.ams", kScript);

    TEST_ASSERT_TRUE(ok);

    EngineSnapshot snap {};
    f.engine.getSnapshot(snap);
    TEST_ASSERT_EQUAL(EngineStatus::LOADED, snap.status);
}

/**
 * FAIL — assert reachable NONEXISTENT: state name does not appear in the
 * script at all.  activate() must return false, status must be ERROR, and
 * lastError must identify the unknown state name.
 *
 * Expected error substring: "NONEXISTENT"
 * (from: "assert reachable: unknown state 'NONEXISTENT'...")
 */
void test_assert_reachable_fail_unknown_state()
{
    static const char kScript[] =
        "pus.apid = 1\n"
        "pus.service 1 as TC\n"
        "pus.service 5 as EVENT\n"
        "\n"
        "state WAIT:\n"
        "  transition to END when TC.command == LAUNCH\n"
        "\n"
        "state END:\n"
        "  on_enter:\n"
        "    EVENT.info \"DONE\"\n"
        "\n"
        "assert:\n"
        "  reachable NONEXISTENT\n";

    AssertFixture f;
    const bool ok = f.load("reach_unknown.ams", kScript);

    TEST_ASSERT_FALSE(ok);

    EngineSnapshot snap {};
    f.engine.getSnapshot(snap);
    TEST_ASSERT_EQUAL(EngineStatus::ERROR, snap.status);
    TEST_ASSERT_NOT_NULL(strstr(snap.lastError, "NONEXISTENT"));
}

/**
 * FAIL — assert reachable ISLAND: ISLAND is a declared state but has no
 * incoming edges from WAIT or END, so it is unreachable from state[0].
 *
 * Graph:  WAIT ──TC.LAUNCH──► END
 *         ISLAND  (no incoming edges, not reachable)
 *
 * Expected error substring: "not reachable"
 * (from: "assert reachable: 'ISLAND' is not reachable from initial state")
 */
void test_assert_reachable_fail_isolated_state()
{
    static const char kScript[] =
        "pus.apid = 1\n"
        "pus.service 1 as TC\n"
        "pus.service 5 as EVENT\n"
        "\n"
        "state WAIT:\n"
        "  transition to END when TC.command == LAUNCH\n"
        "\n"
        "state END:\n"
        "  on_enter:\n"
        "    EVENT.info \"DONE\"\n"
        "\n"
        "state ISLAND:\n"
        "  on_enter:\n"
        "    EVENT.info \"ISOLATED\"\n"
        "\n"
        "assert:\n"
        "  reachable ISLAND\n";

    AssertFixture f;
    const bool ok = f.load("reach_isolated.ams", kScript);

    TEST_ASSERT_FALSE(ok);

    EngineSnapshot snap {};
    f.engine.getSnapshot(snap);
    TEST_ASSERT_EQUAL(EngineStatus::ERROR, snap.status);
    TEST_ASSERT_NOT_NULL(strstr(snap.lastError, "not reachable"));
}

// ── Tests: assert no_dead_states ─────────────────────────────────────────────

/**
 * PASS — assert no_dead_states: WAIT → END (both states reachable from WAIT).
 * activate() must return true and status must be LOADED.
 */
void test_assert_no_dead_states_pass()
{
    static const char kScript[] =
        "pus.apid = 1\n"
        "pus.service 1 as TC\n"
        "pus.service 5 as EVENT\n"
        "\n"
        "state WAIT:\n"
        "  transition to END when TC.command == LAUNCH\n"
        "\n"
        "state END:\n"
        "  on_enter:\n"
        "    EVENT.info \"DONE\"\n"
        "\n"
        "assert:\n"
        "  no_dead_states\n";

    AssertFixture f;
    const bool ok = f.load("nodead_pass.ams", kScript);

    TEST_ASSERT_TRUE(ok);

    EngineSnapshot snap {};
    f.engine.getSnapshot(snap);
    TEST_ASSERT_EQUAL(EngineStatus::LOADED, snap.status);
}

/**
 * FAIL — assert no_dead_states: ISLAND has no incoming edges from any
 * reachable state, so it is unreachable from state[0].
 *
 * Graph:  WAIT ──TC.LAUNCH──► END
 *         ISLAND  (declared, never reached)
 *
 * Expected error substrings: "no_dead_states" and "ISLAND"
 * (from: "assert no_dead_states: 'ISLAND' is unreachable")
 */
void test_assert_no_dead_states_fail()
{
    static const char kScript[] =
        "pus.apid = 1\n"
        "pus.service 1 as TC\n"
        "pus.service 5 as EVENT\n"
        "\n"
        "state WAIT:\n"
        "  transition to END when TC.command == LAUNCH\n"
        "\n"
        "state END:\n"
        "  on_enter:\n"
        "    EVENT.info \"DONE\"\n"
        "\n"
        "state ISLAND:\n"
        "  on_enter:\n"
        "    EVENT.info \"ISOLATED\"\n"
        "\n"
        "assert:\n"
        "  no_dead_states\n";

    AssertFixture f;
    const bool ok = f.load("nodead_fail.ams", kScript);

    TEST_ASSERT_FALSE(ok);

    EngineSnapshot snap {};
    f.engine.getSnapshot(snap);
    TEST_ASSERT_EQUAL(EngineStatus::ERROR, snap.status);
    TEST_ASSERT_NOT_NULL(strstr(snap.lastError, "no_dead_states"));
    TEST_ASSERT_NOT_NULL(strstr(snap.lastError, "ISLAND"));
}

// ── Tests: assert max_transition_depth < N ───────────────────────────────────

/**
 * PASS — assert max_transition_depth < 3.
 *
 * Linear chain: WAIT(depth 0) ──TC.LAUNCH──► FLIGHT(depth 1)
 *                              ──fallback──► END(depth 2)
 * Longest acyclic path depth = 2.  2 < 3 → assert passes.
 */
void test_assert_max_depth_pass()
{
    static const char kScript[] =
        "pus.apid = 1\n"
        "pus.service 1 as TC\n"
        "pus.service 5 as EVENT\n"
        "\n"
        "state WAIT:\n"
        "  transition to FLIGHT when TC.command == LAUNCH\n"
        "\n"
        "state FLIGHT:\n"
        "  fallback transition to END after 5000ms\n"
        "\n"
        "state END:\n"
        "  on_enter:\n"
        "    EVENT.info \"DONE\"\n"
        "\n"
        "assert:\n"
        "  max_transition_depth < 3\n";

    AssertFixture f;
    const bool ok = f.load("maxdepth_pass.ams", kScript);

    TEST_ASSERT_TRUE(ok);

    EngineSnapshot snap {};
    f.engine.getSnapshot(snap);
    TEST_ASSERT_EQUAL(EngineStatus::LOADED, snap.status);
}

/**
 * FAIL — assert max_transition_depth < 2: depth equals the limit (not strictly
 * less).
 *
 * Same 3-state chain as the pass test; longest path depth = 2.
 * 2 >= 2 → assert fails.
 *
 * Expected error substrings: "max_transition_depth" and "actual depth is 2"
 * (from: "assert max_transition_depth < 2: actual depth is 2")
 */
void test_assert_max_depth_fail_equals_limit()
{
    static const char kScript[] =
        "pus.apid = 1\n"
        "pus.service 1 as TC\n"
        "pus.service 5 as EVENT\n"
        "\n"
        "state WAIT:\n"
        "  transition to FLIGHT when TC.command == LAUNCH\n"
        "\n"
        "state FLIGHT:\n"
        "  fallback transition to END after 5000ms\n"
        "\n"
        "state END:\n"
        "  on_enter:\n"
        "    EVENT.info \"DONE\"\n"
        "\n"
        "assert:\n"
        "  max_transition_depth < 2\n";

    AssertFixture f;
    const bool ok = f.load("maxdepth_fail.ams", kScript);

    TEST_ASSERT_FALSE(ok);

    EngineSnapshot snap {};
    f.engine.getSnapshot(snap);
    TEST_ASSERT_EQUAL(EngineStatus::ERROR, snap.status);
    TEST_ASSERT_NOT_NULL(strstr(snap.lastError, "max_transition_depth"));
    TEST_ASSERT_NOT_NULL(strstr(snap.lastError, "actual depth is 2"));
}

/**
 * FAIL — assert max_transition_depth with a cycle in the graph.
 *
 * Graph:  WAIT ──TC.LAUNCH──► FLIGHT ──fallback 3000ms──► WAIT  (cycle)
 *
 * DFS detects WAIT already on the path when processing FLIGHT's fallback
 * target → hasCycle = true.  Any max_transition_depth assert on a cyclic
 * graph must fail with "cycle detected".
 *
 * Expected error substring: "cycle detected"
 * (from: "assert max_transition_depth < 10: cycle detected (unbounded depth)")
 */
void test_assert_max_depth_cycle_detected()
{
    static const char kScript[] =
        "pus.apid = 1\n"
        "pus.service 1 as TC\n"
        "pus.service 5 as EVENT\n"
        "\n"
        "state WAIT:\n"
        "  transition to FLIGHT when TC.command == LAUNCH\n"
        "\n"
        "state FLIGHT:\n"
        "  fallback transition to WAIT after 3000ms\n"
        "\n"
        "assert:\n"
        "  max_transition_depth < 10\n";

    AssertFixture f;
    const bool ok = f.load("cycle.ams", kScript);

    TEST_ASSERT_FALSE(ok);

    EngineSnapshot snap {};
    f.engine.getSnapshot(snap);
    TEST_ASSERT_EQUAL(EngineStatus::ERROR, snap.status);
    TEST_ASSERT_NOT_NULL(strstr(snap.lastError, "cycle detected"));
}

// ── Tests: multiple assertions in one block ───────────────────────────────────

/**
 * PASS — four assertions in one assert: block; all must pass.
 *
 * Graph: WAIT → FLIGHT → END  (3 states, fully connected, depth 2)
 * Assertions: reachable END, reachable FLIGHT, no_dead_states,
 *             max_transition_depth < 5.
 */
void test_assert_multiple_all_pass()
{
    static const char kScript[] =
        "pus.apid = 1\n"
        "pus.service 1 as TC\n"
        "pus.service 5 as EVENT\n"
        "\n"
        "state WAIT:\n"
        "  transition to FLIGHT when TC.command == LAUNCH\n"
        "\n"
        "state FLIGHT:\n"
        "  fallback transition to END after 5000ms\n"
        "\n"
        "state END:\n"
        "  on_enter:\n"
        "    EVENT.info \"DONE\"\n"
        "\n"
        "assert:\n"
        "  reachable END\n"
        "  reachable FLIGHT\n"
        "  no_dead_states\n"
        "  max_transition_depth < 5\n";

    AssertFixture f;
    const bool ok = f.load("multi_pass.ams", kScript);

    TEST_ASSERT_TRUE(ok);

    EngineSnapshot snap {};
    f.engine.getSnapshot(snap);
    TEST_ASSERT_EQUAL(EngineStatus::LOADED, snap.status);
}

/**
 * FAIL — first assertion passes, second fails (early-exit on first failure).
 *
 * Graph: WAIT → END + ISLAND (unreachable).
 * Assertions: reachable END (pass), no_dead_states (fail — ISLAND).
 *
 * Engine must stop at the failing assertion and report its error.
 *
 * Expected error substring: "no_dead_states"
 */
void test_assert_multiple_second_fails()
{
    static const char kScript[] =
        "pus.apid = 1\n"
        "pus.service 1 as TC\n"
        "pus.service 5 as EVENT\n"
        "\n"
        "state WAIT:\n"
        "  transition to END when TC.command == LAUNCH\n"
        "\n"
        "state END:\n"
        "  on_enter:\n"
        "    EVENT.info \"DONE\"\n"
        "\n"
        "state ISLAND:\n"
        "  on_enter:\n"
        "    EVENT.info \"ISOLATED\"\n"
        "\n"
        "assert:\n"
        "  reachable END\n"
        "  no_dead_states\n";

    AssertFixture f;
    const bool ok = f.load("multi_fail.ams", kScript);

    TEST_ASSERT_FALSE(ok);

    EngineSnapshot snap {};
    f.engine.getSnapshot(snap);
    TEST_ASSERT_EQUAL(EngineStatus::ERROR, snap.status);
    TEST_ASSERT_NOT_NULL(strstr(snap.lastError, "no_dead_states"));
}

// ── Baseline: no assert block ─────────────────────────────────────────────────

/**
 * BASELINE — script with no assert: block activates successfully.
 * Confirms that the assert block is entirely optional (AMS-4.14.5).
 */
void test_no_assert_block_activates_ok()
{
    static const char kScript[] =
        "pus.apid = 1\n"
        "pus.service 1 as TC\n"
        "pus.service 5 as EVENT\n"
        "\n"
        "state WAIT:\n"
        "  transition to END when TC.command == LAUNCH\n"
        "\n"
        "state END:\n"
        "  on_enter:\n"
        "    EVENT.info \"DONE\"\n";

    AssertFixture f;
    const bool ok = f.load("noassert.ams", kScript);

    TEST_ASSERT_TRUE(ok);

    EngineSnapshot snap {};
    f.engine.getSnapshot(snap);
    TEST_ASSERT_EQUAL(EngineStatus::LOADED, snap.status);
}

// ── Tests: assert no_silent_terminals ────────────────────────────────────────

/**
 * PASS — all states have outgoing edges.
 *
 * Graph: WAIT ──TC.LAUNCH──► HOLD ──fallback 10000ms──► END
 *        WAIT: has transition (not silent)
 *        HOLD: has fallback (not silent)
 *        END:  has fallback self-loop (not silent)
 *
 * activate() must return true and status must be LOADED.
 */
void test_assert_no_silent_terminals_pass()
{
    static const char kScript[] =
        "pus.apid = 1\n"
        "pus.service 1 as TC\n"
        "pus.service 5 as EVENT\n"
        "\n"
        "state WAIT:\n"
        "  transition to HOLD when TC.command == LAUNCH\n"
        "\n"
        "state HOLD:\n"
        "  on_enter:\n"
        "    EVENT.info \"HOLDING\"\n"
        "  fallback transition to END after 10000ms\n"
        "\n"
        "state END:\n"
        "  on_enter:\n"
        "    EVENT.info \"DONE\"\n"
        "  fallback transition to END after 99999ms\n"
        "\n"
        "assert:\n"
        "  no_silent_terminals\n";

    AssertFixture f;
    const bool ok = f.load("nst_pass.ams", kScript);

    TEST_ASSERT_TRUE(ok);

    EngineSnapshot snap {};
    f.engine.getSnapshot(snap);
    TEST_ASSERT_EQUAL(EngineStatus::LOADED, snap.status);
}

/**
 * PASS — terminal state has an HK slot; HK/LOG slots count as an exit
 * because the state is still actively reporting rather than silently halted.
 *
 * Graph: WAIT ──TC.LAUNCH──► REPORT
 *        WAIT:   has transition (not silent)
 *        REPORT: hkSlotCount = 1 (not silent)
 *
 * activate() must return true and status must be LOADED.
 */
void test_assert_no_silent_terminals_pass_with_hk()
{
    static const char kScript[] =
        "pus.apid = 1\n"
        "pus.service 3 as HK\n"
        "pus.service 5 as EVENT\n"
        "pus.service 1 as TC\n"
        "include SIM_GPS as GPS\n"
        "\n"
        "state WAIT:\n"
        "  transition to REPORT when TC.command == LAUNCH\n"
        "\n"
        "state REPORT:\n"
        "  on_enter:\n"
        "    EVENT.info \"REPORTING\"\n"
        "  every 2000ms:\n"
        "    HK.report {\n"
        "      gps_alt: GPS.alt\n"
        "    }\n"
        "\n"
        "assert:\n"
        "  no_silent_terminals\n";

    AssertFixture f;
    const bool ok = f.load("nst_pass_hk.ams", kScript);

    TEST_ASSERT_TRUE(ok);

    EngineSnapshot snap {};
    f.engine.getSnapshot(snap);
    TEST_ASSERT_EQUAL(EngineStatus::LOADED, snap.status);
}

/**
 * FAIL — DEAD state has no transitions, no fallback, no on_error, no HK, no LOG.
 *
 * Graph: WAIT ──TC.LAUNCH──► DEAD
 *        WAIT: has transition (not silent)
 *        DEAD: only has on_enter event — qualifies as a silent terminal
 *
 * activate() must return false; lastError must identify 'DEAD'.
 *
 * Expected error substring: "no_silent_terminals" and "DEAD"
 */
void test_assert_no_silent_terminals_fail()
{
    static const char kScript[] =
        "pus.apid = 1\n"
        "pus.service 1 as TC\n"
        "pus.service 5 as EVENT\n"
        "\n"
        "state WAIT:\n"
        "  transition to DEAD when TC.command == LAUNCH\n"
        "\n"
        "state DEAD:\n"
        "  on_enter:\n"
        "    EVENT.info \"SILENT\"\n"
        "\n"
        "assert:\n"
        "  no_silent_terminals\n";

    AssertFixture f;
    const bool ok = f.load("nst_fail.ams", kScript);

    TEST_ASSERT_FALSE(ok);

    EngineSnapshot snap {};
    f.engine.getSnapshot(snap);
    TEST_ASSERT_EQUAL(EngineStatus::ERROR, snap.status);
    TEST_ASSERT_NOT_NULL(strstr(snap.lastError, "no_silent_terminals"));
    TEST_ASSERT_NOT_NULL(strstr(snap.lastError, "DEAD"));
}

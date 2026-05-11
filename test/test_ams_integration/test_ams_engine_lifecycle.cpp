/**
 * @file  test_ams_engine_lifecycle.cpp
 * @brief SITL integration tests — AMS engine lifecycle (begin/activate/arm/deactivate).
 *
 * Verifies that the engine transitions through its operational states correctly
 * when driven by in-memory sim drivers and a time-deterministic clock.
 */

#include <unity.h>

#include "ams/mission_script_engine.h"
#include "ams/ams_driver_registry.h"

using ares::ams::GpsEntry;
using ares::ams::BaroEntry;
using ares::ams::ComEntry;
using ares::ams::ImuEntry;

#include "sim_ams_scripts.h"
#include "sim_clock.h"
#include "sim_gps_driver.h"
#include "sim_baro_driver.h"
#include "sim_imu_driver.h"
#include "sim_radio_driver.h"
#include "sim_storage_driver.h"

using ares::ams::MissionScriptEngine;
using ares::ams::EngineSnapshot;
using ares::ams::EngineStatus;

// ── Shared flat flight profile (all sensors at resting values) ───────────────

static const ares::sim::FlightProfile kRestProfile = {
    .count   = 2U,
    .samples = {
        { .timeMs = 0U,      .gpsLatDeg = 40.0f, .gpsLonDeg = -3.0f,
          .gpsAltM = 0.0f,   .gpsSpeedKmh = 0.0f, .gpsSats = 8U, .gpsFix = true,
          .baroAltM = 0.0f,  .baroPressurePa = 101325.0f, .baroTempC = 20.0f,
          .accelX = 0.0f,    .accelY = 0.0f, .accelZ = 9.81f, .imuTempC = 25.0f },
        { .timeMs = 60000U,  .gpsLatDeg = 40.0f, .gpsLonDeg = -3.0f,
          .gpsAltM = 0.0f,   .gpsSpeedKmh = 0.0f, .gpsSats = 8U, .gpsFix = true,
          .baroAltM = 0.0f,  .baroPressurePa = 101325.0f, .baroTempC = 20.0f,
          .accelX = 0.0f,    .accelY = 0.0f, .accelZ = 9.81f, .imuTempC = 25.0f },
    }
};

// ── Helper: build a fully initialised engine for each test ───────────────────

struct SimFixture
{
    ares::sim::SimStorageDriver storage;
    ares::sim::SimGpsDriver     gps{kRestProfile};
    ares::sim::SimBaroDriver    baro{kRestProfile};
    ares::sim::SimImuDriver     imu{kRestProfile};
    ares::sim::SimRadioDriver   radio;

    GpsEntry  gpsEntry  = { "SIM_GPS",  &gps  };
    BaroEntry baroEntry = { "SIM_BARO", &baro };
    ComEntry  comEntry  = { "SIM_COM",  &radio};
    ImuEntry  imuEntry  = { "SIM_IMU",  &imu  };

    MissionScriptEngine engine{
        storage,
        &gpsEntry,  1U,
        &baroEntry, 1U,
        &comEntry,  1U,
        &imuEntry,  1U
    };
};

// ── Tests ────────────────────────────────────────────────────────────────────

void test_begin_returns_true()
{
    SimFixture f;
    TEST_ASSERT_TRUE(f.storage.begin());
    TEST_ASSERT_TRUE(f.engine.begin());
}

void test_initial_status_is_idle()
{
    SimFixture f;
    (void)f.storage.begin();
    (void)f.engine.begin();

    EngineSnapshot snap{};
    f.engine.getSnapshot(snap);

    TEST_ASSERT_EQUAL(EngineStatus::IDLE, snap.status);
}

void test_activate_missing_file_returns_false()
{
    SimFixture f;
    (void)f.storage.begin();
    (void)f.engine.begin();

    // No file registered — activate should fail gracefully.
    const bool ok = f.engine.activate("nonexistent.ams");
    TEST_ASSERT_FALSE(ok);
}

void test_activate_sets_status_loaded()
{
    SimFixture f;
    (void)f.storage.begin();
    f.storage.registerFile("/missions/lifecycle.ams", ares::sim::kScriptLifecycle);
    (void)f.engine.begin();

    TEST_ASSERT_TRUE(f.engine.activate("lifecycle.ams"));

    EngineSnapshot snap{};
    f.engine.getSnapshot(snap);
    TEST_ASSERT_EQUAL(EngineStatus::LOADED, snap.status);
}

void test_activate_reports_correct_state_name()
{
    SimFixture f;
    (void)f.storage.begin();
    f.storage.registerFile("/missions/lifecycle.ams", ares::sim::kScriptLifecycle);
    (void)f.engine.begin();
    (void)f.engine.activate("lifecycle.ams");

    EngineSnapshot snap{};
    f.engine.getSnapshot(snap);

    // Engine starts at WAIT state (first state in the script).
    TEST_ASSERT_EQUAL_STRING("WAIT", snap.stateName);
}

void test_arm_transitions_engine_to_running()
{
    SimFixture f;
    (void)f.storage.begin();
    f.storage.registerFile("/missions/lifecycle.ams", ares::sim::kScriptLifecycle);
    (void)f.engine.begin();
    (void)f.engine.activate("lifecycle.ams");

    TEST_ASSERT_TRUE(f.engine.arm());

    EngineSnapshot snap{};
    f.engine.getSnapshot(snap);
    TEST_ASSERT_EQUAL(EngineStatus::RUNNING, snap.status);
}

void test_arm_on_idle_engine_returns_false()
{
    SimFixture f;
    (void)f.storage.begin();
    (void)f.engine.begin();

    // arm() before activate() — engine is IDLE, not LOADED.
    TEST_ASSERT_FALSE(f.engine.arm());
}

void test_tick_consumes_launch_tc_and_transitions()
{
    SimFixture f;
    (void)f.storage.begin();
    f.storage.registerFile("/missions/lifecycle.ams", ares::sim::kScriptLifecycle);
    (void)f.engine.begin();
    (void)f.engine.activate("lifecycle.ams");
    (void)f.engine.arm();   // injects LAUNCH TC internally

    // One tick: WAIT + LAUNCH TC should fire the transition to END.
    ares::sim::clock::reset();
    f.engine.tick(ares::sim::clock::nowMs());

    EngineSnapshot snap{};
    f.engine.getSnapshot(snap);
    TEST_ASSERT_EQUAL_STRING("END", snap.stateName);
}

void test_terminal_state_marks_engine_complete()
{
    SimFixture f;
    (void)f.storage.begin();
    f.storage.registerFile("/missions/lifecycle.ams", ares::sim::kScriptLifecycle);
    (void)f.engine.begin();
    (void)f.engine.activate("lifecycle.ams");
    (void)f.engine.arm();

    ares::sim::clock::reset();

    // Tick 1: WAIT → END (LAUNCH TC consumed).
    f.engine.tick(ares::sim::clock::nowMs());
    ares::sim::clock::advanceMs(100U);

    // Tick 2: END state — no transitions, no actions → COMPLETE.
    f.engine.tick(ares::sim::clock::nowMs());

    EngineSnapshot snap{};
    f.engine.getSnapshot(snap);
    TEST_ASSERT_EQUAL(EngineStatus::COMPLETE, snap.status);
}

void test_deactivate_returns_engine_to_idle()
{
    SimFixture f;
    (void)f.storage.begin();
    f.storage.registerFile("/missions/lifecycle.ams", ares::sim::kScriptLifecycle);
    (void)f.engine.begin();
    (void)f.engine.activate("lifecycle.ams");
    (void)f.engine.arm();

    f.engine.deactivate();

    EngineSnapshot snap{};
    f.engine.getSnapshot(snap);
    TEST_ASSERT_EQUAL(EngineStatus::IDLE, snap.status);
    TEST_ASSERT_EQUAL_STRING("", snap.activeFile);
}

void test_engine_reactivatable_after_deactivate()
{
    SimFixture f;
    (void)f.storage.begin();
    f.storage.registerFile("/missions/lifecycle.ams", ares::sim::kScriptLifecycle);
    (void)f.engine.begin();

    // First activation cycle.
    (void)f.engine.activate("lifecycle.ams");
    (void)f.engine.arm();
    f.engine.deactivate();

    // Second activation cycle — engine must accept it cleanly.
    TEST_ASSERT_TRUE(f.engine.activate("lifecycle.ams"));

    EngineSnapshot snap{};
    f.engine.getSnapshot(snap);
    TEST_ASSERT_EQUAL(EngineStatus::LOADED, snap.status);
}

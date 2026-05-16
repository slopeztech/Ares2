/**
 * @file  test_ams_tasks.cpp
 * @brief SITL integration tests — AMS-4.13 Background Tasks with state filter.
 *
 * Verifies that the 'when in' state-filter clause controls task execution:
 *   - Task fires every 100 ms while the engine is in the filtered-in state.
 *   - Task is silenced (timer reset each tick) while outside the filter.
 *   - Task fires promptly on re-entry after a state exclusion period.
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

// ── Flight profile ────────────────────────────────────────────────────────────

static const ares::sim::FlightProfile kTaskProfile = {
    .count   = 1U,
    .samples = {
        { .timeMs = 0U, .gpsLatDeg = 40.0f, .gpsLonDeg = -3.0f,
          .gpsAltM = 10.0f, .gpsSpeedKmh = 0.0f,
          .gpsHdop = 1.0f, .gpsSats = 9U, .gpsFix = true,
          .baroAltM = 10.0f, .baroPressurePa = 101000.0f, .baroTempC = 20.0f,
          .accelX = 0.0f, .accelY = 0.0f, .accelZ = 9.81f, .imuTempC = 25.0f },
    }
};

// ── Fixture ───────────────────────────────────────────────────────────────────

struct TaskFixture
{
    ares::sim::SimStorageDriver storage;
    ares::sim::SimGpsDriver     gps{kTaskProfile};
    ares::sim::SimBaroDriver    baro{kTaskProfile};
    ares::sim::SimImuDriver     imu{kTaskProfile};
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
// kScriptTaskStateFilter: script with 'when in' state filter parses correctly.
// ─────────────────────────────────────────────────────────────────────────────

void test_task_state_filter_script_parses_ok()
{
    TaskFixture f;
    f.init("/missions/task_sf.ams", ares::sim::kScriptTaskStateFilter);
    TEST_ASSERT_TRUE(f.engine.activate("task_sf.ams"));

    EngineSnapshot snap{};
    f.engine.getSnapshot(snap);
    TEST_ASSERT_EQUAL(EngineStatus::LOADED, snap.status);
}

// ─────────────────────────────────────────────────────────────────────────────
// Task fires in the filtered-in state.
//
// After arm + tick(0), the engine is in ACTIVE.  At t=200 ms the task has
// run twice (at t=100 and t=200), each emitting one EVENT.info frame via
// radio.  sendCount must be > 0.
// ─────────────────────────────────────────────────────────────────────────────

void test_task_fires_in_filtered_state()
{
    TaskFixture f;
    f.init("/missions/task_sf.ams", ares::sim::kScriptTaskStateFilter);
    TEST_ASSERT_TRUE(f.engine.activate("task_sf.ams"));
    TEST_ASSERT_TRUE(f.engine.arm());

    f.engine.tick(ares::sim::clock::nowMs());  // t=0 — enter ACTIVE

    EngineSnapshot snap{};
    f.engine.getSnapshot(snap);
    TEST_ASSERT_EQUAL_STRING("ACTIVE", snap.stateName);

    f.radio.resetCapture();

    // Advance to t=200 ms — task must have fired at least once.
    ares::sim::clock::advanceMs(200U);
    f.engine.tick(ares::sim::clock::nowMs());

    TEST_ASSERT_GREATER_THAN(0U, f.radio.sendCount());
}

// ─────────────────────────────────────────────────────────────────────────────
// Task is silenced while in the filtered-out state (RECOVERY).
//
// At t=600 ms TIME.elapsed in ACTIVE exceeds 500 ms → transition to RECOVERY.
// After the transition, no new EVENT frames should be sent.
// ─────────────────────────────────────────────────────────────────────────────

void test_task_skips_when_state_filtered_out()
{
    TaskFixture f;
    f.init("/missions/task_sf.ams", ares::sim::kScriptTaskStateFilter);
    TEST_ASSERT_TRUE(f.engine.activate("task_sf.ams"));
    TEST_ASSERT_TRUE(f.engine.arm());

    f.engine.tick(ares::sim::clock::nowMs());  // t=0 — ACTIVE

    // Cross the 500 ms threshold → transition to RECOVERY.
    ares::sim::clock::advanceMs(600U);
    f.engine.tick(ares::sim::clock::nowMs());  // t=600 — RECOVERY

    EngineSnapshot snap{};
    f.engine.getSnapshot(snap);
    TEST_ASSERT_EQUAL_STRING("RECOVERY", snap.stateName);

    // No task events expected in RECOVERY.
    f.radio.resetCapture();

    ares::sim::clock::advanceMs(300U);
    f.engine.tick(ares::sim::clock::nowMs());  // t=900 — still RECOVERY

    TEST_ASSERT_EQUAL(0U, f.radio.sendCount());
}

// ─────────────────────────────────────────────────────────────────────────────
// Task fires promptly on re-entry into the filtered state.
//
// Uses kScriptTaskStateFilterLoop (ACTIVE → RECOVERY → ACTIVE cycle).
// After re-entering ACTIVE, the task must emit at least one EVENT frame
// within the next 100 ms period (AMS-4.13.4 timer-reset guarantee).
// ─────────────────────────────────────────────────────────────────────────────

void test_task_resumes_promptly_on_state_reentry()
{
    TaskFixture f;
    f.init("/missions/task_loop.ams", ares::sim::kScriptTaskStateFilterLoop);
    TEST_ASSERT_TRUE(f.engine.activate("task_loop.ams"));
    TEST_ASSERT_TRUE(f.engine.arm());

    f.engine.tick(ares::sim::clock::nowMs());  // t=0 — ACTIVE

    // Cross 300 ms → transition ACTIVE → RECOVERY.
    ares::sim::clock::advanceMs(400U);
    f.engine.tick(ares::sim::clock::nowMs());  // t=400 — RECOVERY

    EngineSnapshot snap{};
    f.engine.getSnapshot(snap);
    TEST_ASSERT_EQUAL_STRING("RECOVERY", snap.stateName);

    // Spend ~200 ms in RECOVERY (timer resets every tick).
    ares::sim::clock::advanceMs(100U);
    f.engine.tick(ares::sim::clock::nowMs());  // t=500 — RECOVERY (timer reset to 500)

    // Cross the RECOVERY 200 ms timeout → re-enter ACTIVE.
    // TIME.elapsed in RECOVERY = (601-400)=201 > 200 → transition fires.
    ares::sim::clock::advanceMs(101U);
    f.engine.tick(ares::sim::clock::nowMs());  // t=601 — ACTIVE re-entered

    f.engine.getSnapshot(snap);
    TEST_ASSERT_EQUAL_STRING("ACTIVE", snap.stateName);

    // After re-entry, the task timer was last reset to t=500 (last RECOVERY tick).
    // At t=601, 601-500=101 >= 100 → task fires immediately on re-entry tick.
    // Reset capture AFTER the re-entry tick to count only re-entry task fires.
    f.radio.resetCapture();

    // One more tick well inside the next period to confirm task is active again.
    ares::sim::clock::advanceMs(100U);
    f.engine.tick(ares::sim::clock::nowMs());  // t=701

    TEST_ASSERT_GREATER_THAN(0U, f.radio.sendCount());
}

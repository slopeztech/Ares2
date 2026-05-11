/**
 * @file  test_ams_on_exit_set.cpp
 * @brief SITL integration tests — `set` actions inside `on_exit:` blocks.
 *
 * The AMS standard (AMS-4.9) allows zero or more `set` actions inside an
 * `on_exit:` block.  These execute synchronously when the engine transitions
 * OUT of the state (via any transition: normal, fallback, or error-recovery).
 *
 * The firmware has always had the runtime infrastructure for this (the
 * `onExitSetActions` array and the iteration in `exitStateLocked`).  This
 * test suite verifies end-to-end that:
 *   1. A variable set in `on_exit:` is written before the next state evaluates
 *      its entry conditions.
 *   2. A variable RHS condition in the NEXT state can consume the value set
 *      in the previous state's `on_exit:`.
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

// ── Profile: ascent from 0 m to 300 m ────────────────────────────────────────

static const ares::sim::FlightProfile kExitSetProfile = {
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

// ── Fixture ───────────────────────────────────────────────────────────────────

struct ExitSetFixture
{
    ares::sim::SimStorageDriver storage;
    ares::sim::SimGpsDriver     gps{kExitSetProfile};
    ares::sim::SimBaroDriver    baro{kExitSetProfile};
    ares::sim::SimImuDriver     imu{kExitSetProfile};
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
// Verify the script with set in on_exit parses without error.
// ─────────────────────────────────────────────────────────────────────────────

void test_on_exit_set_script_parses_successfully()
{
    ExitSetFixture f;
    f.init("/missions/exit_set.ams", ares::sim::kScriptOnExitSet);

    const bool ok = f.engine.activate("exit_set.ams");
    TEST_ASSERT_TRUE(ok);
}

// ─────────────────────────────────────────────────────────────────────────────
// Verify that the variable written in on_exit: is available in the next state.
//
// Profile starts at BARO.alt = 0 m; on_exit sets `exit_alt = BARO.alt = 0`.
// FLIGHT condition is `BARO.alt > exit_alt` (= 0).  At t=2000 ms the altitude
// is 150 m, satisfying the condition and triggering the END transition.
// ─────────────────────────────────────────────────────────────────────────────

void test_on_exit_set_variable_used_in_next_state_condition()
{
    ExitSetFixture f;
    f.init("/missions/exit_set2.ams", ares::sim::kScriptOnExitSet);
    TEST_ASSERT_TRUE(f.engine.activate("exit_set2.ams"));
    TEST_ASSERT_TRUE(f.engine.arm());

    ares::sim::clock::reset();

    // First tick at t=0: WAIT on_enter fires.
    f.engine.tick(ares::sim::clock::nowMs());

    // Inject TC LAUNCH at t=0 to trigger transition from WAIT to FLIGHT.
    // WAIT on_exit fires: set exit_alt = BARO.alt (≈0 m at t=0).
    f.engine.injectTcCommand("LAUNCH");
    f.engine.tick(ares::sim::clock::nowMs());

    EngineSnapshot snap{};
    f.engine.getSnapshot(snap);

    // Should now be in FLIGHT (BARO.alt > exit_alt not yet satisfied at t=0).
    TEST_ASSERT_EQUAL_STRING("FLIGHT", snap.stateName);

    // Advance to t=2000 ms: BARO.alt ≈ 150 m > exit_alt (≈ 0 m).
    ares::sim::clock::advanceMs(2000U);
    f.engine.tick(ares::sim::clock::nowMs());
    f.engine.getSnapshot(snap);

    // Transition to END must have fired.
    TEST_ASSERT_EQUAL_STRING("END", snap.stateName);
}

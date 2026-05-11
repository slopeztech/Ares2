/**
 * @file  test_ams_engine_control.cpp
 * @brief SITL integration tests — AMS engine control-flow APIs.
 *
 * Covers: ABORT TC force-deactivation vs. explicit-transition consumption,
 * setExecutionEnabled pause/resume (including Bug #5 hold-window regression),
 * requestTelemetry on-demand HK emission, and notifyPyroFired status bits.
 */

#include <unity.h>

#include "ams/mission_script_engine.h"
#include "ams/ams_driver_registry.h"
#include "comms/ares_radio_protocol.h"

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

// ── Shared flight profile ─────────────────────────────────────────────────────
// Identical to the scenario test profile: ascent from 0 m to 300 m over 15 s.
// The hold-window tests rely on altitude exceeding 50 m at t ≈ 2500 ms and
// reaching 210 m at t = 4000 ms.

static const ares::sim::FlightProfile kCtrlProfile = {
    .count   = 5U,
    .samples = {
        { .timeMs =     0U, .gpsLatDeg = 40.0f, .gpsLonDeg = -3.0f,
          .gpsAltM =   0.0f, .gpsSpeedKmh =  0.0f, .gpsSats = 9U, .gpsFix = true,
          .baroAltM =  0.0f, .baroPressurePa = 101325.0f, .baroTempC = 20.0f,
          .accelX = 0.0f, .accelY = 0.0f, .accelZ =  9.81f, .imuTempC = 25.0f },
        { .timeMs =  2000U, .gpsLatDeg = 40.0f, .gpsLonDeg = -3.0f,
          .gpsAltM =  20.0f, .gpsSpeedKmh = 36.0f, .gpsSats = 9U, .gpsFix = true,
          .baroAltM = 20.0f, .baroPressurePa =  99000.0f, .baroTempC = 19.0f,
          .accelX = 0.5f, .accelY = 0.0f, .accelZ = 14.0f,  .imuTempC = 26.0f },
        { .timeMs =  4000U, .gpsLatDeg = 40.0f, .gpsLonDeg = -3.0f,
          .gpsAltM = 210.0f, .gpsSpeedKmh = 72.0f, .gpsSats = 9U, .gpsFix = true,
          .baroAltM = 210.0f, .baroPressurePa = 98700.0f, .baroTempC = 17.0f,
          .accelX = 0.3f, .accelY = 0.0f, .accelZ = 12.0f,  .imuTempC = 27.0f },
        { .timeMs =  8000U, .gpsLatDeg = 40.0f, .gpsLonDeg = -3.0f,
          .gpsAltM = 300.0f, .gpsSpeedKmh =  5.0f, .gpsSats = 9U, .gpsFix = true,
          .baroAltM = 300.0f, .baroPressurePa = 97700.0f, .baroTempC = 15.0f,
          .accelX = 0.0f, .accelY = 0.0f, .accelZ =  9.81f, .imuTempC = 28.0f },
        { .timeMs = 15000U, .gpsLatDeg = 40.0f, .gpsLonDeg = -3.0f,
          .gpsAltM = 300.0f, .gpsSpeedKmh =  3.0f, .gpsSats = 9U, .gpsFix = true,
          .baroAltM = 300.0f, .baroPressurePa = 97700.0f, .baroTempC = 15.0f,
          .accelX = 0.0f, .accelY = 0.0f, .accelZ =  9.81f, .imuTempC = 28.0f },
    }
};

// ── Fixture ───────────────────────────────────────────────────────────────────

struct ControlFixture
{
    ares::sim::SimStorageDriver storage;
    ares::sim::SimGpsDriver     gps{kCtrlProfile};
    ares::sim::SimBaroDriver    baro{kCtrlProfile};
    ares::sim::SimImuDriver     imu{kCtrlProfile};
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
// ABORT TC: force-deactivation when not consumed by any transition (AMS-5.1).
// ─────────────────────────────────────────────────────────────────────────────

void test_abort_tc_deactivates_engine()
{
    ControlFixture f;
    f.init("/missions/flight.ams", ares::sim::kScriptFlight);
    ares::sim::clock::reset();

    (void)f.engine.activate("flight.ams");
    (void)f.engine.arm();

    // t=0: LAUNCH TC fires WAIT → FLIGHT.
    f.engine.tick(ares::sim::clock::nowMs());

    EngineSnapshot snap{};
    f.engine.getSnapshot(snap);
    TEST_ASSERT_EQUAL_STRING("FLIGHT", snap.stateName);

    // kScriptFlight FLIGHT state has no ABORT transition.
    (void)f.engine.injectTcCommand("ABORT");
    ares::sim::clock::advanceMs(1U);
    f.engine.tick(ares::sim::clock::nowMs());

    // Unhandled ABORT → engine force-deactivated → IDLE.
    f.engine.getSnapshot(snap);
    TEST_ASSERT_EQUAL(EngineStatus::IDLE, snap.status);
}

// ─────────────────────────────────────────────────────────────────────────────
// ABORT TC: consumed by an explicit transition instead of force-deactivating.
// ─────────────────────────────────────────────────────────────────────────────

void test_abort_tc_consumed_by_explicit_transition()
{
    ControlFixture f;
    f.init("/missions/ctrl.ams", ares::sim::kScriptAbortTransition);
    ares::sim::clock::reset();

    (void)f.engine.activate("ctrl.ams");
    (void)f.engine.arm();

    // t=0: enter FLIGHT.
    f.engine.tick(ares::sim::clock::nowMs());

    EngineSnapshot snap{};
    f.engine.getSnapshot(snap);
    TEST_ASSERT_EQUAL_STRING("FLIGHT", snap.stateName);

    // ABORT TC — FLIGHT has an explicit "transition to SAFE when TC.command == ABORT".
    (void)f.engine.injectTcCommand("ABORT");
    ares::sim::clock::advanceMs(1U);
    f.engine.tick(ares::sim::clock::nowMs());

    f.engine.getSnapshot(snap);
    // Engine remains RUNNING in SAFE; was NOT force-deactivated.
    TEST_ASSERT_EQUAL(EngineStatus::RUNNING, snap.status);
    TEST_ASSERT_EQUAL_STRING("SAFE", snap.stateName);
}

// ─────────────────────────────────────────────────────────────────────────────
// Pause: tick() is a no-op while execution is disabled.
// ─────────────────────────────────────────────────────────────────────────────

void test_pause_stops_tick_execution()
{
    ControlFixture f;
    f.init("/missions/flight.ams", ares::sim::kScriptFlight);
    ares::sim::clock::reset();

    (void)f.engine.activate("flight.ams");
    (void)f.engine.arm();

    // Enter FLIGHT state.
    f.engine.tick(ares::sim::clock::nowMs());

    // Pause.
    f.engine.setExecutionEnabled(false);

    EngineSnapshot snap{};
    f.engine.getSnapshot(snap);
    TEST_ASSERT_EQUAL(EngineStatus::LOADED, snap.status);

    // Advance well past the 5000 ms TIME.elapsed threshold in kScriptFlight.
    ares::sim::clock::advanceMs(10000U);
    f.engine.tick(ares::sim::clock::nowMs());

    // tick() must have been a no-op; state did not change.
    f.engine.getSnapshot(snap);
    TEST_ASSERT_EQUAL(EngineStatus::LOADED, snap.status);
    TEST_ASSERT_EQUAL_STRING("FLIGHT", snap.stateName);
}

// ─────────────────────────────────────────────────────────────────────────────
// Resume: setExecutionEnabled(true) restores RUNNING status.
// ─────────────────────────────────────────────────────────────────────────────

void test_resume_restores_running_status()
{
    ControlFixture f;
    f.init("/missions/flight.ams", ares::sim::kScriptFlight);
    ares::sim::clock::reset();

    (void)f.engine.activate("flight.ams");
    (void)f.engine.arm();
    f.engine.tick(ares::sim::clock::nowMs());

    f.engine.setExecutionEnabled(false);

    EngineSnapshot snap{};
    f.engine.getSnapshot(snap);
    TEST_ASSERT_EQUAL(EngineStatus::LOADED, snap.status);

    f.engine.setExecutionEnabled(true);

    f.engine.getSnapshot(snap);
    TEST_ASSERT_EQUAL(EngineStatus::RUNNING, snap.status);
}

// ─────────────────────────────────────────────────────────────────────────────
// Bug #5 regression: pausing during a hold window must clear the hold timer
// so the condition cannot fire spuriously on the first tick after resuming.
// ─────────────────────────────────────────────────────────────────────────────

void test_pause_clears_transition_hold_windows()
{
    ControlFixture f;
    f.init("/missions/hold.ams", ares::sim::kScriptHoldWindow);
    ares::sim::clock::reset();

    (void)f.engine.activate("hold.ams");
    (void)f.engine.arm();

    // t=0: enter FLIGHT via LAUNCH TC.
    f.engine.tick(ares::sim::clock::nowMs());

    // t=4000: BARO.alt = 210 m > 50 m → hold window armed.
    ares::sim::clock::advanceMs(4000U);
    f.engine.tick(ares::sim::clock::nowMs());

    // Pause immediately: hold window is cleared (Bug #5 fix).
    f.engine.setExecutionEnabled(false);

    // Advance 1001 ms past what would have been the hold deadline.
    // Without the fix the hold timer would accumulate and fire on resume.
    ares::sim::clock::advanceMs(1001U);

    // Resume.
    f.engine.setExecutionEnabled(true);

    // First tick after resume: hold window was reset, so it must restart
    // from nowMs; elapsed = 0 < 1000 ms → transition must NOT fire yet.
    f.engine.tick(ares::sim::clock::nowMs());

    EngineSnapshot snap{};
    f.engine.getSnapshot(snap);
    TEST_ASSERT_EQUAL_STRING("FLIGHT", snap.stateName);
}

// ─────────────────────────────────────────────────────────────────────────────
// requestTelemetry: emits exactly one HK frame on demand (AMS-4.3.1).
// ─────────────────────────────────────────────────────────────────────────────

void test_request_telemetry_emits_hk()
{
    ControlFixture f;
    f.init("/missions/flight.ams", ares::sim::kScriptFlight);
    ares::sim::clock::reset();

    (void)f.engine.activate("flight.ams");
    (void)f.engine.arm();

    // Enter FLIGHT state (has one HK slot: every 1000 ms).
    f.engine.tick(ares::sim::clock::nowMs());

    // Discard EVENT frame sent on FLIGHT on_enter.
    f.radio.resetCapture();

    // requestTelemetry must send one HK frame regardless of the slot timer.
    const bool sent = f.engine.requestTelemetry(ares::sim::clock::nowMs());
    TEST_ASSERT_TRUE(sent);
    TEST_ASSERT_EQUAL(1U, f.radio.sendCount());
}

// ─────────────────────────────────────────────────────────────────────────────
// notifyPyroFired: STATUS_PYRO_A_FIRED bit is set after channel 0 fires.
// ─────────────────────────────────────────────────────────────────────────────

void test_notify_pyro_a_sets_status_bit()
{
    ControlFixture f;
    f.init("/missions/flight.ams", ares::sim::kScriptFlight);

    f.engine.notifyPyroFired(0U);

    const ares::proto::StatusBits bits = f.engine.getStatusBits();
    TEST_ASSERT_TRUE((bits & ares::proto::STATUS_PYRO_A_FIRED) != 0U);
}

// ─────────────────────────────────────────────────────────────────────────────
// notifyPyroFired: STATUS_PYRO_B_FIRED bit is set after channel 1 fires.
// ─────────────────────────────────────────────────────────────────────────────

void test_notify_pyro_b_sets_status_bit()
{
    ControlFixture f;
    f.init("/missions/flight.ams", ares::sim::kScriptFlight);

    f.engine.notifyPyroFired(1U);

    const ares::proto::StatusBits bits = f.engine.getStatusBits();
    TEST_ASSERT_TRUE((bits & ares::proto::STATUS_PYRO_B_FIRED) != 0U);
}

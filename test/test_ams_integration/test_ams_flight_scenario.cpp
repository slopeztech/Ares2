/**
 * @file  test_ams_flight_scenario.cpp
 * @brief SITL integration tests — full mission scenario replay.
 *
 * Uses a deterministic FlightProfile and time-driven tick loop to exercise
 * the AMS engine through a realistic multi-state mission, verifying state
 * transitions, telemetry emission, and sensor-driven conditions.
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

// ── Flight profile: low-speed ascent to 300 m, then hold ────────────────────

static const ares::sim::FlightProfile kFlightProfile = {
    .count   = 5U,
    .samples = {
        //  t=0 s  : ground at 0 m
        { .timeMs = 0U,     .gpsLatDeg = 40.416f, .gpsLonDeg = -3.703f,
          .gpsAltM = 0.0f,  .gpsSpeedKmh = 0.0f, .gpsSats = 9U, .gpsFix = true,
          .baroAltM = 0.0f,  .baroPressurePa = 101325.0f, .baroTempC = 20.0f,
          .accelX = 0.0f,   .accelY = 0.0f, .accelZ = 9.81f, .imuTempC = 25.0f },
        //  t=2 s  : liftoff, accelerating
        { .timeMs = 2000U,  .gpsLatDeg = 40.416f, .gpsLonDeg = -3.703f,
          .gpsAltM = 20.0f, .gpsSpeedKmh = 36.0f, .gpsSats = 9U, .gpsFix = true,
          .baroAltM = 20.0f, .baroPressurePa = 99000.0f, .baroTempC = 19.0f,
          .accelX = 0.5f,   .accelY = 0.0f, .accelZ = 14.0f, .imuTempC = 26.0f },
        //  t=4 s  : mid-ascent, past 200 m threshold
        { .timeMs = 4000U,  .gpsLatDeg = 40.417f, .gpsLonDeg = -3.703f,
          .gpsAltM = 210.0f, .gpsSpeedKmh = 72.0f, .gpsSats = 9U, .gpsFix = true,
          .baroAltM = 210.0f, .baroPressurePa = 98700.0f, .baroTempC = 17.0f,
          .accelX = 0.3f,   .accelY = 0.0f, .accelZ = 12.0f, .imuTempC = 27.0f },
        //  t=8 s  : apogee hold at 300 m
        { .timeMs = 8000U,  .gpsLatDeg = 40.418f, .gpsLonDeg = -3.703f,
          .gpsAltM = 300.0f, .gpsSpeedKmh = 5.0f, .gpsSats = 9U, .gpsFix = true,
          .baroAltM = 300.0f, .baroPressurePa = 97700.0f, .baroTempC = 15.0f,
          .accelX = 0.0f,   .accelY = 0.0f, .accelZ = 9.81f, .imuTempC = 28.0f },
        //  t=15 s : recovery hold
        { .timeMs = 15000U, .gpsLatDeg = 40.418f, .gpsLonDeg = -3.702f,
          .gpsAltM = 300.0f, .gpsSpeedKmh = 3.0f, .gpsSats = 9U, .gpsFix = true,
          .baroAltM = 300.0f, .baroPressurePa = 97700.0f, .baroTempC = 15.0f,
          .accelX = 0.0f,   .accelY = 0.0f, .accelZ = 9.81f, .imuTempC = 28.0f },
    }
};

// ── Helper: build a fully initialised engine ─────────────────────────────────

struct FlightFixture
{
    ares::sim::SimStorageDriver storage;
    ares::sim::SimGpsDriver     gps{kFlightProfile};
    ares::sim::SimBaroDriver    baro{kFlightProfile};
    ares::sim::SimImuDriver     imu{kFlightProfile};
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

    void init(const char* path, const char* content)
    {
        // Initialise all drivers before the engine begins so that
        // sensor reads and radio sends are operational from the first tick.
        (void)storage.begin();
        storage.registerFile(path, content);
        (void)gps.begin();
        (void)baro.begin();
        (void)imu.begin();
        (void)radio.begin();
        (void)engine.begin();
    }
};

// ── Tests ────────────────────────────────────────────────────────────────────

void test_flight_script_activates_successfully()
{
    FlightFixture f;
    f.init("/missions/flight.ams", ares::sim::kScriptFlight);

    TEST_ASSERT_TRUE(f.engine.activate("flight.ams"));

    EngineSnapshot snap{};
    f.engine.getSnapshot(snap);
    TEST_ASSERT_EQUAL(EngineStatus::LOADED, snap.status);
    TEST_ASSERT_EQUAL_STRING("WAIT", snap.stateName);
}

void test_flight_arm_transitions_to_running()
{
    FlightFixture f;
    f.init("/missions/flight.ams", ares::sim::kScriptFlight);
    (void)f.engine.activate("flight.ams");

    TEST_ASSERT_TRUE(f.engine.arm());

    EngineSnapshot snap{};
    f.engine.getSnapshot(snap);
    TEST_ASSERT_EQUAL(EngineStatus::RUNNING, snap.status);
}

void test_flight_wait_advances_to_flight_state()
{
    FlightFixture f;
    f.init("/missions/flight.ams", ares::sim::kScriptFlight);
    (void)f.engine.activate("flight.ams");
    (void)f.engine.arm();  // injects LAUNCH TC

    ares::sim::clock::reset();
    f.engine.tick(ares::sim::clock::nowMs());

    EngineSnapshot snap{};
    f.engine.getSnapshot(snap);
    // LAUNCH TC should have fired the WAIT→FLIGHT transition.
    TEST_ASSERT_EQUAL_STRING("FLIGHT", snap.stateName);
}

void test_flight_emits_hk_telemetry_frames()
{
    FlightFixture f;
    f.init("/missions/flight.ams", ares::sim::kScriptFlight);
    (void)f.engine.activate("flight.ams");
    (void)f.engine.arm();

    ares::sim::clock::reset();

    // Advance into FLIGHT state (one tick at t=0 consumes the LAUNCH TC).
    f.engine.tick(ares::sim::clock::nowMs());

    f.radio.resetCapture();   // reset after on_enter EVENT frame

    // Tick at t=1000 ms: first HK.report slot is due (every 1000 ms).
    ares::sim::clock::advanceMs(1000U);
    f.engine.tick(ares::sim::clock::nowMs());

    TEST_ASSERT_GREATER_THAN(0U, f.radio.sendCount());
    TEST_ASSERT_GREATER_THAN(0U, f.radio.lastFrameLen());
}

void test_flight_time_transition_fires_at_5s()
{
    FlightFixture f;
    f.init("/missions/flight.ams", ares::sim::kScriptFlight);
    (void)f.engine.activate("flight.ams");
    (void)f.engine.arm();

    ares::sim::clock::reset();

    // Tick loop: 100 ms steps up to 6000 ms — the TIME.elapsed > 5000 transition
    // should fire somewhere in the 5000–6000 ms window.
    static constexpr uint32_t kStepMs = 100U;
    static constexpr uint32_t kMaxMs  = 6000U;

    for (uint32_t t = 0U; t <= kMaxMs; t += kStepMs)
    {
        ares::sim::clock::reset();
        ares::sim::clock::advanceMs(t);
        f.engine.tick(ares::sim::clock::nowMs());
    }

    EngineSnapshot snap{};
    f.engine.getSnapshot(snap);
    // When the engine reaches COMPLETE, running_ is false and getSnapshot()
    // reports stateName="IDLE" — verify via status instead.  COMPLETE is
    // only reachable by passing through END via the TIME.elapsed > 5000
    // transition, which is exactly what this test verifies.
    TEST_ASSERT_EQUAL(EngineStatus::COMPLETE, snap.status);
}

void test_flight_reaches_complete_after_end_state()
{
    FlightFixture f;
    f.init("/missions/flight.ams", ares::sim::kScriptFlight);
    (void)f.engine.activate("flight.ams");
    (void)f.engine.arm();

    ares::sim::clock::reset();

    // Run enough ticks to pass the 5 s threshold and enter END.
    // Then one more tick in END → COMPLETE.
    static constexpr uint32_t kStepMs = 500U;
    static constexpr uint32_t kMaxMs  = 7000U;

    for (uint32_t t = 0U; t <= kMaxMs; t += kStepMs)
    {
        ares::sim::clock::reset();
        ares::sim::clock::advanceMs(t);
        f.engine.tick(ares::sim::clock::nowMs());
    }

    EngineSnapshot snap{};
    f.engine.getSnapshot(snap);
    TEST_ASSERT_EQUAL(EngineStatus::COMPLETE, snap.status);
}

void test_sensor_transition_fires_on_altitude_threshold()
{
    FlightFixture f;
    f.init("/missions/sensor.ams", ares::sim::kScriptSensorTransition);
    (void)f.engine.activate("sensor.ams");
    (void)f.engine.arm();

    ares::sim::clock::reset();

    // The flight profile crosses BARO.alt = 200 at ~t=4000 ms.
    // Run ticks at 200 ms steps up to 5 s.
    static constexpr uint32_t kStepMs = 200U;
    static constexpr uint32_t kMaxMs  = 5000U;

    for (uint32_t t = 0U; t <= kMaxMs; t += kStepMs)
    {
        ares::sim::clock::reset();
        ares::sim::clock::advanceMs(t);
        f.engine.tick(ares::sim::clock::nowMs());
    }

    EngineSnapshot snap{};
    f.engine.getSnapshot(snap);

    // Engine should be in RECOVERY or END (transition fired past 200 m).
    const bool inRecoveryOrEnd =
        (strcmp(snap.stateName, "RECOVERY") == 0) ||
        (strcmp(snap.stateName, "END") == 0);
    TEST_ASSERT_TRUE(inRecoveryOrEnd);
}

void test_multiple_hk_frames_accumulate_correctly()
{
    FlightFixture f;
    f.init("/missions/flight.ams", ares::sim::kScriptFlight);
    (void)f.engine.activate("flight.ams");
    (void)f.engine.arm();

    ares::sim::clock::reset();

    // Enter FLIGHT state.
    f.engine.tick(ares::sim::clock::nowMs());
    f.radio.resetCapture();

    // Advance 3100 ms in 100 ms steps — should trigger 3 HK.report frames
    // (at t=1000, t=2000, t=3000 from FLIGHT state entry).
    for (uint32_t dt = 100U; dt <= 3100U; dt += 100U)
    {
        ares::sim::clock::reset();
        ares::sim::clock::advanceMs(dt);
        f.engine.tick(ares::sim::clock::nowMs());
    }

    // At least 3 HK frames should have been transmitted.
    TEST_ASSERT_GREATER_OR_EQUAL(3U, f.radio.sendCount());
}

void test_hk_slot_starvation_skips_samples()
{
    // AMS-4.4 starvation guard: a single catch-up tick after a 5-period gap
    // must emit exactly ONE HK.report and ONE EVENT.warning (not 5 bursts).
    FlightFixture f;
    f.init("/missions/flight.ams", ares::sim::kScriptFlight);
    (void)f.engine.activate("flight.ams");
    (void)f.engine.arm();  // pre-injects LAUNCH TC

    ares::sim::clock::reset();

    // t=0: consume LAUNCH TC → WAIT → FLIGHT (on_enter event queued).
    f.engine.tick(ares::sim::clock::nowMs());

    EngineSnapshot snap{};
    f.engine.getSnapshot(snap);
    TEST_ASSERT_EQUAL_STRING("FLIGHT", snap.stateName);

    // t=1: drain the deferred on_enter LIFTOFF event so the radio counter
    // starts clean before the starvation tick.
    ares::sim::clock::advanceMs(1U);
    f.engine.tick(ares::sim::clock::nowMs());

    // Clear counters so only the starvation tick is measured.
    f.radio.resetCapture();

    // Jump 3 000 ms (= 3× the 1 000 ms HK period) in a single tick.
    // The kScriptFlight TIME.elapsed > 5 000 ms transition must NOT fire
    // here (3001 ms < 5000 ms), leaving the engine in FLIGHT.
    // Without the guard this leaves the timer 2 000 ms behind and produces
    // 2 extra burst HK frames on subsequent ticks.
    ares::sim::clock::advanceMs(3000U);
    f.engine.tick(ares::sim::clock::nowMs());

    // Guard: 1 HK frame + 1 EVENT.warning (skipped 2 samples).
    TEST_ASSERT_EQUAL(2U, f.radio.sendCount());

    // After the reset the next regular tick must produce exactly 1 HK frame
    // and no further warnings.
    f.radio.resetCapture();
    ares::sim::clock::advanceMs(1000U);
    f.engine.tick(ares::sim::clock::nowMs());
    TEST_ASSERT_EQUAL(1U, f.radio.sendCount());
}

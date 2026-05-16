/**
 * @file  test_ams_pulse.cpp
 * @brief SITL integration tests — AMS PULSE.fire command execution (AMS-4.17).
 *
 * Covers: pulse channel A/B firing on state entry, default vs. override
 * duration, notifyPulseFired status bits, execution guard (disabled engine),
 * and null-driver graceful skip.
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
#include "sim_pulse_driver.h"

using ares::ams::MissionScriptEngine;
using ares::ams::EngineSnapshot;
using ares::ams::EngineStatus;

// ── Minimal flight profile (no altitude/speed needed for pulse tests) ─────────

static const ares::sim::FlightProfile kPulseProfile = {
    .count   = 1U,
    .samples = {
        { .timeMs =  0U, .gpsLatDeg = 40.0f, .gpsLonDeg = -3.0f,
          .gpsAltM =  0.0f, .gpsSpeedKmh = 0.0f, .gpsSats = 9U, .gpsFix = true,
          .baroAltM = 0.0f, .baroPressurePa = 101325.0f, .baroTempC = 20.0f,
          .accelX = 0.0f, .accelY = 0.0f, .accelZ = 9.81f, .imuTempC = 25.0f },
    }
};

// ── Fixture ───────────────────────────────────────────────────────────────────

struct PulseFixture
{
    ares::sim::SimStorageDriver storage;
    ares::sim::SimGpsDriver     gps{kPulseProfile};
    ares::sim::SimBaroDriver    baro{kPulseProfile};
    ares::sim::SimImuDriver     imu{kPulseProfile};
    ares::sim::SimRadioDriver   radio;
    ares::sim::SimPulseDriver   pulse;

    GpsEntry  gpsEntry  = { "SIM_GPS",  &gps   };
    BaroEntry baroEntry = { "SIM_BARO", &baro  };
    ComEntry  comEntry  = { "SIM_COM",  &radio };
    ImuEntry  imuEntry  = { "SIM_IMU",  &imu   };

    MissionScriptEngine engine{
        storage,
        &gpsEntry,  1U,
        &baroEntry, 1U,
        &comEntry,  1U,
        &imuEntry,  1U,
        &pulse
    };

    void init(const char* path, const char* content)
    {
        (void)storage.begin();
        storage.registerFile(path, content);
        (void)gps.begin();
        (void)baro.begin();
        (void)imu.begin();
        (void)radio.begin();
        (void)pulse.begin();
        (void)engine.begin();
    }

    /** Activate + arm + tick once.
     *  arm() pre-queues a LAUNCH TC, so the first tick transitions WAIT→DEPLOY
     *  and executes DEPLOY's on_enter (including any PULSE.fire actions).
     */
    void activateAndArm(const char* filename)
    {
        ares::sim::clock::reset();
        (void)engine.activate(filename);
        (void)engine.arm();
        engine.tick(ares::sim::clock::nowMs());
    }

    /** Inject an additional LAUNCH TC and tick — for scripts where arm() alone
     *  is not enough to trigger the desired transition. */
    void injectLaunchAndTick()
    {
        (void)engine.injectTcCommand("LAUNCH");
        ares::sim::clock::advanceMs(1U);
        engine.tick(ares::sim::clock::nowMs());
    }
};

// ── Tests ─────────────────────────────────────────────────────────────────────

// ─────────────────────────────────────────────────────────────────────────────
// AMS-4.17: PULSE.fire A fires drogue channel when entering the DEPLOY state.
// ─────────────────────────────────────────────────────────────────────────────

void test_pulse_fire_a_calls_driver_on_state_entry()
{
    PulseFixture f;
    f.init("/missions/pulse.ams", ares::sim::kScriptPulseFire);

    // arm() pre-queues LAUNCH TC → first tick fires WAIT→DEPLOY.
    f.activateAndArm("pulse.ams");

    EngineSnapshot snap{};
    f.engine.getSnapshot(snap);
    TEST_ASSERT_EQUAL_STRING("DEPLOY", snap.stateName);

    // Drogue channel must have been fired exactly once on DEPLOY entry.
    TEST_ASSERT_EQUAL(1U, f.pulse.getFireCount(PulseChannel::CH_A));
}

// ─────────────────────────────────────────────────────────────────────────────
// AMS-4.17: PULSE.fire B fires main channel when entering the DEPLOY state.
// ─────────────────────────────────────────────────────────────────────────────

void test_pulse_fire_b_calls_driver_on_state_entry()
{
    PulseFixture f;
    f.init("/missions/pulse.ams", ares::sim::kScriptPulseFire);
    f.activateAndArm("pulse.ams");

    TEST_ASSERT_EQUAL(1U, f.pulse.getFireCount(PulseChannel::CH_B));
}

// ─────────────────────────────────────────────────────────────────────────────
// AMS-4.17: After PULSE.fire A, the engine STATUS_PULSE_A_FIRED bit is set.
// ─────────────────────────────────────────────────────────────────────────────

void test_pulse_fire_sets_status_bit_a()
{
    PulseFixture f;
    f.init("/missions/pulse.ams", ares::sim::kScriptPulseFire);

    // Before activating — status bits must be clear.
    TEST_ASSERT_TRUE((f.engine.getStatusBits() & ares::proto::STATUS_PULSE_A_FIRED) == 0U);

    // arm() pre-queues LAUNCH → first tick fires DEPLOY on_enter → PULSE.fire A.
    f.activateAndArm("pulse.ams");

    TEST_ASSERT_TRUE((f.engine.getStatusBits() & ares::proto::STATUS_PULSE_A_FIRED) != 0U);
}

// ─────────────────────────────────────────────────────────────────────────────
// AMS-4.17: After PULSE.fire B, the engine STATUS_PULSE_B_FIRED bit is set.
// ─────────────────────────────────────────────────────────────────────────────

void test_pulse_fire_sets_status_bit_b()
{
    PulseFixture f;
    f.init("/missions/pulse.ams", ares::sim::kScriptPulseFire);

    TEST_ASSERT_TRUE((f.engine.getStatusBits() & ares::proto::STATUS_PULSE_B_FIRED) == 0U);

    f.activateAndArm("pulse.ams");

    TEST_ASSERT_TRUE((f.engine.getStatusBits() & ares::proto::STATUS_PULSE_B_FIRED) != 0U);
}

// ─────────────────────────────────────────────────────────────────────────────
// AMS-4.17: PULSE.fire A 500ms uses the override duration, not the default.
// ─────────────────────────────────────────────────────────────────────────────

void test_pulse_fire_duration_override()
{
    PulseFixture f;
    f.init("/missions/pulse_dur.ams", ares::sim::kScriptPulseDuration);

    // arm() pre-queues LAUNCH → first tick fires WAIT→FIRE with 500ms override.
    f.activateAndArm("pulse_dur.ams");

    EngineSnapshot snap{};
    f.engine.getSnapshot(snap);
    TEST_ASSERT_EQUAL_STRING("FIRE", snap.stateName);

    // Driver must have recorded exactly 500 ms (not the default ares::FIRE_DURATION_MS).
    TEST_ASSERT_EQUAL(500U, f.pulse.getLastDurationMs(PulseChannel::CH_A));
}

// ─────────────────────────────────────────────────────────────────────────────
// AMS-4.17: Pulse does NOT fire when execution is disabled (paused engine).
// ─────────────────────────────────────────────────────────────────────────────

void test_pulse_fire_only_when_execution_enabled()
{
    PulseFixture f;
    f.init("/missions/pulse.ams", ares::sim::kScriptPulseFire);

    (void)f.engine.activate("pulse.ams");
    (void)f.engine.arm();          // sets RUNNING + pre-queues LAUNCH TC

    // Immediately pause before the first tick.
    f.engine.setExecutionEnabled(false);

    f.engine.tick(ares::sim::clock::nowMs());

    // With execution disabled, no transitions fire and pulse must NOT fire.
    TEST_ASSERT_EQUAL(0U, f.pulse.getFireCount(PulseChannel::CH_A));
    TEST_ASSERT_EQUAL(0U, f.pulse.getFireCount(PulseChannel::CH_B));
}

// ─────────────────────────────────────────────────────────────────────────────
// AMS-4.17: Engine with no PulseInterface does NOT crash on PULSE.fire commands.
// ─────────────────────────────────────────────────────────────────────────────

void test_pulse_fire_no_driver_no_crash()
{
    ares::sim::SimStorageDriver storage;
    ares::sim::SimGpsDriver     gps{kPulseProfile};
    ares::sim::SimBaroDriver    baro{kPulseProfile};
    ares::sim::SimImuDriver     imu{kPulseProfile};
    ares::sim::SimRadioDriver   radio;

    GpsEntry  gpsEntry  = { "SIM_GPS",  &gps   };
    BaroEntry baroEntry = { "SIM_BARO", &baro  };
    ComEntry  comEntry  = { "SIM_COM",  &radio };
    ImuEntry  imuEntry  = { "SIM_IMU",  &imu   };

    // No PulseInterface passed — pulseIface_ is null.
    MissionScriptEngine engine{
        storage,
        &gpsEntry,  1U,
        &baroEntry, 1U,
        &comEntry,  1U,
        &imuEntry,  1U,
        nullptr  // pulse = null
    };

    (void)storage.begin();
    storage.registerFile("/missions/pulse.ams", ares::sim::kScriptPulseFire);
    (void)gps.begin();
    (void)baro.begin();
    (void)imu.begin();
    (void)radio.begin();
    (void)engine.begin();

    (void)engine.activate("pulse.ams");
    (void)engine.arm();
    // arm() pre-queues LAUNCH → first tick transitions WAIT→DEPLOY and runs
    // executePulseActionsLocked with pulseIface_=nullptr.  Must not crash.
    engine.tick(ares::sim::clock::nowMs());

    // Engine must NOT crash; it should be RUNNING in DEPLOY (null pulse = silent skip).
    EngineSnapshot snap{};
    engine.getSnapshot(snap);
    TEST_ASSERT_EQUAL(EngineStatus::RUNNING, snap.status);
    TEST_ASSERT_EQUAL_STRING("DEPLOY", snap.stateName);
}

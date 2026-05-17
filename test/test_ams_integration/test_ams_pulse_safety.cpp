/**
 * @file  test_ams_pulse_safety.cpp
 * @brief SITL integration tests — AMS-4.19 Pulse Safety System.
 *
 * Covers all five safety directives introduced in AMS-4.19:
 *   AMS-4.19.1  PULSE.arm two-phase fire (channel must be armed before PULSE.fire)
 *   AMS-4.19.2  pulse.min_altitude  (altitude guard)
 *   AMS-4.19.3  pulse.arm_timeout   (auto-disarm after N ms)
 *   AMS-4.19.4  pulse.require_continuity (bridgewire continuity check)
 *   AMS-4.19.5  pulse.safe_delay    (minimum elapsed time after arm() before any PULSE.fire)
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

// ── Helper constants ──────────────────────────────────────────────────────────

static const ares::sim::FlightProfile kGroundProfile = {
    .count   = 1U,
    .samples = {
        { .timeMs = 0U, .gpsLatDeg = 40.0f, .gpsLonDeg = -3.0f,
          .gpsAltM = 0.0f, .gpsSpeedKmh = 0.0f, .gpsSats = 9U, .gpsFix = true,
          .baroAltM = 0.0f, .baroPressurePa = 101325.0f, .baroTempC = 20.0f,
          .accelX = 0.0f, .accelY = 0.0f, .accelZ = 9.81f, .imuTempC = 25.0f },
    }
};

static const ares::sim::FlightProfile kHighAltProfile = {
    .count   = 1U,
    .samples = {
        { .timeMs = 0U, .gpsLatDeg = 40.0f, .gpsLonDeg = -3.0f,
          .gpsAltM = 500.0f, .gpsSpeedKmh = 0.0f, .gpsSats = 9U, .gpsFix = true,
          .baroAltM = 500.0f, .baroPressurePa = 95000.0f, .baroTempC = 15.0f,
          .accelX = 0.0f, .accelY = 0.0f, .accelZ = 9.81f, .imuTempC = 25.0f },
    }
};

// ── Fixture ───────────────────────────────────────────────────────────────────

struct SafetyFixture
{
    const ares::sim::FlightProfile& profile;

    ares::sim::SimStorageDriver storage;
    ares::sim::SimGpsDriver     gps;
    ares::sim::SimBaroDriver    baro;
    ares::sim::SimImuDriver     imu;
    ares::sim::SimRadioDriver   radio;
    ares::sim::SimPulseDriver   pulse;

    GpsEntry  gpsEntry;
    BaroEntry baroEntry;
    ComEntry  comEntry;
    ImuEntry  imuEntry;

    MissionScriptEngine engine;

    explicit SafetyFixture(const ares::sim::FlightProfile& p = kGroundProfile)
        : profile(p),
          gps(p), baro(p), imu(p),
          gpsEntry  { "SIM_GPS",  &gps   },
          baroEntry { "SIM_BARO", &baro  },
          comEntry  { "SIM_COM",  &radio },
          imuEntry  { "SIM_IMU",  &imu   },
          engine    { storage,
                      &gpsEntry,  1U,
                      &baroEntry, 1U,
                      &comEntry,  1U,
                      &imuEntry,  1U,
                      &pulse }
    {}

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

    /** Activate + arm only (no tick). */
    bool activateAndArm(const char* filename)
    {
        ares::sim::clock::reset();
        if (!engine.activate(filename)) { return false; }
        return engine.arm();
    }

    /** arm() + tick once at given simulated time.
     *
     *  Clock is advanced by 1 ms before arm() so that activationMs_ != 0.
     *  This ensures the AMS-4.19.5 safe_delay gate (which is disabled when
     *  activationMs_ == 0) is properly exercised.  Elapsed time seen by the
     *  gate is therefore tickMs milliseconds (not tickMs + 1).
     */
    void armAndTickAt(const char* filename, uint64_t tickMs)
    {
        ares::sim::clock::reset();
        ares::sim::clock::advanceMs(1U);               // activationMs_ = 1, not 0
        (void)engine.activate(filename);
        (void)engine.arm();
        ares::sim::clock::advanceMs(tickMs);
        engine.tick(ares::sim::clock::nowMs());
    }

    /** Activate + arm + advance time + tick. */
    void advanceAndTick(uint64_t advanceMs)
    {
        ares::sim::clock::advanceMs(advanceMs);
        engine.tick(ares::sim::clock::nowMs());
    }
};

// ─────────────────────────────────────────────────────────────────────────────
// AMS-4.19.5: pulse.safe_delay blocks PULSE.fire if elapsed < safe_delay
// ─────────────────────────────────────────────────────────────────────────────

static const char kScriptSafeDelay[] =
    "include SIM_GPS as GPS\n"
    "include SIM_BARO as BARO\n"
    "include SIM_COM as COM\n"
    "include SIM_IMU as IMU\n"
    "\n"
    "pus.apid = 1\n"
    "pus.service 3 as HK\n"
    "pus.service 5 as EVENT\n"
    "pus.service 1 as TC\n"
    "\n"
    "pulse.channel A as DROGUE\n"
    "pulse.safe_delay 5000\n"
    "\n"
    "state WAIT:\n"
    "  on_enter:\n"
    "    EVENT.info \"WAITING\"\n"
    "  transition to FIRE when TC.command == LAUNCH\n"
    "\n"
    "state FIRE:\n"
    "  on_enter:\n"
    "    PULSE.fire DROGUE\n"
    "    EVENT.info \"FIRE\"\n";

void test_pulse_safe_delay_blocks_early_fire()
{
    SafetyFixture f;
    f.init("/missions/sd.ams", kScriptSafeDelay);

    // arm() → activationMs_ = ~0 ms.  Tick immediately (1 ms elapsed).
    f.armAndTickAt("sd.ams", 1U);

    // PULSE.fire blocked because 1 ms < 5000 ms safe_delay.
    TEST_ASSERT_EQUAL(0U, f.pulse.getFireCount(PulseChannel::CH_A));
}

void test_pulse_safe_delay_allows_fire_after_delay()
{
    SafetyFixture f;
    f.init("/missions/sd.ams", kScriptSafeDelay);

    // arm() at t=0, tick at t=6000 ms (> 5000 ms safe_delay).
    f.armAndTickAt("sd.ams", 6000U);

    // Channel A must have fired.
    TEST_ASSERT_EQUAL(1U, f.pulse.getFireCount(PulseChannel::CH_A));
}

// ─────────────────────────────────────────────────────────────────────────────
// AMS-4.19.2: pulse.min_altitude blocks / allows based on baro altitude
// ─────────────────────────────────────────────────────────────────────────────

static const char kScriptMinAlt[] =
    "include SIM_GPS as GPS\n"
    "include SIM_BARO as BARO\n"
    "include SIM_COM as COM\n"
    "include SIM_IMU as IMU\n"
    "\n"
    "pus.apid = 1\n"
    "pus.service 3 as HK\n"
    "pus.service 5 as EVENT\n"
    "pus.service 1 as TC\n"
    "\n"
    "pulse.channel A as DROGUE\n"
    "pulse.min_altitude 200\n"
    "\n"
    "state WAIT:\n"
    "  on_enter:\n"
    "    EVENT.info \"WAITING\"\n"
    "  transition to FIRE when TC.command == LAUNCH\n"
    "\n"
    "state FIRE:\n"
    "  on_enter:\n"
    "    PULSE.fire DROGUE\n"
    "    EVENT.info \"FIRE\"\n";

void test_pulse_min_altitude_blocks_below_threshold()
{
    // kGroundProfile has baroAltM = 0 m, min required is 200 m.
    SafetyFixture f(kGroundProfile);
    f.init("/missions/ma.ams", kScriptMinAlt);
    f.armAndTickAt("ma.ams", 1U);

    // PULSE.fire blocked because alt (0 m) < min_altitude (200 m).
    TEST_ASSERT_EQUAL(0U, f.pulse.getFireCount(PulseChannel::CH_A));
}

void test_pulse_min_altitude_allows_above_threshold()
{
    // kHighAltProfile has baroAltM = 500 m, min required is 200 m.
    SafetyFixture f(kHighAltProfile);
    f.init("/missions/ma.ams", kScriptMinAlt);
    f.armAndTickAt("ma.ams", 1U);

    // PULSE.fire allowed because alt (500 m) >= min_altitude (200 m).
    TEST_ASSERT_EQUAL(1U, f.pulse.getFireCount(PulseChannel::CH_A));
}

// ─────────────────────────────────────────────────────────────────────────────
// AMS-4.19.4: pulse.require_continuity parse error when no hardware pin wired
// ─────────────────────────────────────────────────────────────────────────────

static const char kScriptRequireCont[] =
    "include SIM_GPS as GPS\n"
    "include SIM_BARO as BARO\n"
    "include SIM_COM as COM\n"
    "include SIM_IMU as IMU\n"
    "\n"
    "pus.apid = 1\n"
    "pus.service 3 as HK\n"
    "pus.service 5 as EVENT\n"
    "pus.service 1 as TC\n"
    "\n"
    "pulse.channel A as DROGUE\n"
    "pulse.require_continuity A\n"
    "\n"
    "state WAIT:\n"
    "  on_enter:\n"
    "    EVENT.info \"WAITING\"\n"
    "  transition to FIRE when TC.command == LAUNCH\n"
    "\n"
    "state FIRE:\n"
    "  on_enter:\n"
    "    PULSE.fire DROGUE\n"
    "    EVENT.info \"FIRE\"\n";

void test_pulse_require_continuity_parse_error_no_pin()
{
    // SimPulseDriver defaults hasContPin = false for all channels.
    SafetyFixture f;
    f.init("/missions/rc.ams", kScriptRequireCont);

    // activate() must fail because ch A has no hardware continuity pin.
    const bool ok = f.engine.activate("rc.ams");
    TEST_ASSERT_FALSE(ok);

    EngineSnapshot snap{};
    f.engine.getSnapshot(snap);
    TEST_ASSERT_EQUAL(EngineStatus::ERROR, snap.status);
    TEST_ASSERT_NOT_NULL(strstr(snap.lastError, "no hardware continuity"));
}

void test_pulse_require_continuity_blocks_open_circuit()
{
    // Enable cont pin so the directive parses, then set open circuit AFTER
    // init() so that SimPulseDriver::begin() → reset() does not clobber it.
    SafetyFixture f;
    f.pulse.setHasContPin(PulseChannel::CH_A, true);
    f.init("/missions/rc.ams", kScriptRequireCont);
    f.pulse.setContinuity(PulseChannel::CH_A, false);  // set after begin()/reset()
    f.armAndTickAt("rc.ams", 1U);

    // PULSE.fire blocked because continuity is false.
    TEST_ASSERT_EQUAL(0U, f.pulse.getFireCount(PulseChannel::CH_A));
}

void test_pulse_require_continuity_allows_intact_circuit()
{
    // Enable cont pin and continuity is intact (default = true).
    SafetyFixture f;
    f.pulse.setHasContPin(PulseChannel::CH_A, true);
    f.init("/missions/rc.ams", kScriptRequireCont);
    f.armAndTickAt("rc.ams", 1U);

    // PULSE.fire allowed because continuity is true.
    TEST_ASSERT_EQUAL(1U, f.pulse.getFireCount(PulseChannel::CH_A));
}

// ─────────────────────────────────────────────────────────────────────────────
// AMS-4.19.1: PULSE.arm two-phase fire
// ─────────────────────────────────────────────────────────────────────────────

// 3-state: WAIT --(LAUNCH)--> ARM (PULSE.arm) --(ABORT)--> FIRE (PULSE.fire).
//
// IMPORTANT: the PULSE.arm action must live in a state entered AFTER arm() so
// that executionEnabled_==true when sendOnEnterEventLocked() fires.  The initial
// WAIT state's on_enter is called at activate() time (executionEnabled_=false),
// meaning any PULSE.arm action placed there is suppressed and pulseArmed_[] stays
// false even after arm() is called.
static const char kScriptTwoPhaseArm[] =
    "include SIM_GPS as GPS\n"
    "include SIM_BARO as BARO\n"
    "include SIM_COM as COM\n"
    "include SIM_IMU as IMU\n"
    "\n"
    "pus.apid = 1\n"
    "pus.service 3 as HK\n"
    "pus.service 5 as EVENT\n"
    "pus.service 1 as TC\n"
    "\n"
    "pulse.channel A as DROGUE\n"
    "\n"
    "state WAIT:\n"
    "  on_enter:\n"
    "    EVENT.info \"WAITING\"\n"
    "  transition to ARM when TC.command == LAUNCH\n"
    "\n"
    "state ARM:\n"
    "  on_enter:\n"
    "    PULSE.arm DROGUE\n"
    "    EVENT.info \"ARMED\"\n"
    "  transition to FIRE when TC.command == ABORT\n"
    "\n"
    "state FIRE:\n"
    "  on_enter:\n"
    "    PULSE.fire DROGUE\n"
    "    EVENT.info \"FIRE\"\n";

// WAIT --(LAUNCH)--> ARM (PULSE.arm + PULSE.fire in same on_enter, terminal state).
// executePulseArmActionsLocked runs before executePulseActionsLocked within
// sendOnEnterEventLocked, so the arm token is present when the fire is attempted.
static const char kScriptArmFireSameState[] =
    "include SIM_GPS as GPS\n"
    "include SIM_BARO as BARO\n"
    "include SIM_COM as COM\n"
    "include SIM_IMU as IMU\n"
    "\n"
    "pus.apid = 1\n"
    "pus.service 3 as HK\n"
    "pus.service 5 as EVENT\n"
    "pus.service 1 as TC\n"
    "\n"
    "pulse.channel A as DROGUE\n"
    "\n"
    "state WAIT:\n"
    "  on_enter:\n"
    "    EVENT.info \"WAITING\"\n"
    "  transition to ARM when TC.command == LAUNCH\n"
    "\n"
    "state ARM:\n"
    "  on_enter:\n"
    "    PULSE.arm DROGUE\n"
    "    PULSE.fire DROGUE\n"
    "    EVENT.info \"ARMED AND FIRED\"\n";

// PULSE.arm in WAIT's on_enter sets pulseNeedsArm[A]=true at parse time,
// but the runtime arm action is suppressed (executionEnabled_=false during
// activate()).  WAIT transitions to FIRE on ABORT, bypassing any ARM state.
// pulseArmed_[A] is never set → PULSE.fire is blocked.
static const char kScriptArmNeverExecuted[] =
    "include SIM_GPS as GPS\n"
    "include SIM_BARO as BARO\n"
    "include SIM_COM as COM\n"
    "include SIM_IMU as IMU\n"
    "\n"
    "pus.apid = 1\n"
    "pus.service 3 as HK\n"
    "pus.service 5 as EVENT\n"
    "pus.service 1 as TC\n"
    "\n"
    "pulse.channel A as DROGUE\n"
    "\n"
    "state WAIT:\n"
    "  on_enter:\n"
    "    PULSE.arm DROGUE\n"
    "    EVENT.info \"WAITING\"\n"
    "  transition to FIRE when TC.command == ABORT\n"
    "\n"
    "state FIRE:\n"
    "  on_enter:\n"
    "    PULSE.fire DROGUE\n"
    "    EVENT.info \"FIRE\"\n";

void test_pulse_arm_fires_after_two_phase_sequence()
{
    // arm() queues LAUNCH → tick(1) enters ARM via WAIT→ARM.  ARM on_enter
    // executes PULSE.arm (executionEnabled_=true), setting pulseArmed_[A]=true.
    // ABORT then transitions ARM→FIRE; PULSE.fire passes the arm gate.
    SafetyFixture f;
    f.init("/missions/arm.ams", kScriptTwoPhaseArm);

    ares::sim::clock::reset();
    (void)f.engine.activate("arm.ams");
    (void)f.engine.arm();                          // queues LAUNCH

    ares::sim::clock::advanceMs(1U);
    f.engine.tick(ares::sim::clock::nowMs());       // tick 1: WAIT→ARM, arm set

    (void)f.engine.injectTcCommand("ABORT");
    ares::sim::clock::advanceMs(1U);
    f.engine.tick(ares::sim::clock::nowMs());       // tick 2: ARM→FIRE, PULSE.fire

    TEST_ASSERT_EQUAL(1U, f.pulse.getFireCount(PulseChannel::CH_A));
}

void test_pulse_arm_blocked_when_arm_state_skipped()
{
    // PULSE.arm appears in WAIT's on_enter: this sets pulseNeedsArm[A]=true at
    // parse time, but the runtime action is suppressed because the engine calls
    // enterStateLocked(WAIT) at activate() time when executionEnabled_=false.
    // ABORT transitions WAIT→FIRE directly, bypassing any armed state.
    // Since pulseArmed_[A] was never set, the safety gate blocks PULSE.fire.
    SafetyFixture f;
    f.init("/missions/narm.ams", kScriptArmNeverExecuted);

    ares::sim::clock::reset();
    (void)f.engine.activate("narm.ams");
    (void)f.engine.arm();                          // queues LAUNCH

    // tick 1: WAIT has only an ABORT transition; LAUNCH does not match → stays
    ares::sim::clock::advanceMs(1U);
    f.engine.tick(ares::sim::clock::nowMs());

    // ABORT overwrites LAUNCH in pendingTc_; WAIT→FIRE on next tick.
    (void)f.engine.injectTcCommand("ABORT");
    ares::sim::clock::advanceMs(1U);
    f.engine.tick(ares::sim::clock::nowMs());       // tick 2: WAIT→FIRE, arm not set

    // PULSE.fire must be blocked because pulseArmed_[A] == false.
    TEST_ASSERT_EQUAL(0U, f.pulse.getFireCount(PulseChannel::CH_A));
}

void test_pulse_arm_same_state_arm_and_fire()
{
    // arm() queues LAUNCH → tick(1) enters ARM via WAIT→ARM.  ARM's on_enter
    // runs PULSE.arm then PULSE.fire in sequence; the arm token is present when
    // executePulseActionsLocked is called (ordering guaranteed by
    // sendOnEnterEventLocked).
    SafetyFixture f;
    f.init("/missions/af.ams", kScriptArmFireSameState);

    ares::sim::clock::reset();
    (void)f.engine.activate("af.ams");
    (void)f.engine.arm();                          // queues LAUNCH

    ares::sim::clock::advanceMs(1U);
    f.engine.tick(ares::sim::clock::nowMs());       // tick 1: WAIT→ARM, arm+fire

    TEST_ASSERT_EQUAL(1U, f.pulse.getFireCount(PulseChannel::CH_A));
}

// ─────────────────────────────────────────────────────────────────────────────
// AMS-4.19.3: pulse.arm_timeout auto-disarms the channel
// ─────────────────────────────────────────────────────────────────────────────

// Same 3-state flow as kScriptTwoPhaseArm but with pulse.arm_timeout 2000.
static const char kScriptArmTimeout[] =
    "include SIM_GPS as GPS\n"
    "include SIM_BARO as BARO\n"
    "include SIM_COM as COM\n"
    "include SIM_IMU as IMU\n"
    "\n"
    "pus.apid = 1\n"
    "pus.service 3 as HK\n"
    "pus.service 5 as EVENT\n"
    "pus.service 1 as TC\n"
    "\n"
    "pulse.channel A as DROGUE\n"
    "pulse.arm_timeout 2000\n"
    "\n"
    "state WAIT:\n"
    "  on_enter:\n"
    "    EVENT.info \"WAITING\"\n"
    "  transition to ARM when TC.command == LAUNCH\n"
    "\n"
    "state ARM:\n"
    "  on_enter:\n"
    "    PULSE.arm DROGUE\n"
    "    EVENT.info \"ARMED\"\n"
    "  transition to FIRE when TC.command == ABORT\n"
    "\n"
    "state FIRE:\n"
    "  on_enter:\n"
    "    PULSE.fire DROGUE\n"
    "    EVENT.info \"FIRE\"\n";

void test_pulse_arm_timeout_fires_before_expiry()
{
    // arm() at t=0; tick(1) enters ARM (arm set at t=1 ms).
    // ABORT injected at t=1; tick at t=100 ms transitions ARM→FIRE.
    // Arm age = 99 ms << 2000 ms timeout → fire allowed.
    SafetyFixture f;
    f.init("/missions/at.ams", kScriptArmTimeout);

    ares::sim::clock::reset();
    (void)f.engine.activate("at.ams");
    (void)f.engine.arm();

    ares::sim::clock::advanceMs(1U);
    f.engine.tick(ares::sim::clock::nowMs());       // t=1: WAIT→ARM, arm set

    (void)f.engine.injectTcCommand("ABORT");
    ares::sim::clock::advanceMs(99U);              // t=100 ms
    f.engine.tick(ares::sim::clock::nowMs());       // ARM→FIRE, armAge=99ms < 2000ms

    TEST_ASSERT_EQUAL(1U, f.pulse.getFireCount(PulseChannel::CH_A));
}

void test_pulse_arm_timeout_disarms_automatically()
{
    // arm() at t=0; tick(1) enters ARM (arm set at t=1 ms).
    // Advance to t=2002 ms (> 2000 ms): checkPulseArmTimeoutsLocked clears
    // pulseArmed_[A].  ABORT then transitions ARM→FIRE but the arm token is
    // gone, so PULSE.fire is blocked.
    SafetyFixture f;
    f.init("/missions/at.ams", kScriptArmTimeout);

    ares::sim::clock::reset();
    (void)f.engine.activate("at.ams");
    (void)f.engine.arm();

    ares::sim::clock::advanceMs(1U);
    f.engine.tick(ares::sim::clock::nowMs());       // t=1: WAIT→ARM, arm set

    ares::sim::clock::advanceMs(2001U);             // t=2002 ms
    f.engine.tick(ares::sim::clock::nowMs());       // arm_timeout fires, arm cleared

    (void)f.engine.injectTcCommand("ABORT");
    ares::sim::clock::advanceMs(1U);               // t=2003 ms
    f.engine.tick(ares::sim::clock::nowMs());       // ARM→FIRE, arm expired

    TEST_ASSERT_EQUAL(0U, f.pulse.getFireCount(PulseChannel::CH_A));
}

// ─────────────────────────────────────────────────────────────────────────────
// Parser validation tests
// ─────────────────────────────────────────────────────────────────────────────

void test_pulse_safe_delay_out_of_range_fails()
{
    static const char kScript[] =
        "include SIM_GPS as GPS\n"
        "include SIM_BARO as BARO\n"
        "include SIM_COM as COM\n"
        "include SIM_IMU as IMU\n"
        "\n"
        "pus.apid = 1\n"
        "pus.service 3 as HK\n"
        "pus.service 5 as EVENT\n"
        "pus.service 1 as TC\n"
        "\n"
        "pulse.channel A\n"
        "pulse.safe_delay 50\n"  // below minimum 100
        "\n"
        "state WAIT:\n"
        "  on_enter:\n"
        "    EVENT.info \"W\"\n";

    SafetyFixture f;
    f.init("/missions/bad_sd.ams", kScript);
    TEST_ASSERT_FALSE(f.engine.activate("bad_sd.ams"));
    EngineSnapshot snap{};
    f.engine.getSnapshot(snap);
    TEST_ASSERT_EQUAL(EngineStatus::ERROR, snap.status);
    TEST_ASSERT_NOT_NULL(strstr(snap.lastError, "safe_delay"));
}

void test_pulse_min_altitude_out_of_range_fails()
{
    static const char kScript[] =
        "include SIM_GPS as GPS\n"
        "include SIM_BARO as BARO\n"
        "include SIM_COM as COM\n"
        "include SIM_IMU as IMU\n"
        "\n"
        "pus.apid = 1\n"
        "pus.service 3 as HK\n"
        "pus.service 5 as EVENT\n"
        "pus.service 1 as TC\n"
        "\n"
        "pulse.channel A\n"
        "pulse.min_altitude 0\n"  // 0 is below minimum 1
        "\n"
        "state WAIT:\n"
        "  on_enter:\n"
        "    EVENT.info \"W\"\n";

    SafetyFixture f;
    f.init("/missions/bad_ma.ams", kScript);
    TEST_ASSERT_FALSE(f.engine.activate("bad_ma.ams"));
    EngineSnapshot snap{};
    f.engine.getSnapshot(snap);
    TEST_ASSERT_EQUAL(EngineStatus::ERROR, snap.status);
    TEST_ASSERT_NOT_NULL(strstr(snap.lastError, "min_altitude"));
}

void test_pulse_arm_timeout_out_of_range_fails()
{
    static const char kScript[] =
        "include SIM_GPS as GPS\n"
        "include SIM_BARO as BARO\n"
        "include SIM_COM as COM\n"
        "include SIM_IMU as IMU\n"
        "\n"
        "pus.apid = 1\n"
        "pus.service 3 as HK\n"
        "pus.service 5 as EVENT\n"
        "pus.service 1 as TC\n"
        "\n"
        "pulse.channel A\n"
        "pulse.arm_timeout 100\n"  // below minimum 500
        "\n"
        "state WAIT:\n"
        "  on_enter:\n"
        "    EVENT.info \"W\"\n";

    SafetyFixture f;
    f.init("/missions/bad_at.ams", kScript);
    TEST_ASSERT_FALSE(f.engine.activate("bad_at.ams"));
    EngineSnapshot snap{};
    f.engine.getSnapshot(snap);
    TEST_ASSERT_EQUAL(EngineStatus::ERROR, snap.status);
    TEST_ASSERT_NOT_NULL(strstr(snap.lastError, "arm_timeout"));
}

void test_pulse_safety_directives_before_state_required()
{
    static const char kScript[] =
        "include SIM_GPS as GPS\n"
        "include SIM_BARO as BARO\n"
        "include SIM_COM as COM\n"
        "include SIM_IMU as IMU\n"
        "\n"
        "pus.apid = 1\n"
        "pus.service 3 as HK\n"
        "pus.service 5 as EVENT\n"
        "pus.service 1 as TC\n"
        "\n"
        "pulse.channel A\n"
        "\n"
        "state WAIT:\n"
        "  on_enter:\n"
        "    EVENT.info \"W\"\n"
        "\n"
        "pulse.safe_delay 1000\n";  // after state — error

    SafetyFixture f;
    f.init("/missions/bad_order.ams", kScript);
    TEST_ASSERT_FALSE(f.engine.activate("bad_order.ams"));
    EngineSnapshot snap{};
    f.engine.getSnapshot(snap);
    TEST_ASSERT_EQUAL(EngineStatus::ERROR, snap.status);
}

// ─────────────────────────────────────────────────────────────────────────────
// T1: pulseXFired_ flags must clear between sessions (Bug 1 regression test)
// ─────────────────────────────────────────────────────────────────────────────

void test_pulse_status_bits_clear_after_reactivate()
{
    // Session 1: kHighAltProfile → alt=500m ≥ min_alt 200m → PULSE.fire succeeds.
    SafetyFixture f(kHighAltProfile);
    f.init("/missions/ma.ams", kScriptMinAlt);
    f.armAndTickAt("ma.ams", 1U);

    // Confirm channel A fired in session 1.
    TEST_ASSERT_EQUAL(1U, f.pulse.getFireCount(PulseChannel::CH_A));

    // Deactivate: pulseAFired_ must be reset to false (Bug 1 fix).
    f.engine.deactivate();

    // Session 2: re-activate and arm (no tick, so no fire).
    ares::sim::clock::reset();
    (void)f.engine.activate("ma.ams");
    (void)f.engine.arm();

    // getStatusBits() must NOT report STATUS_PULSE_A_FIRED from session 1.
    const ares::proto::StatusBits bits = f.engine.getStatusBits();
    TEST_ASSERT_EQUAL(0U, bits & ares::proto::STATUS_PULSE_A_FIRED);
}

// ─────────────────────────────────────────────────────────────────────────────
// T2: pulse.safe_delay allows fire at exact boundary (elapsed == delay)
// ─────────────────────────────────────────────────────────────────────────────

void test_pulse_safe_delay_allows_at_exact_boundary()
{
    // armAndTickAt advances 1 ms before arm() (activationMs_=1), then
    // advances tickMs=5000 before tick.  elapsed = 5001-1 = 5000 ms.
    // Gate: elapsed < 5000 → 5000 < 5000 = false → fire allowed.
    SafetyFixture f;
    f.init("/missions/sd.ams", kScriptSafeDelay);
    f.armAndTickAt("sd.ams", 5000U);

    TEST_ASSERT_EQUAL(1U, f.pulse.getFireCount(PulseChannel::CH_A));
}

// ─────────────────────────────────────────────────────────────────────────────
// T3: pulse.arm_timeout allows fire at exact boundary (armAge == timeout)
// ─────────────────────────────────────────────────────────────────────────────

void test_pulse_arm_timeout_allows_at_exact_boundary()
{
    // arm() at t=1 (activationMs_=1, queues LAUNCH).
    // tick(2) → WAIT→ARM, PULSE.arm executes, pulseArmedMs_[A]=2.
    // ABORT injected; tick(2002) → armAge = 2002-2 = 2000 ms.
    // Gate: armAge > 2000 → 2000 > 2000 = false → NOT timed out → fire allowed.
    SafetyFixture f;
    f.init("/missions/at.ams", kScriptArmTimeout);

    ares::sim::clock::reset();
    ares::sim::clock::advanceMs(1U);                // t=1
    (void)f.engine.activate("at.ams");
    (void)f.engine.arm();                           // activationMs_=1, queues LAUNCH

    ares::sim::clock::advanceMs(1U);                // t=2
    f.engine.tick(ares::sim::clock::nowMs());        // WAIT→ARM, arm set at t=2

    (void)f.engine.injectTcCommand("ABORT");
    ares::sim::clock::advanceMs(2000U);             // t=2002
    f.engine.tick(ares::sim::clock::nowMs());        // armAge=2000 ms → not expired → FIRE

    TEST_ASSERT_EQUAL(1U, f.pulse.getFireCount(PulseChannel::CH_A));
}

// ─────────────────────────────────────────────────────────────────────────────
// T4: pulse.safe_delay combined with PULSE.arm — delay still blocks when arm set
// ─────────────────────────────────────────────────────────────────────────────

// 3-state design identical to kScriptTwoPhaseArm but with pulse.safe_delay 1000.
static const char kScriptSafeDelayWithArm[] =
    "include SIM_GPS as GPS\n"
    "include SIM_BARO as BARO\n"
    "include SIM_COM as COM\n"
    "include SIM_IMU as IMU\n"
    "\n"
    "pus.apid = 1\n"
    "pus.service 3 as HK\n"
    "pus.service 5 as EVENT\n"
    "pus.service 1 as TC\n"
    "\n"
    "pulse.channel A as DROGUE\n"
    "pulse.safe_delay 1000\n"
    "\n"
    "state WAIT:\n"
    "  on_enter:\n"
    "    EVENT.info \"WAITING\"\n"
    "  transition to ARM when TC.command == LAUNCH\n"
    "\n"
    "state ARM:\n"
    "  on_enter:\n"
    "    PULSE.arm DROGUE\n"
    "    EVENT.info \"ARMED\"\n"
    "  transition to FIRE when TC.command == ABORT\n"
    "\n"
    "state FIRE:\n"
    "  on_enter:\n"
    "    PULSE.fire DROGUE\n"
    "    EVENT.info \"FIRE\"\n";

void test_pulse_safe_delay_combined_with_arm()
{
    // Even after PULSE.arm executes, safe_delay must still block PULSE.fire
    // if elapsed < 1000 ms.  This verifies that the two gates are independent.
    //
    // Sequence:
    //   t=1:   activate, arm() → activationMs_=1, LAUNCH queued.
    //   t=100: tick → WAIT→ARM, PULSE.arm executes (pulseArmed_[A] = true).
    //   t=500: ABORT injected; tick → ARM→FIRE, PULSE.fire attempted.
    //          elapsed = 500-1 = 499 ms < 1000 ms → safe_delay blocks.
    SafetyFixture f;
    f.init("/missions/sda.ams", kScriptSafeDelayWithArm);

    ares::sim::clock::reset();
    ares::sim::clock::advanceMs(1U);                // t=1
    (void)f.engine.activate("sda.ams");
    (void)f.engine.arm();                           // activationMs_=1

    ares::sim::clock::advanceMs(99U);               // t=100
    f.engine.tick(ares::sim::clock::nowMs());        // WAIT→ARM, arm set

    (void)f.engine.injectTcCommand("ABORT");
    ares::sim::clock::advanceMs(400U);              // t=500
    f.engine.tick(ares::sim::clock::nowMs());        // ARM→FIRE, elapsed=499ms < 1000ms

    TEST_ASSERT_EQUAL(0U, f.pulse.getFireCount(PulseChannel::CH_A));
}

// ─────────────────────────────────────────────────────────────────────────────
// T5: PULSE.arm on an undeclared alias fails at parse time
// ─────────────────────────────────────────────────────────────────────────────

void test_pulse_arm_undeclared_channel_parse_error()
{
    // DROGUE alias is never introduced with "pulse.channel A as DROGUE".
    static const char kScript[] =
        "include SIM_GPS as GPS\n"
        "include SIM_BARO as BARO\n"
        "include SIM_COM as COM\n"
        "include SIM_IMU as IMU\n"
        "\n"
        "pus.apid = 1\n"
        "pus.service 3 as HK\n"
        "pus.service 5 as EVENT\n"
        "pus.service 1 as TC\n"
        "\n"
        "state WAIT:\n"
        "  on_enter:\n"
        "    EVENT.info \"WAITING\"\n"
        "  transition to ARM when TC.command == LAUNCH\n"
        "\n"
        "state ARM:\n"
        "  on_enter:\n"
        "    PULSE.arm DROGUE\n"   // DROGUE never declared
        "    EVENT.info \"ARMED\"\n";

    SafetyFixture f;
    f.init("/missions/ua.ams", kScript);
    TEST_ASSERT_FALSE(f.engine.activate("ua.ams"));
    EngineSnapshot snap{};
    f.engine.getSnapshot(snap);
    TEST_ASSERT_EQUAL(EngineStatus::ERROR, snap.status);
}

// ─────────────────────────────────────────────────────────────────────────────
// T6: pulse.min_altitude with no baro driver registered blocks fire (TD2)
// ─────────────────────────────────────────────────────────────────────────────

void test_pulse_min_altitude_no_baro_blocks_fire()
{
    // Script declares pulse.min_altitude 200 but does NOT include SIM_BARO.
    // The engine is constructed with baroCount=0.  checkPulseSafetyLocked()
    // must block fire and emit the explicit "no baro driver" diagnostic.
    static const char kScript[] =
        "include SIM_GPS as GPS\n"
        "include SIM_COM as COM\n"
        "include SIM_IMU as IMU\n"
        "\n"
        "pus.apid = 1\n"
        "pus.service 3 as HK\n"
        "pus.service 5 as EVENT\n"
        "pus.service 1 as TC\n"
        "\n"
        "pulse.channel A as DROGUE\n"
        "pulse.min_altitude 200\n"
        "\n"
        "state WAIT:\n"
        "  on_enter:\n"
        "    EVENT.info \"WAITING\"\n"
        "  transition to FIRE when TC.command == LAUNCH\n"
        "\n"
        "state FIRE:\n"
        "  on_enter:\n"
        "    PULSE.fire DROGUE\n"
        "    EVENT.info \"FIRE\"\n";

    ares::sim::SimStorageDriver storage;
    ares::sim::SimGpsDriver     gps{kGroundProfile};
    ares::sim::SimImuDriver     imu{kGroundProfile};
    ares::sim::SimRadioDriver   radio;
    ares::sim::SimPulseDriver   pulse;

    GpsEntry gpsEntry{"SIM_GPS", &gps};
    ComEntry comEntry{"SIM_COM", &radio};
    ImuEntry imuEntry{"SIM_IMU", &imu};

    // Construct engine with no baro entries (baroCount = 0).
    MissionScriptEngine engine{storage,
                               &gpsEntry, 1U,
                               nullptr, 0U,
                               &comEntry, 1U,
                               &imuEntry, 1U,
                               &pulse};

    (void)storage.begin();
    storage.registerFile("/missions/nb.ams", kScript);
    (void)gps.begin();
    (void)imu.begin();
    (void)radio.begin();
    (void)pulse.begin();
    (void)engine.begin();

    ares::sim::clock::reset();
    ares::sim::clock::advanceMs(1U);
    (void)engine.activate("nb.ams");
    (void)engine.arm();
    ares::sim::clock::advanceMs(1U);
    engine.tick(ares::sim::clock::nowMs());

    // PULSE.fire must be blocked: no baro driver registered.
    TEST_ASSERT_EQUAL(0U, pulse.getFireCount(PulseChannel::CH_A));
}

// ─────────────────────────────────────────────────────────────────────────────
// T7: duplicate pulse.safe_delay directive fails at parse time (TD1)
// ─────────────────────────────────────────────────────────────────────────────

void test_pulse_safe_delay_duplicate_directive_fails()
{
    static const char kScript[] =
        "include SIM_GPS as GPS\n"
        "include SIM_BARO as BARO\n"
        "include SIM_COM as COM\n"
        "include SIM_IMU as IMU\n"
        "\n"
        "pus.apid = 1\n"
        "pus.service 3 as HK\n"
        "pus.service 5 as EVENT\n"
        "pus.service 1 as TC\n"
        "\n"
        "pulse.channel A\n"
        "pulse.safe_delay 1000\n"
        "pulse.safe_delay 2000\n"   // duplicate — must fail
        "\n"
        "state WAIT:\n"
        "  on_enter:\n"
        "    EVENT.info \"W\"\n";

    SafetyFixture f;
    f.init("/missions/dup_sd.ams", kScript);
    TEST_ASSERT_FALSE(f.engine.activate("dup_sd.ams"));
    EngineSnapshot snap{};
    f.engine.getSnapshot(snap);
    TEST_ASSERT_EQUAL(EngineStatus::ERROR, snap.status);
    TEST_ASSERT_NOT_NULL(strstr(snap.lastError, "duplicate"));
}

// ─────────────────────────────────────────────────────────────────────────────
// T8: pulse.require_continuity applied to one channel only; other fires freely
// ─────────────────────────────────────────────────────────────────────────────

static const char kScriptTwoChanPartialCont[] =
    "include SIM_GPS as GPS\n"
    "include SIM_BARO as BARO\n"
    "include SIM_COM as COM\n"
    "include SIM_IMU as IMU\n"
    "\n"
    "pus.apid = 1\n"
    "pus.service 3 as HK\n"
    "pus.service 5 as EVENT\n"
    "pus.service 1 as TC\n"
    "\n"
    "pulse.channel A as DROGUE\n"
    "pulse.channel B as APOGEE\n"
    "pulse.require_continuity A\n"   // only A requires continuity check
    "\n"
    "state WAIT:\n"
    "  on_enter:\n"
    "    EVENT.info \"WAITING\"\n"
    "  transition to FIRE when TC.command == LAUNCH\n"
    "\n"
    "state FIRE:\n"
    "  on_enter:\n"
    "    PULSE.fire DROGUE\n"
    "    PULSE.fire APOGEE\n"
    "    EVENT.info \"FIRE\"\n";

void test_pulse_require_continuity_partial_channels()
{
    // Channel A: hasContPin=true (so parse succeeds), but circuit is open
    //            → PULSE.fire DROGUE blocked.
    // Channel B: no require_continuity directive → PULSE.fire APOGEE succeeds.
    SafetyFixture f;
    f.pulse.setHasContPin(PulseChannel::CH_A, true);
    f.init("/missions/pc.ams", kScriptTwoChanPartialCont);
    f.pulse.setContinuity(PulseChannel::CH_A, false);   // open circuit (set after begin())
    f.armAndTickAt("pc.ams", 1U);

    TEST_ASSERT_EQUAL(0U, f.pulse.getFireCount(PulseChannel::CH_A));  // blocked: open circuit
    TEST_ASSERT_EQUAL(1U, f.pulse.getFireCount(PulseChannel::CH_B));  // no continuity req
}

// ─────────────────────────────────────────────────────────────────────────────
// T9: pulse.require_continuity with null pulse interface — no crash (TD3)
// ─────────────────────────────────────────────────────────────────────────────

void test_pulse_require_continuity_null_interface_no_crash()
{
    // When pulseIface_==nullptr the parse-time hasContPin() check is skipped
    // (null guard in parsePulseRequireContinuityLineLocked), so the directive
    // is silently accepted.  At runtime, the continuity check is also null-
    // guarded, so the fire is attempted but PULSE.fire is a no-op (null driver).
    // The engine must not crash.
    static const char kScript[] =
        "include SIM_GPS as GPS\n"
        "include SIM_BARO as BARO\n"
        "include SIM_COM as COM\n"
        "include SIM_IMU as IMU\n"
        "\n"
        "pus.apid = 1\n"
        "pus.service 3 as HK\n"
        "pus.service 5 as EVENT\n"
        "pus.service 1 as TC\n"
        "\n"
        "pulse.channel A as DROGUE\n"
        "pulse.require_continuity A\n"
        "\n"
        "state WAIT:\n"
        "  on_enter:\n"
        "    EVENT.info \"WAITING\"\n"
        "  transition to FIRE when TC.command == LAUNCH\n"
        "\n"
        "state FIRE:\n"
        "  on_enter:\n"
        "    PULSE.fire DROGUE\n"
        "    EVENT.info \"FIRE\"\n";

    ares::sim::SimStorageDriver storage;
    ares::sim::SimGpsDriver     gps{kGroundProfile};
    ares::sim::SimBaroDriver    baro{kGroundProfile};
    ares::sim::SimImuDriver     imu{kGroundProfile};
    ares::sim::SimRadioDriver   radio;

    GpsEntry  gpsEntry{"SIM_GPS",  &gps};
    BaroEntry baroEntry{"SIM_BARO", &baro};
    ComEntry  comEntry{"SIM_COM",  &radio};
    ImuEntry  imuEntry{"SIM_IMU",  &imu};

    // pulseIface = nullptr: simulates SITL or a board with no pyro driver linked.
    MissionScriptEngine engine{storage,
                               &gpsEntry,  1U,
                               &baroEntry, 1U,
                               &comEntry,  1U,
                               &imuEntry,  1U,
                               nullptr};

    (void)storage.begin();
    storage.registerFile("/missions/npi.ams", kScript);
    (void)gps.begin();
    (void)baro.begin();
    (void)imu.begin();
    (void)radio.begin();
    (void)engine.begin();

    // activate() must succeed: null interface skips the hasContPin() parse check.
    TEST_ASSERT_TRUE(engine.activate("npi.ams"));

    ares::sim::clock::reset();
    ares::sim::clock::advanceMs(1U);
    (void)engine.arm();
    ares::sim::clock::advanceMs(1U);
    engine.tick(ares::sim::clock::nowMs());  // must not crash

    EngineSnapshot snap{};
    engine.getSnapshot(snap);
    // Engine must be RUNNING (or COMPLETE after terminal FIRE state).
    TEST_ASSERT_TRUE(snap.status == EngineStatus::RUNNING
                  || snap.status == EngineStatus::COMPLETE);
}

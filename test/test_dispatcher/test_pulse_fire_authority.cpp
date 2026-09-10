/**
 * @file  test_pulse_fire_authority.cpp
 * @brief ARES-P0-002: single safety authority for remote pulse actuation.
 *
 * FIRE_PULSE_A/B/C/D telecommands must be actuated exclusively through
 * MissionScriptEngine::requestPulseFire(), so a radio command is subject to
 * the identical AMS-4.19 gates (arm token, arm timeout, safe_delay,
 * continuity, altitude) as a script-declared PULSE.fire action.  There is
 * no telecommand bypass (APUS-7.2, AMS-4.19).
 *
 * Covers the negative gates enumerated in ARES-P0-002:
 *   - engine not RUNNING (never armed)
 *   - execution paused after arm (RUNNING -> LOADED via setExecutionEnabled)
 *   - channel not armed (PULSE.arm required but never executed)
 *   - arm token expired (pulse.arm_timeout)
 *   - safe_delay not yet elapsed
 *   - continuity open (pulse.require_continuity)
 *   - no pulse driver attached
 * and the positive/driver paths:
 *   - all gates satisfied -> fire succeeds, ACK
 *   - channel already fired -> driver rejects the second attempt, NACK
 */
#include <unity.h>

#include "comms/radio_dispatcher.h"
#include "ams/mission_script_engine.h"
#include "ams/ams_driver_registry.h"

#include "sim_clock.h"
#include "sim_storage_driver.h"
#include "sim_gps_driver.h"
#include "sim_baro_driver.h"
#include "sim_imu_driver.h"
#include "sim_radio_driver.h"
#include "sim_pulse_driver.h"

#include <cstring>

using namespace ares::proto;

using ares::ams::MissionScriptEngine;
using ares::ams::GpsEntry;
using ares::ams::BaroEntry;
using ares::ams::ComEntry;
using ares::ams::ImuEntry;

static const ares::sim::FlightProfile kRestProfile = {
    .count   = 1U,
    .samples = {
        { .timeMs          = 0U,
          .gpsLatDeg       = 40.0f,  .gpsLonDeg      = -3.0f,
          .gpsAltM         = 0.0f,   .gpsSpeedKmh    = 0.0f,
          .gpsSats         = 8U,     .gpsFix         = true,
          .baroAltM        = 0.0f,   .baroPressurePa = 101325.0f,
          .baroTempC       = 20.0f,
          .accelX          = 0.0f,   .accelY         = 0.0f,
          .accelZ          = 9.81f,  .imuTempC       = 25.0f },
    }
};

// Channel A requires PULSE.arm + has a 2000 ms arm_timeout + continuity pin.
// Channel B has no per-channel gate — used to exercise the driver-level
// "already fired" rejection independently of the arm-token gate.
// pulse.safe_delay applies globally to both channels.
static const char kScriptRemoteFire[] =
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
    "pulse.arm_timeout 2000\n"
    "pulse.safe_delay 1000\n"
    "pulse.require_continuity A\n"
    "\n"
    "state WAIT:\n"
    "  on_enter:\n"
    "    EVENT.info \"WAITING\"\n"
    "  transition to ARM when TC.command == LAUNCH\n"
    "\n"
    "state ARM:\n"
    "  on_enter:\n"
    "    PULSE.arm DROGUE\n"
    "    EVENT.info \"ARMED\"\n";

// ── Fixture ───────────────────────────────────────────────────────────────────

struct PulseAuthorityFixture
{
    ares::sim::SimStorageDriver storage;
    ares::sim::SimGpsDriver     gps{kRestProfile};
    ares::sim::SimBaroDriver    baro{kRestProfile};
    ares::sim::SimImuDriver     imu{kRestProfile};
    ares::sim::SimRadioDriver   engineRadio;
    ares::sim::SimPulseDriver   pulse;

    GpsEntry  gpsEntry  = { "SIM_GPS",  &gps        };
    BaroEntry baroEntry = { "SIM_BARO", &baro       };
    ComEntry  comEntry  = { "SIM_COM",  &engineRadio };
    ImuEntry  imuEntry  = { "SIM_IMU",  &imu        };

    MissionScriptEngine engine{
        storage,
        &gpsEntry,  1U,
        &baroEntry, 1U,
        &comEntry,  1U,
        &imuEntry,  1U,
        &pulse
    };

    ares::sim::SimRadioDriver dispatchRadio;
    ares::RadioDispatcher     dispatcher{ dispatchRadio, engine };

    void init()
    {
        (void)storage.begin();
        (void)gps.begin();
        (void)baro.begin();
        (void)imu.begin();
        (void)pulse.begin();
        // The shared script always declares pulse.require_continuity A, which
        // requires a wired continuity-sense pin at parse time regardless of
        // which gate an individual test exercises.
        pulse.setHasContPin(PulseChannel::CH_A, true);
        (void)engine.begin();
        (void)dispatchRadio.begin();
        storage.registerFile("/missions/rf.ams", kScriptRemoteFire);
    }

    /**
     * Activate + arm() at simulated t=1 ms (activationMs_=1, avoiding the
     * activationMs_==0 safe_delay bypass), then optionally tick to move
     * WAIT -> ARM (running channel A's PULSE.arm).
     */
    void activateAndArm(bool tickToArmed)
    {
        ares::sim::clock::reset();
        ares::sim::clock::advanceMs(1U);
        (void)engine.activate("rf.ams");
        (void)engine.arm();
        if (tickToArmed)
        {
            (void)engine.injectTcCommand("LAUNCH");
            ares::sim::clock::advanceMs(1U);
            engine.tick(ares::sim::clock::nowMs());
        }
    }
};

/** Build a minimal non-fragmented FIRE_PULSE_x wire frame (open mode, no MAC). */
static uint16_t make_fire_pulse_cmd(uint8_t* buf, uint8_t seq, CommandId id)
{
    Frame tx = {};
    tx.ver        = PROTOCOL_VERSION;
    tx.node       = NODE_ROCKET;
    tx.type       = MsgType::COMMAND;
    tx.seq        = seq;
    // FLAG_ACK_REQ is required to observe the completion ACK/NACK (which
    // carries the actual PulseFireResult-derived FailureCode); the
    // unconditional acceptance ACK is always FailureCode::NONE.
    tx.flags      = static_cast<uint8_t>(FLAG_PRIORITY | FLAG_ACK_REQ);
    tx.payload[0] = static_cast<uint8_t>(Priority::PRI_CRITICAL);
    tx.payload[1] = static_cast<uint8_t>(id);
    tx.payload[2] = 0U;
    tx.payload[3] = 0U;
    tx.payload[4] = 0U;
    tx.payload[5] = 0U;
    tx.len        = 6U;  // sizeof(CommandHeader)

    uint16_t outLen = 0U;
    (void)encode(tx, buf, MAX_FRAME_LEN, outLen);
    TEST_ASSERT_GREATER_THAN_UINT16(0U, outLen);
    return outLen;
}

/** Decode the ACK/NACK frame last sent on @p radio and return its FailureCode. */
static FailureCode last_failure_code(ares::sim::SimRadioDriver& radio)
{
    Frame ack = {};
    if (!decode(radio.lastFrame(), radio.lastFrameLen(), ack)) { return FailureCode::EXECUTION_ERROR; }
    if (ack.len < static_cast<uint8_t>(sizeof(AckPayload))) { return FailureCode::NONE; }
    AckPayload ap = {};
    (void)memcpy(&ap, ack.payload, sizeof(ap));
    return static_cast<FailureCode>(ap.failureCode);
}

// ── Never armed: engine IDLE (no script activated) ───────────────────────────

void test_remote_fire_engine_not_running_rejected()
{
    PulseAuthorityFixture f;
    f.init();

    uint8_t wire[MAX_FRAME_LEN];
    const uint16_t len = make_fire_pulse_cmd(wire, 1U, CommandId::FIRE_PULSE_A);
    TEST_ASSERT_TRUE(f.dispatchRadio.injectBytes(wire, len));
    f.dispatcher.poll(1000U);

    TEST_ASSERT_EQUAL(FailureCode::PRECONDITION_FAIL, last_failure_code(f.dispatchRadio));
    TEST_ASSERT_EQUAL(0U, f.pulse.getFireCount(PulseChannel::CH_A));
}

// ── Execution paused after arm ────────────────────────────────────────────────

void test_remote_fire_paused_execution_rejected()
{
    PulseAuthorityFixture f;
    f.init();
    f.activateAndArm(/*tickToArmed=*/true);
    f.pulse.setContinuity(PulseChannel::CH_A, true);
    f.engine.setExecutionEnabled(false);  // RUNNING -> LOADED

    uint8_t wire[MAX_FRAME_LEN];
    const uint16_t len = make_fire_pulse_cmd(wire, 2U, CommandId::FIRE_PULSE_A);
    TEST_ASSERT_TRUE(f.dispatchRadio.injectBytes(wire, len));
    f.dispatcher.poll(2000U);

    TEST_ASSERT_EQUAL(FailureCode::PRECONDITION_FAIL, last_failure_code(f.dispatchRadio));
    TEST_ASSERT_EQUAL(0U, f.pulse.getFireCount(PulseChannel::CH_A));
}

// ── Channel not armed (PULSE.arm never executed) ─────────────────────────────

void test_remote_fire_channel_not_armed_rejected()
{
    PulseAuthorityFixture f;
    f.init();
    f.activateAndArm(/*tickToArmed=*/false);  // stays in WAIT; PULSE.arm never ran

    uint8_t wire[MAX_FRAME_LEN];
    const uint16_t len = make_fire_pulse_cmd(wire, 3U, CommandId::FIRE_PULSE_A);
    TEST_ASSERT_TRUE(f.dispatchRadio.injectBytes(wire, len));
    // nowMs=1500 clears the global safe_delay (elapsed 1499 >= 1000) so the
    // rejection below is isolated to the arm gate, not safe_delay.
    f.dispatcher.poll(1500U);

    TEST_ASSERT_EQUAL(FailureCode::PRECONDITION_FAIL, last_failure_code(f.dispatchRadio));
    TEST_ASSERT_EQUAL(0U, f.pulse.getFireCount(PulseChannel::CH_A));
}

// ── Arm token expired (pulse.arm_timeout 2000) ────────────────────────────────

void test_remote_fire_arm_expired_rejected()
{
    PulseAuthorityFixture f;
    f.init();
    f.activateAndArm(/*tickToArmed=*/true);  // pulseArmedMs_[A] = 2 (t after 1ms+1ms)
    f.pulse.setContinuity(PulseChannel::CH_A, true);

    uint8_t wire[MAX_FRAME_LEN];
    const uint16_t len = make_fire_pulse_cmd(wire, 4U, CommandId::FIRE_PULSE_A);
    TEST_ASSERT_TRUE(f.dispatchRadio.injectBytes(wire, len));
    // armAge = 2100 - 2 = 2098 ms > 2000 ms arm_timeout.
    f.dispatcher.poll(2100U);

    TEST_ASSERT_EQUAL(FailureCode::PRECONDITION_FAIL, last_failure_code(f.dispatchRadio));
    TEST_ASSERT_EQUAL(0U, f.pulse.getFireCount(PulseChannel::CH_A));
}

// ── safe_delay not yet elapsed (pulse.safe_delay 1000, global) ───────────────

void test_remote_fire_safe_delay_blocks_early_fire()
{
    PulseAuthorityFixture f;
    f.init();
    f.activateAndArm(/*tickToArmed=*/true);  // activationMs_ = 1
    f.pulse.setContinuity(PulseChannel::CH_A, true);

    uint8_t wire[MAX_FRAME_LEN];
    const uint16_t len = make_fire_pulse_cmd(wire, 5U, CommandId::FIRE_PULSE_A);
    TEST_ASSERT_TRUE(f.dispatchRadio.injectBytes(wire, len));
    // elapsed since activationMs_(1) = 500 - 1 = 499 ms < 1000 ms safe_delay.
    f.dispatcher.poll(500U);

    TEST_ASSERT_EQUAL(FailureCode::PRECONDITION_FAIL, last_failure_code(f.dispatchRadio));
    TEST_ASSERT_EQUAL(0U, f.pulse.getFireCount(PulseChannel::CH_A));
}

// ── Continuity open (pulse.require_continuity A) ─────────────────────────────

void test_remote_fire_continuity_open_rejected()
{
    PulseAuthorityFixture f;
    f.init();
    f.activateAndArm(/*tickToArmed=*/true);
    f.pulse.setContinuity(PulseChannel::CH_A, false);  // open circuit (after begin()/reset())

    uint8_t wire[MAX_FRAME_LEN];
    const uint16_t len = make_fire_pulse_cmd(wire, 6U, CommandId::FIRE_PULSE_A);
    TEST_ASSERT_TRUE(f.dispatchRadio.injectBytes(wire, len));
    f.dispatcher.poll(2000U);  // clears safe_delay and arm_timeout gates

    TEST_ASSERT_EQUAL(FailureCode::PRECONDITION_FAIL, last_failure_code(f.dispatchRadio));
    TEST_ASSERT_EQUAL(0U, f.pulse.getFireCount(PulseChannel::CH_A));
}

// ── All gates satisfied: fire succeeds ────────────────────────────────────────

void test_remote_fire_all_gates_satisfied_succeeds()
{
    PulseAuthorityFixture f;
    f.init();
    f.activateAndArm(/*tickToArmed=*/true);
    // Continuity default is true after reset() — intact circuit.

    uint8_t wire[MAX_FRAME_LEN];
    const uint16_t len = make_fire_pulse_cmd(wire, 7U, CommandId::FIRE_PULSE_A);
    TEST_ASSERT_TRUE(f.dispatchRadio.injectBytes(wire, len));
    f.dispatcher.poll(2000U);  // safe_delay and arm_timeout satisfied

    TEST_ASSERT_EQUAL(FailureCode::NONE, last_failure_code(f.dispatchRadio));
    TEST_ASSERT_EQUAL(1U, f.pulse.getFireCount(PulseChannel::CH_A));
}

// ── Channel already fired: driver rejects, dispatcher NACKs EXECUTION_ERROR ──

void test_remote_fire_already_fired_rejected_by_driver()
{
    // Channel B has no PULSE.arm/continuity/altitude directive, so the arm
    // gate cannot mask the driver-level "already fired" rejection.
    PulseAuthorityFixture f;
    f.init();
    f.activateAndArm(/*tickToArmed=*/true);

    uint8_t wire1[MAX_FRAME_LEN];
    const uint16_t len1 = make_fire_pulse_cmd(wire1, 8U, CommandId::FIRE_PULSE_B);
    TEST_ASSERT_TRUE(f.dispatchRadio.injectBytes(wire1, len1));
    f.dispatcher.poll(2000U);  // safe_delay satisfied — first fire succeeds
    TEST_ASSERT_EQUAL(FailureCode::NONE, last_failure_code(f.dispatchRadio));
    TEST_ASSERT_EQUAL(1U, f.pulse.getFireCount(PulseChannel::CH_B));

    uint8_t wire2[MAX_FRAME_LEN];
    const uint16_t len2 = make_fire_pulse_cmd(wire2, 9U, CommandId::FIRE_PULSE_B);
    TEST_ASSERT_TRUE(f.dispatchRadio.injectBytes(wire2, len2));
    f.dispatcher.poll(2001U);

    TEST_ASSERT_EQUAL(FailureCode::EXECUTION_ERROR, last_failure_code(f.dispatchRadio));
    TEST_ASSERT_EQUAL(1U, f.pulse.getFireCount(PulseChannel::CH_B));  // unchanged
}

// ── No pulse driver attached: engine RUNNING but pulseIface_ == nullptr ──────

void test_remote_fire_no_driver_attached_rejected()
{
    ares::sim::SimStorageDriver storage;
    ares::sim::SimGpsDriver     gps{kRestProfile};
    ares::sim::SimBaroDriver    baro{kRestProfile};
    ares::sim::SimImuDriver     imu{kRestProfile};
    ares::sim::SimRadioDriver   engineRadio;

    GpsEntry  gpsEntry  = { "SIM_GPS",  &gps        };
    BaroEntry baroEntry = { "SIM_BARO", &baro       };
    ComEntry  comEntry  = { "SIM_COM",  &engineRadio };
    ImuEntry  imuEntry  = { "SIM_IMU",  &imu        };

    // No PulseInterface passed to the engine — deliberately reproduces a
    // build that omits the pulse driver while still running a mission.
    MissionScriptEngine engine{
        storage, &gpsEntry, 1U, &baroEntry, 1U, &comEntry, 1U, &imuEntry, 1U
    };

    ares::sim::SimRadioDriver dispatchRadio;
    ares::RadioDispatcher     dispatcher{ dispatchRadio, engine };

    (void)storage.begin();
    (void)gps.begin();
    (void)baro.begin();
    (void)imu.begin();
    (void)engine.begin();
    (void)dispatchRadio.begin();

    static const char kMinimalScript[] =
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
        "    EVENT.info \"WAITING\"\n";
    storage.registerFile("/missions/nd.ams", kMinimalScript);

    TEST_ASSERT_TRUE(engine.activate("nd.ams"));
    TEST_ASSERT_TRUE(engine.arm());  // status_=RUNNING, executionEnabled_=true

    uint8_t wire[MAX_FRAME_LEN];
    const uint16_t len = make_fire_pulse_cmd(wire, 10U, CommandId::FIRE_PULSE_A);
    TEST_ASSERT_TRUE(dispatchRadio.injectBytes(wire, len));
    dispatcher.poll(1000U);

    TEST_ASSERT_EQUAL(FailureCode::EXECUTION_ERROR, last_failure_code(dispatchRadio));
}

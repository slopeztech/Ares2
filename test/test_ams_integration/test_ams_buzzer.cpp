/**
 * @file  test_ams_buzzer.cpp
 * @brief SITL integration tests — AMS BUZZER.beep command execution (AMS-4.20).
 *
 * Covers: beep fires on state entry, default vs. explicit frequency, null-driver
 * graceful skip, parse-time rejection of out-of-range duration/frequency/repeat,
 * multiple beep actions in one state, execution guard (disabled engine),
 * repeat-count syntax (Nx), and BUZZER.beep inside every: slots.
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
#include "sim_buzzer_driver.h"

using ares::ams::MissionScriptEngine;
using ares::ams::EngineSnapshot;
using ares::ams::EngineStatus;

// ── Minimal flight profile (no altitude/speed needed for buzzer tests) ─────────

static const ares::sim::FlightProfile kBuzzerProfile = {
    .count   = 1U,
    .samples = {
        { .timeMs =  0U, .gpsLatDeg = 40.0f, .gpsLonDeg = -3.0f,
          .gpsAltM =  0.0f, .gpsSpeedKmh = 0.0f, .gpsSats = 9U, .gpsFix = true,
          .baroAltM = 0.0f, .baroPressurePa = 101325.0f, .baroTempC = 20.0f,
          .accelX = 0.0f, .accelY = 0.0f, .accelZ = 9.81f, .imuTempC = 25.0f },
    }
};

// ── Fixture ───────────────────────────────────────────────────────────────────

struct BuzzerFixture
{
    ares::sim::SimStorageDriver storage;
    ares::sim::SimGpsDriver     gps{kBuzzerProfile};
    ares::sim::SimBaroDriver    baro{kBuzzerProfile};
    ares::sim::SimImuDriver     imu{kBuzzerProfile};
    ares::sim::SimRadioDriver   radio;
    ares::sim::SimBuzzerDriver  buzzer;

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
        nullptr,      // no pulse interface
        &buzzer
    };

    void init(const char* path, const char* content)
    {
        (void)storage.begin();
        storage.registerFile(path, content);
        (void)gps.begin();
        (void)baro.begin();
        (void)imu.begin();
        (void)radio.begin();
        (void)buzzer.begin();
        (void)engine.begin();
    }

    /** Activate + arm + tick once.  arm() pre-queues LAUNCH TC so the first
     *  tick transitions WAIT→DEPLOY and runs DEPLOY's on_enter. */
    void activateAndArm(const char* filename)
    {
        ares::sim::clock::reset();
        (void)engine.activate(filename);
        (void)engine.arm();
        engine.tick(ares::sim::clock::nowMs());
    }
};

// Variant with no buzzer driver (null) to test graceful skip.
struct BuzzerNullFixture
{
    ares::sim::SimStorageDriver storage;
    ares::sim::SimGpsDriver     gps{kBuzzerProfile};
    ares::sim::SimBaroDriver    baro{kBuzzerProfile};
    ares::sim::SimImuDriver     imu{kBuzzerProfile};
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
        &imuEntry,  1U,
        nullptr,
        nullptr   // buzzerIface = null
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

    void activateAndArm(const char* filename)
    {
        ares::sim::clock::reset();
        (void)engine.activate(filename);
        (void)engine.arm();
        engine.tick(ares::sim::clock::nowMs());
    }
};

// ── Tests ─────────────────────────────────────────────────────────────────────

// ─────────────────────────────────────────────────────────────────────────────
// AMS-4.20: BUZZER.beep fires driver beep() on state entry.
// ─────────────────────────────────────────────────────────────────────────────

void test_buzzer_beep_fires_on_enter()
{
    BuzzerFixture f;
    f.init("/missions/buzzer.ams", ares::sim::kScriptBuzzerBeep);
    f.activateAndArm("buzzer.ams");

    EngineSnapshot snap{};
    f.engine.getSnapshot(snap);
    TEST_ASSERT_EQUAL_STRING("DEPLOY", snap.stateName);

    // Driver must have received exactly one beep() call.
    TEST_ASSERT_EQUAL(1U, f.buzzer.getBeepCount());
    TEST_ASSERT_EQUAL(500U, f.buzzer.getLastDurationMs());
}

// ─────────────────────────────────────────────────────────────────────────────
// AMS-4.20: Without an explicit Hz suffix, freqHz stored is 0 (use driver default).
// ─────────────────────────────────────────────────────────────────────────────

void test_buzzer_beep_default_freq_zero()
{
    BuzzerFixture f;
    f.init("/missions/buzzer.ams", ares::sim::kScriptBuzzerBeep);
    f.activateAndArm("buzzer.ams");

    TEST_ASSERT_EQUAL(0U, f.buzzer.getLastFreqHz());
}

// ─────────────────────────────────────────────────────────────────────────────
// AMS-4.20: Explicit "Nhz" suffix is passed through to the driver.
// ─────────────────────────────────────────────────────────────────────────────

void test_buzzer_beep_custom_freq()
{
    BuzzerFixture f;
    f.init("/missions/buzzer.ams", ares::sim::kScriptBuzzerBeepFreq);
    f.activateAndArm("buzzer.ams");

    TEST_ASSERT_EQUAL(1U,    f.buzzer.getBeepCount());
    TEST_ASSERT_EQUAL(1000U, f.buzzer.getLastDurationMs());
    TEST_ASSERT_EQUAL(3000U, f.buzzer.getLastFreqHz());
}

// ─────────────────────────────────────────────────────────────────────────────
// AMS-4.20: null buzzerIface — engine must not crash, beep silently skipped.
// ─────────────────────────────────────────────────────────────────────────────

void test_buzzer_null_driver_skipped()
{
    BuzzerNullFixture f;
    f.init("/missions/buzzer.ams", ares::sim::kScriptBuzzerBeep);
    f.activateAndArm("buzzer.ams");

    // Engine must be in RUNNING or COMPLETE (not ERROR).
    EngineSnapshot snap{};
    f.engine.getSnapshot(snap);
    TEST_ASSERT_NOT_EQUAL(static_cast<int>(EngineStatus::ERROR), static_cast<int>(snap.status));
    TEST_ASSERT_EQUAL_STRING("DEPLOY", snap.stateName);
}

// ─────────────────────────────────────────────────────────────────────────────
// AMS-4.20: Duration below BUZZER_MIN_DURATION_MS must be rejected at parse time.
// ─────────────────────────────────────────────────────────────────────────────

void test_buzzer_duration_too_short_rejected()
{
    BuzzerFixture f;
    f.init("/missions/buzzer.ams", ares::sim::kScriptBuzzerDurationTooShort);

    ares::sim::clock::reset();
    const bool activated = f.engine.activate("buzzer.ams");

    // activate() parses the script — must return false on invalid duration.
    TEST_ASSERT_FALSE(activated);

    EngineSnapshot snap{};
    f.engine.getSnapshot(snap);
    TEST_ASSERT_EQUAL(static_cast<int>(EngineStatus::ERROR), static_cast<int>(snap.status));
}

// ─────────────────────────────────────────────────────────────────────────────
// AMS-4.20: Frequency below BUZZER_MIN_FREQ_HZ must be rejected at parse time.
// ─────────────────────────────────────────────────────────────────────────────

void test_buzzer_freq_below_min_rejected()
{
    BuzzerFixture f;
    f.init("/missions/buzzer.ams", ares::sim::kScriptBuzzerFreqTooLow);

    ares::sim::clock::reset();
    const bool activated = f.engine.activate("buzzer.ams");

    TEST_ASSERT_FALSE(activated);

    EngineSnapshot snap{};
    f.engine.getSnapshot(snap);
    TEST_ASSERT_EQUAL(static_cast<int>(EngineStatus::ERROR), static_cast<int>(snap.status));
}

// ─────────────────────────────────────────────────────────────────────────────
// AMS-4.20: Frequency above BUZZER_MAX_FREQ_HZ must be rejected at parse time.
// ─────────────────────────────────────────────────────────────────────────────

void test_buzzer_freq_above_max_rejected()
{
    BuzzerFixture f;
    f.init("/missions/buzzer.ams", ares::sim::kScriptBuzzerFreqTooHigh);

    ares::sim::clock::reset();
    const bool activated = f.engine.activate("buzzer.ams");

    TEST_ASSERT_FALSE(activated);

    EngineSnapshot snap{};
    f.engine.getSnapshot(snap);
    TEST_ASSERT_EQUAL(static_cast<int>(EngineStatus::ERROR), static_cast<int>(snap.status));
}

// ─────────────────────────────────────────────────────────────────────────────
// AMS-4.20: Two BUZZER.beep actions in the same on_enter block both execute.
// ─────────────────────────────────────────────────────────────────────────────

void test_buzzer_multiple_beeps_same_state()
{
    BuzzerFixture f;
    f.init("/missions/buzzer.ams", ares::sim::kScriptBuzzerMultiple);
    f.activateAndArm("buzzer.ams");

    // Both beep() calls must have been issued; last one wins for lastXxx getters.
    TEST_ASSERT_EQUAL(2U,    f.buzzer.getBeepCount());
    TEST_ASSERT_EQUAL(400U,  f.buzzer.getLastDurationMs());
    TEST_ASSERT_EQUAL(2000U, f.buzzer.getLastFreqHz());
}

// ─────────────────────────────────────────────────────────────────────────────
// AMS-4.20: setExecutionEnabled(false) prevents buzzer from firing.
// ─────────────────────────────────────────────────────────────────────────────

void test_buzzer_no_fire_when_engine_disabled()
{
    BuzzerFixture f;
    f.init("/missions/buzzer.ams", ares::sim::kScriptBuzzerBeep);

    ares::sim::clock::reset();
    (void)f.engine.activate("buzzer.ams");
    (void)f.engine.arm();
    f.engine.setExecutionEnabled(false);
    f.engine.tick(ares::sim::clock::nowMs());

    // Execution was disabled — buzzer must NOT have been called.
    TEST_ASSERT_EQUAL(0U, f.buzzer.getBeepCount());
}

// ─────────────────────────────────────────────────────────────────────────────
// AMS-4.20: BUZZER.beep Nms Rx — repeat count stored and forwarded.
// ─────────────────────────────────────────────────────────────────────────────

void test_buzzer_repeat_count_stored()
{
    BuzzerFixture f;
    f.init("/missions/buzzer.ams", ares::sim::kScriptBuzzerRepeat);
    f.activateAndArm("buzzer.ams");

    TEST_ASSERT_EQUAL(1U,   f.buzzer.getBeepCount());
    TEST_ASSERT_EQUAL(300U, f.buzzer.getLastDurationMs());
    TEST_ASSERT_EQUAL(0U,   f.buzzer.getLastFreqHz());
    TEST_ASSERT_EQUAL(3U,   f.buzzer.getLastRepeatCount());
}

// ─────────────────────────────────────────────────────────────────────────────
// AMS-4.20: BUZZER.beep Nms Fhz Rx — frequency + repeat count.
// ─────────────────────────────────────────────────────────────────────────────

void test_buzzer_repeat_with_freq()
{
    BuzzerFixture f;
    f.init("/missions/buzzer.ams", ares::sim::kScriptBuzzerRepeatFreq);
    f.activateAndArm("buzzer.ams");

    TEST_ASSERT_EQUAL(1U,    f.buzzer.getBeepCount());
    TEST_ASSERT_EQUAL(200U,  f.buzzer.getLastDurationMs());
    TEST_ASSERT_EQUAL(2000U, f.buzzer.getLastFreqHz());
    TEST_ASSERT_EQUAL(5U,    f.buzzer.getLastRepeatCount());
}

// ─────────────────────────────────────────────────────────────────────────────
// AMS-4.20: no Nx suffix → repeat count defaults to 1.
// ─────────────────────────────────────────────────────────────────────────────

void test_buzzer_default_repeat_is_one()
{
    BuzzerFixture f;
    f.init("/missions/buzzer.ams", ares::sim::kScriptBuzzerBeep);
    f.activateAndArm("buzzer.ams");

    TEST_ASSERT_EQUAL(1U, f.buzzer.getLastRepeatCount());
}

// ─────────────────────────────────────────────────────────────────────────────
// AMS-4.20: Nx above BUZZER_MAX_REPEAT_COUNT is rejected at parse time.
// ─────────────────────────────────────────────────────────────────────────────

void test_buzzer_repeat_too_high_rejected()
{
    BuzzerFixture f;
    f.init("/missions/buzzer.ams", ares::sim::kScriptBuzzerRepeatTooHigh);

    ares::sim::clock::reset();
    const bool ok = f.engine.activate("buzzer.ams");
    TEST_ASSERT_FALSE(ok);
    EngineSnapshot snap{};
    f.engine.getSnapshot(snap);
    TEST_ASSERT_EQUAL(static_cast<int>(EngineStatus::ERROR), static_cast<int>(snap.status));
}

// ─────────────────────────────────────────────────────────────────────────────
// AMS-4.20: BUZZER.beep inside every: fires on each cadence tick.
// ─────────────────────────────────────────────────────────────────────────────

void test_buzzer_fires_in_every_slot()
{
    BuzzerFixture f;
    f.init("/missions/buzzer.ams", ares::sim::kScriptBuzzerInEvery);

    ares::sim::clock::reset();
    (void)f.engine.activate("buzzer.ams");
    (void)f.engine.arm();

    // First tick: WAIT → DEPLOY (on_enter, no buzzer in this script).
    f.engine.tick(ares::sim::clock::nowMs());
    const uint32_t countAfterEnter = f.buzzer.getBeepCount();

    // Advance clock by exactly one cadence period and tick.
    ares::sim::clock::advanceMs(100U);
    f.engine.tick(ares::sim::clock::nowMs());

    // The every slot fires once → beep() called exactly once (plus any on_enter).
    TEST_ASSERT_EQUAL(countAfterEnter + 1U, f.buzzer.getBeepCount());
    TEST_ASSERT_EQUAL(150U, f.buzzer.getLastDurationMs());
    TEST_ASSERT_EQUAL(0U,   f.buzzer.getLastFreqHz());
    TEST_ASSERT_EQUAL(1U,   f.buzzer.getLastRepeatCount());

    // Advance again and verify it fires a second time.
    ares::sim::clock::advanceMs(100U);
    f.engine.tick(ares::sim::clock::nowMs());
    TEST_ASSERT_EQUAL(countAfterEnter + 2U, f.buzzer.getBeepCount());
}

// ─────────────────────────────────────────────────────────────────────────────
// AMS-4.20: BUZZER.beep Rx in every: — repeat count forwarded on each tick.
// ─────────────────────────────────────────────────────────────────────────────

void test_buzzer_every_repeat_count()
{
    BuzzerFixture f;
    f.init("/missions/buzzer.ams", ares::sim::kScriptBuzzerEveryRepeat);

    ares::sim::clock::reset();
    (void)f.engine.activate("buzzer.ams");
    (void)f.engine.arm();

    // Transition tick + advance 200ms for the every cadence.
    f.engine.tick(ares::sim::clock::nowMs());
    ares::sim::clock::advanceMs(200U);
    f.engine.tick(ares::sim::clock::nowMs());

    TEST_ASSERT_EQUAL(100U, f.buzzer.getLastDurationMs());
    TEST_ASSERT_EQUAL(2U,   f.buzzer.getLastRepeatCount());
}

// ─────────────────────────────────────────────────────────────────────────────
// AMS-4.20: every slot buzzer does NOT fire when execution is disabled.
// ─────────────────────────────────────────────────────────────────────────────

void test_buzzer_every_no_fire_when_disabled()
{
    BuzzerFixture f;
    f.init("/missions/buzzer.ams", ares::sim::kScriptBuzzerInEvery);

    ares::sim::clock::reset();
    (void)f.engine.activate("buzzer.ams");
    (void)f.engine.arm();
    // Transition tick — buzzer not in on_enter of this script.
    f.engine.tick(ares::sim::clock::nowMs());
    f.engine.setExecutionEnabled(false);

    ares::sim::clock::advanceMs(100U);
    f.engine.tick(ares::sim::clock::nowMs());

    // Execution disabled — beep must not have been called during every slot.
    TEST_ASSERT_EQUAL(0U, f.buzzer.getBeepCount());
}

/**
 * @file  test_ams_tick_scheduling.cpp
 * @brief SITL integration tests — AMS-5.9 Adaptive Tick Scheduling.
 *
 * Verifies nextWakeupMs() returns the correct sleep duration for each
 * engine state as defined in the AMS-5.9 scheduling table:
 *
 *   | Condition                                    | Return value           |
 *   |----------------------------------------------|------------------------|
 *   | Engine not running / status ≠ RUNNING         | nowMs + SENSOR_RATE_MS |
 *   | pendingOnEnterEvent_ set                      | nowMs + SENSOR_RATE_MS |
 *   | Active state has transitions or guard conds   | nowMs + SENSOR_RATE_MS |
 *   | Pure reporting state (no conds) — far HK slot | nowMs + kRadioMaxSleepMs |
 *   | Pure reporting state — HK slot due soon       | HK slot due time       |
 */

#include <unity.h>

#include "ams/mission_script_engine.h"
#include "ams/ams_driver_registry.h"
#include "config.h"

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
using ares::ams::EngineStatus;

// kRadioMaxSleepMs is a private constexpr inside MissionScriptEngine.
// The standard (AMS-5.9) documents its value as 50 ms.
static constexpr uint64_t kExpectedRadioCap = 50U;

// SENSOR_RATE_MS is a public constant in config.h (ares namespace).
static constexpr uint64_t kExpectedSensorRate =
    static_cast<uint64_t>(ares::SENSOR_RATE_MS);

// ── Flight profile (minimal, constant altitude) ───────────────────────────────

static const ares::sim::FlightProfile kTickProfile = {
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

struct TickFixture
{
    ares::sim::SimStorageDriver storage;
    ares::sim::SimGpsDriver     gps{kTickProfile};
    ares::sim::SimBaroDriver    baro{kTickProfile};
    ares::sim::SimImuDriver     imu{kTickProfile};
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
// IDLE engine (begin() called, nothing activated) returns SENSOR_RATE_MS.
// ─────────────────────────────────────────────────────────────────────────────

void test_wakeup_idle_engine_returns_sensor_rate()
{
    TickFixture f;
    f.init("/missions/hk.ams", ares::sim::kScriptTickHkOnly);

    // Engine is in IDLE state — running_ is false, status != RUNNING.
    const uint64_t wakeup = f.engine.nextWakeupMs(0U);
    TEST_ASSERT_EQUAL_UINT64(kExpectedSensorRate, wakeup);
}

// ─────────────────────────────────────────────────────────────────────────────
// LOADED engine (activated but not armed) returns SENSOR_RATE_MS.
// ─────────────────────────────────────────────────────────────────────────────

void test_wakeup_loaded_engine_returns_sensor_rate()
{
    TickFixture f;
    f.init("/missions/hk.ams", ares::sim::kScriptTickHkOnly);

    TEST_ASSERT_TRUE(f.engine.activate("hk.ams"));
    // status_ == LOADED, not RUNNING → fast-path returns SENSOR_RATE_MS.
    const uint64_t wakeup = f.engine.nextWakeupMs(0U);
    TEST_ASSERT_EQUAL_UINT64(kExpectedSensorRate, wakeup);
}

// ─────────────────────────────────────────────────────────────────────────────
// RUNNING engine in a state with transitions returns SENSOR_RATE_MS.
//
// kScriptTickWithTransition has WAIT with TIME.elapsed > 10 000 transition.
// Because transitionCount > 0, nextWakeupMs must always return nowMs +
// SENSOR_RATE_MS to guarantee fresh sensor evaluation every tick (AMS-5.9).
// ─────────────────────────────────────────────────────────────────────────────

void test_wakeup_state_with_transitions_returns_sensor_rate()
{
    TickFixture f;
    f.init("/missions/trans.ams", ares::sim::kScriptTickWithTransition);

    TEST_ASSERT_TRUE(f.engine.activate("trans.ams"));
    TEST_ASSERT_TRUE(f.engine.arm());

    f.engine.tick(0U);  // consume LAUNCH TC — stays in WAIT (TIME.elapsed transition)

    const uint64_t wakeup = f.engine.nextWakeupMs(0U);
    TEST_ASSERT_EQUAL_UINT64(kExpectedSensorRate, wakeup);
}

// ─────────────────────────────────────────────────────────────────────────────
// Pure reporting state with HK every 1000 ms: capped at kRadioMaxSleepMs.
//
// At t=0, the next HK slot fires at t=1000, which is beyond nowMs+50.
// nextWakeupMs must return nowMs + kRadioMaxSleepMs = 50 (AMS-5.9).
// ─────────────────────────────────────────────────────────────────────────────

void test_wakeup_pure_reporting_state_capped_at_radio_max()
{
    TickFixture f;
    f.init("/missions/hk.ams", ares::sim::kScriptTickHkOnly);

    TEST_ASSERT_TRUE(f.engine.activate("hk.ams"));
    TEST_ASSERT_TRUE(f.engine.arm());

    f.engine.tick(0U);

    // No transitions, no conditions — capped by kRadioMaxSleepMs.
    // next HK due = 0 + 1000 = 1000 > 0 + 50 → no tightening.
    const uint64_t wakeup = f.engine.nextWakeupMs(0U);
    TEST_ASSERT_EQUAL_UINT64(kExpectedRadioCap, wakeup);
}

// ─────────────────────────────────────────────────────────────────────────────
// Pure reporting state with HK every 100 ms: deadline tightened to HK slot.
//
// After entering REPORT at t=0, lastHkSlotMs[0] = 0.
// At nowMs=60: next = 60+50=110, HK due = 0+100=100 < 110
// → nextWakeupMs must return 100 (the HK slot due time).
// ─────────────────────────────────────────────────────────────────────────────

void test_wakeup_hk_slot_tightens_deadline()
{
    TickFixture f;
    f.init("/missions/hkfast.ams", ares::sim::kScriptTickHkFast);

    TEST_ASSERT_TRUE(f.engine.activate("hkfast.ams"));
    TEST_ASSERT_TRUE(f.engine.arm());

    f.engine.tick(0U);  // t=0 — HK not yet due (0-0=0 < 100)

    // At t=60: next_cap = 60+50=110, HK due = 0+100=100 < 110 → returns 100.
    const uint64_t wakeup = f.engine.nextWakeupMs(60U);
    TEST_ASSERT_EQUAL_UINT64(100U, wakeup);
}

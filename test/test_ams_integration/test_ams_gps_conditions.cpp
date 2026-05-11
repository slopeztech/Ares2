/**
 * @file  test_ams_gps_conditions.cpp
 * @brief SITL integration tests — GPS.sats / GPS.hdop transition conditions.
 *
 * Verifies that the AMS engine correctly evaluates GPS satellite count and
 * HDOP fields as transition condition operands (AMS-4.5 / AMS-4.6).
 *
 * Test coverage:
 *   - GPS.sats > N  fires when satellite count exceeds threshold.
 *   - GPS.hdop < N  fires when HDOP is below threshold.
 *   - GPS.sats > N  does NOT fire when count is at or below threshold.
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

// ── Profile: 9 satellites, HDOP 1.0 (good fix) ──────────────────────────────

static const ares::sim::FlightProfile kGoodFixProfile = {
    .count   = 1U,
    .samples = {
        { .timeMs = 0U, .gpsLatDeg = 40.0f, .gpsLonDeg = -3.0f,
          .gpsAltM = 10.0f, .gpsSpeedKmh = 0.0f,
          .gpsHdop = 1.0f, .gpsSats = 9U, .gpsFix = true,
          .baroAltM = 10.0f, .baroPressurePa = 101000.0f, .baroTempC = 20.0f,
          .accelX = 0.0f, .accelY = 0.0f, .accelZ = 9.81f, .imuTempC = 25.0f },
    }
};

// ── Profile: 4 satellites, HDOP 3.5 (poor fix) ──────────────────────────────

static const ares::sim::FlightProfile kPoorFixProfile = {
    .count   = 1U,
    .samples = {
        { .timeMs = 0U, .gpsLatDeg = 40.0f, .gpsLonDeg = -3.0f,
          .gpsAltM = 10.0f, .gpsSpeedKmh = 0.0f,
          .gpsHdop = 3.5f, .gpsSats = 4U, .gpsFix = true,
          .baroAltM = 10.0f, .baroPressurePa = 101000.0f, .baroTempC = 20.0f,
          .accelX = 0.0f, .accelY = 0.0f, .accelZ = 9.81f, .imuTempC = 25.0f },
    }
};

// ── Fixture ───────────────────────────────────────────────────────────────────

struct GpsFixture
{
    ares::sim::SimStorageDriver storage;
    ares::sim::SimGpsDriver     gps;
    ares::sim::SimBaroDriver    baro;
    ares::sim::SimImuDriver     imu;
    ares::sim::SimRadioDriver   radio;

    GpsEntry  gpsEntry;
    BaroEntry baroEntry;
    ComEntry  comEntry;
    ImuEntry  imuEntry;

    MissionScriptEngine engine;

    explicit GpsFixture(const ares::sim::FlightProfile& profile)
        : gps{profile},
          baro{profile},
          imu{profile},
          gpsEntry  { "SIM_GPS",  &gps   },
          baroEntry { "SIM_BARO", &baro  },
          comEntry  { "SIM_COM",  &radio },
          imuEntry  { "SIM_IMU",  &imu   },
          engine{
              storage,
              &gpsEntry,  1U,
              &baroEntry, 1U,
              &comEntry,  1U,
              &imuEntry,  1U
          }
    {}

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
// GPS.sats > 6 with 9 satellites — transition must fire on first tick.
// ─────────────────────────────────────────────────────────────────────────────

void test_gps_sats_transition_fires_above_threshold()
{
    GpsFixture f{kGoodFixProfile};
    f.init("/missions/sats.ams", ares::sim::kScriptGpsSats);
    TEST_ASSERT_TRUE(f.engine.activate("sats.ams"));
    TEST_ASSERT_TRUE(f.engine.arm());

    ares::sim::clock::reset();
    f.engine.tick(ares::sim::clock::nowMs());

    EngineSnapshot snap{};
    f.engine.getSnapshot(snap);

    // Transition GPS.sats > 6 satisfied immediately (9 > 6).
    TEST_ASSERT_EQUAL_STRING("ARMED", snap.stateName);
}

// ─────────────────────────────────────────────────────────────────────────────
// GPS.hdop < 2.0 with HDOP 1.0 — transition must fire on first tick.
// ─────────────────────────────────────────────────────────────────────────────

void test_gps_hdop_transition_fires_below_threshold()
{
    GpsFixture f{kGoodFixProfile};
    f.init("/missions/hdop.ams", ares::sim::kScriptGpsHdop);
    TEST_ASSERT_TRUE(f.engine.activate("hdop.ams"));
    TEST_ASSERT_TRUE(f.engine.arm());

    ares::sim::clock::reset();
    f.engine.tick(ares::sim::clock::nowMs());

    EngineSnapshot snap{};
    f.engine.getSnapshot(snap);

    // Transition GPS.hdop < 2.0 satisfied immediately (1.0 < 2.0).
    TEST_ASSERT_EQUAL_STRING("ARMED", snap.stateName);
}

// ─────────────────────────────────────────────────────────────────────────────
// GPS.sats > 6 with only 4 satellites — transition must NOT fire.
// ─────────────────────────────────────────────────────────────────────────────

void test_gps_sats_blocks_when_below_threshold()
{
    GpsFixture f{kPoorFixProfile};
    f.init("/missions/sats_block.ams", ares::sim::kScriptGpsSatsBlock);
    TEST_ASSERT_TRUE(f.engine.activate("sats_block.ams"));
    TEST_ASSERT_TRUE(f.engine.arm());

    ares::sim::clock::reset();

    // Run 5 ticks at t=0 ms — sats stay at 4, threshold never crossed.
    for (uint8_t i = 0U; i < 5U; i++)
    {
        f.engine.tick(ares::sim::clock::nowMs());
    }

    EngineSnapshot snap{};
    f.engine.getSnapshot(snap);

    // Still in WAIT — GPS.sats > 6 condition not satisfied (4 ≤ 6).
    TEST_ASSERT_EQUAL_STRING("WAIT", snap.stateName);
    TEST_ASSERT_EQUAL(EngineStatus::RUNNING, snap.status);
}

// ─────────────────────────────────────────────────────────────────────────────
// GPS.hdop < 2.0 with HDOP 3.5 — transition must NOT fire.
// ─────────────────────────────────────────────────────────────────────────────

void test_gps_hdop_blocks_when_above_threshold()
{
    GpsFixture f{kPoorFixProfile};
    f.init("/missions/hdop_block.ams", ares::sim::kScriptGpsHdop);
    TEST_ASSERT_TRUE(f.engine.activate("hdop_block.ams"));
    TEST_ASSERT_TRUE(f.engine.arm());

    ares::sim::clock::reset();

    for (uint8_t i = 0U; i < 5U; i++)
    {
        f.engine.tick(ares::sim::clock::nowMs());
    }

    EngineSnapshot snap{};
    f.engine.getSnapshot(snap);

    // Still in WAIT — GPS.hdop < 2.0 not satisfied (3.5 ≥ 2.0).
    TEST_ASSERT_EQUAL_STRING("WAIT", snap.stateName);
    TEST_ASSERT_EQUAL(EngineStatus::RUNNING, snap.status);
}

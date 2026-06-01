/**
 * @file  test_ams_via_routing.cpp
 * @brief Integration tests for A2-3 multi-radio HK routing (every Nms via ALIAS:).
 *
 * Verifies that HK slots with a comAlias dispatch frames to the correct
 * RadioInterface at runtime, independently of the primary COM driver, and that
 * slots without a comAlias continue to use primaryCom_ unchanged.
 *
 * Test count: 3
 */

#include <unity.h>

#include "ams/mission_script_engine.h"
#include "ams/ams_driver_registry.h"

using ares::ams::GpsEntry;
using ares::ams::BaroEntry;
using ares::ams::ComEntry;
using ares::ams::ImuEntry;
using ares::ams::MissionScriptEngine;
using ares::ams::EngineStatus;
using ares::ams::EngineSnapshot;

#include "sim_clock.h"
#include "sim_gps_driver.h"
#include "sim_baro_driver.h"
#include "sim_imu_driver.h"
#include "sim_radio_driver.h"
#include "sim_storage_driver.h"

// ── Minimal flight profile ────────────────────────────────────────────────────

static const ares::sim::FlightProfile kViaProfile = {
    .count   = 1U,
    .samples = {
        { .timeMs        = 0U,
          .gpsLatDeg     = 40.0f,   .gpsLonDeg     = -3.0f,
          .gpsAltM       = 0.0f,    .gpsSpeedKmh   = 0.0f,
          .gpsSats       = 6U,      .gpsFix        = true,
          .baroAltM      = 0.0f,    .baroPressurePa = 101325.0f, .baroTempC = 20.0f,
          .accelX        = 0.0f,    .accelY        = 0.0f,  .accelZ    = 9.81f,
          .imuTempC      = 25.0f },
    }
};

// ── Dual-radio fixture ────────────────────────────────────────────────────────
//
// Two SimRadioDriver instances backed by different ComEntry model strings so
// that separate 'include' directives in the script resolve to distinct drivers.
//   radio1 ("SIM_COM")  → primary COM, target of plain 'every Nms:' slots.
//   radio2 ("SIM_COM2") → secondary COM, target of 'every Nms via COM2:' slots.

struct DualRadioFixture
{
    ares::sim::SimStorageDriver storage;
    ares::sim::SimGpsDriver     gps  { kViaProfile };
    ares::sim::SimBaroDriver    baro { kViaProfile };
    ares::sim::SimImuDriver     imu  { kViaProfile };
    ares::sim::SimRadioDriver   radio1;   ///< primary  — model "SIM_COM"
    ares::sim::SimRadioDriver   radio2;   ///< secondary — model "SIM_COM2"

    GpsEntry  gpsEntry  = { "SIM_GPS",  &gps  };
    BaroEntry baroEntry = { "SIM_BARO", &baro };
    ComEntry  comEntries[2] = {
        { "SIM_COM",  &radio1 },
        { "SIM_COM2", &radio2 },
    };
    ImuEntry  imuEntry  = { "SIM_IMU",  &imu  };

    MissionScriptEngine engine {
        storage,
        &gpsEntry,  1U,
        &baroEntry, 1U,
        comEntries, 2U,
        &imuEntry,  1U
    };

    void init(const char* path, const char* content)
    {
        (void)storage.begin();
        storage.registerFile(path, content);
        (void)gps.begin();
        (void)baro.begin();
        (void)imu.begin();
        (void)radio1.begin();
        (void)radio2.begin();
        (void)engine.begin();
    }
};

// ── Tests ─────────────────────────────────────────────────────────────────────

/**
 * @test  HK slot with 'via COM2:' dispatches frames to radio2 only.
 *
 * A single 'every 500ms via COM2:' slot must send exactly one frame to radio2
 * after 500 ms and nothing to radio1 (the primary COM).
 */
void test_via_slot_routes_to_named_com()
{
    DualRadioFixture f;

    static const char kScript[] =
        "include SIM_COM as COM1\n"
        "include SIM_COM2 as COM2\n"
        "include SIM_BARO as BARO\n"
        "\n"
        "pus.apid = 1\n"
        "pus.service 3 as HK\n"
        "pus.service 5 as EVENT\n"
        "\n"
        "state FLY:\n"
        "  every 500ms via COM2:\n"
        "    HK.report {\n"
        "      baro_alt: BARO.alt\n"
        "    }\n"
        "  transition to END when TIME.elapsed > 5000\n"
        "\n"
        "state END:\n"
        "  on_enter:\n"
        "    EVENT.info \"done\"\n";

    f.init("/missions/via_named.ams", kScript);
    TEST_ASSERT_TRUE(f.engine.activate("via_named.ams"));
    TEST_ASSERT_TRUE(f.engine.arm());

    // Tick at t=0: consume queued LAUNCH TC; TIME.elapsed > 5000 is false; no HK yet.
    f.engine.tick(ares::sim::clock::nowMs());

    // Clear any frames emitted by state entry (e.g. on_enter EVENT).
    f.radio1.resetCapture();
    f.radio2.resetCapture();

    // Advance to 500 ms: the single HK slot is now due.
    ares::sim::clock::advanceMs(500U);
    f.engine.tick(ares::sim::clock::nowMs());

    // Frame must go to radio2 (the COM2 alias), not to the primary radio1.
    TEST_ASSERT_EQUAL(1U, f.radio2.sendCount());
    TEST_ASSERT_EQUAL(0U, f.radio1.sendCount());
}

/**
 * @test  HK slot without 'via' falls back to primaryCom_ (radio1).
 *
 * A plain 'every 500ms:' slot must send its frame to the primary COM driver
 * (radio1) and leave radio2 untouched — regression guard for the default path.
 */
void test_no_via_slot_routes_to_primary_com()
{
    DualRadioFixture f;

    static const char kScript[] =
        "include SIM_COM as COM1\n"
        "include SIM_COM2 as COM2\n"
        "include SIM_BARO as BARO\n"
        "\n"
        "pus.apid = 1\n"
        "pus.service 3 as HK\n"
        "pus.service 5 as EVENT\n"
        "\n"
        "state FLY:\n"
        "  every 500ms:\n"
        "    HK.report {\n"
        "      baro_alt: BARO.alt\n"
        "    }\n"
        "  transition to END when TIME.elapsed > 5000\n"
        "\n"
        "state END:\n"
        "  on_enter:\n"
        "    EVENT.info \"done\"\n";

    f.init("/missions/no_via.ams", kScript);
    TEST_ASSERT_TRUE(f.engine.activate("no_via.ams"));
    TEST_ASSERT_TRUE(f.engine.arm());

    f.engine.tick(ares::sim::clock::nowMs());
    f.radio1.resetCapture();
    f.radio2.resetCapture();

    ares::sim::clock::advanceMs(500U);
    f.engine.tick(ares::sim::clock::nowMs());

    // Frame must go to the primary COM (radio1); radio2 must be silent.
    TEST_ASSERT_EQUAL(1U, f.radio1.sendCount());
    TEST_ASSERT_EQUAL(0U, f.radio2.sendCount());
}

/**
 * @test  Two HK slots at the same interval — one to primary, one to secondary.
 *
 * When both slots fire simultaneously, each must send exactly one frame to its
 * designated radio without cross-contamination.
 */
void test_two_slots_two_radios_independent_counts()
{
    DualRadioFixture f;

    static const char kScript[] =
        "include SIM_COM as COM1\n"
        "include SIM_COM2 as COM2\n"
        "include SIM_BARO as BARO\n"
        "\n"
        "pus.apid = 1\n"
        "pus.service 3 as HK\n"
        "pus.service 5 as EVENT\n"
        "\n"
        "state FLY:\n"
        "  every 500ms:\n"
        "    HK.report {\n"
        "      baro_alt: BARO.alt\n"
        "    }\n"
        "  every 500ms via COM2:\n"
        "    HK.report {\n"
        "      baro_alt: BARO.alt\n"
        "    }\n"
        "  transition to END when TIME.elapsed > 5000\n"
        "\n"
        "state END:\n"
        "  on_enter:\n"
        "    EVENT.info \"done\"\n";

    f.init("/missions/two_slots.ams", kScript);
    TEST_ASSERT_TRUE(f.engine.activate("two_slots.ams"));
    TEST_ASSERT_TRUE(f.engine.arm());

    f.engine.tick(ares::sim::clock::nowMs());
    f.radio1.resetCapture();
    f.radio2.resetCapture();

    ares::sim::clock::advanceMs(500U);
    f.engine.tick(ares::sim::clock::nowMs());

    // Each radio receives exactly one frame from its respective slot.
    TEST_ASSERT_EQUAL(1U, f.radio1.sendCount());
    TEST_ASSERT_EQUAL(1U, f.radio2.sendCount());
}

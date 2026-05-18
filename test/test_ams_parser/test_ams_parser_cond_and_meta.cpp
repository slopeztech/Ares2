/**
 * @file  test_ams_parser_cond_and_meta.cpp
 * @brief Parser corpus — falling/rising conditions, delta conditions,
 *        and radio.config directive coverage.
 *
 * These tests use the local `CondParserFixture` defined in this file and
 * exercise `parseScriptLocked` paths not currently covered by the SITL
 * integration suite.
 *
 * Test count: 9
 */

#include <unity.h>
#include <cstdio>
#include <cstring>

#include "ams/mission_script_engine.h"
#include "ams/ams_driver_registry.h"
#include "hal/storage/storage_interface.h"

using ares::ams::BaroEntry;
using ares::ams::ComEntry;
using ares::ams::EngineSnapshot;
using ares::ams::EngineStatus;
using ares::ams::GpsEntry;
using ares::ams::ImuEntry;
using ares::ams::MissionScriptEngine;

#include "sim_clock.h"
#include "sim_baro_driver.h"
#include "sim_gps_driver.h"
#include "sim_imu_driver.h"
#include "sim_radio_driver.h"
#include "sim_storage_driver.h"

// ── Quiescent profile ─────────────────────────────────────────────────────────

static const ares::sim::FlightProfile kCondProfile = {
    .count   = 1U,
    .samples = {
        { .timeMs        = 0U,
          .gpsLatDeg     = 0.0f,   .gpsLonDeg  = 0.0f,    .gpsAltM      = 0.0f,
          .gpsSpeedKmh   = 0.0f,   .gpsHdop    = 1.0f,
          .gpsSats       = 6U,     .gpsFix     = true,
          .baroAltM      = 0.0f,   .baroPressurePa = 101325.0f, .baroTempC = 20.0f,
          .accelX        = 0.0f,   .accelY     = 0.0f,    .accelZ      = 9.81f,
          .imuTempC      = 25.0f },
    }
};

// ── Fixture ───────────────────────────────────────────────────────────────────

struct CondParserFixture
{
    ares::sim::SimStorageDriver storage;
    ares::sim::SimGpsDriver     gps  { kCondProfile };
    ares::sim::SimBaroDriver    baro { kCondProfile };
    ares::sim::SimImuDriver     imu  { kCondProfile };
    ares::sim::SimRadioDriver   radio;

    GpsEntry  gpsEntry  = { "SIM_GPS",  &gps   };
    BaroEntry baroEntry = { "SIM_BARO", &baro  };
    ComEntry  comEntry  = { "SIM_COM",  &radio };
    ImuEntry  imuEntry  = { "SIM_IMU",  &imu   };

    MissionScriptEngine engine {
        storage,
        &gpsEntry,  1U,
        &baroEntry, 1U,
        &comEntry,  1U,
        &imuEntry,  1U
    };

    bool load(const char* name, const char* content)
    {
        char path[64] = {};
        (void)snprintf(path, sizeof(path), "/missions/%s", name);

        (void)storage.begin();
        storage.registerFile(path, content);
        (void)gps.begin();
        (void)baro.begin();
        (void)imu.begin();
        (void)radio.begin();
        (void)engine.begin();

        return engine.activate(name);
    }
};

// ── Test 1: radio.config directive parses successfully ────────────────────────

void test_parser_radio_config_accepted()
{
    CondParserFixture fx;

    static const char kScript[] =
        "include SIM_BARO as BARO\n"
        "include SIM_COM as COM\n"
        "\n"
        "pus.apid = 1\n"
        "pus.service 3 as HK\n"
        "pus.service 5 as EVENT\n"
        "pus.service 1 as TC\n"
        "\n"
        "radio.config telem_interval = 5000\n"
        "\n"
        "state WAIT:\n"
        "  transition to END when TC.command == LAUNCH\n"
        "\n"
        "state END:\n"
        "  on_enter:\n"
        "    EVENT.info \"OK\"\n";

    const bool ok = fx.load("radio_config_test.ams", kScript);

    EngineSnapshot snap {};
    fx.engine.getSnapshot(snap);
    TEST_ASSERT_TRUE_MESSAGE(ok, snap.lastError);
}

// ── Test 2: BARO.alt falling condition parses successfully ────────────────────

void test_parser_falling_cond_accepted()
{
    CondParserFixture fx;

    static const char kScript[] =
        "include SIM_BARO as BARO\n"
        "include SIM_COM as COM\n"
        "\n"
        "pus.apid = 1\n"
        "pus.service 5 as EVENT\n"
        "pus.service 1 as TC\n"
        "\n"
        "state FLY:\n"
        "  transition to END when BARO.alt falling\n"
        "\n"
        "state END:\n"
        "  on_enter:\n"
        "    EVENT.info \"DONE\"\n";

    const bool ok = fx.load("falling_cond_test.ams", kScript);

    EngineSnapshot snap {};
    fx.engine.getSnapshot(snap);
    TEST_ASSERT_TRUE_MESSAGE(ok, snap.lastError);
}

// ── Test 3: BARO.alt rising condition parses successfully ─────────────────────

void test_parser_rising_cond_accepted()
{
    CondParserFixture fx;

    static const char kScript[] =
        "include SIM_BARO as BARO\n"
        "include SIM_COM as COM\n"
        "\n"
        "pus.apid = 1\n"
        "pus.service 5 as EVENT\n"
        "pus.service 1 as TC\n"
        "\n"
        "state FLY:\n"
        "  transition to END when BARO.alt rising\n"
        "\n"
        "state END:\n"
        "  on_enter:\n"
        "    EVENT.info \"DONE\"\n";

    const bool ok = fx.load("rising_cond_test.ams", kScript);

    EngineSnapshot snap {};
    fx.engine.getSnapshot(snap);
    TEST_ASSERT_TRUE_MESSAGE(ok, snap.lastError);
}

// ── Test 4: BARO.alt delta > 50.0 parses successfully ────────────────────────

void test_parser_delta_gt_accepted()
{
    CondParserFixture fx;

    static const char kScript[] =
        "include SIM_BARO as BARO\n"
        "include SIM_COM as COM\n"
        "\n"
        "pus.apid = 1\n"
        "pus.service 5 as EVENT\n"
        "pus.service 1 as TC\n"
        "\n"
        "state FLY:\n"
        "  transition to END when BARO.alt delta > 50.0\n"
        "\n"
        "state END:\n"
        "  on_enter:\n"
        "    EVENT.info \"DONE\"\n";

    const bool ok = fx.load("delta_gt_test.ams", kScript);

    EngineSnapshot snap {};
    fx.engine.getSnapshot(snap);
    TEST_ASSERT_TRUE_MESSAGE(ok, snap.lastError);
}

// ── Test 5: falling condition with unknown alias is rejected ──────────────────

void test_parser_falling_unknown_alias_error()
{
    CondParserFixture fx;

    static const char kScript[] =
        "include SIM_BARO as BARO\n"
        "include SIM_COM as COM\n"
        "\n"
        "pus.apid = 1\n"
        "pus.service 5 as EVENT\n"
        "\n"
        "state FLY:\n"
        "  transition to END when NOALIAS.alt falling\n"
        "\n"
        "state END:\n"
        "  on_enter:\n"
        "    EVENT.info \"DONE\"\n";

    const bool ok = fx.load("falling_bad_alias.ams", kScript);

    TEST_ASSERT_FALSE_MESSAGE(ok, "unknown alias in falling cond must be rejected");

    EngineSnapshot snap {};
    fx.engine.getSnapshot(snap);
    TEST_ASSERT_EQUAL(EngineStatus::ERROR, snap.status);
    TEST_ASSERT_TRUE(snap.lastError[0] != '\0');
}

// ── Test 6: delta condition with invalid threshold rejected ───────────────────

void test_parser_delta_invalid_threshold_error()
{
    CondParserFixture fx;

    static const char kScript[] =
        "include SIM_BARO as BARO\n"
        "include SIM_COM as COM\n"
        "\n"
        "pus.apid = 1\n"
        "pus.service 5 as EVENT\n"
        "\n"
        "state FLY:\n"
        "  transition to END when BARO.alt delta > notanumber\n"
        "\n"
        "state END:\n"
        "  on_enter:\n"
        "    EVENT.info \"DONE\"\n";

    const bool ok = fx.load("delta_bad_thr.ams", kScript);

    TEST_ASSERT_FALSE_MESSAGE(ok, "non-numeric delta threshold must be rejected");

    EngineSnapshot snap {};
    fx.engine.getSnapshot(snap);
    TEST_ASSERT_EQUAL(EngineStatus::ERROR, snap.status);
}

// ── Test 7: delta condition with bad operator rejected ────────────────────────

void test_parser_delta_bad_operator_error()
{
    CondParserFixture fx;

    static const char kScript[] =
        "include SIM_BARO as BARO\n"
        "include SIM_COM as COM\n"
        "\n"
        "pus.apid = 1\n"
        "pus.service 5 as EVENT\n"
        "\n"
        "state FLY:\n"
        "  transition to END when BARO.alt delta != 50.0\n"
        "\n"
        "state END:\n"
        "  on_enter:\n"
        "    EVENT.info \"DONE\"\n";

    const bool ok = fx.load("delta_bad_op.ams", kScript);

    TEST_ASSERT_FALSE_MESSAGE(ok, "unsupported delta operator must be rejected");

    EngineSnapshot snap {};
    fx.engine.getSnapshot(snap);
    TEST_ASSERT_EQUAL(EngineStatus::ERROR, snap.status);
}

// ── Test 8: delta condition with invalid LHS (no dot) rejected ────────────────

void test_parser_delta_invalid_lhs_error()
{
    CondParserFixture fx;

    static const char kScript[] =
        "include SIM_BARO as BARO\n"
        "include SIM_COM as COM\n"
        "\n"
        "pus.apid = 1\n"
        "pus.service 5 as EVENT\n"
        "\n"
        "state FLY:\n"
        "  transition to END when no_dot_here delta < 5.0\n"
        "\n"
        "state END:\n"
        "  on_enter:\n"
        "    EVENT.info \"DONE\"\n";

    const bool ok = fx.load("delta_bad_lhs.ams", kScript);

    TEST_ASSERT_FALSE_MESSAGE(ok, "missing dot in delta LHS must be rejected");

    EngineSnapshot snap {};
    fx.engine.getSnapshot(snap);
    TEST_ASSERT_EQUAL(EngineStatus::ERROR, snap.status);
}

// ── Test 9: sensor condition with unknown alias rejected ──────────────────────

void test_parser_sensor_cond_unknown_alias_error()
{
    CondParserFixture fx;

    static const char kScript[] =
        "include SIM_BARO as BARO\n"
        "include SIM_COM as COM\n"
        "\n"
        "pus.apid = 1\n"
        "pus.service 5 as EVENT\n"
        "\n"
        "state FLY:\n"
        "  transition to END when UNKNOWN_ALIAS.alt > 100.0\n"
        "\n"
        "state END:\n"
        "  on_enter:\n"
        "    EVENT.info \"DONE\"\n";

    const bool ok = fx.load("sensor_cond_bad_alias.ams", kScript);

    TEST_ASSERT_FALSE_MESSAGE(ok, "unknown alias in sensor condition must be rejected");

    EngineSnapshot snap {};
    fx.engine.getSnapshot(snap);
    TEST_ASSERT_EQUAL(EngineStatus::ERROR, snap.status);
}

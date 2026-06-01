/**
 * @file  test_ams_parser_cond_and_meta.cpp
 * @brief Parser corpus — falling/rising conditions, delta conditions,
 *        and radio.config directive coverage.
 *
 * These tests use the local `CondParserFixture` defined in this file and
 * exercise `parseScriptLocked` paths not currently covered by the SITL
 * integration suite.
 *
 * Test count: 21
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
// ── Helpers shared by radio.config rejection tests ────────────────────────────

namespace {
    bool loadScriptWithRadioValue(CondParserFixture& fx,
                                  const char*        scriptName,
                                  const char*        radioLine)
    {
        char script[512] = {};
        snprintf(script, sizeof(script),
            "include SIM_BARO as BARO\n"
            "include SIM_COM as COM\n"
            "\n"
            "pus.apid = 1\n"
            "pus.service 3 as HK\n"
            "pus.service 5 as EVENT\n"
            "pus.service 1 as TC\n"
            "\n"
            "%s\n"
            "\n"
            "state WAIT:\n"
            "  transition to END when TC.command == LAUNCH\n"
            "\n"
            "state END:\n"
            "  on_enter:\n"
            "    EVENT.info \"OK\"\n",
            radioLine);
        return fx.load(scriptName, script);
    }
} // namespace

// ── Test 10: radio.config with NaN value is rejected ─────────────────────────

void test_parser_radio_config_nan_rejected()
{
    CondParserFixture fx;
    const bool ok = loadScriptWithRadioValue(
        fx,
        "radio_config_nan.ams",
        "radio.config telem_interval = nan");

    TEST_ASSERT_FALSE_MESSAGE(ok, "radio.config nan must be rejected");

    EngineSnapshot snap {};
    fx.engine.getSnapshot(snap);
    TEST_ASSERT_EQUAL(EngineStatus::ERROR, snap.status);
    TEST_ASSERT_TRUE(snap.lastError[0] != '\0');
}

// ── Test 11: radio.config with Inf value is rejected ─────────────────────────

void test_parser_radio_config_inf_rejected()
{
    CondParserFixture fx;
    const bool ok = loadScriptWithRadioValue(
        fx,
        "radio_config_inf.ams",
        "radio.config telem_interval = inf");

    TEST_ASSERT_FALSE_MESSAGE(ok, "radio.config inf must be rejected");

    EngineSnapshot snap {};
    fx.engine.getSnapshot(snap);
    TEST_ASSERT_EQUAL(EngineStatus::ERROR, snap.status);
    TEST_ASSERT_TRUE(snap.lastError[0] != '\0');
}

// ── Test 12: radio.config with alphanumeric suffix is rejected ────────────────

void test_parser_radio_config_alpha_suffix_rejected()
{
    CondParserFixture fx;
    const bool ok = loadScriptWithRadioValue(
        fx,
        "radio_config_alpha_suffix.ams",
        "radio.config telem_interval = 5000abc");

    TEST_ASSERT_FALSE_MESSAGE(ok, "radio.config value with trailing 'abc' must be rejected");

    EngineSnapshot snap {};
    fx.engine.getSnapshot(snap);
    TEST_ASSERT_EQUAL(EngineStatus::ERROR, snap.status);
    TEST_ASSERT_TRUE(snap.lastError[0] != '\0');
}

// ── Test 13: radio.config with extra token is rejected ───────────────────────

void test_parser_radio_config_extra_token_rejected()
{
    CondParserFixture fx;
    const bool ok = loadScriptWithRadioValue(
        fx,
        "radio_config_extra_token.ams",
        "radio.config telem_interval = 5000 extra");

    TEST_ASSERT_FALSE_MESSAGE(ok, "radio.config value with trailing ' extra' must be rejected");

    EngineSnapshot snap {};
    fx.engine.getSnapshot(snap);
    TEST_ASSERT_EQUAL(EngineStatus::ERROR, snap.status);
    TEST_ASSERT_TRUE(snap.lastError[0] != '\0');
}
// ── Test 14: log_every below LOG_INTERVAL_MIN_MS is rejected ─────────────────

void test_parser_log_every_below_min_rejected()
{
    CondParserFixture fx;

    static const char kScript[] =
        "include SIM_BARO as BARO\n"
        "include SIM_COM as COM\n"
        "\n"
        "pus.apid = 1\n"
        "pus.service 1 as TC\n"
        "\n"
        "state WAIT:\n"
        "  log_every 1ms:\n"
        "    LOG.report {\n"
        "      baro_alt: BARO.alt\n"
        "    }\n"
        "  transition to END when TC.command == LAUNCH\n"
        "\n"
        "state END:\n";

    const bool ok = fx.load("log_every_below_min.ams", kScript);

    TEST_ASSERT_FALSE_MESSAGE(ok,
        "log_every 1ms must be rejected (below LOG_INTERVAL_MIN_MS = 100)");

    EngineSnapshot snap {};
    fx.engine.getSnapshot(snap);
    TEST_ASSERT_EQUAL(EngineStatus::ERROR, snap.status);
    TEST_ASSERT_TRUE(snap.lastError[0] != '\0');
}

// ── Test 15: log_every at LOG_INTERVAL_MIN_MS is accepted ────────────────────

void test_parser_log_every_at_min_accepted()
{
    CondParserFixture fx;

    static const char kScript[] =
        "include SIM_BARO as BARO\n"
        "include SIM_COM as COM\n"
        "\n"
        "pus.apid = 1\n"
        "pus.service 1 as TC\n"
        "\n"
        "state WAIT:\n"
        "  log_every 100ms:\n"
        "    LOG.report {\n"
        "      baro_alt: BARO.alt\n"
        "    }\n"
        "  transition to END when TC.command == LAUNCH\n"
        "\n"
        "state END:\n";

    const bool ok = fx.load("log_every_at_min.ams", kScript);

    TEST_ASSERT_TRUE_MESSAGE(ok,
        "log_every 100ms must be accepted (equals LOG_INTERVAL_MIN_MS)");

    EngineSnapshot snap {};
    fx.engine.getSnapshot(snap);
    TEST_ASSERT_EQUAL(EngineStatus::LOADED, snap.status);
}

// ── Test 16: pulse.min_altitude 0 is rejected (below [1,50000]) ───────────────
// Exercises the TokenCursor readUint32 + val==0 guard added in P3-4.

void test_parser_pulse_min_alt_zero_rejected()
{
    CondParserFixture fx;

    static const char kScript[] =
        "include SIM_BARO as BARO\n"
        "include SIM_COM as COM\n"
        "\n"
        "pus.apid = 1\n"
        "pus.service 1 as TC\n"
        "\n"
        "pulse.channel A\n"
        "pulse.min_altitude 0\n"
        "\n"
        "state WAIT:\n"
        "  transition to END when TC.command == LAUNCH\n"
        "\n"
        "state END:\n";

    const bool ok = fx.load("pulse_min_alt_zero.ams", kScript);

    TEST_ASSERT_FALSE_MESSAGE(ok,
        "pulse.min_altitude 0 must be rejected (below minimum of 1 m)");

    EngineSnapshot snap {};
    fx.engine.getSnapshot(snap);
    TEST_ASSERT_EQUAL(EngineStatus::ERROR, snap.status);
    TEST_ASSERT_TRUE(snap.lastError[0] != '\0');
}

// ── Test 17: pulse.min_altitude 1 is accepted (at lower bound) ────────────────

void test_parser_pulse_min_alt_boundary_accepted()
{
    CondParserFixture fx;

    static const char kScript[] =
        "include SIM_BARO as BARO\n"
        "include SIM_COM as COM\n"
        "\n"
        "pus.apid = 1\n"
        "pus.service 1 as TC\n"
        "\n"
        "pulse.channel A\n"
        "pulse.min_altitude 1\n"
        "\n"
        "state WAIT:\n"
        "  transition to END when TC.command == LAUNCH\n"
        "\n"
        "state END:\n";

    const bool ok = fx.load("pulse_min_alt_boundary.ams", kScript);

    TEST_ASSERT_TRUE_MESSAGE(ok,
        "pulse.min_altitude 1 must be accepted (equals lower bound)");

    EngineSnapshot snap {};
    fx.engine.getSnapshot(snap);
    TEST_ASSERT_EQUAL(EngineStatus::LOADED, snap.status);
}

// ── Test 18: pulse.safe_delay below minimum (99 ms) is rejected ───────────────
// Exercises the val < 100 guard converted to TokenCursor in P3-4.

void test_parser_pulse_safe_delay_below_min_rejected()
{
    CondParserFixture fx;

    static const char kScript[] =
        "include SIM_COM as COM\n"
        "\n"
        "pus.apid = 1\n"
        "pus.service 1 as TC\n"
        "\n"
        "pulse.channel A\n"
        "pulse.safe_delay 99\n"
        "\n"
        "state WAIT:\n"
        "  transition to END when TC.command == LAUNCH\n"
        "\n"
        "state END:\n";

    const bool ok = fx.load("pulse_safe_delay_below_min.ams", kScript);

    TEST_ASSERT_FALSE_MESSAGE(ok,
        "pulse.safe_delay 99 must be rejected (below minimum of 100 ms)");

    EngineSnapshot snap {};
    fx.engine.getSnapshot(snap);
    TEST_ASSERT_EQUAL(EngineStatus::ERROR, snap.status);
    TEST_ASSERT_TRUE(snap.lastError[0] != '\0');
}

// ── Test 19: radio.config dotted key (monitor.alt.high) is accepted ───────────
// Exercises TokenCursor.readKeyBounded which allows '.' in the key name.

void test_parser_radio_config_dotted_key_accepted()
{
    CondParserFixture fx;

    static const char kScript[] =
        "include SIM_BARO as BARO\n"
        "include SIM_COM as COM\n"
        "\n"
        "pus.apid = 1\n"
        "pus.service 1 as TC\n"
        "pus.service 5 as EVENT\n"
        "\n"
        "radio.config monitor.alt.high = 5000\n"
        "\n"
        "state WAIT:\n"
        "  transition to END when TC.command == LAUNCH\n"
        "\n"
        "state END:\n"
        "  on_enter:\n"
        "    EVENT.info \"done\"\n";

    const bool ok = fx.load("radio_config_dotted_key.ams", kScript);

    EngineSnapshot snap {};
    fx.engine.getSnapshot(snap);
    TEST_ASSERT_TRUE_MESSAGE(ok, snap.lastError);
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

// ── Test 20: every Nms via COM_ALIAS: is accepted ────────────────────────────
// Exercises the A2-3 multi-radio routing extension: a valid COM alias in the
// via clause must be accepted and the slot's comAlias must be populated.

void test_parser_every_via_com_accepted()
{
    CondParserFixture fx;

    static const char kScript[] =
        "include SIM_COM as COM1\n"
        "include SIM_BARO as BARO\n"
        "\n"
        "pus.apid = 1\n"
        "pus.service 3 as HK\n"
        "pus.service 5 as EVENT\n"
        "pus.service 1 as TC\n"
        "\n"
        "state FLY:\n"
        "  every 500ms via COM1:\n"
        "    HK.report {\n"
        "      baro_alt: BARO.alt\n"
        "    }\n"
        "  transition to END when TC.command == LAUNCH\n"
        "\n"
        "state END:\n"
        "  on_enter:\n"
        "    EVENT.info \"done\"\n";

    const bool ok = fx.load("every_via_com.ams", kScript);

    EngineSnapshot snap {};
    fx.engine.getSnapshot(snap);
    TEST_ASSERT_TRUE_MESSAGE(ok, snap.lastError);
    TEST_ASSERT_EQUAL(EngineStatus::LOADED, snap.status);
}

// ── Test 21: every Nms via non-COM alias is rejected ─────────────────────────
// A BARO alias is not a COM peripheral; the parser must reject the via clause.

void test_parser_every_via_noncom_rejected()
{
    CondParserFixture fx;

    static const char kScript[] =
        "include SIM_BARO as BARO\n"
        "include SIM_COM as COM\n"
        "\n"
        "pus.apid = 1\n"
        "pus.service 3 as HK\n"
        "pus.service 1 as TC\n"
        "\n"
        "state FLY:\n"
        "  every 500ms via BARO:\n"
        "    HK.report {\n"
        "      baro_alt: BARO.alt\n"
        "    }\n"
        "  transition to END when TC.command == LAUNCH\n"
        "\n"
        "state END:\n";

    const bool ok = fx.load("every_via_noncom.ams", kScript);

    TEST_ASSERT_FALSE_MESSAGE(ok, "every via non-COM alias must be rejected");

    EngineSnapshot snap {};
    fx.engine.getSnapshot(snap);
    TEST_ASSERT_EQUAL(EngineStatus::ERROR, snap.status);
    TEST_ASSERT_TRUE(snap.lastError[0] != '\0');
}

// ── Test 22: every Nms via unregistered alias is rejected ─────────────────────
// An alias that was never declared with 'include' must be rejected.

void test_parser_every_via_unknown_alias_rejected()
{
    CondParserFixture fx;

    static const char kScript[] =
        "include SIM_COM as COM\n"
        "include SIM_BARO as BARO\n"
        "\n"
        "pus.apid = 1\n"
        "pus.service 3 as HK\n"
        "pus.service 1 as TC\n"
        "\n"
        "state FLY:\n"
        "  every 500ms via GHOST:\n"
        "    HK.report {\n"
        "      baro_alt: BARO.alt\n"
        "    }\n"
        "  transition to END when TC.command == LAUNCH\n"
        "\n"
        "state END:\n";

    const bool ok = fx.load("every_via_ghost.ams", kScript);

    TEST_ASSERT_FALSE_MESSAGE(ok, "every via unregistered alias must be rejected");

    EngineSnapshot snap {};
    fx.engine.getSnapshot(snap);
    TEST_ASSERT_EQUAL(EngineStatus::ERROR, snap.status);
    TEST_ASSERT_TRUE(snap.lastError[0] != '\0');
}

// ── Test 23: every Nms via COM: with interval below minimum is rejected ────────
// The 100 ms floor (TELEMETRY_INTERVAL_MIN / APUS-19.3) applies even with via.

void test_parser_every_via_below_min_interval_rejected()
{
    CondParserFixture fx;

    static const char kScript[] =
        "include SIM_COM as COM\n"
        "include SIM_BARO as BARO\n"
        "\n"
        "pus.apid = 1\n"
        "pus.service 3 as HK\n"
        "pus.service 1 as TC\n"
        "\n"
        "state FLY:\n"
        "  every 99ms via COM:\n"
        "    HK.report {\n"
        "      baro_alt: BARO.alt\n"
        "    }\n"
        "  transition to END when TC.command == LAUNCH\n"
        "\n"
        "state END:\n";

    const bool ok = fx.load("every_via_below_min.ams", kScript);

    TEST_ASSERT_FALSE_MESSAGE(ok, "every via interval below 100 ms must be rejected");

    EngineSnapshot snap {};
    fx.engine.getSnapshot(snap);
    TEST_ASSERT_EQUAL(EngineStatus::ERROR, snap.status);
    TEST_ASSERT_TRUE(snap.lastError[0] != '\0');
}

// ── Test 24: every Nms via COM: with trailing garbage is rejected ─────────────
// Characters after the closing ':' are not allowed.

void test_parser_every_via_trailing_garbage_rejected()
{
    CondParserFixture fx;

    static const char kScript[] =
        "include SIM_COM as COM\n"
        "include SIM_BARO as BARO\n"
        "\n"
        "pus.apid = 1\n"
        "pus.service 3 as HK\n"
        "pus.service 1 as TC\n"
        "\n"
        "state FLY:\n"
        "  every 500ms via COM: garbage\n"
        "    HK.report {\n"
        "      baro_alt: BARO.alt\n"
        "    }\n"
        "  transition to END when TC.command == LAUNCH\n"
        "\n"
        "state END:\n";

    const bool ok = fx.load("every_via_trailing.ams", kScript);

    TEST_ASSERT_FALSE_MESSAGE(ok, "every via with trailing chars must be rejected");

    EngineSnapshot snap {};
    fx.engine.getSnapshot(snap);
    TEST_ASSERT_EQUAL(EngineStatus::ERROR, snap.status);
    TEST_ASSERT_TRUE(snap.lastError[0] != '\0');
}

// ── Test 25: every Nms via with empty alias name is rejected ──────────────────
// "every 500ms via :" has nothing between 'via ' and ':'.

void test_parser_every_via_empty_alias_rejected()
{
    CondParserFixture fx;

    static const char kScript[] =
        "include SIM_COM as COM\n"
        "include SIM_BARO as BARO\n"
        "\n"
        "pus.apid = 1\n"
        "pus.service 3 as HK\n"
        "pus.service 1 as TC\n"
        "\n"
        "state FLY:\n"
        "  every 500ms via :\n"
        "    HK.report {\n"
        "      baro_alt: BARO.alt\n"
        "    }\n"
        "  transition to END when TC.command == LAUNCH\n"
        "\n"
        "state END:\n";

    const bool ok = fx.load("every_via_empty_alias.ams", kScript);

    TEST_ASSERT_FALSE_MESSAGE(ok, "every via with empty alias must be rejected");

    EngineSnapshot snap {};
    fx.engine.getSnapshot(snap);
    TEST_ASSERT_EQUAL(EngineStatus::ERROR, snap.status);
    TEST_ASSERT_TRUE(snap.lastError[0] != '\0');
}

// ── Test 26: two every blocks — one plain, one with via — both accepted ────────
// A state may have multiple HK slots; mixing plain 'every' and 'every via'
// must parse cleanly without conflating the slot COM alias fields.

void test_parser_every_via_two_slots_accepted()
{
    CondParserFixture fx;

    static const char kScript[] =
        "include SIM_COM as COM1\n"
        "include SIM_COM as COM2\n"
        "include SIM_BARO as BARO\n"
        "\n"
        "pus.apid = 1\n"
        "pus.service 3 as HK\n"
        "pus.service 5 as EVENT\n"
        "pus.service 1 as TC\n"
        "\n"
        "state FLY:\n"
        "  every 500ms:\n"
        "    HK.report {\n"
        "      baro_alt: BARO.alt\n"
        "    }\n"
        "  every 1000ms via COM2:\n"
        "    HK.report {\n"
        "      baro_alt: BARO.alt\n"
        "    }\n"
        "  transition to END when TC.command == LAUNCH\n"
        "\n"
        "state END:\n"
        "  on_enter:\n"
        "    EVENT.info \"done\"\n";

    const bool ok = fx.load("every_via_two_slots.ams", kScript);

    EngineSnapshot snap {};
    fx.engine.getSnapshot(snap);
    TEST_ASSERT_TRUE_MESSAGE(ok, snap.lastError);
    TEST_ASSERT_EQUAL(EngineStatus::LOADED, snap.status);
}

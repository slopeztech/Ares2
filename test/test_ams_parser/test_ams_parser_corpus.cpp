/**
 * @file  test_ams_parser_corpus.cpp
 * @brief Parser robustness corpus — edge-case and malformed script inputs.
 *
 * Tests exercise `parseScriptLocked` paths not covered by the SITL integration
 * suite: UTF-8 BOM, CRLF / mixed line endings, missing trailing newline, C-style
 * block comments, scripts with only line comments, and non-ASCII identifier bytes.
 *
 * All tests run under [env:sim] (platform = native + AMS engine sources).
 * No arm() / tick() calls are needed — activate() alone drives all paths.
 *
 * Test count: 7
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

// ── Quiescent flight profile (sensors are never ticked in these tests) ────────

static const ares::sim::FlightProfile kParserProfile = {
    .count   = 1U,
    .samples = {
        { .timeMs        = 0U,
          .gpsLatDeg     = 0.0f,      .gpsLonDeg  = 0.0f,      .gpsAltM      = 0.0f,
          .gpsSpeedKmh   = 0.0f,      .gpsHdop    = 1.0f,
          .gpsSats       = 6U,        .gpsFix     = true,
          .baroAltM      = 0.0f,      .baroPressurePa = 101325.0f, .baroTempC = 20.0f,
          .accelX        = 0.0f,      .accelY     = 0.0f,      .accelZ      = 9.81f,
          .imuTempC      = 25.0f },
    }
};

// ── Test fixture ──────────────────────────────────────────────────────────────

struct ParserFixture
{
    ares::sim::SimStorageDriver storage;
    ares::sim::SimGpsDriver     gps  { kParserProfile };
    ares::sim::SimBaroDriver    baro { kParserProfile };
    ares::sim::SimImuDriver     imu  { kParserProfile };
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

// ── Test 1: UTF-8 BOM at start of file ───────────────────────────────────────

void test_parser_utf8_bom_rejected()
{
    ParserFixture fx;

    static const char kBomScript[] =
        "\xEF\xBB\xBF"
        "pus.apid = 1\n"
        "pus.service 1 as TC\n"
        "pus.service 5 as EVENT\n"
        "\n"
        "state WAIT:\n"
        "  transition to DONE when TC.command == LAUNCH\n"
        "\n"
        "state DONE:\n"
        "  on_enter:\n"
        "    EVENT.info \"OK\"\n";

    const bool ok = fx.load("bom_test.ams", kBomScript);

    TEST_ASSERT_FALSE_MESSAGE(ok, "BOM-prefixed script must not activate");

    EngineSnapshot snap {};
    fx.engine.getSnapshot(snap);
    TEST_ASSERT_EQUAL_MESSAGE(EngineStatus::ERROR, snap.status,
        "engine status must be ERROR after BOM parse failure");
    TEST_ASSERT_TRUE_MESSAGE(snap.lastError[0] != '\0',
        "lastError must be set after BOM parse failure");
}

// ── Test 2: CRLF line endings ─────────────────────────────────────────────────

void test_parser_crlf_endings_accepted()
{
    ParserFixture fx;

    static const char kCrlfScript[] =
        "pus.apid = 1\r\n"
        "pus.service 1 as TC\r\n"
        "pus.service 5 as EVENT\r\n"
        "\r\n"
        "state WAIT:\r\n"
        "  transition to DONE when TC.command == LAUNCH\r\n"
        "\r\n"
        "state DONE:\r\n"
        "  on_enter:\r\n"
        "    EVENT.info \"OK\"\r\n";

    const bool ok = fx.load("crlf_test.ams", kCrlfScript);

    // Use lastError as the failure message so failures are self-diagnosing.
    EngineSnapshot snap {};
    fx.engine.getSnapshot(snap);
    TEST_ASSERT_TRUE_MESSAGE(ok, snap.lastError);
    TEST_ASSERT_EQUAL_MESSAGE(EngineStatus::LOADED, snap.status,
        "engine status must be LOADED for CRLF script");
}

// ── Test 3: Mixed CRLF and LF line endings ────────────────────────────────────

void test_parser_mixed_crlf_lf_accepted()
{
    ParserFixture fx;

    static const char kMixedEolScript[] =
        "pus.apid = 1\r\n"
        "pus.service 1 as TC\n"
        "pus.service 5 as EVENT\r\n"
        "\n"
        "state WAIT:\r\n"
        "  transition to DONE when TC.command == LAUNCH\n"
        "\r\n"
        "state DONE:\n"
        "  on_enter:\r\n"
        "    EVENT.info \"OK\"\n";

    const bool ok = fx.load("mixed_eol_test.ams", kMixedEolScript);

    TEST_ASSERT_TRUE_MESSAGE(ok, "Mixed CRLF/LF script must activate successfully");

    EngineSnapshot snap {};
    fx.engine.getSnapshot(snap);
    TEST_ASSERT_EQUAL_MESSAGE(EngineStatus::LOADED, snap.status,
        "engine status must be LOADED for mixed-EOL script");
}

// ── Test 4: Script without a trailing newline ─────────────────────────────────

void test_parser_no_trailing_newline_accepted()
{
    ParserFixture fx;

    static const char kNoNewlineScript[] =
        "pus.apid = 1\n"
        "pus.service 1 as TC\n"
        "pus.service 5 as EVENT\n"
        "\n"
        "state WAIT:\n"
        "  transition to DONE when TC.command == LAUNCH\n"
        "\n"
        "state DONE:\n"
        "  on_enter:\n"
        "    EVENT.info \"OK\"";

    const bool ok = fx.load("no_newline_test.ams", kNoNewlineScript);

    TEST_ASSERT_TRUE_MESSAGE(ok, "Script without trailing newline must activate successfully");

    EngineSnapshot snap {};
    fx.engine.getSnapshot(snap);
    TEST_ASSERT_EQUAL_MESSAGE(EngineStatus::LOADED, snap.status,
        "engine status must be LOADED when trailing newline is absent");
}

// ── Test 5: C-style block comment (unsupported syntax) ────────────────────────

void test_parser_block_comment_rejected()
{
    ParserFixture fx;

    static const char kBlockCommentScript[] =
        "/* Configuration block */\n"
        "pus.apid = 1\n"
        "pus.service 1 as TC\n"
        "pus.service 5 as EVENT\n"
        "\n"
        "state WAIT:\n"
        "  transition to DONE when TC.command == LAUNCH\n"
        "\n"
        "state DONE:\n"
        "  on_enter:\n"
        "    EVENT.info \"OK\"\n";

    const bool ok = fx.load("block_comment_test.ams", kBlockCommentScript);

    TEST_ASSERT_FALSE_MESSAGE(ok, "Block-comment syntax must not activate");

    EngineSnapshot snap {};
    fx.engine.getSnapshot(snap);
    TEST_ASSERT_EQUAL_MESSAGE(EngineStatus::ERROR, snap.status,
        "engine status must be ERROR after block-comment parse failure");
    TEST_ASSERT_TRUE_MESSAGE(snap.lastError[0] != '\0',
        "lastError must be set after block-comment parse failure");
}

// ── Test 6: Script containing only line comments ──────────────────────────────

void test_parser_only_comments_no_states_error()
{
    ParserFixture fx;

    static const char kOnlyCommentsScript[] =
        "// AMS script\n"
        "// No directives, no states.\n"
        "// This must be rejected.\n";

    const bool ok = fx.load("only_comments_test.ams", kOnlyCommentsScript);

    TEST_ASSERT_FALSE_MESSAGE(ok, "Comment-only script must not activate");

    EngineSnapshot snap {};
    fx.engine.getSnapshot(snap);
    TEST_ASSERT_EQUAL_MESSAGE(EngineStatus::ERROR, snap.status,
        "engine status must be ERROR for comment-only script");
    TEST_ASSERT_NOT_NULL_MESSAGE(
        strstr(snap.lastError, "no states"),
        "lastError must mention 'no states' for a comment-only script");
}

// ── Test 7: Non-ASCII (UTF-8) bytes in a state identifier ─────────────────────
// AMS-4.1 requires state names to be [A-Za-z0-9_]+.  UTF-8 multi-byte
// sequences contain bytes > 0x7F which fail the identifier validator.
void test_parser_unicode_state_name_accepted()
{
    ParserFixture fx;

    static const char kUnicodeScript[] =
        "pus.apid = 1\n"
        "pus.service 1 as TC\n"
        "pus.service 5 as EVENT\n"
        "\n"
        "state WA\xC3\x89T:\n"
        "  transition to DONE when TC.command == LAUNCH\n"
        "\n"
        "state DONE:\n"
        "  on_enter:\n"
        "    EVENT.info \"OK\"\n";

    const bool ok = fx.load("unicode_name_test.ams", kUnicodeScript);

    TEST_ASSERT_FALSE_MESSAGE(ok,
        "Non-ASCII state name must be rejected (AMS-4.1: only [A-Za-z0-9_] allowed)");

    EngineSnapshot snap {};
    fx.engine.getSnapshot(snap);
    TEST_ASSERT_EQUAL_MESSAGE(EngineStatus::ERROR, snap.status,
        "engine must be in ERROR for script with non-ASCII state name");
    TEST_ASSERT_NOT_NULL_MESSAGE(strstr(snap.lastError, "invalid characters"),
        snap.lastError);
}

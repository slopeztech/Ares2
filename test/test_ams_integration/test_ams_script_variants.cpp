/**
 * @file  test_ams_script_variants.cpp
 * @brief SITL integration tests — AMS language features and script variants.
 *
 * Covers: parser error on invalid script, fallback transitions (AMS-4.9.2),
 * guard conditions with ERROR and on_error recovery (AMS-4.7 / AMS-4.10.2),
 * persistence hold windows (AMS-4.6.1), CONFIRM debounce mode (AMS-4.11.3),
 * and listScripts storage enumeration.
 */

#include <unity.h>

#include "ams/mission_script_engine.h"
#include "ams/ams_driver_registry.h"
#include "hal/storage/storage_interface.h"

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

// ── Shared flight profile ─────────────────────────────────────────────────────
// Ascent from 0 m to 300 m.  BARO.alt > 50 m is reached near t = 2500 ms;
// hold window closes near t = 3500 ms (50 m + 1000 ms).  Guard tests use
// the fact that at t = 1 ms the altitude is still effectively 0 m,
// violating the BARO.alt > 1000 guard in kScriptGuardViolation.

static const ares::sim::FlightProfile kVariantProfile = {
    .count   = 5U,
    .samples = {
        { .timeMs =     0U, .gpsLatDeg = 40.0f, .gpsLonDeg = -3.0f,
          .gpsAltM =   0.0f, .gpsSpeedKmh =  0.0f, .gpsSats = 9U, .gpsFix = true,
          .baroAltM =  0.0f, .baroPressurePa = 101325.0f, .baroTempC = 20.0f,
          .accelX = 0.0f, .accelY = 0.0f, .accelZ =  9.81f, .imuTempC = 25.0f },
        { .timeMs =  2000U, .gpsLatDeg = 40.0f, .gpsLonDeg = -3.0f,
          .gpsAltM =  20.0f, .gpsSpeedKmh = 36.0f, .gpsSats = 9U, .gpsFix = true,
          .baroAltM = 20.0f, .baroPressurePa =  99000.0f, .baroTempC = 19.0f,
          .accelX = 0.5f, .accelY = 0.0f, .accelZ = 14.0f,  .imuTempC = 26.0f },
        { .timeMs =  4000U, .gpsLatDeg = 40.0f, .gpsLonDeg = -3.0f,
          .gpsAltM = 210.0f, .gpsSpeedKmh = 72.0f, .gpsSats = 9U, .gpsFix = true,
          .baroAltM = 210.0f, .baroPressurePa = 98700.0f, .baroTempC = 17.0f,
          .accelX = 0.3f, .accelY = 0.0f, .accelZ = 12.0f,  .imuTempC = 27.0f },
        { .timeMs =  8000U, .gpsLatDeg = 40.0f, .gpsLonDeg = -3.0f,
          .gpsAltM = 300.0f, .gpsSpeedKmh =  5.0f, .gpsSats = 9U, .gpsFix = true,
          .baroAltM = 300.0f, .baroPressurePa = 97700.0f, .baroTempC = 15.0f,
          .accelX = 0.0f, .accelY = 0.0f, .accelZ =  9.81f, .imuTempC = 28.0f },
        { .timeMs = 15000U, .gpsLatDeg = 40.0f, .gpsLonDeg = -3.0f,
          .gpsAltM = 300.0f, .gpsSpeedKmh =  3.0f, .gpsSats = 9U, .gpsFix = true,
          .baroAltM = 300.0f, .baroPressurePa = 97700.0f, .baroTempC = 15.0f,
          .accelX = 0.0f, .accelY = 0.0f, .accelZ =  9.81f, .imuTempC = 28.0f },
    }
};

// ── Fixture ───────────────────────────────────────────────────────────────────

struct ScriptFixture
{
    ares::sim::SimStorageDriver storage;
    ares::sim::SimGpsDriver     gps{kVariantProfile};
    ares::sim::SimBaroDriver    baro{kVariantProfile};
    ares::sim::SimImuDriver     imu{kVariantProfile};
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
// Parser error: a script with no state blocks must cause activate() to fail.
// ─────────────────────────────────────────────────────────────────────────────

void test_parser_error_on_empty_script()
{
    // A syntactically valid AMS header but no state declarations.
    static const char kBadScript[] =
        "include SIM_GPS as GPS\n"
        "pus.apid = 1\n"
        "pus.service 3 as HK\n";

    ScriptFixture f;
    f.init("/missions/bad.ams", kBadScript);

    // activate() must return false — the parser reports "script has no states".
    const bool ok = f.engine.activate("bad.ams");
    TEST_ASSERT_FALSE(ok);

    // After a parse error the engine sets status=ERROR (not IDLE).
    EngineSnapshot snap{};
    f.engine.getSnapshot(snap);
    TEST_ASSERT_EQUAL(EngineStatus::ERROR, snap.status);
}

// ─────────────────────────────────────────────────────────────────────────────
// Fallback: unconditional transition fires after the declared timeout (AMS-4.9.2).
// ─────────────────────────────────────────────────────────────────────────────

void test_fallback_transition_fires_on_timeout()
{
    ScriptFixture f;
    f.init("/missions/fallback.ams", ares::sim::kScriptFallback);
    ares::sim::clock::reset();

    (void)f.engine.activate("fallback.ams");
    (void)f.engine.arm();

    // t=0: enter FLIGHT (via LAUNCH).  No regular transition — only fallback.
    f.engine.tick(ares::sim::clock::nowMs());

    EngineSnapshot snap{};
    f.engine.getSnapshot(snap);
    TEST_ASSERT_EQUAL_STRING("FLIGHT", snap.stateName);

    // t=2001: fallback threshold (2000 ms) has elapsed → engine enters SAFE.
    ares::sim::clock::advanceMs(2001U);
    f.engine.tick(ares::sim::clock::nowMs());

    f.engine.getSnapshot(snap);
    TEST_ASSERT_EQUAL(EngineStatus::RUNNING, snap.status);
    TEST_ASSERT_EQUAL_STRING("SAFE", snap.stateName);
}

// ─────────────────────────────────────────────────────────────────────────────
// Guard violation: EngineStatus::ERROR when a conditions: block is violated.
// ─────────────────────────────────────────────────────────────────────────────

void test_guard_violation_sets_error_status()
{
    // kScriptGuardViolation: FLIGHT conditions: BARO.alt > 1000
    // With kVariantProfile at t=1 ms, alt ≈ 0 m → guard always violated.
    ScriptFixture f;
    f.init("/missions/guard.ams", ares::sim::kScriptGuardViolation);
    ares::sim::clock::reset();

    (void)f.engine.activate("guard.ams");
    (void)f.engine.arm();

    // t=0: LAUNCH TC consumed → enters FLIGHT.
    f.engine.tick(ares::sim::clock::nowMs());

    // t=1: guard check in FLIGHT — BARO.alt ≈ 0 m, fails BARO.alt > 1000.
    ares::sim::clock::advanceMs(1U);
    f.engine.tick(ares::sim::clock::nowMs());

    EngineSnapshot snap{};
    f.engine.getSnapshot(snap);
    TEST_ASSERT_EQUAL(EngineStatus::ERROR, snap.status);
}

// ─────────────────────────────────────────────────────────────────────────────
// on_error recovery: guard violation redirects to SAFE instead of halting.
// ─────────────────────────────────────────────────────────────────────────────

void test_on_error_recovery_transition_replaces_halt()
{
    // kScriptGuardRecovery: same guard + on_error: transition to SAFE.
    ScriptFixture f;
    f.init("/missions/recovery.ams", ares::sim::kScriptGuardRecovery);
    ares::sim::clock::reset();

    (void)f.engine.activate("recovery.ams");
    (void)f.engine.arm();

    // t=0: enter FLIGHT.
    f.engine.tick(ares::sim::clock::nowMs());

    // t=1: guard fails → on_error recovery transition fires → SAFE.
    ares::sim::clock::advanceMs(1U);
    f.engine.tick(ares::sim::clock::nowMs());

    EngineSnapshot snap{};
    f.engine.getSnapshot(snap);
    // Engine is in SAFE, not ERROR; execution continues.
    TEST_ASSERT_EQUAL(EngineStatus::RUNNING, snap.status);
    TEST_ASSERT_EQUAL_STRING("SAFE", snap.stateName);
}

// ─────────────────────────────────────────────────────────────────────────────
// Hold window (AMS-4.6.1): transition does NOT fire before the window elapses.
// ─────────────────────────────────────────────────────────────────────────────

void test_hold_window_prevents_early_fire()
{
    // kScriptHoldWindow: transition to HIGH_ALT when BARO.alt > 50 for 1000 ms.
    ScriptFixture f;
    f.init("/missions/hold.ams", ares::sim::kScriptHoldWindow);
    ares::sim::clock::reset();

    (void)f.engine.activate("hold.ams");
    (void)f.engine.arm();

    // t=0: enter FLIGHT.
    f.engine.tick(ares::sim::clock::nowMs());

    // t=4000: alt = 210 m > 50 m → hold window armed (timer starts).
    ares::sim::clock::advanceMs(4000U);
    f.engine.tick(ares::sim::clock::nowMs());

    // t=4500: 500 ms < 1000 ms hold window → transition must NOT fire.
    ares::sim::clock::advanceMs(500U);
    f.engine.tick(ares::sim::clock::nowMs());

    EngineSnapshot snap{};
    f.engine.getSnapshot(snap);
    TEST_ASSERT_EQUAL_STRING("FLIGHT", snap.stateName);
}

// ─────────────────────────────────────────────────────────────────────────────
// Hold window (AMS-4.6.1): transition fires once the window has fully elapsed.
// ─────────────────────────────────────────────────────────────────────────────

void test_hold_window_fires_after_window()
{
    ScriptFixture f;
    f.init("/missions/hold.ams", ares::sim::kScriptHoldWindow);
    ares::sim::clock::reset();

    (void)f.engine.activate("hold.ams");
    (void)f.engine.arm();

    // t=0: enter FLIGHT.
    f.engine.tick(ares::sim::clock::nowMs());

    // t=4000: hold window armed.
    ares::sim::clock::advanceMs(4000U);
    f.engine.tick(ares::sim::clock::nowMs());

    // t=5001: 1001 ms ≥ 1000 ms → hold window has elapsed → fires.
    ares::sim::clock::advanceMs(1001U);
    f.engine.tick(ares::sim::clock::nowMs());

    EngineSnapshot snap{};
    f.engine.getSnapshot(snap);
    TEST_ASSERT_EQUAL_STRING("HIGH_ALT", snap.stateName);
}

// ─────────────────────────────────────────────────────────────────────────────
// CONFIRM debounce (AMS-4.11.3): transition requires two successive injects.
// ─────────────────────────────────────────────────────────────────────────────

void test_confirm_mode_requires_two_injects()
{
    // kScriptConfirmLaunch: transition to FLIGHT when TC.command == LAUNCH confirm 2.
    // arm() queues LAUNCH internally but does NOT increment tcConfirmCount_.
    ScriptFixture f;
    f.init("/missions/confirm.ams", ares::sim::kScriptConfirmLaunch);
    ares::sim::clock::reset();

    (void)f.engine.activate("confirm.ams");
    (void)f.engine.arm();  // pendingTc_=LAUNCH, tcConfirmCount_[LAUNCH]=0

    // t=0: count=0 < 2 → CONFIRM not met → still in WAIT.
    f.engine.tick(ares::sim::clock::nowMs());

    EngineSnapshot snap{};
    f.engine.getSnapshot(snap);
    TEST_ASSERT_EQUAL_STRING("WAIT", snap.stateName);

    // First inject: count=1, still not enough.
    (void)f.engine.injectTcCommand("LAUNCH");
    ares::sim::clock::advanceMs(1U);
    f.engine.tick(ares::sim::clock::nowMs());

    f.engine.getSnapshot(snap);
    TEST_ASSERT_EQUAL_STRING("WAIT", snap.stateName);

    // Second inject: count=2 ≥ 2 → CONFIRM met → transition fires.
    (void)f.engine.injectTcCommand("LAUNCH");
    ares::sim::clock::advanceMs(1U);
    f.engine.tick(ares::sim::clock::nowMs());

    f.engine.getSnapshot(snap);
    TEST_ASSERT_EQUAL_STRING("FLIGHT", snap.stateName);
}

// ─────────────────────────────────────────────────────────────────────────────
// listScripts: only .ams files from /missions are returned.
// ─────────────────────────────────────────────────────────────────────────────

void test_list_scripts_returns_ams_files()
{
    ScriptFixture f;
    // Register files directly (no engine activation needed for listScripts).
    (void)f.storage.begin();
    f.storage.registerFile("/missions/alpha.ams", "include SIM_GPS as GPS\npus.apid=1\n");
    f.storage.registerFile("/missions/beta.ams",  "include SIM_GPS as GPS\npus.apid=1\n");
    f.storage.registerFile("/missions/notes.txt", "not a script");
    (void)f.engine.begin();

    FileEntry entries[4] = {};
    uint8_t count = 0U;

    const bool ok = f.engine.listScripts(entries, 4U, count);
    TEST_ASSERT_TRUE(ok);
    // Only the two .ams files must be returned; .txt is filtered out.
    TEST_ASSERT_EQUAL(2U, count);
}

// ─────────────────────────────────────────────────────────────────────────────
// Line-too-long: a script with a directive that exceeds AMS_MAX_LINE_LEN must
// cause activate() to fail with an explicit parse error (AMS-8.5).
// Regression for: "Líneas > AMS_MAX_LINE_LEN se truncan silenciosamente" (P0).
// ─────────────────────────────────────────────────────────────────────────────

void test_parser_error_on_line_too_long()
{
    // Build a script whose third line exceeds AMS_MAX_LINE_LEN (128) chars.
    // The var declaration below is 136 characters long (measured statically).
    // readNextScriptLineLocked must detect this and call setErrorLocked before
    // the directive content is ever parsed (AMS-8.5).
    static const char kLongVarLine[] =
        "var aaaa_bbbb_cccc_dddd_eeee_ffff_gggg_hhhh_iiii_jjjj_kkkk_llll_mmmm_nnnn_oooo_pppp_qqqq_rrrr_ssss_tttt_uuuu_vvvv_wwww_xxxx : float";
    // 136 chars: "var "(4) + 124-char name + " : float"(8).
    // Verify the invariant so a future editor notices immediately if they
    // shorten the string.
    static_assert(sizeof(kLongVarLine) - 1U > ares::AMS_MAX_LINE_LEN - 1U,
                  "kLongVarLine must be longer than AMS_MAX_LINE_LEN");

    static const char kLongLineScript[] =
        "include SIM_BARO as BARO\n"
        "pus.apid = 1\n"
        "var aaaa_bbbb_cccc_dddd_eeee_ffff_gggg_hhhh_iiii_jjjj_kkkk_llll_mmmm_nnnn_oooo_pppp_qqqq_rrrr_ssss_tttt_uuuu_vvvv_wwww_xxxx : float\n"
        "state IDLE:\n"
        "  transition to END when TC == LAUNCH\n"
        "state END:\n";

    ScriptFixture f;
    f.init("/missions/longline.ams", kLongLineScript);

    // activate() must return false — the parser detects the oversized line.
    const bool ok = f.engine.activate("longline.ams");
    TEST_ASSERT_FALSE(ok);

    // Engine must be in ERROR, not IDLE.
    EngineSnapshot snap{};
    f.engine.getSnapshot(snap);
    TEST_ASSERT_EQUAL(EngineStatus::ERROR, snap.status);

    // lastError must contain "line too long" — not be empty or generic.
    TEST_ASSERT_TRUE(snap.lastError[0] != '\0');
    TEST_ASSERT_NOT_NULL(strstr(snap.lastError, "line too long"));
}

// ─────────────────────────────────────────────────────────────────────────────
// Metadata-order enforcement (AMS-4.2): directives that must precede any
// state: block are rejected when placed after one.
// ─────────────────────────────────────────────────────────────────────────────

// Helper: verify that a script with the given forbidden line placed after a
// state block causes activate() to fail with an error containing "must appear
// before any state block".
static void checkMetadataAfterStateRejected(const char* badScript, const char* keyword)
{
    ScriptFixture f;
    f.init("/missions/order.ams", badScript);

    const bool ok = f.engine.activate("order.ams");
    TEST_ASSERT_FALSE_MESSAGE(ok, keyword);

    EngineSnapshot snap{};
    f.engine.getSnapshot(snap);
    TEST_ASSERT_EQUAL_MESSAGE(EngineStatus::ERROR, snap.status, keyword);
    TEST_ASSERT_NOT_NULL_MESSAGE(strstr(snap.lastError, "must appear before any state block"), keyword);
}

void test_parser_error_include_after_state()
{
    static const char kScript[] =
        "state INIT:\n"
        "state END:\n"
        "include SIM_BARO as BARO\n";
    checkMetadataAfterStateRejected(kScript, "include");
}

void test_parser_error_var_after_state()
{
    static const char kScript[] =
        "state INIT:\n"
        "state END:\n"
        "var x : float = 1.0\n";
    checkMetadataAfterStateRejected(kScript, "var");
}

void test_parser_error_const_after_state()
{
    static const char kScript[] =
        "state INIT:\n"
        "state END:\n"
        "const THRESH : float = 500.0\n";
    checkMetadataAfterStateRejected(kScript, "const");
}

void test_parser_error_radio_config_after_state()
{
    static const char kScript[] =
        "state INIT:\n"
        "state END:\n"
        "radio.config sf=7 bw=125000 cr=5 pwr=14\n";
    checkMetadataAfterStateRejected(kScript, "radio.config");
}

void test_parser_error_pus_service_after_state()
{
    static const char kScript[] =
        "state INIT:\n"
        "state END:\n"
        "pus.service 3 as HK\n";
    checkMetadataAfterStateRejected(kScript, "pus.service");
}

void test_parser_error_pus_apid_after_state()
{
    static const char kScript[] =
        "state INIT:\n"
        "state END:\n"
        "pus.apid = 1\n";
    checkMetadataAfterStateRejected(kScript, "pus.apid");
}

// ─────────────────────────────────────────────────────────────────────────────
// Levenshtein "did you mean" suggestions (case-sensitive name typos, AMS-4.6).
// ─────────────────────────────────────────────────────────────────────────────

// A transition target that differs only in case (e.g. "flight" vs "FLIGHT")
// must produce a "did you mean 'FLIGHT'?" hint in the error message.
void test_typo_state_name_case_suggests_correction()
{
    static const char kScript[] =
        "state INIT:\n"
        "  transition to flight when TIME.elapsed > 1000\n"
        "state FLIGHT:\n";

    ScriptFixture f;
    f.init("/missions/typo_case.ams", kScript);

    const bool ok = f.engine.activate("typo_case.ams");
    TEST_ASSERT_FALSE(ok);

    EngineSnapshot snap{};
    f.engine.getSnapshot(snap);
    TEST_ASSERT_EQUAL(EngineStatus::ERROR, snap.status);
    TEST_ASSERT_NOT_NULL_MESSAGE(strstr(snap.lastError, "did you mean 'FLIGHT'"),
                                 snap.lastError);
}

// A one-character typo in a transition target (e.g. "FLIGH" vs "FLIGHT")
// must also produce a "did you mean 'FLIGHT'?" hint.
void test_typo_state_name_single_char_suggests_correction()
{
    static const char kScript[] =
        "state INIT:\n"
        "  transition to FLIGH when TIME.elapsed > 1000\n"
        "state FLIGHT:\n";

    ScriptFixture f;
    f.init("/missions/typo_onechar.ams", kScript);

    const bool ok = f.engine.activate("typo_onechar.ams");
    TEST_ASSERT_FALSE(ok);

    EngineSnapshot snap{};
    f.engine.getSnapshot(snap);
    TEST_ASSERT_EQUAL(EngineStatus::ERROR, snap.status);
    TEST_ASSERT_NOT_NULL_MESSAGE(strstr(snap.lastError, "did you mean 'FLIGHT'"),
                                 snap.lastError);
}


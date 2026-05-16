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
#include <cinttypes>

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
// Comment support (AMS-4.0): both // and # lines must be silently skipped
// in metadata, between state blocks, and inside state blocks.
// ─────────────────────────────────────────────────────────────────────────────

void test_comment_slash_slash_skipped()
{
    static const char kScript[] =
        "// Mission: comment test\n"
        "include SIM_BARO as BARO\n"
        "pus.apid = 1\n"
        "// service declarations\n"
        "pus.service 5 as EVENT\n"
        "pus.service 1 as TC\n"
        "// --- state machine ---\n"
        "state INIT:\n"
        "  // wait for launch\n"
        "  transition to END when TC.command == LAUNCH\n"
        "// second state\n"
        "state END:\n"
        "  on_enter:\n"
        "    EVENT.info \"done\"\n";

    ScriptFixture f;
    f.init("/missions/comment_slash.ams", kScript);

    const bool ok = f.engine.activate("comment_slash.ams");
    TEST_ASSERT_TRUE_MESSAGE(ok, "// comments must be silently skipped");

    EngineSnapshot snap{};
    f.engine.getSnapshot(snap);
    TEST_ASSERT_EQUAL(EngineStatus::LOADED, snap.status);
}

void test_comment_hash_skipped()
{
    static const char kScript[] =
        "# Mission: hash comment test\n"
        "include SIM_BARO as BARO\n"
        "pus.apid = 1\n"
        "# service declarations\n"
        "pus.service 5 as EVENT\n"
        "pus.service 1 as TC\n"
        "# --- state machine ---\n"
        "state INIT:\n"
        "  # wait for launch\n"
        "  transition to END when TC.command == LAUNCH\n"
        "# second state\n"
        "state END:\n"
        "  on_enter:\n"
        "    EVENT.info \"done\"\n";

    ScriptFixture f;
    f.init("/missions/comment_hash.ams", kScript);

    const bool ok = f.engine.activate("comment_hash.ams");
    TEST_ASSERT_TRUE_MESSAGE(ok, "# comments must be silently skipped");

    EngineSnapshot snap{};
    f.engine.getSnapshot(snap);
    TEST_ASSERT_EQUAL(EngineStatus::LOADED, snap.status);
}

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
// on_timeout vs fallback tie-breaking (AMS-5.1).
//
// kScriptOnTimeoutVsFallbackTie: HOLD declares both on_timeout 3000ms and
// fallback after 3000ms.  At t=3001ms both are due; tick order guarantees
// on_timeout (step 6) fires before fallback (step 7) → TIMEOUT_WIN.
// ─────────────────────────────────────────────────────────────────────────────

void test_on_timeout_beats_fallback_equal_threshold()
{
    // Both on_timeout and fallback are set to 3000ms.
    // on_timeout is evaluated first (AMS-5.1 step 6 < step 7), so TIMEOUT_WIN
    // must be entered; FALLBACK_WIN must remain unreachable.
    ScriptFixture f;
    f.init("/missions/tie.ams", ares::sim::kScriptOnTimeoutVsFallbackTie);
    (void)f.engine.activate("tie.ams");
    (void)f.engine.arm();  // pendingTc_ = LAUNCH

    // t=0: LAUNCH consumed → enter HOLD
    f.engine.tick(ares::sim::clock::nowMs());

    EngineSnapshot snap{};
    f.engine.getSnapshot(snap);
    TEST_ASSERT_EQUAL_STRING("HOLD", snap.stateName);

    // t=3001ms: both on_timeout and fallback are due; on_timeout wins (step 6)
    ares::sim::clock::advanceMs(3001U);
    f.engine.tick(ares::sim::clock::nowMs());

    f.engine.getSnapshot(snap);
    TEST_ASSERT_EQUAL(EngineStatus::RUNNING, snap.status);
    TEST_ASSERT_EQUAL_STRING("TIMEOUT_WIN", snap.stateName);
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
// Guard violation stores condition detail in lastError_ (visible via API).
//
// kScriptGuardViolation: FLIGHT conditions: BARO.alt > 1000
// After violation, lastError must describe the failing condition
// (e.g. "guard BARO.alt > 1000.000 failed (val=0.000)"), not be generic.
// ─────────────────────────────────────────────────────────────────────────────

void test_guard_violation_lastError_contains_condition()
{
    ScriptFixture f;
    f.init("/missions/guard.ams", ares::sim::kScriptGuardViolation);

    (void)f.engine.activate("guard.ams");
    (void)f.engine.arm();

    f.engine.tick(ares::sim::clock::nowMs());  // t=0: enters FLIGHT

    ares::sim::clock::advanceMs(1U);
    f.engine.tick(ares::sim::clock::nowMs());  // t=1: BARO.alt ≈ 0 < 1000 → violation

    EngineSnapshot snap{};
    f.engine.getSnapshot(snap);
    TEST_ASSERT_EQUAL(EngineStatus::ERROR, snap.status);
    TEST_ASSERT_NOT_NULL_MESSAGE(strstr(snap.lastError, "guard BARO.alt"), snap.lastError);
    TEST_ASSERT_NOT_NULL_MESSAGE(strstr(snap.lastError, "failed"),         snap.lastError);
}

// ─────────────────────────────────────────────────────────────────────────────
// Guard skipped when variable RHS is not yet set (AMS-5.6 fail-safe).
//
// kScriptGuardVarNotSet: FLIGHT conditions: BARO.alt > min_alt
// `min_alt` is declared as `var` but never set by any set action, so
// `resolveVarThresholdLocked` returns false every tick.  AMS-5.6 specifies
// that an unresolvable guard RHS is treated as "holds" (fail-safe), keeping
// the engine RUNNING rather than raising an ERROR.
// ─────────────────────────────────────────────────────────────────────────────
void test_guard_skipped_when_var_not_set()
{
    ScriptFixture f;
    f.init("/missions/guard_var.ams", ares::sim::kScriptGuardVarNotSet);
    (void)f.engine.activate("guard_var.ams");
    (void)f.engine.arm();

    // t=0: LAUNCH TC consumed → enter FLIGHT.
    f.engine.tick(ares::sim::clock::nowMs());

    // t=1: guard evaluated — BARO.alt ≈ 0 m, min_alt.valid == false.
    // resolveVarThresholdLocked returns false → holds stays true → no ERROR.
    ares::sim::clock::advanceMs(1U);
    f.engine.tick(ares::sim::clock::nowMs());

    EngineSnapshot snap{};
    f.engine.getSnapshot(snap);
    TEST_ASSERT_EQUAL(EngineStatus::RUNNING, snap.status);
    TEST_ASSERT_EQUAL_STRING("FLIGHT", snap.stateName);
}

// ─────────────────────────────────────────────────────────────────────────────
// Shadowed transition detection (AMS-5.1).
//
// kScriptShadowedTransition: ASCENT has two sensor transitions where the
// second is unreachable (BARO.alt > 200 fires before BARO.alt > 500).
// warnShadowedTransitionsLocked emits LOG_W + EVENT.info at load time.
// Engine must parse without error; the dominator transition fires correctly.
// ─────────────────────────────────────────────────────────────────────────────

void test_shadowed_transition_parses_ok()
{
    // Shadowed transition must not prevent the script from loading.
    ScriptFixture f;
    f.init("/missions/shadow.ams", ares::sim::kScriptShadowedTransition);
    (void)f.engine.activate("shadow.ams");

    EngineSnapshot snap{};
    f.engine.getSnapshot(snap);
    TEST_ASSERT_EQUAL(EngineStatus::LOADED, snap.status);
    TEST_ASSERT_EQUAL_STRING("", snap.lastError);
}

void test_shadowed_transition_dominator_fires()
{
    // At t=4000ms, alt=210m > 200 → tr[0] (HIGH_ALT) fires.
    // tr[1] (VERY_HIGH: alt > 500) is shadowed and must never be evaluated.
    ScriptFixture f;
    f.init("/missions/shadow.ams", ares::sim::kScriptShadowedTransition);
    (void)f.engine.activate("shadow.ams");
    (void)f.engine.arm();  // pendingTc_ = LAUNCH

    // t=0: LAUNCH TC consumed → enter ASCENT
    f.engine.tick(ares::sim::clock::nowMs());

    // t=4000: alt=210m > 200 → tr[0] (HIGH_ALT) fires
    ares::sim::clock::advanceMs(4000U);
    f.engine.tick(ares::sim::clock::nowMs());

    EngineSnapshot snap{};
    f.engine.getSnapshot(snap);
    TEST_ASSERT_EQUAL(EngineStatus::RUNNING, snap.status);
    TEST_ASSERT_EQUAL_STRING("HIGH_ALT", snap.stateName);
}

// ─────────────────────────────────────────────────────────────────────────────

void test_on_error_recovery_transition_replaces_halt()
{
    // kScriptGuardRecovery: same guard + on_error: transition to SAFE.
    ScriptFixture f;
    f.init("/missions/recovery.ams", ares::sim::kScriptGuardRecovery);

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

// pus.apid = 4 is outside the AMS-9 defined range (0..3) and must be
// rejected at parse time with a clear range error (not at runtime).
void test_parser_error_pus_apid_out_of_range()
{
    static const char kScript[] =
        "pus.apid = 4\n"
        "state INIT:\n";

    ScriptFixture f;
    f.init("/missions/apid_oor.ams", kScript);

    const bool ok = f.engine.activate("apid_oor.ams");
    TEST_ASSERT_FALSE(ok);

    EngineSnapshot snap{};
    f.engine.getSnapshot(snap);
    TEST_ASSERT_EQUAL(EngineStatus::ERROR, snap.status);
    // Error must mention the valid range so the author knows what to fix.
    TEST_ASSERT_NOT_NULL_MESSAGE(strstr(snap.lastError, "0..3"), snap.lastError);
}

// ─────────────────────────────────────────────────────────────────────────────
// State-name character validation (AMS-4.1): identifiers must match
// [A-Za-z0-9_]+.  Spaces, hyphens, or any other non-identifier characters
// must be rejected at parse time with an actionable error.
// ─────────────────────────────────────────────────────────────────────────────

// "MY STATE" — embedded space: sscanf %15[^:] accepts it but the identifier
// validator must reject it so that transition targets are unambiguous.
void test_parser_error_state_name_with_space()
{
    static const char kScript[] =
        "state MY STATE:\n"
        "  transition to END when TIME.elapsed > 1000\n"
        "state END:\n";

    ScriptFixture f;
    f.init("/missions/bad_sname.ams", kScript);

    const bool ok = f.engine.activate("bad_sname.ams");
    TEST_ASSERT_FALSE(ok);

    EngineSnapshot snap{};
    f.engine.getSnapshot(snap);
    TEST_ASSERT_EQUAL(EngineStatus::ERROR, snap.status);
    TEST_ASSERT_NOT_NULL_MESSAGE(strstr(snap.lastError, "invalid characters"),
                                 snap.lastError);
}

// "MY-STATE" — hyphen: also rejected by the identifier validator.
void test_parser_error_state_name_with_hyphen()
{
    static const char kScript[] =
        "state MY-STATE:\n"
        "  transition to END when TIME.elapsed > 1000\n"
        "state END:\n";

    ScriptFixture f;
    f.init("/missions/bad_sname2.ams", kScript);

    const bool ok = f.engine.activate("bad_sname2.ams");
    TEST_ASSERT_FALSE(ok);

    EngineSnapshot snap{};
    f.engine.getSnapshot(snap);
    TEST_ASSERT_EQUAL(EngineStatus::ERROR, snap.status);
    TEST_ASSERT_NOT_NULL_MESSAGE(strstr(snap.lastError, "invalid characters"),
                                 snap.lastError);
}

// ─────────────────────────────────────────────────────────────────────────────
// Duplicate detection (AMS-4.1 / AMS-4.2): state names and priorities blocks
// must be unique per script / per state.
// ─────────────────────────────────────────────────────────────────────────────

// Two state blocks with the same name must be rejected at parse time.
void test_parser_error_duplicate_state_name()
{
    static const char kScript[] =
        "state INIT:\n"
        "  transition to INIT when TIME.elapsed > 1000\n"
        "state INIT:\n";

    ScriptFixture f;
    f.init("/missions/dup_state.ams", kScript);

    const bool ok = f.engine.activate("dup_state.ams");
    TEST_ASSERT_FALSE(ok);

    EngineSnapshot snap{};
    f.engine.getSnapshot(snap);
    TEST_ASSERT_EQUAL(EngineStatus::ERROR, snap.status);
    TEST_ASSERT_NOT_NULL_MESSAGE(strstr(snap.lastError, "duplicate state name"),
                                 snap.lastError);
}

// A priorities directive that appears twice inside the same state block must
// be rejected; the second occurrence would silently overwrite the first.
void test_parser_error_duplicate_priorities()
{
    static const char kScript[] =
        "state INIT:\n"
        "  priorities hk=2 log=1\n"
        "  priorities hk=3 log=2\n"
        "  transition to END when TIME.elapsed > 1000\n"
        "state END:\n";

    ScriptFixture f;
    f.init("/missions/dup_prio.ams", kScript);

    const bool ok = f.engine.activate("dup_prio.ams");
    TEST_ASSERT_FALSE(ok);

    EngineSnapshot snap{};
    f.engine.getSnapshot(snap);
    TEST_ASSERT_EQUAL(EngineStatus::ERROR, snap.status);
    TEST_ASSERT_NOT_NULL_MESSAGE(strstr(snap.lastError, "duplicate priorities"),
                                 snap.lastError);
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

// ─────────────────────────────────────────────────────────────────────────────
// Security: a checkpoint whose fileName contains a path-traversal sequence
// (e.g. "../../etc/passwd") must be silently discarded during begin().
// The engine must remain IDLE — no ERROR, no file read outside /missions/.
// (AMS security hardening — path-traversal via checkpoint)
// ─────────────────────────────────────────────────────────────────────────────
void test_checkpoint_path_traversal_discarded()
{
    // Valid v3 checkpoint record except the fileName field contains a
    // path-traversal sequence that isSafeFileName() must reject.
    static const char kMaliciousCheckpoint[] =
        "3|../../etc/passwd|0|1|1|1|1|0|0|0";

    ScriptFixture f;
    // Initialise peripherals and storage, then register the malicious
    // checkpoint before engine.begin() — which internally calls
    // tryRestoreResumePointLocked().
    (void)f.storage.begin();
    f.storage.registerFile("/missions/.ams_resume.chk", kMaliciousCheckpoint);
    (void)f.gps.begin();
    (void)f.baro.begin();
    (void)f.imu.begin();
    (void)f.radio.begin();
    (void)f.engine.begin();

    // The malicious checkpoint must have been discarded — engine stays IDLE.
    EngineSnapshot snap{};
    f.engine.getSnapshot(snap);
    TEST_ASSERT_EQUAL(EngineStatus::IDLE, snap.status);
}

// ─────────────────────────────────────────────────────────────────────────────
// v4 checkpoint: a record whose CRC32 does not match the content must be
// silently discarded during begin() and the engine must remain IDLE (AMS-8.5).
// ─────────────────────────────────────────────────────────────────────────────

// Minimal CRC32 IEEE 802.3 (reflected) — mirrors crc32Compute() in the engine.
static uint32_t testCrc32Compute(const char* data, size_t len)
{
    uint32_t crc = 0xFFFFFFFFU;
    for (size_t i = 0U; i < len; i++)
    {
        crc ^= static_cast<uint8_t>(data[i]);
        for (uint8_t b = 0U; b < 8U; b++)
        {
            crc = (crc >> 1U) ^ ((crc & 1U) ? 0xEDB88320U : 0U);
        }
    }
    return crc ^ 0xFFFFFFFFU;
}

void test_checkpoint_crc_corruption_discarded()
{
    // A syntactically correct v4 record with a deliberately wrong CRC (DEADBEEF).
    // Format: {ver}|{file}|{stateIdx}|{exec}|{running}|{status}|{seq}|
    //         {stElap}|{hkElap}|{logElap}|{varCount}|
    //         {hkCount}|{logCount}|
    //         {cc0}|{cc1}|{cc2}|{cc3}|
    //         {hold0}|{condElap0}|{hold1}|{condElap1}|{hold2}|{condElap2}|{hold3}|{condElap3}|
    //         {CRC_HEX}
    static const char kBadCrcCheckpoint[] =
        "4|test.ams|0|1|1|1|1|0|0|0|0|0|0|0|0|0|0|0|0|0|0|0|0|0|0|DEADBEEF";

    ScriptFixture f;
    (void)f.storage.begin();
    f.storage.registerFile("/missions/.ams_resume.chk", kBadCrcCheckpoint);
    f.storage.registerFile("/missions/test.ams",
        "state INIT:\n  transition to END when TC == LAUNCH\nstate END:\n");
    (void)f.gps.begin();
    (void)f.baro.begin();
    (void)f.imu.begin();
    (void)f.radio.begin();
    (void)f.engine.begin();

    // CRC mismatch — checkpoint must be discarded, engine remains IDLE.
    EngineSnapshot snap{};
    f.engine.getSnapshot(snap);
    TEST_ASSERT_EQUAL(EngineStatus::IDLE, snap.status);
}

// ─────────────────────────────────────────────────────────────────────────────
// Truncated resume-state record: v1 with only 5 of the required 10 header
// fields.  parseCheckpointHeaderLocked expects exactly 10 parsed tokens; with
// fewer the return value is false and the engine must discard the record and
// stay IDLE without reading garbage from uninitialised sscanf output variables
// (AMS-8.5 / [P1] sscanf safety audit).
// ─────────────────────────────────────────────────────────────────────────────
void test_checkpoint_truncated_record_discarded()
{
    // v1 skips the CRC check; with only 5 tokens the header parse returns
    // parsed=5 ≠ 10 and tryRestoreResumePointLocked calls clearResumePointLocked.
    static const char kTruncated[] = "1|trunc.ams|0|1|1";

    ScriptFixture f;
    (void)f.storage.begin();
    f.storage.registerFile("/missions/.ams_resume.chk", kTruncated);
    f.storage.registerFile("/missions/trunc.ams",
        "state INIT:\n  transition to END when TIME.elapsed > 1000\nstate END:\n");
    (void)f.gps.begin();
    (void)f.baro.begin();
    (void)f.imu.begin();
    (void)f.radio.begin();
    (void)f.engine.begin();

    // Structurally malformed — must be discarded, engine stays IDLE.
    EngineSnapshot snap{};
    f.engine.getSnapshot(snap);
    TEST_ASSERT_EQUAL(EngineStatus::IDLE, snap.status);
}

// ─────────────────────────────────────────────────────────────────────────────
// v4 checkpoint: after restore, tcConfirmCount_ is repopulated so that a
// mission needing CONFIRM 3 proceeds after a single extra inject (AMS-8.5).
// ─────────────────────────────────────────────────────────────────────────────
void test_checkpoint_v4_confirm_restored()
{
    // Mission: transition to DONE when LAUNCH is confirmed 3 times total.
    // We build a v4 checkpoint with tcConfirmCount_[1]=2 (LAUNCH TC index=1).
    // After restore the engine only needs one more inject to satisfy confirm-3.
    //
    // Checkpoint format (v4):
    //   {ver}|{file}|{stateIdx}|{exec}|{running}|{status}|{seq}|
    //   {stElap}|{hkElap}|{logElap}|{varCount}|
    //   {hkCount}|{logCount}|
    //   {cc0}|{cc1}|{cc2}|{cc3}|
    //   {hold0}|{condElap0}|{hold1}|{condElap1}|{hold2}|{condElap2}|{hold3}|{condElap3}|
    //   {CRC}
    //
    // Field breakdown:
    //   ver=4, file=confirm3.ams, stateIdx=0 (WAIT), exec=1, running=1,
    //   status=1 (RUNNING), seq=0, stElap=0, hkElap=0, logElap=0,
    //   varCount=0, hkCount=0, logCount=0,
    //   cc0=0, cc1=2, cc2=0, cc3=0  (LAUNCH=TC index 1, already confirmed 2x)
    //   hold0=0|condElap0=0  (not holding)
    //   hold1=0|condElap1=0
    //   hold2=0|condElap2=0
    //   hold3=0|condElap3=0
    static const char kBodyNoCrc[] =
        "4|confirm3.ams|0|1|1|1|0|0|0|0|0|0|0|0|2|0|0|0|0|0|0|0|0|0|0";
    // Compute correct CRC over the body.
    const uint32_t crc = testCrc32Compute(kBodyNoCrc,
                                          sizeof(kBodyNoCrc) - 1U);  // exclude NUL
    char kCheckpoint[96] = {};
    (void)snprintf(kCheckpoint, sizeof(kCheckpoint),
                   "%s|%08" PRIX32, kBodyNoCrc, crc);

    // Mission: needs 3 total LAUNCH CONFIRMs.
    static const char kMission[] =
        "include SIM_GPS as GPS\n"
        "include SIM_BARO as BARO\n"
        "include SIM_COM as COM\n"
        "include SIM_IMU as IMU\n"
        "pus.apid = 1\n"
        "pus.service 1 as TC\n"
        "state WAIT:\n"
        "  transition to DONE when TC.command == LAUNCH confirm 3\n"
        "state DONE:\n";

    ScriptFixture f;
    (void)f.storage.begin();
    f.storage.registerFile("/missions/.ams_resume.chk", kCheckpoint);
    f.storage.registerFile("/missions/confirm3.ams", kMission);
    (void)f.gps.begin();
    (void)f.baro.begin();
    (void)f.imu.begin();
    (void)f.radio.begin();
    // begin() restores the checkpoint — engine becomes RUNNING in WAIT.
    (void)f.engine.begin();

    EngineSnapshot snap{};
    f.engine.getSnapshot(snap);
    TEST_ASSERT_EQUAL(EngineStatus::RUNNING, snap.status);
    TEST_ASSERT_EQUAL_STRING("WAIT", snap.stateName);

    // One more LAUNCH inject: count goes 2→3 ≥ 3 → transition fires.
    (void)f.engine.injectTcCommand("LAUNCH");
    ares::sim::clock::advanceMs(1U);
    f.engine.tick(ares::sim::clock::nowMs());
    // Terminal state DONE has no transitions; engine becomes COMPLETE on next tick.
    ares::sim::clock::advanceMs(1U);
    f.engine.tick(ares::sim::clock::nowMs());

    f.engine.getSnapshot(snap);
    // DONE is a terminal state (no outbound transitions) → engine becomes COMPLETE.
    // After COMPLETE, running_=false so stateName returns "IDLE" in getSnapshot.
    TEST_ASSERT_EQUAL(EngineStatus::COMPLETE, snap.status);
}

// ─────────────────────────────────────────────────────────────────────────────
// millis() rollover in checkpoint restore: stateElapsed > nowMs causes
// uint64_t underflow in stateEnterMs_ = nowMs - stateElapsed, which is
// well-defined for unsigned arithmetic and self-corrects when elapsed time
// is recomputed in tick() as nowMs - stateEnterMs_ (AMS-8.3 / millis64).
//
// Scenario: clock = 50 ms, checkpoint stateElapsed = 200 ms.
//   stateEnterMs_ = 50 - 200 = UINT64_MAX - 149  (uint64_t underflow).
//   tick(50):  elapsed = 50 - stateEnterMs_ = 200 ms  (self-healing wrap).
//   on_timeout 100 ms fires immediately (200 >= 100); engine reaches COMPLETE.
// ─────────────────────────────────────────────────────────────────────────────
void test_checkpoint_millis_rollover_elapsed_wrap()
{
    // Mission: WAIT → DONE on on_timeout 100 ms.  No sensors, no TC.
    static const char kMission[] =
        "pus.apid = 1\n"
        "state WAIT:\n"
        "  on_timeout 100ms:\n"
        "    transition to DONE\n"
        "state DONE:\n";

    // v4 checkpoint: stateElapsed = 200 (larger than nowMs = 50 → underflow).
    // Field order: ver|file|stateIdx|exec|running|status|seq|
    //              stElap|hkElap|logElap|varCount|hkCount|logCount|
    //              cc0|cc1|cc2|cc3|hold0|cElap0|hold1|cElap1|hold2|cElap2|hold3|cElap3
    static const char kBodyNoCrc[] =
        "4|timeout_wrap.ams|0|1|1|1|0|200|0|0|0|0|0|0|0|0|0|0|0|0|0|0|0|0|0";
    const uint32_t crc = testCrc32Compute(kBodyNoCrc, sizeof(kBodyNoCrc) - 1U);
    char kCheckpoint[128] = {};
    (void)snprintf(kCheckpoint, sizeof(kCheckpoint),
                   "%s|%08" PRIX32, kBodyNoCrc, crc);

    ScriptFixture f;
    ares::sim::clock::advanceMs(50U);  // nowMs = 50

    (void)f.storage.begin();
    f.storage.registerFile("/missions/.ams_resume.chk", kCheckpoint);
    f.storage.registerFile("/missions/timeout_wrap.ams", kMission);
    (void)f.gps.begin();
    (void)f.baro.begin();
    (void)f.imu.begin();
    (void)f.radio.begin();
    // begin() at nowMs=50 restores checkpoint; stateElapsed=200 > nowMs=50
    // → stateEnterMs_ = 50 - 200 = UINT64_MAX - 149  (uint64_t underflow).
    (void)f.engine.begin();

    EngineSnapshot snap{};
    f.engine.getSnapshot(snap);
    TEST_ASSERT_EQUAL(EngineStatus::RUNNING, snap.status);
    TEST_ASSERT_EQUAL_STRING("WAIT", snap.stateName);

    // tick(50): elapsed = 50 - stateEnterMs_ wraps back to 200 ms (correct).
    // on_timeout 100 ms fires (200 >= 100) → transitions to DONE.
    f.engine.tick(static_cast<uint64_t>(ares::sim::clock::nowMs()));

    // tick(51): DONE has no transitions/HK/LOG → engine reaches COMPLETE.
    ares::sim::clock::advanceMs(1U);
    f.engine.tick(static_cast<uint64_t>(ares::sim::clock::nowMs()));

    f.engine.getSnapshot(snap);
    TEST_ASSERT_EQUAL(EngineStatus::COMPLETE, snap.status);
}

// ─────────────────────────────────────────────────────────────────────────────
// ABORT during CONFIRM-N in progress (AMS-4.11.3 + AMS-4.13):
//   inject 2 of 3 required LAUNCH commands, then inject ABORT.
//   The engine must force-deactivate (IDLE) — FLIGHT must never be entered.
// ─────────────────────────────────────────────────────────────────────────────
void test_abort_during_confirm_n_in_progress()
{
    // kScriptConfirm3Launch: WAIT → FLIGHT requires 3 consecutive LAUNCH injects.
    // No explicit ABORT transition → ABORT TC causes force-deactivation.
    ScriptFixture f;
    f.init("/missions/confirm3.ams", ares::sim::kScriptConfirm3Launch);

    (void)f.engine.activate("confirm3.ams");
    (void)f.engine.arm();  // pendingTc_=LAUNCH, tcConfirmCount_[LAUNCH]=0

    // t=0: count=0 < 3 → still WAIT.
    f.engine.tick(ares::sim::clock::nowMs());

    EngineSnapshot snap{};
    f.engine.getSnapshot(snap);
    TEST_ASSERT_EQUAL_STRING("WAIT", snap.stateName);

    // First inject: count becomes 1. Still not enough (1 < 3).
    (void)f.engine.injectTcCommand("LAUNCH");
    ares::sim::clock::advanceMs(1U);
    f.engine.tick(ares::sim::clock::nowMs());

    f.engine.getSnapshot(snap);
    TEST_ASSERT_EQUAL_STRING("WAIT", snap.stateName);

    // Second inject: count becomes 2. Still not enough (2 < 3).
    (void)f.engine.injectTcCommand("LAUNCH");
    ares::sim::clock::advanceMs(1U);
    f.engine.tick(ares::sim::clock::nowMs());

    f.engine.getSnapshot(snap);
    TEST_ASSERT_EQUAL_STRING("WAIT", snap.stateName);

    // Inject ABORT — no explicit ABORT transition → force-deactivates engine.
    (void)f.engine.injectTcCommand("ABORT");
    ares::sim::clock::advanceMs(1U);
    f.engine.tick(ares::sim::clock::nowMs());

    f.engine.getSnapshot(snap);
    // Engine must be IDLE (force-deactivated), never FLIGHT, never ERROR.
    TEST_ASSERT_EQUAL(EngineStatus::IDLE, snap.status);
    // When not running, getSnapshot() returns "IDLE" as the state name.
    TEST_ASSERT_EQUAL_STRING("IDLE", snap.stateName);
}

// ─────────────────────────────────────────────────────────────────────────────
// Resume with deleted script (AMS-8.5):
//   A checkpoint pointing to a .ams file that no longer exists must be
//   silently discarded during begin() — engine remains IDLE, no crash.
// ─────────────────────────────────────────────────────────────────────────────
void test_checkpoint_resume_missing_file()
{
    // Build a valid v4 checkpoint pointing to "missing.ams" (which is not
    // registered in SimStorageDriver).  loadFromStorageLocked will fail and
    // tryRestoreResumePointLocked must call clearResumePointLocked + return.
    static const char kBodyNoCrc[] =
        "4|missing.ams|0|1|1|1|0|0|0|0|0|0|0|0|0|0|0|0|0|0|0|0|0|0|0";
    const uint32_t crc = testCrc32Compute(kBodyNoCrc, sizeof(kBodyNoCrc) - 1U);
    char kCheckpoint[96] = {};
    (void)snprintf(kCheckpoint, sizeof(kCheckpoint),
                   "%s|%08" PRIX32, kBodyNoCrc, crc);

    ScriptFixture f;
    (void)f.storage.begin();
    // Register the checkpoint but NOT the .ams file it references.
    f.storage.registerFile("/missions/.ams_resume.chk", kCheckpoint);
    (void)f.gps.begin();
    (void)f.baro.begin();
    (void)f.imu.begin();
    (void)f.radio.begin();
    (void)f.engine.begin();

    EngineSnapshot snap{};
    f.engine.getSnapshot(snap);
    // The checkpoint CRC is valid and the filename is safe, so the engine
    // attempts loadFromStorageLocked("missing.ams").  That call sets
    // status=ERROR ("script file not found") and returns false, after which
    // tryRestoreResumePointLocked clears the checkpoint and returns.
    // Engine ends in ERROR — not crashed, not RUNNING.
    TEST_ASSERT_EQUAL(EngineStatus::ERROR, snap.status);
}

// ─────────────────────────────────────────────────────────────────────────────
// Negative threshold — `IMU.accel_z < -9.81` (AMS-4.6.2 sign invariant):
//   parseConditionRhsThresholdLocked calls parseFloatValue which uses strtof,
//   so negative RHS literals are parsed without any sign-normalisation step.
//   This test verifies end-to-end that the transition fires when accelZ drops
//   below the negative threshold.
// ─────────────────────────────────────────────────────────────────────────────
void test_negative_threshold_accel_z_transition_fires()
{
    // t=0: accelZ = 9.81 m/s² (normal gravity, well above −9.81).
    // t=2000: accelZ = −20.0 m/s² (simulated impact shock).
    static const ares::sim::FlightProfile kImpactProfile = {
        .count = 2U,
        .samples = {
            { .timeMs =    0U, .accelZ =   9.81f },
            { .timeMs = 2000U, .accelZ = -20.0f  },
        }
    };
    static const char kScript[] =
        "include SIM_IMU as IMU\n"
        "pus.apid = 1\n"
        "pus.service 1 as TC\n"
        "state WAIT:\n"
        "  transition to FLIGHT when TC.command == LAUNCH\n"
        "state FLIGHT:\n"
        "  transition to IMPACT when IMU.accel_z < -9.81\n"
        "state IMPACT:\n";

    ares::sim::SimStorageDriver storage;
    ares::sim::SimImuDriver     imu{kImpactProfile};
    ImuEntry imuEntry = { "SIM_IMU", &imu };
    MissionScriptEngine engine{
        storage, nullptr, 0U, nullptr, 0U, nullptr, 0U, &imuEntry, 1U
    };

    (void)storage.begin();
    storage.registerFile("/missions/neg_thr.ams", kScript);
    (void)imu.begin();
    (void)engine.begin();

    const bool ok = engine.activate("neg_thr.ams");
    TEST_ASSERT_TRUE_MESSAGE(ok, "script with negative threshold must parse ok");
    (void)engine.arm();

    // tick(0): LAUNCH TC fires → enters FLIGHT; no condition eval this tick.
    engine.tick(ares::sim::clock::nowMs());
    EngineSnapshot snap{};
    engine.getSnapshot(snap);
    TEST_ASSERT_EQUAL_STRING("FLIGHT", snap.stateName);

    // tick(2001): accelZ ≈ −20.0 m/s² < −9.81 → transition to IMPACT fires.
    ares::sim::clock::advanceMs(2001U);
    engine.tick(ares::sim::clock::nowMs());
    engine.getSnapshot(snap);
    TEST_ASSERT_EQUAL(EngineStatus::RUNNING, snap.status);
    TEST_ASSERT_EQUAL_STRING("IMPACT", snap.stateName);
}

// ─────────────────────────────────────────────────────────────────────────────
// Negative delta threshold — `IMU.accel_z delta < -1.0` (AMS-4.6.2):
//   On the first evaluation in a state transitionPrevValid_ is false, so the
//   delta is not yet computable and the condition does not fire (baseline is
//   primed instead).  On the second evaluation the actual inter-tick drop is
//   measured; if it is more negative than −1.0 the transition fires.
// ─────────────────────────────────────────────────────────────────────────────
void test_negative_delta_threshold_accel_z_transition_fires()
{
    // accelZ falls from 9.81 to 0.0 over 5000 ms.
    // At tick(10ms) the IMU cache has expired (TTL = AMS_SENSOR_CACHE_TTL_MS = 5ms)
    // so the first real read gives accelZ ≈ 9.79; baseline is primed, no fire.
    // At tick(2510ms) the cache has expired again; accelZ ≈ 4.88;
    // delta ≈ 4.88 − 9.79 = −4.91 < −1.0 → transition fires.
    static const ares::sim::FlightProfile kDropProfile = {
        .count = 2U,
        .samples = {
            { .timeMs =    0U, .accelZ = 9.81f },
            { .timeMs = 5000U, .accelZ = 0.0f  },
        }
    };
    static const char kScript[] =
        "include SIM_IMU as IMU\n"
        "pus.apid = 1\n"
        "pus.service 1 as TC\n"
        "state WAIT:\n"
        "  transition to FLIGHT when TC.command == LAUNCH\n"
        "state FLIGHT:\n"
        "  transition to DESCEND when IMU.accel_z delta < -1.0\n"
        "state DESCEND:\n";

    ares::sim::SimStorageDriver storage;
    ares::sim::SimImuDriver     imu{kDropProfile};
    ImuEntry imuEntry = { "SIM_IMU", &imu };
    MissionScriptEngine engine{
        storage, nullptr, 0U, nullptr, 0U, nullptr, 0U, &imuEntry, 1U
    };

    (void)storage.begin();
    storage.registerFile("/missions/neg_delta.ams", kScript);
    (void)imu.begin();
    (void)engine.begin();

    const bool ok = engine.activate("neg_delta.ams");
    TEST_ASSERT_TRUE_MESSAGE(ok, "script with negative delta threshold must parse ok");
    (void)engine.arm();

    // tick(0): LAUNCH TC fires → enters FLIGHT; transitionPrevValid_ reset to false.
    engine.tick(ares::sim::clock::nowMs());
    EngineSnapshot snap{};
    engine.getSnapshot(snap);
    TEST_ASSERT_EQUAL_STRING("FLIGHT", snap.stateName);

    // tick(10ms): IMU cache TTL (5ms) has expired → fresh read gives accelZ ≈ 9.79 m/s²;
    // transitionPrevValid_=false so no delta is computed yet; baseline is primed.
    ares::sim::clock::advanceMs(10U);
    engine.tick(ares::sim::clock::nowMs());
    engine.getSnapshot(snap);
    TEST_ASSERT_EQUAL_STRING("FLIGHT", snap.stateName);

    // tick(2510ms): cache expires again → accelZ ≈ 4.88 m/s²;
    // delta ≈ 4.88 − 9.79 = −4.91 < −1.0 → transition fires.
    ares::sim::clock::advanceMs(2500U);
    engine.tick(ares::sim::clock::nowMs());
    engine.getSnapshot(snap);
    TEST_ASSERT_EQUAL(EngineStatus::RUNNING, snap.status);
    TEST_ASSERT_EQUAL_STRING("DESCEND", snap.stateName);
}

// ─────────────────────────────────────────────────────────────────────────────
// IMU.gyro_mag — angular rate magnitude √(gx²+gy²+gz²) (AMS-4.5):
//   Verifies that `gyro_mag` parses correctly and that a `> threshold`
//   transition fires when the gyroscope vector magnitude exceeds the limit.
//   Useful for tumbling detection: a spinning rocket will have large gyro_mag
//   even if individual axes alternate sign.
// ─────────────────────────────────────────────────────────────────────────────
void test_gyro_mag_transition_fires_on_tumbling()
{
    // gyroX ramps from 0 to 200 deg/s over 1000 ms (gyroY/Z stay at 0).
    // gyro_mag at t=10ms  ≈ 2.0 deg/s  — below threshold 100.
    // gyro_mag at t=1010ms ≈ 200 deg/s — above threshold 100 → transition fires.
    static const ares::sim::FlightProfile kTumbleProfile = {
        .count = 2U,
        .samples = {
            { .timeMs =    0U, .gyroX =   0.0f },
            { .timeMs = 1000U, .gyroX = 200.0f },
        }
    };
    static const char kScript[] =
        "include SIM_IMU as IMU\n"
        "pus.apid = 1\n"
        "pus.service 1 as TC\n"
        "state WAIT:\n"
        "  transition to FLIGHT when TC.command == LAUNCH\n"
        "state FLIGHT:\n"
        "  transition to TUMBLING when IMU.gyro_mag > 100.0\n"
        "state TUMBLING:\n";

    ares::sim::SimStorageDriver storage;
    ares::sim::SimImuDriver     imu{kTumbleProfile};
    ImuEntry imuEntry = { "SIM_IMU", &imu };
    MissionScriptEngine engine{
        storage, nullptr, 0U, nullptr, 0U, nullptr, 0U, &imuEntry, 1U
    };

    (void)storage.begin();
    storage.registerFile("/missions/gyro_mag.ams", kScript);
    (void)imu.begin();
    (void)engine.begin();

    const bool ok = engine.activate("gyro_mag.ams");
    TEST_ASSERT_TRUE_MESSAGE(ok, "script with gyro_mag condition must parse ok");
    (void)engine.arm();

    // tick(0): LAUNCH TC fires → enters FLIGHT.
    engine.tick(ares::sim::clock::nowMs());
    EngineSnapshot snap{};
    engine.getSnapshot(snap);
    TEST_ASSERT_EQUAL_STRING("FLIGHT", snap.stateName);

    // tick(10ms): IMU cache TTL (5ms) has expired → gyro_mag ≈ 2.0 < 100.0; stays FLIGHT.
    ares::sim::clock::advanceMs(10U);
    engine.tick(ares::sim::clock::nowMs());
    engine.getSnapshot(snap);
    TEST_ASSERT_EQUAL_STRING("FLIGHT", snap.stateName);

    // tick(1010ms): cache expires again → gyroX ≈ 200; gyro_mag ≈ 200 > 100.0 → TUMBLING.
    ares::sim::clock::advanceMs(1000U);
    engine.tick(ares::sim::clock::nowMs());
    engine.getSnapshot(snap);
    TEST_ASSERT_EQUAL(EngineStatus::RUNNING, snap.status);
    TEST_ASSERT_EQUAL_STRING("TUMBLING", snap.stateName);
}

// ── test_var_field_in_log_report ──────────────────────────────────────────────
/**
 * Verify AMS-4.8: a declared AMS variable can be used as a field in
 * LOG.report.  The variable is set via a `set` action in `on_enter:`,
 * then logged in the next state.
 *
 * Flow:
 *   1. Activate + arm kScriptVarFieldInLog.
 *   2. Tick at t=0 → WAIT entered; on_enter sets ground_alt = BARO.alt (≈0).
 *   3. Arm fires LAUNCH TC → engine transitions to FLIGHT (same tick or next).
 *   4. Tick at t=10ms → LOG slot due; LOG.report writes CSV row with ga value.
 *   5. Assert script parses without error (engine status RUNNING).
 *   6. Assert log CSV output contains a column named "ga".
 *   7. Assert CSV data row (not "nan") — variable was valid at log time.
 */
void test_var_field_in_log_report()
{
    ScriptFixture f;
    f.init("/missions/var_log.ams", ares::sim::kScriptVarFieldInLog);
    TEST_ASSERT_TRUE_MESSAGE(f.engine.activate("var_log.ams"),
                             "kScriptVarFieldInLog must parse without error");
    TEST_ASSERT_TRUE(f.engine.arm());

    // tick(0): LAUNCH fires → WAIT on_enter sets ground_alt, transitions to FLIGHT.
    f.engine.tick(ares::sim::clock::nowMs());

    EngineSnapshot snap{};
    f.engine.getSnapshot(snap);
    TEST_ASSERT_EQUAL(EngineStatus::RUNNING, snap.status);
    TEST_ASSERT_EQUAL_STRING("FLIGHT", snap.stateName);

    // tick(10ms): LOG slot due → CSV header + first data row written.
    ares::sim::clock::advanceMs(10U);
    f.engine.tick(ares::sim::clock::nowMs());

    const char* content = f.storage.appendedContent();
    TEST_ASSERT_NOT_NULL_MESSAGE(content, "storage must have log content");

    // CSV header must contain the "ga" column label.
    TEST_ASSERT_NOT_NULL_MESSAGE(strstr(content, "ga"),
                                 "LOG CSV header must contain column 'ga'");

    // Data row must not be "nan" — the variable was set before logging.
    TEST_ASSERT_NULL_MESSAGE(strstr(content, ",nan,"),
                             "LOG CSV row must not contain 'nan' for ground_alt");
    TEST_ASSERT_NULL_MESSAGE(strstr(content, ",nan\n"),
                             "LOG CSV row must not end with ',nan'");
}


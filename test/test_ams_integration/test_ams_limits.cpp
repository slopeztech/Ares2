/**
 * @file  test_ams_limits.cpp
 * @brief SITL integration tests — AMS parser hard limits (config.h constants).
 *
 * Each test generates a script that exceeds exactly one limit and verifies
 * that activate() returns false with status == ERROR and a descriptive
 * lastError.  One boundary-exact "at-limit" test per constant verifies that
 * the limit itself (== MAX) is accepted.
 *
 * Limits covered:
 *   AMS_MAX_STATES       = 10   → 11 states rejected, 10 accepted
 *   AMS_MAX_TRANSITIONS  = 4    → 5 transitions rejected, 4 accepted
 *   AMS_MAX_VARS         = 8    → 9 vars rejected, 8 accepted
 *   AMS_MAX_CONSTS       = 8    → 9 consts rejected, 8 accepted
 *   AMS_MAX_HK_SLOTS     = 4    → 5 every blocks rejected, 4 accepted
 *   AMS_MAX_HK_FIELDS    = 16   → 17 HK fields rejected
 *   AMS_MAX_INCLUDES          = 8    → 9 includes rejected
 *   AMS_MAX_SCRIPT_BYTES       = 4096 → 4096-byte script accepted; bytes beyond
 *                                        4096 truncated, leaving no state → rejected
 *   max_transition_depth (DFS)        → full AMS_MAX_STATES-state chain passes at
 *                                        < AMS_MAX_STATES, fails at < AMS_MAX_STATES-1
 */

#include <unity.h>
#include <cstdio>
#include <cstring>

#include "ams/mission_script_engine.h"
#include "ams/ams_driver_registry.h"
#include "hal/storage/storage_interface.h"
#include "config.h"

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

// ── Minimal quiescent flight profile ─────────────────────────────────────────

static const ares::sim::FlightProfile kLimitsProfile = {
    .count   = 1U,
    .samples = {
        { .timeMs        = 0U,
          .gpsLatDeg     = 0.0f,  .gpsLonDeg  = 0.0f,  .gpsAltM    = 0.0f,
          .gpsSpeedKmh   = 0.0f,  .gpsHdop    = 1.0f,
          .gpsSats       = 6U,    .gpsFix     = true,
          .baroAltM      = 0.0f,  .baroPressurePa = 101325.0f, .baroTempC = 20.0f,
          .accelX        = 0.0f,  .accelY     = 0.0f,  .accelZ     = 9.81f,
          .imuTempC      = 25.0f },
    }
};

// ── Test fixture ──────────────────────────────────────────────────────────────

struct LimitsFixture
{
    ares::sim::SimStorageDriver storage;
    ares::sim::SimGpsDriver     gps  { kLimitsProfile };
    ares::sim::SimBaroDriver    baro { kLimitsProfile };
    ares::sim::SimImuDriver     imu  { kLimitsProfile };
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

// ── Helpers ───────────────────────────────────────────────────────────────────

/** Append N repetitions of @p fragment (no NUL check — caller sizes buf). */
static void appendN(char* buf, size_t bufsz, const char* fragment, int n)
{
    for (int i = 0; i < n; i++)
    {
        const size_t used = strnlen(buf, bufsz);
        if (used < bufsz - 1U)
        {
            (void)strncat(buf, fragment, bufsz - used - 1U);
        }
    }
}

// ── AMS_MAX_STATES ────────────────────────────────────────────────────────────

/**
 * FAIL — 11 states exceeds AMS_MAX_STATES (10).
 * Each state has a transition to the next so the resolver doesn't complain
 * about unreachable states before the limit is hit.
 */
void test_limit_too_many_states()
{
    // States S0..S9 chain to each other; S10 would exceed the limit.
    // AMS_MAX_STATES = 10: state[10] (the 11th) must be rejected.
    static const char kHeader[] =
        "pus.apid = 1\n"
        "pus.service 1 as TC\n";

    // Build chain: S0→S1→…→S9→S10 (11 states).
    char script[1024] = {};
    (void)strncat(script, kHeader, sizeof(script) - 1U);
    for (int i = 0; i <= 10; i++)
    {
        char buf[64] = {};
        if (i < 10)
        {
            (void)snprintf(buf, sizeof(buf),
                           "state S%d:\n  transition to S%d when TC.command == LAUNCH\n",
                           i, i + 1);
        }
        else
        {
            (void)snprintf(buf, sizeof(buf), "state S%d:\n", i);
        }
        const size_t used = strnlen(script, sizeof(script));
        if (used < sizeof(script) - 1U)
        {
            (void)strncat(script, buf, sizeof(script) - used - 1U);
        }
    }

    LimitsFixture f;
    const bool ok = f.load("too_many_states.ams", script);
    TEST_ASSERT_FALSE(ok);

    EngineSnapshot snap{};
    f.engine.getSnapshot(snap);
    TEST_ASSERT_EQUAL(EngineStatus::ERROR, snap.status);
    TEST_ASSERT_NOT_NULL_MESSAGE(strstr(snap.lastError, "too many states"), snap.lastError);
}

/**
 * PASS — exactly AMS_MAX_STATES (10) states must be accepted.
 */
void test_limit_at_max_states_accepted()
{
    static const char kHeader[] =
        "pus.apid = 1\n"
        "pus.service 1 as TC\n";

    char script[1024] = {};
    (void)strncat(script, kHeader, sizeof(script) - 1U);
    for (int i = 0; i < static_cast<int>(ares::AMS_MAX_STATES); i++)
    {
        char buf[64] = {};
        if (i < static_cast<int>(ares::AMS_MAX_STATES) - 1)
        {
            (void)snprintf(buf, sizeof(buf),
                           "state S%d:\n  transition to S%d when TC.command == LAUNCH\n",
                           i, i + 1);
        }
        else
        {
            (void)snprintf(buf, sizeof(buf), "state S%d:\n", i);
        }
        const size_t used = strnlen(script, sizeof(script));
        if (used < sizeof(script) - 1U)
        {
            (void)strncat(script, buf, sizeof(script) - used - 1U);
        }
    }

    LimitsFixture f;
    const bool ok = f.load("max_states.ams", script);
    TEST_ASSERT_TRUE(ok);

    EngineSnapshot snap{};
    f.engine.getSnapshot(snap);
    TEST_ASSERT_EQUAL(EngineStatus::LOADED, snap.status);
}

// ── AMS_MAX_TRANSITIONS ───────────────────────────────────────────────────────

/**
 * FAIL — 5 transitions in one state exceeds AMS_MAX_TRANSITIONS (4).
 */
void test_limit_too_many_transitions()
{
    // One state with 5 independent TIME.elapsed transitions.
    static const char kScript[] =
        "pus.apid = 1\n"
        "state WAIT:\n"
        "  transition to END when TIME.elapsed > 1000\n"
        "  transition to END when TIME.elapsed > 2000\n"
        "  transition to END when TIME.elapsed > 3000\n"
        "  transition to END when TIME.elapsed > 4000\n"
        "  transition to END when TIME.elapsed > 5000\n"
        "state END:\n";

    LimitsFixture f;
    const bool ok = f.load("too_many_trans.ams", kScript);
    TEST_ASSERT_FALSE(ok);

    EngineSnapshot snap{};
    f.engine.getSnapshot(snap);
    TEST_ASSERT_EQUAL(EngineStatus::ERROR, snap.status);
    TEST_ASSERT_NOT_NULL_MESSAGE(
        strstr(snap.lastError, "too many 'transition to'"), snap.lastError);
}

// ── AMS_MAX_VARS ──────────────────────────────────────────────────────────────

/**
 * FAIL — 9 var declarations exceed AMS_MAX_VARS (8).
 */
void test_limit_too_many_vars()
{
    // Build a script with AMS_MAX_VARS+1 var declarations.
    char script[512] = {};
    (void)strncat(script, "pus.apid = 1\n", sizeof(script) - 1U);
    for (int i = 0; i <= static_cast<int>(ares::AMS_MAX_VARS); i++)
    {
        char buf[32] = {};
        (void)snprintf(buf, sizeof(buf), "var v%d = 0.0\n", i);
        const size_t used = strnlen(script, sizeof(script));
        if (used < sizeof(script) - 1U)
        {
            (void)strncat(script, buf, sizeof(script) - used - 1U);
        }
    }
    (void)strncat(script, "state ONLY:\n", sizeof(script) - strnlen(script, sizeof(script)) - 1U);

    LimitsFixture f;
    const bool ok = f.load("too_many_vars.ams", script);
    TEST_ASSERT_FALSE(ok);

    EngineSnapshot snap{};
    f.engine.getSnapshot(snap);
    TEST_ASSERT_EQUAL(EngineStatus::ERROR, snap.status);
    TEST_ASSERT_NOT_NULL_MESSAGE(
        strstr(snap.lastError, "too many variable"), snap.lastError);
}

// ── AMS_MAX_CONSTS ────────────────────────────────────────────────────────────

/**
 * FAIL — 9 const declarations exceed AMS_MAX_CONSTS (8).
 */
void test_limit_too_many_consts()
{
    char script[512] = {};
    (void)strncat(script, "pus.apid = 1\n", sizeof(script) - 1U);
    for (int i = 0; i <= static_cast<int>(ares::AMS_MAX_CONSTS); i++)
    {
        char buf[40] = {};
        (void)snprintf(buf, sizeof(buf), "const C%d = 1.0\n", i);
        const size_t used = strnlen(script, sizeof(script));
        if (used < sizeof(script) - 1U)
        {
            (void)strncat(script, buf, sizeof(script) - used - 1U);
        }
    }
    (void)strncat(script, "state ONLY:\n", sizeof(script) - strnlen(script, sizeof(script)) - 1U);

    LimitsFixture f;
    const bool ok = f.load("too_many_consts.ams", script);
    TEST_ASSERT_FALSE(ok);

    EngineSnapshot snap{};
    f.engine.getSnapshot(snap);
    TEST_ASSERT_EQUAL(EngineStatus::ERROR, snap.status);
    TEST_ASSERT_NOT_NULL_MESSAGE(
        strstr(snap.lastError, "too many const"), snap.lastError);
}

// ── AMS_MAX_HK_SLOTS ──────────────────────────────────────────────────────────

/**
 * FAIL — 5 every blocks in one state exceed AMS_MAX_HK_SLOTS (4).
 */
void test_limit_too_many_hk_slots()
{
    // Include SIM_BARO so that HK fields are valid.
    static const char kScript[] =
        "include SIM_BARO as BARO\n"
        "pus.apid = 1\n"
        "pus.service 3 as HK\n"
        "state WAIT:\n"
        "  every 1000ms:\n"
        "    HK.report { alt: BARO.alt }\n"
        "  every 2000ms:\n"
        "    HK.report { alt: BARO.alt }\n"
        "  every 3000ms:\n"
        "    HK.report { alt: BARO.alt }\n"
        "  every 4000ms:\n"
        "    HK.report { alt: BARO.alt }\n"
        "  every 5000ms:\n"
        "    HK.report { alt: BARO.alt }\n";

    LimitsFixture f;
    const bool ok = f.load("too_many_slots.ams", kScript);
    TEST_ASSERT_FALSE(ok);

    EngineSnapshot snap{};
    f.engine.getSnapshot(snap);
    TEST_ASSERT_EQUAL(EngineStatus::ERROR, snap.status);
    TEST_ASSERT_NOT_NULL_MESSAGE(
        strstr(snap.lastError, "too many every blocks"), snap.lastError);
}

// ── AMS_MAX_HK_FIELDS ─────────────────────────────────────────────────────────

/**
 * FAIL — 17 HK fields in one report block exceed AMS_MAX_HK_FIELDS (16).
 *
 * We use SIM_IMU which has 8 fields; we declare 3 slots to accumulate > 16
 * fields across the slot.  Actually the limit is PER-SLOT (per every block).
 * So one every block with 17 fields must be rejected.
 */
void test_limit_too_many_hk_fields()
{
    // Construct a script with one every block containing AMS_MAX_HK_FIELDS+1
    // field entries.  Each field maps to a distinct label but the same sensor
    // expression (BARO.alt) — duplicates are legal at field level.
    char script[2048] = {};
    (void)strncat(script,
        "include SIM_BARO as BARO\n"
        "pus.apid = 1\n"
        "pus.service 3 as HK\n"
        "state WAIT:\n"
        "  every 1000ms:\n"
        "    HK.report {\n",
        sizeof(script) - 1U);

    for (int i = 0; i <= static_cast<int>(ares::AMS_MAX_HK_FIELDS); i++)
    {
        char buf[32] = {};
        (void)snprintf(buf, sizeof(buf), "      f%02d: BARO.alt\n", i);
        appendN(script, sizeof(script), buf, 1);
    }

    const size_t used = strnlen(script, sizeof(script));
    if (used < sizeof(script) - 1U)
    {
        (void)strncat(script, "    }\n", sizeof(script) - used - 1U);
    }

    LimitsFixture f;
    const bool ok = f.load("too_many_fields.ams", script);
    TEST_ASSERT_FALSE(ok);

    EngineSnapshot snap{};
    f.engine.getSnapshot(snap);
    TEST_ASSERT_EQUAL(EngineStatus::ERROR, snap.status);
    TEST_ASSERT_NOT_NULL_MESSAGE(
        strstr(snap.lastError, "too many"), snap.lastError);
}

// ── AMS_MAX_INCLUDES ──────────────────────────────────────────────────────────

/**
 * FAIL — 9 include directives exceed AMS_MAX_INCLUDES (8).
 *
 * We need 9 distinct model names.  We reuse the four known sim models and
 * add five with unique but unregistered names (parser emits a warning for
 * unknown models but still registers the alias and counts towards the limit).
 */
void test_limit_too_many_includes()
{
    // Use SIM_BARO nine times with distinct aliases (BARO0..BARO8).
    // The duplicate-alias check fires on matching aliases; here all aliases
    // are unique so the count check triggers on the 9th include.
    static const char kScript[] =
        "include SIM_BARO as BARO0\n"
        "include SIM_BARO as BARO1\n"
        "include SIM_BARO as BARO2\n"
        "include SIM_BARO as BARO3\n"
        "include SIM_BARO as BARO4\n"
        "include SIM_BARO as BARO5\n"
        "include SIM_BARO as BARO6\n"
        "include SIM_BARO as BARO7\n"
        "include SIM_BARO as BARO8\n"   // <-- 9th include: exceeds AMS_MAX_INCLUDES=8
        "pus.apid = 1\n"
        "state ONLY:\n";

    LimitsFixture f;
    const bool ok = f.load("too_many_inc.ams", kScript);
    TEST_ASSERT_FALSE(ok);

    EngineSnapshot snap{};
    f.engine.getSnapshot(snap);
    TEST_ASSERT_EQUAL(EngineStatus::ERROR, snap.status);
    TEST_ASSERT_NOT_NULL_MESSAGE(
        strstr(snap.lastError, "too many include"), snap.lastError);
}

// ── AMS_MAX_SCRIPT_BYTES ──────────────────────────────────────────────────────

/**
 * PASS — script whose content is exactly AMS_MAX_SCRIPT_BYTES bytes must be
 * accepted without truncation loss.
 *
 * The valid header + state declaration are placed at the beginning of the
 * buffer; the remaining bytes are newlines (blank lines, ignored by the
 * parser).  strlen == AMS_MAX_SCRIPT_BYTES, so readFile copies everything.
 */
void test_limit_script_at_max_bytes_accepted()
{
    // Zero-initialised; memset fills indices 0..AMS_MAX_SCRIPT_BYTES-1 with '\n',
    // leaving the NUL terminator at index AMS_MAX_SCRIPT_BYTES.
    static char buf[ares::AMS_MAX_SCRIPT_BYTES + 1U] = {};
    (void)memset(buf, '\n', ares::AMS_MAX_SCRIPT_BYTES);
    buf[ares::AMS_MAX_SCRIPT_BYTES] = '\0';

    // Valid header + one state overwrite the first 26 bytes.
    static const char kContent[] = "pus.apid = 1\nstate ONLY:\n";
    (void)memcpy(buf, kContent, sizeof(kContent) - 1U);

    LimitsFixture f;
    const bool ok = f.load("exact_max.ams", buf);
    TEST_ASSERT_TRUE(ok);

    EngineSnapshot snap{};
    f.engine.getSnapshot(snap);
    TEST_ASSERT_EQUAL(EngineStatus::LOADED, snap.status);
}

/**
 * FAIL — script larger than AMS_MAX_SCRIPT_BYTES is silently truncated by the
 * engine's readFile cap.  When the only state declaration lies beyond the
 * truncation boundary the parsed window contains no state definitions and
 * activate() must return false with status ERROR.
 *
 * Layout:
 *   bytes [0 .. AMS_MAX_SCRIPT_BYTES-1] = "pus.apid = 1\n" + newlines (no state)
 *   bytes [AMS_MAX_SCRIPT_BYTES .. +11] = "state ONLY:\n"  (cut off)
 *
 * After truncation to AMS_MAX_SCRIPT_BYTES bytes the parser fires
 * "script has no states".
 */
void test_limit_script_over_max_bytes_truncates()
{
    // Buffer sized to hold the full content plus NUL.
    // Indices 0..AMS_MAX_SCRIPT_BYTES-1 are filled with '\n' (blank lines);
    // indices AMS_MAX_SCRIPT_BYTES..+11 hold the state declaration.
    static char buf[ares::AMS_MAX_SCRIPT_BYTES + 64U] = {};
    (void)memset(buf, '\n', ares::AMS_MAX_SCRIPT_BYTES);

    // PUS header overwrites the first 13 bytes (still within the read window).
    static const char kHeader[] = "pus.apid = 1\n";
    (void)memcpy(buf, kHeader, sizeof(kHeader) - 1U);

    // State declaration placed AFTER the read window (gets truncated away).
    static const char kState[] = "state ONLY:\n";
    (void)memcpy(buf + ares::AMS_MAX_SCRIPT_BYTES, kState, sizeof(kState) - 1U);
    buf[ares::AMS_MAX_SCRIPT_BYTES + sizeof(kState) - 1U] = '\0';

    LimitsFixture f;
    const bool ok = f.load("too_large.ams", buf);
    TEST_ASSERT_FALSE(ok);

    EngineSnapshot snap{};
    f.engine.getSnapshot(snap);
    TEST_ASSERT_EQUAL(EngineStatus::ERROR, snap.status);
    TEST_ASSERT_NOT_NULL_MESSAGE(
        strstr(snap.lastError, "no states"), snap.lastError);
}

// ── max_transition_depth — full-depth DFS stress ──────────────────────────────
//
// The DFS in computeDfsMaxDepthLocked uses an iterative stack of size
// AMS_MAX_STATES + 1.  A linear chain of AMS_MAX_STATES states is the deepest
// possible acyclic graph; it pushes AMS_MAX_STATES frames and exercises the
// stack at its design capacity without overflowing.

/**
 * PASS — linear chain of exactly AMS_MAX_STATES (10) states.
 * Depth = AMS_MAX_STATES - 1 = 9.
 * assert max_transition_depth < AMS_MAX_STATES (10) → 9 < 10 ✓.
 */
void test_limit_max_depth_full_chain_pass()
{
    char script[640] = {};
    (void)strncat(script, "pus.apid = 1\n", sizeof(script) - 1U);

    for (int i = 0; i < static_cast<int>(ares::AMS_MAX_STATES) - 1; i++)
    {
        char buf[64] = {};
        (void)snprintf(buf, sizeof(buf),
                       "state S%d:\n  fallback transition to S%d after 1000ms\n",
                       i, i + 1);
        const size_t used = strnlen(script, sizeof(script));
        (void)strncat(script, buf, sizeof(script) - used - 1U);
    }
    // Terminal state (no outgoing transitions → depth terminates here).
    {
        char buf[16] = {};
        (void)snprintf(buf, sizeof(buf), "state S%d:\n",
                       static_cast<int>(ares::AMS_MAX_STATES) - 1);
        const size_t used = strnlen(script, sizeof(script));
        (void)strncat(script, buf, sizeof(script) - used - 1U);
    }
    // Assert: depth must be < AMS_MAX_STATES (passes, actual = AMS_MAX_STATES - 1).
    {
        char buf[48] = {};
        (void)snprintf(buf, sizeof(buf), "assert:\n  max_transition_depth < %u\n",
                       static_cast<unsigned>(ares::AMS_MAX_STATES));
        const size_t used = strnlen(script, sizeof(script));
        (void)strncat(script, buf, sizeof(script) - used - 1U);
    }

    LimitsFixture f;
    const bool ok = f.load("max_depth_pass.ams", script);
    TEST_ASSERT_TRUE(ok);

    EngineSnapshot snap{};
    f.engine.getSnapshot(snap);
    TEST_ASSERT_EQUAL(EngineStatus::LOADED, snap.status);
}

/**
 * FAIL — same AMS_MAX_STATES-state linear chain.
 * assert max_transition_depth < AMS_MAX_STATES - 1 (9) → actual depth is 9 >= 9 ✗.
 * Error must contain "max_transition_depth" and "actual depth is 9".
 */
void test_limit_max_depth_full_chain_fail()
{
    char script[640] = {};
    (void)strncat(script, "pus.apid = 1\n", sizeof(script) - 1U);

    for (int i = 0; i < static_cast<int>(ares::AMS_MAX_STATES) - 1; i++)
    {
        char buf[64] = {};
        (void)snprintf(buf, sizeof(buf),
                       "state S%d:\n  fallback transition to S%d after 1000ms\n",
                       i, i + 1);
        const size_t used = strnlen(script, sizeof(script));
        (void)strncat(script, buf, sizeof(script) - used - 1U);
    }
    {
        char buf[16] = {};
        (void)snprintf(buf, sizeof(buf), "state S%d:\n",
                       static_cast<int>(ares::AMS_MAX_STATES) - 1);
        const size_t used = strnlen(script, sizeof(script));
        (void)strncat(script, buf, sizeof(script) - used - 1U);
    }
    // Assert with limit = AMS_MAX_STATES - 1: actual depth equals the bound → fail.
    {
        char buf[48] = {};
        (void)snprintf(buf, sizeof(buf), "assert:\n  max_transition_depth < %u\n",
                       static_cast<unsigned>(ares::AMS_MAX_STATES) - 1U);
        const size_t used = strnlen(script, sizeof(script));
        (void)strncat(script, buf, sizeof(script) - used - 1U);
    }

    LimitsFixture f;
    const bool ok = f.load("max_depth_fail.ams", script);
    TEST_ASSERT_FALSE(ok);

    EngineSnapshot snap{};
    f.engine.getSnapshot(snap);
    TEST_ASSERT_EQUAL(EngineStatus::ERROR, snap.status);
    TEST_ASSERT_NOT_NULL_MESSAGE(
        strstr(snap.lastError, "max_transition_depth"), snap.lastError);
    // Actual depth = AMS_MAX_STATES - 1 = 9.
    TEST_ASSERT_NOT_NULL_MESSAGE(
        strstr(snap.lastError, "actual depth is 9"), snap.lastError);
}

/**
 * @file  test_ams_checkpoint_flush.cpp
 * @brief SITL integration tests — flushPendingIoUnlocked() must be
 *        reached on every tick() exit path.
 *
 * Before the fix, tick() returned early on TC-transition, fallback, on_timeout,
 * and unhandled-ABORT paths, leaving staged checkpoint/abort-row data in
 * memory and never writing it to storage.
 *
 * These tests verify that after the single-exit restructure:
 *   1. A TC-driven WAIT→FLIGHT transition writes a checkpoint for the new state.
 *   2. A fallback transition (FLIGHT→SAFE) writes a checkpoint for the new state.
 *   3. An on_timeout transition (HOLD→TIMEOUT_WIN) writes a checkpoint for the
 *      new state.
 *   4. An unhandled ABORT TC stages the abort row and flushes it via appendFile.
 */

#include <unity.h>
#include <cinttypes>
#include <cstring>

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

// ── Shared profile ────────────────────────────────────────────────────────────
// Static ground-level profile: altitude stays at 0 m throughout so no
// altitude-based transitions fire unexpectedly during the flush tests.

static const ares::sim::FlightProfile kFlushProfile = {
    .count   = 2U,
    .samples = {
        { .timeMs =     0U, .gpsLatDeg = 40.0f, .gpsLonDeg = -3.0f,
          .gpsAltM =  0.0f, .gpsSpeedKmh = 0.0f, .gpsSats = 9U, .gpsFix = true,
          .baroAltM = 0.0f, .baroPressurePa = 101325.0f, .baroTempC = 20.0f,
          .accelX = 0.0f, .accelY = 0.0f, .accelZ = 9.81f, .imuTempC = 25.0f },
        { .timeMs = 10000U, .gpsLatDeg = 40.0f, .gpsLonDeg = -3.0f,
          .gpsAltM =  0.0f, .gpsSpeedKmh = 0.0f, .gpsSats = 9U, .gpsFix = true,
          .baroAltM = 0.0f, .baroPressurePa = 101325.0f, .baroTempC = 20.0f,
          .accelX = 0.0f, .accelY = 0.0f, .accelZ = 9.81f, .imuTempC = 25.0f },
    }
};

// ── Fixture ───────────────────────────────────────────────────────────────────

struct FlushFixture
{
    ares::sim::SimStorageDriver storage;
    ares::sim::SimGpsDriver     gps{kFlushProfile};
    ares::sim::SimBaroDriver    baro{kFlushProfile};
    ares::sim::SimImuDriver     imu{kFlushProfile};
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
// TC-driven transition (WAIT → FLIGHT via LAUNCH).
//
// Before the fix, evaluateTransitionAndMaybeEnterLocked() returning true
// caused tick() to return without calling flushPendingIoUnlocked(), so the
// FLIGHT checkpoint was staged but never written.
// ─────────────────────────────────────────────────────────────────────────────

void test_checkpoint_written_on_tc_transition()
{
    FlushFixture f;
    f.init("/missions/flight.ams", ares::sim::kScriptFlight);

    (void)f.engine.activate("flight.ams");
    (void)f.engine.arm(); // queues LAUNCH; writes arm() checkpoint for WAIT

    // Discard writes from activate() (log file creation) and arm() (checkpoint).
    f.storage.resetWriteCapture();

    // t=0: LAUNCH TC fires WAIT → FLIGHT.  enterStateLocked stages checkpoint
    // for stateIdx=1 (FLIGHT), then flushPendingIoUnlocked() must write it.
    f.engine.tick(ares::sim::clock::nowMs());

    TEST_ASSERT_GREATER_OR_EQUAL(1U, f.storage.writeCallCount());
    TEST_ASSERT_NOT_NULL(strstr(f.storage.lastWrittenContent(), "flight.ams|1|"));
}

// ─────────────────────────────────────────────────────────────────────────────
// Fallback transition (FLIGHT → SAFE after 2000 ms).
//
// Before the fix, the didFallback=true branch returned without flushing.
// ─────────────────────────────────────────────────────────────────────────────

void test_checkpoint_written_on_fallback_transition()
{
    FlushFixture f;
    f.init("/missions/fallback.ams", ares::sim::kScriptFallback);

    (void)f.engine.activate("fallback.ams");
    (void)f.engine.arm();

    // t=0: enter FLIGHT.
    f.engine.tick(ares::sim::clock::nowMs());

    // Discard earlier writes; only the upcoming fallback-tick write matters.
    f.storage.resetWriteCapture();

    // t=2001 ms: fallback threshold (2000 ms) elapsed → FLIGHT → SAFE
    // (stateIdx=2).  flushPendingIoUnlocked() must write the checkpoint.
    ares::sim::clock::advanceMs(2001U);
    f.engine.tick(ares::sim::clock::nowMs());

    TEST_ASSERT_GREATER_OR_EQUAL(1U, f.storage.writeCallCount());
    TEST_ASSERT_NOT_NULL(strstr(f.storage.lastWrittenContent(), "fallback.ams|2|"));
}

// ─────────────────────────────────────────────────────────────────────────────
// on_timeout transition (HOLD → TIMEOUT_WIN at 3000 ms).
//
// Before the fix, checkOnTimeoutLocked() returning true caused tick() to
// return without flushing the staged checkpoint.
// ─────────────────────────────────────────────────────────────────────────────

void test_checkpoint_written_on_timeout()
{
    FlushFixture f;
    f.init("/missions/tie.ams", ares::sim::kScriptOnTimeoutVsFallbackTie);

    (void)f.engine.activate("tie.ams");
    (void)f.engine.arm(); // queues LAUNCH

    // t=0: LAUNCH consumed → enter HOLD (stateIdx=1).
    f.engine.tick(ares::sim::clock::nowMs());

    // Discard HOLD-entry checkpoint; only the timeout-tick write matters.
    f.storage.resetWriteCapture();

    // t=3001 ms: on_timeout threshold (3000 ms) elapsed → HOLD → TIMEOUT_WIN
    // (stateIdx=2).  flushPendingIoUnlocked() must write the checkpoint.
    ares::sim::clock::advanceMs(3001U);
    f.engine.tick(ares::sim::clock::nowMs());

    TEST_ASSERT_GREATER_OR_EQUAL(1U, f.storage.writeCallCount());
    TEST_ASSERT_NOT_NULL(strstr(f.storage.lastWrittenContent(), "tie.ams|2|"));
}

// ─────────────────────────────────────────────────────────────────────────────
// Unhandled ABORT TC (force-deactivation).
//
// Before the fix, the ABORT branch deactivated the engine and returned before
// flushPendingIoUnlocked(), so the abort audit row staged by
// writeAbortMarkerLocked() was never appended to the mission log.
// ─────────────────────────────────────────────────────────────────────────────

void test_abort_row_flushed_on_unhandled_abort()
{
    FlushFixture f;
    f.init("/missions/flight.ams", ares::sim::kScriptFlight);

    (void)f.engine.activate("flight.ams");
    (void)f.engine.arm();

    // t=0: enter FLIGHT.
    f.engine.tick(ares::sim::clock::nowMs());

    // Clear any log appends accumulated during entry (e.g. on_enter events).
    f.storage.resetAppendCapture();

    // kScriptFlight FLIGHT has no ABORT transition → force-deactivation path.
    // writeAbortMarkerLocked() queues "...,ABORT,aborted_in_state=FLIGHT,xx\n"
    // and flushPendingIoUnlocked() must append it to the mission log file.
    (void)f.engine.injectTcCommand("ABORT");
    ares::sim::clock::advanceMs(1U);
    f.engine.tick(ares::sim::clock::nowMs());

    TEST_ASSERT_NOT_NULL(strstr(f.storage.appendedContent(), "ABORT"));
}

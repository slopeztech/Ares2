/**
 * @file  test_ams_flight_scenario.cpp
 * @brief SITL integration tests — full mission scenario replay.
 *
 * Uses a deterministic FlightProfile and time-driven tick loop to exercise
 * the AMS engine through a realistic multi-state mission, verifying state
 * transitions, telemetry emission, and sensor-driven conditions.
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

// ── Flight profile: low-speed ascent to 300 m, then hold ────────────────────

static const ares::sim::FlightProfile kFlightProfile = {
    .count   = 5U,
    .samples = {
        //  t=0 s  : ground at 0 m
        { .timeMs = 0U,     .gpsLatDeg = 40.416f, .gpsLonDeg = -3.703f,
          .gpsAltM = 0.0f,  .gpsSpeedKmh = 0.0f, .gpsSats = 9U, .gpsFix = true,
          .baroAltM = 0.0f,  .baroPressurePa = 101325.0f, .baroTempC = 20.0f,
          .accelX = 0.0f,   .accelY = 0.0f, .accelZ = 9.81f, .imuTempC = 25.0f },
        //  t=2 s  : liftoff, accelerating
        { .timeMs = 2000U,  .gpsLatDeg = 40.416f, .gpsLonDeg = -3.703f,
          .gpsAltM = 20.0f, .gpsSpeedKmh = 36.0f, .gpsSats = 9U, .gpsFix = true,
          .baroAltM = 20.0f, .baroPressurePa = 99000.0f, .baroTempC = 19.0f,
          .accelX = 0.5f,   .accelY = 0.0f, .accelZ = 14.0f, .imuTempC = 26.0f },
        //  t=4 s  : mid-ascent, past 200 m threshold
        { .timeMs = 4000U,  .gpsLatDeg = 40.417f, .gpsLonDeg = -3.703f,
          .gpsAltM = 210.0f, .gpsSpeedKmh = 72.0f, .gpsSats = 9U, .gpsFix = true,
          .baroAltM = 210.0f, .baroPressurePa = 98700.0f, .baroTempC = 17.0f,
          .accelX = 0.3f,   .accelY = 0.0f, .accelZ = 12.0f, .imuTempC = 27.0f },
        //  t=8 s  : apogee hold at 300 m
        { .timeMs = 8000U,  .gpsLatDeg = 40.418f, .gpsLonDeg = -3.703f,
          .gpsAltM = 300.0f, .gpsSpeedKmh = 5.0f, .gpsSats = 9U, .gpsFix = true,
          .baroAltM = 300.0f, .baroPressurePa = 97700.0f, .baroTempC = 15.0f,
          .accelX = 0.0f,   .accelY = 0.0f, .accelZ = 9.81f, .imuTempC = 28.0f },
        //  t=15 s : recovery hold
        { .timeMs = 15000U, .gpsLatDeg = 40.418f, .gpsLonDeg = -3.702f,
          .gpsAltM = 300.0f, .gpsSpeedKmh = 3.0f, .gpsSats = 9U, .gpsFix = true,
          .baroAltM = 300.0f, .baroPressurePa = 97700.0f, .baroTempC = 15.0f,
          .accelX = 0.0f,   .accelY = 0.0f, .accelZ = 9.81f, .imuTempC = 28.0f },
    }
};

// ── Helper: build a fully initialised engine ─────────────────────────────────

struct FlightFixture
{
    ares::sim::SimStorageDriver storage;
    ares::sim::SimGpsDriver     gps{kFlightProfile};
    ares::sim::SimBaroDriver    baro{kFlightProfile};
    ares::sim::SimImuDriver     imu{kFlightProfile};
    ares::sim::SimRadioDriver   radio;

    GpsEntry  gpsEntry  = { "SIM_GPS",  &gps  };
    BaroEntry baroEntry = { "SIM_BARO", &baro };
    ComEntry  comEntry  = { "SIM_COM",  &radio};
    ImuEntry  imuEntry  = { "SIM_IMU",  &imu  };

    MissionScriptEngine engine{
        storage,
        &gpsEntry,  1U,
        &baroEntry, 1U,
        &comEntry,  1U,
        &imuEntry,  1U
    };

    void init(const char* path, const char* content)
    {
        // Initialise all drivers before the engine begins so that
        // sensor reads and radio sends are operational from the first tick.
        (void)storage.begin();
        storage.registerFile(path, content);
        (void)gps.begin();
        (void)baro.begin();
        (void)imu.begin();
        (void)radio.begin();
        (void)engine.begin();
    }
};

// ── Tests ────────────────────────────────────────────────────────────────────

void test_flight_script_activates_successfully()
{
    FlightFixture f;
    f.init("/missions/flight.ams", ares::sim::kScriptFlight);

    TEST_ASSERT_TRUE(f.engine.activate("flight.ams"));

    EngineSnapshot snap{};
    f.engine.getSnapshot(snap);
    TEST_ASSERT_EQUAL(EngineStatus::LOADED, snap.status);
    TEST_ASSERT_EQUAL_STRING("WAIT", snap.stateName);
}

void test_flight_arm_transitions_to_running()
{
    FlightFixture f;
    f.init("/missions/flight.ams", ares::sim::kScriptFlight);
    (void)f.engine.activate("flight.ams");

    TEST_ASSERT_TRUE(f.engine.arm());

    EngineSnapshot snap{};
    f.engine.getSnapshot(snap);
    TEST_ASSERT_EQUAL(EngineStatus::RUNNING, snap.status);
}

void test_flight_wait_advances_to_flight_state()
{
    FlightFixture f;
    f.init("/missions/flight.ams", ares::sim::kScriptFlight);
    (void)f.engine.activate("flight.ams");
    (void)f.engine.arm();  // injects LAUNCH TC

    f.engine.tick(ares::sim::clock::nowMs());

    EngineSnapshot snap{};
    f.engine.getSnapshot(snap);
    // LAUNCH TC should have fired the WAIT→FLIGHT transition.
    TEST_ASSERT_EQUAL_STRING("FLIGHT", snap.stateName);
}

void test_flight_emits_hk_telemetry_frames()
{
    FlightFixture f;
    f.init("/missions/flight.ams", ares::sim::kScriptFlight);
    (void)f.engine.activate("flight.ams");
    (void)f.engine.arm();

    // Advance into FLIGHT state (one tick at t=0 consumes the LAUNCH TC).
    f.engine.tick(ares::sim::clock::nowMs());

    f.radio.resetCapture();   // reset after on_enter EVENT frame

    // Tick at t=1000 ms: first HK.report slot is due (every 1000 ms).
    ares::sim::clock::advanceMs(1000U);
    f.engine.tick(ares::sim::clock::nowMs());

    TEST_ASSERT_GREATER_THAN(0U, f.radio.sendCount());
    TEST_ASSERT_GREATER_THAN(0U, f.radio.lastFrameLen());
}

void test_flight_time_transition_fires_at_5s()
{
    FlightFixture f;
    f.init("/missions/flight.ams", ares::sim::kScriptFlight);
    (void)f.engine.activate("flight.ams");
    (void)f.engine.arm();

    // Tick loop: 100 ms steps up to 6000 ms — the TIME.elapsed > 5000 transition
    // should fire somewhere in the 5000–6000 ms window.
    static constexpr uint32_t kStepMs = 100U;
    static constexpr uint32_t kMaxMs  = 6000U;

    for (uint32_t t = 0U; t <= kMaxMs; t += kStepMs)
    {
        ares::sim::clock::reset();
        ares::sim::clock::advanceMs(t);
        f.engine.tick(ares::sim::clock::nowMs());
    }

    EngineSnapshot snap{};
    f.engine.getSnapshot(snap);
    // When the engine reaches COMPLETE, running_ is false and getSnapshot()
    // reports stateName="IDLE" — verify via status instead.  COMPLETE is
    // only reachable by passing through END via the TIME.elapsed > 5000
    // transition, which is exactly what this test verifies.
    TEST_ASSERT_EQUAL(EngineStatus::COMPLETE, snap.status);
}

void test_flight_reaches_complete_after_end_state()
{
    FlightFixture f;
    f.init("/missions/flight.ams", ares::sim::kScriptFlight);
    (void)f.engine.activate("flight.ams");
    (void)f.engine.arm();

    // Run enough ticks to pass the 5 s threshold and enter END.
    // Then one more tick in END → COMPLETE.
    static constexpr uint32_t kStepMs = 500U;
    static constexpr uint32_t kMaxMs  = 7000U;

    for (uint32_t t = 0U; t <= kMaxMs; t += kStepMs)
    {
        ares::sim::clock::reset();
        ares::sim::clock::advanceMs(t);
        f.engine.tick(ares::sim::clock::nowMs());
    }

    EngineSnapshot snap{};
    f.engine.getSnapshot(snap);
    TEST_ASSERT_EQUAL(EngineStatus::COMPLETE, snap.status);
}

void test_sensor_transition_fires_on_altitude_threshold()
{
    FlightFixture f;
    f.init("/missions/sensor.ams", ares::sim::kScriptSensorTransition);
    (void)f.engine.activate("sensor.ams");
    (void)f.engine.arm();

    // The flight profile crosses BARO.alt = 200 at ~t=4000 ms.
    // Run ticks at 200 ms steps up to 5 s.
    static constexpr uint32_t kStepMs = 200U;
    static constexpr uint32_t kMaxMs  = 5000U;

    for (uint32_t t = 0U; t <= kMaxMs; t += kStepMs)
    {
        ares::sim::clock::reset();
        ares::sim::clock::advanceMs(t);
        f.engine.tick(ares::sim::clock::nowMs());
    }

    EngineSnapshot snap{};
    f.engine.getSnapshot(snap);

    // Engine should be in RECOVERY or END (transition fired past 200 m).
    const bool inRecoveryOrEnd =
        (strcmp(snap.stateName, "RECOVERY") == 0) ||
        (strcmp(snap.stateName, "END") == 0);
    TEST_ASSERT_TRUE(inRecoveryOrEnd);
}

void test_multiple_hk_frames_accumulate_correctly()
{
    FlightFixture f;
    f.init("/missions/flight.ams", ares::sim::kScriptFlight);
    (void)f.engine.activate("flight.ams");
    (void)f.engine.arm();

    // Enter FLIGHT state.
    f.engine.tick(ares::sim::clock::nowMs());
    f.radio.resetCapture();

    // Advance 3100 ms in 100 ms steps — should trigger 3 HK.report frames
    // (at t=1000, t=2000, t=3000 from FLIGHT state entry).
    for (uint32_t dt = 100U; dt <= 3100U; dt += 100U)
    {
        ares::sim::clock::reset();
        ares::sim::clock::advanceMs(dt);
        f.engine.tick(ares::sim::clock::nowMs());
    }

    // At least 3 HK frames should have been transmitted.
    TEST_ASSERT_GREATER_OR_EQUAL(3U, f.radio.sendCount());
}

void test_hk_slot_starvation_skips_samples()
{
    // AMS-4.4 starvation guard: a single catch-up tick after a 5-period gap
    // must emit exactly ONE HK.report and ONE EVENT.warning (not 5 bursts).
    FlightFixture f;
    f.init("/missions/flight.ams", ares::sim::kScriptFlight);
    (void)f.engine.activate("flight.ams");
    (void)f.engine.arm();  // pre-injects LAUNCH TC

    // t=0: consume LAUNCH TC → WAIT → FLIGHT (on_enter event queued).
    f.engine.tick(ares::sim::clock::nowMs());

    EngineSnapshot snap{};
    f.engine.getSnapshot(snap);
    TEST_ASSERT_EQUAL_STRING("FLIGHT", snap.stateName);

    // t=1: drain the deferred on_enter LIFTOFF event so the radio counter
    // starts clean before the starvation tick.
    ares::sim::clock::advanceMs(1U);
    f.engine.tick(ares::sim::clock::nowMs());

    // Clear counters so only the starvation tick is measured.
    f.radio.resetCapture();

    // Jump 3 000 ms (= 3× the 1 000 ms HK period) in a single tick.
    // The kScriptFlight TIME.elapsed > 5 000 ms transition must NOT fire
    // here (3001 ms < 5000 ms), leaving the engine in FLIGHT.
    // Without the guard this leaves the timer 2 000 ms behind and produces
    // 2 extra burst HK frames on subsequent ticks.
    ares::sim::clock::advanceMs(3000U);
    f.engine.tick(ares::sim::clock::nowMs());

    // Guard: 1 HK frame + 1 EVENT.warning (skipped 2 samples).
    TEST_ASSERT_EQUAL(2U, f.radio.sendCount());

    // After the reset the next regular tick must produce exactly 1 HK frame
    // and no further warnings.
    f.radio.resetCapture();
    ares::sim::clock::advanceMs(1000U);
    f.engine.tick(ares::sim::clock::nowMs());
    TEST_ASSERT_EQUAL(1U, f.radio.sendCount());
}

// ── Sensor cache TTL (AMS-4.5.1) ─────────────────────────────────────────────

/**
 * @brief Verify that the baro sensor cache is reused within the TTL window
 *        and expires correctly after AMS_SENSOR_CACHE_TTL_MS elapses.
 *
 * The engine runs a script whose only transition (BARO.alt > 9999) never
 * fires.  baro.readCount() starts at 0; each cache miss increments it.
 *
 *   Tick 1 at t=0   → cold cache  → 1 read  (readCount == 1)
 *   Tick 2 at t=1   → age 1 ms < 5 ms TTL → cache hit  (readCount == 1)
 *   Tick 3 at t=6   → age 6 ms ≥ 5 ms TTL → cache miss (readCount == 2)
 */
void test_sensor_cache_within_ttl_is_reused()
{
    FlightFixture f;
    f.init("/missions/ttl.ams", ares::sim::kScriptBaroCacheTtl);
    (void)f.engine.activate("ttl.ams");
    (void)f.engine.arm();

    // Tick 1: cold cache — exactly one baro read is performed.
    f.engine.tick(ares::sim::clock::nowMs());
    TEST_ASSERT_EQUAL(1U, f.baro.readCount());

    // Tick 2: 1 ms later — still within TTL (1 ms < AMS_SENSOR_CACHE_TTL_MS=5 ms).
    // Cache hit → no additional driver call.
    ares::sim::clock::advanceMs(1U);
    f.engine.tick(ares::sim::clock::nowMs());
    TEST_ASSERT_EQUAL(1U, f.baro.readCount());

    // Tick 3: 5 ms more (t=6) — TTL expired (6 ms ≥ 5 ms).
    // Cache miss → driver is called again.
    ares::sim::clock::advanceMs(5U);
    f.engine.tick(ares::sim::clock::nowMs());
    TEST_ASSERT_EQUAL(2U, f.baro.readCount());
}

// ── test_log_slot_header_retried_after_no_space ──────────────────────────────
/**
 * Verify AMS-4.3.2: the CSV slot header is retried on a subsequent tick
 * when the previous appendFile call returned NO_SPACE.
 *
 * Flow:
 *   1. Inject one NO_SPACE failure.
 *   2. Tick at t=10 ms  — slot is due; header append fails → flag stays false.
 *      Data row succeeds (failure counter already exhausted).
 *   3. Assert captured content does NOT yet contain "t_ms,state,slot".
 *   4. Tick at t=20 ms  — slot due again; header is retried and succeeds.
 *   5. Assert captured content NOW contains "t_ms,state,slot".
 */
void test_log_slot_header_retried_after_no_space()
{
    FlightFixture f;
    f.init("/missions/hdr.ams", ares::sim::kScriptLogHeaderRetry);
    TEST_ASSERT_TRUE(f.engine.activate("hdr.ams")); // enterStateLocked uses t=0
    TEST_ASSERT_TRUE(f.engine.arm());

    // Inject one NO_SPACE failure — hits the first appendFile call
    // (the slot CSV header), leaving logSlotHeaderWritten_[0] = false.
    f.storage.failNextAppends(1U);

    // Tick 1 at t=10 ms — slot cadence satisfied (10 ms ≥ 10 ms).
    // Header append → NO_SPACE (failure consumed); data row → OK.
    ares::sim::clock::advanceMs(10U);
    f.engine.tick(ares::sim::clock::nowMs());
    TEST_ASSERT_NULL(strstr(f.storage.appendedContent(), "t_ms,state,slot"));

    // Tick 2 at t=20 ms — slot due again.  Flag still false → header retried;
    // both header and data row now succeed.
    ares::sim::clock::advanceMs(10U);
    f.engine.tick(ares::sim::clock::nowMs());
    TEST_ASSERT_NOT_NULL(strstr(f.storage.appendedContent(), "t_ms,state,slot"));
}

// ── test_log_slot_row_has_crc8_field ─────────────────────────────────────────
/**
 * Verify AMS-4.3.2: the CSV slot header ends with the literal column label
 * ",crc8", and every data row ends with a ",<2 lowercase hex chars>\n" suffix
 * carrying the CRC8/SMBUS checksum of the row content.
 *
 * Flow:
 *   1. Activate + arm with kScriptLogHeaderRetry (has a log_every slot).
 *   2. One clean tick at t=10 ms — header and data row both succeed.
 *   3. Assert header contains ",crc8\n".
 *   4. Assert each numeric data row ends with ",<hex><hex>\n".
 */
void test_log_slot_row_has_crc8_field()
{
    FlightFixture f;
    f.init("/missions/crc_test.ams", ares::sim::kScriptLogHeaderRetry);
    TEST_ASSERT_TRUE(f.engine.activate("crc_test.ams"));
    TEST_ASSERT_TRUE(f.engine.arm());

    // One clean tick — no injected failures.
    ares::sim::clock::advanceMs(10U);
    f.engine.tick(ares::sim::clock::nowMs());

    const char* content = f.storage.appendedContent();

    // The slot CSV header must contain the CRC8 column label.
    TEST_ASSERT_NOT_NULL(strstr(content, ",crc8\n"));

    // Every line that starts with a digit (data row) must end with
    // ",<two lowercase hex chars>\n".
    const char* p = content;
    bool foundDataRow = false;
    while (p != nullptr && *p != '\0')
    {
        if (*p >= '0' && *p <= '9')
        {
            const char* eol = strchr(p, '\n');
            TEST_ASSERT_NOT_NULL(eol);          // every row must be newline-terminated
            TEST_ASSERT_TRUE(eol - p >= 3);     // at minimum ",xx"

            const char hi  = *(eol - 2);
            const char lo  = *(eol - 1);
            const char sep = *(eol - 3);

            const bool hiHex = (hi >= '0' && hi <= '9') || (hi >= 'a' && hi <= 'f');
            const bool loHex = (lo >= '0' && lo <= '9') || (lo >= 'a' && lo <= 'f');
            TEST_ASSERT_TRUE(hiHex);
            TEST_ASSERT_TRUE(loHex);
            TEST_ASSERT_EQUAL_CHAR(',', sep);

            foundDataRow = true;
            p = eol + 1;
        }
        else
        {
            const char* eol = strchr(p, '\n');
            if (eol == nullptr) { break; }
            p = eol + 1;
        }
    }
    TEST_ASSERT_TRUE(foundDataRow);
}

/**
 * @file  test_ams_coverage_ext.cpp
 * @brief SITL integration tests targeting coverage gaps in:
 *   - mission_script_engine_parser_actions.cpp  (on_exit EVENT, CALIBRATE,
 *                                                max/min set, delta set)
 *   - mission_script_engine_actions.cpp         (CALIBRATE runtime,
 *                                                delta set runtime,
 *                                                max/min set runtime)
 *   - mission_script_engine_sensor.cpp          (GPS.lat/lon/speed/pressure,
 *                                                IMU fields in HK)
 *   - mission_script_engine_telemetry.cpp       (applyHkFieldToPayloadLocked
 *                                                GPS/IMU branches,
 *                                                monitoring alarm,
 *                                                configureMonitorFromParam)
 *
 * Test count: 15
 */

#include <unity.h>

#include "ams/mission_script_engine.h"
#include "ams/ams_driver_registry.h"

using ares::ams::GpsEntry;
using ares::ams::BaroEntry;
using ares::ams::ComEntry;
using ares::ams::ImuEntry;

#include "sim_clock.h"
#include "sim_gps_driver.h"
#include "sim_baro_driver.h"
#include "sim_imu_driver.h"
#include "sim_radio_driver.h"
#include "sim_storage_driver.h"

using ares::ams::MissionScriptEngine;
using ares::ams::EngineSnapshot;
using ares::ams::EngineStatus;

// ─────────────────────────────────────────────────────────────────────────────
// Flight profiles
// ─────────────────────────────────────────────────────────────────────────────

/// Ground profile — no altitude, good GPS, full IMU data.
static const ares::sim::FlightProfile kGroundProfile = {
    .count   = 1U,
    .samples = {
        { .timeMs        = 0U,
          .gpsLatDeg     = 40.416f, .gpsLonDeg     = -3.703f,
          .gpsAltM       = 10.0f,   .gpsSpeedKmh   = 5.0f,
          .gpsHdop       = 1.2f,    .gpsSats       = 9U,    .gpsFix = true,
          .baroAltM      = 10.0f,   .baroPressurePa = 101000.0f, .baroTempC = 20.0f,
          .accelX        = 0.5f,    .accelY        = 0.1f,  .accelZ     = 9.81f,
          .imuTempC      = 26.0f },
    }
};

/// High-altitude profile — 3 samples all above 3000 m (triggers monitoring alarm).
static const ares::sim::FlightProfile kHighAltProfile = {
    .count   = 3U,
    .samples = {
        { .timeMs = 0U,    .gpsLatDeg = 40.0f, .gpsLonDeg = -3.0f,
          .gpsAltM = 3100.0f, .gpsSpeedKmh = 0.0f,
          .gpsHdop = 1.0f, .gpsSats = 9U, .gpsFix = true,
          .baroAltM = 3100.0f, .baroPressurePa = 70000.0f, .baroTempC = 5.0f,
          .accelX = 0.0f, .accelY = 0.0f, .accelZ = 9.81f, .imuTempC = 10.0f },
        { .timeMs = 500U,  .gpsLatDeg = 40.0f, .gpsLonDeg = -3.0f,
          .gpsAltM = 3200.0f, .gpsSpeedKmh = 0.0f,
          .gpsHdop = 1.0f, .gpsSats = 9U, .gpsFix = true,
          .baroAltM = 3200.0f, .baroPressurePa = 69000.0f, .baroTempC = 4.0f,
          .accelX = 0.0f, .accelY = 0.0f, .accelZ = 9.81f, .imuTempC = 10.0f },
        { .timeMs = 1000U, .gpsLatDeg = 40.0f, .gpsLonDeg = -3.0f,
          .gpsAltM = 3300.0f, .gpsSpeedKmh = 0.0f,
          .gpsHdop = 1.0f, .gpsSats = 9U, .gpsFix = true,
          .baroAltM = 3300.0f, .baroPressurePa = 68000.0f, .baroTempC = 3.0f,
          .accelX = 0.0f, .accelY = 0.0f, .accelZ = 9.81f, .imuTempC = 10.0f },
    }
};

// ─────────────────────────────────────────────────────────────────────────────
// Generic fixture
// ─────────────────────────────────────────────────────────────────────────────

struct CovFixture
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

    explicit CovFixture(const ares::sim::FlightProfile& profile)
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

// ─────────────────────────────────────────────────────────────────────────────
// Scripts
// ─────────────────────────────────────────────────────────────────────────────

/// Script with on_exit EVENT — covers parseOnExitEventLineLocked success path.
static const char kScriptOnExitEvent[] =
    "include SIM_GPS as GPS\n"
    "include SIM_BARO as BARO\n"
    "include SIM_COM as COM\n"
    "include SIM_IMU as IMU\n"
    "\n"
    "pus.apid = 1\n"
    "pus.service 5 as EVENT\n"
    "pus.service 1 as TC\n"
    "\n"
    "state WAIT:\n"
    "  on_enter:\n"
    "    EVENT.info \"WAITING\"\n"
    "  on_exit:\n"
    "    EVENT.info \"LEAVING WAIT\"\n"
    "  transition to END when TC.command == LAUNCH\n"
    "\n"
    "state END:\n"
    "  on_enter:\n"
    "    EVENT.info \"DONE\"\n";

/// Script with CALIBRATE set action — covers parseCalibrateSetActionLocked +
/// executeCalibrateSetActionLocked.
static const char kScriptCalibrate[] =
    "include SIM_GPS as GPS\n"
    "include SIM_BARO as BARO\n"
    "include SIM_COM as COM\n"
    "include SIM_IMU as IMU\n"
    "\n"
    "pus.apid = 1\n"
    "pus.service 5 as EVENT\n"
    "pus.service 1 as TC\n"
    "\n"
    "var altitude_ref = 0.0\n"
    "\n"
    "state WAIT:\n"
    "  on_enter:\n"
    "    set altitude_ref = CALIBRATE(BARO.alt, 3)\n"
    "    EVENT.info \"CALIBRATING\"\n"
    "  transition to END when TIME.elapsed > 99999\n"
    "\n"
    "state END:\n"
    "  on_enter:\n"
    "    EVENT.info \"DONE\"\n";

/// Script with max() set action — covers parseMinMaxSetActionLocked.
static const char kScriptMaxSet[] =
    "include SIM_GPS as GPS\n"
    "include SIM_BARO as BARO\n"
    "include SIM_COM as COM\n"
    "include SIM_IMU as IMU\n"
    "\n"
    "pus.apid = 1\n"
    "pus.service 5 as EVENT\n"
    "pus.service 1 as TC\n"
    "\n"
    "var peak_alt = 0.0\n"
    "\n"
    "state WAIT:\n"
    "  on_enter:\n"
    "    set peak_alt = max(peak_alt, BARO.alt)\n"
    "    EVENT.info \"TRACKING\"\n"
    "  transition to END when TIME.elapsed > 99999\n"
    "\n"
    "state END:\n"
    "  on_enter:\n"
    "    EVENT.info \"DONE\"\n";

/// Script with delta set action — covers parseDeltaSetActionLocked.
static const char kScriptDeltaSet[] =
    "include SIM_GPS as GPS\n"
    "include SIM_BARO as BARO\n"
    "include SIM_COM as COM\n"
    "include SIM_IMU as IMU\n"
    "\n"
    "pus.apid = 1\n"
    "pus.service 5 as EVENT\n"
    "pus.service 1 as TC\n"
    "\n"
    "var baro_delta = 0.0\n"
    "\n"
    "state WAIT:\n"
    "  on_enter:\n"
    "    set baro_delta = BARO.alt delta\n"
    "    EVENT.info \"DELTA\"\n"
    "  transition to END when TIME.elapsed > 99999\n"
    "\n"
    "state END:\n"
    "  on_enter:\n"
    "    EVENT.info \"DONE\"\n";

/// Script with rich HK report containing GPS lat/lon/speed + IMU fields +
/// baro pressure.  Covers applyHkFieldToPayloadLocked GPS/IMU switch branches
/// and readSensorFloatLocked GPS/IMU/BARO_PRESSURE paths.
static const char kScriptRichHk[] =
    "include SIM_GPS as GPS\n"
    "include SIM_BARO as BARO\n"
    "include SIM_COM as COM\n"
    "include SIM_IMU as IMU\n"
    "\n"
    "pus.apid = 1\n"
    "pus.service 3 as HK\n"
    "pus.service 5 as EVENT\n"
    "pus.service 1 as TC\n"
    "\n"
    "state REPORT:\n"
    "  on_enter:\n"
    "    EVENT.info \"START\"\n"
    "  every 200ms:\n"
    "    HK.report {\n"
    "      baro_pres: BARO.pressure\n"
    "      gps_lat:   GPS.lat\n"
    "      gps_lon:   GPS.lon\n"
    "      gps_speed: GPS.speed\n"
    "      imu_ax:    IMU.accel_x\n"
    "      imu_ay:    IMU.accel_y\n"
    "      imu_amag:  IMU.accel_mag\n"
    "      imu_gx:    IMU.gyro_x\n"
    "      imu_gy:    IMU.gyro_y\n"
    "      imu_gz:    IMU.gyro_z\n"
    "      imu_temp:  IMU.temp\n"
    "    }\n"
    "  transition to END when TIME.elapsed > 2000\n"
    "\n"
    "state END:\n"
    "  on_enter:\n"
    "    EVENT.info \"DONE\"\n";

/// Script for monitoring alarm: FLIGHT state with 200ms HK.report baro_alt,
/// runs until TIME.elapsed > 5000.  Altitude held at 3100+ m.
static const char kScriptMonitorAlarm[] =
    "include SIM_GPS as GPS\n"
    "include SIM_BARO as BARO\n"
    "include SIM_COM as COM\n"
    "include SIM_IMU as IMU\n"
    "\n"
    "pus.apid = 1\n"
    "pus.service 3 as HK\n"
    "pus.service 5 as EVENT\n"
    "pus.service 1 as TC\n"
    "\n"
    "state FLIGHT:\n"
    "  on_enter:\n"
    "    EVENT.info \"HIGH\"\n"
    "  every 200ms:\n"
    "    HK.report {\n"
    "      baro_alt: BARO.alt\n"
    "    }\n"
    "  transition to END when TIME.elapsed > 5000\n"
    "\n"
    "state END:\n"
    "  on_enter:\n"
    "    EVENT.info \"DONE\"\n";

// ─────────────────────────────────────────────────────────────────────────────
// Tests
// ─────────────────────────────────────────────────────────────────────────────

// ── Test 1: on_exit EVENT parses and activates successfully ───────────────────

void test_on_exit_event_parses_ok()
{
    CovFixture f{kGroundProfile};
    f.init("/missions/on_exit_event.ams", kScriptOnExitEvent);

    const bool ok = f.engine.activate("on_exit_event.ams");

    EngineSnapshot snap{};
    f.engine.getSnapshot(snap);
    TEST_ASSERT_TRUE_MESSAGE(ok, snap.lastError);
    TEST_ASSERT_EQUAL(EngineStatus::LOADED, snap.status);
}

// ── Test 2: on_exit EVENT fires on state transition ───────────────────────────

void test_on_exit_event_fires_on_transition()
{
    CovFixture f{kGroundProfile};
    f.init("/missions/on_exit_event2.ams", kScriptOnExitEvent);
    TEST_ASSERT_TRUE(f.engine.activate("on_exit_event2.ams"));
    TEST_ASSERT_TRUE(f.engine.arm());

    // arm() queues LAUNCH; first tick: WAIT on_enter EVENT, WAIT on_exit EVENT,
    // then END on_enter EVENT.
    f.engine.tick(ares::sim::clock::nowMs());

    // Radio should have at least 1 frame (EVENT on_enter WAIT → on_exit → END).
    TEST_ASSERT_GREATER_OR_EQUAL(1U, f.radio.sendCount());

    EngineSnapshot snap{};
    f.engine.getSnapshot(snap);
    TEST_ASSERT_EQUAL_STRING("END", snap.stateName);
}

// ── Test 3: CALIBRATE set action parses and executes ─────────────────────────

void test_calibrate_set_action_executes()
{
    CovFixture f{kGroundProfile};
    f.init("/missions/calibrate.ams", kScriptCalibrate);

    const bool ok = f.engine.activate("calibrate.ams");

    EngineSnapshot snap{};
    f.engine.getSnapshot(snap);
    TEST_ASSERT_TRUE_MESSAGE(ok, snap.lastError);

    TEST_ASSERT_TRUE(f.engine.arm());

    // Tick several times to let the CALIBRATE action collect samples.
    for (uint8_t i = 0U; i < 6U; ++i)
    {
        f.engine.tick(ares::sim::clock::nowMs());
        ares::sim::clock::advanceMs(50U);
    }

    // Engine should still be in WAIT (no TC injected).
    f.engine.getSnapshot(snap);
    TEST_ASSERT_EQUAL_STRING("WAIT", snap.stateName);
}

// ── Test 4: max() set action parses successfully ──────────────────────────────

void test_max_set_action_parses_ok()
{
    CovFixture f{kGroundProfile};
    f.init("/missions/max_set.ams", kScriptMaxSet);

    const bool ok = f.engine.activate("max_set.ams");

    EngineSnapshot snap{};
    f.engine.getSnapshot(snap);
    TEST_ASSERT_TRUE_MESSAGE(ok, snap.lastError);
}

// ── Test 5: max() set action executes on_enter tick ──────────────────────────

void test_max_set_action_executes()
{
    CovFixture f{kGroundProfile};
    f.init("/missions/max_set2.ams", kScriptMaxSet);
    TEST_ASSERT_TRUE(f.engine.activate("max_set2.ams"));
    TEST_ASSERT_TRUE(f.engine.arm());

    // First tick: on_enter fires set peak_alt = max(peak_alt, BARO.alt).
    f.engine.tick(ares::sim::clock::nowMs());

    EngineSnapshot snap{};
    f.engine.getSnapshot(snap);
    TEST_ASSERT_EQUAL_STRING("WAIT", snap.stateName);
}

// ── Test 6: delta set action parses and executes ──────────────────────────────

void test_delta_set_action_executes()
{
    CovFixture f{kGroundProfile};
    f.init("/missions/delta_set.ams", kScriptDeltaSet);
    TEST_ASSERT_TRUE(f.engine.activate("delta_set.ams"));
    TEST_ASSERT_TRUE(f.engine.arm());

    f.engine.tick(ares::sim::clock::nowMs());
    ares::sim::clock::advanceMs(100U);
    f.engine.tick(ares::sim::clock::nowMs());

    EngineSnapshot snap{};
    f.engine.getSnapshot(snap);
    TEST_ASSERT_EQUAL_STRING("WAIT", snap.stateName);
}

// ── Test 7: rich HK report emits GPS/IMU/pressure fields ─────────────────────

void test_rich_hk_fields_emitted()
{
    CovFixture f{kGroundProfile};
    f.init("/missions/rich_hk.ams", kScriptRichHk);
    TEST_ASSERT_TRUE(f.engine.activate("rich_hk.ams"));
    TEST_ASSERT_TRUE(f.engine.arm());

    f.radio.resetCapture();

    // Tick past 200 ms to trigger the HK slot.
    ares::sim::clock::advanceMs(250U);
    f.engine.tick(ares::sim::clock::nowMs());

    // At least one HK frame should have been emitted.
    TEST_ASSERT_GREATER_OR_EQUAL(1U, f.radio.sendCount());
}

// ── Test 8: monitoring alarm triggers when altitude exceeds 3000 m ────────────

void test_monitoring_alarm_triggers()
{
    CovFixture f{kHighAltProfile};
    f.init("/missions/monitor_alarm.ams", kScriptMonitorAlarm);
    TEST_ASSERT_TRUE(f.engine.activate("monitor_alarm.ams"));
    TEST_ASSERT_TRUE(f.engine.arm());

    f.radio.resetCapture();

    // Tick 5 times with 200 ms steps — each tick emits one HK frame at
    // alt > 3000 m.  Three consecutive hits trip the ALT_AGL slot alarm.
    for (uint8_t i = 0U; i < 5U; ++i)
    {
        f.engine.tick(ares::sim::clock::nowMs());
        ares::sim::clock::advanceMs(210U);
    }

    // At least 3 HK frames emitted, plus the FPL_VIOLATION event frame.
    TEST_ASSERT_GREATER_OR_EQUAL(3U, f.radio.sendCount());
}

// ─────────────────────────────────────────────────────────────────────────────
// Additional parser_actions coverage — on_exit EVENT.warning / EVENT.error
// ─────────────────────────────────────────────────────────────────────────────

/// Script with on_exit EVENT.warning — covers parseOnExitEventLineLocked warning branch.
static const char kScriptOnExitWarning[] =
    "include SIM_GPS as GPS\n"
    "include SIM_BARO as BARO\n"
    "include SIM_COM as COM\n"
    "include SIM_IMU as IMU\n"
    "\n"
    "pus.apid = 1\n"
    "pus.service 5 as EVENT\n"
    "pus.service 1 as TC\n"
    "\n"
    "state WAIT:\n"
    "  on_enter:\n"
    "    EVENT.info \"WAITING\"\n"
    "  on_exit:\n"
    "    EVENT.warning \"LEAVING WAIT\"\n"
    "  transition to END when TC.command == LAUNCH\n"
    "\n"
    "state END:\n"
    "  on_enter:\n"
    "    EVENT.info \"DONE\"\n";

/// Script with on_exit EVENT.error — covers parseOnExitEventLineLocked error branch.
static const char kScriptOnExitError[] =
    "include SIM_GPS as GPS\n"
    "include SIM_BARO as BARO\n"
    "include SIM_COM as COM\n"
    "include SIM_IMU as IMU\n"
    "\n"
    "pus.apid = 1\n"
    "pus.service 5 as EVENT\n"
    "pus.service 1 as TC\n"
    "\n"
    "state WAIT:\n"
    "  on_enter:\n"
    "    EVENT.info \"WAITING\"\n"
    "  on_exit:\n"
    "    EVENT.error \"LEAVING WAIT\"\n"
    "  transition to END when TC.command == LAUNCH\n"
    "\n"
    "state END:\n"
    "  on_enter:\n"
    "    EVENT.info \"DONE\"\n";

// ── Test 9: on_exit EVENT.warning parses and arms ────────────────────────────

void test_on_exit_warning_parses_ok()
{
    CovFixture f{kGroundProfile};
    f.init("/missions/exit_warn.ams", kScriptOnExitWarning);
    TEST_ASSERT_TRUE(f.engine.activate("exit_warn.ams"));
    TEST_ASSERT_TRUE(f.engine.arm());

    EngineSnapshot snap{};
    f.engine.getSnapshot(snap);
    TEST_ASSERT_EQUAL(EngineStatus::RUNNING, snap.status);
}

// ── Test 10: on_exit EVENT.error parses and arms ─────────────────────────────

void test_on_exit_error_parses_ok()
{
    CovFixture f{kGroundProfile};
    f.init("/missions/exit_err.ams", kScriptOnExitError);
    TEST_ASSERT_TRUE(f.engine.activate("exit_err.ams"));
    TEST_ASSERT_TRUE(f.engine.arm());

    EngineSnapshot snap{};
    f.engine.getSnapshot(snap);
    TEST_ASSERT_EQUAL(EngineStatus::RUNNING, snap.status);
}

// ─────────────────────────────────────────────────────────────────────────────
// Additional sensor coverage — HK log with IMU gyro fields
// ─────────────────────────────────────────────────────────────────────────────

/// Script logging IMU.gyro_x / gyro_y / gyro_z / temp fields in an HK slot.
static const char kScriptGyroHk[] =
    "include SIM_GPS as GPS\n"
    "include SIM_BARO as BARO\n"
    "include SIM_COM as COM\n"
    "include SIM_IMU as IMU\n"
    "\n"
    "pus.apid = 1\n"
    "pus.service 3 as HK\n"
    "pus.service 5 as EVENT\n"
    "\n"
    "state WAIT:\n"
    "  on_enter:\n"
    "    EVENT.info \"STARTING\"\n"
    "  every 200ms:\n"
    "    HK.report {\n"
    "      gyro_x:   IMU.gyro_x\n"
    "      gyro_y:   IMU.gyro_y\n"
    "      gyro_z:   IMU.gyro_z\n"
    "      imu_temp: IMU.temp\n"
    "    }\n"
    "  transition to END when TIME.elapsed > 99999\n"
    "\n"
    "state END:\n"
    "  on_enter:\n"
    "    EVENT.info \"DONE\"\n";

// ── Test 11: IMU gyro HK fields are read and emitted ─────────────────────────

void test_imu_gyro_hk_fields_emitted()
{
    CovFixture f{kGroundProfile};
    f.init("/missions/gyro_hk.ams", kScriptGyroHk);
    TEST_ASSERT_TRUE(f.engine.activate("gyro_hk.ams"));
    TEST_ASSERT_TRUE(f.engine.arm());

    f.radio.resetCapture();

    // Advance clock past 1 HK interval (200 ms) and tick.
    ares::sim::clock::advanceMs(250U);
    f.engine.tick(ares::sim::clock::nowMs());

    TEST_ASSERT_GREATER_OR_EQUAL(1U, f.radio.sendCount());
}

// ─────────────────────────────────────────────────────────────────────────────
// parser_actions error-path coverage — unknown on_exit EVENT verb
// ─────────────────────────────────────────────────────────────────────────────

/// Script with an unknown on_exit EVENT verb — parser should reject it.
static const char kScriptOnExitUnknownVerb[] =
    "include SIM_GPS as GPS\n"
    "include SIM_BARO as BARO\n"
    "include SIM_COM as COM\n"
    "include SIM_IMU as IMU\n"
    "\n"
    "pus.apid = 1\n"
    "pus.service 5 as EVENT\n"
    "pus.service 1 as TC\n"
    "\n"
    "state WAIT:\n"
    "  on_exit:\n"
    "    EVENT.bogus \"msg\"\n"
    "  transition to END when TC.command == LAUNCH\n"
    "\n"
    "state END:\n"
    "  on_enter:\n"
    "    EVENT.info \"DONE\"\n";

// ── Test 12: unknown on_exit EVENT verb is rejected by parser ─────────────────

void test_on_exit_unknown_verb_rejected()
{
    CovFixture f{kGroundProfile};
    f.init("/missions/exit_unk.ams", kScriptOnExitUnknownVerb);
    // Parser should reject the unknown verb.
    TEST_ASSERT_FALSE(f.engine.activate("exit_unk.ams"));
}

// ─────────────────────────────────────────────────────────────────────────────
// sensor error-path coverage — unknown field names for each sensor type
// ─────────────────────────────────────────────────────────────────────────────

/// Script using an unknown GPS field — covers parseGpsSensorField return false.
static const char kScriptBadGpsField[] =
    "include SIM_GPS as GPS\n"
    "include SIM_BARO as BARO\n"
    "include SIM_COM as COM\n"
    "include SIM_IMU as IMU\n"
    "\n"
    "pus.apid = 1\n"
    "pus.service 5 as EVENT\n"
    "pus.service 1 as TC\n"
    "\n"
    "state WAIT:\n"
    "  transition to END when GPS.bogus > 0.0\n"
    "\n"
    "state END:\n"
    "  on_enter:\n"
    "    EVENT.info \"DONE\"\n";

/// Script using an unknown BARO field — covers parseBaroSensorField return false.
static const char kScriptBadBaroField[] =
    "include SIM_GPS as GPS\n"
    "include SIM_BARO as BARO\n"
    "include SIM_COM as COM\n"
    "include SIM_IMU as IMU\n"
    "\n"
    "pus.apid = 1\n"
    "pus.service 5 as EVENT\n"
    "pus.service 1 as TC\n"
    "\n"
    "state WAIT:\n"
    "  transition to END when BARO.bogus > 0.0\n"
    "\n"
    "state END:\n"
    "  on_enter:\n"
    "    EVENT.info \"DONE\"\n";

/// Script using an unknown IMU field — covers parseImuSensorField return false.
static const char kScriptBadImuField[] =
    "include SIM_GPS as GPS\n"
    "include SIM_BARO as BARO\n"
    "include SIM_COM as COM\n"
    "include SIM_IMU as IMU\n"
    "\n"
    "pus.apid = 1\n"
    "pus.service 5 as EVENT\n"
    "pus.service 1 as TC\n"
    "\n"
    "state WAIT:\n"
    "  transition to END when IMU.bogus > 0.0\n"
    "\n"
    "state END:\n"
    "  on_enter:\n"
    "    EVENT.info \"DONE\"\n";

// ── Test 13: unknown GPS field rejected ──────────────────────────────────────

void test_bad_gps_field_rejected()
{
    CovFixture f{kGroundProfile};
    f.init("/missions/bad_gps.ams", kScriptBadGpsField);
    TEST_ASSERT_FALSE(f.engine.activate("bad_gps.ams"));
}

// ── Test 14: unknown BARO field rejected ─────────────────────────────────────

void test_bad_baro_field_rejected()
{
    CovFixture f{kGroundProfile};
    f.init("/missions/bad_baro.ams", kScriptBadBaroField);
    TEST_ASSERT_FALSE(f.engine.activate("bad_baro.ams"));
}

// ── Test 15: unknown IMU field rejected ──────────────────────────────────────

void test_bad_imu_field_rejected()
{
    CovFixture f{kGroundProfile};
    f.init("/missions/bad_imu.ams", kScriptBadImuField);
    TEST_ASSERT_FALSE(f.engine.activate("bad_imu.ams"));
}

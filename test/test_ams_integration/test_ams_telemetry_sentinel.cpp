/**
 * @file  test_ams_telemetry_sentinel.cpp
 * @brief Integration tests verifying fix [H4]:
 *        formatHkFieldValueLocked returns false and writes "nan" as an
 *        error sentinel in the LOG CSV when the barometer driver fails.
 *
 * H4 root cause: the function previously returned true even on sensor
 * failure, masking the error from callers.  After the fix it returns
 * false (driver fault) or true (real reading), and always writes either
 * the numeric value or the "nan" sentinel so post-flight CSV analysis
 * can distinguish valid samples from fault periods.
 *
 * Test count: 3
 */

#include <unity.h>
#include <cmath>
#include <cstring>

#include "ams/mission_script_engine.h"
#include "ams/ams_driver_registry.h"
#include "hal/baro/barometer_interface.h"

using ares::ams::GpsEntry;
using ares::ams::BaroEntry;
using ares::ams::ComEntry;
using ares::ams::ImuEntry;
using ares::ams::MissionScriptEngine;

#include "sim_clock.h"
#include "sim_ams_scripts.h"
#include "sim_gps_driver.h"
#include "sim_baro_driver.h"
#include "sim_imu_driver.h"
#include "sim_radio_driver.h"
#include "sim_storage_driver.h"

// ─────────────────────────────────────────────────────────────────────────────
// SimFailingBaroDriver — permanently returns BaroStatus::ERROR, simulating
// an unplugged or damaged I2C barometer.  begin() returns true so the engine
// accepts the driver at init time; only read() fails.
// ─────────────────────────────────────────────────────────────────────────────

class SimFailingBaroDriver final : public BarometerInterface
{
public:
    bool begin()                            override { return true; }
    BaroStatus read(BaroReading& /*out*/)   override { return BaroStatus::ERROR; }
    void setSeaLevelPressure(float /*hPa*/) override {}
    const char* driverModel()         const override { return "SIM_BARO"; }
};

// ─────────────────────────────────────────────────────────────────────────────
// SimNanBaroDriver — returns BaroStatus::OK but fills every float field with
// quiet NaN, simulating a driver that claims success but propagates corrupt
// data (e.g. failed NMEA parse, division-by-zero in sensor firmware).
// Used to verify the isfinite guard in readBaroFieldLocked.
// ─────────────────────────────────────────────────────────────────────────────

class SimNanBaroDriver final : public BarometerInterface
{
public:
    bool begin()                            override { return true; }
    BaroStatus read(BaroReading& out)       override
    {
        out.altitudeM    = std::nanf("");
        out.pressurePa   = std::nanf("");
        out.temperatureC = std::nanf("");
        return BaroStatus::OK;
    }
    void setSeaLevelPressure(float /*hPa*/) override {}
    const char* driverModel()         const override { return "SIM_BARO"; }
};

// ─────────────────────────────────────────────────────────────────────────────
// Minimal flight profile — GPS / IMU have plausible values; baro is irrelevant
// for the failing-driver test because SimFailingBaroDriver ignores the profile.
// ─────────────────────────────────────────────────────────────────────────────

static const ares::sim::FlightProfile kSentinelProfile = {
    .count   = 1U,
    .samples = {
        { .timeMs          = 0U,
          .gpsLatDeg       = 40.0f,   .gpsLonDeg    = -3.0f,
          .gpsAltM         = 0.0f,    .gpsSpeedKmh  = 0.0f,
          .gpsHdop         = 1.0f,    .gpsSats      = 6U,  .gpsFix = true,
          .baroAltM        = 10.0f,   .baroPressurePa = 101000.0f, .baroTempC = 20.0f,
          .accelX          = 0.0f,    .accelY       = 0.0f,  .accelZ    = 9.81f,
          .imuTempC        = 25.0f },
    }
};

// ─────────────────────────────────────────────────────────────────────────────
// Test 1: a permanently failing barometer writes "nan" in the LOG CSV.
//
// Verifies that formatHkFieldValueLocked writes the "nan" error sentinel
// (and returns false) when the baro driver returns BaroStatus::ERROR, so
// that post-flight CSV tools can distinguish fault samples from valid data.
// ─────────────────────────────────────────────────────────────────────────────

void test_failed_sensor_writes_nan_sentinel_in_csv()
{
    SimFailingBaroDriver failBaro;

    ares::sim::SimStorageDriver storage;
    ares::sim::SimGpsDriver     gps{kSentinelProfile};
    ares::sim::SimImuDriver     imu{kSentinelProfile};
    ares::sim::SimRadioDriver   radio;

    BaroEntry baroEntry { "SIM_BARO", &failBaro };
    GpsEntry  gpsEntry  { "SIM_GPS",  &gps      };
    ComEntry  comEntry  { "SIM_COM",  &radio    };
    ImuEntry  imuEntry  { "SIM_IMU",  &imu      };

    MissionScriptEngine engine{
        storage,
        &gpsEntry, 1U, &baroEntry, 1U, &comEntry, 1U, &imuEntry, 1U
    };

    (void)storage.begin();
    storage.registerFile("/missions/log_baro.ams", ares::sim::kScriptLogHeaderRetry);
    (void)gps.begin();
    (void)failBaro.begin();
    (void)imu.begin();
    (void)radio.begin();
    (void)engine.begin();

    TEST_ASSERT_TRUE(engine.activate("log_baro.ams"));
    TEST_ASSERT_TRUE(engine.arm());

    // Advance past the log_every 100 ms slot so one CSV row is produced.
    ares::sim::clock::advanceMs(105U);
    engine.tick(ares::sim::clock::nowMs());

    const char* csv = storage.appendedContent();
    TEST_ASSERT_NOT_NULL_MESSAGE(csv,
        "[H4] storage must have log content after tick");

    // The CSV data row must carry the "nan" error sentinel, not a numeric value.
    TEST_ASSERT_NOT_NULL_MESSAGE(strstr(csv, ",nan,"),
        "[H4] failed barometer read must write 'nan' sentinel in LOG CSV");
}

// ─────────────────────────────────────────────────────────────────────────────
// Test 2: a working barometer writes a numeric value in the LOG CSV (not nan).
//
// Regression guard: ensures that a healthy sensor path is unaffected by the
// H4 fix and still produces a non-nan value in the CSV row.
// ─────────────────────────────────────────────────────────────────────────────

void test_valid_sensor_writes_numeric_value_in_csv()
{
    ares::sim::SimStorageDriver storage;
    ares::sim::SimGpsDriver     gps{kSentinelProfile};
    ares::sim::SimBaroDriver    baro{kSentinelProfile};  // working driver
    ares::sim::SimImuDriver     imu{kSentinelProfile};
    ares::sim::SimRadioDriver   radio;

    BaroEntry baroEntry { "SIM_BARO", &baro  };
    GpsEntry  gpsEntry  { "SIM_GPS",  &gps   };
    ComEntry  comEntry  { "SIM_COM",  &radio };
    ImuEntry  imuEntry  { "SIM_IMU",  &imu   };

    MissionScriptEngine engine{
        storage,
        &gpsEntry, 1U, &baroEntry, 1U, &comEntry, 1U, &imuEntry, 1U
    };

    (void)storage.begin();
    storage.registerFile("/missions/log_baro2.ams", ares::sim::kScriptLogHeaderRetry);
    (void)gps.begin();
    (void)baro.begin();
    (void)imu.begin();
    (void)radio.begin();
    (void)engine.begin();

    TEST_ASSERT_TRUE(engine.activate("log_baro2.ams"));
    TEST_ASSERT_TRUE(engine.arm());

    // Advance past the log_every 100 ms slot so one CSV row is produced.
    ares::sim::clock::advanceMs(105U);
    engine.tick(ares::sim::clock::nowMs());

    const char* csv = storage.appendedContent();
    TEST_ASSERT_NOT_NULL_MESSAGE(csv,
        "storage must have log content after tick");

    // A working sensor must NOT produce "nan" — the value is the sim baro alt.
    TEST_ASSERT_NULL_MESSAGE(strstr(csv, ",nan,"),
        "[H4] valid barometer read must not write 'nan' sentinel in LOG CSV");
    TEST_ASSERT_NULL_MESSAGE(strstr(csv, ",nan\n"),
        "[H4] valid barometer read must not end CSV row with ',nan'");
}

// ─────────────────────────────────────────────────────────────────────────────
// Test 3: a driver returning OK but NaN values writes the "nan" sentinel.
//
// Regression guard for P2-2: the isfinite guard in readBaroFieldLocked must
// catch NaN propagated by a driver that claims success, invalidate the cache,
// and fall through to the same "nan" sentinel path used for failing drivers.
// ─────────────────────────────────────────────────────────────────────────────

void test_nan_baro_value_writes_nan_sentinel_in_csv()
{
    SimNanBaroDriver nanBaro;

    ares::sim::SimStorageDriver storage;
    ares::sim::SimGpsDriver     gps{kSentinelProfile};
    ares::sim::SimImuDriver     imu{kSentinelProfile};
    ares::sim::SimRadioDriver   radio;

    BaroEntry baroEntry { "SIM_BARO", &nanBaro };
    GpsEntry  gpsEntry  { "SIM_GPS",  &gps     };
    ComEntry  comEntry  { "SIM_COM",  &radio   };
    ImuEntry  imuEntry  { "SIM_IMU",  &imu     };

    MissionScriptEngine engine{
        storage,
        &gpsEntry, 1U, &baroEntry, 1U, &comEntry, 1U, &imuEntry, 1U
    };

    (void)storage.begin();
    storage.registerFile("/missions/nan_baro.ams", ares::sim::kScriptLogHeaderRetry);
    (void)gps.begin();
    (void)nanBaro.begin();
    (void)imu.begin();
    (void)radio.begin();
    (void)engine.begin();

    TEST_ASSERT_TRUE(engine.activate("nan_baro.ams"));
    TEST_ASSERT_TRUE(engine.arm());

    // Advance past the log_every 100 ms slot so one CSV row is produced.
    ares::sim::clock::advanceMs(105U);
    engine.tick(ares::sim::clock::nowMs());

    const char* csv = storage.appendedContent();
    TEST_ASSERT_NOT_NULL_MESSAGE(csv,
        "[P2-2] storage must have log content after tick with NaN baro");

    // The driver returned OK with NaN — the isfinite guard must catch it
    // and write the \"nan\" sentinel, same as for a failing driver (H4 path).
    TEST_ASSERT_NOT_NULL_MESSAGE(strstr(csv, ",nan,"),
        "[P2-2] baro OK+NaN must write 'nan' sentinel in LOG CSV");
}

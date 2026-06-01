/**
 * @file  test_bmp280_begin.cpp
 * @brief Unit tests for Bmp280Driver::begin().
 *
 * Tests:
 *   1. Happy path: chip ID = 0x58, status NVM done, calibration OK → begin() true.
 *   2. Wrong chip ID: driver returns 0x00 → begin() false.
 *   3. I2C NACK on REG_STATUS during NVM poll → begin() false.
 *
 * ScriptedWire is a TwoWire subclass that scripts the BMP280 I2C register
 * map by tracking the last register address written and returning canned
 * responses in read().  No hardware is required.
 *
 * I2C transaction model used by Bmp280Driver:
 *   readRegs: beginTransmission → write(reg) → endTransmission(false)
 *             → requestFrom(addr, len) → read() × len
 *   writeReg: beginTransmission → write(reg) → write(val) → endTransmission()
 */
#include <unity.h>

#include "drivers/baro/bmp280_driver.h"

#include <cstring>

// ── ScriptedWire ─────────────────────────────────────────────────────────────

/**
 * @brief Scripted TwoWire for BMP280 begin() tests.
 *
 * @param failStatusRead  When true, endTransmission(false) returns 1 (NACK)
 *                        when REG_STATUS (0xF3) is the pending register.
 * @param wrongChipId     When true, REG_CHIP_ID (0xD0) returns 0x00 instead
 *                        of 0x58, causing begin() to reject the device.
 */
class ScriptedWire final : public TwoWire
{
public:
    ScriptedWire(bool failStatusRead, bool wrongChipId, bool provideReadData = false)
        : failStatusRead_(failStatusRead)
        , wrongChipId_(wrongChipId)
        , provideReadData_(provideReadData)
    {
        (void)memset(readBuf_, 0, sizeof(readBuf_));
    }

    void beginTransmission(uint8_t /*addr*/) override
    {
        lastReg_    = 0U;
        writeCount_ = 0;
        readReady_  = false;
        readPos_    = 0U;
        readLen_    = 0U;
    }

    int write(uint8_t data) override
    {
        if (writeCount_ == 0) { lastReg_ = data; }
        writeCount_++;
        return 1;
    }

    // Write-stop: used by writeReg (RESET, CONFIG, CTRL_MEAS).
    uint8_t endTransmission() override { return 0U; }

    // Repeated-start: used by readRegs address phase.
    uint8_t endTransmission(bool /*stop*/) override
    {
        // Simulate I2C NACK for REG_STATUS when failStatusRead_ is set.
        if (failStatusRead_ && lastReg_ == 0xF3U)
        {
            return 1U;  // NACK → readRegs returns false → readReg returns false
        }

        // Prepare read buffer for the requested register.
        (void)memset(readBuf_, 0, sizeof(readBuf_));
        readPos_ = 0U;
        readLen_ = 0U;

        switch (lastReg_)
        {
            case 0xD0U:  // REG_CHIP_ID
                readBuf_[0] = wrongChipId_ ? 0x00U : 0x58U;
                readLen_    = 1U;
                break;

            case 0xF3U:  // REG_STATUS — bit 0 = im_update = 0 → NVM load done
                readBuf_[0] = 0x00U;
                readLen_    = 1U;
                break;

            case 0x88U:
            {
                // REG_CALIB — 24 bytes, little-endian 16-bit pairs.
                // T1=27504 (u16), T2=26435 (s16), T3=0 (s16),
                // P1=36477 (u16, must be > 0 for compensatePressure),
                // P2..P9 = 0.
                constexpr uint16_t kT1 = 27504U;
                constexpr uint16_t kT2 = static_cast<uint16_t>(static_cast<int16_t>(26435));
                constexpr uint16_t kP1 = 36477U;
                readBuf_[0] = static_cast<uint8_t>(kT1 & 0xFFU);
                readBuf_[1] = static_cast<uint8_t>(kT1 >> 8U);
                readBuf_[2] = static_cast<uint8_t>(kT2 & 0xFFU);
                readBuf_[3] = static_cast<uint8_t>(kT2 >> 8U);
                // T3 bytes [4..5] = 0.
                readBuf_[6] = static_cast<uint8_t>(kP1 & 0xFFU);
                readBuf_[7] = static_cast<uint8_t>(kP1 >> 8U);
                // P2..P9 bytes [8..23] = 0.
                readLen_ = 24U;
                break;
            }

            case 0xF7U:  // REG_PRESS_MSB — 6-byte burst (press[0..2], temp[3..5])
            {
                if (provideReadData_)
                {
                    // rawPress = 457218 (≈101353 Pa at sea level with our calib)
                    //   buf[0] = rawPress >> 12 = 0x6F
                    //   buf[1] = (rawPress >> 4) & 0xFF = 0xA0
                    //   buf[2] = (rawPress & 0xF) << 4  = 0x20
                    // rawTemp = 519888 (≈25.15 °C with T1=27504, T2=26435, T3=0)
                    //   buf[3] = rawTemp >> 12 = 0x7E
                    //   buf[4] = (rawTemp >> 4) & 0xFF = 0xED
                    //   buf[5] = (rawTemp & 0xF) << 4  = 0x00
                    readBuf_[0] = 0x6FU;
                    readBuf_[1] = 0xA0U;
                    readBuf_[2] = 0x20U;
                    readBuf_[3] = 0x7EU;
                    readBuf_[4] = 0xEDU;
                    readBuf_[5] = 0x00U;
                    readLen_    = 6U;
                }
                break;
            }

            default:
                // Unknown register — 0 bytes available; readRegs will get
                // requestFrom(addr, len) != len and return false.
                readLen_ = 0U;
                break;
        }

        readReady_ = true;
        return 0U;
    }

    uint8_t requestFrom(uint8_t /*addr*/, uint8_t len) override
    {
        return readReady_ ? len : 0U;
    }

    int read() override
    {
        if (readPos_ < readLen_) { return static_cast<int>(readBuf_[readPos_++]); }
        return 0;  // Return 0 for any reads beyond readLen_.
    }

private:
    bool    failStatusRead_;
    bool    wrongChipId_;
    bool    provideReadData_;

    uint8_t lastReg_    = 0U;
    int     writeCount_ = 0;
    uint8_t readBuf_[32];
    uint8_t readLen_    = 0U;
    uint8_t readPos_    = 0U;
    bool    readReady_  = false;
};

// ── Tests ─────────────────────────────────────────────────────────────────────

/**
 * Happy path: correct chip ID (0x58), status register NVM done (0x00),
 * valid calibration data.  begin() must return true.
 */
void test_bmp280_begin_happy_path()
{
    ScriptedWire wire(false, false);
    Bmp280Driver drv(wire, 0x76U);
    TEST_ASSERT_TRUE(drv.begin());
}

/**
 * Wrong chip ID: REG_CHIP_ID returns 0x00 (not 0x58 or 0x60).
 * begin() must return false.
 */
void test_bmp280_begin_wrong_chip_id_rejected()
{
    ScriptedWire wire(false, true);
    Bmp280Driver drv(wire, 0x76U);
    TEST_ASSERT_FALSE(drv.begin());
}

/**
 * I2C NACK on REG_STATUS during NVM-load poll (P3-6 fix).
 *
 * Before the fix: (void)readReg(...) silently ignored the NACK, status
 * remained 0x00 → bit 0 = 0 → loop exited as if NVM were done.  The
 * silent continue masked a hardware fault.
 *
 * After the fix: readReg failure causes begin() to return false immediately,
 * surfacing the bus error to the caller.
 */
void test_bmp280_begin_status_reg_i2c_fail_returns_false()
{
    ScriptedWire wire(true, false);  // fail REG_STATUS read; correct chip ID
    Bmp280Driver drv(wire, 0x76U);
    TEST_ASSERT_FALSE(drv.begin());
}

/**
 * read() before begin(): driver must return BaroStatus::NOT_READY.
 */
void test_bmp280_read_not_ready()
{
    ScriptedWire wire(false, false);
    Bmp280Driver drv(wire, 0x76U);
    BaroReading r;
    TEST_ASSERT_EQUAL_INT(static_cast<int>(BaroStatus::NOT_READY),
                          static_cast<int>(drv.read(r)));
}

/**
 * Happy path read(): after successful begin(), read() must return OK with
 * finite temperature (~25 °C) and pressure (~101353 Pa).
 *
 * Raw values encoded for ScriptedWire (provideReadData=true):
 *   rawPress = 457218  → T1=27504, T2=26435, T3=0, P1=36477, P2..P9=0
 *   rawTemp  = 519888
 */
void test_bmp280_read_happy_path()
{
    ScriptedWire wire(false, false, true);  // provide burst read data
    Bmp280Driver drv(wire, 0x76U);
    TEST_ASSERT_TRUE(drv.begin());

    BaroReading r;
    TEST_ASSERT_EQUAL_INT(static_cast<int>(BaroStatus::OK),
                          static_cast<int>(drv.read(r)));
    // Temperature expected ≈25.15 °C (±0.5 °C tolerance)
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 25.15f, r.temperatureC);
    // Pressure expected ≈101353 Pa (±2000 Pa tolerance)
    TEST_ASSERT_FLOAT_WITHIN(2000.0f, 101353.0f, r.pressurePa);
    // Altitude near sea level with default sea-level pressure (±100 m tolerance)
    TEST_ASSERT_FLOAT_WITHIN(100.0f, 0.0f, r.altitudeM);
}

/**
 * setSeaLevelPressure(): valid input updates altitude baseline;
 * invalid inputs (negative, zero, NaN) are silently rejected.
 */
void test_bmp280_set_sea_level_pressure()
{
    ScriptedWire wire(false, false, true);
    Bmp280Driver drv(wire, 0x76U);
    TEST_ASSERT_TRUE(drv.begin());

    // Valid update: 1020 hPa — our measured pressure (~1013.5 hPa) is below
    // the new sea-level reference, so altitude must be positive.
    drv.setSeaLevelPressure(1020.0f);
    BaroReading r1;
    TEST_ASSERT_EQUAL_INT(static_cast<int>(BaroStatus::OK),
                          static_cast<int>(drv.read(r1)));
    TEST_ASSERT_GREATER_THAN_FLOAT(0.0f, r1.altitudeM);

    // Invalid inputs must not crash and must leave the previous value intact.
    drv.setSeaLevelPressure(-10.0f);
    drv.setSeaLevelPressure(0.0f);
    // NaN
    drv.setSeaLevelPressure(0.0f / 0.0f);
    BaroReading r2;
    // sea-level pressure unchanged — result should equal r1
    TEST_ASSERT_EQUAL_INT(static_cast<int>(BaroStatus::OK),
                          static_cast<int>(drv.read(r2)));
    TEST_ASSERT_FLOAT_WITHIN(1.0f, r1.altitudeM, r2.altitudeM);
}

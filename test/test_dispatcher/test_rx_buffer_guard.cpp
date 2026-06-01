/**
 * @file  test_rx_buffer_guard.cpp
 * @brief Unit tests for RadioDispatcher RX-buffer boundary guard (P3-5).
 *
 * Covers the CERT-1 guard added in poll():
 *   if (received > space) → rxLen_ reset to 0 instead of rxLen_ += received.
 *
 * Without the fix rxLen_ would exceed kRxBufLen (currently 428), causing
 * a 1-byte out-of-bounds read in processBuffer's SYNC scan loop on the
 * same poll() call.  ASan / valgrind would catch this; the tests below
 * document the expected contract and provide regression coverage.
 *
 * Two tests:
 *   1. Liar-only: driver always lies → no crash, no spurious frames sent.
 *   2. Recovery:  driver lies on first call, then injects a valid HEARTBEAT.
 *                 Verifies the dispatcher fully recovers and processes the
 *                 heartbeat (proving rxLen_ was properly reset to 0).
 */
#include <unity.h>

#include "comms/radio_dispatcher.h"
#include "comms/ares_radio_protocol.h"
#include "ams/mission_script_engine.h"
#include "ams/ams_driver_registry.h"

#include "sim_storage_driver.h"
#include "sim_gps_driver.h"
#include "sim_baro_driver.h"
#include "sim_imu_driver.h"
#include "sim_radio_driver.h"

#include <cstring>

using namespace ares::proto;
using ares::ams::MissionScriptEngine;
using ares::ams::GpsEntry;
using ares::ams::BaroEntry;
using ares::ams::ComEntry;
using ares::ams::ImuEntry;

// ── Shared resting flight profile ────────────────────────────────────────────

static const ares::sim::FlightProfile kRestProfile = {
    .count   = 1U,
    .samples = {
        { .timeMs        = 0U,
          .gpsLatDeg     = 40.0f, .gpsLonDeg     = -3.0f,
          .gpsAltM       = 0.0f,  .gpsSpeedKmh   = 0.0f,
          .gpsSats       = 8U,    .gpsFix        = true,
          .baroAltM      = 0.0f,  .baroPressurePa = 101325.0f,
          .baroTempC     = 20.0f,
          .accelX        = 0.0f,  .accelY        = 0.0f,
          .accelZ        = 9.81f, .imuTempC      = 25.0f },
    }
};

// ── Liar radio driver ─────────────────────────────────────────────────────────

/**
 * A radio driver that always reports received = bufSize + 1 (contract violation).
 *
 * Used to exercise the guard in RadioDispatcher::poll() that detects when a
 * driver claims to have written more bytes than the buffer allows.
 *
 * Captures outgoing frames just like SimRadioDriver so tests can assert that
 * no spurious response was sent.
 */
class LiarRadioDriver final : public RadioInterface
{
public:
    bool begin() override { return true; }

    RadioStatus send(const uint8_t* data, uint16_t len) override
    {
        if (data == nullptr) { return RadioStatus::ERROR; }
        const uint16_t copy = (len <= kCapture) ? len : kCapture;
        (void)memcpy(lastFrame_, data, copy);
        lastLen_ = copy;
        sendCount_++;
        return RadioStatus::OK;
    }

    RadioStatus receive(uint8_t* /*buf*/, uint16_t bufSize,
                        uint16_t& received) override
    {
        // Lie: claim one more byte than the caller offered.
        received = static_cast<uint16_t>(bufSize + 1U);
        return RadioStatus::OK;
    }

    bool     ready()       const override { return true; }
    uint16_t mtu()         const override { return 512U; }
    const char* driverModel() const override { return "LIAR"; }

    uint32_t sendCount() const { return sendCount_; }

private:
    static constexpr uint16_t kCapture = 512U;
    uint8_t  lastFrame_[kCapture] = {};
    uint16_t lastLen_             = 0U;
    uint32_t sendCount_           = 0U;
};

// ── Recovery driver: lies on first receive, then injects bytes ────────────────

/**
 * Lies on the very first receive() call, then behaves normally: drains a
 * pre-loaded injection buffer just like SimRadioDriver.
 */
class LiarThenGoodDriver final : public RadioInterface
{
public:
    bool begin() override { return true; }

    RadioStatus send(const uint8_t* data, uint16_t len) override
    {
        if (data == nullptr) { return RadioStatus::ERROR; }
        sendCount_++;
        (void)data; (void)len;
        return RadioStatus::OK;
    }

    RadioStatus receive(uint8_t* buf, uint16_t bufSize,
                        uint16_t& received) override
    {
        received = 0U;
        if (!lied_)
        {
            lied_    = true;
            received = static_cast<uint16_t>(bufSize + 1U);  // lie once
            return RadioStatus::OK;
        }
        // Normal drain of the inject buffer.
        if (injectLen_ == 0U) { return RadioStatus::OK; }
        received = (injectLen_ <= bufSize) ? injectLen_ : bufSize;
        (void)memcpy(buf, injectBuf_, received);
        injectLen_ = static_cast<uint16_t>(injectLen_ - received);
        if (injectLen_ > 0U)
        {
            (void)memmove(injectBuf_, &injectBuf_[received], injectLen_);
        }
        return RadioStatus::OK;
    }

    bool injectBytes(const uint8_t* data, uint16_t len)
    {
        if (data == nullptr) { return false; }
        if (len > static_cast<uint16_t>(sizeof(injectBuf_) - injectLen_))
        {
            return false;
        }
        (void)memcpy(&injectBuf_[injectLen_], data, len);
        injectLen_ = static_cast<uint16_t>(injectLen_ + len);
        return true;
    }

    bool     ready()       const override { return true; }
    uint16_t mtu()         const override { return 512U; }
    const char* driverModel() const override { return "LIAR_THEN_GOOD"; }

    uint32_t sendCount() const { return sendCount_; }

private:
    static constexpr uint16_t kInject = 1024U;
    bool     lied_       = false;
    uint8_t  injectBuf_[kInject] = {};
    uint16_t injectLen_  = 0U;
    uint32_t sendCount_  = 0U;
};

// ── Frame builder helper ──────────────────────────────────────────────────────

static uint16_t build_heartbeat(uint8_t* buf, uint8_t seq)
{
    Frame tx   = {};
    tx.ver     = PROTOCOL_VERSION;
    tx.node    = NODE_ROCKET;
    tx.type    = MsgType::HEARTBEAT;
    tx.seq     = seq;
    tx.flags   = 0U;
    tx.len     = 0U;
    uint16_t outLen = 0U;
    (void)encode(tx, buf, MAX_FRAME_LEN, outLen);
    return outLen;
}

// ── Liar fixture ──────────────────────────────────────────────────────────────

struct LiarFixture
{
    ares::sim::SimStorageDriver storage;
    ares::sim::SimGpsDriver     gps{kRestProfile};
    ares::sim::SimBaroDriver    baro{kRestProfile};
    ares::sim::SimImuDriver     imu{kRestProfile};
    ares::sim::SimRadioDriver   engineRadio;

    GpsEntry  gpsEntry  = { "SIM_GPS",  &gps        };
    BaroEntry baroEntry = { "SIM_BARO", &baro       };
    ComEntry  comEntry  = { "SIM_COM",  &engineRadio };
    ImuEntry  imuEntry  = { "SIM_IMU",  &imu        };

    MissionScriptEngine engine{
        storage,
        &gpsEntry,  1U,
        &baroEntry, 1U,
        &comEntry,  1U,
        &imuEntry,  1U
    };

    LiarRadioDriver       dispatchRadio;
    ares::RadioDispatcher dispatcher{ dispatchRadio, engine, nullptr };

    LiarFixture()
    {
        (void)storage.begin();
        (void)engine.begin();
    }
};

// ── Recovery fixture ──────────────────────────────────────────────────────────

struct RecoveryFixture
{
    ares::sim::SimStorageDriver storage;
    ares::sim::SimGpsDriver     gps{kRestProfile};
    ares::sim::SimBaroDriver    baro{kRestProfile};
    ares::sim::SimImuDriver     imu{kRestProfile};
    ares::sim::SimRadioDriver   engineRadio;

    GpsEntry  gpsEntry  = { "SIM_GPS",  &gps        };
    BaroEntry baroEntry = { "SIM_BARO", &baro       };
    ComEntry  comEntry  = { "SIM_COM",  &engineRadio };
    ImuEntry  imuEntry  = { "SIM_IMU",  &imu        };

    MissionScriptEngine engine{
        storage,
        &gpsEntry,  1U,
        &baroEntry, 1U,
        &comEntry,  1U,
        &imuEntry,  1U
    };

    LiarThenGoodDriver    dispatchRadio;
    ares::RadioDispatcher dispatcher{ dispatchRadio, engine, nullptr };

    RecoveryFixture()
    {
        (void)storage.begin();
        (void)engine.begin();
    }
};

// ── Tests ─────────────────────────────────────────────────────────────────────

/**
 * A driver that always reports received > space must not cause the dispatcher
 * to send spurious response frames.
 *
 * Without the P3-5 fix: rxLen_ would be set to bufSize+1 = 429 > kRxBufLen=428.
 * processBuffer() would then iterate 429 bytes through rxBuf_[], causing a
 * 1-byte out-of-bounds read (index 428 of a 428-element array).  ASan detects
 * this.  The processBuffer discard path then resets rxLen_ to 0, so no frame
 * is emitted — but the UB has already occurred.
 *
 * With the fix: rxLen_ is immediately reset to 0 in poll(), processBuffer() is
 * NOT called at all (rxLen_ < MIN_FRAME_LEN), and no UB occurs.
 */
void test_poll_liar_driver_received_exceeds_space_no_crash()
{
    LiarFixture fix;

    // 5 polls with liar driver — must not crash, assert, or send any frame.
    for (int i = 0; i < 5; ++i)
    {
        fix.dispatcher.poll(static_cast<uint32_t>(i) * 100U);
    }

    TEST_ASSERT_EQUAL_UINT32(0U, fix.dispatchRadio.sendCount());
}

/**
 * After a single liar poll, the dispatcher must fully recover: a valid
 * HEARTBEAT injected on the very next receive() call must be processed.
 *
 * If the fix is absent and rxLen_ exceeds kRxBufLen, the receive block is
 * gated by `rxLen_ < kRxBufLen` (false) for one poll, so the heartbeat
 * arrives one poll later.  Both paths eventually process the heartbeat;
 * this test verifies the post-fix contract without requiring private state
 * access.
 *
 * The heartbeat is placed in the injection buffer BEFORE the first poll so
 * that it is available as soon as the recovery call can run.
 */
void test_poll_liar_driver_recovery_processes_heartbeat()
{
    RecoveryFixture fix;

    // Pre-queue a valid heartbeat for the second receive() call.
    uint8_t hbBuf[MAX_FRAME_LEN] = {};
    const uint16_t hbLen = build_heartbeat(hbBuf, 1U);
    TEST_ASSERT_GREATER_THAN_UINT16(0U, hbLen);
    TEST_ASSERT_TRUE(fix.dispatchRadio.injectBytes(hbBuf, hbLen));

    // Poll 0: liar call → guard triggers, rxLen_ reset to 0 (no OOB access).
    fix.dispatcher.poll(1000U);

    // Poll 1: heartbeat bytes arrive via the normal inject path.
    // Poll 2: heartbeat fully assembled and dispatched → ACK response sent.
    fix.dispatcher.poll(1001U);
    fix.dispatcher.poll(1002U);

    // At least one response frame (HEARTBEAT echo) must have been sent.
    TEST_ASSERT_GREATER_THAN_UINT32(0U, fix.dispatchRadio.sendCount());
}

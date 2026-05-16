/**
 * @file  test_fragment_reassembly.cpp
 * @brief Unit tests for RadioDispatcher fragment reassembly and session management.
 *
 * Tests the APUS-15 fragmented-transfer receive path through
 * RadioDispatcher::poll() using SimRadioDriver byte injection.
 *
 * Covers:
 *   - Two-segment transfer reassembles cleanly and executes the assembled command.
 *   - Orphaned single-segment transfer produces no phantom command execution.
 *   - Timed-out session is discarded on the next fragment arrival (APUS-15.4).
 *   - Session collision (different transferId while one is active) → NACK QUEUE_FULL.
 */
#include <unity.h>

#include "comms/radio_dispatcher.h"
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

// ── Shared flat flight profile (resting values) ──────────────────────────────

static const ares::sim::FlightProfile kRestProfile = {
    .count   = 1U,
    .samples = {
        { .timeMs        = 0U,
          .gpsLatDeg     = 40.0f,  .gpsLonDeg    = -3.0f,
          .gpsAltM       = 0.0f,   .gpsSpeedKmh  = 0.0f,
          .gpsSats       = 8U,     .gpsFix       = true,
          .baroAltM      = 0.0f,   .baroPressurePa = 101325.0f,
          .baroTempC     = 20.0f,
          .accelX        = 0.0f,   .accelY       = 0.0f,
          .accelZ        = 9.81f,  .imuTempC     = 25.0f },
    }
};

// ── Test fixture ─────────────────────────────────────────────────────────────

/**
 * Provides a fully initialised RadioDispatcher backed by a MissionScriptEngine
 * running on sim hardware drivers.
 *
 * `dispatchRadio` is the transport-layer radio used by the dispatcher (supports
 * byte injection via injectBytes()).  `engineRadio` is the AMS COM-driver used
 * by the engine for AMS SEND/RECEIVE operations (kept separate to avoid
 * conflating telemetry TX with dispatcher ACK TX).
 */
struct DispatchFixture
{
    ares::sim::SimStorageDriver storage;
    ares::sim::SimGpsDriver     gps{kRestProfile};
    ares::sim::SimBaroDriver    baro{kRestProfile};
    ares::sim::SimImuDriver     imu{kRestProfile};
    ares::sim::SimRadioDriver   engineRadio;   ///< Engine's COM-layer radio driver.

    GpsEntry  gpsEntry  = { "SIM_GPS",  &gps       };
    BaroEntry baroEntry = { "SIM_BARO", &baro      };
    ComEntry  comEntry  = { "SIM_COM",  &engineRadio };
    ImuEntry  imuEntry  = { "SIM_IMU",  &imu       };

    MissionScriptEngine engine{
        storage,
        &gpsEntry,  1U,
        &baroEntry, 1U,
        &comEntry,  1U,
        &imuEntry,  1U
    };

    ares::sim::SimRadioDriver dispatchRadio;   ///< Dispatcher transport radio.
    ares::RadioDispatcher     dispatcher{ dispatchRadio, engine, nullptr };

    DispatchFixture()
    {
        (void)storage.begin();
        (void)engine.begin();
        (void)dispatchRadio.begin();
    }
};

// ── Helper ───────────────────────────────────────────────────────────────────

/**
 * Build one wire-encoded COMMAND fragment and store it in @p buf.
 *
 * @param[out] buf          Buffer of at least MAX_FRAME_LEN bytes.
 * @param[in]  seq          Frame sequence number.
 * @param[in]  transferId   Transfer session identifier.
 * @param[in]  segNum       Segment index (0-based).
 * @param[in]  totalSegs    Total segments in this transfer.
 * @param[in]  data         Segment payload bytes (may be nullptr if dataLen==0).
 * @param[in]  dataLen      Number of bytes in @p data.
 * @return Wire frame length (0 on encoding error).
 */
static uint16_t make_cmd_frag(uint8_t*       buf,
                               uint8_t        seq,
                               uint16_t       transferId,
                               uint16_t       segNum,
                               uint16_t       totalSegs,
                               const uint8_t* data,
                               uint8_t        dataLen)
{
    Frame tx = {};
    tx.ver  = PROTOCOL_VERSION;
    tx.node = NODE_ROCKET;  // destination = rocket (ground → rocket command)
    tx.type = MsgType::COMMAND;
    tx.seq  = seq;
    if (!encodeFrag(tx, transferId, segNum, totalSegs, data, dataLen))
    {
        return 0U;
    }
    return encode(tx, buf, MAX_FRAME_LEN);
}

// ── Tests ─────────────────────────────────────────────────────────────────────

/**
 * A two-segment COMMAND transfer must reassemble cleanly and execute the
 * assembled command.
 *
 * Transfer layout:
 *   payload = CommandHeader{ priority=PRI_LOW(3), commandId=REQUEST_STATUS(0x20) }
 *   seg 0/2 — carries the priority byte (1 byte)
 *   seg 1/2 — carries the commandId byte (1 byte)
 *
 * Expected behaviour (APUS-15.5):
 *   After seg 0: one per-segment ACK sent.
 *   After seg 1: one per-segment ACK sent, assembly completes, REQUEST_STATUS
 *                executes → dispatcher sends a TELEMETRY response frame.
 *   Total sendCount > 2 after both polls.
 */
void test_frag_two_segments_reassemble()
{
    DispatchFixture f;

    // The reassembly buffer uses a fixed stride of kFragSegDataSize (194) bytes
    // per segment.  To produce a valid CommandHeader at payload[0..1], both
    // bytes must travel inside seg 0 (at offset 0).  Seg 1 carries a dummy
    // byte at offset 194; the 3-byte assembled payload still yields
    // payload[0]=PRI_LOW, payload[1]=REQUEST_STATUS.
    const uint8_t seg0data[2] = {
        static_cast<uint8_t>(Priority::PRI_LOW),
        static_cast<uint8_t>(CommandId::REQUEST_STATUS)
    };
    const uint8_t seg1data[1] = { 0x00U };  // dummy — triggers assembly completion

    uint8_t wire0[MAX_FRAME_LEN];
    const uint16_t len0 = make_cmd_frag(wire0, 10U, 0x0001U, 0U, 2U, seg0data, 2U);
    TEST_ASSERT_GREATER_THAN_UINT16(0U, len0);

    uint8_t wire1[MAX_FRAME_LEN];
    const uint16_t len1 = make_cmd_frag(wire1, 11U, 0x0001U, 1U, 2U, seg1data, 1U);
    TEST_ASSERT_GREATER_THAN_UINT16(0U, len1);

    // Feed seg 0 and poll — expect one ACK.
    TEST_ASSERT_TRUE(f.dispatchRadio.injectBytes(wire0, len0));
    f.dispatcher.poll(1000U);
    TEST_ASSERT_EQUAL_UINT32(1U, f.dispatchRadio.sendCount());

    // Feed seg 1 and poll — reassembly complete; REQUEST_STATUS sends TELEMETRY.
    TEST_ASSERT_TRUE(f.dispatchRadio.injectBytes(wire1, len1));
    f.dispatcher.poll(2000U);

    // At minimum: ACK for seg1 + TELEMETRY response from REQUEST_STATUS.
    TEST_ASSERT_GREATER_THAN_UINT32(2U, f.dispatchRadio.sendCount());
}

/**
 * An orphaned single-segment transfer (only seg 0 of 3 arrives) must NOT
 * trigger command execution.
 *
 * Only one ACK (for the single received segment) must be sent.  No
 * additional frames are expected since the assembly never completes.
 */
void test_frag_orphaned_session_no_phantom_execution()
{
    DispatchFixture f;

    const uint8_t segData[1] = { static_cast<uint8_t>(Priority::PRI_LOW) };

    uint8_t wire[MAX_FRAME_LEN];
    const uint16_t len = make_cmd_frag(wire, 5U, 0x0002U, 0U, 3U, segData, 1U);
    TEST_ASSERT_GREATER_THAN_UINT16(0U, len);

    // Inject only the first of three expected segments.
    TEST_ASSERT_TRUE(f.dispatchRadio.injectBytes(wire, len));
    f.dispatcher.poll(1000U);

    // Exactly one ACK for the one segment received; no command execution.
    TEST_ASSERT_EQUAL_UINT32(1U, f.dispatchRadio.sendCount());

    // Additional polls with no new data must not change the send count.
    f.dispatcher.poll(2000U);
    f.dispatcher.poll(3000U);
    TEST_ASSERT_EQUAL_UINT32(1U, f.dispatchRadio.sendCount());
}

/**
 * After kFragTimeoutMs (30 s) of inactivity the session must be discarded.
 * The next arriving fragment of the same transfer must be accepted as a
 * fresh new session (APUS-15.4).
 *
 * Sequence:
 *   T=0:      seg 0/3 arrives → new session started, ACK #1 sent.
 *   T=35000:  same seg 0/3 re-arrives → old session timed out and discarded,
 *             new session started, ACK #2 sent.
 */
void test_frag_session_timeout_discards_and_restarts()
{
    DispatchFixture f;

    const uint8_t segData[1] = { static_cast<uint8_t>(Priority::PRI_LOW) };

    uint8_t wire[MAX_FRAME_LEN];
    const uint16_t len = make_cmd_frag(wire, 20U, 0x0003U, 0U, 3U, segData, 1U);
    TEST_ASSERT_GREATER_THAN_UINT16(0U, len);

    // T=0: segment arrives, session created.
    TEST_ASSERT_TRUE(f.dispatchRadio.injectBytes(wire, len));
    f.dispatcher.poll(0U);
    TEST_ASSERT_EQUAL_UINT32(1U, f.dispatchRadio.sendCount());

    // T=35000 (past kFragTimeoutMs=30000): same segment re-arrives.
    // The dispatcher must detect the timeout, discard the old session,
    // and start a fresh one — sending a second ACK.
    TEST_ASSERT_TRUE(f.dispatchRadio.injectBytes(wire, len));
    f.dispatcher.poll(35000U);
    TEST_ASSERT_EQUAL_UINT32(2U, f.dispatchRadio.sendCount());
}

/**
 * When a fragment for a DIFFERENT transferId arrives while a session is already
 * active, the dispatcher must send a NACK with FailureCode::QUEUE_FULL (APUS-15.3).
 *
 * Sequence:
 *   seg 0 of transferId=0x0010 → new session, ACK.
 *   seg 0 of transferId=0x0011 → collision, NACK.
 *   Total sendCount == 2.
 */
void test_frag_session_collision_sends_nack()
{
    DispatchFixture f;

    const uint8_t segData[1] = { 0x00U };

    uint8_t wire1[MAX_FRAME_LEN];
    const uint16_t len1 = make_cmd_frag(wire1, 30U, 0x0010U, 0U, 2U, segData, 1U);
    TEST_ASSERT_GREATER_THAN_UINT16(0U, len1);

    uint8_t wire2[MAX_FRAME_LEN];
    const uint16_t len2 = make_cmd_frag(wire2, 31U, 0x0011U, 0U, 2U, segData, 1U);
    TEST_ASSERT_GREATER_THAN_UINT16(0U, len2);

    // First fragment: new session, ACK.
    TEST_ASSERT_TRUE(f.dispatchRadio.injectBytes(wire1, len1));
    f.dispatcher.poll(1000U);
    TEST_ASSERT_EQUAL_UINT32(1U, f.dispatchRadio.sendCount());

    // Second fragment with a different transferId: NACK QUEUE_FULL.
    TEST_ASSERT_TRUE(f.dispatchRadio.injectBytes(wire2, len2));
    f.dispatcher.poll(2000U);
    TEST_ASSERT_EQUAL_UINT32(2U, f.dispatchRadio.sendCount());

    // Decode the second outgoing frame and verify it is a NACK.
    const uint8_t* nackWire = f.dispatchRadio.lastFrame();
    const uint16_t nackLen  = f.dispatchRadio.lastFrameLen();
    Frame nackFrame = {};
    TEST_ASSERT_TRUE(decode(nackWire, nackLen, nackFrame));
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(MsgType::ACK),
                            static_cast<uint8_t>(nackFrame.type));
    // AckPayload.failureCode must be QUEUE_FULL (0x06).
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(FailureCode::QUEUE_FULL),
                            nackFrame.payload[2U]);
}

/**
 * After an orphaned session (1 of 3 segments arrived) times out, a completely
 * new transfer must be accepted, reassembled, and its command executed.
 *
 * This verifies that the single session slot is fully recycled after the
 * kFragTimeoutMs expiry and does not retain stale data from the orphaned
 * transfer (APUS-15.4 / APUS-15.5).
 *
 * Sequence:
 *   T=0:      seg 0/3 of transfer 0x0099 arrives → session A started, ACK #1.
 *             Segments 1 and 2 are never sent (orphaned).
 *   T=35000:  seg 0/2 of NEW transfer 0x00AA arrives past kFragTimeoutMs →
 *             session A discarded, session B started, ACK #2. sendCount==2.
 *   T=36000:  seg 1/2 of transfer 0x00AA arrives → assembly completes,
 *             REQUEST_STATUS executes → ACK #3 + TELEMETRY. sendCount>2.
 */
void test_frag_orphaned_session_expires_and_slot_recycled()
{
    DispatchFixture f;

    // ── Transfer A (orphaned — only seg 0/3 ever sent) ───────────────────────
    const uint8_t dummyData[1] = { 0x00U };
    uint8_t wireA[MAX_FRAME_LEN];
    const uint16_t lenA = make_cmd_frag(wireA, 40U, 0x0099U, 0U, 3U, dummyData, 1U);
    TEST_ASSERT_GREATER_THAN_UINT16(0U, lenA);

    TEST_ASSERT_TRUE(f.dispatchRadio.injectBytes(wireA, lenA));
    f.dispatcher.poll(0U);
    TEST_ASSERT_EQUAL_UINT32(1U, f.dispatchRadio.sendCount());   // ACK for seg 0/A

    // ── Transfer B (complete 2-segment transfer, sent after timeout) ─────────
    // Seg 0 carries the full CommandHeader so the assembled command is valid.
    const uint8_t seg0data[2] = {
        static_cast<uint8_t>(Priority::PRI_LOW),
        static_cast<uint8_t>(CommandId::REQUEST_STATUS)
    };
    const uint8_t seg1data[1] = { 0x00U };  // dummy — triggers assembly completion

    uint8_t wireB0[MAX_FRAME_LEN];
    const uint16_t lenB0 = make_cmd_frag(wireB0, 41U, 0x00AAU, 0U, 2U, seg0data, 2U);
    TEST_ASSERT_GREATER_THAN_UINT16(0U, lenB0);

    uint8_t wireB1[MAX_FRAME_LEN];
    const uint16_t lenB1 = make_cmd_frag(wireB1, 42U, 0x00AAU, 1U, 2U, seg1data, 1U);
    TEST_ASSERT_GREATER_THAN_UINT16(0U, lenB1);

    // T=35000 ms — session A has timed out (kFragTimeoutMs=30000).
    // Injecting seg 0 of B must discard session A and start a fresh session B.
    TEST_ASSERT_TRUE(f.dispatchRadio.injectBytes(wireB0, lenB0));
    f.dispatcher.poll(35000U);
    TEST_ASSERT_EQUAL_UINT32(2U, f.dispatchRadio.sendCount());   // ACK for seg 0/B

    // T=36000 ms — seg 1 of B completes the transfer; REQUEST_STATUS executes.
    TEST_ASSERT_TRUE(f.dispatchRadio.injectBytes(wireB1, lenB1));
    f.dispatcher.poll(36000U);
    // At minimum: ACK for seg 1/B + TELEMETRY from REQUEST_STATUS.
    TEST_ASSERT_GREATER_THAN_UINT32(2U, f.dispatchRadio.sendCount());
}

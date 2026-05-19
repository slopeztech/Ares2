/**
 * @file  test_command_dispatch.cpp
 * @brief Unit tests for RadioDispatcher non-fragmented command dispatch.
 *
 * Covers the paths that test_fragment_reassembly does NOT exercise:
 *   - Non-fragmented COMMAND frames: REQUEST_STATUS, REQUEST_CONFIG,
 *     VERIFY_CONFIG, SET_MODE, SET_FCS_ACTIVE, SET_CONFIG_PARAM,
 *     REQUEST_TELEMETRY, SET_TELEM_INTERVAL, ARM_FLIGHT, ABORT,
 *     FIRE_PULSE_A/B/C/D, FACTORY_RESET.
 *   - FLAG_ACK_REQ completion ACK/NACK path in drainCmdQueue.
 *   - HEARTBEAT frame → HEARTBEAT response.
 *   - TELEMETRY / EVENT / ACK frames → silently discarded.
 *   - Routing failure (wrong node) → ROUTING_FAIL NACK.
 *   - Duplicate command (same SEQ) → silently discarded.
 *   - Buffer with no SYNC_0 byte → all bytes discarded.
 *   - Buffer with leading garbage followed by valid frame → garbage stripped.
 *   - CRC-corrupted frame → decode failure, SYNC byte skipped.
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

// ── Shared resting flight profile ────────────────────────────────────────────

static const ares::sim::FlightProfile kRestProfile = {
    .count   = 1U,
    .samples = {
        { .timeMs          = 0U,
          .gpsLatDeg       = 40.0f,  .gpsLonDeg      = -3.0f,
          .gpsAltM         = 0.0f,   .gpsSpeedKmh    = 0.0f,
          .gpsSats         = 8U,     .gpsFix         = true,
          .baroAltM        = 0.0f,   .baroPressurePa = 101325.0f,
          .baroTempC       = 20.0f,
          .accelX          = 0.0f,   .accelY         = 0.0f,
          .accelZ          = 9.81f,  .imuTempC       = 25.0f },
    }
};

// ── Test fixture ──────────────────────────────────────────────────────────────

struct CmdDispatchFixture
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

    ares::sim::SimRadioDriver dispatchRadio;
    ares::RadioDispatcher     dispatcher{ dispatchRadio, engine, nullptr };

    CmdDispatchFixture()
    {
        (void)storage.begin();
        (void)engine.begin();
        (void)dispatchRadio.begin();
    }
};

// ── Frame build helpers ───────────────────────────────────────────────────────

/**
 * Build a minimal non-fragmented COMMAND wire frame.
 *
 * Payload layout: CommandHeader(2) = { priority, commandId } + extra bytes.
 *
 * @param[out] buf      Buffer of at least MAX_FRAME_LEN bytes.
 * @param[in]  seq      Frame sequence number.
 * @param[in]  flags    Frame flags (must NOT include FLAG_FRAGMENT).
 * @param[in]  id       Command identifier.
 * @param[in]  extra    Additional payload bytes after CommandHeader (optional).
 * @param[in]  extraLen Length of @p extra.
 * @return Wire frame length (0 on error).
 */
static uint16_t make_cmd(uint8_t*       buf,
                          uint8_t        seq,
                          uint8_t        flags,
                          CommandId      id,
                          const uint8_t* extra    = nullptr,
                          uint8_t        extraLen = 0U)
{
    Frame tx = {};
    tx.ver        = PROTOCOL_VERSION;
    tx.node       = NODE_ROCKET;
    tx.type       = MsgType::COMMAND;
    tx.seq        = seq;
    tx.flags      = flags;
    tx.payload[0] = static_cast<uint8_t>(Priority::PRI_LOW);
    tx.payload[1] = static_cast<uint8_t>(id);
    tx.len        = 2U;

    if (extra != nullptr && extraLen > 0U)
    {
        (void)memcpy(&tx.payload[2U], extra, extraLen);
        tx.len = static_cast<uint8_t>(2U + extraLen);
    }

    return encode(tx, buf, MAX_FRAME_LEN);
}

/** Build a HEARTBEAT wire frame addressed to NODE_ROCKET. */
static uint16_t make_heartbeat(uint8_t* buf, uint8_t seq)
{
    Frame tx = {};
    tx.ver   = PROTOCOL_VERSION;
    tx.node  = NODE_ROCKET;
    tx.type  = MsgType::HEARTBEAT;
    tx.seq   = seq;
    tx.flags = 0U;
    tx.len   = 0U;
    return encode(tx, buf, MAX_FRAME_LEN);
}

/**
 * Build a TELEMETRY wire frame (e.g. a spurious ground echo).
 *
 * TelemetryPayload must be >= sizeof(TelemetryPayload) == 78 bytes for
 * decode() to accept the frame, so the payload is zero-filled to that size.
 */
static uint16_t make_telemetry_frame(uint8_t* buf, uint8_t seq)
{
    Frame tx = {};
    tx.ver   = PROTOCOL_VERSION;
    tx.node  = NODE_ROCKET;
    tx.type  = MsgType::TELEMETRY;
    tx.seq   = seq;
    tx.flags = 0U;
    // Zero-fill to satisfy meetsMinPayloadLen() == sizeof(TelemetryPayload).
    tx.len   = static_cast<uint8_t>(sizeof(TelemetryPayload));
    (void)memset(tx.payload, 0, sizeof(TelemetryPayload));
    return encode(tx, buf, MAX_FRAME_LEN);
}

/**
 * Build an EVENT wire frame.
 *
 * EventHeader is 6 bytes; zero-filled to satisfy meetsMinPayloadLen().
 */
static uint16_t make_event_frame(uint8_t* buf, uint8_t seq)
{
    Frame tx = {};
    tx.ver   = PROTOCOL_VERSION;
    tx.node  = NODE_ROCKET;
    tx.type  = MsgType::EVENT;
    tx.seq   = seq;
    tx.flags = 0U;
    // Zero-fill to satisfy meetsMinPayloadLen() == sizeof(EventHeader).
    tx.len   = static_cast<uint8_t>(sizeof(EventHeader));
    (void)memset(tx.payload, 0, sizeof(EventHeader));
    return encode(tx, buf, MAX_FRAME_LEN);
}

/**
 * Build a zero-payload ACK wire frame (no AckPayload data).
 * Exercises the "RX ACK (no retry data)" branch in dispatchFrame.
 */
static uint16_t make_ack_short_frame(uint8_t* buf, uint8_t seq)
{
    Frame tx = {};
    tx.ver   = PROTOCOL_VERSION;
    tx.node  = NODE_ROCKET;
    tx.type  = MsgType::ACK;
    tx.seq   = seq;
    tx.flags = 0U;
    tx.len   = 0U;  // Below sizeof(AckPayload) == 4.
    return encode(tx, buf, MAX_FRAME_LEN);
}

/** Build an ACK wire frame carrying a full AckPayload for @p originalSeq. */
static uint16_t make_ack_frame(uint8_t* buf, uint8_t seq, uint8_t originalSeq)
{
    AckPayload ap = {};
    ap.originalSeq  = originalSeq;
    ap.originalNode = NODE_GROUND;
    ap.failureCode  = 0U;
    ap.failureData  = 0U;

    Frame tx = {};
    tx.ver   = PROTOCOL_VERSION;
    tx.node  = NODE_ROCKET;
    tx.type  = MsgType::ACK;
    tx.seq   = seq;
    tx.flags = 0U;
    tx.len   = static_cast<uint8_t>(sizeof(ap));
    (void)memcpy(tx.payload, &ap, sizeof(ap));
    return encode(tx, buf, MAX_FRAME_LEN);
}

// ── Non-fragmented COMMAND tests ──────────────────────────────────────────────

/**
 * REQUEST_STATUS non-fragmented: acceptance ACK + TELEMETRY status frame.
 * Expected sendCount == 2.
 */
void test_cmd_nonfrag_request_status()
{
    CmdDispatchFixture f;
    uint8_t wire[MAX_FRAME_LEN];
    const uint16_t len = make_cmd(wire, 1U, 0U, CommandId::REQUEST_STATUS);
    TEST_ASSERT_GREATER_THAN_UINT16(0U, len);

    TEST_ASSERT_TRUE(f.dispatchRadio.injectBytes(wire, len));
    f.dispatcher.poll(1000U);

    // Acceptance ACK + TELEMETRY response from executeCommand.
    TEST_ASSERT_EQUAL_UINT32(2U, f.dispatchRadio.sendCount());
}

/**
 * REQUEST_CONFIG non-fragmented: acceptance ACK + TELEMETRY config table.
 * Expected sendCount == 2.
 */
void test_cmd_nonfrag_request_config()
{
    CmdDispatchFixture f;
    uint8_t wire[MAX_FRAME_LEN];
    const uint16_t len = make_cmd(wire, 2U, 0U, CommandId::REQUEST_CONFIG);
    TEST_ASSERT_GREATER_THAN_UINT16(0U, len);

    TEST_ASSERT_TRUE(f.dispatchRadio.injectBytes(wire, len));
    f.dispatcher.poll(1000U);

    // Acceptance ACK + TELEMETRY config report.
    TEST_ASSERT_EQUAL_UINT32(2U, f.dispatchRadio.sendCount());
}

/**
 * VERIFY_CONFIG with all default parameters valid: only acceptance ACK sent.
 * Expected sendCount == 1.
 */
void test_cmd_nonfrag_verify_config_clean()
{
    CmdDispatchFixture f;
    uint8_t wire[MAX_FRAME_LEN];
    const uint16_t len = make_cmd(wire, 3U, 0U, CommandId::VERIFY_CONFIG);
    TEST_ASSERT_GREATER_THAN_UINT16(0U, len);

    TEST_ASSERT_TRUE(f.dispatchRadio.injectBytes(wire, len));
    f.dispatcher.poll(1000U);

    TEST_ASSERT_EQUAL_UINT32(1U, f.dispatchRadio.sendCount());
}

/**
 * SET_MODE (enable, mode=1): acceptance ACK only (no FLAG_ACK_REQ).
 * Expected sendCount == 1.
 */
void test_cmd_nonfrag_set_mode_enable()
{
    CmdDispatchFixture f;
    const uint8_t extra[1U] = { 0x01U };
    uint8_t wire[MAX_FRAME_LEN];
    const uint16_t len = make_cmd(wire, 4U, 0U, CommandId::SET_MODE, extra, 1U);
    TEST_ASSERT_GREATER_THAN_UINT16(0U, len);

    TEST_ASSERT_TRUE(f.dispatchRadio.injectBytes(wire, len));
    f.dispatcher.poll(1000U);

    TEST_ASSERT_EQUAL_UINT32(1U, f.dispatchRadio.sendCount());
}

/**
 * SET_MODE (disable, mode=0): acceptance ACK only.
 * Expected sendCount == 1.
 */
void test_cmd_nonfrag_set_mode_disable()
{
    CmdDispatchFixture f;
    const uint8_t extra[1U] = { 0x00U };
    uint8_t wire[MAX_FRAME_LEN];
    const uint16_t len = make_cmd(wire, 5U, 0U, CommandId::SET_MODE, extra, 1U);

    TEST_ASSERT_TRUE(f.dispatchRadio.injectBytes(wire, len));
    f.dispatcher.poll(1000U);

    TEST_ASSERT_EQUAL_UINT32(1U, f.dispatchRadio.sendCount());
}

/**
 * SET_MODE with invalid mode byte (mode=2): executeCommand returns
 * INVALID_PARAM; acceptance ACK already sent; no FLAG_ACK_REQ.
 * Expected sendCount == 1.
 */
void test_cmd_nonfrag_set_mode_invalid()
{
    CmdDispatchFixture f;
    const uint8_t extra[1U] = { 0x02U };  // mode > 1 is invalid
    uint8_t wire[MAX_FRAME_LEN];
    const uint16_t len = make_cmd(wire, 6U, 0U, CommandId::SET_MODE, extra, 1U);

    TEST_ASSERT_TRUE(f.dispatchRadio.injectBytes(wire, len));
    f.dispatcher.poll(1000U);

    TEST_ASSERT_EQUAL_UINT32(1U, f.dispatchRadio.sendCount());
}

/**
 * SET_FCS_ACTIVE (enabled=1): acceptance ACK only.
 * Expected sendCount == 1.
 */
void test_cmd_nonfrag_set_fcs_active()
{
    CmdDispatchFixture f;
    const uint8_t extra[1U] = { 0x01U };
    uint8_t wire[MAX_FRAME_LEN];
    const uint16_t len = make_cmd(wire, 7U, 0U, CommandId::SET_FCS_ACTIVE, extra, 1U);

    TEST_ASSERT_TRUE(f.dispatchRadio.injectBytes(wire, len));
    f.dispatcher.poll(1000U);

    TEST_ASSERT_EQUAL_UINT32(1U, f.dispatchRadio.sendCount());
}

/**
 * SET_CONFIG_PARAM with a valid TELEM_INTERVAL_MS value (5000 ms inside
 * [100, 60000]).  Acceptance ACK only (no FLAG_ACK_REQ).
 * Expected sendCount == 1.
 */
void test_cmd_nonfrag_set_config_param_valid()
{
    CmdDispatchFixture f;

    const float intervalF = 5000.0f;
    uint32_t raw = 0U;
    (void)memcpy(&raw, &intervalF, sizeof(raw));

    const uint8_t extra[5U] = {
        static_cast<uint8_t>(ConfigParamId::TELEM_INTERVAL_MS),
        static_cast<uint8_t>( raw        & 0xFFU),
        static_cast<uint8_t>((raw >>  8U) & 0xFFU),
        static_cast<uint8_t>((raw >> 16U) & 0xFFU),
        static_cast<uint8_t>((raw >> 24U) & 0xFFU),
    };
    uint8_t wire[MAX_FRAME_LEN];
    const uint16_t len = make_cmd(wire, 8U, 0U,
                                  CommandId::SET_CONFIG_PARAM, extra, 5U);

    TEST_ASSERT_TRUE(f.dispatchRadio.injectBytes(wire, len));
    f.dispatcher.poll(1000U);

    TEST_ASSERT_EQUAL_UINT32(1U, f.dispatchRadio.sendCount());
}

/**
 * SET_CONFIG_PARAM with an out-of-range value (99 ms below minimum 100 ms):
 * executeCommand returns INVALID_PARAM; acceptance ACK already sent.
 * Expected sendCount == 1.
 */
void test_cmd_nonfrag_set_config_param_out_of_range()
{
    CmdDispatchFixture f;

    const float badInterval = 99.0f;
    uint32_t raw = 0U;
    (void)memcpy(&raw, &badInterval, sizeof(raw));

    const uint8_t extra[5U] = {
        static_cast<uint8_t>(ConfigParamId::TELEM_INTERVAL_MS),
        static_cast<uint8_t>( raw        & 0xFFU),
        static_cast<uint8_t>((raw >>  8U) & 0xFFU),
        static_cast<uint8_t>((raw >> 16U) & 0xFFU),
        static_cast<uint8_t>((raw >> 24U) & 0xFFU),
    };
    uint8_t wire[MAX_FRAME_LEN];
    const uint16_t len = make_cmd(wire, 9U, 0U,
                                  CommandId::SET_CONFIG_PARAM, extra, 5U);

    TEST_ASSERT_TRUE(f.dispatchRadio.injectBytes(wire, len));
    f.dispatcher.poll(1000U);

    TEST_ASSERT_EQUAL_UINT32(1U, f.dispatchRadio.sendCount());
}

/**
 * ARM_FLIGHT on an engine that has not loaded a script: engine_.arm() returns
 * false → PRECONDITION_FAIL; acceptance ACK sent; no FLAG_ACK_REQ.
 * Expected sendCount == 1.
 */
void test_cmd_nonfrag_arm_flight_precondition_fail()
{
    CmdDispatchFixture f;
    uint8_t wire[MAX_FRAME_LEN];
    const uint16_t len = make_cmd(wire, 10U, 0U, CommandId::ARM_FLIGHT);

    TEST_ASSERT_TRUE(f.dispatchRadio.injectBytes(wire, len));
    f.dispatcher.poll(1000U);

    TEST_ASSERT_EQUAL_UINT32(1U, f.dispatchRadio.sendCount());
}

/**
 * ABORT without FLAG_PRIORITY: executeCommand detects the missing critical
 * flag and returns PRECONDITION_FAIL before touching the engine.
 * Acceptance ACK is already sent by handleCommand; no FLAG_ACK_REQ.
 * Expected sendCount == 1.
 */
void test_cmd_nonfrag_abort_missing_priority_rejected()
{
    CmdDispatchFixture f;
    uint8_t wire[MAX_FRAME_LEN];
    // Flags = 0: FLAG_PRIORITY (0x04) is absent → critical command rejected.
    const uint16_t len = make_cmd(wire, 11U, 0U, CommandId::ABORT);

    TEST_ASSERT_TRUE(f.dispatchRadio.injectBytes(wire, len));
    f.dispatcher.poll(1000U);

    TEST_ASSERT_EQUAL_UINT32(1U, f.dispatchRadio.sendCount());
}

/**
 * ABORT with FLAG_PRIORITY on an idle engine: the critical-flag check passes;
 * injectTcCommand("ABORT") fails in IDLE (no script loaded) → EXECUTION_ERROR;
 * acceptance ACK sent; no FLAG_ACK_REQ.
 * Expected sendCount == 1.
 */
void test_cmd_nonfrag_abort_with_priority_idle_engine()
{
    CmdDispatchFixture f;
    uint8_t wire[MAX_FRAME_LEN];
    const uint16_t len = make_cmd(wire, 12U,
                                  FLAG_PRIORITY, CommandId::ABORT);

    TEST_ASSERT_TRUE(f.dispatchRadio.injectBytes(wire, len));
    f.dispatcher.poll(1000U);

    TEST_ASSERT_EQUAL_UINT32(1U, f.dispatchRadio.sendCount());
}

/**
 * FIRE_PULSE_A with FLAG_PRIORITY but engine not RUNNING: returns
 * PRECONDITION_FAIL.  Acceptance ACK sent; no FLAG_ACK_REQ.
 * Expected sendCount == 1.
 */
void test_cmd_nonfrag_fire_pulse_a_engine_not_running()
{
    CmdDispatchFixture f;
    uint8_t wire[MAX_FRAME_LEN];
    const uint16_t len = make_cmd(wire, 13U,
                                  FLAG_PRIORITY, CommandId::FIRE_PULSE_A);

    TEST_ASSERT_TRUE(f.dispatchRadio.injectBytes(wire, len));
    f.dispatcher.poll(1000U);

    TEST_ASSERT_EQUAL_UINT32(1U, f.dispatchRadio.sendCount());
}

/**
 * FACTORY_RESET with FLAG_PRIORITY on idle engine: injectTcCommand("RESET")
 * result is engine-defined; acceptance ACK always sent; no FLAG_ACK_REQ.
 * Expected sendCount == 1.
 */
void test_cmd_nonfrag_factory_reset_with_priority()
{
    CmdDispatchFixture f;
    uint8_t wire[MAX_FRAME_LEN];
    const uint16_t len = make_cmd(wire, 14U,
                                  FLAG_PRIORITY, CommandId::FACTORY_RESET);

    TEST_ASSERT_TRUE(f.dispatchRadio.injectBytes(wire, len));
    f.dispatcher.poll(1000U);

    TEST_ASSERT_EQUAL_UINT32(1U, f.dispatchRadio.sendCount());
}

/**
 * REQUEST_TELEMETRY on an idle engine (no active HK slots):
 * engine_.requestTelemetry() returns false → EXECUTION_ERROR.
 * Acceptance ACK sent; no FLAG_ACK_REQ.
 * Expected sendCount == 1.
 */
void test_cmd_nonfrag_request_telemetry_idle_engine()
{
    CmdDispatchFixture f;
    uint8_t wire[MAX_FRAME_LEN];
    const uint16_t len = make_cmd(wire, 15U, 0U, CommandId::REQUEST_TELEMETRY);

    TEST_ASSERT_TRUE(f.dispatchRadio.injectBytes(wire, len));
    f.dispatcher.poll(1000U);

    TEST_ASSERT_EQUAL_UINT32(1U, f.dispatchRadio.sendCount());
}

/**
 * SET_TELEM_INTERVAL with a valid interval (5000 ms inside [100, 60000]) on an
 * idle engine with no active HK slots: setTelemInterval() returns false →
 * EXECUTION_ERROR; acceptance ACK sent; no FLAG_ACK_REQ.
 * Expected sendCount == 1.
 */
void test_cmd_nonfrag_set_telem_interval_valid()
{
    CmdDispatchFixture f;
    const uint32_t intervalMs = 5000U;
    const uint8_t extra[4U] = {
        static_cast<uint8_t>( intervalMs        & 0xFFU),
        static_cast<uint8_t>((intervalMs >>  8U) & 0xFFU),
        static_cast<uint8_t>((intervalMs >> 16U) & 0xFFU),
        static_cast<uint8_t>((intervalMs >> 24U) & 0xFFU),
    };
    uint8_t wire[MAX_FRAME_LEN];
    const uint16_t len = make_cmd(wire, 16U, 0U,
                                  CommandId::SET_TELEM_INTERVAL, extra, 4U);

    TEST_ASSERT_TRUE(f.dispatchRadio.injectBytes(wire, len));
    f.dispatcher.poll(1000U);

    TEST_ASSERT_EQUAL_UINT32(1U, f.dispatchRadio.sendCount());
}

/**
 * FLAG_ACK_REQ set on VERIFY_CONFIG: drainCmdQueue sends completion ACK after
 * executeCommand() returns NONE.
 * Expected sendCount == 2  (acceptance ACK + completion ACK).
 */
void test_cmd_flag_ack_req_triggers_completion_ack()
{
    CmdDispatchFixture f;
    uint8_t wire[MAX_FRAME_LEN];
    const uint16_t len = make_cmd(wire, 17U,
                                  FLAG_ACK_REQ, CommandId::VERIFY_CONFIG);

    TEST_ASSERT_TRUE(f.dispatchRadio.injectBytes(wire, len));
    f.dispatcher.poll(1000U);

    // Acceptance ACK (handleCommand) + completion ACK (drainCmdQueue).
    TEST_ASSERT_EQUAL_UINT32(2U, f.dispatchRadio.sendCount());
}

// ── Duplicate detection test ──────────────────────────────────────────────────

/**
 * Second occurrence of the same SEQ must be silently discarded; sendCount
 * must not increase after the first poll.
 */
void test_cmd_duplicate_is_discarded()
{
    CmdDispatchFixture f;
    uint8_t wire[MAX_FRAME_LEN];
    const uint16_t len = make_cmd(wire, 20U, 0U, CommandId::REQUEST_STATUS);
    TEST_ASSERT_GREATER_THAN_UINT16(0U, len);

    // First occurrence — processed normally.
    TEST_ASSERT_TRUE(f.dispatchRadio.injectBytes(wire, len));
    f.dispatcher.poll(1000U);
    const uint32_t afterFirst = f.dispatchRadio.sendCount();
    TEST_ASSERT_EQUAL_UINT32(2U, afterFirst);

    // Second occurrence with the identical SEQ — must be silently discarded.
    TEST_ASSERT_TRUE(f.dispatchRadio.injectBytes(wire, len));
    f.dispatcher.poll(2000U);
    TEST_ASSERT_EQUAL_UINT32(afterFirst, f.dispatchRadio.sendCount());
}

// ── Anti-replay tests ([H5]) ──────────────────────────────────────────────────

/**
 * SEQ N-1 replayed after SEQ N has been accepted: the old code accepted it
 * (equality-only check); the sliding-window bitmap must reject it.
 * Expected: second sendCount equals first (replay is silently dropped).
 */
void test_cmd_replay_previous_seq_rejected()
{
    CmdDispatchFixture f;
    uint8_t wireN[MAX_FRAME_LEN];
    uint8_t wireN1[MAX_FRAME_LEN];

    // Accept SEQ = 50.
    const uint16_t lenN  = make_cmd(wireN,  50U, 0U, CommandId::REQUEST_STATUS);
    TEST_ASSERT_TRUE(f.dispatchRadio.injectBytes(wireN, lenN));
    f.dispatcher.poll(1000U);
    const uint32_t after50 = f.dispatchRadio.sendCount();

    // Accept SEQ = 51.
    const uint16_t lenN1 = make_cmd(wireN1, 51U, 0U, CommandId::REQUEST_STATUS);
    TEST_ASSERT_TRUE(f.dispatchRadio.injectBytes(wireN1, lenN1));
    f.dispatcher.poll(2000U);
    const uint32_t after51 = f.dispatchRadio.sendCount();
    TEST_ASSERT_GREATER_THAN_UINT32(after50, after51);

    // Replay of SEQ = 50 (one behind current high) — must be discarded.
    TEST_ASSERT_TRUE(f.dispatchRadio.injectBytes(wireN, lenN));
    f.dispatcher.poll(3000U);
    TEST_ASSERT_EQUAL_UINT32(after51, f.dispatchRadio.sendCount());
}

/**
 * SEQ that is 64 (kWindowSize) behind the current high is outside the
 * look-back window and must be silently discarded.
 */
void test_cmd_replay_outside_window_rejected()
{
    CmdDispatchFixture f;
    uint8_t wire10[MAX_FRAME_LEN];
    uint8_t wire74[MAX_FRAME_LEN];

    // Accept SEQ = 10.
    const uint16_t len10 = make_cmd(wire10, 10U, 0U, CommandId::REQUEST_STATUS);
    TEST_ASSERT_TRUE(f.dispatchRadio.injectBytes(wire10, len10));
    f.dispatcher.poll(1000U);

    // Advance high to SEQ = 74 (fwd = 64 → full window reset, 10 is now outside).
    const uint16_t len74 = make_cmd(wire74, 74U, 0U, CommandId::REQUEST_STATUS);
    TEST_ASSERT_TRUE(f.dispatchRadio.injectBytes(wire74, len74));
    f.dispatcher.poll(2000U);
    const uint32_t after74 = f.dispatchRadio.sendCount();

    // Replay of SEQ = 10 (64 behind 74) — outside window → discarded.
    TEST_ASSERT_TRUE(f.dispatchRadio.injectBytes(wire10, len10));
    f.dispatcher.poll(3000U);
    TEST_ASSERT_EQUAL_UINT32(after74, f.dispatchRadio.sendCount());
}

// ── HEARTBEAT test ────────────────────────────────────────────────────────────

/**
 * HEARTBEAT frame → dispatcher responds with one HEARTBEAT frame.
 * Expected sendCount == 1.
 */
void test_heartbeat_gets_response()
{
    CmdDispatchFixture f;
    uint8_t wire[MAX_FRAME_LEN];
    const uint16_t len = make_heartbeat(wire, 30U);
    TEST_ASSERT_GREATER_THAN_UINT16(0U, len);

    TEST_ASSERT_TRUE(f.dispatchRadio.injectBytes(wire, len));
    f.dispatcher.poll(1000U);

    TEST_ASSERT_EQUAL_UINT32(1U, f.dispatchRadio.sendCount());
}

// ── Discard tests ─────────────────────────────────────────────────────────────

/**
 * TELEMETRY frame (ground echo): silently discarded, nothing sent.
 * Expected sendCount == 0.
 */
void test_telemetry_frame_silently_discarded()
{
    CmdDispatchFixture f;
    uint8_t wire[MAX_FRAME_LEN];
    const uint16_t len = make_telemetry_frame(wire, 31U);
    TEST_ASSERT_GREATER_THAN_UINT16(0U, len);

    TEST_ASSERT_TRUE(f.dispatchRadio.injectBytes(wire, len));
    f.dispatcher.poll(1000U);

    TEST_ASSERT_EQUAL_UINT32(0U, f.dispatchRadio.sendCount());
}

/**
 * EVENT frame: silently discarded, nothing sent.
 * Expected sendCount == 0.
 */
void test_event_frame_silently_discarded()
{
    CmdDispatchFixture f;
    uint8_t wire[MAX_FRAME_LEN];
    const uint16_t len = make_event_frame(wire, 32U);
    TEST_ASSERT_GREATER_THAN_UINT16(0U, len);

    TEST_ASSERT_TRUE(f.dispatchRadio.injectBytes(wire, len));
    f.dispatcher.poll(1000U);

    TEST_ASSERT_EQUAL_UINT32(0U, f.dispatchRadio.sendCount());
}

/**
 * ACK frame with full AckPayload: dispatcher calls clearRetryForSeq() (no
 * matching retry slot active, so no effect) and sends nothing.
 * Expected sendCount == 0.
 */
void test_ack_frame_clears_retry_no_side_effect()
{
    CmdDispatchFixture f;
    uint8_t wire[MAX_FRAME_LEN];
    const uint16_t len = make_ack_frame(wire, 33U, 5U);
    TEST_ASSERT_GREATER_THAN_UINT16(0U, len);

    TEST_ASSERT_TRUE(f.dispatchRadio.injectBytes(wire, len));
    f.dispatcher.poll(1000U);

    TEST_ASSERT_EQUAL_UINT32(0U, f.dispatchRadio.sendCount());
}

// ── Routing failure test ──────────────────────────────────────────────────────

/**
 * COMMAND addressed to NODE_PAYLOAD (wrong node): dispatcher sends a
 * ROUTING_FAIL NACK.  Expected sendCount == 1.
 */
void test_cmd_wrong_node_sends_routing_nack()
{
    CmdDispatchFixture f;

    Frame tx = {};
    tx.ver        = PROTOCOL_VERSION;
    tx.node       = NODE_PAYLOAD;       // Not NODE_ROCKET or NODE_BROADCAST.
    tx.type       = MsgType::COMMAND;
    tx.seq        = 40U;
    tx.flags      = 0U;
    tx.payload[0] = static_cast<uint8_t>(Priority::PRI_LOW);
    tx.payload[1] = static_cast<uint8_t>(CommandId::REQUEST_STATUS);
    tx.len        = 2U;

    uint8_t wire[MAX_FRAME_LEN];
    const uint16_t len = encode(tx, wire, sizeof(wire));
    TEST_ASSERT_GREATER_THAN_UINT16(0U, len);

    TEST_ASSERT_TRUE(f.dispatchRadio.injectBytes(wire, len));
    f.dispatcher.poll(1000U);

    // ROUTING_FAIL NACK.
    TEST_ASSERT_EQUAL_UINT32(1U, f.dispatchRadio.sendCount());
}

// ── Buffer-level corruption and garbage tests ─────────────────────────────────

/**
 * Buffer containing only non-SYNC bytes (no 0xAE): processBuffer discards all
 * bytes and sends nothing.  Expected sendCount == 0.
 */
void test_buffer_no_sync_byte_discards_all()
{
    CmdDispatchFixture f;

    // None of these bytes equals SYNC_0 (0xAE).
    const uint8_t garbage[16U] = {
        0x00U, 0x11U, 0x22U, 0x33U, 0x44U, 0x55U, 0x66U, 0x77U,
        0x88U, 0x99U, 0xAAU, 0xBBU, 0xCCU, 0xDDU, 0xEEU, 0xFFU
    };

    TEST_ASSERT_TRUE(f.dispatchRadio.injectBytes(
        garbage, static_cast<uint16_t>(sizeof(garbage))));
    f.dispatcher.poll(1000U);

    TEST_ASSERT_EQUAL_UINT32(0U, f.dispatchRadio.sendCount());
}

/**
 * Buffer with four non-SYNC leading bytes followed by a valid frame:
 * processBuffer strips the garbage and decodes the frame normally.
 * Expected sendCount == 2 (acceptance ACK + REQUEST_STATUS TELEMETRY).
 */
void test_buffer_leading_garbage_then_valid_frame()
{
    CmdDispatchFixture f;

    uint8_t wire[MAX_FRAME_LEN];
    const uint16_t frameLen = make_cmd(wire, 50U, 0U, CommandId::REQUEST_STATUS);
    TEST_ASSERT_GREATER_THAN_UINT16(0U, frameLen);

    // Prepend four bytes that are guaranteed not to be SYNC_0 (0xAE).
    const uint8_t prefix[4U] = { 0x00U, 0x11U, 0x22U, 0x33U };
    uint8_t combined[4U + MAX_FRAME_LEN] = {};
    (void)memcpy(combined, prefix, 4U);
    (void)memcpy(combined + 4U, wire, frameLen);
    const uint16_t totalLen = static_cast<uint16_t>(4U + frameLen);

    TEST_ASSERT_TRUE(f.dispatchRadio.injectBytes(combined, totalLen));
    f.dispatcher.poll(1000U);

    // Frame decoded and executed despite leading garbage.
    TEST_ASSERT_EQUAL_UINT32(2U, f.dispatchRadio.sendCount());
}

/**
 * CRC-corrupted frame: the last byte (part of CRC-32) is flipped so decode()
 * fails.  processBuffer skips the SYNC byte and finds no further valid frame.
 * Expected sendCount == 0.
 */
void test_buffer_corrupt_crc_frame_skipped()
{
    CmdDispatchFixture f;

    uint8_t wire[MAX_FRAME_LEN];
    const uint16_t len = make_cmd(wire, 51U, 0U, CommandId::REQUEST_STATUS);
    TEST_ASSERT_GREATER_THAN_UINT16(0U, len);

    // Flip the last byte (CRC-32 LSB) to corrupt the CRC.
    wire[static_cast<uint16_t>(len - 1U)] ^= 0xFFU;

    TEST_ASSERT_TRUE(f.dispatchRadio.injectBytes(wire, len));
    f.dispatcher.poll(1000U);

    TEST_ASSERT_EQUAL_UINT32(0U, f.dispatchRadio.sendCount());
}

/**
 * ACK frame with payload length 0 (below sizeof(AckPayload) == 4): dispatcher
 * takes the "no retry data" log branch in dispatchFrame and sends nothing.
 * Expected sendCount == 0.
 */
void test_ack_frame_short_no_payload()
{
    CmdDispatchFixture f;
    uint8_t wire[MAX_FRAME_LEN];
    const uint16_t len = make_ack_short_frame(wire, 52U);
    TEST_ASSERT_GREATER_THAN_UINT16(0U, len);

    TEST_ASSERT_TRUE(f.dispatchRadio.injectBytes(wire, len));
    f.dispatcher.poll(1000U);

    TEST_ASSERT_EQUAL_UINT32(0U, f.dispatchRadio.sendCount());
}

/**
 * COMMAND with commandId 0x25 (beyond CommandId::LAST == 0x24): executeCommand
 * returns UNKNOWN_COMMAND.  FLAG_ACK_REQ triggers a completion ACK whose
 * failureCode must equal FailureCode::UNKNOWN_COMMAND.
 * Expected sendCount == 2  (acceptance ACK + completion ACK/NACK).
 */
void test_cmd_nonfrag_unknown_commandid()
{
    CmdDispatchFixture f;
    Frame tx = {};
    tx.ver        = PROTOCOL_VERSION;
    tx.node       = NODE_ROCKET;
    tx.type       = MsgType::COMMAND;
    tx.seq        = 53U;
    tx.flags      = FLAG_ACK_REQ;
    tx.payload[0] = static_cast<uint8_t>(Priority::PRI_LOW);
    tx.payload[1] = 0x25U;  // One past CommandId::LAST (0x24).
    tx.len        = 2U;

    uint8_t wire[MAX_FRAME_LEN];
    const uint16_t len = encode(tx, wire, sizeof(wire));
    TEST_ASSERT_GREATER_THAN_UINT16(0U, len);

    TEST_ASSERT_TRUE(f.dispatchRadio.injectBytes(wire, len));
    f.dispatcher.poll(1000U);

    // Acceptance ACK (handleCommand) + completion ACK/NACK (drainCmdQueue).
    TEST_ASSERT_EQUAL_UINT32(2U, f.dispatchRadio.sendCount());

    // Decode the last sent frame (completion ACK) and verify the failure code.
    Frame ackFrame = {};
    TEST_ASSERT_TRUE(decode(f.dispatchRadio.lastFrame(),
                             f.dispatchRadio.lastFrameLen(), ackFrame));
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(MsgType::ACK),
                             static_cast<uint8_t>(ackFrame.type));
    TEST_ASSERT_GREATER_OR_EQUAL_UINT8(
        static_cast<uint8_t>(sizeof(AckPayload)), ackFrame.len);
    AckPayload ack = {};
    (void)memcpy(&ack, ackFrame.payload, sizeof(ack));
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(FailureCode::UNKNOWN_COMMAND), ack.failureCode);
}

/**
 * SET_MODE with only the CommandHeader (payload len == 2, mode byte missing):
 * executeCommand returns INVALID_PARAM.  Acceptance ACK sent; no FLAG_ACK_REQ.
 * Expected sendCount == 1.
 */
void test_cmd_nonfrag_set_mode_short_payload()
{
    CmdDispatchFixture f;
    // No extra bytes → frame.len == 2 < 3 required by SET_MODE.
    uint8_t wire[MAX_FRAME_LEN];
    const uint16_t len = make_cmd(wire, 54U, 0U, CommandId::SET_MODE);

    TEST_ASSERT_TRUE(f.dispatchRadio.injectBytes(wire, len));
    f.dispatcher.poll(1000U);

    TEST_ASSERT_EQUAL_UINT32(1U, f.dispatchRadio.sendCount());
}

/**
 * SET_FCS_ACTIVE with only the CommandHeader (frame.len == 2 < 3 required):
 * executeCommand returns INVALID_PARAM.
 * Expected sendCount == 1.
 */
void test_cmd_nonfrag_set_fcs_short_payload()
{
    CmdDispatchFixture f;
    uint8_t wire[MAX_FRAME_LEN];
    const uint16_t len = make_cmd(wire, 55U, 0U, CommandId::SET_FCS_ACTIVE);

    TEST_ASSERT_TRUE(f.dispatchRadio.injectBytes(wire, len));
    f.dispatcher.poll(1000U);

    TEST_ASSERT_EQUAL_UINT32(1U, f.dispatchRadio.sendCount());
}

/**
 * SET_FCS_ACTIVE with enabled value 2 (not 0 or 1): INVALID_PARAM.
 * Acceptance ACK sent; no FLAG_ACK_REQ.
 * Expected sendCount == 1.
 */
void test_cmd_nonfrag_set_fcs_invalid_value()
{
    CmdDispatchFixture f;
    const uint8_t extra[1U] = { 0x02U };  // 2 is invalid (only 0 or 1 accepted).
    uint8_t wire[MAX_FRAME_LEN];
    const uint16_t len = make_cmd(wire, 56U, 0U,
                                  CommandId::SET_FCS_ACTIVE, extra, 1U);

    TEST_ASSERT_TRUE(f.dispatchRadio.injectBytes(wire, len));
    f.dispatcher.poll(1000U);

    TEST_ASSERT_EQUAL_UINT32(1U, f.dispatchRadio.sendCount());
}

/**
 * SET_TELEM_INTERVAL with an out-of-range interval (50 ms, below minimum 100):
 * executeCommand returns INVALID_PARAM.  Acceptance ACK only.
 * Expected sendCount == 1.
 */
void test_cmd_nonfrag_set_telem_interval_out_of_range()
{
    CmdDispatchFixture f;
    const uint32_t intervalMs = 50U;  // Below minimum 100 ms.
    const uint8_t extra[4U] = {
        static_cast<uint8_t>( intervalMs        & 0xFFU),
        static_cast<uint8_t>((intervalMs >>  8U) & 0xFFU),
        static_cast<uint8_t>((intervalMs >> 16U) & 0xFFU),
        static_cast<uint8_t>((intervalMs >> 24U) & 0xFFU),
    };
    uint8_t wire[MAX_FRAME_LEN];
    const uint16_t len = make_cmd(wire, 57U, 0U,
                                  CommandId::SET_TELEM_INTERVAL, extra, 4U);

    TEST_ASSERT_TRUE(f.dispatchRadio.injectBytes(wire, len));
    f.dispatcher.poll(1000U);

    TEST_ASSERT_EQUAL_UINT32(1U, f.dispatchRadio.sendCount());
}

/**
 * SET_CONFIG_PARAM with payload shorter than the required 7 bytes
 * (CommandHeader(2) + paramId(1) + value_le32(4)): INVALID_PARAM.
 * Acceptance ACK only.
 * Expected sendCount == 1.
 */
void test_cmd_nonfrag_set_config_param_short_payload()
{
    CmdDispatchFixture f;
    // Provide only paramId (1 byte extra) → total payload = 3 < 7.
    const uint8_t extra[1U] = {
        static_cast<uint8_t>(ConfigParamId::TELEM_INTERVAL_MS)
    };
    uint8_t wire[MAX_FRAME_LEN];
    const uint16_t len = make_cmd(wire, 58U, 0U,
                                  CommandId::SET_CONFIG_PARAM, extra, 1U);

    TEST_ASSERT_TRUE(f.dispatchRadio.injectBytes(wire, len));
    f.dispatcher.poll(1000U);

    TEST_ASSERT_EQUAL_UINT32(1U, f.dispatchRadio.sendCount());
}

/**
 * Buffer starting with valid SYNC bytes but with a declared payload length
 * (0xFF) that exceeds MAX_PAYLOAD_LEN (200): processBuffer detects the
 * corrupt length field and skips the SYNC_0 byte (CERT-1).
 * Expected sendCount == 0.
 */
void test_buffer_corrupt_length_field_skips_sync()
{
    CmdDispatchFixture f;

    // Wire bytes: SYNC(4) + VER + FLAGS + NODE + TYPE + SEQ + PAYLOAD_LEN
    // PAYLOAD_LEN = 0xFF (255 > MAX_PAYLOAD_LEN 200) → totalLen would be 269.
    // Pad to MIN_FRAME_LEN (14) so processBuffer enters the loop.
    uint8_t buf[14U] = {};
    buf[0U]  = SYNC_0;           // 0xAE
    buf[1U]  = SYNC_1;           // 0x55
    buf[2U]  = SYNC_2;           // 0xC3
    buf[3U]  = SYNC_3;           // 0x1A
    buf[4U]  = PROTOCOL_VERSION; // 0x02
    buf[5U]  = 0x00U;            // flags
    buf[6U]  = NODE_ROCKET;      // 0x01
    buf[7U]  = static_cast<uint8_t>(MsgType::COMMAND);  // 0x03
    buf[8U]  = 0x01U;            // seq
    buf[9U]  = 0xFFU;            // payloadLen = 255 > MAX_PAYLOAD_LEN (200)
    // bytes 10-13: arbitrary CRC filler (not reached due to length check)

    TEST_ASSERT_TRUE(f.dispatchRadio.injectBytes(
        buf, static_cast<uint16_t>(sizeof(buf))));
    f.dispatcher.poll(1000U);

    TEST_ASSERT_EQUAL_UINT32(0U, f.dispatchRadio.sendCount());
}

/**
 * Two HEARTBEAT frames injected back-to-back in a single injectBytes call:
 * processBuffer consumes the first frame, shifts the remainder via memmove
 * (tail > 0 branch on line 154), then processes the second frame.
 * Expected sendCount == 2.
 */
void test_buffer_two_consecutive_heartbeats()
{
    CmdDispatchFixture f;

    uint8_t wire0[MAX_FRAME_LEN];
    uint8_t wire1[MAX_FRAME_LEN];
    const uint16_t len0 = make_heartbeat(wire0, 60U);
    const uint16_t len1 = make_heartbeat(wire1, 61U);
    TEST_ASSERT_GREATER_THAN_UINT16(0U, len0);
    TEST_ASSERT_GREATER_THAN_UINT16(0U, len1);

    uint8_t combined[MAX_FRAME_LEN * 2U] = {};
    (void)memcpy(combined, wire0, len0);
    (void)memcpy(combined + len0, wire1, len1);
    const uint16_t totalLen = static_cast<uint16_t>(len0 + len1);

    TEST_ASSERT_TRUE(f.dispatchRadio.injectBytes(combined, totalLen));
    f.dispatcher.poll(1000U);

    // Two HEARTBEAT responses: one per frame.
    TEST_ASSERT_EQUAL_UINT32(2U, f.dispatchRadio.sendCount());
}

/**
 * Nine VERIFY_CONFIG commands with FLAG_ACK_REQ sent in a single poll():
 * the first 8 fill the command queue; the 9th triggers the QUEUE_FULL NACK
 * (handleCommand → enqueueCmd fails → FLAG_ACK_REQ → sendAckNack QUEUE_FULL).
 *
 * Expected sendCount:
 *   9 acceptance ACKs  (handleCommand, one per command)
 * + 1 QUEUE_FULL NACK  (9th command, handleCommand queue-full branch)
 * + 8 completion ACKs  (drainCmdQueue, FLAG_ACK_REQ, result NONE)
 * = 18
 */
void test_cmd_queue_full_sends_nack()
{
    CmdDispatchFixture f;

    static constexpr uint8_t kBatchSize  = 9U;
    static constexpr uint8_t kFirstSeq  = 70U;

    // Build all 9 frames into one contiguous buffer.
    uint8_t combined[MAX_FRAME_LEN * kBatchSize] = {};
    uint16_t offset = 0U;

    for (uint8_t i = 0U; i < kBatchSize; ++i)
    {
        uint8_t wire[MAX_FRAME_LEN] = {};
        const uint16_t flen = make_cmd(
            wire,
            static_cast<uint8_t>(kFirstSeq + i),
            FLAG_ACK_REQ,
            CommandId::VERIFY_CONFIG);
        TEST_ASSERT_GREATER_THAN_UINT16(0U, flen);
        (void)memcpy(combined + offset, wire, flen);
        offset = static_cast<uint16_t>(offset + flen);
    }

    TEST_ASSERT_TRUE(f.dispatchRadio.injectBytes(combined, offset));
    f.dispatcher.poll(1000U);

    // 9 acceptance + 1 QUEUE_FULL NACK + 8 completion = 18.
    TEST_ASSERT_EQUAL_UINT32(18U, f.dispatchRadio.sendCount());
}

/**
 * SET_TELEM_INTERVAL with only 1 extra byte (frame.len == 3 < 6 required):
 * executeCommand hits the short-payload guard and returns INVALID_PARAM.
 * Acceptance ACK sent; no FLAG_ACK_REQ.
 * Expected sendCount == 1.
 */
void test_cmd_nonfrag_set_telem_interval_short_payload()
{
    CmdDispatchFixture f;
    // 1 extra byte → frame.len = 3 (CommandHeader=2 + 1), below the 6-byte minimum.
    const uint8_t extra[1U] = { 0x01U };
    uint8_t wire[MAX_FRAME_LEN];
    const uint16_t len = make_cmd(wire, 80U, 0U,
                                  CommandId::SET_TELEM_INTERVAL, extra, 1U);

    TEST_ASSERT_TRUE(f.dispatchRadio.injectBytes(wire, len));
    f.dispatcher.poll(1000U);

    TEST_ASSERT_EQUAL_UINT32(1U, f.dispatchRadio.sendCount());
}

/**
 * COMMAND with commandId 0x09: within the declared range [0x01..0x24] but
 * without a corresponding case in the switch → hits the default branch and
 * returns UNKNOWN_COMMAND.  FLAG_ACK_REQ triggers a completion ACK whose
 * failureCode must equal FailureCode::UNKNOWN_COMMAND.
 * (Note: 0x07=FIRE_PULSE_C and 0x08=FIRE_PULSE_D are now valid commands;
 *  0x09 is the first true gap in the enum.)
 * Expected sendCount == 2  (acceptance ACK + completion ACK/NACK).
 */
void test_cmd_nonfrag_gap_commandid_default_case()
{
    CmdDispatchFixture f;
    Frame tx = {};
    tx.ver        = PROTOCOL_VERSION;
    tx.node       = NODE_ROCKET;
    tx.type       = MsgType::COMMAND;
    tx.seq        = 81U;
    tx.flags      = FLAG_ACK_REQ;
    tx.payload[0] = static_cast<uint8_t>(Priority::PRI_LOW);
    tx.payload[1] = 0x09U;  // In range [0x01..0x24] but has no case in switch.
    tx.len        = 2U;

    uint8_t wire[MAX_FRAME_LEN];
    const uint16_t len = encode(tx, wire, sizeof(wire));
    TEST_ASSERT_GREATER_THAN_UINT16(0U, len);

    TEST_ASSERT_TRUE(f.dispatchRadio.injectBytes(wire, len));
    f.dispatcher.poll(1000U);

    // Acceptance ACK (handleCommand) + completion ACK/NACK (drainCmdQueue).
    TEST_ASSERT_EQUAL_UINT32(2U, f.dispatchRadio.sendCount());

    // Decode the last sent frame (completion ACK) and verify the failure code.
    Frame ackFrame = {};
    TEST_ASSERT_TRUE(decode(f.dispatchRadio.lastFrame(),
                             f.dispatchRadio.lastFrameLen(), ackFrame));
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(MsgType::ACK),
                             static_cast<uint8_t>(ackFrame.type));
    TEST_ASSERT_GREATER_OR_EQUAL_UINT8(
        static_cast<uint8_t>(sizeof(AckPayload)), ackFrame.len);
    AckPayload ack = {};
    (void)memcpy(&ack, ackFrame.payload, sizeof(ack));
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(FailureCode::UNKNOWN_COMMAND), ack.failureCode);
}

/**
 * SET_CONFIG_PARAM with an unknown paramId (0x07, past ConfigParamId::LAST
 * == 0x06): applyConfigParam calls findConfigParam() which returns nullptr,
 * returning INVALID_PARAM.  Acceptance ACK only.
 * Expected sendCount == 1.
 */
void test_cmd_nonfrag_set_config_unknown_paramid()
{
    CmdDispatchFixture f;

    const float value = 1000.0f;
    uint32_t raw = 0U;
    (void)memcpy(&raw, &value, sizeof(raw));

    const uint8_t extra[5U] = {
        0x07U,  // paramId = 0x07 (past ConfigParamId::LAST == 0x06)
        static_cast<uint8_t>( raw        & 0xFFU),
        static_cast<uint8_t>((raw >>  8U) & 0xFFU),
        static_cast<uint8_t>((raw >> 16U) & 0xFFU),
        static_cast<uint8_t>((raw >> 24U) & 0xFFU),
    };
    uint8_t wire[MAX_FRAME_LEN];
    const uint16_t len = make_cmd(wire, 82U, 0U,
                                  CommandId::SET_CONFIG_PARAM, extra, 5U);

    TEST_ASSERT_TRUE(f.dispatchRadio.injectBytes(wire, len));
    f.dispatcher.poll(1000U);

    TEST_ASSERT_EQUAL_UINT32(1U, f.dispatchRadio.sendCount());
}

/**
 * SET_CONFIG_PARAM with MONITOR_ALT_HIGH_M and a valid value (4000.0f):
 * applyConfigParam passes the range check, calls configureMonitorFromParam()
 * (no-op side effect in sim), and returns NONE.
 * Covers the success path (LOG_I + return NONE) in applyConfigParam.
 * Expected sendCount == 1 (acceptance ACK only, no FLAG_ACK_REQ).
 */
void test_cmd_nonfrag_set_config_param_monitor_valid()
{
    CmdDispatchFixture f;

    const float altM = 4000.0f;
    uint32_t raw = 0U;
    (void)memcpy(&raw, &altM, sizeof(raw));

    const uint8_t extra[5U] = {
        static_cast<uint8_t>(ConfigParamId::MONITOR_ALT_HIGH_M),
        static_cast<uint8_t>( raw        & 0xFFU),
        static_cast<uint8_t>((raw >>  8U) & 0xFFU),
        static_cast<uint8_t>((raw >> 16U) & 0xFFU),
        static_cast<uint8_t>((raw >> 24U) & 0xFFU),
    };
    uint8_t wire[MAX_FRAME_LEN];
    const uint16_t len = make_cmd(wire, 83U, 0U,
                                  CommandId::SET_CONFIG_PARAM, extra, 5U);

    TEST_ASSERT_TRUE(f.dispatchRadio.injectBytes(wire, len));
    f.dispatcher.poll(1000U);

    TEST_ASSERT_EQUAL_UINT32(1U, f.dispatchRadio.sendCount());
}

// ── FIRE_PULSE_B / C / D tests ────────────────────────────────────────────────

/**
 * FIRE_PULSE_B without FLAG_PRIORITY: isCritical guard rejects with
 * PRECONDITION_FAIL before entering the switch.  Acceptance ACK only.
 * Expected sendCount == 1.
 */
void test_cmd_nonfrag_fire_pulse_b_missing_priority_rejected()
{
    CmdDispatchFixture f;
    uint8_t wire[MAX_FRAME_LEN];
    const uint16_t len = make_cmd(wire, 84U, 0U, CommandId::FIRE_PULSE_B);
    TEST_ASSERT_GREATER_THAN_UINT16(0U, len);

    TEST_ASSERT_TRUE(f.dispatchRadio.injectBytes(wire, len));
    f.dispatcher.poll(1000U);

    TEST_ASSERT_EQUAL_UINT32(1U, f.dispatchRadio.sendCount());
}

/**
 * FIRE_PULSE_C without FLAG_PRIORITY: isCritical guard rejects with
 * PRECONDITION_FAIL before entering the switch.  Acceptance ACK only.
 * Expected sendCount == 1.
 */
void test_cmd_nonfrag_fire_pulse_c_missing_priority_rejected()
{
    CmdDispatchFixture f;
    uint8_t wire[MAX_FRAME_LEN];
    const uint16_t len = make_cmd(wire, 85U, 0U, CommandId::FIRE_PULSE_C);
    TEST_ASSERT_GREATER_THAN_UINT16(0U, len);

    TEST_ASSERT_TRUE(f.dispatchRadio.injectBytes(wire, len));
    f.dispatcher.poll(1000U);

    TEST_ASSERT_EQUAL_UINT32(1U, f.dispatchRadio.sendCount());
}

/**
 * FIRE_PULSE_D without FLAG_PRIORITY: isCritical guard rejects with
 * PRECONDITION_FAIL before entering the switch.  Acceptance ACK only.
 * Expected sendCount == 1.
 */
void test_cmd_nonfrag_fire_pulse_d_missing_priority_rejected()
{
    CmdDispatchFixture f;
    uint8_t wire[MAX_FRAME_LEN];
    const uint16_t len = make_cmd(wire, 86U, 0U, CommandId::FIRE_PULSE_D);
    TEST_ASSERT_GREATER_THAN_UINT16(0U, len);

    TEST_ASSERT_TRUE(f.dispatchRadio.injectBytes(wire, len));
    f.dispatcher.poll(1000U);

    TEST_ASSERT_EQUAL_UINT32(1U, f.dispatchRadio.sendCount());
}

/**
 * FIRE_PULSE_C with FLAG_PRIORITY but engine not RUNNING: executeCommand
 * returns PRECONDITION_FAIL.  Acceptance ACK sent; no FLAG_ACK_REQ.
 * Expected sendCount == 1.
 */
void test_cmd_nonfrag_fire_pulse_c_engine_not_running()
{
    CmdDispatchFixture f;
    uint8_t wire[MAX_FRAME_LEN];
    const uint16_t len = make_cmd(wire, 87U,
                                  FLAG_PRIORITY, CommandId::FIRE_PULSE_C);
    TEST_ASSERT_GREATER_THAN_UINT16(0U, len);

    TEST_ASSERT_TRUE(f.dispatchRadio.injectBytes(wire, len));
    f.dispatcher.poll(1000U);

    TEST_ASSERT_EQUAL_UINT32(1U, f.dispatchRadio.sendCount());
}

/**
 * FIRE_PULSE_D with FLAG_PRIORITY but engine not RUNNING: executeCommand
 * returns PRECONDITION_FAIL.  Acceptance ACK sent; no FLAG_ACK_REQ.
 * Expected sendCount == 1.
 */
void test_cmd_nonfrag_fire_pulse_d_engine_not_running()
{
    CmdDispatchFixture f;
    uint8_t wire[MAX_FRAME_LEN];
    const uint16_t len = make_cmd(wire, 88U,
                                  FLAG_PRIORITY, CommandId::FIRE_PULSE_D);
    TEST_ASSERT_GREATER_THAN_UINT16(0U, len);

    TEST_ASSERT_TRUE(f.dispatchRadio.injectBytes(wire, len));
    f.dispatcher.poll(1000U);

    TEST_ASSERT_EQUAL_UINT32(1U, f.dispatchRadio.sendCount());
}

// ── SET_TELEM_INTERVAL boundary tests ([M2]) ──────────────────────────────────

/**
 * SET_TELEM_INTERVAL with interval == TELEMETRY_INTERVAL_MIN (100 ms):
 * exactly at the lower bound → accepted by range check.
 * Engine has no active HK slots → setTelemInterval() returns false →
 * EXECUTION_ERROR.  FLAG_ACK_REQ triggers completion ACK.
 * Expected sendCount == 2; failureCode == EXECUTION_ERROR.
 */
void test_cmd_nonfrag_set_telem_interval_at_min()
{
    CmdDispatchFixture f;
    const uint32_t intervalMs = ares::TELEMETRY_INTERVAL_MIN;  // 100 ms
    const uint8_t extra[4U] = {
        static_cast<uint8_t>( intervalMs        & 0xFFU),
        static_cast<uint8_t>((intervalMs >>  8U) & 0xFFU),
        static_cast<uint8_t>((intervalMs >> 16U) & 0xFFU),
        static_cast<uint8_t>((intervalMs >> 24U) & 0xFFU),
    };
    uint8_t wire[MAX_FRAME_LEN];
    const uint16_t len = make_cmd(wire, 89U, FLAG_ACK_REQ,
                                  CommandId::SET_TELEM_INTERVAL, extra, 4U);

    TEST_ASSERT_TRUE(f.dispatchRadio.injectBytes(wire, len));
    f.dispatcher.poll(1000U);

    // Acceptance ACK + completion ACK (EXECUTION_ERROR: no HK slots).
    TEST_ASSERT_EQUAL_UINT32(2U, f.dispatchRadio.sendCount());

    Frame ackFrame = {};
    TEST_ASSERT_TRUE(decode(f.dispatchRadio.lastFrame(),
                             f.dispatchRadio.lastFrameLen(), ackFrame));
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(MsgType::ACK),
                             static_cast<uint8_t>(ackFrame.type));
    TEST_ASSERT_GREATER_OR_EQUAL_UINT8(
        static_cast<uint8_t>(sizeof(AckPayload)), ackFrame.len);
    AckPayload ack = {};
    (void)memcpy(&ack, ackFrame.payload, sizeof(ack));
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(FailureCode::EXECUTION_ERROR), ack.failureCode);
}

/**
 * SET_TELEM_INTERVAL with interval == TELEMETRY_INTERVAL_MAX (60 000 ms):
 * exactly at the upper bound → accepted by range check.
 * Engine has no active HK slots → setTelemInterval() returns false →
 * EXECUTION_ERROR.  FLAG_ACK_REQ triggers completion ACK.
 * Expected sendCount == 2; failureCode == EXECUTION_ERROR.
 */
void test_cmd_nonfrag_set_telem_interval_at_max()
{
    CmdDispatchFixture f;
    const uint32_t intervalMs = ares::TELEMETRY_INTERVAL_MAX;  // 60 000 ms
    const uint8_t extra[4U] = {
        static_cast<uint8_t>( intervalMs        & 0xFFU),
        static_cast<uint8_t>((intervalMs >>  8U) & 0xFFU),
        static_cast<uint8_t>((intervalMs >> 16U) & 0xFFU),
        static_cast<uint8_t>((intervalMs >> 24U) & 0xFFU),
    };
    uint8_t wire[MAX_FRAME_LEN];
    const uint16_t len = make_cmd(wire, 90U, FLAG_ACK_REQ,
                                  CommandId::SET_TELEM_INTERVAL, extra, 4U);

    TEST_ASSERT_TRUE(f.dispatchRadio.injectBytes(wire, len));
    f.dispatcher.poll(1000U);

    // Acceptance ACK + completion ACK (EXECUTION_ERROR: no HK slots).
    TEST_ASSERT_EQUAL_UINT32(2U, f.dispatchRadio.sendCount());

    Frame ackFrame = {};
    TEST_ASSERT_TRUE(decode(f.dispatchRadio.lastFrame(),
                             f.dispatchRadio.lastFrameLen(), ackFrame));
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(MsgType::ACK),
                             static_cast<uint8_t>(ackFrame.type));
    TEST_ASSERT_GREATER_OR_EQUAL_UINT8(
        static_cast<uint8_t>(sizeof(AckPayload)), ackFrame.len);
    AckPayload ack = {};
    (void)memcpy(&ack, ackFrame.payload, sizeof(ack));
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(FailureCode::EXECUTION_ERROR), ack.failureCode);
}

/**
 * SET_TELEM_INTERVAL with interval == TELEMETRY_INTERVAL_MAX + 1 (60 001 ms):
 * one above the upper bound → INVALID_PARAM.  FLAG_ACK_REQ triggers
 * completion ACK.
 * Expected sendCount == 2; failureCode == INVALID_PARAM.
 */
void test_cmd_nonfrag_set_telem_interval_above_max()
{
    CmdDispatchFixture f;
    const uint32_t intervalMs = ares::TELEMETRY_INTERVAL_MAX + 1U;  // 60 001 ms
    const uint8_t extra[4U] = {
        static_cast<uint8_t>( intervalMs        & 0xFFU),
        static_cast<uint8_t>((intervalMs >>  8U) & 0xFFU),
        static_cast<uint8_t>((intervalMs >> 16U) & 0xFFU),
        static_cast<uint8_t>((intervalMs >> 24U) & 0xFFU),
    };
    uint8_t wire[MAX_FRAME_LEN];
    const uint16_t len = make_cmd(wire, 91U, FLAG_ACK_REQ,
                                  CommandId::SET_TELEM_INTERVAL, extra, 4U);

    TEST_ASSERT_TRUE(f.dispatchRadio.injectBytes(wire, len));
    f.dispatcher.poll(1000U);

    // Acceptance ACK + completion ACK (INVALID_PARAM: out of range).
    TEST_ASSERT_EQUAL_UINT32(2U, f.dispatchRadio.sendCount());

    Frame ackFrame = {};
    TEST_ASSERT_TRUE(decode(f.dispatchRadio.lastFrame(),
                             f.dispatchRadio.lastFrameLen(), ackFrame));
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(MsgType::ACK),
                             static_cast<uint8_t>(ackFrame.type));
    TEST_ASSERT_GREATER_OR_EQUAL_UINT8(
        static_cast<uint8_t>(sizeof(AckPayload)), ackFrame.len);
    AckPayload ack = {};
    (void)memcpy(&ack, ackFrame.payload, sizeof(ack));
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(FailureCode::INVALID_PARAM), ack.failureCode);
}

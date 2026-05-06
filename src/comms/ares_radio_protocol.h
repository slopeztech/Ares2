/**
 * @file  ares_radio_protocol.h
 * @brief ARES telemetry frame protocol v2 — PUS-adapted (APUS-1..15).
 *
 * Transport-agnostic binary frame protocol for radio links.
 * Pure C++ with no Arduino dependency — compiles on native targets.
 * Adapted from ECSS-E-ST-70-41C (ESA Packet Utilisation Standard).
 *
 * Wire format:
 *   SYNC(4) + VER + FLAGS + NODE + TYPE + SEQ + LEN + PAYLOAD + CRC-32
 *
 * PUS service mapping (APUS-5):
 *   TYPE 0x01 TELEMETRY → ST[3]  Housekeeping
 *   TYPE 0x02 EVENT     → ST[5]  Event Reporting
 *   TYPE 0x03 COMMAND   → ST[8]  Function Management
 *   TYPE 0x04 ACK/NACK  → ST[1]  Request Verification
 *   TYPE 0x05 HEARTBEAT → ST[17] Connection Test
 *
 * No dynamic allocation (PO10-3).  All buffers are static.
 */
#pragma once

#include <cstdint>

namespace ares
{
namespace proto
{

// ── Protocol version ───────────────────────────────────────
constexpr uint8_t PROTOCOL_VERSION = 0x02;  ///< Wire protocol version (APUS-4.1).

// ── Frame sync bytes (4-byte marker, APUS-4.3) ────────────
constexpr uint8_t SYNC_0   = 0xAE;  ///< Sync preamble byte 0.
constexpr uint8_t SYNC_1   = 0x55;  ///< Sync preamble byte 1.
constexpr uint8_t SYNC_2   = 0xC3;  ///< Sync preamble byte 2.
constexpr uint8_t SYNC_3   = 0x1A;  ///< Sync preamble byte 3.
constexpr uint8_t SYNC_LEN = 4;     ///< Number of sync bytes in the frame header.

// ── Frame structure constants ──────────────────────────────
// Wire: SYNC(4) + VER+FLAGS+NODE+TYPE+SEQ+LEN + PAYLOAD + CRC32
constexpr uint8_t  HEADER_LEN      = 10;   ///< SYNC(4) + 6 header fields.
constexpr uint8_t  CRC_LEN         = 4;    ///< CRC-32 footer (APUS-1).
constexpr uint8_t  MAX_PAYLOAD_LEN = 200;  ///< Maximum payload bytes per frame.
constexpr uint16_t MAX_FRAME_LEN   = HEADER_LEN + MAX_PAYLOAD_LEN + CRC_LEN;  ///< Maximum wire frame size in bytes.
constexpr uint16_t MIN_FRAME_LEN   = HEADER_LEN + CRC_LEN;  ///< Minimum frame size (zero-payload).

// ── Node identification (APUS-10) ──────────────────────────
constexpr uint8_t NODE_BROADCAST  = 0x00;  ///< Broadcast address — all nodes.
constexpr uint8_t NODE_ROCKET     = 0x01;  ///< Flight computer node (APID 0x01).
constexpr uint8_t NODE_GROUND     = 0x02;  ///< Ground station node (APID 0x02).
constexpr uint8_t NODE_PAYLOAD    = 0x03;  ///< Payload bay node (APID 0x03).
constexpr uint8_t NODE_UNASSIGNED = 0xFF;  ///< Node ID not configured.

// ── Flags byte (APUS-4) ───────────────────────────────────
constexpr uint8_t FLAG_ACK_REQ    = 0x01;  ///< Bit 0: ACK required (APUS-9).
constexpr uint8_t FLAG_RETRANSMIT = 0x02;  ///< Bit 1: retransmission flag.
constexpr uint8_t FLAG_PRIORITY   = 0x04;  ///< Bit 2: high-priority frame (APUS-2).
constexpr uint8_t FLAG_FRAGMENT   = 0x08;  ///< Bit 3: fragmented transfer (APUS-15).
constexpr uint8_t FLAGS_RESERVED  = 0xF0;  ///< Bits 4–7: reserved, must be zero.

// ── Retransmission constants (APUS-4.5) ────────────────────
constexpr uint16_t ACK_TIMEOUT_MS = 1000;  ///< Wait for ACK before retry (ms).
constexpr uint8_t  MAX_RETRIES    = 3;     ///< Maximum retransmission attempts.

// ── Fragmentation constants (APUS-15) ──────────────────────
constexpr uint8_t  FRAG_HEADER_LEN  = 6;   ///< Fragmentation sub-header size in bytes.
constexpr uint8_t  MAX_FRAG_PAYLOAD = MAX_PAYLOAD_LEN - FRAG_HEADER_LEN;  ///< Maximum data bytes per fragment.
constexpr uint16_t MAX_FRAG_TOTAL   = 65535;  ///< Maximum number of fragments per transfer.

// ── PUS service types (APUS-5) ─────────────────────────────
// Maps ARES frame TYPE to ECSS-E-ST-70-41C service numbers.
/** PUS service type carried in the frame header (APUS-5). */
enum class MsgType : uint8_t
{
    NONE      = 0x00,   ///< Uninitialised sentinel — must not be transmitted.
    FIRST     = 0x01,
    TELEMETRY = 0x01,   ///< ST[3]  HK report (rocket → ground).
    EVENT     = 0x02,   ///< ST[5]  event report (rocket → ground).
    COMMAND   = 0x03,   ///< ST[8]  function management (ground → rocket).
    ACK       = 0x04,   ///< ST[1]  request verification (either direction).
    HEARTBEAT = 0x05,   ///< ST[17] connection test (either direction).
    LAST      = HEARTBEAT,
};

/**
 * Decoded APUS frame (APUS-3.5).
 * Statically allocated — no heap (PO10-3).
 */
struct Frame
{
    uint8_t  ver;                        ///< Protocol version.
    uint8_t  flags;                      ///< Frame flags (FLAG_* bitmask).
    uint8_t  node;                       ///< Sender node ID (APID).
    MsgType  type;                       ///< PUS service type.
    uint8_t  seq;                        ///< Rolling sequence number.
    uint8_t  len;                        ///< Payload length in bytes.
    uint8_t  payload[MAX_PAYLOAD_LEN];   ///< Raw payload bytes.
};

/**
 * Fragmentation sub-header embedded at the start of a fragment payload (APUS-15).
 */
struct FragHeader
{
    uint16_t transferId;     ///< Unique transfer session identifier.
    uint16_t segmentNum;     ///< Segment index (0-based).
    uint16_t totalSegments;  ///< Total segments in this transfer.
} __attribute__((packed));
static_assert(sizeof(FragHeader) == 6,
              "APUS-15: FragHeader must be 6 bytes");

/**
 * Command priority levels (APUS-2).
 */
enum class Priority : uint8_t
{
    FIRST        = 0,
    PRI_CRITICAL = 0,   // Abort, pyro fire
    PRI_HIGH     = 1,   // Flight control, FCS overrides
    PRI_NORMAL   = 2,   // Sensor requests, config changes
    PRI_LOW      = 3,   // Logging, diagnostics, heartbeat
    LAST         = PRI_LOW,
};

/**
 * Command identifiers for ST[8] function management (APUS-7).
 */
enum class CommandId : uint8_t
{
    FIRST              = 0x01,
    ARM_FLIGHT         = 0x01,
    ABORT              = 0x02,
    FIRE_PYRO_A        = 0x03,
    FIRE_PYRO_B        = 0x04,
    SET_MODE           = 0x05,
    SET_FCS_ACTIVE     = 0x06,
    REQUEST_TELEMETRY  = 0x10,
    SET_TELEM_INTERVAL = 0x11,
    REQUEST_STATUS     = 0x20,
    REQUEST_CONFIG     = 0x21,
    SET_CONFIG_PARAM   = 0x22,
    VERIFY_CONFIG      = 0x23,
    FACTORY_RESET      = 0x24,
    LAST               = FACTORY_RESET,
};

/**
 * Event severity codes for ST[5] event reporting (APUS-8).
 */
enum class EventSeverity : uint8_t
{
    FIRST = 0,
    INFO  = 0,
    WARN  = 1,
    ERR   = 2,
    LAST  = ERR,
};

/**
 * Event identifiers for ST[5] event reporting (APUS-8).
 */
enum class EventId : uint8_t
{
    FIRST           = 0x01,
    MODE_CHANGE     = 0x01,
    PHASE_CHANGE    = 0x02,
    FCS_RULE_FIRED  = 0x03,
    PYRO_FIRED      = 0x04,
    ABORT_TRIGGERED = 0x05,
    SENSOR_FAILURE  = 0x06,
    CRC_FAILURE     = 0x07,
    FPL_VIOLATION   = 0x08,
    LINK_LOST       = 0x09,
    LINK_RESTORED   = 0x0A,
    LAST            = LINK_RESTORED,
};

/**
 * Failure codes for ST[1] ACK/NACK request verification (APUS-9).
 */
enum class FailureCode : uint8_t
{
    FIRST             = 0x00,
    NONE              = 0x00,  // success (ACK)
    CRC_INVALID       = 0x01,
    UNKNOWN_TYPE      = 0x02,
    UNKNOWN_COMMAND   = 0x03,
    PRECONDITION_FAIL = 0x04,
    EXECUTION_ERROR   = 0x05,
    QUEUE_FULL        = 0x06,
    INVALID_PARAM     = 0x07,
    ROUTING_FAIL      = 0x08,  ///< Packet could not be routed to any handler (APUS-14.3).
    LAST              = ROUTING_FAIL,
};

/**
 * Compact system status byte constants (APUS-3.2).
 * Explicit mask constants replace bit-fields for MISRA-13.3 compliance.
 */
static constexpr uint8_t STATUS_ARMED        = 0x01U;  ///< Bit 0: flight mode armed.
static constexpr uint8_t STATUS_FCS_ACTIVE   = 0x02U;  ///< Bit 1: flight control system active.
static constexpr uint8_t STATUS_GPS_VALID    = 0x04U;  ///< Bit 2: GPS fix valid.
static constexpr uint8_t STATUS_PYRO_A_FIRED = 0x08U;  ///< Bit 3: pyro channel A fired.
static constexpr uint8_t STATUS_PYRO_B_FIRED = 0x10U;  ///< Bit 4: pyro channel B fired.
static constexpr uint8_t STATUS_DELTA_FRAME  = 0x20U;  ///< Bit 5: delta-encoded frame (APUS-3.3).
static constexpr uint8_t STATUS_RESERVED     = 0xC0U;  ///< Bits 6–7: reserved, must be zero.

/// Status byte type alias (APUS-3.2).
using StatusBits = uint8_t;
static_assert(sizeof(StatusBits) == 1U, "APUS-3.6: StatusBits must be 1 byte");

/**
 * Housekeeping telemetry payload for ST[3] HK reports (APUS-6).
 * All multi-byte fields are in little-endian (native) byte order.
 * PTC/PFC codes follow APUS-11 (ECSS-E-ST-70-41C §7.3).
 */
struct TelemetryPayload
{
    uint32_t   timestampMs;     ///< Millisecond uptime at sample time (APUS-13). PTC=3, PFC=14
    uint8_t    flightPhase;     ///< Current flight phase index.                  PTC=2, PFC=8
    StatusBits statusBits;      ///< Packed system status flags (APUS-3.2).       PTC=6, PFC=8
    float      altitudeAglM;    ///< Altitude above ground level in metres.       PTC=5, PFC=1
    float      verticalVelMs;   ///< Vertical velocity in m/s.                    PTC=5, PFC=1
    float      accelMag;        ///< Accelerometer magnitude in m/s².             PTC=5, PFC=1
    float      pressurePa;      ///< Barometric pressure in Pascals.              PTC=5, PFC=1
    float      temperatureC;    ///< Temperature in degrees Celsius.              PTC=5, PFC=1
    int32_t    latitudeE7;      ///< Latitude × 10⁷ (degrees × 1e7).             PTC=4, PFC=14
    int32_t    longitudeE7;     ///< Longitude × 10⁷ (degrees × 1e7).            PTC=4, PFC=14
    uint16_t   gpsAltDm;        ///< GPS altitude in decimetres.                  PTC=3, PFC=12
    uint8_t    gpsSats;         ///< Number of GPS satellites in use.             PTC=3, PFC=8
    uint8_t    batteryPct;      ///< Battery level percentage (0–100).            PTC=3, PFC=8
} __attribute__((packed));
static_assert(sizeof(TelemetryPayload) == 38,
              "APUS-3.6: TelemetryPayload must be 38 bytes");

/**
 * Command payload header for ST[8] function management (APUS-7).
 */
struct CommandHeader
{
    uint8_t priority;       ///< Priority level (Priority enum value).
    uint8_t commandId;      ///< Command identifier (CommandId enum value).
} __attribute__((packed));
static_assert(sizeof(CommandHeader) == 2,
              "APUS-3.6: CommandHeader must be 2 bytes");

/**
 * Event payload header for ST[5] event reporting (APUS-8).
 */
struct EventHeader
{
    uint32_t timestampMs;   ///< Millisecond uptime at event time.
    uint8_t  severity;      ///< Severity level (EventSeverity enum value).
    uint8_t  eventId;       ///< Event identifier (EventId enum value).
} __attribute__((packed));
static_assert(sizeof(EventHeader) == 6,
              "APUS-3.6: EventHeader must be 6 bytes");

/**
 * ACK/NACK payload for ST[1] request verification (APUS-9).
 */
struct AckPayload
{
    uint8_t  originalSeq;   ///< Sequence number of the original TC.
    uint8_t  originalNode;  ///< Node ID of the original TC sender.
    uint8_t  failureCode;   ///< Failure code (FailureCode enum; 0 = success).
    uint8_t  failureData;   ///< Command-specific diagnostic byte.
} __attribute__((packed));
static_assert(sizeof(AckPayload) == 4,
              "APUS-3.6: AckPayload must be 4 bytes");

// ── Functions ──────────────────────────────────────────────

/**
 * CRC-32 (Ethernet polynomial 0xEDB88320, init 0xFFFFFFFF).
 * Bytewise computation — no lookup table (APUS-1.4).
 */
uint32_t crc32(const uint8_t* data, uint16_t len);

/**
 * Encode a Frame into a wire buffer.
 * @param[in]  frame   Populated frame (ver, flags, node, type, seq, len, payload).
 * @param[out] buf     Destination wire buffer.
 * @param[in]  bufLen  Size of @p buf in bytes.
 * @return Total bytes written (HEADER + payload + CRC), or 0 on error.
 */
uint16_t encode(const Frame& frame, uint8_t* buf, uint16_t bufLen);

/**
 * Decode a wire buffer into a Frame.
 * @param[in]  buf     Wire buffer starting at SYNC_0.
 * @param[in]  bufLen  Available bytes in @p buf.
 * @param[out] frame   Populated on success.
 * @return true if sync, version, length, and CRC are valid.
 */
bool decode(const uint8_t* buf, uint16_t bufLen, Frame& frame);

/**
 * Extract fragmentation sub-header from a frame's payload.
 * @param[in]  frame  Decoded frame.
 * @param[out] frag   Populated on success.
 * @return false if FLAG_FRAGMENT is not set or payload is too short.
 */
bool decodeFrag(const Frame& frame, FragHeader& frag);

/**
 * Check if SEQ is a duplicate (same as lastSeq).
 * @return true if @p seq equals @p lastSeq.
 */
bool isDuplicate(uint8_t seq, uint8_t lastSeq);

// ── ST[20] parameter management (APUS-16) ────────────────────────────────────

/**
 * On-board parameter identifiers for SET_CONFIG_PARAM / REQUEST_CONFIG.
 * Mapped to fields configurable at runtime (APUS-16.3: constexpr table).
 */
enum class ConfigParamId : uint8_t
{
    FIRST                = 0x01,
    TELEM_INTERVAL_MS    = 0x01,  ///< Telemetry period in ms (uint32 via float).
    MONITOR_ALT_HIGH_M   = 0x02,  ///< Altitude alarm high threshold (m).
    MONITOR_ALT_LOW_M    = 0x03,  ///< Altitude alarm low threshold (m).
    MONITOR_ACCEL_HIGH   = 0x04,  ///< Accel magnitude alarm threshold (m/s²).
    MONITOR_TEMP_HIGH_C  = 0x05,  ///< Temperature alarm high threshold (°C).
    MONITOR_TEMP_LOW_C   = 0x06,  ///< Temperature alarm low threshold (°C).
    LAST                 = 0x06,
};

// ── ST[12] on-board monitoring (APUS-12) ─────────────────────────────────────

/** Monitored parameter identifiers — maps to TelemetryPayload fields (APUS-12.1). */
enum class MonitorParamId : uint8_t
{
    ALTITUDE_AGL_M  = 0x01,  ///< Altitude above ground level (m).
    VERTICAL_VEL_MS = 0x02,  ///< Vertical velocity, positive = up (m/s).
    ACCEL_MAG       = 0x03,  ///< Inertial acceleration magnitude (m/s²).
    PRESSURE_PA     = 0x04,  ///< Barometric pressure (Pa).
    TEMPERATURE_C   = 0x05,  ///< Board / baro temperature (°C).
};

/** APUS-12 monitoring state machine states. */
enum class MonitoringState : uint8_t
{
    MON_DISABLED = 0U,  ///< Monitor disabled — evaluation skipped (APUS-12.5).
    MON_ENABLED  = 1U,  ///< Monitor active, no limit violation detected.
    MON_ALARM    = 2U,  ///< Limit exceeded for consecutiveRequired samples.
};

} // namespace proto
} // namespace ares

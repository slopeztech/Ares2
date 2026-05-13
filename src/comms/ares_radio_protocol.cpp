/**
 * @file  ares_radio_protocol.cpp
 * @brief Implementation of the ARES PUS-adapted radio protocol v2.
 *
 * All encode/decode functions are reentrant and allocate nothing on
 * the heap (PO10-3).  CRC-32 uses the Ethernet reflected polynomial
 * with bytewise computation (no LUT, APUS-1.4).
 */

#include "ares_radio_protocol.h"
#include "debug/ares_log.h"

#include <cstring>

namespace ares
{
namespace proto
{

static constexpr const char* TAG_PROTO = "PROTO";

// ── CRC-32 (APUS-1) ──────────────────────────────────────
// Ethernet polynomial 0xEDB88320 (reflected).
// Bytewise bitwise computation — deterministic, no table needed.
uint32_t crc32(const uint8_t* data, uint16_t len)
{
    uint32_t crc = 0xFFFFFFFFU;
    for (uint16_t i = 0; i < len; ++i)
    {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8; ++bit)
        {
            if ((crc & 1U) != 0U)
            {
                crc = (crc >> 1U) ^ 0xEDB88320U;
            }
            else
            {
                crc >>= 1U;
            }
        }
    }
    return crc ^ 0xFFFFFFFFU;
}

// ── Encode ────────────────────────────────────────────────
uint16_t encode(const Frame& frame, uint8_t* buf, uint16_t bufLen)
{
    // Validate inputs before computing derived values (CERT-1)
    if (buf == nullptr)                        { return 0; }
    if (frame.len > MAX_PAYLOAD_LEN)           { return 0; }
    if ((frame.flags & FLAGS_RESERVED) != 0U)  { return 0; } // APUS-4.2

    const uint16_t totalLen =
        static_cast<uint16_t>(HEADER_LEN) + frame.len + CRC_LEN;

    if (bufLen < totalLen)                     { return 0; }

    // Sync marker (APUS-4.3)
    buf[0] = SYNC_0;
    buf[1] = SYNC_1;
    buf[2] = SYNC_2;
    buf[3] = SYNC_3;

    // Header fields
    buf[4] = frame.ver;
    buf[5] = frame.flags;
    buf[6] = frame.node;
    buf[7] = static_cast<uint8_t>(frame.type);
    buf[8] = frame.seq;
    buf[9] = frame.len;

    // Payload (safe: len ≤ MAX_PAYLOAD_LEN checked above)
    if (frame.len > 0)
    {
        memcpy(&buf[HEADER_LEN], frame.payload, frame.len);
    }

    // CRC-32 over VER..PAYLOAD (APUS-1.2: excludes SYNC marker).
    // Scope: bytes [SYNC_LEN .. HEADER_LEN + len - 1].
    const uint16_t crcDataLen =
        static_cast<uint16_t>(HEADER_LEN - SYNC_LEN) + frame.len;
    const uint32_t checksum = crc32(&buf[SYNC_LEN], crcDataLen);

    const uint16_t crcOffset = HEADER_LEN + frame.len;
    buf[crcOffset]     = static_cast<uint8_t>( checksum         & 0xFFU);
    buf[crcOffset + 1] = static_cast<uint8_t>((checksum >> 8U)  & 0xFFU);
    buf[crcOffset + 2] = static_cast<uint8_t>((checksum >> 16U) & 0xFFU);
    buf[crcOffset + 3] = static_cast<uint8_t>((checksum >> 24U) & 0xFFU);

    return totalLen;
}

static bool hasValidSync(const uint8_t* buf)
{
    return buf[0] == SYNC_0 && buf[1] == SYNC_1
        && buf[2] == SYNC_2 && buf[3] == SYNC_3;
}

static bool isKnownNode(uint8_t node)
{
    return node == NODE_BROADCAST || node == NODE_ROCKET
        || node == NODE_GROUND || node == NODE_PAYLOAD;
}

static bool isKnownMsgType(uint8_t rawType)
{
    return rawType >= static_cast<uint8_t>(MsgType::FIRST)
        && rawType <= static_cast<uint8_t>(MsgType::LAST);
}

static bool meetsMinPayloadLen(uint8_t rawType, uint8_t payloadLen)
{
    // Minimum payload size per TYPE (APUS-5.3) — index = rawType − FIRST
    static const uint8_t kMinPayload[] = {
        static_cast<uint8_t>(sizeof(TelemetryPayload)),  // 0x01 TELEMETRY
        static_cast<uint8_t>(sizeof(EventHeader)),       // 0x02 EVENT
        static_cast<uint8_t>(sizeof(CommandHeader)),     // 0x03 COMMAND
        static_cast<uint8_t>(sizeof(AckPayload)),        // 0x04 ACK
        0U,                                               // 0x05 HEARTBEAT
    };

    const uint8_t typeIdx = rawType - static_cast<uint8_t>(MsgType::FIRST);
    return payloadLen >= kMinPayload[typeIdx];
}

static bool crcMatches(const uint8_t* buf, uint8_t payloadLen)
{
    // Verify CRC-32 (APUS-1.2: scope = VER..PAYLOAD, excludes SYNC)
    const uint16_t crcDataLen =
        static_cast<uint16_t>(HEADER_LEN - SYNC_LEN) + payloadLen;
    const uint32_t computed = crc32(&buf[SYNC_LEN], crcDataLen);

    const uint16_t crcOffset = HEADER_LEN + payloadLen;
    const uint32_t received  =
        static_cast<uint32_t>(buf[crcOffset]) |
        (static_cast<uint32_t>(buf[crcOffset + 1]) << 8U)  |
        (static_cast<uint32_t>(buf[crcOffset + 2]) << 16U) |
        (static_cast<uint32_t>(buf[crcOffset + 3]) << 24U);

    return computed == received;
}

static void populateFrameFromBuffer(const uint8_t* buf,
                                    uint8_t        rawType,
                                    uint8_t        payloadLen,
                                    Frame&         frame)
{
    frame.ver   = buf[4];
    frame.flags = buf[5];
    frame.node  = buf[6];
    frame.type  = static_cast<MsgType>(rawType);
    frame.seq   = buf[8];
    frame.len   = payloadLen;

    if (payloadLen > 0)
    {
        memcpy(frame.payload, &buf[HEADER_LEN], payloadLen);
    }
}

// ── Decode ────────────────────────────────────────────────

/// Validates all wire-frame preconditions before populating a Frame.
/// On success *payloadLen and *rawType are set from the buffer.
/// CERT-1: all inputs validated before use; no side-effects on failure.
static bool validateDecodeHeader(const uint8_t* buf,
                                 uint16_t       bufLen,
                                 uint8_t&       payloadLen,
                                 uint8_t&       rawType)
{
    // Verify sync marker (APUS-4.3)
    if (!hasValidSync(buf)) { return false; }

    // Check protocol version (APUS-14.2)
    if (buf[4] != PROTOCOL_VERSION)
    {
        LOG_D(TAG_PROTO, "decode reject: version=0x%02X expected=0x%02X",
              static_cast<unsigned>(buf[4]),
              static_cast<unsigned>(PROTOCOL_VERSION));
        return false;
    }

    // Reject reserved flag bits (APUS-4.2)
    if ((buf[5] & FLAGS_RESERVED) != 0U)
    {
        LOG_D(TAG_PROTO, "decode reject: reserved flags set 0x%02X",
              static_cast<unsigned>(buf[5] & FLAGS_RESERVED));
        return false;
    }

    // Validate NODE against the APID table (APUS-10.1, APUS-10.2)
    if (!isKnownNode(buf[6]))
    {
        LOG_D(TAG_PROTO, "decode reject: unknown node=0x%02X",
              static_cast<unsigned>(buf[6]));
        return false;
    }

    payloadLen = buf[9];
    if (payloadLen > MAX_PAYLOAD_LEN)
    {
        LOG_D(TAG_PROTO, "decode reject: payloadLen=%u > MAX=%u",
              static_cast<unsigned>(payloadLen),
              static_cast<unsigned>(MAX_PAYLOAD_LEN));
        return false;
    }

    rawType = buf[7];
    if (!isKnownMsgType(rawType))
    {
        LOG_D(TAG_PROTO, "decode reject: unknown type=0x%02X",
              static_cast<unsigned>(rawType));
        return false;
    }

    // Fragmented frames carry FragHeader + segment data instead of the nominal
    // type payload, so the type-specific minimum size does not apply (APUS-15.1).
    const bool isFragment = ((buf[5] & FLAG_FRAGMENT) != 0U);
    if (isFragment)
    {
        if (payloadLen < FRAG_HEADER_LEN)
        {
            LOG_D(TAG_PROTO, "decode reject: fragment payloadLen=%u < FRAG_HEADER_LEN",
                  static_cast<unsigned>(payloadLen));
            return false;
        }
    }
    else if (!meetsMinPayloadLen(rawType, payloadLen))
    {
        LOG_D(TAG_PROTO, "decode reject: type=0x%02X payloadLen=%u below minimum",
              static_cast<unsigned>(rawType),
              static_cast<unsigned>(payloadLen));
        return false;
    }

    const uint16_t totalLen =
        static_cast<uint16_t>(HEADER_LEN) + payloadLen + CRC_LEN;
    if (bufLen < totalLen) { return false; }  // incomplete — caller retries

    if (!crcMatches(buf, payloadLen))
    {
        LOG_D(TAG_PROTO, "decode reject: CRC mismatch seq=0x%02X type=0x%02X",
              static_cast<unsigned>(buf[8]),
              static_cast<unsigned>(rawType));
        return false;
    }

    return true;
}

bool decode(const uint8_t* buf, uint16_t bufLen, Frame& frame)
{
    if (buf == nullptr)         { return false; }
    if (bufLen < MIN_FRAME_LEN) { return false; }

    uint8_t payloadLen = 0U;
    uint8_t rawType    = 0U;
    if (!validateDecodeHeader(buf, bufLen, payloadLen, rawType)) { return false; }

    // Populate frame (APUS-3.5: static struct, no heap)
    populateFrameFromBuffer(buf, rawType, payloadLen, frame);
    return true;
}

// ── Fragmentation sub-header encode (APUS-15) ────────────
bool encodeFrag(Frame&         frame,
                uint16_t       transferId,
                uint16_t       segmentNum,
                uint16_t       totalSegments,
                const uint8_t* data,
                uint8_t        dataLen)
{
    if (data == nullptr)              { return false; }
    if (dataLen > MAX_FRAG_PAYLOAD)   { return false; }
    if (totalSegments == 0U)          { return false; }
    if (segmentNum >= totalSegments)  { return false; }

    // Set FLAG_FRAGMENT (preserve any existing flags already set by caller).
    frame.flags = static_cast<uint8_t>(frame.flags | FLAG_FRAGMENT);

    // Write FragHeader in little-endian byte order (APUS-15).
    frame.payload[0] = static_cast<uint8_t>(transferId        & 0xFFU);
    frame.payload[1] = static_cast<uint8_t>(transferId   >> 8U);
    frame.payload[2] = static_cast<uint8_t>(segmentNum        & 0xFFU);
    frame.payload[3] = static_cast<uint8_t>(segmentNum   >> 8U);
    frame.payload[4] = static_cast<uint8_t>(totalSegments     & 0xFFU);
    frame.payload[5] = static_cast<uint8_t>(totalSegments >> 8U);

    // Append segment data after the sub-header (safe: dataLen <= MAX_FRAG_PAYLOAD).
    if (dataLen > 0U)
    {
        (void)memcpy(&frame.payload[FRAG_HEADER_LEN], data, dataLen);
    }

    frame.len = static_cast<uint8_t>(FRAG_HEADER_LEN + dataLen);
    return true;
}

// ── Fragmentation sub-header decode (APUS-15) ────────────
bool decodeFrag(const Frame& frame, FragHeader& frag)
{
    if ((frame.flags & FLAG_FRAGMENT) == 0U) { return false; }
    if (frame.len < FRAG_HEADER_LEN)        { return false; }

    // Read uint16_t fields in little-endian order (APUS-15)
    frag.transferId    = static_cast<uint16_t>(frame.payload[0])
                       | (static_cast<uint16_t>(frame.payload[1]) << 8U);
    frag.segmentNum    = static_cast<uint16_t>(frame.payload[2])
                       | (static_cast<uint16_t>(frame.payload[3]) << 8U);
    frag.totalSegments = static_cast<uint16_t>(frame.payload[4])
                       | (static_cast<uint16_t>(frame.payload[5]) << 8U);

    // Validate fragment header (APUS-15.3)
    if (frag.totalSegments == 0)                  { return false; }
    if (frag.segmentNum >= frag.totalSegments)    { return false; }

    return true;
}

// ── Duplicate detection (APUS-4.6) ───────────────────────
bool isDuplicate(uint8_t seq, uint8_t lastSeq)
{
    return seq == lastSeq;
}

} // namespace proto
} // namespace ares

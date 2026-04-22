# ARES Radio Protocol v2

**Transport-agnostic telemetry frame protocol**

Source: [src/comms/ares_radio_protocol.h](../src/comms/ares_radio_protocol.h),
        [src/comms/ares_radio_protocol.cpp](../src/comms/ares_radio_protocol.cpp)

---

## Overview

Binary frame protocol for telemetry links. Transport-agnostic — designed
to work over LoRa, nRF24, or any other radio backend. Pure C++ with
no Arduino dependencies — compiles and tests on native (desktop) targets.

Key properties:
- 4-byte sync marker for frame alignment
- CRC-32 integrity check (Ethernet polynomial)
- Fixed maximum frame size (214 bytes)
- Static buffers only — no dynamic allocation (PO10-3)
- Support for ACK, retransmission, and fragmentation

---

## Wire Format

```
+------+------+------+------+-----+-------+------+------+-----+-----+---------+---------+
|SYNC_0|SYNC_1|SYNC_2|SYNC_3| VER | FLAGS | NODE | TYPE | SEQ | LEN | PAYLOAD | CRC-32  |
| 0xAE | 0x55 | 0xC3 | 0x1A |     |       |      |      |     |     | 0..200  | 4 bytes |
+------+------+------+------+-----+-------+------+------+-----+-----+---------+---------+
 byte 0  1      2      3      4     5       6      7      8     9    10..       ...
```

| Field   | Offset | Size    | Description                              |
|---------|--------|---------|------------------------------------------|
| SYNC    | 0      | 4 bytes | `0xAE 0x55 0xC3 0x1A` — frame marker    |
| VER     | 4      | 1 byte  | Protocol version (`0x02`)                |
| FLAGS   | 5      | 1 byte  | Bit flags (see below)                    |
| NODE    | 6      | 1 byte  | Sender node ID                           |
| TYPE    | 7      | 1 byte  | Message type                             |
| SEQ     | 8      | 1 byte  | Sequence number (0–255, wrapping)        |
| LEN     | 9      | 1 byte  | Payload length (0–200)                   |
| PAYLOAD | 10     | 0–200   | Message-specific data                    |
| CRC-32  | 10+LEN | 4 bytes | CRC over bytes [4 .. 9+LEN]              |

### Size Constraints

| Constant          | Value | Description              |
|-------------------|-------|--------------------------|
| `HEADER_LEN`      | 10    | SYNC(4) + header fields  |
| `CRC_LEN`         | 4     | CRC-32                   |
| `MAX_PAYLOAD_LEN` | 200   | Maximum payload bytes    |
| `MAX_FRAME_LEN`   | 214   | Header + max payload + CRC |
| `MIN_FRAME_LEN`   | 14    | Header + CRC (empty payload) |

---

## Flags Byte

| Bit | Mask   | Name             | Description                  |
|-----|--------|------------------|------------------------------|
| 0   | `0x01` | `FLAG_ACK_REQ`   | ACK required from receiver   |
| 1   | `0x02` | `FLAG_RETRANSMIT`| This is a retransmission     |
| 2   | `0x04` | `FLAG_PRIORITY`  | High-priority frame          |
| 3   | `0x08` | `FLAG_FRAGMENT`  | Fragmented frame             |
| 4–7 | `0xF0` | Reserved         | Must be zero                 |

---

## Message Types (PUS service mapping)

Frame `TYPE` maps to ECSS-E-ST-70-41C (PUS) service numbers per APUS-5:

| Value  | Enum        | PUS Service | Direction        | Description                |
|--------|-------------|-------------|------------------|----------------------------|
| `0x01` | TELEMETRY   | ST[3]  HK   | Rocket → Ground  | Periodic housekeeping data |
| `0x02` | EVENT       | ST[5]  ER   | Rocket → Ground  | Phase transition / alert   |
| `0x03` | COMMAND     | ST[8]  FM   | Ground → Rocket  | Function management        |
| `0x04` | ACK         | ST[1]  RV   | Either           | Request verification       |
| `0x05` | HEARTBEAT   | ST[17] CT   | Either           | Connection test / keepalive|

---

## Node Addressing

| Value  | Constant          | Description             |
|--------|-------------------|-------------------------|
| `0x00` | `NODE_BROADCAST`  | Address all nodes       |
| `0x01` | `NODE_ROCKET`     | Flight computer (APID 0x01) |
| `0x02` | `NODE_GROUND`     | Ground station (APID 0x02) |
| `0x03` | `NODE_PAYLOAD`    | Payload bay node (APID 0x03) |
| `0xFF` | `NODE_UNASSIGNED` | Node ID not configured  |

---

## CRC-32

- Polynomial: `0xEDB88320` (Ethernet, reflected)
- Initial value: `0xFFFFFFFF`
- Final XOR: `0xFFFFFFFF`
- Scope: bytes [4 .. 9+LEN] (VER through end of PAYLOAD)
- Implementation: bytewise computation (no lookup table, saves flash)

Test vector: `"123456789"` → `0xCBF43926`

---

## Fragmentation

When `FLAG_FRAGMENT` is set, the first 6 bytes of payload form a
fragmentation sub-header (`FRAG_HEADER_LEN = 6`):

| Offset | Size   | Field       | Description                      |
|--------|--------|-------------|----------------------------------|
| 0      | 1 byte | `fragId`    | Groups related fragments         |
| 1      | 2 bytes| `fragIndex` | 0-based index within the group   |
| 3      | 2 bytes| `fragTotal` | Total number of fragments        |
| 5      | 1 byte | (reserved)  | Padding to 6-byte alignment      |

Constraints:
- `fragTotal` must be ≥ 1
- `fragIndex` must be < `fragTotal`
- Max useful data per fragment: `MAX_FRAG_PAYLOAD` = 194 bytes (`MAX_PAYLOAD_LEN` − `FRAG_HEADER_LEN`)

---

## Retransmission

When `FLAG_ACK_REQ` is set:
- Sender waits up to `ACK_TIMEOUT_MS` (1000 ms) for an ACK
- Retransmit up to `MAX_RETRIES` (3) times with `FLAG_RETRANSMIT` set
- Receiver uses `isDuplicate(seq, lastSeq)` to filter retransmissions

---

## API Functions

```cpp
// CRC-32 computation
uint32_t crc32(const uint8_t* data, uint16_t len);

// Encode Frame → wire buffer. Returns total bytes or 0 on error.
uint16_t encode(const Frame& frame, uint8_t* buf, uint16_t bufLen);

// Decode wire buffer → Frame. Returns true if valid.
bool decode(const uint8_t* buf, uint16_t bufLen, Frame& frame);

// Extract fragmentation header from payload.
bool decodeFrag(const Frame& frame, FragHeader& frag);

// Check for duplicate sequence number.
bool isDuplicate(uint8_t seq, uint8_t lastSeq);
```

---

## Testing

The protocol has 30 unit tests running on the **native** platform
(desktop, no hardware required):

```bash
pio test -e native
```

Test categories:
- CRC-32 known vectors and edge cases
- Encode/decode roundtrip (empty, normal, max payload, with flags)
- Decode rejection (bad sync, bad version, reserved flags, bad CRC,
  truncated, null buffer, too short)
- Encode edge cases (null buffer, buffer too small, oversized payload,
  reserved flags)
- Wire format verification (sync bytes, header field positions)
- Fragmentation (valid, no flag, payload too short, index out of range,
  total zero)
- Duplicate detection (same seq, different seq, wraparound)

# ARES APUS Standard — Amateur Packet Utilisation Standard

**Amateur Rocket Embedded System**
Communications protocol rules for packet integrity, command
prioritisation, data encoding, and RF link discipline.
Derived from ECSS-E-ST-70-41C (ESA Packet Utilisation Standard),
adapted for amateur rocketry on ESP32-S3.
Companion to the [ARES Coding Standard](ares_coding_standard.md)
and the [ARES Radio Protocol v2](../architecture/ares_radio_protocol.md).

---

## Context

ARES communicates over a low-power radio link in a low-bandwidth,
high-noise environment where every byte matters. The physical layer
is **not fixed** — the protocol is designed to be radio-agnostic and
must operate correctly over any supported transport.

### Supported radio configurations

| Radio Module     | Modulation | Frequency Band | Typical Data Rate | Max Payload |
|------------------|------------|----------------|-------------------|-------------|
| SX1276 / SX1278  | LoRa       | 433 MHz (ITU R1)| 0.3 – 37.5 kbps  | 255 bytes   |
| SX1276 / SX1278  | LoRa       | 868 MHz (EU)   | 0.3 – 37.5 kbps  | 255 bytes   |
| SX1276 / SX1278  | LoRa       | 915 MHz (US)   | 0.3 – 37.5 kbps  | 255 bytes   |
| SX1280 / SX1281  | LoRa       | 2.4 GHz (ISM)  | Up to 253 kbps   | 255 bytes   |
| SX1262 / SX1268  | LoRa / FSK | 150 – 960 MHz  | 0.3 – 300 kbps   | 255 bytes   |
| CC1101 / CC1125  | FSK / GFSK | 433 / 868 MHz  | 1.2 – 500 kbps   | 255 bytes   |
| nRF24L01+        | GFSK       | 2.4 GHz        | 250 kbps – 2 Mbps| 32 bytes    |
| Custom / Future  | Any        | Any licensed   | Any               | ≥ 32 bytes  |

The **default** configuration for ARES v2 is LoRa at 433.125 MHz /
SF10 / BW 125 kHz (~2.4 kbps effective). A 868 MHz profile is also
supported when using a compatible radio module and equivalent regional
settings, but the APUS standard
applies identically regardless of the physical layer. All radio-
specific parameters (frequency, spreading factor, bandwidth, TX
power) are configured in `config.h` and are **not** part of this
protocol standard.

### Design assumptions

Bit flips, partial frames, and complete packet loss are expected in
normal operation on **any** of the above transports. The protocol
must guarantee that no corrupted data ever reaches application logic,
that safety-critical commands are never starved by lower-priority
traffic, and that the encoding overhead is minimal and deterministic.

> **Note:** When using radios with a smaller MTU (e.g. nRF24L01+
> with 32-byte payloads), the ARES frame must be segmented at the
> transport layer. See APUS-15 for segmentation rules.

The real ESA PUS (ECSS-E-ST-70-41C) organises spacecraft
communications into numbered *services*. ARES adopts the relevant
concepts and maps them to its own subsystems:

| PUS Service | ESA Name                     | ARES Mapping                          | Status       |
|-------------|------------------------------|---------------------------------------|--------------|
| ST[1]       | Request Verification         | ACK / NACK frames                     | Implemented  |
| ST[3]       | Housekeeping                 | Periodic telemetry (TELEMETRY frames) | Implemented  |
| ST[4]       | Parameter Statistics         | Sensor min/max/mean aggregation       | Planned      |
| ST[5]       | Event Reporting              | Phase transitions (EVENT frames)      | Implemented  |
| ST[6]       | Memory Management            | Flash config read/write               | Planned      |
| ST[8]       | Function Management          | Remote commands (COMMAND frames)      | Implemented  |
| ST[9]       | Time Management              | On-board clock sync                   | Planned      |
| ST[11]      | Time-Based Scheduling        | FCS `TIME_AFTER_PHASE` triggers       | Implemented  |
| ST[12]      | On-Board Monitoring          | Sensor limit checking                 | Planned      |
| ST[13]      | Large Packet Transfer        | Segmented log download                | Planned      |
| ST[17]      | Connection Test              | HEARTBEAT frames                      | Implemented  |
| ST[19]      | Event-Action                 | FCS engine (flight_control module)    | Implemented  |
| ST[20]      | Parameter Management         | On-board parameter get/set            | Planned      |

Services not listed (ST[2], ST[14], ST[15], ST[18], ST[21], ST[22], ST[23])
are defined in ECSS-E-ST-70-41C but are **not applicable** to ARES due to
hardware constraints or scope (e.g. no on-board file system, no multi-hop
routing, no on-board procedure interpreter).

---

## APUS-1 — Packet Integrity (CRC-32)

Every packet **must** be validated before processing.
ARES uses CRC-32 (Ethernet polynomial `0xEDB88320`) over the
entire header and payload region, providing 32-bit error detection.

### Wire format

```
SYNC(4) | VER | FLAGS | NODE | TYPE | SEQ | LEN | PAYLOAD(0..200) | CRC-32(4)
```

CRC scope: bytes `[4 .. 9 + LEN]` (VER through end of PAYLOAD).

### Rules

| ID       | Rule                                                              |
|----------|-------------------------------------------------------------------|
| APUS-1.1 | Every received frame **must** be CRC-validated before dispatch.   |
| APUS-1.2 | CRC mismatch → **discard** immediately. No partial processing.    |
| APUS-1.3 | Optionally request retransmission (if link is bidirectional).     |
| APUS-1.4 | CRC computation must be O(n) bytewise — no lookup tables >256 B. |
| APUS-1.5 | The sender **must** compute and append CRC before transmission.   |

### Implementation reference

```cpp
// radio_protocol.h — already implemented
uint32_t crc32(const uint8_t* data, uint16_t len);
```

### Rationale

XOR-8 or XOR-16 checksums detect single-bit errors but miss burst
errors common in RF environments. CRC-32 detects all single-bit,
double-bit, and burst errors up to 32 bits — appropriate for the
error profile of low-power radio links (LoRa, FSK, GFSK). The
bytewise implementation uses ~80 bytes of flash and no RAM tables.

---

## APUS-2 — Command Priority

Safety-critical commands (abort, parachute deployment) must never be
delayed by routine traffic. ARES defines four priority levels mapped
to the `FLAG_PRIORITY` bit and an internal queue discipline.

### Priority levels

| Level | Value | Name       | Use Cases                                |
|-------|-------|------------|------------------------------------------|
| 0     | `0`   | CRITICAL   | Abort, pyro fire, emergency state change |
| 1     | `1`   | HIGH       | Flight control commands, FCS overrides   |
| 2     | `2`   | NORMAL     | Sensor requests, config changes          |
| 3     | `3`   | LOW        | Logging, diagnostics, heartbeat          |

### Rules

| ID       | Rule                                                                |
|----------|---------------------------------------------------------------------|
| APUS-2.1 | The execution queue is **always** ordered by priority, then by time. |
| APUS-2.2 | A CRITICAL command **may** preempt any running non-critical action.  |
| APUS-2.3 | CRITICAL commands must be **retransmitted 3×** by the sender.       |
| APUS-2.4 | Each priority level must have a bounded queue depth (PO10-2).       |
| APUS-2.5 | Priority inversion is forbidden — use RTOS mutex priority inheritance. |

### Flag mapping

The `FLAG_PRIORITY` bit (`0x04`) in the frame flags byte marks
HIGH and CRITICAL frames. The exact priority level is encoded in
the first byte of the payload when `TYPE == COMMAND`:

```
PAYLOAD[0] = priority level (0–3)
PAYLOAD[1..] = command-specific data
```

### Rationale

PUS ST[8] (Function Management) defines telecommand verification
with acceptance and execution stages. In ARES, the queue discipline
ensures that an abort command arriving during a telemetry burst is
immediately promoted and executed, with the flight control script
evaluating it within the current control cycle.

---

## APUS-3 — Data Encoding Efficiency

Bandwidth is scarce on low-power radio links (e.g. 2.4 kbps with
LoRa SF10, ~1.85 s per max frame). Even on faster transports, data
must be packed as tightly as possible without introducing
computational overhead or dynamic memory allocation.

### Encoding strategies

#### APUS-3.1 — Numeric command identifiers

String-based identifiers are **forbidden** in wire formats.
All services, subtypes, and states use `uint8_t` enumerations.

```cpp
// ❌ Forbidden
"DEPLOY_PARACHUTE"       // 16 bytes

// ✅ Required
MsgType::COMMAND (0x03)  // 1 byte
```

#### APUS-3.2 — Bit-field packing

Use C++ bit-fields for flags and compact state representation.
Total struct size must be calculated at compile time with
`static_assert`.

```cpp
struct CompactStatus
{
    uint8_t flightPhase : 4;   // 0–15 (8 phases defined)
    uint8_t armed       : 1;
    uint8_t fcsActive   : 1;
    uint8_t error       : 1;
    uint8_t reserved    : 1;
};
static_assert(sizeof(CompactStatus) == 1, "must be 1 byte");
```

#### APUS-3.3 — Delta encoding for telemetry

For high-frequency sensor data (altitude, pressure), transmit the
difference from the previous sample instead of the absolute value.
This reduces the dynamic range and allows smaller integer types.

```cpp
int16_t altitudeDelta = currentAltCm - previousAltCm;
```

Constraints:
- The reference frame (absolute value) must be sent periodically
  (at least every 10th frame) for re-synchronisation.
- Delta overflow must saturate, never wrap silently (CERT-4).

#### APUS-3.4 — Encoding complexity

| ID       | Rule                                                     |
|----------|----------------------------------------------------------|
| APUS-3.4 | All encoding/decoding must be O(1) per field.            |
| APUS-3.5 | No dynamic memory allocation in encode/decode (PO10-3).  |
| APUS-3.6 | `static_assert` on packed struct sizes is mandatory.     |

---

## APUS-4 — RF Link Discipline

Rules for reliable communication over the radio link, regardless of
the underlying physical layer (LoRa, FSK, GFSK, or other).

### Communication model

```
Rocket ──TX──► Ground    (telemetry, events)
Ground ──TX──► Rocket    (commands, ACKs)
```

The primary link is simplex (rocket → ground telemetry).
Command uplink is optional and depends on hardware configuration.

### Frame encapsulation

| ID       | Rule                                                              |
|----------|-------------------------------------------------------------------|
| APUS-4.1 | Every packet must be encapsulated in the ARES v2 wire format.     |
| APUS-4.2 | Raw struct transmission is **forbidden** — always use `encode()`. |
| APUS-4.3 | The 4-byte sync marker (`0xAE 0x55 0xC3 0x1A`) is mandatory.     |

### Acknowledgement protocol

| ID       | Rule                                                              |
|----------|-------------------------------------------------------------------|
| APUS-4.4 | If `FLAG_ACK_REQ` is set, the receiver must respond with an ACK.  |
| APUS-4.5 | ACK timeout: 1000 ms. After timeout, retransmit up to 3×.        |
| APUS-4.6 | Retransmissions must set `FLAG_RETRANSMIT` and preserve the SEQ.  |
| APUS-4.7 | Duplicate SEQ detection is mandatory on the receiver side.        |

### Implementation status

| Rule      | Status          | Module                               |
|-----------|-----------------|--------------------------------------|
| APUS-4.4  | ✅ Implemented   | `src/comms/radio_dispatcher.cpp` — `poll()` called from `loop()` every SENSOR_RATE_MS interval |
| APUS-4.5  | ⏳ Planned       | Retransmit logic not yet implemented (ground-side responsibility for now) |
| APUS-4.6  | ⏳ Planned       | `FLAG_RETRANSMIT` checked in `isDuplicate()` path |
| APUS-4.7  | ✅ Implemented   | `RadioDispatcher::handleCommand()` — `seenFirst_` guard + `isDuplicate(seq, lastRxSeq_)` |

### Transmission cadence

| Traffic Type       | Rate / Policy                                     |
|--------------------|---------------------------------------------------|
| Telemetry          | Configurable, default 0.5 Hz (2000 ms interval)  |
| Events             | Immediate on state transition                     |
| CRITICAL commands  | Repeat 3× with 100 ms spacing                    |
| Heartbeat          | Every 5 s when idle                               |

### Error mitigation

| Problem         | Mitigation                          | APUS Rule  |
|-----------------|-------------------------------------|------------|
| RF noise        | CRC-32 + sync marker                | APUS-1, 4.3 |
| Packet loss     | Retransmission + redundancy         | APUS-4.5   |
| Latency         | Priority queue                      | APUS-2     |
| Bandwidth       | Bit packing + delta encoding        | APUS-3     |
| Duplicate frames | SEQ-based deduplication            | APUS-4.7   |

---

## APUS-5 — Service Identification (PUS Mapping)

Each frame carries a `TYPE` field that maps to a PUS service concept.
Within each type, the payload structure is fixed and documented.

### Type registry

| TYPE | Enum        | PUS Service | Direction       | Description              |
|------|-------------|-------------|-----------------|--------------------------|
| 0x01 | TELEMETRY   | ST[3]       | Rocket → Ground | Periodic sensor data     |
| 0x02 | EVENT       | ST[5]       | Rocket → Ground | State transition / alert |
| 0x03 | COMMAND     | ST[8]       | Ground → Rocket | Remote command           |
| 0x04 | ACK         | ST[1]       | Either          | Request verification     |
| 0x05 | HEARTBEAT   | ST[17]      | Either          | Connection test          |

### Rules

| ID       | Rule                                                           |
|----------|----------------------------------------------------------------|
| APUS-5.1 | Unknown TYPE values must be logged and discarded.              |
| APUS-5.2 | Each TYPE must have a documented, fixed payload schema.        |
| APUS-5.3 | Payload length must be validated against the expected schema.  |

---

## APUS-6 — Telemetry Frame Structure (ST[3])

Defines the payload layout for `TYPE == TELEMETRY (0x01)`.

### Payload format

| Offset | Size | Type     | Field            | Unit     |
|--------|------|----------|------------------|----------|
| 0      | 4    | uint32_t | timestampMs      | ms       |
| 4      | 1    | uint8_t  | flightPhase      | enum     |
| 5      | 1    | uint8_t  | statusBits       | bitfield |
| 6      | 4    | float    | altitudeAglM     | m        |
| 10     | 4    | float    | verticalVelMs    | m/s      |
| 14     | 4    | float    | accelMag         | m/s²     |
| 18     | 4    | float    | pressurePa       | Pa       |
| 22     | 4    | float    | temperatureC     | °C       |
| 26     | 4    | int32_t  | latitudeE7       | deg×1e7  |
| 30     | 4    | int32_t  | longitudeE7      | deg×1e7  |
| 34     | 2    | uint16_t | gpsAltDm         | dm       |
| 36     | 1    | uint8_t  | gpsSats          | count    |
| 37     | 1    | uint8_t  | batteryPct       | %        |

Total: **38 bytes** (fits in a single frame with margin).

### statusBits layout

| Bit | Name       | Description             |
|-----|------------|-------------------------|
| 0   | armed      | Flight FSM armed        |
| 1   | fcsActive  | FCS engine active       |
| 2   | gpsValid   | GPS fix acquired        |
| 3   | pyroAFired | Drogue pyro has fired   |
| 4   | pyroBFired | Main pyro has fired     |
| 5–7 | reserved   | Must be zero            |

### Implementation status (APUS-6)

| Bit field    | Status | Source in firmware                                                                  |
|--------------|--------|-------------------------------------------------------------------------------------|
| `armed`      | ✅     | `status_ == EngineStatus::RUNNING` — set by `MissionScriptEngine::arm()`            |
| `fcsActive`  | ✅     | `executionEnabled_` — toggled by `SET_MODE` / `SET_FCS_ACTIVE` commands             |
| `gpsValid`   | ✅     | Iterates `gpsDrivers_[]` and calls `GpsInterface::hasFix()` on each registered GPS  |
| `pyroAFired` | ✅     | `pyroAFired_` — set via `MissionScriptEngine::notifyPyroFired(0)` after FIRE_PYRO_A |
| `pyroBFired` | ✅     | `pyroBFired_` — set via `MissionScriptEngine::notifyPyroFired(1)` after FIRE_PYRO_B |

Populated by `MissionScriptEngine::buildStatusBitsLocked()` (private, locked helper) and
exposed via `MissionScriptEngine::getStatusBits()` (public, thread-safe).  All three TX
paths that produce a `TelemetryPayload` call this helper:

- `sendHkReportLocked()` — periodic HK on `hk.every` cadence
- `sendHkReportSlotLocked()` — per-slot HK (AMS-4.3.1 multi-slot)
- `RadioDispatcher::executeCommand()` → `REQUEST_STATUS` — immediate status response

> **Note on pyro tracking**: `pyroAFired_` / `pyroBFired_` are set only after a
> confirmed FIRE_PYRO_A/B execution.  When the HAL pyro driver is connected, the
> caller must invoke `notifyPyroFired(channel)` after the GPIO pulse completes.
> Until then, both bits remain `0` (conservative — safer than false positives).

---

## APUS-7 — Command Frame Structure (ST[8])

Defines the payload layout for `TYPE == COMMAND (0x03)`.

### Payload format

| Offset | Size | Type    | Field     | Description               |
|--------|------|---------|-----------|---------------------------|
| 0      | 1    | uint8_t | priority  | 0=CRITICAL … 3=LOW        |
| 1      | 1    | uint8_t | commandId | Command enumeration        |
| 2      | n    | varies  | params    | Command-specific arguments |

### Command identifiers

| ID   | Name               | Priority | Params              |
|------|--------------------|----------|----------------------|
| 0x01 | ARM_FLIGHT         | HIGH     | —                    |
| 0x02 | ABORT              | CRITICAL | —                    |
| 0x03 | FIRE_PYRO_A        | CRITICAL | duration_ms (uint16) |
| 0x04 | FIRE_PYRO_B        | CRITICAL | duration_ms (uint16) |
| 0x05 | SET_MODE           | HIGH     | mode (uint8)         |
| 0x06 | SET_FCS_ACTIVE     | HIGH     | active (uint8)       |
| 0x10 | REQUEST_TELEMETRY  | NORMAL   | —                    |
| 0x11 | SET_TELEM_INTERVAL | NORMAL   | interval_ms (uint16) |
| 0x20 | REQUEST_STATUS     | LOW      | —                    |
| 0x21 | REQUEST_CONFIG     | LOW      | —                    |
| 0x22 | SET_CONFIG_PARAM   | HIGH     | key (uint8), value (varies) |
| 0x23 | VERIFY_CONFIG      | LOW      | —                    |
| 0x24 | FACTORY_RESET      | CRITICAL | —                    |

### Rules

| ID       | Rule                                                          |
|----------|---------------------------------------------------------------|
| APUS-7.1 | Unknown commandId values must be rejected with NACK.          |
| APUS-7.2 | Pyro commands require `armed == true` — reject otherwise.     |
| APUS-7.3 | CRITICAL commands must carry `FLAG_PRIORITY` in the flags.    |

---

## APUS-8 — Event Frame Structure (ST[5])

Defines the payload layout for `TYPE == EVENT (0x02)`.

### Payload format

| Offset | Size | Type     | Field       | Description              |
|--------|------|----------|-------------|--------------------------|
| 0      | 4    | uint32_t | timestampMs | Event time (millis)      |
| 4      | 1    | uint8_t  | severity    | 0=INFO, 1=WARN, 2=ERROR |
| 5      | 1    | uint8_t  | eventId     | Event enumeration        |
| 6      | n    | varies   | context     | Event-specific data      |

### Event identifiers

| ID   | Name             | Severity | Description                     |
|------|------------------|----------|---------------------------------|
| 0x01 | MODE_CHANGE      | INFO     | Operating mode transition       |
| 0x02 | PHASE_CHANGE     | INFO     | Flight phase transition         |
| 0x03 | FCS_RULE_FIRED   | INFO     | FCS rule triggered              |
| 0x04 | PYRO_FIRED       | WARN     | Pyro channel activated          |
| 0x05 | ABORT_TRIGGERED  | ERROR    | Flight aborted                  |
| 0x06 | SENSOR_FAILURE   | ERROR    | Sensor read error               |
| 0x07 | CRC_FAILURE      | WARN     | Received frame CRC mismatch     |
| 0x08 | FPL_VIOLATION    | WARN     | Flight plan envelope exceeded   |
| 0x09 | LINK_LOST        | WARN     | Heartbeat timeout (link down)   |
| 0x0A | LINK_RESTORED    | INFO     | Link recovered after loss       |

---

## APUS-9 — Request Verification Protocol (ST[1] Extended)

PUS-C ST[1] defines a multi-stage verification lifecycle for every
telecommand. ARES implements a simplified version adapted to the
single-hop radio link.

### PUS-C verification stages

The full ECSS-E-ST-70-41C request lifecycle contains five stages,
each generating a success or failure report:

```
TC received
  │
  ├─ (1) Acceptance ─────── TM[1,1] success / TM[1,2] failure
  │
  ├─ (2) Start of execution TM[1,3] success / TM[1,4] failure
  │
  ├─ (3) Progress ────────── TM[1,5] success / TM[1,6] failure  (×N)
  │
  ├─ (4) Completion ──────── TM[1,7] success / TM[1,8] failure
  │
  └─ (5) Routing failure ── TM[1,10] (packet could not be routed)
```

Each stage is controlled by **ack flags** in the TC secondary header.
The sender sets which stages it wants reports for. In OPUS2
(`ccsds-space-packet.asn`), the TC secondary header contains:

```asn1
ROOT-TC-SECONDARY-HEADER ::= SEQUENCE {
    ack-successful-completion   BOOLEAN,
    ack-successful-progress     BOOLEAN,
    ack-successful-start        BOOLEAN,
    ack-successful-acceptance   BOOLEAN,
    source-id                   ROOT-APPLICATION-PROCESS-USER-ID
}
```

### ARES simplification

ARES collapses verification into two stages:

| Stage       | ARES Frame | PUS Mapping   | When                               |
|-------------|------------|---------------|------------------------------------|
| Acceptance  | ACK        | TM[1,1]       | Frame CRC valid, type known        |
| Rejection   | NACK       | TM[1,2]       | CRC fail, unknown type, precondition fail |
| Completion  | ACK        | TM[1,7]       | Command executed successfully      |
| Exec Fail   | NACK       | TM[1,8]       | Command execution failed           |

Progress reports (TM[1,5]/[1,6]) are omitted — commands in ARES
execute atomically within a single RTOS cycle.

### Failure notice structure

Per PUS-C, each failure report carries a **failure notice** containing
a coded reason and optional diagnostic data. ARES encodes this in
the ACK/NACK payload:

| Offset | Size | Field          | Description                        |
|--------|------|----------------|------------------------------------|
| 0      | 1    | originalSeq    | SEQ of the failed command          |
| 1      | 1    | originalNode   | NODE (APID) of the failed command  |
| 2      | 1    | failureCode    | Enumerated reason                  |
| 3      | 1    | failureData    | Command-specific diagnostic byte   |

### Failure codes

| Code | Name                 | Description                         |
|------|----------------------|-------------------------------------|
| 0x01 | CRC_INVALID          | CRC mismatch on received TC         |
| 0x02 | UNKNOWN_TYPE         | Unrecognised message TYPE           |
| 0x03 | UNKNOWN_COMMAND      | Valid TYPE but unknown commandId    |
| 0x04 | PRECONDITION_FAIL    | e.g. pyro command while disarmed   |
| 0x05 | EXECUTION_ERROR      | Runtime error during execution     |
| 0x06 | QUEUE_FULL           | Priority queue overflow            |
| 0x07 | INVALID_PARAM        | Parameter out of valid range       |

### Rules

| ID        | Rule                                                              |
|-----------|-------------------------------------------------------------------|
| APUS-9.1  | Every accepted command **must** generate an acceptance ACK.       |
| APUS-9.2  | Every rejected command **must** generate a NACK with failure code.|
| APUS-9.3  | Completion ACK/NACK is sent only if `FLAG_ACK_REQ` was set.      |
| APUS-9.4  | Failure codes must be defined at compile time — no dynamic strings.|
| APUS-9.5  | The `originalSeq` in a NACK must match the SEQ of the failing TC.|

### Implementation status

| Rule      | Status          | Module                               |
|-----------|-----------------|--------------------------------------|
| APUS-9.1  | ✅ Implemented   | `RadioDispatcher::handleCommand()` — acceptance ACK (`FailureCode::NONE`) sent before execution |
| APUS-9.2  | ✅ Implemented   | `RadioDispatcher::sendAckNack()` — NACK sent with `FailureCode` on all error paths |
| APUS-9.3  | ✅ Implemented   | Completion ACK/NACK gated by `(frame.flags & FLAG_ACK_REQ) != 0` |
| APUS-9.4  | ✅ Implemented   | `FailureCode` enum defined in `ares_radio_protocol.h`; no heap strings |
| APUS-9.5  | ✅ Implemented   | `AckPayload.originalSeq = frame.seq` in `sendAckNack()` |

---

## APUS-10 — CCSDS Space Packet Mapping

PUS-C packets are carried inside CCSDS Space Packets (CCSDS 133.0-B).
ARES uses a simplified mapping that preserves the essential fields
while fitting within the radio transport MTU (typically 255 bytes
for LoRa, as low as 32 bytes for nRF24-class radios).

### CCSDS primary header (reference)

The standard CCSDS primary header is 6 bytes:

```
┌───────────────────────────────────────────────────────────┐
│ Bits 0-2   │ Packet Version Number (always 000 = CCSDS)  │
│ Bit  3     │ Packet Type (0=TM, 1=TC)                    │
│ Bit  4     │ Secondary Header Flag                       │
│ Bits 5-15  │ Application Process Identifier (APID) 11b   │
│ Bits 16-17 │ Sequence Flags (11 = standalone)             │
│ Bits 18-31 │ Packet Sequence Count (14 bits)              │
│ Bits 32-47 │ Packet Data Length (16 bits, = N-1)          │
└───────────────────────────────────────────────────────────┘
```

In OPUS2 ASN.1 (`ccsds-space-packet.asn`):

```asn1
ROOT-PACKET-ID ::= SEQUENCE {
    packet-type               ROOT-PACKET-TYPE,        -- TM(0) / TC(1)
    secondary-header-flag     ROOT-SECONDARY-HEADER-FLAG,
    application-process-id    ROOT-APPLICATION-PROCESS-ID  -- 0..2047
}

ROOT-PACKET-SEQUENCE-CONTROL ::= SEQUENCE {
    sequence-flags            ROOT-SEQUENCE-FLAGS,     -- standalone(3)
    packet-sequence-count     ROOT-PACKET-SEQUENCE-COUNT  -- 0..16383
}
```

### ARES wire format vs CCSDS mapping

| CCSDS Field               | ARES Field    | Bits | Simplification                   |
|---------------------------|---------------|------|----------------------------------|
| Packet Version Number     | (implicit)    | —    | Always 0, not transmitted        |
| Packet Type               | TYPE          | 8    | Extended to full byte enum       |
| Secondary Header Flag     | FLAGS         | 8    | Merged with priority/ack flags   |
| APID                      | NODE          | 8    | Reduced to 8-bit node ID         |
| Sequence Count            | SEQ           | 8    | Reduced to 8-bit (wraps at 256)  |
| Packet Data Length         | LEN           | 8    | Max payload 200 bytes            |
| Packet Error Control      | CRC-32        | 32   | Upgraded from CRC-16 to CRC-32   |

### APID assignment

PUS-C requires each application process to have a unique APID (0–2047).
ARES assigns APIDs as follows:

| APID | Node       | Description                    |
|------|------------|--------------------------------|
| 0x00 | Broadcast  | All nodes (heartbeat, events)  |
| 0x01 | Rocket     | Flight computer primary        |
| 0x02 | Ground     | Ground station                 |
| 0x03 | Payload    | Payload bay controller         |
| 0xFF | Reserved   | Diagnostic / test mode         |

### Request ID

PUS-C identifies each telecommand uniquely by its **Request ID**,
which is the combination of (Packet Version Number, APID, Sequence
Count). This is used to correlate verification reports (ST[1]) with
the original command. In ARES, the Request ID is simplified to
`(NODE, SEQ)`.

### Rules

| ID        | Rule                                                           |
|-----------|----------------------------------------------------------------|
| APUS-10.1 | NODE field must be validated against the APID table.           |
| APUS-10.2 | Unknown APID/NODE values must be rejected (routing failure).   |
| APUS-10.3 | Sequence count wraps at 256 — receiver must handle wrap-around.|
| APUS-10.4 | APID 0x00 frames are processed by all nodes (broadcast).      |

---

## APUS-11 — Parameter Type Codes (PTC/PFC)

PUS-C defines a **Parameter Type Code (PTC)** and **Parameter Format
Code (PFC)** system for self-describing data fields. ARES adopts a
subset for on-board parameter management and housekeeping.

### PTC definitions (ECSS-E-ST-70-41C §7.3)

| PTC | Name              | Description                              |
|-----|-------------------|------------------------------------------|
| 1   | Boolean           | Single boolean value                     |
| 2   | Enumerated        | Enumerated value (mapped to integer)     |
| 3   | Unsigned Integer  | Unsigned integer of PFC-defined width    |
| 4   | Signed Integer    | Signed integer (two's complement)        |
| 5   | Real              | IEEE 754 floating point                  |
| 6   | Bit String        | Arbitrary bit sequence                   |
| 7   | Octet String      | Arbitrary byte sequence                  |
| 8   | Character String  | ASCII / UTF-8 string                     |
| 9   | Absolute Time     | CUC or CDS time format                  |
| 10  | Relative Time     | Duration                                 |
| 11  | Deduced           | Type determined at runtime               |
| 12  | Packet            | Embedded sub-packet                      |

### ARES PTC subset

ARES uses only these PTC values in telemetry and parameter frames:

| PTC | PFC | ARES C++ Type | Use Case                           |
|-----|-----|---------------|------------------------------------|
| 1   | 0   | `bool`        | Status flags (armed, gpsValid)     |
| 3   | 14  | `uint32_t`    | Timestamps, counters               |
| 3   | 12  | `uint16_t`    | GPS altitude, intervals            |
| 3   | 8   | `uint8_t`     | Enums, percentages                 |
| 4   | 14  | `int32_t`     | Lat/Lon (×1e7), deltas             |
| 4   | 12  | `int16_t`     | Altitude deltas                    |
| 5   | 1   | `float`       | Pressure, temperature, velocity    |

### Rules

| ID        | Rule                                                               |
|-----------|--------------------------------------------------------------------|
| APUS-11.1 | All telemetry parameters must have a documented PTC/PFC pair.      |
| APUS-11.2 | PTC=11 (deduced) is **forbidden** — all types must be static.      |
| APUS-11.3 | PTC/PFC values must match the C++ type in `static_assert` checks.  |
| APUS-11.4 | New PTC/PFC pairs require a document update and code review.       |

---

## APUS-12 — On-Board Monitoring (ST[12] Adapted)

PUS-C ST[12] provides autonomous monitoring of on-board parameters
against defined limits. ARES adapts this for flight safety checks.

### Monitoring model

Each monitored parameter has a **Parameter Monitoring Definition**
containing:

```cpp
struct MonitoringDefinition
{
    uint8_t  parameterId;       // Links to PTC/PFC registry
    float    lowLimit;          // Below → LOW_SEVERITY event
    float    highLimit;         // Above → HIGH_SEVERITY event
    float    expectedLow;      // Expected nominal range lower
    float    expectedHigh;     // Expected nominal range upper
    uint8_t  consecutiveCount; // N violations before trigger
    bool     enabled;
};
```

### Check types (per ECSS-E-ST-70-41C §6.12)

| Check Type          | PUS Name              | ARES Behaviour                     |
|---------------------|-----------------------|------------------------------------|
| Limit Check         | Expected Value Check  | Value outside [low, high]          |
| Delta Check         | Delta Check           | |current − previous| > threshold   |
| Validity Check      | Check Validity        | Sensor reports valid flag          |

### Event severity mapping

Per PUS-C, monitoring violations generate events with severity:

| PUS Severity     | ARES eventId        | Action                            |
|------------------|---------------------|-----------------------------------|
| Informative      | —                   | Log only                          |
| Low Severity     | SENSOR_WARN (0x09)  | Telemetry flag + ground alert     |
| Medium Severity  | FPL_VIOLATION (0x08)| FCS evaluation + event frame      |
| High Severity    | ABORT_TRIGGERED     | Automatic abort if configured     |

### Transition detection

Monitoring states follow a PUS-C state machine:

```
         ┌──────────┐
    ─────► DISABLED  │◄──── disable_monitoring()
         └────┬─────┘
              │ enable_monitoring()
         ┌────▼─────┐
    ─────► ENABLED   │
         └────┬─────┘
              │ value outside limits
         ┌────▼──────────┐
         │ ALARM (N of M) │── consecutiveCount reached → EVENT
         └───────────────┘
```

### Rules

| ID        | Rule                                                              |
|-----------|-------------------------------------------------------------------|
| APUS-12.1 | Monitoring definitions must be declared at compile time.          |
| APUS-12.2 | Limit violations generate events only after `consecutiveCount`.   |
| APUS-12.3 | Delta checks must use saturating subtraction (no wrap).           |
| APUS-12.4 | Monitoring evaluation runs in the same RTOS cycle as telemetry.   |
| APUS-12.5 | Disabled monitors must not consume CPU cycles.                    |

---

## APUS-13 — Time Management (ST[9] Adapted)

PUS-C ST[9] covers on-board time management including distribution,
synchronisation, and rate adjustment. ARES implements a minimal
subset for mission-elapsed-time (MET) and GPS time correlation.

### Time representations

PUS-C defines two standard time formats:

| Format | Name                         | Resolution   | ARES Use              |
|--------|------------------------------|--------------|-----------------------|
| CUC    | CCSDS Unsegmented Time Code  | Configurable | Not used (too complex)|
| CDS    | CCSDS Day Segmented          | 1 ms         | Not used              |
| —      | ARES millisecond counter     | 1 ms         | `millis()` / MET      |

ARES uses a 32-bit unsigned millisecond counter (`uint32_t`) relative
to boot time. GPS-derived UTC is available separately when the fix
is valid.

### Time fields in packets

| Frame Type  | Time Field      | Source                  | Resolution |
|-------------|-----------------|-------------------------|------------|
| TELEMETRY   | `timestampMs`   | `millis()` at sample    | 1 ms       |
| EVENT       | `timestampMs`   | `millis()` at event     | 1 ms       |
| COMMAND     | —               | No timestamp (ground)   | —          |
| ACK         | —               | Implicit (response)     | —          |

### GPS time correlation

When a valid GPS fix is available, the ground station can compute:

```
UTC_event = GPS_UTC_at_last_fix + (event_timestampMs - fix_timestampMs)
```

This correlation is performed ground-side. The rocket does not
maintain an RTC or UTC clock.

### Rules

| ID        | Rule                                                              |
|-----------|-------------------------------------------------------------------|
| APUS-13.1 | `timestampMs` must be captured at the point of measurement, not   |
|           | at the point of transmission.                                     |
| APUS-13.2 | The millisecond counter must use a monotonic source (`millis()`). |
| APUS-13.3 | Counter overflow (49.7 days) is acceptable — flights are <1 hour. |
| APUS-13.4 | Time-based FCS triggers must compare against MET, not wall clock. |
| APUS-13.5 | GPS time is advisory — never use it for flight-critical decisions.|

---

## APUS-14 — APID Routing and Service Dispatch

PUS-C requires a routing layer that dispatches incoming packets to
the correct application process (identified by APID) and then to
the correct service/subservice within that process.

### OPUS2 routing model

In the OPUS2 toolset, the routing architecture has two layers:

1. **APID Router** — routes incoming space packets to the correct
   application process based on the APID field
2. **Capability Router** — within an application process, routes
   requests to the correct service/capability handler based on
   service type and message subtype

### ARES simplified routing

ARES collapses both layers into a single dispatcher:

```
Incoming frame
  │
  ├─ CRC check (APUS-1)
  │
  ├─ NODE check (APUS-10)
  │    ├─ Not for us → discard
  │    └─ Match or broadcast → continue
  │
  ├─ TYPE dispatch (APUS-5)
  │    ├─ TELEMETRY → telemetry_handler()
  │    ├─ COMMAND   → command_dispatcher()
  │    ├─ EVENT     → event_handler()
  │    ├─ ACK       → ack_handler()
  │    ├─ HEARTBEAT → heartbeat_handler()
  │    └─ Unknown   → log + discard (APUS-5.1)
  │
  └─ Priority insertion (APUS-2)
```

### Routing failure handling

Per PUS-C, if a packet cannot be routed, a **failed routing
verification report** (TM[1,10]) is generated. ARES maps this
to a NACK with `failureCode = 0x02` (UNKNOWN_TYPE) or implicit
discard for unknown NODE values.

### Rules

| ID        | Rule                                                              |
|-----------|-------------------------------------------------------------------|
| APUS-14.1 | Routing must complete in O(1) — use jump tables, not linear search.|
| APUS-14.2 | Every routing decision must be logged (debug level).              |
| APUS-14.3 | Routing failures must generate a NACK if the source is known.    |
| APUS-14.4 | The dispatcher must not block — non-matching frames are discarded.|

### Implementation status

| Rule      | Status          | Module                               |
|-----------|-----------------|--------------------------------------|
| APUS-14.1 | ✅ Implemented   | `RadioDispatcher::dispatchFrame()` — O(1) `switch(frame.type)` |
| APUS-14.2 | ✅ Implemented   | Every branch calls `LOG_D(TAG, ...)` before discard or dispatch |
| APUS-14.3 | ✅ Implemented   | Unknown NODE → LOG_D discard; decode failure → LOG_D skip; unknown commandId → `FailureCode::UNKNOWN_COMMAND` NACK |
| APUS-14.4 | ✅ Implemented   | `poll()` drains FIFO without blocking; `processBuffer()` returns early if incomplete frame |

---

## APUS-15 — Large Packet Transfer (ST[13] Adapted)

PUS-C ST[13] handles transfer of data that exceeds the maximum
packet size. ARES adapts this for downloading flight logs and
sensor recordings after landing.

### Segmentation model

Data larger than the 200-byte payload limit is split into segments:

```
┌──────────────────────────────────────────────────┐
│ Large data block (e.g. flight log, N bytes)      │
├───────────┬───────────┬───────────┬──────────────┤
│ Segment 0 │ Segment 1 │ Segment 2 │ ... Segment K│
│ (200 B)   │ (200 B)   │ (200 B)   │ (remainder)  │
└───────────┴───────────┴───────────┴──────────────┘
```

### Segment header

Each segment frame has an additional header prepended to the payload:

| Offset | Size | Type     | Field        | Description              |
|--------|------|----------|--------------|--------------------------|
| 0      | 2    | uint16_t | transferId   | Unique transfer session  |
| 2      | 2    | uint16_t | segmentNum   | Segment index (0-based)  |
| 4      | 2    | uint16_t | totalSegments| Total segments in transfer|
| 6      | n    | uint8_t[]| data         | Segment payload          |

### Transfer protocol

```
Ground ── REQUEST_LOG ──► Rocket
Ground ◄── SEG[0] ─────── Rocket
Ground ── ACK(SEG[0]) ──► Rocket
Ground ◄── SEG[1] ─────── Rocket
Ground ── ACK(SEG[1]) ──► Rocket
...
Ground ◄── SEG[K] ─────── Rocket    (last segment)
Ground ── ACK(SEG[K]) ──► Rocket
Ground: reassemble → verify CRC-32 over complete block
```

### Rules

| ID        | Rule                                                             |
|-----------|------------------------------------------------------------------|
| APUS-15.1 | Each segment must be individually CRC-protected (APUS-1).        |
| APUS-15.2 | The complete reassembled block must have an end-to-end checksum. |
| APUS-15.3 | Segment retransmission uses the same SEQ + `FLAG_RETRANSMIT`.   |
| APUS-15.4 | Transfer sessions must timeout after 30 s of inactivity.        |
| APUS-15.5 | Only one transfer session may be active at a time (no overlap). |
| APUS-15.6 | Large transfers are LOW priority — never during flight.          |

---

## APUS-16 — Parameter Management (ST[20] Adapted)

PUS-C ST[20] provides capabilities to report and modify on-board
parameter values at runtime. ARES adapts this for in-flight
configuration and ground-station diagnostics.

### Parameter model

Each on-board parameter is defined by:

```cpp
struct ParameterDescriptor
{
    uint8_t   id;          // Unique parameter identifier
    uint8_t   ptc;         // Parameter Type Code (APUS-11)
    uint8_t   pfc;         // Parameter Format Code
    uint8_t   size;        // Size in bytes
    void*     address;     // Pointer to the value in memory
    bool      writable;    // Can be modified by telecommand
};
```

### Capabilities (from OPUS2 checklist)

| Capability                    | ARES Support | Description                   |
|-------------------------------|--------------|-------------------------------|
| Report parameter values       | Yes          | REQUEST_STATUS / REQUEST_CONFIG|
| Set parameter values          | Yes          | SET_MODE, SET_TELEM_INTERVAL  |
| Report parameter definitions  | Planned      | Return PTC/PFC/address info   |
| Change parameter definitions  | No           | Too dynamic for embedded use  |

### Command mapping

| APUS-7 commandId   | ST[20] Capability          | Direction       |
|---------------------|----------------------------|-----------------|
| 0x10 REQUEST_TELEMETRY | Report parameter values  | Ground → Rocket |
| 0x11 SET_TELEM_INTERVAL| Set parameter values     | Ground → Rocket |
| 0x20 REQUEST_STATUS    | Report parameter values   | Ground → Rocket |
| 0x21 REQUEST_CONFIG    | Report parameter definitions | Ground → Rocket |

### Rules

| ID        | Rule                                                             |
|-----------|------------------------------------------------------------------|
| APUS-16.1 | Read-only parameters must reject SET commands with NACK.         |
| APUS-16.2 | Parameter values must be validated against PTC/PFC constraints.  |
| APUS-16.3 | Parameter table must be `constexpr` — no dynamic registration.   |
| APUS-16.4 | SET commands during flight require HIGH or CRITICAL priority.    |

---

## APUS-17 — Packet Secondary Headers

PUS-C defines secondary headers for both TC and TM packets. These
provide additional context beyond the CCSDS primary header.

### TC secondary header (ECSS-E-ST-70-41C §7.4.1)

The standard TC secondary header contains:

| Field                    | Bits | ARES Mapping           |
|--------------------------|------|------------------------|
| TC Packet PUS Version    | 4    | Implicit (v2)          |
| Ack flags (4 stages)     | 4    | `FLAG_ACK_REQ` (1 bit) |
| Service Type             | 8    | `TYPE` field           |
| Message Subtype          | 8    | `commandId` field      |
| Source ID                | 16   | `NODE` (8-bit)         |

### TM secondary header (ECSS-E-ST-70-41C §7.4.2)

| Field                    | Bits | ARES Mapping           |
|--------------------------|------|------------------------|
| TM Packet PUS Version    | 4    | Implicit (v2)          |
| Time Reference Status    | 4    | Not used               |
| Service Type             | 8    | `TYPE` field           |
| Message Subtype          | 8    | Event/telemetry subtype|
| Message Type Counter     | 16   | `SEQ` (8-bit)          |
| Destination ID           | 16   | `NODE` (8-bit)         |
| Time                     | var  | `timestampMs` (32-bit) |

### Ack flag mapping

PUS-C allows selective acknowledgement per verification stage:

| Bit | PUS Stage                    | ARES Implementation      |
|-----|------------------------------|--------------------------|
| 0   | Acceptance (TM[1,1/2])       | Always reported          |
| 1   | Start of execution (TM[1,3]) | Not implemented          |
| 2   | Progress (TM[1,5])           | Not implemented          |
| 3   | Completion (TM[1,7/8])       | Only if `FLAG_ACK_REQ`   |

### Rules

| ID        | Rule                                                             |
|-----------|------------------------------------------------------------------|
| APUS-17.1 | Service Type + Message Subtype must uniquely identify a packet.  |
| APUS-17.2 | The PUS version field is not transmitted — always assumed v2.    |
| APUS-17.3 | Source/Destination IDs must match the APID table (APUS-10).      |

---

## APUS-18 — Packet Error Control

PUS-C specifies a Packet Error Control (PEC) field as the last field
of every space packet. ECSS-E-ST-70-41C mandates CRC-16-CCITT for
the PEC. ARES **upgrades** this to CRC-32 for stronger error
detection on noisy low-power radio links.

### Comparison

| Property                | PUS-C PEC (CRC-16)   | ARES PEC (CRC-32)     |
|-------------------------|----------------------|-----------------------|
| Polynomial              | 0x1021 (CCITT)       | 0xEDB88320 (Ethernet) |
| Detection capability    | ≤16-bit bursts       | ≤32-bit bursts        |
| Overhead                | 2 bytes              | 4 bytes               |
| False positive rate     | 1 in 65,536          | 1 in 4,294,967,296    |
| Computation             | Bytewise, O(n)       | Bytewise, O(n)        |

### Rationale

Low-power radio links (LoRa, FSK, etc.) experience burst errors
exceeding 16 bits during motor burn (vibration + EMI), especially
on 433/868 MHz bands. CRC-32 provides adequate protection for the
2-byte overhead cost. Given the max frame size of ~210 bytes, CRC-32
detects all error patterns up to 32 bits and has a hamming distance
≥ 4 for messages up to 32,768 bits.

### Rules

| ID        | Rule                                                             |
|-----------|------------------------------------------------------------------|
| APUS-18.1 | CRC-32 replaces PUS-C CRC-16 for all ARES packets.              |
| APUS-18.2 | CRC scope is identical to PUS-C PEC scope (entire data field).   |
| APUS-18.3 | If interoperating with PUS-C ground systems, a CRC-16 adapter   |
|           | must be provided at the ground station.                          |

---

## APUS-19 — Housekeeping Report Structure (ST[3] Extended)

PUS-C ST[3] defines housekeeping as periodic collection and
downlink of on-board parameter sets. ARES TELEMETRY frames
implement a single fixed-structure housekeeping report.

### PUS-C housekeeping model

In full PUS-C, housekeeping is highly configurable:

- Multiple **HK structure definitions** (parameter groups)
- Each structure has a unique **Structure ID**
- Structures can be created, deleted, and modified by telecommand
- Collection intervals are independently configurable
- Parameters are referenced by ID from the parameter pool (ST[20])

### ARES simplification

ARES uses a **single, fixed** housekeeping structure (the telemetry
frame defined in APUS-6) with a fixed collection interval. This
avoids dynamic memory allocation and simplifies verification.

| PUS-C Capability                     | ARES Support  |
|--------------------------------------|---------------|
| Create HK report structure           | No (fixed)    |
| Delete HK report structure           | No            |
| Enable/disable periodic reporting    | Yes (SET_TELEM_INTERVAL) |
| Modify collection interval           | Yes (SET_TELEM_INTERVAL) |
| One-shot HK report                   | Yes (REQUEST_TELEMETRY)  |
| Append parameters to structure       | No (fixed)    |

### Super-commutation

PUS-C supports **super-commutation** — sampling a parameter multiple
times within a single collection interval. ARES does not implement
this. All parameters are sampled once per collection cycle.

### Rules

| ID        | Rule                                                             |
|-----------|------------------------------------------------------------------|
| APUS-19.1 | The HK structure is defined at compile time and cannot change.   |
| APUS-19.2 | All parameters in the HK structure are sampled atomically.       |
| APUS-19.3 | Minimum collection interval is 100 ms (10 Hz).                   |
| APUS-19.4 | One-shot reports use the same structure as periodic reports.      |

---

## APUS-20 — Event Severity Model (ST[5] Extended)

PUS-C ST[5] defines four severity levels for on-board events.
ARES maps these to its event system with specific actions.

### PUS-C severity levels (ECSS-E-ST-70-41C §6.5)

| PUS Severity    | Value | ARES Severity | Autonomous Action           |
|-----------------|-------|---------------|-----------------------------|
| Informative     | 1     | INFO (0)      | Log only, downlink at normal rate |
| Low Severity    | 2     | WARN (1)      | Immediate downlink, FCS check |
| Medium Severity | 3     | —             | Not used (merged with WARN) |
| High Severity   | 4     | ERROR (2)     | Immediate downlink + possible abort |

### Event-Action linking (ST[19])

PUS-C ST[19] allows linking events to automatic actions. ARES
implements this through the FCS engine:

| Event                | Automatic Action                    | Configurable |
|----------------------|-------------------------------------|--------------|
| PHASE_CHANGE         | Update telemetry cadence            | No           |
| FPL_VIOLATION        | Evaluate abort conditions           | Yes (FCS)    |
| SENSOR_FAILURE       | Switch to redundant sensor          | No           |
| ABORT_TRIGGERED      | Immediate parachute deployment      | No           |

### Event buffering

PUS-C requires that events are buffered if the downlink is
temporarily unavailable. ARES buffers up to **16 events** in a
ring buffer. If the buffer overflows, the oldest INFO event is
dropped first (priority-based eviction).

### Rules

| ID        | Rule                                                             |
|-----------|------------------------------------------------------------------|
| APUS-20.1 | Events must be generated synchronously at the point of detection.|
| APUS-20.2 | Event buffer overflow must never drop ERROR events.              |
| APUS-20.3 | Event-Action links must be defined at compile time.              |
| APUS-20.4 | Each event must carry a monotonic timestamp (APUS-13.1).         |

---

## APUS-21 — Parameter Statistics Reporting (ST[4] Adapted)

PUS-C ST[4] provides on-board statistical evaluation of sampled
parameters over a defined interval (min, max, mean, standard
deviation). ARES adapts this for flight-phase sensor summaries.

### Statistical model

Each statistic definition references a parameter by its ID
(APUS-11 PTC/PFC registry) and a sampling window:

```cpp
struct StatisticsDefinition
{
    uint8_t   parameterId;    // From APUS-16 parameter table
    uint16_t  windowMs;       // Evaluation window in ms
    float     minValue;       // Minimum observed in window
    float     maxValue;       // Maximum observed in window
    float     meanValue;      // Running mean (Welford's algorithm)
    uint16_t  sampleCount;    // Number of samples in window
};
```

### Capabilities

| PUS-C Capability                         | ARES Support | Notes                       |
|------------------------------------------|--------------|-----------------------------|
| Enable/disable statistics evaluation     | Yes          | Per parameter, compile-time |
| Report parameter statistics values       | Yes          | On-demand via command       |
| Reset statistics                         | Yes          | Per window or global        |
| Define statistics evaluation interval    | Partial      | Fixed at compile time       |
| One-shot statistics report               | Yes          | REQUEST_STATUS variant      |

### Report format

Statistics reports reuse the TELEMETRY frame (TYPE = 0x01) with a
dedicated subtype byte:

| Offset | Size | Type     | Field        | Description                  |
|--------|------|----------|--------------|------------------------------|
| 0      | 1    | uint8_t  | parameterId  | Parameter being reported     |
| 1      | 4    | float    | minValue     | Minimum in window            |
| 5      | 4    | float    | maxValue     | Maximum in window            |
| 9      | 4    | float    | meanValue    | Mean in window               |
| 13     | 2    | uint16_t | sampleCount  | Samples collected            |
| 15     | 2    | uint16_t | windowMs     | Window duration              |

### Rules

| ID        | Rule                                                              |
|-----------|-------------------------------------------------------------------|
| APUS-21.1 | Statistics must use Welford's online algorithm (no array storage). |
| APUS-21.2 | Standard deviation is omitted — too expensive for ESP32-S3.       |
| APUS-21.3 | Statistics evaluation must not allocate dynamic memory.            |
| APUS-21.4 | Window size must be a compile-time constant.                      |
| APUS-21.5 | Statistics reset must zero min/max/mean atomically (ISR-safe).    |

---

## APUS-22 — Memory Management (ST[6] Adapted)

PUS-C ST[6] provides capabilities to load, dump, and check on-board
memory contents. ARES adapts this for non-volatile configuration
storage on the ESP32-S3 flash (NVS / SPIFFS / LittleFS).

### Capabilities

| PUS-C Capability             | ARES Support | Notes                            |
|------------------------------|--------------|----------------------------------|
| Load memory (raw write)      | No           | Too dangerous for flight safety  |
| Dump memory (raw read)       | Partial      | Config-only, not arbitrary RAM   |
| Check memory (checksum)      | Yes          | CRC-32 of config partition       |
| Load by address              | No           | Forbidden — use named parameters |

### Supported operations

| commandId | Name              | Description                          |
|-----------|-------------------|--------------------------------------|
| 0x21      | REQUEST_CONFIG    | Dump current config (key-value)      |
| 0x22      | SET_CONFIG_PARAM  | Write a named config parameter       |
| 0x23      | VERIFY_CONFIG     | Return CRC-32 of config partition    |
| 0x24      | FACTORY_RESET     | Restore default configuration        |

### Security constraints

| ID        | Rule                                                              |
|-----------|-------------------------------------------------------------------|
| APUS-22.1 | Raw memory address access is **forbidden** (no pointer arithmetic).|
| APUS-22.2 | Config writes require CRITICAL or HIGH priority (APUS-2).        |
| APUS-22.3 | Config writes must be followed by a verification read-back.       |
| APUS-22.4 | Config partition CRC must be verified at boot.                    |
| APUS-22.5 | FACTORY_RESET requires `armed == false`.                          |
| APUS-22.6 | All flash writes use wear-levelling (ESP-IDF NVS or equivalent). |

---

## APUS-23 — Connection Test (ST[17] Adapted)

PUS-C ST[17] provides a simple connection test capability used to
verify that the communication link is active. ARES implements this
through HEARTBEAT frames.

### Heartbeat model

```
Ground ── HEARTBEAT ──► Rocket    (optional, ground-initiated)
Rocket ── HEARTBEAT ──► Ground    (periodic, rocket-initiated)
```

Both directions use `TYPE == HEARTBEAT (0x05)`.

### Payload format

| Offset | Size | Type     | Field        | Description              |
|--------|------|----------|--------------|--------------------------|
| 0      | 4    | uint32_t | timestampMs  | Sender's MET             |
| 4      | 1    | uint8_t  | statusByte   | Compact health summary   |

The `statusByte` is a bitfield:

| Bit | Name       | Description                   |
|-----|------------|-------------------------------|
| 0   | radioOk    | Radio module responding       |
| 1   | sensorsOk  | All sensors nominal           |
| 2   | gpsOk      | GPS fix valid                 |
| 3   | flashOk    | Flash storage healthy         |
| 4   | fcsOk      | FCS engine running            |
| 5–7 | reserved   | Must be zero                  |

### Link loss detection

| Parameter              | Value        | Notes                        |
|------------------------|--------------|------------------------------|
| Heartbeat interval     | 5 s (idle)   | Configurable via `config.h`  |
| Link loss timeout      | 15 s         | 3 missed heartbeats          |
| Action on link loss    | Log + event  | EVENT `LINK_LOST` (0x09)     |
| Action on link restore | Log + event  | EVENT `LINK_RESTORED` (0x0A) |

### Rules

| ID        | Rule                                                              |
|-----------|-------------------------------------------------------------------|
| APUS-23.1 | Heartbeats are LOW priority — never preempt critical traffic.    |
| APUS-23.2 | Heartbeat interval must increase during flight (save bandwidth). |
| APUS-23.3 | Link loss must generate an on-board EVENT, not just a log entry. |
| APUS-23.4 | Heartbeat response (echo) is optional and must not be required.  |

---

## APUS-24 — Time-Based Scheduling (ST[11] Adapted)

PUS-C ST[11] provides capabilities to schedule telecommand execution
at a future time. ARES implements this through the FCS engine's
`TIME_AFTER_PHASE` trigger mechanism.

### Scheduling model

ARES does not support arbitrary TC scheduling (uploading a command
for future execution). Instead, time-based actions are statically
defined in the FCS flight plan:

```cpp
struct FcsTimeTrigger
{
    uint8_t   phase;          // Flight phase to reference
    uint32_t  delayMs;        // Milliseconds after phase entry
    uint8_t   actionId;       // Action to execute
    bool      enabled;        // Can be disabled by command
};
```

### PUS-C comparison

| PUS-C Capability                      | ARES Support | Notes                        |
|---------------------------------------|--------------|------------------------------|
| Insert activity (schedule TC)         | No           | Static FCS rules only        |
| Delete scheduled activity             | No           | Disable via command          |
| Enable/disable schedule               | Partial      | Per-rule enable/disable      |
| Reset schedule                        | No           | Reboot reloads defaults      |
| Time-shift all activities             | No           | Not applicable               |
| Report scheduled activities           | Yes          | REQUEST_CONFIG returns rules |
| Sub-schedule groups                   | No           | Single schedule              |

### Rules

| ID        | Rule                                                              |
|-----------|-------------------------------------------------------------------|
| APUS-24.1 | Time triggers must reference MET, not wall clock (APUS-13.4).    |
| APUS-24.2 | All triggers must be defined at compile time.                     |
| APUS-24.3 | Trigger evaluation must complete within the RTOS control cycle.   |
| APUS-24.4 | Disabled triggers must not consume CPU cycles.                    |
| APUS-24.5 | A trigger that fires must generate an FCS_RULE_FIRED event.      |

---

## APUS-25 — Excluded PUS Services (Tailoring Justification)

ECSS-E-ST-70-41C defines 23 services. ARES excludes the following
with explicit rationale per ECSS-E-ST-70-41C §5.3 (tailoring):

| PUS Service | ESA Name                        | Exclusion Rationale                                       |
|-------------|---------------------------------|-----------------------------------------------------------|
| ST[2]       | Device Access                   | No raw bus access to external devices; all sensors are managed through dedicated drivers with type-safe APIs. Raw I²C/SPI commanding is a safety risk. |
| ST[10]      | Diagnostic Data Reporting       | Covered by ST[3] (housekeeping) with optional high-rate mode. Adding a separate diagnostic service would duplicate telemetry infrastructure. |
| ST[14]      | Real-Time Forwarding Control    | Single-hop architecture — no intermediate nodes to forward through. APID routing (APUS-14) covers all dispatch. |
| ST[15]      | On-Board Storage & Retrieval    | No on-board mass storage filesystem. Flight logs are stored in raw flash partitions and downloaded via ST[13] (large packet transfer). |
| ST[16]      | (Reserved by ESA)               | Not defined in current ECSS-E-ST-70-41C revision. |
| ST[18]      | On-Board Control Procedures     | No on-board procedure interpreter; all flight logic is compiled C++ in the FCS engine. Uploading runtime procedures is a safety risk for amateur rocketry. |
| ST[21]      | Request Sequencing              | Subsumed by FCS engine (ST[11]/ST[19]). No need for a separate request sequencing service. |
| ST[22]      | Position-Based Scheduling       | No position-based triggers; all triggers are time-based (APUS-24) or event-based (APUS-20). |
| ST[23]      | File Management                 | No on-board filesystem. Configuration uses key-value NVS storage (APUS-22). |

### Tailoring compliance

Per ECSS-E-ST-70-41C §5.3, service tailoring is permitted when:

1. The service is not required by the mission profile
2. The exclusion is documented with rationale
3. The functionality is either not needed or covered by another service

All exclusions above satisfy these criteria. If a future ARES
revision adds capabilities (e.g. SD card storage), the relevant
services should be reconsidered.

---

## Cross-References

| APUS Rule  | Related Standards                        |
|------------|------------------------------------------|
| APUS-1     | CERT-1, CERT-2, DO-6                     |
| APUS-2     | RTOS-4, RTOS-5, PO10-2                   |
| APUS-3     | PO10-3, MISRA-1, MISRA-7                 |
| APUS-4     | CERT-1, DO-7, RTOS-1                     |
| APUS-5     | CERT-5, PO10-5                           |
| APUS-6     | MISRA-1, MISRA-7                         |
| APUS-7     | CERT-1, DO-6, RTOS-4                     |
| APUS-8     | DO-8, PO10-5                             |
| APUS-9     | CERT-1, CERT-5, DO-6                     |
| APUS-10    | CERT-5, PO10-5, APUS-5                   |
| APUS-11    | MISRA-1, MISRA-7, APUS-6                 |
| APUS-12    | RTOS-4, CERT-4, DO-8                     |
| APUS-13    | RTOS-1, CERT-2                           |
| APUS-14    | APUS-1, APUS-5, RTOS-4                   |
| APUS-15    | APUS-1, CERT-1, PO10-3                   |
| APUS-16    | APUS-11, CERT-5, PO10-3                  |
| APUS-17    | APUS-5, APUS-10                          |
| APUS-18    | APUS-1, CERT-2, DO-6                     |
| APUS-19    | APUS-6, APUS-11, MISRA-7                 |
| APUS-20    | APUS-8, RTOS-4, DO-8                     |
| APUS-21    | APUS-11, APUS-12, MISRA-7               |
| APUS-22    | CERT-1, CERT-5, PO10-3                   |
| APUS-23    | APUS-4, RTOS-1, CERT-2                   |
| APUS-24    | APUS-13, RTOS-4, DO-8                    |
| APUS-25    | (tailoring justification — no impl.)     |

---

## References

- ECSS-E-ST-70-41C — *Telemetry and telecommand packet utilisation*
  (European Cooperation for Space Standardization, April 2016)
- CCSDS 133.0-B-2 — *Space Packet Protocol* (Blue Book, June 2020)
- OPUS2 Toolset — <https://gitlab.esa.int/PUS-C/opus2>
  (ESA Contract 4000133440/20/NL/CRS, N7 Space)
- OPUS2 User Manual — <https://gitlab.esa.int/taste/taste-setup/-/wikis/Opus2_User_Manual>
- ARES Radio Protocol v2 — `src/comms/radio_protocol.h`
- ARES FCS Engine — `src/controllers/flight_control.h` (PUS ST[11], ST[19])

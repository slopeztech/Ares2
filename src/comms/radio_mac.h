/**
 * @file  radio_mac.h
 * @brief HMAC-SHA256 authentication for APUS radio COMMAND frames (APUS-17).
 *
 * Provides truncated HMAC-SHA256 tag computation and verification for
 * transport-command authentication.
 *
 * Wire format addition:
 *   When FLAG_MAC is set on a COMMAND frame the last @c HMAC_LEN bytes of the
 *   payload are a truncated HMAC-SHA256 tag.  The tag covers the header fields
 *   plus the real command payload:
 *
 *     MAC_input = [VER(1), FLAGS(1), NODE(1), TYPE(1), SEQ(1), cmd_len(1),
 *                  payload[0 .. cmd_len-1]]
 *
 *   where @c cmd_len = @c frame.len - @c HMAC_LEN.
 *
 * Key management:
 *   A 16-byte (128-bit) pre-shared key is stored in LittleFS via DeviceConfig.
 *   When no key is configured the dispatcher accepts frames without a MAC tag
 *   (backwards-compatible open mode).  Once a key is provisioned all incoming
 *   COMMAND frames MUST carry FLAG_MAC and pass verification.
 *
 * Standards: APUS-17 (radio command authentication).
 * Coding: no heap (PO10-3); pure portable C++17; no Arduino dependency.
 */
#pragma once

#include "comms/ares_radio_protocol.h"

#include <cstdint>

namespace ares
{
namespace proto
{

/// Length of the truncated HMAC-SHA256 tag appended to COMMAND payloads.
static constexpr uint8_t HMAC_LEN     = 8U;

/// Expected key length in bytes (128-bit key).
static constexpr uint8_t HMAC_KEY_LEN = 16U;

// ── Low-level HMAC primitives ─────────────────────────────────────────────────

/**
 * @brief Compute a truncated HMAC-SHA256 tag.
 *
 * Writes the first @c HMAC_LEN bytes of HMAC-SHA256(key, msg) to @p mac.
 * Portable implementation — no external libraries (PO10-3).
 *
 * @param[in]  key     Key bytes (must not be nullptr).
 * @param[in]  keyLen  Key length in bytes (0 < keyLen ≤ 64).
 * @param[in]  msg     Message bytes (must not be nullptr when msgLen > 0).
 * @param[in]  msgLen  Message length in bytes.
 * @param[out] mac     Destination buffer, exactly @c HMAC_LEN bytes.
 *
 * @pre key  != nullptr
 * @pre mac  != nullptr
 * @pre keyLen > 0
 */
void computeHmac8(const uint8_t* key,    uint8_t  keyLen,
                  const uint8_t* msg,    uint16_t msgLen,
                  uint8_t*       mac);

/**
 * @brief Constant-time comparison of a truncated HMAC-SHA256 tag.
 *
 * Computes the expected tag and compares it to @p expected using a
 * byte-XOR accumulator so the comparison always runs to completion
 * regardless of where a mismatch occurs (prevents timing oracles).
 *
 * @param[in]  key      Key bytes.
 * @param[in]  keyLen   Key length in bytes.
 * @param[in]  msg      Message bytes.
 * @param[in]  msgLen   Message length in bytes.
 * @param[in]  expected Tag to verify against (exactly @c HMAC_LEN bytes).
 * @return true  if the computed tag matches @p expected.
 *         false otherwise.
 */
bool verifyHmac8(const uint8_t* key,      uint8_t  keyLen,
                 const uint8_t* msg,      uint16_t msgLen,
                 const uint8_t* expected);

// ── High-level frame MAC operations ──────────────────────────────────────────

/**
 * @brief Append a MAC tag to a COMMAND frame and set FLAG_MAC.
 *
 * Sets FLAG_MAC in @c frame.flags, computes HMAC-SHA256 over the MAC input
 * (APUS-17: header fields + command payload), appends @c HMAC_LEN bytes to
 * @c frame.payload, and increments @c frame.len by @c HMAC_LEN.
 *
 * @param[in]     key     MAC key bytes.
 * @param[in]     keyLen  Key length in bytes.
 * @param[in,out] frame   COMMAND frame to authenticate.
 * @return true on success.
 *         false if @c frame.type != COMMAND, @c frame.len > MAX_PAYLOAD_LEN - HMAC_LEN,
 *         or @p key is nullptr.
 *
 * @pre frame.type == MsgType::COMMAND
 * @pre frame.len  <= MAX_PAYLOAD_LEN - HMAC_LEN
 */
bool appendCommandMac(const uint8_t* key, uint8_t keyLen, Frame& frame);

/**
 * @brief Verify the MAC tag on a decoded COMMAND frame.
 *
 * Extracts the MAC tag (last @c HMAC_LEN bytes of @c frame.payload),
 * rebuilds the MAC input, and compares using a constant-time comparison.
 *
 * @param[in] key     MAC key bytes.
 * @param[in] keyLen  Key length in bytes.
 * @param[in] frame   Decoded COMMAND frame carrying FLAG_MAC.
 * @return true  if the frame carries FLAG_MAC, has sufficient payload length,
 *               and the recomputed tag matches.
 *         false otherwise.
 *
 * @note Does NOT strip the MAC bytes from @c frame.  Callers that dispatch
 *       the command payload should compute @c cmd_len = frame.len - HMAC_LEN
 *       and use only @c frame.payload[0 .. cmd_len-1].
 */
bool verifyCommandMac(const uint8_t* key, uint8_t keyLen,
                      const Frame& frame);

} // namespace proto
} // namespace ares

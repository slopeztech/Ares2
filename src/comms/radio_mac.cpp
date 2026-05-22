/**
 * @file  radio_mac.cpp
 * @brief Portable HMAC-SHA256 for APUS radio COMMAND authentication (APUS-17).
 *
 * SHA-256 implemented per FIPS 180-4 §6.2.  HMAC constructed per RFC 2104.
 * No external libraries — compiles on both ESP32 and native host targets.
 *
 * All buffers are stack-allocated (PO10-3).  No VLAs (MISRA-18.8).
 * Every arithmetic result stored into a narrower type is explicitly cast
 * (MISRA Rules 10.3, 10.4).
 */

#include "comms/radio_mac.h"

#include <cstring>

namespace ares
{
namespace proto
{

// ── SHA-256 internals ─────────────────────────────────────────────────────────

namespace
{

static constexpr uint8_t kSha256Block  = 64U;  ///< SHA-256 block size (bytes).
static constexpr uint8_t kSha256Digest = 32U;  ///< SHA-256 digest size (bytes).

// FIPS 180-4 §4.2.2 — cube-root fractional parts of first 64 primes.
static const uint32_t kK[64] = {
    0x428a2f98U, 0x71374491U, 0xb5c0fbcfU, 0xe9b5dba5U,
    0x3956c25bU, 0x59f111f1U, 0x923f82a4U, 0xab1c5ed5U,
    0xd807aa98U, 0x12835b01U, 0x243185beU, 0x550c7dc3U,
    0x72be5d74U, 0x80deb1feU, 0x9bdc06a7U, 0xc19bf174U,
    0xe49b69c1U, 0xefbe4786U, 0x0fc19dc6U, 0x240ca1ccU,
    0x2de92c6fU, 0x4a7484aaU, 0x5cb0a9dcU, 0x76f988daU,
    0x983e5152U, 0xa831c66dU, 0xb00327c8U, 0xbf597fc7U,
    0xc6e00bf3U, 0xd5a79147U, 0x06ca6351U, 0x14292967U,
    0x27b70a85U, 0x2e1b2138U, 0x4d2c6dfcU, 0x53380d13U,
    0x650a7354U, 0x766a0abbU, 0x81c2c92eU, 0x92722c85U,
    0xa2bfe8a1U, 0xa81a664bU, 0xc24b8b70U, 0xc76c51a3U,
    0xd192e819U, 0xd6990624U, 0xf40e3585U, 0x106aa070U,
    0x19a4c116U, 0x1e376c08U,  0x2748774cU, 0x34b0bcb5U,
    0x391c0cb3U, 0x4ed8aa4aU, 0x5b9cca4fU, 0x682e6ff3U,
    0x748f82eeU, 0x78a5636fU, 0x84c87814U, 0x8cc70208U,
    0x90befffaU, 0xa4506cebU, 0xbef9a3f7U, 0xc67178f2U,
};

// FIPS 180-4 §5.3.3 — square-root fractional parts of first 8 primes.
static const uint32_t kH0[8] = {
    0x6a09e667U, 0xbb67ae85U, 0x3c6ef372U, 0xa54ff53aU,
    0x510e527fU, 0x9b05688cU, 0x1f83d9abU, 0x5be0cd19U,
};

struct Sha256Ctx
{
    uint32_t h[8];               ///< Current hash state.
    uint64_t bitCount;           ///< Total bits processed so far.
    uint8_t  buf[kSha256Block];  ///< Partial block buffer.
    uint8_t  bufLen;             ///< Valid bytes in buf (0 ≤ bufLen < kSha256Block).
};

static inline uint32_t rotr32(uint32_t x, uint8_t n)
{
    return (x >> n) | (x << static_cast<uint8_t>(32U - n));
}

/// Load four big-endian bytes as a uint32_t (FIPS 180-4 §3.1).
static inline uint32_t load32be(const uint8_t* p)
{
    return (static_cast<uint32_t>(p[0]) << 24U)
         | (static_cast<uint32_t>(p[1]) << 16U)
         | (static_cast<uint32_t>(p[2]) <<  8U)
         |  static_cast<uint32_t>(p[3]);
}

/// Expand 16-word message block to 64-word schedule (FIPS 180-4 §6.2.2 step 1).
static void buildSchedule(const uint8_t* block, uint32_t w[64])
{
    for (uint8_t i = 0U; i < 16U; ++i)
    {
        w[i] = load32be(&block[static_cast<uint8_t>(i * 4U)]);
    }
    for (uint8_t i = 16U; i < 64U; ++i)
    {
        const uint32_t s0 = rotr32(w[i - 15U], 7U)
                          ^ rotr32(w[i - 15U], 18U)
                          ^ (w[i - 15U] >> 3U);
        const uint32_t s1 = rotr32(w[i -  2U], 17U)
                          ^ rotr32(w[i -  2U], 19U)
                          ^ (w[i -  2U] >> 10U);
        w[i] = w[i - 16U] + s0 + w[i - 7U] + s1;
    }
}

/// Execute one SHA-256 compression round (FIPS 180-4 §6.2.2, steps 3a-3h).
/// @p s  Working state array [a,b,c,d,e,f,g,h] modified in place.
static void sha256Round(uint32_t* s, uint32_t wi, uint32_t ki)
{
    const uint32_t capS1 = rotr32(s[4], 6U) ^ rotr32(s[4], 11U) ^ rotr32(s[4], 25U);
    const uint32_t ch    = (s[4] & s[5]) ^ (~s[4] & s[6]);
    const uint32_t t1    = s[7] + capS1 + ch + ki + wi;
    const uint32_t capS0 = rotr32(s[0], 2U) ^ rotr32(s[0], 13U) ^ rotr32(s[0], 22U);
    const uint32_t maj   = (s[0] & s[1]) ^ (s[0] & s[2]) ^ (s[1] & s[2]);

    s[7] = s[6];   s[6] = s[5];   s[5] = s[4];
    s[4] = s[3] + t1;
    s[3] = s[2];   s[2] = s[1];   s[1] = s[0];
    s[0] = t1 + capS0 + maj;
}

/// Compress one 64-byte block into the SHA-256 state (FIPS 180-4 §6.2.2).
static void sha256ProcessBlock(Sha256Ctx& ctx, const uint8_t* block)
{
    uint32_t w[64];
    buildSchedule(block, w);

    uint32_t s[8];
    for (uint8_t i = 0U; i < 8U; ++i) { s[i] = ctx.h[i]; }

    for (uint8_t i = 0U; i < 64U; ++i) { sha256Round(s, w[i], kK[i]); }

    for (uint8_t i = 0U; i < 8U; ++i) { ctx.h[i] += s[i]; }
}

static void sha256Init(Sha256Ctx& ctx)
{
    (void)memcpy(ctx.h, kH0, sizeof(ctx.h));
    ctx.bitCount = 0U;
    ctx.bufLen   = 0U;
    (void)memset(ctx.buf, 0, sizeof(ctx.buf));
}

static void sha256Update(Sha256Ctx& ctx, const uint8_t* data, uint16_t len)
{
    ctx.bitCount += static_cast<uint64_t>(len) * 8U;
    uint16_t off = 0U;

    if (ctx.bufLen > 0U)
    {
        const uint8_t space = static_cast<uint8_t>(kSha256Block - ctx.bufLen);
        const uint8_t copy  = (static_cast<uint16_t>(len) <= static_cast<uint16_t>(space))
                              ? static_cast<uint8_t>(len) : space;
        (void)memcpy(&ctx.buf[ctx.bufLen], data, copy);
        ctx.bufLen = static_cast<uint8_t>(ctx.bufLen + copy);
        off        = static_cast<uint16_t>(off + copy);
        if (ctx.bufLen == kSha256Block)
        {
            sha256ProcessBlock(ctx, ctx.buf);
            ctx.bufLen = 0U;
        }
    }

    while (static_cast<uint16_t>(len - off) >= static_cast<uint16_t>(kSha256Block))
    {
        sha256ProcessBlock(ctx, &data[off]);
        off = static_cast<uint16_t>(off + kSha256Block);
    }

    const uint8_t rem = static_cast<uint8_t>(len - off);
    if (rem > 0U)
    {
        (void)memcpy(ctx.buf, &data[off], rem);
        ctx.bufLen = rem;
    }
}

/// Finalise the SHA-256 digest and write 32 bytes to @p digest.
static void sha256Final(Sha256Ctx& ctx, uint8_t digest[kSha256Digest])
{
    // Append 0x80 padding byte.
    ctx.buf[ctx.bufLen] = 0x80U;
    ctx.bufLen = static_cast<uint8_t>(ctx.bufLen + 1U);

    // If the partial block has no room for the 8-byte length field,
    // zero-pad and compress it now, then start a fresh block.
    if (ctx.bufLen > 56U)
    {
        (void)memset(&ctx.buf[ctx.bufLen], 0U,
                     static_cast<size_t>(kSha256Block - ctx.bufLen));
        sha256ProcessBlock(ctx, ctx.buf);
        ctx.bufLen = 0U;
    }

    (void)memset(&ctx.buf[ctx.bufLen], 0U, static_cast<size_t>(56U - ctx.bufLen));

    // Encode bit count as big-endian 64-bit integer (FIPS 180-4 §5.1.1).
    const uint64_t bc = ctx.bitCount;
    ctx.buf[56] = static_cast<uint8_t>((bc >> 56U) & 0xFFU);
    ctx.buf[57] = static_cast<uint8_t>((bc >> 48U) & 0xFFU);
    ctx.buf[58] = static_cast<uint8_t>((bc >> 40U) & 0xFFU);
    ctx.buf[59] = static_cast<uint8_t>((bc >> 32U) & 0xFFU);
    ctx.buf[60] = static_cast<uint8_t>((bc >> 24U) & 0xFFU);
    ctx.buf[61] = static_cast<uint8_t>((bc >> 16U) & 0xFFU);
    ctx.buf[62] = static_cast<uint8_t>((bc >>  8U) & 0xFFU);
    ctx.buf[63] = static_cast<uint8_t>( bc          & 0xFFU);
    sha256ProcessBlock(ctx, ctx.buf);

    // Serialise state as big-endian 32-bit words.
    for (uint8_t i = 0U; i < 8U; ++i)
    {
        const uint8_t b = static_cast<uint8_t>(i * 4U);
        digest[b    ] = static_cast<uint8_t>((ctx.h[i] >> 24U) & 0xFFU);
        digest[b + 1U] = static_cast<uint8_t>((ctx.h[i] >> 16U) & 0xFFU);
        digest[b + 2U] = static_cast<uint8_t>((ctx.h[i] >>  8U) & 0xFFU);
        digest[b + 3U] = static_cast<uint8_t>( ctx.h[i]          & 0xFFU);
    }
}

} // anonymous namespace

// ── Public HMAC-SHA256 API ────────────────────────────────────────────────────

void computeHmac8(const uint8_t* key,    uint8_t  keyLen,
                  const uint8_t* msg,    uint16_t msgLen,
                  uint8_t*       mac)
{
    // Normalise key: if keyLen > block size, key = SHA256(key) (RFC 2104 §2).
    uint8_t paddedKey[kSha256Block] = {};
    if (keyLen > kSha256Block)
    {
        Sha256Ctx ctx;
        sha256Init(ctx);
        sha256Update(ctx, key, static_cast<uint16_t>(keyLen));
        sha256Final(ctx, paddedKey);
    }
    else
    {
        (void)memcpy(paddedKey, key, keyLen);
    }

    // Build ipad-XOR and opad-XOR key variants (RFC 2104 §2).
    uint8_t ikey[kSha256Block];
    uint8_t okey[kSha256Block];
    for (uint8_t i = 0U; i < kSha256Block; ++i)
    {
        ikey[i] = static_cast<uint8_t>(paddedKey[i] ^ 0x36U);
        okey[i] = static_cast<uint8_t>(paddedKey[i] ^ 0x5CU);
    }

    // Inner hash: SHA256(ikey || msg).
    uint8_t inner[kSha256Digest];
    {
        Sha256Ctx ctx;
        sha256Init(ctx);
        sha256Update(ctx, ikey, static_cast<uint16_t>(kSha256Block));
        sha256Update(ctx, msg,  msgLen);
        sha256Final(ctx, inner);
    }

    // Outer hash: SHA256(okey || inner).
    uint8_t full[kSha256Digest];
    {
        Sha256Ctx ctx;
        sha256Init(ctx);
        sha256Update(ctx, okey,  static_cast<uint16_t>(kSha256Block));
        sha256Update(ctx, inner, static_cast<uint16_t>(kSha256Digest));
        sha256Final(ctx, full);
    }

    (void)memcpy(mac, full, HMAC_LEN);
}

bool verifyHmac8(const uint8_t* key,      uint8_t  keyLen,
                 const uint8_t* msg,      uint16_t msgLen,
                 const uint8_t* expected)
{
    uint8_t computed[HMAC_LEN] = {};
    computeHmac8(key, keyLen, msg, msgLen, computed);

    // Constant-time comparison — full loop regardless of mismatch position
    // (prevents timing side-channel, OWASP A07).
    uint8_t diff = 0U;
    for (uint8_t i = 0U; i < HMAC_LEN; ++i)
    {
        diff |= static_cast<uint8_t>(computed[i] ^ expected[i]);
    }
    return diff == 0U;
}

// ── Frame-level helpers ───────────────────────────────────────────────────────

namespace
{

/**
 * @brief Build the MAC input byte sequence from frame header + command payload.
 *
 * MAC_input = [VER(1), FLAGS(1), NODE(1), TYPE(1), SEQ(1), cmd_len(1),
 *              payload[0 .. cmd_len-1]]
 *
 * @param[in]  frame    Frame carrying FLAG_MAC (flags must already reflect it).
 * @param[in]  cmdLen   Actual command payload length (= frame.len - HMAC_LEN).
 * @param[out] buf      Destination buffer (at least 6 + cmdLen bytes).
 * @return Total number of bytes written to @p buf.
 */
static uint16_t buildMacInput(const Frame& frame,
                               uint8_t      cmdLen,
                               uint8_t*     buf)
{
    buf[0] = frame.ver;
    buf[1] = frame.flags;
    buf[2] = frame.node;
    buf[3] = static_cast<uint8_t>(frame.type);
    buf[4] = frame.seq;
    buf[5] = cmdLen;

    if (cmdLen > 0U)
    {
        (void)memcpy(&buf[6U], frame.payload, cmdLen);
    }

    return static_cast<uint16_t>(6U + cmdLen);
}

} // anonymous namespace

bool appendCommandMac(const uint8_t* key, uint8_t keyLen, Frame& frame)
{
    if (key == nullptr)                                   { return false; }
    if (frame.type != MsgType::COMMAND)                   { return false; }
    if (frame.len > static_cast<uint8_t>(MAX_PAYLOAD_LEN - HMAC_LEN)) { return false; }

    // Set FLAG_MAC before building the input so the tag covers the updated flags.
    frame.flags = static_cast<uint8_t>(frame.flags | FLAG_MAC);

    const uint8_t cmdLen = frame.len;  // original payload length before appending

    // Build MAC input: 6-byte header fields + command payload.
    uint8_t macInput[6U + MAX_PAYLOAD_LEN] = {};
    const uint16_t inputLen = buildMacInput(frame, cmdLen, macInput);

    // Compute and append the truncated tag.
    computeHmac8(key, keyLen, macInput, inputLen, &frame.payload[cmdLen]);
    frame.len = static_cast<uint8_t>(cmdLen + HMAC_LEN);

    return true;
}

bool verifyCommandMac(const uint8_t* key, uint8_t keyLen,
                      const Frame& frame)
{
    if (key == nullptr)                              { return false; }
    if ((frame.flags & FLAG_MAC) == 0U)              { return false; }
    if (frame.type != MsgType::COMMAND)              { return false; }

    // Require at minimum sizeof(CommandHeader) of real payload + the MAC tag.
    static constexpr uint8_t kMinLen =
        static_cast<uint8_t>(sizeof(CommandHeader) + HMAC_LEN);
    if (frame.len < kMinLen)                         { return false; }

    const uint8_t cmdLen = static_cast<uint8_t>(frame.len - HMAC_LEN);

    // Rebuild the same MAC input used during appendCommandMac.
    uint8_t macInput[6U + MAX_PAYLOAD_LEN] = {};
    const uint16_t inputLen = buildMacInput(frame, cmdLen, macInput);

    // Constant-time comparison against the embedded tag.
    return verifyHmac8(key, keyLen, macInput, inputLen, &frame.payload[cmdLen]);
}

} // namespace proto
} // namespace ares

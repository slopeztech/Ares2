/**
 * @file  test_radio_mac.cpp
 * @brief Unit tests for the HMAC-SHA256 radio authentication layer (APUS-17).
 *
 * Coverage:
 *   Primitives:
 *     TC-MAC-1  computeHmac8 — RFC 4231 Test Case 1 (known-good vector)
 *     TC-MAC-2  computeHmac8 — RFC 4231 Test Case 2 (known-good vector)
 *     TC-MAC-3  computeHmac8 is deterministic (same in → same out)
 *     TC-MAC-4  computeHmac8 produces distinct output for different keys
 *     TC-MAC-5  computeHmac8 produces distinct output for different messages
 *     TC-MAC-6  verifyHmac8 accepts a valid tag
 *     TC-MAC-7  verifyHmac8 rejects a single-bit-flipped tag
 *
 *   Frame-level operations:
 *     TC-MAC-8   appendCommandMac sets FLAG_MAC and increments len by HMAC_LEN
 *     TC-MAC-9   appendCommandMac rejects non-COMMAND frame types
 *     TC-MAC-10  appendCommandMac rejects oversized frames (not enough room for tag)
 *     TC-MAC-11  verifyCommandMac round-trip with matching key passes
 *     TC-MAC-12  verifyCommandMac fails with a different key
 *     TC-MAC-13  verifyCommandMac fails when FLAG_MAC is not set
 */

#include <unity.h>

#include "comms/ares_radio_protocol.h"
#include "comms/radio_mac.h"

#include <cstring>

using namespace ares::proto;

// ─────────────────────────────────────────────────────────────────────────────
//  Helpers
// ─────────────────────────────────────────────────────────────────────────────

/** Build a minimal valid COMMAND frame with @p cmdLen payload bytes. */
static Frame makeCommandFrame(uint8_t cmdLen)
{
    Frame f = {};
    f.ver   = PROTOCOL_VERSION;
    f.flags = 0U;
    f.node  = NODE_GROUND;
    f.type  = MsgType::COMMAND;
    f.seq   = 1U;
    f.len   = cmdLen;
    // Populate payload with recognisable pattern.
    for (uint8_t i = 0U; i < cmdLen && i < MAX_PAYLOAD_LEN; i++)
    {
        f.payload[i] = static_cast<uint8_t>(0xA0U | (i & 0x0FU));
    }
    return f;
}

// ─────────────────────────────────────────────────────────────────────────────
//  TC-MAC-1: RFC 4231 Test Case 1
//  Key:  0x0b repeated 20 times (20 bytes)
//  Data: "Hi There"            (8 bytes)
//  Expected HMAC-SHA256 first 8 bytes:
//    b0 34 4c 61 d8 db 38 53
// ─────────────────────────────────────────────────────────────────────────────
void test_computeHmac8_rfc4231_tc1()
{
    static const uint8_t key[20] = {
        0x0b, 0x0b, 0x0b, 0x0b, 0x0b, 0x0b, 0x0b, 0x0b, 0x0b, 0x0b,
        0x0b, 0x0b, 0x0b, 0x0b, 0x0b, 0x0b, 0x0b, 0x0b, 0x0b, 0x0b,
    };
    static const uint8_t msg[8] = {
        'H', 'i', ' ', 'T', 'h', 'e', 'r', 'e'
    };
    static const uint8_t expected[HMAC_LEN] = {
        0xb0, 0x34, 0x4c, 0x61, 0xd8, 0xdb, 0x38, 0x53
    };

    uint8_t mac[HMAC_LEN] = {};
    computeHmac8(key, static_cast<uint8_t>(sizeof(key)),
                 msg, static_cast<uint16_t>(sizeof(msg)),
                 mac);

    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected, mac, HMAC_LEN);
}

// ─────────────────────────────────────────────────────────────────────────────
//  TC-MAC-2: RFC 4231 Test Case 2
//  Key:  "Jefe"                 (4 bytes: 0x4a 0x65 0x66 0x65)
//  Data: "what do ya want for nothing?" (28 bytes)
//  Expected HMAC-SHA256 first 8 bytes:
//    5b dc c1 46 bf 60 75 4e
// ─────────────────────────────────────────────────────────────────────────────
void test_computeHmac8_rfc4231_tc2()
{
    static const uint8_t key[4] = { 0x4a, 0x65, 0x66, 0x65 };
    static const uint8_t msg[28] = {
        'w','h','a','t',' ','d','o',' ','y','a',' ',
        'w','a','n','t',' ','f','o','r',' ',
        'n','o','t','h','i','n','g','?'
    };
    static const uint8_t expected[HMAC_LEN] = {
        0x5b, 0xdc, 0xc1, 0x46, 0xbf, 0x60, 0x75, 0x4e
    };

    uint8_t mac[HMAC_LEN] = {};
    computeHmac8(key, static_cast<uint8_t>(sizeof(key)),
                 msg, static_cast<uint16_t>(sizeof(msg)),
                 mac);

    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected, mac, HMAC_LEN);
}

// ─────────────────────────────────────────────────────────────────────────────
//  TC-MAC-3: determinism — same input always produces same output
// ─────────────────────────────────────────────────────────────────────────────
void test_computeHmac8_deterministic()
{
    static const uint8_t key[HMAC_KEY_LEN] = {
        0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
        0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f, 0x10,
    };
    static const uint8_t msg[6] = { 0x41, 0x52, 0x45, 0x53, 0x32, 0x00 };

    uint8_t mac1[HMAC_LEN] = {};
    uint8_t mac2[HMAC_LEN] = {};
    computeHmac8(key, HMAC_KEY_LEN, msg, static_cast<uint16_t>(sizeof(msg)), mac1);
    computeHmac8(key, HMAC_KEY_LEN, msg, static_cast<uint16_t>(sizeof(msg)), mac2);

    TEST_ASSERT_EQUAL_UINT8_ARRAY(mac1, mac2, HMAC_LEN);
}

// ─────────────────────────────────────────────────────────────────────────────
//  TC-MAC-4: different key → different MAC
// ─────────────────────────────────────────────────────────────────────────────
void test_computeHmac8_distinct_keys()
{
    static const uint8_t keyA[HMAC_KEY_LEN] = {
        0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88,
        0x99, 0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff, 0x00,
    };
    static const uint8_t keyB[HMAC_KEY_LEN] = {
        0x00, 0xff, 0xee, 0xdd, 0xcc, 0xbb, 0xaa, 0x99,
        0x88, 0x77, 0x66, 0x55, 0x44, 0x33, 0x22, 0x11,
    };
    static const uint8_t msg[4] = { 0xDE, 0xAD, 0xBE, 0xEF };

    uint8_t macA[HMAC_LEN] = {};
    uint8_t macB[HMAC_LEN] = {};
    computeHmac8(keyA, HMAC_KEY_LEN, msg, static_cast<uint16_t>(sizeof(msg)), macA);
    computeHmac8(keyB, HMAC_KEY_LEN, msg, static_cast<uint16_t>(sizeof(msg)), macB);

    // They must differ in at least one byte.
    bool differ = false;
    for (uint8_t i = 0U; i < HMAC_LEN; i++)
    {
        if (macA[i] != macB[i]) { differ = true; }
    }
    TEST_ASSERT_TRUE(differ);
}

// ─────────────────────────────────────────────────────────────────────────────
//  TC-MAC-5: different message → different MAC
// ─────────────────────────────────────────────────────────────────────────────
void test_computeHmac8_distinct_msgs()
{
    static const uint8_t key[HMAC_KEY_LEN] = {
        0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF, 0x00, 0x11,
        0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99,
    };
    static const uint8_t msgA[3] = { 0x01, 0x02, 0x03 };
    static const uint8_t msgB[3] = { 0x01, 0x02, 0x04 };  // one bit different

    uint8_t macA[HMAC_LEN] = {};
    uint8_t macB[HMAC_LEN] = {};
    computeHmac8(key, HMAC_KEY_LEN, msgA, static_cast<uint16_t>(sizeof(msgA)), macA);
    computeHmac8(key, HMAC_KEY_LEN, msgB, static_cast<uint16_t>(sizeof(msgB)), macB);

    bool differ = false;
    for (uint8_t i = 0U; i < HMAC_LEN; i++)
    {
        if (macA[i] != macB[i]) { differ = true; }
    }
    TEST_ASSERT_TRUE(differ);
}

// ─────────────────────────────────────────────────────────────────────────────
//  TC-MAC-6: verifyHmac8 accepts correct tag
// ─────────────────────────────────────────────────────────────────────────────
void test_verifyHmac8_accepts_valid()
{
    static const uint8_t key[HMAC_KEY_LEN] = {
        0x10, 0x20, 0x30, 0x40, 0x50, 0x60, 0x70, 0x80,
        0x90, 0xa0, 0xb0, 0xc0, 0xd0, 0xe0, 0xf0, 0x01,
    };
    static const uint8_t msg[5] = { 0x05, 0x04, 0x03, 0x02, 0x01 };

    uint8_t mac[HMAC_LEN] = {};
    computeHmac8(key, HMAC_KEY_LEN, msg, static_cast<uint16_t>(sizeof(msg)), mac);

    TEST_ASSERT_TRUE(verifyHmac8(key, HMAC_KEY_LEN,
                                 msg, static_cast<uint16_t>(sizeof(msg)),
                                 mac));
}

// ─────────────────────────────────────────────────────────────────────────────
//  TC-MAC-7: verifyHmac8 rejects a corrupted tag (single-byte flip)
// ─────────────────────────────────────────────────────────────────────────────
void test_verifyHmac8_rejects_corrupted()
{
    static const uint8_t key[HMAC_KEY_LEN] = {
        0x10, 0x20, 0x30, 0x40, 0x50, 0x60, 0x70, 0x80,
        0x90, 0xa0, 0xb0, 0xc0, 0xd0, 0xe0, 0xf0, 0x01,
    };
    static const uint8_t msg[5] = { 0x05, 0x04, 0x03, 0x02, 0x01 };

    uint8_t mac[HMAC_LEN] = {};
    computeHmac8(key, HMAC_KEY_LEN, msg, static_cast<uint16_t>(sizeof(msg)), mac);
    mac[0] ^= 0x01U;  // flip one bit

    TEST_ASSERT_FALSE(verifyHmac8(key, HMAC_KEY_LEN,
                                  msg, static_cast<uint16_t>(sizeof(msg)),
                                  mac));
}

// ─────────────────────────────────────────────────────────────────────────────
//  TC-MAC-8: appendCommandMac sets FLAG_MAC and increments len by HMAC_LEN
// ─────────────────────────────────────────────────────────────────────────────
void test_appendCommandMac_sets_flag_and_len()
{
    static const uint8_t key[HMAC_KEY_LEN] = {
        0x01, 0x23, 0x45, 0x67, 0x89, 0xab, 0xcd, 0xef,
        0xfe, 0xdc, 0xba, 0x98, 0x76, 0x54, 0x32, 0x10,
    };

    Frame f = makeCommandFrame(4U);  // 4-byte payload (CMD header + 2 params)
    const uint8_t originalLen = f.len;

    TEST_ASSERT_TRUE(appendCommandMac(key, HMAC_KEY_LEN, f));

    TEST_ASSERT_BITS(FLAG_MAC, FLAG_MAC, f.flags);
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(originalLen + HMAC_LEN), f.len);
}

// ─────────────────────────────────────────────────────────────────────────────
//  TC-MAC-9: appendCommandMac rejects non-COMMAND frame type
// ─────────────────────────────────────────────────────────────────────────────
void test_appendCommandMac_rejects_non_command()
{
    static const uint8_t key[HMAC_KEY_LEN] = {
        0x01, 0x23, 0x45, 0x67, 0x89, 0xab, 0xcd, 0xef,
        0xfe, 0xdc, 0xba, 0x98, 0x76, 0x54, 0x32, 0x10,
    };

    Frame f = {};
    f.ver  = PROTOCOL_VERSION;
    f.type = MsgType::TELEMETRY;
    f.node = NODE_ROCKET;
    f.len  = 10U;

    TEST_ASSERT_FALSE(appendCommandMac(key, HMAC_KEY_LEN, f));
    TEST_ASSERT_EQUAL_UINT8(0U, f.flags & FLAG_MAC);  // flag must not be set
}

// ─────────────────────────────────────────────────────────────────────────────
//  TC-MAC-10: appendCommandMac rejects oversized frame (no room for tag)
// ─────────────────────────────────────────────────────────────────────────────
void test_appendCommandMac_rejects_oversized()
{
    static const uint8_t key[HMAC_KEY_LEN] = {
        0x01, 0x23, 0x45, 0x67, 0x89, 0xab, 0xcd, 0xef,
        0xfe, 0xdc, 0xba, 0x98, 0x76, 0x54, 0x32, 0x10,
    };

    Frame f = {};
    f.ver  = PROTOCOL_VERSION;
    f.type = MsgType::COMMAND;
    f.node = NODE_GROUND;
    // len set to the maximum payload so there is no room for 8 MAC bytes.
    f.len  = MAX_PAYLOAD_LEN;

    TEST_ASSERT_FALSE(appendCommandMac(key, HMAC_KEY_LEN, f));
}

// ─────────────────────────────────────────────────────────────────────────────
//  TC-MAC-11: verifyCommandMac round-trip with same key passes
// ─────────────────────────────────────────────────────────────────────────────
void test_verifyCommandMac_roundtrip()
{
    static const uint8_t key[HMAC_KEY_LEN] = {
        0xde, 0xad, 0xbe, 0xef, 0xca, 0xfe, 0xba, 0xbe,
        0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
    };

    Frame f = makeCommandFrame(6U);

    TEST_ASSERT_TRUE(appendCommandMac(key, HMAC_KEY_LEN, f));
    TEST_ASSERT_TRUE(verifyCommandMac(key, HMAC_KEY_LEN, f));
}

// ─────────────────────────────────────────────────────────────────────────────
//  TC-MAC-12: verifyCommandMac fails with a wrong key
// ─────────────────────────────────────────────────────────────────────────────
void test_verifyCommandMac_wrong_key()
{
    static const uint8_t keyA[HMAC_KEY_LEN] = {
        0xde, 0xad, 0xbe, 0xef, 0xca, 0xfe, 0xba, 0xbe,
        0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
    };
    static const uint8_t keyB[HMAC_KEY_LEN] = {
        0xba, 0xd0, 0xba, 0xd0, 0xba, 0xd0, 0xba, 0xd0,
        0xba, 0xd0, 0xba, 0xd0, 0xba, 0xd0, 0xba, 0xd0,
    };

    Frame f = makeCommandFrame(4U);

    TEST_ASSERT_TRUE(appendCommandMac(keyA, HMAC_KEY_LEN, f));
    TEST_ASSERT_FALSE(verifyCommandMac(keyB, HMAC_KEY_LEN, f));
}

// ─────────────────────────────────────────────────────────────────────────────
//  TC-MAC-13: verifyCommandMac fails when FLAG_MAC is not set
// ─────────────────────────────────────────────────────────────────────────────
void test_verifyCommandMac_no_flag()
{
    static const uint8_t key[HMAC_KEY_LEN] = {
        0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88,
        0x99, 0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff, 0x00,
    };

    Frame f = makeCommandFrame(4U);
    // Explicitly do NOT set FLAG_MAC.
    f.flags &= static_cast<uint8_t>(~FLAG_MAC);

    TEST_ASSERT_FALSE(verifyCommandMac(key, HMAC_KEY_LEN, f));
}

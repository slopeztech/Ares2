/**
 * @file  test_crc.cpp
 * @brief Unit tests for ares::proto::crc32() (APUS-1 / CRC-32 Ethernet).
 *
 * Covers:
 *   - Standard PKZIP/Ethernet check value ("123456789" → 0xCBF43926)
 *   - Empty input (len = 0 → 0x00000000)
 *   - Single-bit-flip sensitivity
 *   - Byte-order sensitivity
 */
#include <unity.h>

#include "ares_radio_protocol.h"

using namespace ares::proto;

// ── CRC-32 (Ethernet/PKZIP, reflected) tests ───────────────

/// Standard CRC-32 check value: crc32("123456789") must equal 0xCBF43926.
/// This is the canonical acceptance test for this polynomial.
void test_crc32_check_value()
{
    static const uint8_t data[] = {
        '1', '2', '3', '4', '5', '6', '7', '8', '9'
    };
    TEST_ASSERT_EQUAL_UINT32(0xCBF43926U, crc32(data, sizeof(data)));
}

/// Zero-length input: init XOR final cancel out → 0x00000000.
void test_crc32_empty_input()
{
    const uint8_t dummy = 0U;
    TEST_ASSERT_EQUAL_UINT32(0x00000000U, crc32(&dummy, 0U));
}

/// A single bit flip must produce a different CRC (Hamming distance ≥ 2).
void test_crc32_detects_single_bit_flip()
{
    uint8_t data[] = {0xDE, 0xAD, 0xBE, 0xEF};
    const uint32_t original = crc32(data, sizeof(data));

    data[2] ^= 0x01U;  // flip one bit
    const uint32_t flipped = crc32(data, sizeof(data));

    TEST_ASSERT_TRUE(original != flipped);

    data[2] ^= 0x01U;  // restore
}

/// The CRC must be sensitive to byte order (not just content).
void test_crc32_order_sensitive()
{
    static const uint8_t ab[] = {0xAA, 0xBB};
    static const uint8_t ba[] = {0xBB, 0xAA};

    TEST_ASSERT_TRUE(crc32(ab, sizeof(ab)) != crc32(ba, sizeof(ba)));
}

/// Single-byte inputs produce distinct CRCs for distinct values.
void test_crc32_single_byte_distinct()
{
    const uint8_t a = 0x01U;
    const uint8_t b = 0x02U;
    TEST_ASSERT_TRUE(crc32(&a, 1U) != crc32(&b, 1U));
}

/// crc32 is deterministic: identical inputs must always produce identical output.
void test_crc32_deterministic()
{
    static const uint8_t data[] = {0x11, 0x22, 0x33};
    const uint32_t first  = crc32(data, sizeof(data));
    const uint32_t second = crc32(data, sizeof(data));
    TEST_ASSERT_EQUAL_UINT32(first, second);
}

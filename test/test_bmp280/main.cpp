/**
 * @file  main.cpp
 * @brief Unity runner for test_bmp280 tests.
 *
 * Test count: 6
 */
#include <unity.h>

void setUp()    {}
void tearDown() {}

// ── test_bmp280_begin.cpp ─────────────────────────────────────────────────────

extern void test_bmp280_begin_happy_path();
extern void test_bmp280_begin_wrong_chip_id_rejected();
extern void test_bmp280_begin_status_reg_i2c_fail_returns_false();
extern void test_bmp280_read_not_ready();
extern void test_bmp280_read_happy_path();
extern void test_bmp280_set_sea_level_pressure();

// ── Runner ────────────────────────────────────────────────────────────────────

int main()
{
    UNITY_BEGIN();

    RUN_TEST(test_bmp280_begin_happy_path);
    RUN_TEST(test_bmp280_begin_wrong_chip_id_rejected);
    RUN_TEST(test_bmp280_begin_status_reg_i2c_fail_returns_false);
    RUN_TEST(test_bmp280_read_not_ready);
    RUN_TEST(test_bmp280_read_happy_path);
    RUN_TEST(test_bmp280_set_sea_level_pressure);

    return UNITY_END();
}

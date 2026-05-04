/**
 * @file  adxl375_spi_driver.h
 * @brief ADXL375 SPI high-g accelerometer driver (Analog Devices ADXL375).
 *
 * Implements ImuInterface for the ADXL375 ±200 g shock accelerometer
 * over the 4-wire SPI bus.  The SPI variant is preferred when the I2C
 * bus is saturated or faster burst rates are required.
 *
 * SPI configuration (set by begin()):
 *   - Mode: SPI Mode 3 (CPOL=1, CPHA=1) — SCLK idle high, data
 *     captured on falling edge (ADXL375 datasheet §Serial Comm.).
 *   - Clock: 5 MHz max.
 *   - Byte order: MSB first.
 *   - CS: active-low GPIO, managed by the driver.
 *
 * Register framing:
 *   - Write : bit7=0 (R/W̄), bit6=0 (MB), bits[5:0]=address, then data byte.
 *   - Read  : bit7=1 (R/W̄), bit6=0/1 (MB=1 for multi-byte), bits[5:0]=addr.
 *
 * Measurement configuration (set by begin()):
 *   - Output data rate : 100 Hz  (BW_RATE = 0x0A, normal power mode).
 *   - Resolution       : 49 mg/LSB (fixed, FULL_RES=1).
 *   - FIFO             : bypass mode.
 *
 * Thread safety: thread-safe.  An internal FreeRTOS mutex serialises
 *                concurrent read() calls across tasks (CERT-13).
 *
 * @note gyroX/Y/Z and tempC in ImuReading are always 0.0f — the
 *       ADXL375 provides acceleration only.
 */
#pragma once

#include <SPI.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#include "hal/imu/imu_interface.h"

/**
 * Concrete ImuInterface for the Analog Devices ADXL375 over SPI.
 *
 * Init sequence (begin()):
 *   1. Configure CS pin as OUTPUT, idle HIGH.
 *   2. Read DEVID register (0x00) — must return 0xE5.
 *   3. Write POWER_CTL (0x2D) = 0x00: enter standby (safe config writes).
 *   4. Write DATA_FORMAT (0x31) = 0x0B: FULL_RES=1, RANGE=11 (±200 g).
 *   5. Write BW_RATE (0x2C) = 0x0A: normal power, 100 Hz ODR.
 *   6. Write FIFO_CTL (0x38) = 0x00: bypass mode.
 *   7. Write POWER_CTL (0x2D) = 0x08: enter Measure mode.
 *   8. Wait ADXL375_SETTLE_MS for first valid sample.
 *
 * Reading (read()):
 *   1. Burst-read 6 bytes from DATAX0 (0x32) — multi-byte SPI frame.
 *   2. Each axis is little-endian signed 16-bit.
 *   3. Scale: raw * 49 mg/LSB * 9.80665 m/s²/g → m/s².
 */
class Adxl375SpiDriver : public ImuInterface
{
public:
    /**
     * Construct an ADXL375 SPI driver instance.
     * @param[in] spi    SPI bus (must be initialised via spi.begin() before begin()).
     * @param[in] csPin  GPIO number for the chip-select line (active-low).
     * @param[in] freqHz SPI clock frequency in Hz (≤ 5 000 000).
     */
    Adxl375SpiDriver(SPIClass& spi, uint8_t csPin, uint32_t freqHz);

    // Non-copyable, non-movable (CERT-18.3).
    Adxl375SpiDriver(const Adxl375SpiDriver&)            = delete;
    Adxl375SpiDriver& operator=(const Adxl375SpiDriver&) = delete;

    /**
     * Initialise the ADXL375 sensor over SPI.
     * @return true on success; false if DEVID check fails or write errors occur.
     * @post Sensor is in Measure mode, 100 Hz ODR on success.
     */
    bool begin() override;

    /**
     * Read the latest acceleration measurement.
     * @param[out] out  Populated on ImuStatus::OK.  gyroX/Y/Z and tempC are 0.0f.
     * @return Status code (OK, ERROR, or NOT_READY).
     */
    ImuStatus read(ImuReading& out) override;

    const char* driverModel() const override { return "ADXL375_SPI"; }

private:
    /// Assert CS low, transfer one write frame, deassert CS.
    bool writeReg(uint8_t reg, uint8_t value);
    /// Assert CS low, transfer read frame, read @p len bytes, deassert CS.
    bool readRegs(uint8_t reg, uint8_t* buf, uint8_t len);
    /// Inner read implementation — called while imuMutex_ is held.
    ImuStatus readLocked(ImuReading& out);

    SPIClass& spi_;                          ///< SPI bus (injected, not owned).
    uint8_t   csPin_;                        ///< Chip-select GPIO (active-low).
    uint32_t  freqHz_;                       ///< SPI clock frequency in Hz.
    bool      ready_               = false;  ///< true after successful begin().
    uint32_t  lastReinitAttemptMs_ = 0U;     ///< millis() of last lazy re-init attempt.
    uint8_t   consecutiveErrors_   = 0U;     ///< Consecutive readRegs() failures.
    SemaphoreHandle_t imuMutex_    = nullptr; ///< Serialises concurrent calls (CERT-13).
};

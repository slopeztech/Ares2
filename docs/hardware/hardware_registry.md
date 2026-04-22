# ARES Hardware Registry

**Board specifications and peripheral inventory**

---

## Board

### ESP32-S3 Zero Mini

| Property          | Value                                     |
|-------------------|-------------------------------------------|
| MCU               | ESP32-S3 (dual-core Xtensa LX7, 240 MHz) |
| Flash             | 4 MB                                      |
| RAM               | 512 KB SRAM + 8 MB PSRAM (optional)       |
| USB               | Native USB (CDC) + CH340 UART             |
| COM Port          | COM6 (CH340)                              |
| WiFi              | 2.4 GHz 802.11 b/g/n — confirmed working |
| Antenna           | PCB antenna                               |
| WS2812 LED        | GPIO 21                                   |
| Form Factor       | Waveshare Zero Mini (~21×52 mm)           |

---

## Peripherals

### DX-LR03-433T30D (LoRa Module)

| Property          | Value                               |
|-------------------|-------------------------------------|
| Type              | UART LoRa transceiver               |
| Frequency         | 433 MHz (ISM band)                  |
| Default Channel   | CH23 = 433.125 MHz                  |
| TX Power          | 30 dBm (1 W)                        |
| Air Data Rate     | 2.4 kbps                            |
| UART Baud         | 9600                                |
| Interface         | UART TX/RX + AUX (busy indicator)   |
| Mode Pins         | M0/M1 (internal pull-down → NORMAL) |
| Max Packet Size   | 240 bytes                           |

Operating modes (M1:M0):

| M1 | M0 | Mode        | Description               |
|----|----|-------------|---------------------------|
| 0  | 0  | NORMAL      | Transparent TX/RX         |
| 0  | 1  | WOR         | Wake on radio             |
| 1  | 0  | CONFIG      | Parameter configuration   |
| 1  | 1  | DEEP_SLEEP  | Low power                 |

M0/M1 are **not wired** on current prototypes. The module defaults to
NORMAL mode via internal pull-downs. Configuration is done via UART
commands.

### WS2812 RGB LED (Neopixel)

| Property     | Value       |
|--------------|-------------|
| Data Pin     | GPIO 21     |
| Type         | WS2812B     |

LED behaviour per operating mode:

| Mode     | Pattern                                      |
|----------|----------------------------------------------|
| IDLE     | Solid green                                  |
| TEST     | Slow cyan blink (500 ms on / 500 ms off)     |
| FLIGHT   | Solid blue                                   |
| RECOVERY | Slow green blink (300 ms on / 700 ms off)    |
| ERROR    | Fast red triple-blink (3×100 ms, 100 ms gap) |
| BOOT     | Fast green blink (300 ms on / 300 ms off) — system initialising |

---

### BMP280 Barometric Pressure Sensor

| Property          | Value                                     |
|-------------------|-------------------------------------------|
| Type              | I2C barometric pressure + temperature      |
| Sensor            | Bosch BMP280 (AHT20+BMP280 combo module)   |
| I2C Address       | 0x77 (SDO → VCC)                          |
| Pressure Range    | 300 – 1100 hPa                             |
| Altitude Res.     | ~10 cm (×16 oversampling + IIR ×16)       |
| Compensation      | Bosch integer algorithms (int32/int64)     |

Pins: I2C SDA/SCL per board (see [Pin Map](pin_map.md)).  
Full details: [barometer_bmp280.md](barometer_bmp280.md)

---

### BN-220 GPS Module

| Property          | Value                                      |
|-------------------|--------------------------------------------|
| Type              | UART GNSS receiver                         |
| Chipset           | u-blox UBX-M8030-KT                       |
| Antenna           | Built-in ceramic patch                     |
| Protocol          | NMEA 0183 (GGA + RMC)                     |
| UART Baud         | 9600                                       |
| UART Port         | UART1 (RX=GPIO 5, TX=GPIO 6)             |
| Fix Types         | No fix / 2D / 3D                           |
| Update Rate       | 1 Hz default (configurable up to 10 Hz)   |
| Cold Start        | ~26 s                                      |
| Hot Start         | ~1 s                                       |
| Supply            | 3.3 V                                     |

Pins: see [Pin Map](pin_map.md).  
Full details: [gps_bn220.md](gps_bn220.md)

---

### MPU-6050 IMU (Accelerometer + Gyroscope)

| Property          | Value                                         |
|-------------------|-----------------------------------------------|
| Type              | I2C 6-axis IMU (accelerometer + gyroscope)    |
| Sensor            | InvenSense MPU-6050                           |
| I2C Address       | 0x68 (AD0 → GND, default)                    |
| Accel Range       | ±2 g (configured), 16384 LSB/g               |
| Gyro Range        | ±250 deg/s (configured), 131 LSB/(deg/s)     |
| Sample Rate       | 100 Hz (SMPLRT_DIV=9, DLPF=94 Hz)            |
| On-chip Temp      | Yes (raw/340 + 36.53 °C)                      |
| Supply            | 3.3 V                                         |
| Driver            | `Mpu6050Driver` (`src/drivers/imu/`)          |
| HAL Interface     | `ImuInterface` (`src/hal/imu/`)               |

Pins: I2C SDA/SCL per board (see [Pin Map](pin_map.md)).

**AMS sensor tokens:**

| Token            | Unit   | Description                              |
|------------------|--------|------------------------------------------|
| `IMU.accel_x`    | m/s²   | Acceleration X axis                      |
| `IMU.accel_y`    | m/s²   | Acceleration Y axis                      |
| `IMU.accel_z`    | m/s²   | Acceleration Z axis                      |
| `IMU.accel_mag`  | m/s²   | Acceleration vector magnitude `√(x²+y²+z²)` |
| `IMU.gyro_x`     | deg/s  | Angular rate X axis                      |
| `IMU.gyro_y`     | deg/s  | Angular rate Y axis                      |
| `IMU.gyro_z`     | deg/s  | Angular rate Z axis                      |
| `IMU.temp`       | °C     | On-chip temperature                      |

`IMU.accel_mag` is the only IMU field mapped to the fixed-layout
`TelemetryPayload` (HK frame). All IMU tokens are available in `LOG.report`.

---

## Future Peripherals (Planned)

| Peripheral       | Interface | Status        |
|------------------|-----------|---------------|
| Pyro channels    | GPIO      | Pins reserved |
| SD card logger   | SPI       | Planned       |

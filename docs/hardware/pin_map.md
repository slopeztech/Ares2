# ARES Pin Map

**GPIO assignments for ESP32-S3 Zero Mini**

Source: [src/config.h](../../src/config.h)

---

## ESP32-S3 Zero Mini

| GPIO | Function       | Direction | Notes                        |
|------|----------------|-----------|------------------------------|
| 21   | WS2812 LED     | Output    | RGB status LED (Neopixel)    |
| 7    | LoRa UART TX   | Output    | UART2 TX → DX-LR03 RX       |
| 8    | LoRa UART RX   | Input     | UART2 RX ← DX-LR03 TX       |
| 9    | LoRa AUX       | Input     | HIGH = module idle           |
| 1    | I2C0 SDA       | Bidir     | BMP280 (Wire)                |
| 2    | I2C0 SCL       | Output    | BMP280 (Wire, 400 kHz)       |
| 12   | I2C1 SDA       | Bidir     | ADXL375 + MPU-6050 (dedicated IMU bus) |
| 13   | I2C1 SCL       | Output    | ADXL375 + MPU-6050 (50 kHz)            |
| 5    | GPS UART RX    | Input     | UART1                        |
| 6    | GPS UART TX    | Output    | UART1                        |
| 14   | LoRa M0        | Output    | Mode bit 0 (not wired)       |
| 3    | LoRa M1        | Output    | Mode bit 1 (not wired)       |
| 4    | Pulse A        | Output    | Pulse channel A (`PIN_PULSE_A`) |
| 15   | Pulse B        | Output    | Pulse channel B (`PIN_PULSE_B`) |
| 16   | Pulse C/Cont A | Out/In    | Channel C fire **or** cont-sense for A (see below) |
| 17   | Pulse D/Cont B | Out/In    | Channel D fire **or** cont-sense for B (see below) |

I2C assignments:
- **I2C0 (Wire)**: BMP280 on GPIO 1/2
- **I2C1 (TwoWire(1))**: ADXL375 (0x53, primary IMU) and MPU-6050 (0x68, secondary IMU) on GPIO 12/13 — shared bus, distinct addresses

UART assignments:
- **UART0**: USB CDC (Serial monitor)
- **UART1**: GPS (9600 baud)
- **UART2**: LoRa DX-LR03 (9600 baud)

---

## Common Constants

| Parameter               | Value        | Notes                          |
|-------------------------|--------------|--------------------------------|
| Serial baud             | 115200       | USB Serial monitor             |
| I2C frequency           | 400 kHz      | Fast mode                      |
| I2C0 pins (barometer)   | SDA=1, SCL=2 | Wire                           |
| I2C1 pins (IMU)         | SDA=12, SCL=13 | TwoWire(1)                   |
| BMP280 I2C address      | 0x77         | SDO → VCC (combo module)       |
| MPU-6050 I2C address    | 0x68         | AD0 → GND (default)           |
| ADXL375 I2C address     | 0x53         | ALT_ADDRESS/CS → GND (default) |
| GPS baud                | 9600         | Standard NMEA                  |
| LoRa UART baud          | 9600         | DX-LR03 default               |
| Pyro fire duration      | 1000 ms      | E-match ignition pulse         |
| LoRa frequency          | 433.125 MHz (current) / 868 MHz (optional profile) | CH23 (410.125 + 23) for 433 profile |
| LoRa TX power           | 20 dBm       | Configurable up to 30 dBm     |
| WiFi AP password        | `ares1234`   | WPA2 PSK                      |
| WiFi API port           | 80           | HTTP                           |

---

## Pulse channel configuration modes

The four pulse GPIOs (4, 15, 16, 17) can be divided between fire outputs and
continuity-sense inputs by editing `src/config.h` only — no other source files
need to change.

### Mode A — 4-fire / 0-cont (default)

All four GPIOs are configured as push-pull fire outputs.  Continuity is
reported as "intact until fired" (software fallback).

```cpp
// src/config.h
constexpr uint8_t PIN_PULSE_A = 4;   constexpr uint8_t PIN_PULSE_B = 15;
constexpr uint8_t PIN_PULSE_C = 16;  constexpr uint8_t PIN_PULSE_D = 17;
// cont pins all PIN_NO_CONT (default)
```

### Mode B — 2-fire / 2-cont

Channels C and D are disabled as fire outputs (`PIN_NO_FIRE`); their GPIOs are
wired back as INPUT_PULLUP continuity-sense inputs for channels A and B.
Enables `pulse.require_continuity` in AMS scripts (AMS-4.19.4).

```cpp
// src/config.h
constexpr uint8_t PIN_PULSE_A = 4;           constexpr uint8_t PIN_PULSE_B = 15;
constexpr uint8_t PIN_PULSE_C = PIN_NO_FIRE; // GPIO 16 freed
constexpr uint8_t PIN_PULSE_D = PIN_NO_FIRE; // GPIO 17 freed

constexpr uint8_t PIN_PULSE_A_CONT = 16;     // e-match return wire for A
constexpr uint8_t PIN_PULSE_B_CONT = 17;     // e-match return wire for B
```

Hardware note: connect the continuity return wire of e-match A to GPIO 16 (and B
to GPIO 17) through a current-limiting resistor (~1 kΩ).  The pull-up reads HIGH
when the bridgewire is intact, LOW when open circuit.

Any other combination (e.g. 3-fire + 1-cont) works the same way: set unwanted
fire pins to `PIN_NO_FIRE` and assign their GPIO numbers to the desired
`PIN_PULSE_x_CONT` constants.

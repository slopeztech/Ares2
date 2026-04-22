# ARES Pin Map

**GPIO assignments for ESP32-S3 Zero Mini**

Source: [src/config.h](../src/config.h)

---

## ESP32-S3 Zero Mini

| GPIO | Function       | Direction | Notes                        |
|------|----------------|-----------|------------------------------|
| 21   | WS2812 LED     | Output    | RGB status LED (Neopixel)    |
| 7    | LoRa UART TX   | Output    | UART2 TX → DX-LR03 RX       |
| 8    | LoRa UART RX   | Input     | UART2 RX ← DX-LR03 TX       |
| 9    | LoRa AUX       | Input     | HIGH = module idle           |
| 1    | I2C SDA        | Bidir     | BMP280 + MPU-6050            |
| 2    | I2C SCL        | Output    | BMP280 + MPU-6050 (400 kHz)  |
| 5    | GPS UART RX    | Input     | UART1                        |
| 6    | GPS UART TX    | Output    | UART1                        |
| 14   | LoRa M0        | Output    | Mode bit 0 (not wired)       |
| 3    | LoRa M1        | Output    | Mode bit 1 (not wired)       |
| 4    | Pyro DROGUE    | Output    | Drogue parachute channel     |
| 15   | Pyro MAIN      | Output    | Main parachute channel       |

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
| BMP280 I2C address      | 0x77         | SDO → VCC (combo module)       |
| MPU-6050 I2C address    | 0x68         | AD0 → GND (default)           |
| GPS baud                | 9600         | Standard NMEA                  |
| LoRa UART baud          | 9600         | DX-LR03 default               |
| Pyro fire duration      | 1000 ms      | E-match ignition pulse         |
| LoRa frequency          | 433.125 MHz (current) / 868 MHz (optional profile) | CH23 (410.125 + 23) for 433 profile |
| LoRa TX power           | 20 dBm       | Configurable up to 30 dBm     |
| WiFi AP password        | `ares1234`   | WPA2 PSK                      |
| WiFi API port           | 80           | HTTP                           |

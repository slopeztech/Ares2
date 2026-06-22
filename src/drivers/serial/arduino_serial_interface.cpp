#include "drivers/serial/arduino_serial_interface.h"

#include <cstdint>

ArduinoSerialInterface::ArduinoSerialInterface(Print& serial)
    : serial_(serial)
{
}

uint32_t ArduinoSerialInterface::availableForWrite() const
{
    const int available = serial_.availableForWrite();
    if (available <= 0)
    {
        return 0U;
    }
    return static_cast<uint32_t>(available);
}

uint32_t ArduinoSerialInterface::write(const uint8_t* data, uint32_t len)
{
    if (data == nullptr || len == 0U)
    {
        return 0U;
    }
    const size_t written = serial_.write(data, static_cast<size_t>(len));
    if (written > static_cast<size_t>(UINT32_MAX))
    {
        return UINT32_MAX;
    }
    return static_cast<uint32_t>(written);
}
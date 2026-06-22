/**
 * @file  arduino_serial_interface.h
 * @brief Arduino Print adapter for SerialInterface.
 */
#pragma once

#include <Arduino.h>

#include "hal/serial/serial_interface.h"

/**
 * SerialInterface adapter backed by an Arduino Print stream.
 */
class ArduinoSerialInterface : public SerialInterface
{
public:
    explicit ArduinoSerialInterface(Print& serial);

    uint32_t availableForWrite() const override;
    uint32_t write(const uint8_t* data, uint32_t len) override;

private:
    Print& serial_;
};
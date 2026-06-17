/**
 * @file  com_selector.h
 * @brief Runtime COM/radio multiplexer used by API and dispatcher.
 */
#pragma once

#include "hal/driver_selection_interface.h"
#include "hal/radio/radio_interface.h"
#include "sys/hardware/installed_driver_registry.h"

#include <atomic>
#include <cstdint>

class ComSelector final : public RadioInterface
{
public:
    using Entry = ares::hardware::ComDriverEntry;

    ComSelector(const Entry* entries, uint8_t count, uint8_t defaultIndex = 0U);

    ComSelector(const ComSelector&)            = delete;
    ComSelector& operator=(const ComSelector&) = delete;
    ComSelector(ComSelector&&)                 = delete;
    ComSelector& operator=(ComSelector&&)      = delete;

    bool begin() override;
    RadioStatus send(const uint8_t* data, uint16_t len) override;
    RadioStatus receive(uint8_t* buf, uint16_t bufSize, uint16_t& received) override;
    bool ready() const override;
    uint16_t mtu() const override;
    const char* driverModel() const override;

    bool selectDriverByName(const char* model);
    const char* selectedDriverName() const;

private:
    struct SourceList
    {
        const Entry* entries = nullptr;
        uint8_t count = 0U;
    };

    static SourceList buildSourceList(const Entry* entries, uint8_t count);
    static bool safeModelEquals(const char* a, const char* b);
    uint8_t findIndexByName(const char* model) const;

    SourceList sources_{};
    std::atomic<uint8_t> activeIdx_{0U};
};

class ComSelectorControl final : public DriverSelectionInterface
{
public:
    explicit ComSelectorControl(ComSelector& selector) : selector_(selector) {}

    bool selectDriverByName(const char* model) override;
    const char* selectedDriverName() const override;

private:
    ComSelector& selector_;
};

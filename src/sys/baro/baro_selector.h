/**
 * @file  baro_selector.h
 * @brief Runtime barometer multiplexer used by API status endpoints.
 */
#pragma once

#include "hal/baro/barometer_interface.h"
#include "hal/driver_selection_interface.h"
#include "sys/hardware/installed_driver_registry.h"

#include <atomic>
#include <cstdint>

class BaroSelector final : public BarometerInterface
{
public:
    using Entry = ares::hardware::BaroDriverEntry;

    BaroSelector(const Entry* entries, uint8_t count, uint8_t defaultIndex = 0U);

    BaroSelector(const BaroSelector&)            = delete;
    BaroSelector& operator=(const BaroSelector&) = delete;
    BaroSelector(BaroSelector&&)                 = delete;
    BaroSelector& operator=(BaroSelector&&)      = delete;

    bool begin() override;
    BaroStatus read(BaroReading& out) override;
    void setSeaLevelPressure(float hPa) override;
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

class BaroSelectorControl final : public DriverSelectionInterface
{
public:
    explicit BaroSelectorControl(BaroSelector& selector) : selector_(selector) {}

    bool selectDriverByName(const char* model) override;
    const char* selectedDriverName() const override;

private:
    BaroSelector& selector_;
};

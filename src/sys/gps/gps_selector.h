/**
 * @file  gps_selector.h
 * @brief Runtime GPS multiplexer used by API status endpoints.
 */
#pragma once

#include "hal/driver_selection_interface.h"
#include "hal/gps/gps_interface.h"
#include "sys/hardware/installed_driver_registry.h"

#include <atomic>
#include <cstdint>

class GpsSelector final : public GpsInterface
{
public:
    using Entry = ares::hardware::GpsDriverEntry;

    GpsSelector(const Entry* entries, uint8_t count, uint8_t defaultIndex = 0U);

    GpsSelector(const GpsSelector&)            = delete;
    GpsSelector& operator=(const GpsSelector&) = delete;
    GpsSelector(GpsSelector&&)                 = delete;
    GpsSelector& operator=(GpsSelector&&)      = delete;

    bool begin() override;
    void update() override;
    GpsStatus read(GpsReading& out) override;
    bool hasFix() const override;
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

class GpsSelectorControl final : public DriverSelectionInterface
{
public:
    explicit GpsSelectorControl(GpsSelector& selector) : selector_(selector) {}

    bool selectDriverByName(const char* model) override;
    const char* selectedDriverName() const override;

private:
    GpsSelector& selector_;
};

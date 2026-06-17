/**
 * @file  imu_selector.h
 * @brief Runtime IMU multiplexer used by API status endpoints.
 */
#pragma once

#include "hal/imu/imu_interface.h"
#include "hal/imu/imu_selection_interface.h"
#include "sys/hardware/installed_driver_registry.h"

#include <atomic>
#include <cstdint>

/**
 * @brief Exposes one active IMU from a fixed list of physical drivers.
 *
 * read() prefers the selected driver and falls back to other drivers when
 * the preferred one is not ready or returns an error. On fallback success,
 * the selected driver is switched to the working one.
 */
class ImuSelector final : public ImuInterface
{
public:
    using Entry = ares::hardware::ImuDriverEntry;

    ImuSelector(const Entry* entries, uint8_t count, uint8_t defaultIndex = 0U);

    // Non-copyable, non-movable.
    ImuSelector(const ImuSelector&)            = delete;
    ImuSelector& operator=(const ImuSelector&) = delete;
    ImuSelector(ImuSelector&&)                 = delete;
    ImuSelector& operator=(ImuSelector&&)      = delete;

    bool begin() override;
    ImuStatus read(ImuReading& out) override;
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

/**
 * @brief Adapter exposing ImuSelector switching through ImuSelectionInterface.
 */
class ImuSelectorControl final : public ImuSelectionInterface
{
public:
    explicit ImuSelectorControl(ImuSelector& selector) : selector_(selector) {}

    bool selectDriverByName(const char* model) override;
    const char* selectedDriverName() const override;

private:
    ImuSelector& selector_;
};

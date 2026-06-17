/**
 * @file  installed_driver_registry.h
 * @brief Single source of truth for hardware drivers installed on this board.
 */
#pragma once

#include "hal/baro/barometer_interface.h"
#include "hal/gps/gps_interface.h"
#include "hal/imu/imu_interface.h"
#include "hal/radio/radio_interface.h"

#include <cstdint>

namespace ares
{
namespace hardware
{

enum class DriverKind : uint8_t
{
    GPS,
    BARO,
    COM,
    IMU
};

template<typename InterfaceT>
struct DriverEntry
{
    const char* model = nullptr;
    InterfaceT* iface = nullptr;
};

template<typename EntryT>
struct DriverList
{
    EntryT* entries = nullptr;
    uint8_t count   = 0U;
};

using GpsDriverEntry  = DriverEntry<GpsInterface>;
using BaroDriverEntry = DriverEntry<BarometerInterface>;
using ComDriverEntry  = DriverEntry<RadioInterface>;
using ImuDriverEntry  = DriverEntry<ImuInterface>;

struct InstalledHardwareRefs
{
    BarometerInterface* barometer = nullptr;
    GpsInterface*       gps       = nullptr;
    RadioInterface*     radio     = nullptr;
    ImuInterface*       imuPrimary = nullptr;
    ImuInterface*       imuSecondary = nullptr;
};

void bindInstalledHardware(const InstalledHardwareRefs& refs);

const DriverList<GpsDriverEntry>& gpsDrivers();
const DriverList<BaroDriverEntry>& baroDrivers();
const DriverList<ComDriverEntry>& comDrivers();
const DriverList<ImuDriverEntry>& imuDrivers();

const char* defaultGpsDriverModel();
const char* defaultBaroDriverModel();
const char* defaultComDriverModel();
const char* defaultImuDriverModel();
bool isSupportedModel(DriverKind kind, const char* model);

} // namespace hardware
} // namespace ares
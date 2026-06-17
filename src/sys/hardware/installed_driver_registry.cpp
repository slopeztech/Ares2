/**
 * @file  installed_driver_registry.cpp
 * @brief Central registry of hardware drivers installed on this board.
 */

#include "sys/hardware/installed_driver_registry.h"

#include <cstring>

namespace
{
using namespace ares::hardware;

GpsDriverEntry g_gpsEntries[] = {
    { "BN220", nullptr }
};

BaroDriverEntry g_baroEntries[] = {
    { "BMP280", nullptr }
};

ComDriverEntry g_comEntries[] = {
    { "DXLR03", nullptr }
};

ImuDriverEntry g_imuEntries[] = {
    { "MPU6050", nullptr },
    { "ADXL375", nullptr }
};

DriverList<GpsDriverEntry> g_gpsList = {
    g_gpsEntries,
    static_cast<uint8_t>(sizeof(g_gpsEntries) / sizeof(g_gpsEntries[0]))
};

DriverList<BaroDriverEntry> g_baroList = {
    g_baroEntries,
    static_cast<uint8_t>(sizeof(g_baroEntries) / sizeof(g_baroEntries[0]))
};

DriverList<ComDriverEntry> g_comList = {
    g_comEntries,
    static_cast<uint8_t>(sizeof(g_comEntries) / sizeof(g_comEntries[0]))
};

DriverList<ImuDriverEntry> g_imuList = {
    g_imuEntries,
    static_cast<uint8_t>(sizeof(g_imuEntries) / sizeof(g_imuEntries[0]))
};

bool safeModelEquals(const char* a, const char* b)
{
    if (a == nullptr || b == nullptr)
    {
        return false;
    }
    return std::strcmp(a, b) == 0;
}

template<typename EntryT>
bool listContainsModel(const DriverList<EntryT>& drivers, const char* model)
{
    for (uint8_t i = 0U; i < drivers.count; i++)
    {
        if (safeModelEquals(drivers.entries[i].model, model))
        {
            return true;
        }
    }
    return false;
}

} // namespace

namespace ares
{
namespace hardware
{

void bindInstalledHardware(const InstalledHardwareRefs& refs)
{
    g_baroEntries[0].iface = refs.barometer;
    g_gpsEntries[0].iface = refs.gps;
    g_comEntries[0].iface = refs.radio;
    g_imuEntries[0].iface = refs.imuPrimary;
    g_imuEntries[1].iface = refs.imuSecondary;
}

const DriverList<GpsDriverEntry>& gpsDrivers()
{
    return g_gpsList;
}

const DriverList<BaroDriverEntry>& baroDrivers()
{
    return g_baroList;
}

const DriverList<ComDriverEntry>& comDrivers()
{
    return g_comList;
}

const DriverList<ImuDriverEntry>& imuDrivers()
{
    return g_imuList;
}

const char* defaultGpsDriverModel()
{
    return (g_gpsList.count > 0U) ? g_gpsList.entries[0].model : "NONE";
}

const char* defaultBaroDriverModel()
{
    return (g_baroList.count > 0U) ? g_baroList.entries[0].model : "NONE";
}

const char* defaultComDriverModel()
{
    return (g_comList.count > 0U) ? g_comList.entries[0].model : "NONE";
}

const char* defaultImuDriverModel()
{
    return (g_imuList.count > 0U) ? g_imuList.entries[0].model : "NONE";
}

bool isSupportedModel(DriverKind kind, const char* model)
{
    switch (kind)
    {
        case DriverKind::GPS:
            return listContainsModel(g_gpsList, model);
        case DriverKind::BARO:
            return listContainsModel(g_baroList, model);
        case DriverKind::COM:
            return listContainsModel(g_comList, model);
        case DriverKind::IMU:
            return listContainsModel(g_imuList, model);
        default:
            return false;
    }
}

} // namespace hardware
} // namespace ares
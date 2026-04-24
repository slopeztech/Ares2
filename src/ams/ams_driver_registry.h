/**
 * @file  ams_driver_registry.h
 * @brief Compiled-in driver registry for the AMS engine.
 *
 * main.cpp creates static arrays of GpsEntry / BaroEntry / ComEntry /
 * ImuEntry and passes them to MissionScriptEngine.  Scripts then select
 * a specific driver by model name using the 'include' directive:
 *
 *   include BN220   as GPS
 *   include BMP280  as BARO
 *   include LORA    as COM
 *   include MPU6050 as IMU
 *
 * Multiple drivers of the same kind can be registered for multi-instance
 * use (e.g. include BN220 as GPS1 / include BN880 as GPS2).
 *
 * No dynamic allocation — all arrays are statically declared in main.cpp.
 */
#pragma once

#include "hal/baro/barometer_interface.h"
#include "hal/gps/gps_interface.h"
#include "hal/imu/imu_interface.h"
#include "hal/radio/radio_interface.h"

namespace ares
{
namespace ams
{

/** One compiled-in GPS driver entry. */
struct GpsEntry  { const char* model; GpsInterface*       iface; };

/** One compiled-in barometer driver entry. */
struct BaroEntry { const char* model; BarometerInterface* iface; };

/** One compiled-in COM/radio driver entry. */
struct ComEntry  { const char* model; RadioInterface*     iface; };

/** One compiled-in IMU driver entry (iface may be null if hardware not wired). */
struct ImuEntry  { const char* model; ImuInterface*       iface; };

} // namespace ams
} // namespace ares

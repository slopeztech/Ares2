/**
 * @file  ams_driver_registry.h
 * @brief Installed-driver types consumed by the AMS engine.
 *
 * The board-level installed driver registry owns the concrete arrays and
 * passes them to MissionScriptEngine. Scripts then select a specific driver
 * by model name using the 'include' directive:
 *
 *   include BN220   as GPS
 *   include BMP280  as BARO
 *   include DXLR03  as COM
 *   include MPU6050 as IMU
 *
 * SERIAL.report output uses a SerialInterface sink wired by platform bootstrap
 * (not by script include directives), so AMS remains transport-agnostic.
 *
 * Multiple drivers of the same kind can be registered for multi-instance
 * use (e.g. include BN220 as GPS1 / include BN880 as GPS2).
 *
 * No dynamic allocation — all arrays are statically declared in main.cpp.
 */
#pragma once

#include "sys/hardware/installed_driver_registry.h"

namespace ares
{
namespace ams
{

using GpsEntry  = ares::hardware::GpsDriverEntry;
using BaroEntry = ares::hardware::BaroDriverEntry;
using ComEntry  = ares::hardware::ComDriverEntry;
using ImuEntry  = ares::hardware::ImuDriverEntry;
using SerialEntry = ares::hardware::SerialDriverEntry;

} // namespace ams
} // namespace ares

/**
 * @file  imu_selection_interface.h
 * @brief Optional runtime selector for choosing which IMU backs API reads.
 */
#pragma once

#include "hal/driver_selection_interface.h"

/**
 * @brief Runtime IMU-selection hook used by API/device config.
 *
 * Implementations can map a model string (e.g. "MPU6050", "ADXL375")
 * to the active IMU exposed by status endpoints.
 */
class ImuSelectionInterface : public DriverSelectionInterface
{
public:
    ~ImuSelectionInterface() override = default;
};

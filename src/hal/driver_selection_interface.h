/**
 * @file  driver_selection_interface.h
 * @brief Generic runtime driver-selection hook used by API/device config.
 */
#pragma once

class DriverSelectionInterface
{
public:
    virtual ~DriverSelectionInterface() = default;

    /**
     * @brief Select the active driver by model name.
     * @param[in] model  Null-terminated model identifier.
     * @return true if the model is supported and was selected.
     */
    virtual bool selectDriverByName(const char* model) = 0;

    /**
     * @return Model name of the currently selected driver (never nullptr).
     */
    virtual const char* selectedDriverName() const = 0;
};

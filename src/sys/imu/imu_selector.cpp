/**
 * @file  imu_selector.cpp
 * @brief Runtime IMU multiplexer implementation.
 */

#include "sys/imu/imu_selector.h"

#include <cstring>

ImuSelector::ImuSelector(const Entry* entries, uint8_t count, uint8_t defaultIndex)
    : sources_(buildSourceList(entries, count))
{
    if (sources_.count == 0U || sources_.entries == nullptr)
    {
        sources_.count = 0U;
        return;
    }

    const uint8_t idx = (defaultIndex < sources_.count) ? defaultIndex : 0U;
    activeIdx_.store(idx);
}

ImuSelector::SourceList ImuSelector::buildSourceList(const Entry* entries, uint8_t count)
{
    SourceList sources;
    sources.entries = entries;
    sources.count = count;
    return sources;
}

bool ImuSelector::begin()
{
    if (sources_.count == 0U)
    {
        return false;
    }

    const uint8_t idx = activeIdx_.load();
    ImuInterface* const iface = sources_.entries[idx].iface;
    return (iface != nullptr) ? iface->begin() : false;
}

ImuStatus ImuSelector::read(ImuReading& out)
{
    if (sources_.count == 0U)
    {
        return ImuStatus::NOT_READY;
    }

    const uint8_t preferredIdx = activeIdx_.load();
    ImuInterface* const preferred = sources_.entries[preferredIdx].iface;
    if (preferred != nullptr)
    {
        const ImuStatus st = preferred->read(out);
        if (st == ImuStatus::OK)
        {
            return st;
        }
    }

    for (uint8_t i = 0U; i < sources_.count; i++)
    {
        if (i == preferredIdx)
        {
            continue;
        }

        ImuInterface* const iface = sources_.entries[i].iface;
        if (iface == nullptr)
        {
            continue;
        }

        const ImuStatus st = iface->read(out);
        if (st == ImuStatus::OK)
        {
            activeIdx_.store(i);
            return st;
        }
    }

    return ImuStatus::NOT_READY;
}

const char* ImuSelector::driverModel() const
{
    if (sources_.count == 0U)
    {
        return "NONE";
    }

    const uint8_t idx = activeIdx_.load();
    const ImuInterface* const iface = sources_.entries[idx].iface;
    if (iface == nullptr)
    {
        return "NONE";
    }
    return iface->driverModel();
}

bool ImuSelector::selectDriverByName(const char* model)
{
    const uint8_t idx = findIndexByName(model);
    if (idx >= sources_.count)
    {
        return false;
    }

    activeIdx_.store(idx);
    return true;
}

const char* ImuSelector::selectedDriverName() const
{
    if (sources_.count == 0U)
    {
        return "NONE";
    }

    const uint8_t idx = activeIdx_.load();
    const char* const model = sources_.entries[idx].model;
    return (model != nullptr) ? model : "NONE";
}

bool ImuSelector::safeModelEquals(const char* a, const char* b)
{
    if (a == nullptr || b == nullptr)
    {
        return false;
    }
    return std::strcmp(a, b) == 0;
}

uint8_t ImuSelector::findIndexByName(const char* model) const
{
    if (sources_.count == 0U || model == nullptr)
    {
        return sources_.count;
    }

    for (uint8_t i = 0U; i < sources_.count; i++)
    {
        if (safeModelEquals(sources_.entries[i].model, model))
        {
            return i;
        }
    }

    return sources_.count;
}

bool ImuSelectorControl::selectDriverByName(const char* model)
{
    return selector_.selectDriverByName(model);
}

const char* ImuSelectorControl::selectedDriverName() const
{
    return selector_.selectedDriverName();
}

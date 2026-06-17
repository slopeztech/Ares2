/**
 * @file  baro_selector.cpp
 * @brief Runtime barometer multiplexer implementation.
 */

#include "sys/baro/baro_selector.h"

#include <cstring>

BaroSelector::BaroSelector(const Entry* entries, uint8_t count, uint8_t defaultIndex)
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

BaroSelector::SourceList BaroSelector::buildSourceList(const Entry* entries, uint8_t count)
{
    SourceList sources;
    sources.entries = entries;
    sources.count = count;
    return sources;
}

bool BaroSelector::begin()
{
    if (sources_.count == 0U)
    {
        return false;
    }

    const uint8_t idx = activeIdx_.load();
    BarometerInterface* const iface = sources_.entries[idx].iface;
    return (iface != nullptr) ? iface->begin() : false;
}

BaroStatus BaroSelector::read(BaroReading& out)
{
    if (sources_.count == 0U)
    {
        return BaroStatus::NOT_READY;
    }

    const uint8_t preferredIdx = activeIdx_.load();
    BarometerInterface* const preferred = sources_.entries[preferredIdx].iface;
    if (preferred != nullptr)
    {
        const BaroStatus st = preferred->read(out);
        if (st == BaroStatus::OK)
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

        BarometerInterface* const iface = sources_.entries[i].iface;
        if (iface == nullptr)
        {
            continue;
        }

        const BaroStatus st = iface->read(out);
        if (st == BaroStatus::OK)
        {
            activeIdx_.store(i);
            return st;
        }
    }

    return BaroStatus::NOT_READY;
}

void BaroSelector::setSeaLevelPressure(float hPa)
{
    for (uint8_t i = 0U; i < sources_.count; i++)
    {
        BarometerInterface* const iface = sources_.entries[i].iface;
        if (iface != nullptr)
        {
            iface->setSeaLevelPressure(hPa);
        }
    }
}

const char* BaroSelector::driverModel() const
{
    if (sources_.count == 0U)
    {
        return "NONE";
    }

    const uint8_t idx = activeIdx_.load();
    const BarometerInterface* const iface = sources_.entries[idx].iface;
    if (iface == nullptr)
    {
        return "NONE";
    }
    return iface->driverModel();
}

bool BaroSelector::selectDriverByName(const char* model)
{
    const uint8_t idx = findIndexByName(model);
    if (idx >= sources_.count)
    {
        return false;
    }

    activeIdx_.store(idx);
    return true;
}

const char* BaroSelector::selectedDriverName() const
{
    if (sources_.count == 0U)
    {
        return "NONE";
    }

    const uint8_t idx = activeIdx_.load();
    const char* const model = sources_.entries[idx].model;
    return (model != nullptr) ? model : "NONE";
}

bool BaroSelector::safeModelEquals(const char* a, const char* b)
{
    if (a == nullptr || b == nullptr)
    {
        return false;
    }
    return std::strcmp(a, b) == 0;
}

uint8_t BaroSelector::findIndexByName(const char* model) const
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

bool BaroSelectorControl::selectDriverByName(const char* model)
{
    return selector_.selectDriverByName(model);
}

const char* BaroSelectorControl::selectedDriverName() const
{
    return selector_.selectedDriverName();
}

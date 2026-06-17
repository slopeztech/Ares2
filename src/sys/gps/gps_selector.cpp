/**
 * @file  gps_selector.cpp
 * @brief Runtime GPS multiplexer implementation.
 */

#include "sys/gps/gps_selector.h"

#include <cstring>

GpsSelector::GpsSelector(const Entry* entries, uint8_t count, uint8_t defaultIndex)
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

GpsSelector::SourceList GpsSelector::buildSourceList(const Entry* entries, uint8_t count)
{
    SourceList sources;
    sources.entries = entries;
    sources.count = count;
    return sources;
}

bool GpsSelector::begin()
{
    if (sources_.count == 0U)
    {
        return false;
    }

    const uint8_t idx = activeIdx_.load();
    GpsInterface* const iface = sources_.entries[idx].iface;
    return (iface != nullptr) ? iface->begin() : false;
}

void GpsSelector::update()
{
    for (uint8_t i = 0U; i < sources_.count; i++)
    {
        GpsInterface* const iface = sources_.entries[i].iface;
        if (iface != nullptr)
        {
            iface->update();
        }
    }
}

GpsStatus GpsSelector::read(GpsReading& out)
{
    if (sources_.count == 0U)
    {
        return GpsStatus::NOT_READY;
    }

    const uint8_t preferredIdx = activeIdx_.load();
    GpsInterface* const preferred = sources_.entries[preferredIdx].iface;
    if (preferred != nullptr)
    {
        const GpsStatus st = preferred->read(out);
        if (st == GpsStatus::OK)
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

        GpsInterface* const iface = sources_.entries[i].iface;
        if (iface == nullptr)
        {
            continue;
        }

        const GpsStatus st = iface->read(out);
        if (st == GpsStatus::OK)
        {
            activeIdx_.store(i);
            return st;
        }
    }

    return GpsStatus::NOT_READY;
}

bool GpsSelector::hasFix() const
{
    if (sources_.count == 0U)
    {
        return false;
    }

    const uint8_t idx = activeIdx_.load();
    const GpsInterface* const iface = sources_.entries[idx].iface;
    return (iface != nullptr) ? iface->hasFix() : false;
}

const char* GpsSelector::driverModel() const
{
    if (sources_.count == 0U)
    {
        return "NONE";
    }

    const uint8_t idx = activeIdx_.load();
    const GpsInterface* const iface = sources_.entries[idx].iface;
    if (iface == nullptr)
    {
        return "NONE";
    }
    return iface->driverModel();
}

bool GpsSelector::selectDriverByName(const char* model)
{
    const uint8_t idx = findIndexByName(model);
    if (idx >= sources_.count)
    {
        return false;
    }

    activeIdx_.store(idx);
    return true;
}

const char* GpsSelector::selectedDriverName() const
{
    if (sources_.count == 0U)
    {
        return "NONE";
    }

    const uint8_t idx = activeIdx_.load();
    const char* const model = sources_.entries[idx].model;
    return (model != nullptr) ? model : "NONE";
}

bool GpsSelector::safeModelEquals(const char* a, const char* b)
{
    if (a == nullptr || b == nullptr)
    {
        return false;
    }
    return std::strcmp(a, b) == 0;
}

uint8_t GpsSelector::findIndexByName(const char* model) const
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

bool GpsSelectorControl::selectDriverByName(const char* model)
{
    return selector_.selectDriverByName(model);
}

const char* GpsSelectorControl::selectedDriverName() const
{
    return selector_.selectedDriverName();
}

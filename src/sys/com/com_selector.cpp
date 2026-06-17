/**
 * @file  com_selector.cpp
 * @brief Runtime COM/radio multiplexer implementation.
 */

#include "sys/com/com_selector.h"

#include <cstring>

ComSelector::ComSelector(const Entry* entries, uint8_t count, uint8_t defaultIndex)
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

ComSelector::SourceList ComSelector::buildSourceList(const Entry* entries, uint8_t count)
{
    SourceList sources;
    sources.entries = entries;
    sources.count = count;
    return sources;
}

bool ComSelector::begin()
{
    if (sources_.count == 0U)
    {
        return false;
    }

    const uint8_t idx = activeIdx_.load();
    RadioInterface* const iface = sources_.entries[idx].iface;
    return (iface != nullptr) ? iface->begin() : false;
}

RadioStatus ComSelector::send(const uint8_t* data, uint16_t len)
{
    if (sources_.count == 0U)
    {
        return RadioStatus::NOT_READY;
    }

    const uint8_t idx = activeIdx_.load();
    RadioInterface* const iface = sources_.entries[idx].iface;
    return (iface != nullptr) ? iface->send(data, len) : RadioStatus::NOT_READY;
}

RadioStatus ComSelector::receive(uint8_t* buf, uint16_t bufSize, uint16_t& received)
{
    if (sources_.count == 0U)
    {
        received = 0U;
        return RadioStatus::NOT_READY;
    }

    const uint8_t idx = activeIdx_.load();
    RadioInterface* const iface = sources_.entries[idx].iface;
    if (iface == nullptr)
    {
        received = 0U;
        return RadioStatus::NOT_READY;
    }
    return iface->receive(buf, bufSize, received);
}

bool ComSelector::ready() const
{
    if (sources_.count == 0U)
    {
        return false;
    }

    const uint8_t idx = activeIdx_.load();
    const RadioInterface* const iface = sources_.entries[idx].iface;
    return (iface != nullptr) ? iface->ready() : false;
}

uint16_t ComSelector::mtu() const
{
    if (sources_.count == 0U)
    {
        return 0U;
    }

    const uint8_t idx = activeIdx_.load();
    const RadioInterface* const iface = sources_.entries[idx].iface;
    return (iface != nullptr) ? iface->mtu() : 0U;
}

const char* ComSelector::driverModel() const
{
    if (sources_.count == 0U)
    {
        return "NONE";
    }

    const uint8_t idx = activeIdx_.load();
    const RadioInterface* const iface = sources_.entries[idx].iface;
    if (iface == nullptr)
    {
        return "NONE";
    }
    return iface->driverModel();
}

bool ComSelector::selectDriverByName(const char* model)
{
    const uint8_t idx = findIndexByName(model);
    if (idx >= sources_.count)
    {
        return false;
    }

    activeIdx_.store(idx);
    return true;
}

const char* ComSelector::selectedDriverName() const
{
    if (sources_.count == 0U)
    {
        return "NONE";
    }

    const uint8_t idx = activeIdx_.load();
    const char* const model = sources_.entries[idx].model;
    return (model != nullptr) ? model : "NONE";
}

bool ComSelector::safeModelEquals(const char* a, const char* b)
{
    if (a == nullptr || b == nullptr)
    {
        return false;
    }
    return std::strcmp(a, b) == 0;
}

uint8_t ComSelector::findIndexByName(const char* model) const
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

bool ComSelectorControl::selectDriverByName(const char* model)
{
    return selector_.selectDriverByName(model);
}

const char* ComSelectorControl::selectedDriverName() const
{
    return selector_.selectedDriverName();
}

/**
 * @file  sim_radio_driver.h
 * @brief Simulation radio driver — captures outgoing frames (RadioInterface).
 *
 * SimRadioDriver silently discards all transmitted frames but keeps a copy
 * of the last payload in a fixed-size buffer so tests can inspect what the
 * AMS engine actually sent.  Declare in AMS scripts as:
 *
 *   include SIM_COM as COM
 *
 * Thread safety: NOT thread-safe.  Access from a single task only (CERT-13).
 */
#pragma once

#include "hal/radio/radio_interface.h"
#include <cstdint>
#include <cstring>

namespace ares
{
namespace sim
{

/// Maximum capture buffer for a single transmitted frame (bytes).
static constexpr uint16_t SIM_RADIO_CAPTURE_SIZE = 512U;

/**
 * Radio simulator driver.
 *
 * Captures the last transmitted frame so tests can verify the AMS engine
 * is emitting the correct telemetry payloads without a real radio link.
 */
class SimRadioDriver final : public RadioInterface
{
public:
    SimRadioDriver()
        : ready_(false), sendCount_(0U), lastLen_(0U)
    {
        (void)memset(lastFrame_, 0, sizeof(lastFrame_));
    }

    // ── RadioInterface ────────────────────────────────────────────────────────

    bool begin() override
    {
        ready_ = true;
        return true;
    }

    RadioStatus send(const uint8_t* data, uint16_t len) override
    {
        if (!ready_)        { return RadioStatus::NOT_READY; }
        if (data == nullptr){ return RadioStatus::ERROR; }

        // Capture the last frame (truncate if larger than the capture buffer).
        const uint16_t copyLen = (len <= SIM_RADIO_CAPTURE_SIZE)
                                  ? len
                                  : SIM_RADIO_CAPTURE_SIZE;
        (void)memcpy(lastFrame_, data, copyLen);
        lastLen_ = copyLen;
        sendCount_++;

        return RadioStatus::OK;
    }

    RadioStatus receive(uint8_t* /*buf*/, uint16_t /*bufSize*/,
                        uint16_t& received) override
    {
        received = 0U;
        return RadioStatus::OK;   // No inbound traffic in the sim.
    }

    bool ready() const override
    {
        return ready_;
    }

    uint16_t mtu() const override
    {
        return SIM_RADIO_CAPTURE_SIZE;
    }

    const char* driverModel() const override
    {
        return "SIM_COM";
    }

    // ── Sim-only inspection API ───────────────────────────────────────────────

    /** Return total number of frames transmitted since begin(). */
    uint32_t sendCount() const { return sendCount_; }

    /** Pointer to the last captured frame (valid for lastFrameLen() bytes). */
    const uint8_t* lastFrame() const { return lastFrame_; }

    /** Length of the last captured frame in bytes. */
    uint16_t lastFrameLen() const { return lastLen_; }

    /** Reset captured state without re-running begin(). */
    void resetCapture()
    {
        sendCount_ = 0U;
        lastLen_   = 0U;
        (void)memset(lastFrame_, 0, sizeof(lastFrame_));
    }

private:
    bool     ready_;
    uint32_t sendCount_;
    uint16_t lastLen_;
    uint8_t  lastFrame_[SIM_RADIO_CAPTURE_SIZE];
};

} // namespace sim
} // namespace ares

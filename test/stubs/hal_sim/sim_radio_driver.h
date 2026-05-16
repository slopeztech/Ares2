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
#include "ares_assert.h"
#include <freertos/task.h>
#include <cstdint>
#include <cstring>

namespace ares
{
namespace sim
{

/// Maximum capture buffer for a single transmitted frame (bytes).
static constexpr uint16_t SIM_RADIO_CAPTURE_SIZE = 512U;

/// Maximum bytes that can be queued for injection via injectBytes() (bytes).
static constexpr uint16_t SIM_RADIO_INJECT_SIZE  = 1024U;

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
        : ready_(false), sendCount_(0U), lastLen_(0U), rxInjectLen_(0U),
          ownerTask_(nullptr)
    {
        (void)memset(lastFrame_,    0, sizeof(lastFrame_));
        (void)memset(rxInjectBuf_,  0, sizeof(rxInjectBuf_));
    }

    // ── RadioInterface ────────────────────────────────────────────────────────

    bool begin() override
    {
        ownerTask_ = xTaskGetCurrentTaskHandle();
        ready_ = true;
        return true;
    }

    RadioStatus send(const uint8_t* data, uint16_t len) override
    {
        ARES_ASSERT(ownerTask_ != nullptr);
        ARES_ASSERT(xTaskGetCurrentTaskHandle() == ownerTask_);
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

    RadioStatus receive(uint8_t* buf, uint16_t bufSize,
                        uint16_t& received) override
    {
        ARES_ASSERT(ownerTask_ != nullptr);
        ARES_ASSERT(xTaskGetCurrentTaskHandle() == ownerTask_);
        received = 0U;
        if (!ready_) { return RadioStatus::NOT_READY; }
        if (rxInjectLen_ == 0U) { return RadioStatus::OK; }

        // Drain as many queued bytes as fit in the caller's buffer.
        received = (rxInjectLen_ <= bufSize) ? rxInjectLen_ : bufSize;
        (void)memcpy(buf, rxInjectBuf_, received);
        rxInjectLen_ = static_cast<uint16_t>(rxInjectLen_ - received);
        if (rxInjectLen_ > 0U)
        {
            (void)memmove(rxInjectBuf_, &rxInjectBuf_[received], rxInjectLen_);
        }
        return RadioStatus::OK;
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
        sendCount_   = 0U;
        lastLen_     = 0U;
        rxInjectLen_ = 0U;
        (void)memset(lastFrame_,   0, sizeof(lastFrame_));
        (void)memset(rxInjectBuf_, 0, sizeof(rxInjectBuf_));
    }

    /**
     * Queue raw bytes to be returned by the next receive() call(s).
     *
     * The bytes are appended to the internal injection ring buffer.
     * Subsequent calls to receive() drain them FIFO.
     *
     * @param[in] data  Bytes to inject (must not be nullptr).
     * @param[in] len   Number of bytes.
     * @return true on success; false if the injection buffer would overflow.
     */
    bool injectBytes(const uint8_t* data, uint16_t len)
    {
        if (data == nullptr) { return false; }
        if (len > static_cast<uint16_t>(sizeof(rxInjectBuf_) - rxInjectLen_))
        {
            return false;  // Not enough space.
        }
        (void)memcpy(&rxInjectBuf_[rxInjectLen_], data, len);
        rxInjectLen_ = static_cast<uint16_t>(rxInjectLen_ + len);
        return true;
    }

private:
    bool          ready_;
    uint32_t      sendCount_;
    uint16_t      lastLen_;
    uint8_t       lastFrame_[SIM_RADIO_CAPTURE_SIZE];
    uint8_t       rxInjectBuf_[SIM_RADIO_INJECT_SIZE];  ///< Bytes queued for receive().
    uint16_t      rxInjectLen_;                          ///< Valid bytes in rxInjectBuf_.
    TaskHandle_t  ownerTask_;  ///< Task that called begin(); asserted in I/O methods (CERT-13).
};

} // namespace sim
} // namespace ares

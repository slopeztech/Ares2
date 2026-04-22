/**
 * @file  wifi_ap.h
 * @brief WiFi soft-AP service for ESP32-S3.
 *
 * Configures the ESP32 as a WiFi Access Point for ground
 * configuration and monitoring.  Uses parameters from
 * config.h (SSID derived from MAC, password, channel, etc.).
 *
 * Thread safety: begin()/end() must be called from setup().
 *                isReady() is safe from any task (atomic).
 *                clientCount() reads ESP-IDF internal state.
 */
#pragma once

#include <cstdint>
#include <atomic>

/**
 * WiFi Access Point service.
 *
 * Manages the ESP32-S3 soft-AP lifecycle.  The SSID is
 * auto-generated as "ARES-XXXX" where XXXX are the last
 * two bytes of the WiFi MAC address (hex).
 *
 * No dynamic memory — all state is in-object (PO10-3).
 */
class WifiAp
{
public:
    WifiAp() = default;

    // Non-copyable, non-movable (CERT-18.3)
    WifiAp(const WifiAp&)            = delete;
    WifiAp& operator=(const WifiAp&) = delete;
    WifiAp(WifiAp&&)                 = delete;
    WifiAp& operator=(WifiAp&&)      = delete;

    /**
     * Initialise and start the WiFi soft-AP.
     * @return true on success, false if WiFi init failed.
     * @pre  Called once from setup() before any API usage.
     * @post AP is broadcasting and accepting client connections.
     */
    bool begin();

    /**
     * Stop the WiFi soft-AP and release resources.
     * @post AP is no longer broadcasting.
     */
    void end();

    /**
     * @return true if the AP is initialised and running.
     */
    bool isReady() const;

    /**
     * @return Number of currently connected WiFi stations.
     */
    uint8_t clientCount() const;

    /**
     * @return Pointer to the generated SSID string (null-terminated).
     * @pre begin() has been called.
     */
    const char* ssid() const;

private:
    static constexpr uint8_t SSID_MAX_LEN = 16;  ///< "ARES-XXXX" + null.

    char ssid_[SSID_MAX_LEN] = {};                ///< Generated SSID.
    std::atomic<bool> ready_{false};               ///< AP running flag.
};

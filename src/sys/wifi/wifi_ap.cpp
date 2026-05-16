/**
 * @file  wifi_ap.cpp
 * @brief WiFi soft-AP service implementation.
 *
 * Uses the Arduino WiFi library (ESP-IDF wrapper) to
 * configure soft-AP mode with WPA2-PSK authentication.
 * TX power is set to WIFI_TX_POWER_DBM from config.h
 * to conserve battery on the pad.
 */

#include "sys/wifi/wifi_ap.h"
#include "config.h"
#include "ares_assert.h"
#include "debug/ares_log.h"

#include <WiFi.h>
#include <cstdio>

// ── Log tag ─────────────────────────────────────────────────
static constexpr const char* TAG = "WIFI";

// ── Public API ──────────────────────────────────────────────

bool WifiAp::begin(const DeviceConfig& cfg)
{
    ARES_ASSERT(!ready_.load());  // Double-init guard (CERT-18.3)

    // Generate SSID from the last 2 bytes of the WiFi MAC
    uint8_t mac[6] = {};
    WiFi.macAddress(mac);
    snprintf(ssid_, sizeof(ssid_), "ARES-%02X%02X",
             mac[4], mac[5]);

    // Configure soft-AP — password from device config (may be the
    // factory default "ares1234" if no config file has been deployed).
    WiFi.mode(WIFI_AP);

    const bool ok = WiFi.softAP(
        ssid_,
        cfg.wifiPassword(),
        ares::WIFI_AP_CHANNEL,
        0,                          // SSID not hidden
        ares::WIFI_AP_MAX_CLIENTS);

    if (!ok)
    {
        LOG_E(TAG, "softAP failed");
        return false;
    }

    // Set TX power — ESP-IDF uses 0.25 dBm units
    // e.g. 8 dBm → 32 quarter-dBm
    static_assert(ares::WIFI_TX_POWER_DBM * 4 <= 127,
                  "TX power * 4 must fit int8_t (CERT-4)");
    const int8_t quarterDbm = static_cast<int8_t>(
        ares::WIFI_TX_POWER_DBM * 4);
    WiFi.setTxPower(static_cast<wifi_power_t>(quarterDbm));

    ready_.store(true);

    LOG_I(TAG, "AP started: SSID=%s CH=%u PWR=%d dBm",
          ssid_,
          static_cast<uint32_t>(ares::WIFI_AP_CHANNEL),
          static_cast<int32_t>(ares::WIFI_TX_POWER_DBM));
    LOG_I(TAG, "IP=%s port=%u max_clients=%u",
          WiFi.softAPIP().toString().c_str(),
          static_cast<uint32_t>(ares::WIFI_API_PORT),
          static_cast<uint32_t>(ares::WIFI_AP_MAX_CLIENTS));

    return true;
}

void WifiAp::end()
{
    if (ready_.load())
    {
        WiFi.softAPdisconnect(true);
        WiFi.mode(WIFI_OFF);
        ready_.store(false);
        LOG_I(TAG, "AP stopped");
    }
}

bool WifiAp::isReady() const
{
    return ready_.load();
}

uint8_t WifiAp::clientCount() const
{
    if (!ready_.load())
    {
        return 0;
    }
    return static_cast<uint8_t>(WiFi.softAPgetStationNum());
}

const char* WifiAp::ssid() const
{
    return ssid_;
}

/**
 * @file  link_stubs.cpp
 * @brief Linker stubs for api_routing test environment.
 *
 * Provides no-op implementations of symbols referenced by the compiled
 * API handler objects but never actually reached in the routing tests:
 *   - StatusLed::setMode        (called in api_server.cpp::run() only)
 *   - WifiAp::isReady / clientCount  (called in status_handler)
 *   - MissionScriptEngine methods (called only when mission_ != nullptr)
 *
 * All implementations are intentionally trivial — if they are reached the
 * test will simply behave as if the operation succeeded (or returned a safe
 * default).  The routing tests do not exercise these paths.
 */

#include "sys/led/status_led.h"
#include "sys/wifi/wifi_ap.h"
#include "ams/mission_script_engine.h"

// ── StatusLed ────────────────────────────────────────────────────────────────

void StatusLed::setMode(ares::OperatingMode /*mode*/) {}

// ── WifiAp ───────────────────────────────────────────────────────────────────

bool    WifiAp::isReady()     const { return false; }
uint8_t WifiAp::clientCount() const { return 0U; }

// ── MissionScriptEngine ──────────────────────────────────────────────────────

namespace ares::ams {

void MissionScriptEngine::getSnapshot(EngineSnapshot& out) const
{
    out = EngineSnapshot{};
}

bool MissionScriptEngine::arm()                          { return false; }
void MissionScriptEngine::setExecutionEnabled(bool)      {}
bool MissionScriptEngine::injectTcCommand(const char*)   { return false; }
void MissionScriptEngine::deactivate()                   {}
bool MissionScriptEngine::activate(const char*)          { return false; }
bool MissionScriptEngine::listScripts(FileEntry* /*entries*/,
                                      uint8_t    /*maxEntries*/,
                                      uint8_t&   /*outCount*/) const
{
    return false;
}

} // namespace ares::ams

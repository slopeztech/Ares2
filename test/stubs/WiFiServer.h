/**
 * @file  WiFiServer.h
 * @brief Minimal WiFiServer stub for native api_routing test builds.
 *
 * api_server.cpp declares a file-scope `static WiFiServer httpServer(port)`
 * and calls httpServer.begin() / httpServer.setNoDelay() in begin().
 * The run() loop calls httpServer.accept() to poll for connections.
 *
 * Tests never call begin() or run(), so all methods are safe no-ops.
 */
#pragma once

#include "WiFiClient.h"
#include <cstdint>

/**
 * @brief Stub WiFiServer — satisfies the linker without a TCP stack.
 *
 * accept() returns a default-constructed WiFiClient whose connected()
 * returns false, so the run() loop body is never entered even if run()
 * were called.
 */
class WiFiServer
{
public:
    explicit WiFiServer(uint16_t /*port*/) {}

    void begin()              {}
    void setNoDelay(bool /*flag*/) {}

    /** Returns a disconnected client so run() skips the handler body. */
    WiFiClient accept() { return WiFiClient{}; }
};

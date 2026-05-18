/**
 * @file  WiFi.h
 * @brief Minimal WiFi umbrella stub for native api_routing test builds.
 *
 * api_server.cpp includes <WiFi.h>, <WiFiClient.h>, and <WiFiServer.h>
 * separately.  This header satisfies <WiFi.h> by pulling in the other two
 * stubs so a single include path covers all three.
 */
#pragma once

#include "WiFiClient.h"
#include "WiFiServer.h"

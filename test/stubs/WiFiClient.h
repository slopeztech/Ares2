/**
 * @file  WiFiClient.h
 * @brief WiFiClient stub for native api_routing test builds.
 *
 * Provides:
 *   - WiFiClient   — abstract base with all methods called by api_server.cpp
 *                    and the handler translation units.
 *   - SimWiFiClient — injects a canned HTTP request and captures the full
 *                     response so tests can inspect HTTP status codes and body.
 */
#pragma once

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cstdarg>
#include <string>
#include <algorithm>

/**
 * @brief Minimal WiFiClient base class.
 *
 * Satisfies compilation of api_server.cpp, flight_handler.cpp and all other
 * handler TUs that accept a WiFiClient reference.  Default implementations
 * are no-ops / return-safe values so the class can be used as a base for
 * SimWiFiClient.
 */
class WiFiClient
{
public:
    virtual ~WiFiClient() = default;

    // ── Connection management ──────────────────────────────────

    virtual bool connected() { return false; }
    virtual void stop() {}
    virtual void setTimeout(uint32_t /*ms*/) {}

    // ── Read interface ─────────────────────────────────────────

    virtual int available() { return 0; }
    virtual int read()      { return -1; }

    virtual size_t readBytesUntil(char /*term*/, char* buf, size_t len)
    {
        if (len > 0U) { buf[0] = '\0'; }
        return 0U;
    }

    virtual size_t readBytes(char* /*buf*/, size_t /*len*/) { return 0U; }

    // ── Write interface ────────────────────────────────────────

    virtual size_t write(const uint8_t* buf, size_t size)
    {
        (void)buf;
        return size;
    }

    size_t write(const char* buf, size_t size)
    {
        return write(reinterpret_cast<const uint8_t*>(buf), size);
    }

    virtual size_t write(uint8_t c) { return write(&c, 1U); }

    virtual size_t print(const char* s)
    {
        if (s == nullptr) { return 0U; }
        const size_t n = strlen(s);
        return write(reinterpret_cast<const uint8_t*>(s), n);
    }

    /**
     * Formatted print — not virtual because variadic functions cannot be
     * virtual in C++.  Delegates to write() so subclass overrides are
     * still exercised.
     */
    size_t printf(const char* fmt, ...)  // NOLINT(cert-dcl50-cpp)
    {
        char buf[512] = {};
        va_list args;
        va_start(args, fmt);
        const int n = vsnprintf(buf, sizeof(buf), fmt, args);
        va_end(args);
        if (n <= 0) { return 0U; }
        return write(reinterpret_cast<const uint8_t*>(buf),
                     static_cast<size_t>(n));
    }
};

// ─────────────────────────────────────────────────────────────────────────────

/**
 * @brief Inject a hand-crafted HTTP request and capture the full response.
 *
 * Used by test_api_routing to exercise ApiServer::routeRequest() without a
 * live TCP stack.  The request string is provided at construction time and
 * consumed byte-by-byte through the read interface.  All write/print/printf
 * output is appended to an internal std::string for post-call inspection.
 *
 * Usage:
 * @code
 *   SimWiFiClient c{""};          // No bytes needed when calling routeRequest directly
 *   server.routeRequest(c, "GET", "/api/status", "", 0, "");
 *   TEST_ASSERT_EQUAL_INT(200, c.statusCode());
 * @endcode
 */
class SimWiFiClient : public WiFiClient
{
public:
    explicit SimWiFiClient(const char* request)
        : pos_(0U), request_(request != nullptr ? request : "") {}

    // ── Connection ─────────────────────────────────────────────

    bool connected() override { return true; }
    void stop()      override {}
    void setTimeout(uint32_t /*ms*/) override {}

    // ── Read ───────────────────────────────────────────────────

    int available() override
    {
        const size_t rem = request_.size() - pos_;
        return static_cast<int>(rem);
    }

    int read() override
    {
        if (pos_ >= request_.size()) { return -1; }
        return static_cast<uint8_t>(request_[pos_++]);
    }

    size_t readBytesUntil(char term, char* buf, size_t len) override
    {
        size_t i = 0U;
        while (i < (len - 1U) && pos_ < request_.size())
        {
            const char c = request_[pos_++];
            if (c == term) { break; }
            buf[i++] = c;
        }
        buf[i] = '\0';
        return i;
    }

    size_t readBytes(char* buf, size_t len) override
    {
        const size_t toRead = std::min(len, request_.size() - pos_);
        memcpy(buf, request_.c_str() + pos_, toRead);
        pos_ += toRead;
        return toRead;
    }

    // ── Write (captures response) ──────────────────────────────

    size_t write(const uint8_t* buf, size_t size) override
    {
        response_.append(reinterpret_cast<const char*>(buf), size);
        return size;
    }

    // ── Inspection helpers ─────────────────────────────────────

    /** Full HTTP response text captured since construction or last reset. */
    const std::string& response() const { return response_; }

    void resetResponse() { response_.clear(); }

    /**
     * Parse the HTTP status code from the first response line.
     * Expects the format produced by ApiServer: "HTTP/1.1 NNN reason\r\n".
     * @return Numeric status code, or 0 if the line is absent or malformed.
     */
    int statusCode() const
    {
        const char* p = response_.c_str();
        const char* sp = strchr(p, ' ');
        if (sp == nullptr) { return 0; }
        return static_cast<int>(strtol(sp + 1, nullptr, 10));
    }

private:
    size_t      pos_;
    std::string request_;
    std::string response_;
};

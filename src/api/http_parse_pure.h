/**
 * @file  http_parse_pure.h
 * @brief Pure HTTP-parsing helpers — no Arduino / WiFi dependencies.
 *
 * Extracted so that host-native unit tests can exercise the logic
 * without pulling in the full ApiServer.
 *
 * Covers RFC 7230 §3.2.3 optional whitespace (OWS):
 *
 *   OWS = *( SP / HTAB )
 *
 * Both leading OWS (between ": " and the field value) and trailing OWS
 * (spaces/tabs at the end of a header value) must be stripped before
 * the value is compared or stored (RFC 7230 §3.2.6).
 */
#pragma once

#include <cstddef>

namespace ares
{
namespace api
{

/**
 * @brief Skip leading OWS (SP / HTAB) up to @p max characters.
 *
 * The @p max bound prevents an unbounded scan in case the header line
 * contains only whitespace — the caller should pass HTTP_HEADER_MAX.
 *
 * @param p    Pointer to the start of the field value (after the colon).
 * @param max  Maximum number of OWS characters to skip.
 * @return Pointer to the first non-OWS character, or to the NUL terminator /
 *         @p p + @p max, whichever comes first.
 */
inline const char* owsSkipLeading(const char* p, size_t max)
{
    size_t i = 0U;
    while (i < max && (*p == ' ' || *p == '\t'))
    {
        ++p;
        ++i;
    }
    return p;
}

/**
 * @brief Strip trailing OWS from a NUL-terminated buffer in-place.
 *
 * Overwrites trailing SP / HTAB characters with '\0'.
 *
 * @param buf  NUL-terminated string buffer.
 * @param len  Current string length (must equal strlen(buf)).
 * @return New string length after stripping.
 */
inline size_t owsTrimTrailing(char* buf, size_t len)
{
    while (len > 0U && (buf[len - 1U] == ' ' || buf[len - 1U] == '\t'))
    {
        buf[--len] = '\0';
    }
    return len;
}

} // namespace api
} // namespace ares

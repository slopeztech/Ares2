/**
 * @file  ares_util.h
 * @brief Portable, bounds-safe string utility helpers for the ARES firmware.
 *
 * All helpers are inline so each translation unit gets its own copy without
 * violating the One-Definition Rule.
 */
#pragma once

#include <cstddef>
#include <cstring>

namespace ares
{
namespace util
{

/**
 * @brief Copy a NUL-terminated string into a fixed-size buffer and always
 *        NUL-terminate the destination.
 *
 * This is the canonical replacement for the two-step idiom:
 * @code
 *   strncpy(dst, src, n - 1U);
 *   dst[n - 1U] = '\0';
 * @endcode
 *
 * If @p src is longer than @p n - 1 bytes the string is truncated.
 * If @p src is shorter the remaining bytes are NUL-padded by strncpy.
 *
 * @param[out] dst  Destination buffer.  Must have capacity @p n.
 * @param[in]  src  NUL-terminated source string.  Must not be nullptr.
 * @param[in]  n    Total size of @p dst in bytes (pass @c sizeof(dst)).
 *                  Must be non-zero.
 *
 * @note MISRA-C++ M5-0-14 / CERT-STR32: explicit NUL write at dst[n-1]
 *       guarantees termination even when strlen(src) >= n - 1.
 */
// NOLINTNEXTLINE(bugprone-not-null-terminated-result) — NUL written on the next line
inline void copyZ(char* dst, const char* src, std::size_t n) noexcept
{
    strncpy(dst, src, n - 1U);
    dst[n - 1U] = '\0';
}

} // namespace util
} // namespace ares

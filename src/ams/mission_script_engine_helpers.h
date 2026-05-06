/**
 * @file  mission_script_engine_helpers.h
 * @brief Internal arithmetic and string helpers shared across AMS engine
 *        translation units.
 *
 * All helpers are defined @c inline so each including translation unit gets
 * its own copy without violating the One-Definition Rule.
 *
 * Thread safety: all helpers are pure functions with no shared mutable state.
 */
#pragma once

#include <cstdint>
#include <cstdio>
#include <cinttypes>

namespace ares
{
namespace ams
{
namespace detail
{

/**
 * @brief Return true when @p text is non-null and contains only whitespace
 *        or is the empty string.
 *
 * Used by the parser to reject spurious suffix characters after numeric
 * literals (e.g. "500ms" should fail; "500" should pass).
 *
 * @param[in] text  Null-terminated string.  @c nullptr returns @c false.
 * @return @c true if every remaining character is a space, tab, or NUL.
 */
inline bool isOnlyTrailingWhitespace(const char* text)
{
    if (text == nullptr) { return false; }
    while (*text == ' ' || *text == '\t') { ++text; }
    return *text == '\0';
}

/**
 * @brief Compute 10^digits using integer arithmetic (no floating-point).
 *
 * @param[in] digits  Non-negative exponent.  Practical range: 0–9.
 * @return 10 raised to the power of @p digits.
 */
inline uint32_t pow10u(uint8_t digits)
{
    uint32_t value = 1U;
    for (uint8_t i = 0U; i < digits; ++i) { value *= 10U; }
    return value;
}

/**
 * @brief Format a @c float as a fixed-point decimal string without @c printf
 *        @c \%f.
 *
 * Uses integer arithmetic to avoid locale-dependent decimal separators and
 * unwanted double-precision promotion.  The output buffer is always
 * NUL-terminated on success.
 *
 * @param[in]  value    Value to format.
 * @param[in]  decimals Digits after the decimal point (0 = integer output).
 * @param[out] out      Output buffer; NUL-terminated on success.
 * @param[in]  outSize  Capacity of @p out in bytes (must be > 0).
 * @return @c true if the formatted value fits in @p out without truncation.
 * @pre  @p out != @c nullptr, @p outSize > 0.
 */
inline bool formatScaledFloat(float    value,
                               uint8_t  decimals,
                               char*    out,
                               uint32_t outSize)
{
    if (out == nullptr || outSize == 0U) { return false; }

    const uint32_t scale = pow10u(decimals);
    const bool     neg   = (value < 0.0f);
    const float    absV  = neg ? -value : value;
    const float    scalF = (absV * static_cast<float>(scale)) + 0.5f;
    const uint64_t scal  = static_cast<uint64_t>(scalF);
    const uint32_t whole = static_cast<uint32_t>(scal / scale);
    const uint32_t frac  = static_cast<uint32_t>(scal % scale);

    int32_t written = 0;
    if (decimals == 0U)
    {
        written = static_cast<int32_t>(snprintf(out, outSize,
                           "%s%" PRIu32, neg ? "-" : "", whole));
    }
    else
    {
        written = static_cast<int32_t>(snprintf(out, outSize,
                           "%s%" PRIu32 ".%0*" PRIu32,
                           neg ? "-" : "", whole,
                           static_cast<int>(decimals), frac));
    }
    return (written > 0) && (static_cast<uint32_t>(written) < outSize);
}

} // namespace detail
} // namespace ams
} // namespace ares

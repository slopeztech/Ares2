/**
 * @file  ams_token_cursor.h
 * @brief Lightweight no-heap tokenizer cursor for AMS line parsing (AMS-4.20).
 *
 * TokenCursor wraps a @c const @c char* and provides composable helpers to
 * consume tokens in a uniform, bounds-safe way — replacing scattered
 * @c sscanf / @c %n and @c strtol patterns across the AMS parser with
 * explicit, testable operations.
 *
 * Design constraints:
 *   - No heap allocation; no exceptions.
 *   - Works inside FreeRTOS task context (no TLS, no global state).
 *   - All mutating helpers return @c bool and leave the cursor unchanged on
 *     failure (roll-back semantics).
 *   - @c ensureEol() / @c isEol() treat trailing spaces and tabs as
 *     end-of-line, consistent with @c detail::isOnlyTrailingWhitespace().
 *
 * Typical usage pattern:
 * @code
 *   TokenCursor cur(line);
 *   char model[16] = {};
 *   char alias[16] = {};
 *   if (!cur.expectLiteral("include ")
 *       || !cur.readIdentBounded(model, sizeof(model))
 *       || !cur.expectLiteral(" as ")
 *       || !cur.readIdentBounded(alias, sizeof(alias)))
 *   {
 *       setErrorLocked("...");
 *       return false;
 *   }
 *   // cur.remaining() points past the consumed tokens.
 * @endcode
 *
 * Thread safety: TokenCursor carries no shared mutable state; concurrent
 * use of separate instances is inherently safe.
 */
#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>

namespace ares
{
namespace ams
{

/**
 * @brief Cursor over a NUL-terminated C-string for AMS line tokenization.
 */
struct TokenCursor
{
    // ── Construction ─────────────────────────────────────────────────────────

    /**
     * @brief Construct a cursor anchored at @p line.
     * @pre  @p line is non-null and NUL-terminated.
     */
    explicit constexpr TokenCursor(const char* line) noexcept : p_(line) {}

    // ── Position ─────────────────────────────────────────────────────────────

    /** @brief Pointer to the current (unconsumed) character. */
    const char* remaining() const noexcept { return p_; }

    // ── Whitespace ───────────────────────────────────────────────────────────

    /**
     * @brief Skip over any leading spaces and tabs in-place.
     *
     * Never fails; may be a no-op when the cursor already points to a
     * non-whitespace character.
     */
    void skipWs() noexcept
    {
        while (*p_ == ' ' || *p_ == '\t') { ++p_; }
    }

    // ── Literal matching ─────────────────────────────────────────────────────

    /**
     * @brief Expect and consume an exact literal prefix.
     *
     * On success advances the cursor past the literal.
     * On failure the cursor is unchanged (roll-back semantics).
     *
     * @param[in] lit  Literal to match (NUL-terminated, non-null).
     * @return @c true if the input starts with @p lit.
     */
    bool expectLiteral(const char* lit) noexcept
    {
        const char* q = p_;
        while (*lit != '\0')
        {
            if (*q != *lit) { return false; }   // mismatch — cursor unchanged
            ++q;
            ++lit;
        }
        p_ = q;     // full match — commit
        return true;
    }

    // ── Identifier helpers ───────────────────────────────────────────────────

    /**
     * @brief Read a plain identifier @c [A-Za-z0-9_]+ into @p buf.
     *
     * On success advances the cursor past the identifier.
     * On failure (empty input or buffer overflow) the cursor is unchanged.
     *
     * @param[out] buf      Output buffer (NUL-terminated on success).
     * @param[in]  bufSize  Capacity of @p buf in bytes (must be ≥ 2).
     * @return @c true if at least one identifier character was consumed and
     *         the result fits in @p buf.
     */
    bool readIdentBounded(char* buf, size_t bufSize) noexcept
    {
        if (buf == nullptr || bufSize < 2U) { return false; }
        const char* q   = p_;
        size_t      len = 0U;
        while (isIdentChar(*q))
        {
            if (len >= bufSize - 1U) { return false; }   // would overflow — unchanged
            buf[len++] = *q++;
        }
        if (len == 0U) { return false; }   // no identifier chars found
        buf[len] = '\0';
        p_ = q;
        return true;
    }

    /**
     * @brief Read a dotted-identifier key @c [A-Za-z0-9_.]+  into @p buf.
     *
     * Extends @c readIdentBounded to allow @c '.' so that radio.config
     * parameter names such as @c "monitor.alt.high" are consumed in one call.
     *
     * On success advances the cursor past the key.
     * On failure the cursor is unchanged.
     *
     * @param[out] buf      Output buffer (NUL-terminated on success).
     * @param[in]  bufSize  Capacity of @p buf in bytes (must be ≥ 2).
     * @return @c true if at least one character was consumed and the result
     *         fits in @p buf.
     */
    bool readKeyBounded(char* buf, size_t bufSize) noexcept
    {
        if (buf == nullptr || bufSize < 2U) { return false; }
        const char* q   = p_;
        size_t      len = 0U;
        while (isIdentChar(*q) || *q == '.')
        {
            if (len >= bufSize - 1U) { return false; }
            buf[len++] = *q++;
        }
        if (len == 0U) { return false; }
        buf[len] = '\0';
        p_ = q;
        return true;
    }

    // ── Numeric helpers ──────────────────────────────────────────────────────

    /**
     * @brief Read an unsigned decimal integer into @p out.
     *
     * Requires at least one digit at the current position.  Stops at the
     * first non-digit character (does NOT require end-of-line; call
     * @c ensureEol() explicitly if trailing garbage must be rejected).
     * Returns @c false and leaves the cursor unchanged on overflow or when
     * no digits are present.
     *
     * @param[out] out  Parsed value on success.
     * @return @c true if at least one digit was consumed without overflow.
     */
    bool readUint32(uint32_t& out) noexcept
    {
        const char* q   = p_;
        uint64_t    acc = 0U;
        bool        saw = false;

        while (*q >= '0' && *q <= '9')
        {
            saw = true;
            acc = (acc * 10U) + static_cast<uint64_t>(*q - '0');
            if (acc > static_cast<uint64_t>(UINT32_MAX)) { return false; }
            ++q;
        }
        if (!saw) { return false; }

        out = static_cast<uint32_t>(acc);
        p_  = q;
        return true;
    }

    // ── Quoted-string helper ─────────────────────────────────────────────────

    /**
     * @brief Read a double-quoted string into @p buf, stripping the quotes.
     *
     * The cursor must point at the opening @c " on entry.
     * On success the cursor is positioned immediately after the closing @c ".
     * On failure (no opening quote, no closing quote, buffer overflow) the
     * cursor is unchanged.
     *
     * @param[out] buf      Buffer for the content without quotes (NUL-terminated
     *                      on success).  May be @c nullptr if the caller only
     *                      needs to skip the token.
     * @param[in]  bufSize  Capacity of @p buf.  Ignored when @p buf is @c nullptr.
     * @return @c true if a complete quoted string was consumed.
     */
    bool readQuoted(char* buf, size_t bufSize) noexcept
    {
        if (*p_ != '"') { return false; }
        const char* q   = p_ + 1U;   // skip opening '"'
        size_t      len = 0U;

        while (*q != '"' && *q != '\0')
        {
            if (buf != nullptr && bufSize >= 2U)
            {
                if (len >= bufSize - 1U) { return false; }   // would overflow
                buf[len] = *q;
            }
            ++len;
            ++q;
        }
        if (*q != '"') { return false; }   // unterminated string

        if (buf != nullptr && bufSize >= 1U) { buf[len] = '\0'; }
        p_ = q + 1U;   // skip closing '"'
        return true;
    }

    // ── End-of-line helpers ──────────────────────────────────────────────────

    /**
     * @brief Return @c true if only whitespace (or NUL) remains.
     *
     * Non-consuming; does not advance the cursor.
     */
    bool isEol() const noexcept
    {
        const char* q = p_;
        while (*q == ' ' || *q == '\t') { ++q; }
        return *q == '\0';
    }

    /**
     * @brief Advance past trailing whitespace and return @c true only if NUL
     *        is reached.
     *
     * Use after consuming all expected tokens to reject spurious trailing
     * garbage.  Advances the cursor to the NUL on success.
     *
     * @return @c true if nothing but whitespace remained.
     */
    bool ensureEol() noexcept
    {
        skipWs();
        return *p_ == '\0';
    }

private:
    const char* p_;   ///< Current position in the source string.

    /** @brief Return @c true for characters valid in a plain identifier. */
    static bool isIdentChar(char c) noexcept
    {
        return (c >= 'A' && c <= 'Z')
            || (c >= 'a' && c <= 'z')
            || (c >= '0' && c <= '9')
            || (c == '_');
    }
};

} // namespace ams
} // namespace ares

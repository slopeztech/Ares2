/**
 * @file  ares_log.cpp
 * @brief Structured logging implementation.
 *
 * A single static buffer is used for formatting to avoid heap
 * allocation (PO10-3).  Messages longer than LOG_BUF_SIZE are
 * silently truncated — the trailing newline is always preserved
 * so the serial console never shows run-on lines.
 */
#include "ares_log.h"

static ares::log::Level runtimeLevel_ =
    static_cast<ares::log::Level>(ARES_LOG_LEVEL);

// Fixed-size buffer for formatting (PO10-3: no dynamic alloc).
static constexpr uint16_t LOG_BUF_SIZE = 192;
static char logBuf_[LOG_BUF_SIZE];

namespace ares
{
namespace log
{

void setLevel(Level level)
{
    runtimeLevel_ = level;
}

Level getLevel()
{
    return runtimeLevel_;
}

void emitV(char levelChar, const char* tag, const char* fmt, va_list args)
{
    // Cannot use ARES_ASSERT here: the assert macro itself calls
    // LOG_E, which would cause infinite recursion on failure.
    if (tag == nullptr || fmt == nullptr)
    {
        return;
    }

    // Phase 1: write the fixed-format header "[  uptime] L TAG: "
    const uint32_t ms = static_cast<uint32_t>(millis());
    int32_t pos = snprintf(logBuf_, LOG_BUF_SIZE, "[%7u] %c %s: ",
                           ms, levelChar, tag);

    // Clamp: if snprintf failed or overflowed, reserve space for
    // the trailing '\n' + '\0' that we always append.
    if (pos < 0 || pos >= static_cast<int32_t>(LOG_BUF_SIZE))
    {
        pos = LOG_BUF_SIZE - 2;
    }

    // Phase 2: append the caller’s formatted message
    va_list argsCopy;
    va_copy(argsCopy, args);
    int32_t msgLen = vsnprintf(
        logBuf_ + pos,
        static_cast<uint16_t>(LOG_BUF_SIZE - static_cast<uint16_t>(pos)),
        fmt, argsCopy);
    va_end(argsCopy);

    if (msgLen < 0)
    {
        msgLen = 0;
    }
    pos += msgLen;

    // Ensure the buffer always ends with "\n\0" so each log
    // entry occupies exactly one console line.
    if (pos >= static_cast<int32_t>(LOG_BUF_SIZE) - 1)
    {
        pos = LOG_BUF_SIZE - 2;
    }
    logBuf_[pos]     = '\n';
    logBuf_[pos + 1] = '\0';

    (void)Serial.print(logBuf_);  // PO10-7: return value not safety-relevant
}

void emit(char levelChar, const char* tag, const char* fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    emitV(levelChar, tag, fmt, args);
    va_end(args);
}

} // namespace log
} // namespace ares

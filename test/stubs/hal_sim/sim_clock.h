/**
 * @file  sim_clock.h
 * @brief Shared simulation time source for all SITL HAL drivers.
 *
 * All sim drivers derive their timestamps from a single monotonically
 * advancing counter — g_simTimeMs — rather than from wall-clock time.
 * This keeps determinism regardless of host CPU speed and allows tests
 * to advance time in discrete, reproducible steps.
 *
 * Usage:
 *   1. Reset to zero before each test: ares::sim::clock::reset();
 *   2. Advance in the test tick loop:  ares::sim::clock::advanceMs(delta);
 *   3. Query current time:             ares::sim::clock::nowMs();
 *
 * Thread safety: NOT thread-safe.  The sim is single-threaded by design.
 */
#pragma once

#include <cstdint>

namespace ares
{
namespace sim
{
namespace clock
{

// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
inline uint32_t g_simTimeMs = 0U;  // C++17 inline variable — one definition across all TUs.

/** Reset the simulation clock to zero. */
inline void reset()
{
    g_simTimeMs = 0U;
}

/** Advance the simulation clock by @p deltaMs milliseconds. */
inline void advanceMs(uint32_t deltaMs)
{
    g_simTimeMs += deltaMs;
}

/** Return the current simulation time in milliseconds. */
inline uint32_t nowMs()
{
    return g_simTimeMs;
}

} // namespace clock
} // namespace sim
} // namespace ares

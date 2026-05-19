/**
 * @file  mission_script_engine_internal.h
 * @brief AMS engine internal runtime-state type definitions.
 *
 * Contains types used exclusively by the engine at runtime (not the parser
 * data model): deferred I/O staging structs (PendingCheckpoint, PendingAppend)
 * and the ST[12] on-board parameter monitoring types (MonitoringDefinition,
 * MonitoringSlot).
 *
 * Included by mission_script_engine.h; all types live in ares::ams namespace.
 *
 * Design constraints:
 *   - No dynamic allocation (PO10-3)
 *   - kMaxPendingAppends and kMaxMonitorSlots are bounded compile-time constants
 */
#pragma once

#include <cstdint>

#include "comms/ares_radio_protocol.h"
#include "config.h"

namespace ares
{
namespace ams
{

// ── Deferred I/O staging (AMS-8.3) ───────────────────────────────────────────
// Checkpoint writes and CSV log appends are assembled inside the mutex but
// executed by flushPendingIoUnlocked() after the lock is released.  This
// keeps blocking LittleFS latency out of the critical section so that
// injectTcCommand("ABORT") and other API callers cannot timeout on I/O.

/// Worst-case pending appends per tick:
///   1 header + 1 data row per LOG slot = 2 × AMS_MAX_HK_SLOTS = 8,
///   plus 1 legacy header + 1 legacy data row = 10 entries total.
constexpr uint8_t kMaxPendingAppends =
    static_cast<uint8_t>(ares::AMS_MAX_HK_SLOTS * 2U + 2U);

/** @brief Staged checkpoint record awaiting flush to LittleFS (AMS-8.3). */
struct PendingCheckpoint
{
    char    buf[512];  ///< Serialised v4 checkpoint record (static-size member).
    int32_t len;       ///< Valid bytes in buf[]; 0 = nothing staged.
    bool    pending;   ///< True iff buf[] holds an unflushed record.
};

/** @brief Staged CSV log-row awaiting flush to LittleFS (AMS-8.3). */
struct PendingAppend
{
    char     path[ares::STORAGE_MAX_PATH]; ///< Target LittleFS path.
    uint8_t  data[256];                    ///< Serialised row bytes (static-size member).
    uint32_t len;                          ///< Valid bytes in data[].
    /// If non-null, set to true when appendFile succeeds.
    /// Used by CSV header entries so the written flag is only raised
    /// after a confirmed flush — enabling retry on NO_SPACE (AMS-4.3.2).
    bool*    onSuccessFlag;                ///< Set on successful flush; null for data rows.
};

// ── ST[12] on-board parameter monitoring (APUS-12) ───────────────────────────

/**
 * Compile-time monitoring definition for one parameter (APUS-12.1).
 * Defines the parameter to monitor, its limit bounds, and the consecutive-
 * violation count required before transitioning to ALARM (APUS-12.2).
 */
struct MonitoringDefinition
{
    ares::proto::MonitorParamId parameterId;   ///< Field of TelemetryPayload to check.
    float   lowLimit;                          ///< Below this → violation.
    float   highLimit;                         ///< Above this → violation.
    uint8_t consecutiveRequired;               ///< N violations before ALARM (APUS-12.2).
    bool    enabled;                           ///< false = DISABLED (APUS-12.5).
};

/**
 * Runtime state for one monitoring slot: definition + FSM state + counter.
 */
struct MonitoringSlot
{
    MonitoringDefinition         def;            ///< Immutable definition (modifiable via API).
    ares::proto::MonitoringState state;          ///< Current FSM state (DISABLED/ENABLED/ALARM).
    uint8_t                      consecutiveHit; ///< Current consecutive violation count.
};

/// Maximum concurrent ST[12] monitoring slots (APUS-12.3).
constexpr uint8_t kMaxMonitorSlots = 4U;

} // namespace ams
} // namespace ares

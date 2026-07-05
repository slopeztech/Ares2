/**
 * @file  littlefs_storage.h
 * @brief Thread-safe LittleFS storage service for ESP32-S3.
 *
 * All public methods are guarded by a FreeRTOS mutex with
 * priority inheritance (RTOS-4, CERT-13).  The mutex is
 * statically allocated (PO10-3).
 *
 * Thread safety: All public methods are thread-safe.
 *                Safe to call from any task after begin().
 */
#pragma once

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <LittleFS.h>

#include "config.h"
#include "hal/storage/storage_interface.h"

/**
 * Concrete StorageInterface backed by LittleFS on the ESP32
 * internal flash partition.
 *
 * Uses the `spiffs` partition defined in partitions.csv
 * (LittleFS shares the same partition type).
 *
 * Concurrency model:
 *   - A single FreeRTOS mutex serialises all filesystem access.
 *   - The mutex uses priority inheritance to prevent inversion
 *     (RTOS-4.1).
 *   - Every lock attempt has a bounded timeout (RTOS-8).
 *   - At most one file is open at any time, keeping the VFS
 *     file-descriptor pool at O(1).
 */
class LittleFsStorage : public StorageInterface
{
public:
    /** Construct a LittleFS storage instance (filesystem mounted on begin()). */
    LittleFsStorage();

    // Non-copyable, non-movable (CERT-18.3)
    LittleFsStorage(const LittleFsStorage&)            = delete;
    LittleFsStorage& operator=(const LittleFsStorage&) = delete;
    LittleFsStorage(LittleFsStorage&&)                 = delete;
    LittleFsStorage& operator=(LittleFsStorage&&)      = delete;

    /** Mount the LittleFS filesystem and initialise the mutex. @return true on success. */
    bool begin() override;
    /** Unmount the filesystem and release resources. */
    void end() override;

    /** @copydoc StorageInterface::exists */
    StorageStatus exists(const char* path, bool& result) override;
    StorageStatus readFile(const char* path, uint8_t* buf,
                           uint32_t bufSize,
                           uint32_t& bytesRead) override;
    StorageStatus writeFile(const char* path,
                            const uint8_t* data,
                            uint32_t len) override;
    StorageStatus appendFile(const char* path,
                             const uint8_t* data,
                             uint32_t len) override;
    /** @copydoc StorageInterface::removeFile */
    StorageStatus removeFile(const char* path) override;
    StorageStatus renameFile(const char* oldPath,
                             const char* newPath) override;
    /** @copydoc StorageInterface::fileSize */
    StorageStatus fileSize(const char* path, uint32_t& size) override;
    /** @copydoc StorageInterface::info */
    StorageStatus info(StorageInfo& out) override;
    /** @copydoc StorageInterface::health */
    StorageStatus health(StorageHealth& out) override;
    StorageStatus listFiles(const char* dir, FileEntry* entries,
                            uint8_t maxEntries,
                            uint8_t& count) override;
    StorageStatus readFileChunk(const char* path, uint32_t offset,
                                uint8_t* buf, uint32_t bufSize,
                                uint32_t& bytesRead) override;

    /**
     * Flush the internal append-stream cache for the currently open log file.
     *
     * The cache keeps one `File` handle open across consecutive `appendFile()`
     * calls to the same path (to eliminate open/close overhead per row).  The
     * RAM write-behind buffer is committed to flash and the file handle is
     * flushed.  Safe to call even if no file is currently cached.
     *
     * Available for higher-level code to force durability after draining an
     * append queue.  `LittleFsStorage` also calls this automatically before any
     * `readFile()` / `readFileChunk()` on the same path, so downloaded logs
     * always reflect the latest data.
     */
    StorageStatus flushCachedAppend() override;

private:
    /// Validate a file path: non-null, starts with '/', bounded length.
    static bool validatePath(const char* path);

    /// Internal write helper shared by writeFile() and appendFile().
    /// @param[in] path  Validated file path.
    /// @param[in] data  Source buffer.
    /// @param[in] len   Bytes to write.
    /// @param[in] mode  LittleFS open mode ("w" or "a").
    /// @return Status code.
    StorageStatus writeInternal(const char* path,
                                const uint8_t* data,
                                uint32_t len,
                                const char* mode);

    /**
     * Open (or reopen) the append-stream cache for @p path.
     *
     * Flushes and closes any previously cached handle, then opens @p path
     * in append mode and stores it in @c cachedAppendFile_.
     *
     * @pre  mutex_ is held by the caller.
     * @return true on success; false on any filesystem error.
     */
    bool openAppendCacheLocked(const char* path);

    /// Static mutex buffer — no heap allocation (PO10-3).
    StaticSemaphore_t mutexBuf_ = {};

    /// FreeRTOS mutex handle (priority inheritance, RTOS-4.1).
    SemaphoreHandle_t mutex_ = nullptr;

    /// true after successful begin().
    bool mounted_ = false;

    /// Last known storage health and recovery counters.
    StorageHealth health_ = {};

    /// Max path length including leading '/' and null terminator.
    /// LittleFS ESP32 supports longer, but short paths reduce
    /// stack usage and flash wear (CERT-3.2).
    static constexpr uint8_t MAX_PATH_LEN = ares::STORAGE_MAX_PATH;

    /// Mutex acquisition timeout in ticks (RTOS-8).
    static constexpr TickType_t MUTEX_TIMEOUT =
        pdMS_TO_TICKS(ares::STORAGE_MUTEX_TIMEOUT_MS);

    /// Maximum single-file size in bytes.
    /// Prevents a runaway write from filling the partition.
    static constexpr uint32_t MAX_FILE_SIZE = ares::STORAGE_MAX_FILE;

    // ── Append-stream cache + RAM write-behind buffer (STOR-2) ──────────────
    // Two-stage optimisation to make high-rate CSV logging fast:
    //   1. Keep the most-recently-appended file open (cachedAppendFile_) so the
    //      LittleFS open()+close() overhead (~15 ms each) is paid once, not per
    //      row.
    //   2. Accumulate row bytes in a large RAM buffer (appendRamBuf_) and commit
    //      them to flash with a single f.write() only when the buffer fills or
    //      the durability timer (AMS_IO_APPEND_FLUSH_MS) elapses.  A LittleFS
    //      flash program costs ~40-80 ms regardless of size, so batching ~100
    //      rows per write drops the amortised per-row cost to well under 1 ms.
    // The buffer is transparent to callers; it is flushed automatically before
    // any read of the same path and on end(), so downloaded logs are complete.

    /// Currently open file handle for append streaming; invalid if none cached.
    File     cachedAppendFile_;
    /// Path associated with cachedAppendFile_; empty string when no cache.
    char     cachedAppendPath_[MAX_PATH_LEN] = {};
    /// RAM write-behind buffer holding not-yet-flushed row bytes.
    uint8_t  appendRamBuf_[ares::AMS_IO_APPEND_RAM_BYTES] = {};
    /// Valid bytes currently staged in appendRamBuf_.
    uint32_t appendRamLen_ = 0U;
    /// millis() timestamp of the last flash commit; drives the durability timer.
    uint32_t appendRamLastFlushMs_ = 0U;

    /// Commit the RAM write-behind buffer to the cached file handle (no f.flush).
    /// @pre  mutex_ is held by the caller.  cachedAppendFile_ is valid.
    /// @return true on success (or nothing to commit); false on short write.
    bool commitAppendRamBufLocked();

    /// Stage one row into the RAM write-behind buffer, committing to flash when
    /// the buffer fills or the durability timer elapses (STOR-2).
    /// @param[in] data  Row bytes.
    /// @param[in] len   Byte count (already validated as 0 < len <= MAX_FILE_SIZE).
    /// @param[in] path  Destination path (for diagnostics only).
    /// @pre  mutex_ is held.  cachedAppendFile_ is open for @p path.
    /// @return Status code.
    StorageStatus stageAppendRowLocked(const uint8_t* data,
                                       uint32_t       len,
                                       const char*    path);

    /// If @p path is the currently cached append stream, commit its RAM buffer
    /// and close the handle so a following write/remove/rename on the same file
    /// operates on a consistent, fully-flushed file.  No-op otherwise.
    /// @pre  mutex_ is held by the caller.
    void invalidateAppendCacheIfPathLocked(const char* path);
};

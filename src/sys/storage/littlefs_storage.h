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
    LittleFsStorage();

    // Non-copyable, non-movable (CERT-18.3)
    LittleFsStorage(const LittleFsStorage&)            = delete;
    LittleFsStorage& operator=(const LittleFsStorage&) = delete;
    LittleFsStorage(LittleFsStorage&&)                 = delete;
    LittleFsStorage& operator=(LittleFsStorage&&)      = delete;

    bool begin() override;
    void end() override;

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
    StorageStatus removeFile(const char* path) override;
    StorageStatus renameFile(const char* oldPath,
                             const char* newPath) override;
    StorageStatus fileSize(const char* path, uint32_t& size) override;
    StorageStatus info(StorageInfo& out) override;
    StorageStatus health(StorageHealth& out) override;
    StorageStatus listFiles(const char* dir, FileEntry* entries,
                            uint8_t maxEntries,
                            uint8_t& count) override;
    StorageStatus readFileChunk(const char* path, uint32_t offset,
                                uint8_t* buf, uint32_t bufSize,
                                uint32_t& bytesRead) override;

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
};

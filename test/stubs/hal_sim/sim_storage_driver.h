/**
 * @file  sim_storage_driver.h
 * @brief Simulation storage driver — serves in-memory .ams files
 *        (StorageInterface).
 *
 * SimStorageDriver holds a registry of virtual files keyed by their
 * absolute path (e.g. "/missions/flight_test.ams").  File content is
 * provided as a null-terminated string at registration time and held
 * by pointer — no copies are made, so the caller must ensure the buffer
 * outlives the driver instance.
 *
 * Write/append/remove/rename operations are accepted and return OK without
 * performing any actual I/O, which satisfies the AMS engine's internal
 * resume-point and log housekeeping without requiring a real filesystem.
 *
 * Design constraints:
 *   - No dynamic allocation (PO10-3): registry is a fixed-size array.
 *   - Max registered files: SIM_STORAGE_MAX_FILES (compile-time).
 *
 * Thread safety: NOT thread-safe.  Intended for single-threaded SITL only
 *               (CERT-13).
 */
#pragma once

#include "hal/storage/storage_interface.h"
#include <cstdint>
#include <cstring>
#include <cstdio>

namespace ares
{
namespace sim
{

/// Maximum number of virtual files that can be registered (PO10-2).
static constexpr uint8_t SIM_STORAGE_MAX_FILES = 16U;

/**
 * Simulation storage driver.
 *
 * Usage:
 * @code
 * ares::sim::SimStorageDriver storage;
 * storage.registerFile("/missions/test.ams", kTestAmsContent);
 * // pass storage to MissionScriptEngine constructor
 * @endcode
 */
class SimStorageDriver final : public StorageInterface
{
public:
    SimStorageDriver() : mounted_(false), fileCount_(0U)
    {
        for (uint8_t i = 0U; i < SIM_STORAGE_MAX_FILES; i++)
        {
            entries_[i] = VirtualFile{};
        }
    }

    /**
     * Register a virtual file served from a read-only in-memory buffer.
     *
     * @param[in] path     Absolute path (e.g. "/missions/test.ams").
     * @param[in] content  Null-terminated file content.  The pointer must
     *                     remain valid for the lifetime of this driver.
     * @return true if the file was registered; false if the registry is full.
     */
    bool registerFile(const char* path, const char* content)
    {
        if (fileCount_ >= SIM_STORAGE_MAX_FILES) { return false; }
        if (path    == nullptr)                  { return false; }
        if (content == nullptr)                  { return false; }

        VirtualFile& vf = entries_[fileCount_];
        (void)snprintf(vf.path, sizeof(vf.path), "%s", path);
        vf.content    = content;
        vf.contentLen = static_cast<uint32_t>(strlen(content));
        fileCount_++;
        return true;
    }

    // ── StorageInterface ─────────────────────────────────────────────────────

    bool begin() override
    {
        mounted_ = true;
        return true;
    }

    void end() override
    {
        mounted_ = false;
    }

    StorageStatus exists(const char* path, bool& result) override
    {
        if (!mounted_) { return StorageStatus::NOT_READY; }
        result = (findEntry(path) != nullptr);
        return StorageStatus::OK;
    }

    StorageStatus readFile(const char* path,
                           uint8_t*    buf,
                           uint32_t    bufSize,
                           uint32_t&   bytesRead) override
    {
        if (!mounted_) { return StorageStatus::NOT_READY; }

        const VirtualFile* vf = findEntry(path);
        if (vf == nullptr)
        {
            bytesRead = 0U;
            return StorageStatus::NOT_FOUND;
        }

        const uint32_t copyLen = (vf->contentLen < bufSize)
                                  ? vf->contentLen
                                  : bufSize;
        (void)memcpy(buf, vf->content, copyLen);
        bytesRead = copyLen;
        return StorageStatus::OK;
    }

    // Write operations are accepted and silently discarded (log/resume stubs).

    StorageStatus writeFile(const char* /*path*/,
                            const uint8_t* /*data*/,
                            uint32_t /*len*/) override
    {
        return mounted_ ? StorageStatus::OK : StorageStatus::NOT_READY;
    }

    StorageStatus appendFile(const char* /*path*/,
                             const uint8_t* /*data*/,
                             uint32_t /*len*/) override
    {
        return mounted_ ? StorageStatus::OK : StorageStatus::NOT_READY;
    }

    StorageStatus removeFile(const char* /*path*/) override
    {
        return mounted_ ? StorageStatus::OK : StorageStatus::NOT_READY;
    }

    StorageStatus renameFile(const char* /*oldPath*/,
                             const char* /*newPath*/) override
    {
        return mounted_ ? StorageStatus::OK : StorageStatus::NOT_READY;
    }

    StorageStatus fileSize(const char* path, uint32_t& size) override
    {
        if (!mounted_) { return StorageStatus::NOT_READY; }

        const VirtualFile* vf = findEntry(path);
        if (vf == nullptr)
        {
            size = 0U;
            return StorageStatus::NOT_FOUND;
        }

        size = vf->contentLen;
        return StorageStatus::OK;
    }

    StorageStatus info(StorageInfo& out) override
    {
        out.totalBytes = 1024U * 1024U;   // 1 MiB simulated partition
        out.usedBytes  = 0U;
        out.freeBytes  = out.totalBytes;
        return mounted_ ? StorageStatus::OK : StorageStatus::NOT_READY;
    }

    StorageStatus health(StorageHealth& out) override
    {
        out.mounted           = mounted_;
        out.recoveryScanned   = 0U;
        out.recoveredFromBak  = 0U;
        out.removedTmp        = 0U;
        out.removedBak        = 0U;
        out.recoveryErrors    = 0U;
        return StorageStatus::OK;
    }

    StorageStatus listFiles(const char*  dir,
                            FileEntry*   entries,
                            uint8_t      maxEntries,
                            uint8_t&     count) override
    {
        if (!mounted_) { return StorageStatus::NOT_READY; }

        count = 0U;
        const uint32_t dirLen = static_cast<uint32_t>(strlen(dir));

        for (uint8_t i = 0U; i < fileCount_ && count < maxEntries; i++)
        {
            if (strncmp(entries_[i].path, dir, dirLen) == 0)
            {
                (void)snprintf(entries[count].name, sizeof(entries[count].name),
                               "%s", entries_[i].path);
                entries[count].size = entries_[i].contentLen;
                count++;
            }
        }
        return StorageStatus::OK;
    }

    StorageStatus readFileChunk(const char* path,
                                uint32_t    offset,
                                uint8_t*    buf,
                                uint32_t    bufSize,
                                uint32_t&   bytesRead) override
    {
        if (!mounted_) { return StorageStatus::NOT_READY; }

        const VirtualFile* vf = findEntry(path);
        if (vf == nullptr)
        {
            bytesRead = 0U;
            return StorageStatus::NOT_FOUND;
        }

        if (offset >= vf->contentLen)
        {
            bytesRead = 0U;
            return StorageStatus::OK;
        }

        const uint32_t available = vf->contentLen - offset;
        const uint32_t copyLen   = (available < bufSize) ? available : bufSize;
        (void)memcpy(buf, vf->content + offset, copyLen);
        bytesRead = copyLen;
        return StorageStatus::OK;
    }

private:
    struct VirtualFile
    {
        char     path[64]  = {};
        const char* content = nullptr;
        uint32_t contentLen = 0U;
    };

    /** Find a virtual file entry by exact path match. */
    const VirtualFile* findEntry(const char* path) const
    {
        if (path == nullptr) { return nullptr; }
        for (uint8_t i = 0U; i < fileCount_; i++)
        {
            if (strcmp(entries_[i].path, path) == 0)
            {
                return &entries_[i];
            }
        }
        return nullptr;
    }

    bool        mounted_;
    uint8_t     fileCount_;
    VirtualFile entries_[SIM_STORAGE_MAX_FILES];
};

} // namespace sim
} // namespace ares

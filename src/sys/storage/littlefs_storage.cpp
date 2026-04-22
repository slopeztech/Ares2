/**
 * @file  littlefs_storage.cpp
 * @brief Thread-safe LittleFS storage implementation.
 */

#include "sys/storage/littlefs_storage.h"
#include "ares_assert.h"
#include "debug/ares_log.h"

#include <LittleFS.h>
#include <cstring>

// ── Log tag ─────────────────────────────────────────────────
static constexpr const char* TAG = "STOR";
static constexpr const char* TMP_SUFFIX = ".tmp";
static constexpr const char* BAK_SUFFIX = ".bak";
static constexpr uint8_t MAX_RECOVERY_SCAN_FILES = 64;
static constexpr uint16_t COPY_CHUNK_SIZE = 512;

struct RecoveryCounters
{
    uint32_t scanned = 0;
    uint32_t restoredFromBak = 0;
    uint32_t removedTmp = 0;
    uint32_t removedBak = 0;
    uint32_t errors = 0;
};

// ── RAII mutex guard (CERT-18.1) ────────────────────────────

namespace {

/**
 * Scoped mutex lock with bounded timeout.
 * Releases the mutex automatically on destruction, preventing
 * lock leaks on early-return paths (CERT-18.4).
 */
class ScopedLock
{
public:
    explicit ScopedLock(SemaphoreHandle_t m, TickType_t timeout)
        : mutex_(m), acquired_(false)
    {
        ARES_ASSERT(m != nullptr);
        acquired_ = (xSemaphoreTake(m, timeout) == pdTRUE);
    }

    ~ScopedLock()
    {
        if (acquired_)
        {
            // FreeRTOS macro internals trigger cppcheck dangerousTypeCast at call site.
            // cppcheck-suppress dangerousTypeCast
            const BaseType_t giveOk = static_cast<BaseType_t>(xSemaphoreGive(mutex_));
            ARES_ASSERT(giveOk == pdTRUE);
        }
    }

    bool acquired() const { return acquired_; }

    ScopedLock(const ScopedLock&)            = delete;
    ScopedLock& operator=(const ScopedLock&) = delete;

private:
    SemaphoreHandle_t mutex_;
    bool acquired_;
};

/**
 * Ensure every parent directory in the given absolute path exists.
 * Example: "/missions/flight1.ams" creates "/missions" if absent.
 */
static bool ensureParentDirectories(const char* path)
{
    if (path == nullptr || path[0] != '/')
    {
        return false;
    }

    const uint32_t len = static_cast<uint32_t>(
        strnlen(path, static_cast<size_t>(ares::STORAGE_MAX_PATH)));
    if (len == 0U || len >= ares::STORAGE_MAX_PATH)
    {
        return false;
    }

    char dirPath[ares::STORAGE_MAX_PATH] = {};

    // Walk each slash and mkdir incremental parent paths.
    for (uint32_t i = 1U; i < len; i++)
    {
        if (path[i] != '/')
        {
            continue;
        }

        if (i >= sizeof(dirPath))
        {
            return false;
        }

        memcpy(dirPath, path, i);
        dirPath[i] = '\0';

        if (dirPath[0] == '\0' || strcmp(dirPath, "/") == 0)
        {
            continue;
        }

        if (!LittleFS.exists(dirPath))
        {
            if (!LittleFS.mkdir(dirPath))
            {
                return false;
            }
        }
    }

    return true;
}

static bool hasSuffix(const char* text, const char* suffix)
{
    if (text == nullptr || suffix == nullptr)
    {
        return false;
    }

    const uint32_t textLen = static_cast<uint32_t>(strlen(text));
    const uint32_t suffixLen = static_cast<uint32_t>(strlen(suffix));
    if (suffixLen == 0U || textLen < suffixLen)
    {
        return false;
    }

    return strcmp(&text[textLen - suffixLen], suffix) == 0;
}

/**
 * Ensure a top-level directory exists.
 * Uses mkdir first to avoid noisy exists/open checks on missing paths.
 */
static bool ensureDirectory(const char* dir)
{
    if (dir == nullptr || dir[0] != '/')
    {
        return false;
    }

    if (LittleFS.mkdir(dir))
    {
        return true;
    }

    // mkdir may fail when directory already exists; verify safely.
    File f = LittleFS.open(dir);
    const bool ok = f.isDirectory();
    f.close();
    return ok;
}

static bool buildSiblingPath(const char* basePath,
                             const char* suffix,
                             char* outPath,
                             uint32_t outSize)
{
    const int written = snprintf(outPath, outSize, "%s%s", basePath, suffix);
    return (written > 0 && static_cast<uint32_t>(written) < outSize);
}

static bool stripSuffixPath(const char* path,
                            const char* suffix,
                            char* outPath,
                            uint32_t outSize)
{
    const uint32_t pathLen = static_cast<uint32_t>(strlen(path));
    const uint32_t suffixLen = static_cast<uint32_t>(strlen(suffix));
    if (pathLen <= suffixLen || !hasSuffix(path, suffix))
    {
        return false;
    }

    const uint32_t newLen = pathLen - suffixLen;
    if (newLen >= outSize)
    {
        return false;
    }

    memcpy(outPath, path, newLen);
    outPath[newLen] = '\0';
    return true;
}

static void recoverTmpArtifact(const char* filePath, RecoveryCounters& counters)
{
    char basePath[ares::STORAGE_MAX_PATH] = {};
    if (!stripSuffixPath(filePath, TMP_SUFFIX, basePath, sizeof(basePath)))
    {
        return;
    }

    const bool hasBase = LittleFS.exists(basePath);
    char bakPath[ares::STORAGE_MAX_PATH] = {};
    const bool hasBakName = buildSiblingPath(basePath, BAK_SUFFIX,
                                             bakPath, sizeof(bakPath));
    const bool hasBak = hasBakName && LittleFS.exists(bakPath);

    if (!hasBase && hasBak)
    {
        if (LittleFS.rename(bakPath, basePath))
        {
            counters.restoredFromBak++;
        }
        else
        {
            counters.errors++;
        }
    }

    // Drop tmp always; it may be partially written.
    if (LittleFS.remove(filePath))
    {
        counters.removedTmp++;
    }
    else
    {
        counters.errors++;
    }
}

static void recoverBakArtifact(const char* filePath, RecoveryCounters& counters)
{
    char basePath[ares::STORAGE_MAX_PATH] = {};
    if (!stripSuffixPath(filePath, BAK_SUFFIX, basePath, sizeof(basePath)))
    {
        return;
    }

    if (!LittleFS.exists(basePath))
    {
        if (LittleFS.rename(filePath, basePath))
        {
            counters.restoredFromBak++;
        }
        else
        {
            counters.errors++;
        }
        return;
    }

    if (LittleFS.remove(filePath))
    {
        counters.removedBak++;
    }
    else
    {
        counters.errors++;
    }
}

static void recoverArtifactIfNeeded(const char* filePath, RecoveryCounters& counters)
{
    if (hasSuffix(filePath, TMP_SUFFIX))
    {
        recoverTmpArtifact(filePath, counters);
        return;
    }

    if (hasSuffix(filePath, BAK_SUFFIX))
    {
        recoverBakArtifact(filePath, counters);
    }
}

static RecoveryCounters recoverDirectoryTransactions(const char* dir)
{
    RecoveryCounters counters = {};

    if (dir == nullptr || !LittleFS.exists(dir))
    {
        return counters;
    }

    File root = LittleFS.open(dir);
    if (!root || !root.isDirectory())
    {
        counters.errors++;
        return counters;
    }

    uint8_t scanned = 0;
    File entry = root.openNextFile();
    while (entry && scanned < MAX_RECOVERY_SCAN_FILES)
    {
        scanned++;
        counters.scanned++;
        if (!entry.isDirectory())
        {
            const char* filePath = entry.path();
            recoverArtifactIfNeeded(filePath, counters);
        }

        entry = root.openNextFile();
    }

    return counters;
}

} // namespace

// ── Helpers ─────────────────────────────────────────────────

LittleFsStorage::LittleFsStorage() = default;

bool LittleFsStorage::validatePath(const char* path)
{
    if (path == nullptr)
    {
        return false;
    }

    // CERT-3.3: bounded length check on untrusted input
    const uint32_t len = static_cast<uint32_t>(
        strnlen(path, static_cast<size_t>(MAX_PATH_LEN)));

    // CERT-3.2: len must leave room for null terminator
    if (len == 0 || len >= MAX_PATH_LEN)
    {
        return false;
    }

    // LittleFS paths must be absolute
    if (path[0] != '/')
    {
        return false;
    }

    return true;
}

// ── Lifecycle ───────────────────────────────────────────────

bool LittleFsStorage::begin()
{
    // PO10-5: double-init guard
    ARES_ASSERT(mutex_ == nullptr);

    // PO10-3: static allocation — no heap
    mutex_ = xSemaphoreCreateMutexStatic(&mutexBuf_);
    ARES_ASSERT(mutex_ != nullptr);

    // Mount — formatOnFail=true ensures a clean filesystem after
    // flash corruption or first boot.
    if (!LittleFS.begin(true))
    {
        LOG_E(TAG, "mount failed");
        return false;
    }

    mounted_ = true;
    health_ = {};
    health_.mounted = true;

    // Create core directories once so API list operations don't trigger
    // repeated VFS warnings on first-run empty filesystems.
    if (!ensureDirectory(ares::LOG_DIR))
    {
        LOG_W(TAG, "failed to ensure dir: %s", ares::LOG_DIR);
    }
    if (!ensureDirectory(ares::MISSION_DIR))
    {
        LOG_W(TAG, "failed to ensure dir: %s", ares::MISSION_DIR);
    }

    // Recover interrupted transactional writes from previous reset.
    const RecoveryCounters logRc = recoverDirectoryTransactions(ares::LOG_DIR);
    const RecoveryCounters missionRc = recoverDirectoryTransactions(ares::MISSION_DIR);

    health_.recoveryScanned = logRc.scanned + missionRc.scanned;
    health_.recoveredFromBak = logRc.restoredFromBak + missionRc.restoredFromBak;
    health_.removedTmp = logRc.removedTmp + missionRc.removedTmp;
    health_.removedBak = logRc.removedBak + missionRc.removedBak;
    health_.recoveryErrors = logRc.errors + missionRc.errors;

    const uint32_t total = static_cast<uint32_t>(LittleFS.totalBytes());
    const uint32_t used  = static_cast<uint32_t>(LittleFS.usedBytes());
    const uint32_t free_ = (total >= used) ? (total - used) : 0;  // CERT-4

    LOG_I(TAG, "mounted — total=%u used=%u free=%u",
          total, used, free_);
    return true;
}

void LittleFsStorage::end()
{
    ScopedLock guard(mutex_, MUTEX_TIMEOUT);
    if (!guard.acquired())
    {
        LOG_W(TAG, "mutex timeout on end()");   // CERT-10.4
        return;
    }

    if (mounted_)
    {
        LittleFS.end();
        mounted_ = false;
        health_.mounted = false;
        LOG_I(TAG, "unmounted");
    }
}

// ── File operations ─────────────────────────────────────────

StorageStatus LittleFsStorage::exists(const char* path, bool& result)
{
    if (!validatePath(path))  { return StorageStatus::PATH_ERROR; }

    ScopedLock guard(mutex_, MUTEX_TIMEOUT);
    if (!guard.acquired())    { LOG_W(TAG, "mutex timeout"); return StorageStatus::BUSY; }
    if (!mounted_)            { return StorageStatus::NOT_READY; }

    result = LittleFS.exists(path);
    return StorageStatus::OK;
}

StorageStatus LittleFsStorage::readFile(const char* path,
                                         uint8_t* buf,
                                         uint32_t bufSize,
                                         uint32_t& bytesRead)
{
    bytesRead = 0;

    // CERT-1: validate all inputs before use
    if (!validatePath(path))               { return StorageStatus::PATH_ERROR; }
    if (buf == nullptr || bufSize == 0)    { return StorageStatus::ERROR; }

    ScopedLock guard(mutex_, MUTEX_TIMEOUT);
    if (!guard.acquired())                 { LOG_W(TAG, "mutex timeout"); return StorageStatus::BUSY; }
    if (!mounted_)                         { return StorageStatus::NOT_READY; }

    if (!LittleFS.exists(path))            { return StorageStatus::NOT_FOUND; }

    File f = LittleFS.open(path, "r");
    if (!f)
    {
        LOG_W(TAG, "open(r) failed: %s", path);
        return StorageStatus::ERROR;
    }

    // Read up to bufSize bytes — never exceed destination (CERT-2.2)
    const uint32_t fileLen = static_cast<uint32_t>(f.size());
    const uint32_t toRead  = (fileLen < bufSize) ? fileLen : bufSize;
    bytesRead = static_cast<uint32_t>(f.read(buf, toRead));
    f.close();

    ARES_ASSERT(bytesRead <= bufSize);  // PO10-5: post-condition
    return StorageStatus::OK;
}

StorageStatus LittleFsStorage::writeFile(const char* path,
                                          const uint8_t* data,
                                          uint32_t len)
{
    return writeInternal(path, data, len, "w");
}

StorageStatus LittleFsStorage::appendFile(const char* path,
                                           const uint8_t* data,
                                           uint32_t len)
{
    return writeInternal(path, data, len, "a");
}

StorageStatus LittleFsStorage::writeInternal(const char* path,
                                              const uint8_t* data,
                                              uint32_t len,
                                              const char* mode)
{
    // CERT-1: validate all inputs
    if (!validatePath(path))                 { return StorageStatus::PATH_ERROR; }
    if (data == nullptr && len > 0)          { return StorageStatus::ERROR; }
    if (len > MAX_FILE_SIZE)                 { return StorageStatus::NO_SPACE; }

    ScopedLock guard(mutex_, MUTEX_TIMEOUT);
    if (!guard.acquired())                   { LOG_W(TAG, "mutex timeout"); return StorageStatus::BUSY; }
    if (!mounted_)                           { return StorageStatus::NOT_READY; }

    // Ensure parent directories exist before opening file for write/append.
    if (!ensureParentDirectories(path))
    {
        LOG_W(TAG, "mkdir parent failed: %s", path);
        return StorageStatus::ERROR;
    }

    // Pre-flight space check for new writes
    if (mode[0] == 'w')
    {
        const uint32_t total     = static_cast<uint32_t>(LittleFS.totalBytes());
        const uint32_t used      = static_cast<uint32_t>(LittleFS.usedBytes());
        const uint32_t freeSpace = (total >= used) ? (total - used) : 0;  // CERT-4
        if (len > freeSpace)
        {
            LOG_W(TAG, "no space: need=%u free=%u", len, freeSpace);
            return StorageStatus::NO_SPACE;
        }
    }

    char tmpPath[ares::STORAGE_MAX_PATH] = {};
    char bakPath[ares::STORAGE_MAX_PATH] = {};
    if (!buildSiblingPath(path, TMP_SUFFIX, tmpPath, sizeof(tmpPath))
        || !buildSiblingPath(path, BAK_SUFFIX, bakPath, sizeof(bakPath)))
    {
        return StorageStatus::PATH_ERROR;
    }

    // Clean stale transactional artifacts before new write.
    if (LittleFS.exists(tmpPath))
    {
        (void)LittleFS.remove(tmpPath);
    }
    if (LittleFS.exists(bakPath) && LittleFS.exists(path))
    {
        (void)LittleFS.remove(bakPath);
    }

    File tmp = LittleFS.open(tmpPath, "w");
    if (!tmp)
    {
        LOG_W(TAG, "open(tmp) failed: %s", tmpPath);
        return StorageStatus::ERROR;
    }

    uint32_t written = 0;

    if (mode[0] == 'a' && LittleFS.exists(path))
    {
        // Transactional append: copy original file then append new bytes.
        File src = LittleFS.open(path, "r");
        if (!src)
        {
            tmp.close();
            (void)LittleFS.remove(tmpPath);
            return StorageStatus::ERROR;
        }

        uint8_t copyBuf[COPY_CHUNK_SIZE] = {};
        uint32_t copied = 0;
        const uint32_t originalSize = static_cast<uint32_t>(src.size());

        while (copied < originalSize)
        {
            const uint32_t remain = originalSize - copied;
            const uint32_t want = (remain < COPY_CHUNK_SIZE)
                                ? remain
                                : COPY_CHUNK_SIZE;
            const uint32_t rd = static_cast<uint32_t>(src.read(copyBuf, want));
            if (rd == 0U)
            {
                src.close();
                tmp.close();
                (void)LittleFS.remove(tmpPath);
                return StorageStatus::ERROR;
            }

            const uint32_t wr = static_cast<uint32_t>(tmp.write(copyBuf, rd));
            if (wr != rd)
            {
                src.close();
                tmp.close();
                (void)LittleFS.remove(tmpPath);
                return StorageStatus::ERROR;
            }
            copied += rd;
        }
        src.close();
    }

    if (len > 0)
    {
        written = static_cast<uint32_t>(tmp.write(data, len));
    }

    tmp.flush();
    tmp.close();

    if (written != len)
    {
        LOG_W(TAG, "short write: want=%u got=%u path=%s", len, written, path);
        (void)LittleFS.remove(tmpPath);
        return StorageStatus::ERROR;
    }

    // Atomic swap: path -> bak, tmp -> path, then remove bak.
    const bool hadOriginal = LittleFS.exists(path);
    if (hadOriginal)
    {
        if (LittleFS.exists(bakPath))
        {
            (void)LittleFS.remove(bakPath);
        }

        if (!LittleFS.rename(path, bakPath))
        {
            LOG_W(TAG, "rename to bak failed: %s -> %s", path, bakPath);
            (void)LittleFS.remove(tmpPath);
            return StorageStatus::ERROR;
        }
    }

    if (!LittleFS.rename(tmpPath, path))
    {
        LOG_W(TAG, "rename tmp failed: %s -> %s", tmpPath, path);

        if (hadOriginal && LittleFS.exists(bakPath))
        {
            (void)LittleFS.rename(bakPath, path);
        }
        (void)LittleFS.remove(tmpPath);
        return StorageStatus::ERROR;
    }

    if (LittleFS.exists(bakPath))
    {
        (void)LittleFS.remove(bakPath);
    }

    return StorageStatus::OK;
}

StorageStatus LittleFsStorage::removeFile(const char* path)
{
    if (!validatePath(path))  { return StorageStatus::PATH_ERROR; }

    ScopedLock guard(mutex_, MUTEX_TIMEOUT);
    if (!guard.acquired())    { LOG_W(TAG, "mutex timeout"); return StorageStatus::BUSY; }
    if (!mounted_)            { return StorageStatus::NOT_READY; }

    if (!LittleFS.exists(path))  { return StorageStatus::NOT_FOUND; }

    const bool ok = LittleFS.remove(path);
    if (!ok)
    {
        LOG_W(TAG, "remove failed: %s", path);
        return StorageStatus::ERROR;
    }

    return StorageStatus::OK;
}

StorageStatus LittleFsStorage::renameFile(const char* oldPath,
                                           const char* newPath)
{
    if (!validatePath(oldPath))  { return StorageStatus::PATH_ERROR; }
    if (!validatePath(newPath))  { return StorageStatus::PATH_ERROR; }

    ScopedLock guard(mutex_, MUTEX_TIMEOUT);
    if (!guard.acquired())       { LOG_W(TAG, "mutex timeout"); return StorageStatus::BUSY; }
    if (!mounted_)               { return StorageStatus::NOT_READY; }

    if (!LittleFS.exists(oldPath))  { return StorageStatus::NOT_FOUND; }

    const bool ok = LittleFS.rename(oldPath, newPath);
    if (!ok)
    {
        LOG_W(TAG, "rename failed: %s -> %s", oldPath, newPath);
        return StorageStatus::ERROR;
    }

    return StorageStatus::OK;
}

// ── Info ────────────────────────────────────────────────────

StorageStatus LittleFsStorage::fileSize(const char* path, uint32_t& size)
{
    size = 0;

    if (!validatePath(path))  { return StorageStatus::PATH_ERROR; }

    ScopedLock guard(mutex_, MUTEX_TIMEOUT);
    if (!guard.acquired())    { LOG_W(TAG, "mutex timeout"); return StorageStatus::BUSY; }
    if (!mounted_)            { return StorageStatus::NOT_READY; }

    if (!LittleFS.exists(path))  { return StorageStatus::NOT_FOUND; }

    File f = LittleFS.open(path, "r");
    if (!f)                      { return StorageStatus::ERROR; }

    size = static_cast<uint32_t>(f.size());
    f.close();

    return StorageStatus::OK;
}

StorageStatus LittleFsStorage::info(StorageInfo& out)
{
    out = {};  // MISRA-4: zero-init

    ScopedLock guard(mutex_, MUTEX_TIMEOUT);
    if (!guard.acquired())   { LOG_W(TAG, "mutex timeout"); return StorageStatus::BUSY; }
    if (!mounted_)           { return StorageStatus::NOT_READY; }

    out.totalBytes = static_cast<uint32_t>(LittleFS.totalBytes());
    out.usedBytes  = static_cast<uint32_t>(LittleFS.usedBytes());
    out.freeBytes  = (out.totalBytes >= out.usedBytes)
                   ? (out.totalBytes - out.usedBytes)
                   : 0;  // CERT-4: underflow guard

    return StorageStatus::OK;
}

StorageStatus LittleFsStorage::health(StorageHealth& out)
{
    out = {};

    ScopedLock guard(mutex_, MUTEX_TIMEOUT);
    if (!guard.acquired())   { LOG_W(TAG, "mutex timeout"); return StorageStatus::BUSY; }
    if (!mounted_)           { return StorageStatus::NOT_READY; }

    out = health_;
    out.mounted = mounted_;
    return StorageStatus::OK;
}

// ── Directory listing ───────────────────────────────────────

StorageStatus LittleFsStorage::listFiles(const char* dir,
                                          FileEntry* entries,
                                          uint8_t maxEntries,
                                          uint8_t& count)
{
    count = 0;

    if (dir == nullptr || entries == nullptr || maxEntries == 0)
    {
        return StorageStatus::PATH_ERROR;
    }

    ScopedLock guard(mutex_, MUTEX_TIMEOUT);
    if (!guard.acquired())   { LOG_W(TAG, "mutex timeout"); return StorageStatus::BUSY; }
    if (!mounted_)           { return StorageStatus::NOT_READY; }

    // Self-heal core directories if they are removed at runtime.
    const bool isCoreDir = (strcmp(dir, ares::LOG_DIR) == 0)
                        || (strcmp(dir, ares::MISSION_DIR) == 0);
    if (isCoreDir && !ensureDirectory(dir))
    {
        LOG_W(TAG, "ensure dir failed: %s", dir);
        return StorageStatus::ERROR;
    }

    // Avoid noisy VFS error logs from open() when directory doesn't exist.
    if (!LittleFS.exists(dir))
    {
        return StorageStatus::NOT_FOUND;
    }

    File root = LittleFS.open(dir);
    if (!root || !root.isDirectory())
    {
        return StorageStatus::NOT_FOUND;
    }

    File entry = root.openNextFile();
    while (entry && count < maxEntries)  // PO10-2: bounded
    {
        if (!entry.isDirectory())
        {
            strncpy(entries[count].name, entry.path(),
                    sizeof(entries[count].name) - 1U);
            entries[count].name[sizeof(entries[count].name) - 1U] = '\0';
            entries[count].size = static_cast<uint32_t>(entry.size());
            count++;
        }
        entry = root.openNextFile();
    }

    return StorageStatus::OK;
}

// ── Chunked read ────────────────────────────────────────────

StorageStatus LittleFsStorage::readFileChunk(const char* path,
                                              uint32_t offset,
                                              uint8_t* buf,
                                              uint32_t bufSize,
                                              uint32_t& bytesRead)
{
    bytesRead = 0;

    if (!validatePath(path))            { return StorageStatus::PATH_ERROR; }
    if (buf == nullptr || bufSize == 0) { return StorageStatus::ERROR; }

    ScopedLock guard(mutex_, MUTEX_TIMEOUT);
    if (!guard.acquired())              { LOG_W(TAG, "mutex timeout"); return StorageStatus::BUSY; }
    if (!mounted_)                      { return StorageStatus::NOT_READY; }

    if (!LittleFS.exists(path))         { return StorageStatus::NOT_FOUND; }

    File f = LittleFS.open(path, "r");
    if (!f)                             { return StorageStatus::ERROR; }

    const uint32_t fileLen = static_cast<uint32_t>(f.size());
    if (offset >= fileLen)
    {
        f.close();
        return StorageStatus::OK;  // EOF — 0 bytes read
    }

    if (!f.seek(offset))
    {
        f.close();
        LOG_W(TAG, "seek failed: %s offset=%u", path, offset);
        return StorageStatus::ERROR;
    }

    bytesRead = static_cast<uint32_t>(f.read(buf, bufSize));
    f.close();

    ARES_ASSERT(bytesRead <= bufSize);
    return StorageStatus::OK;
}

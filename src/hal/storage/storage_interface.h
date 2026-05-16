/**
 * @file  storage_interface.h
 * @brief Hardware-agnostic persistent storage interface (pure virtual).
 *
 * Provides a file-system abstraction for non-volatile storage.
 * Concrete implementations (e.g. LittleFsStorage) must be
 * **thread-safe** — any task may call any method at any time.
 *
 * Thread safety: Implementations MUST serialise all operations
 *                internally (CERT-13, RTOS-4).
 */
#pragma once

#include <cstdint>

/**
 * Status codes returned by StorageInterface operations.
 */
enum class StorageStatus : uint8_t
{
    OK         = 0,   ///< Operation succeeded.
    ERROR      = 1,   ///< Generic I/O or filesystem error.
    NOT_READY  = 2,   ///< Filesystem not mounted.
    NOT_FOUND  = 3,   ///< File does not exist.
    NO_SPACE   = 4,   ///< Insufficient free space.
    PATH_ERROR = 5,   ///< Invalid or too-long path.
    BUSY       = 6,   ///< Could not acquire lock within timeout.

    FIRST = OK,            // CERT-6.1 — range validation sentinels
    LAST  = BUSY
};

/**
 * Filesystem capacity snapshot.
 * All values in bytes.
 */
struct StorageInfo
{
    uint32_t totalBytes = 0;   ///< Total partition size in bytes.
    uint32_t usedBytes  = 0;   ///< Bytes currently occupied.
    uint32_t freeBytes  = 0;   ///< Bytes available for writing.
};

/**
 * Storage health snapshot for fault-tolerance diagnostics.
 */
struct StorageHealth
{
    bool mounted = false;                ///< Filesystem is mounted and ready.
    uint32_t recoveryScanned = 0;        ///< Files scanned during recovery.
    uint32_t recoveredFromBak = 0;       ///< Files restored from .bak copies.
    uint32_t removedTmp = 0;             ///< Incomplete .tmp files removed.
    uint32_t removedBak = 0;             ///< Old .bak files cleaned up.
    uint32_t recoveryErrors = 0;         ///< Errors during recovery process.
    uint32_t formatCount = 0;            ///< Times the partition was explicitly
                                         ///<   formatted since last power-on
                                         ///<   (see LittleFsStorage::begin()).
};

/**
 * Directory entry returned by listFiles().
 */
struct FileEntry
{
    char     name[64] = {};   ///< Full path (e.g. "/logs/flight_001.bin").
    uint32_t size      = 0;   ///< File size in bytes.
};

/**
 * Abstract persistent storage interface.
 *
 * Concrete implementations provide a thread-safe file I/O layer
 * over the underlying filesystem.  Application code accesses
 * storage through a reference to this interface, never through
 * the concrete type directly.
 */
class StorageInterface
{
public:
    virtual ~StorageInterface() = default;

    /**
     * Mount the filesystem.
     * @pre  Flash partition must be available.
     * @post Filesystem is mounted and ready for I/O on success.
     * @return true on success, false on mount failure.
     */
    virtual bool begin() = 0;

    /**
     * Unmount the filesystem.
     * @post All pending writes are flushed and the partition is released.
     */
    virtual void end() = 0;

    /**
     * Check whether a file exists.
     * @param[in]  path    Null-terminated absolute path (must start with '/').
     * @param[out] result  Set to true if the file exists, false otherwise.
     * @return Status code (see StorageStatus).
     */
    virtual StorageStatus exists(const char* path, bool& result) = 0;

    /**
     * Read an entire file into a caller-provided buffer.
     * @param[in]  path       Null-terminated absolute path.
     * @param[out] buf        Destination buffer (caller-owned).
     * @param[in]  bufSize    Size of @p buf in bytes.
     * @param[out] bytesRead  Actual bytes read (≤ bufSize).
     * @return Status code.  NOT_FOUND if the file does not exist.
     */
    virtual StorageStatus readFile(const char* path, uint8_t* buf,
                                   uint32_t bufSize,
                                   uint32_t& bytesRead) = 0;

    /**
     * Create or overwrite a file with the given data.
     * @param[in] path  Null-terminated absolute path.
     * @param[in] data  Source buffer (caller-owned, not modified).
     * @param[in] len   Number of bytes to write.
     * @return Status code.  NO_SPACE if the partition is full.
     */
    virtual StorageStatus writeFile(const char* path,
                                    const uint8_t* data,
                                    uint32_t len) = 0;

    /**
     * Append data to an existing file, or create it if absent.
     * @param[in] path  Null-terminated absolute path.
     * @param[in] data  Source buffer.
     * @param[in] len   Number of bytes to append.
     * @return Status code.
     */
    virtual StorageStatus appendFile(const char* path,
                                     const uint8_t* data,
                                     uint32_t len) = 0;

    /**
     * Delete a file.
     * @param[in] path  Null-terminated absolute path.
     * @return Status code.  NOT_FOUND if the file does not exist.
     */
    virtual StorageStatus removeFile(const char* path) = 0;

    /**
     * Rename or move a file.
     * @param[in] oldPath  Current path.
     * @param[in] newPath  Desired path.
     * @return Status code.  NOT_FOUND if @p oldPath does not exist.
     */
    virtual StorageStatus renameFile(const char* oldPath,
                                     const char* newPath) = 0;

    /**
     * Get the size of a file in bytes.
     * @param[in]  path  Null-terminated absolute path.
     * @param[out] size  File size in bytes.
     * @return Status code.  NOT_FOUND if the file does not exist.
     */
    virtual StorageStatus fileSize(const char* path, uint32_t& size) = 0;

    /**
     * Query filesystem capacity.
     * @param[out] out  Populated with total / used / free bytes.
     * @return Status code.
     */
    virtual StorageStatus info(StorageInfo& out) = 0;

    /**
     * Query storage health and recovery counters.
     * @param[out] out  Populated with mounted state and recovery metrics.
     * @return Status code.
     */
    virtual StorageStatus health(StorageHealth& out) = 0;

    /**
     * List files in a directory.
     * @param[in]  dir        Null-terminated directory path (e.g. "/logs").
     * @param[out] entries    Caller-owned array populated with results.
     * @param[in]  maxEntries Maximum entries to return.
     * @param[out] count      Actual number of entries written.
     * @return Status code.
     */
    virtual StorageStatus listFiles(const char* dir,
                                    FileEntry* entries,
                                    uint8_t maxEntries,
                                    uint8_t& count) = 0;

    /**
     * Read a chunk of a file at a given offset.
     * @param[in]  path       Null-terminated absolute path.
     * @param[in]  offset     Byte offset to start reading from.
     * @param[out] buf        Destination buffer (caller-owned).
     * @param[in]  bufSize    Size of @p buf in bytes.
     * @param[out] bytesRead  Actual bytes read (≤ bufSize).
     * @return Status code.
     */
    virtual StorageStatus readFileChunk(const char* path,
                                        uint32_t offset,
                                        uint8_t* buf,
                                        uint32_t bufSize,
                                        uint32_t& bytesRead) = 0;
};

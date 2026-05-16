/**
 * @file  storage_handler.cpp
 * @brief GET/DELETE /api/logs endpoint implementations (REST-12).
 *
 * Provides log-file listing, chunked download, and deletion
 * through the StorageInterface abstraction.
 *
 * Implements ApiServer::handleLogsList, handleLogDownload,
 * handleLogDelete, handleLogDeleteAll (declared in api_server.h).
 *
 * Security:
 *   - Only files under LOG_DIR are accessible (REST-12.5).
 *   - Filenames are validated for path traversal (REST-12.4).
 *   - Downloads are forbidden during flight (REST-6.2).
 */

#include "api/api_server.h"
#include "api/api_common.h"
#include "api/storage/storage_pure.h"
#include "debug/ares_log.h"

#include <ArduinoJson.h>
#include <WiFiClient.h>
#include <cinttypes>
#include <cstring>
#include <cstdio>

static constexpr const char* TAG = "API.LOG";

// API server processes one request at a time in a single task.
// Keep large scratch buffers off task stack to avoid canary trips.
static FileEntry g_logEntries[ares::MAX_LOG_FILES] = {};
static char g_jsonBuf[ares::API_MAX_RESPONSE_BODY] = {};

// ── Path safety (REST-11, REST-12.4) ────────────────────────
// Pure helpers delegated to storage_pure.h.
using ares::api::isValidLogFilename;
using ares::api::buildLogPath;

// ── Handlers ────────────────────────────────────────────────

void ApiServer::handleLogsList(WiFiClient& client)
{
    if (storage_ == nullptr)
    {
        sendError(client, 500, "storage not available");
        LOG_E(TAG, "GET /api/logs 500: no storage");
        return;
    }

    memset(g_logEntries, 0, sizeof(g_logEntries));
    uint8_t count = 0;

    const StorageStatus st = storage_->listFiles(
        ares::LOG_DIR, g_logEntries, ares::MAX_LOG_FILES, count);

    if (st == StorageStatus::NOT_FOUND)
    {
        // Directory doesn't exist yet — return empty list
        sendJson(client, 200, "[]", 2);
        LOG_D(TAG, "GET /api/logs 200: dir not found, empty list");
        return;
    }

    if (st != StorageStatus::OK)
    {
        sendError(client, 500, "storage error");
        LOG_E(TAG, "GET /api/logs 500: status=%u",
              static_cast<uint32_t>(st));
        return;
    }

    ARES_ASSERT(count <= ares::MAX_LOG_FILES);

    JsonDocument doc;
    JsonArray arr = doc.to<JsonArray>();

    for (uint8_t i = 0; i < count; i++)
    {
        JsonObject obj = arr.add<JsonObject>();
        obj["name"] = g_logEntries[i].name;
        obj["size"] = g_logEntries[i].size;
    }

    memset(g_jsonBuf, 0, sizeof(g_jsonBuf));
    const size_t len = serializeJson(doc, g_jsonBuf, sizeof(g_jsonBuf));
    ARES_ASSERT(len < sizeof(g_jsonBuf));
    sendJson(client, 200U, g_jsonBuf, static_cast<uint32_t>(len));
    LOG_D(TAG, "GET /api/logs 200: %u files", static_cast<uint32_t>(count));
}

// ── sendFileChunked ──────────────────────────────────────────────────────────

void ApiServer::sendFileChunked(WiFiClient& client,
                                const char* path,
                                const char* filename,
                                const char* contentType,
                                uint32_t    fileSize)
{
    client.printf("HTTP/1.1 200 OK\r\n");
    client.printf("Content-Type: %s\r\n", contentType);
    client.printf("Content-Length: %" PRIu32 "\r\n", fileSize);
    client.printf("Content-Disposition: attachment; filename=\"%s\"\r\n", filename);
    client.print(getCorsHeaders());
    client.print("Connection: close\r\n");
    client.print("\r\n");

    uint8_t  chunk[ares::DOWNLOAD_CHUNK_SIZE] = {};
    uint32_t offset = 0;
    while (offset < fileSize && client.connected())  // PO10-2: bounded
    {
        uint32_t bytesRead = 0;
        const StorageStatus st = storage_->readFileChunk(
            path, offset, chunk, ares::DOWNLOAD_CHUNK_SIZE, bytesRead);
        if (st != StorageStatus::OK || bytesRead == 0)
        {
            break;
        }
        ARES_ASSERT(bytesRead <= ares::DOWNLOAD_CHUNK_SIZE);
        ARES_ASSERT((offset + bytesRead) <= fileSize);
        client.write(chunk, bytesRead);
        offset += bytesRead;
    }
    LOG_I(TAG, "GET %s 200: %" PRIu32 " bytes sent", path, offset);
}

void ApiServer::handleLogDownload(WiFiClient& client,
                                   const char* filename)
{
    // REST-6.2: log file downloads are permitted during active flight.
    // LittleFS access is serialised by the storage mutex (each readFileChunk()
    // and appendFile() call takes and releases the mutex independently), so
    // concurrent read-during-append is safe and produces a consistent
    // snapshot of the file up to its size at download start.

    if (storage_ == nullptr)
    {
        sendError(client, 500, "storage not available");
        return;
    }

    // REST-12.4: validate filename
    if (!isValidLogFilename(filename))
    {
        sendError(client, 400, "invalid filename");
        LOG_W(TAG, "GET /api/logs 400: invalid filename");
        return;
    }

    char path[ares::STORAGE_MAX_PATH] = {};
    if (!buildLogPath(filename, path, sizeof(path)))
    {
        sendError(client, 400, "path too long");
        return;
    }

    uint32_t fileSize = 0;
    const StorageStatus st = storage_->fileSize(path, fileSize);
    if (st == StorageStatus::NOT_FOUND)
    {
        sendError(client, 404, "file not found");
        LOG_W(TAG, "GET %s 404", path);
        return;
    }
    if (st != StorageStatus::OK)
    {
        sendError(client, 500, "storage error");
        return;
    }

    // REST-12.2: send in fixed-size chunks
    sendFileChunked(client, path, filename, "application/octet-stream", fileSize);
}

void ApiServer::handleLogDelete(WiFiClient& client,
                                 const char* filename)
{
    // REST-6.1: log deletion only allowed in IDLE
    if (getMode() != ares::OperatingMode::IDLE)
    {
        sendError(client, 409, "log deletion requires idle mode");
        LOG_W(TAG, "DELETE /api/logs/%s 409: not idle", filename);
        return;
    }

    if (storage_ == nullptr)
    {
        sendError(client, 500, "storage not available");
        return;
    }

    if (!isValidLogFilename(filename))
    {
        sendError(client, 400, "invalid filename");
        LOG_W(TAG, "DELETE /api/logs 400: invalid filename");
        return;
    }

    char path[ares::STORAGE_MAX_PATH] = {};
    if (!buildLogPath(filename, path, sizeof(path)))
    {
        sendError(client, 400, "path too long");
        return;
    }

    const StorageStatus st = storage_->removeFile(path);

    if (st == StorageStatus::NOT_FOUND)
    {
        sendError(client, 404, "file not found");
        LOG_W(TAG, "DELETE %s 404", path);
        return;
    }
    if (st != StorageStatus::OK)
    {
        sendError(client, 500, "storage error");
        LOG_E(TAG, "DELETE %s 500: status=%u",
              path, static_cast<uint32_t>(st));
        return;
    }

    sendNoContent(client, 204);
    LOG_I(TAG, "DELETE %s 204", path);
}

void ApiServer::handleLogDeleteAll(WiFiClient& client)
{
    // REST-6.1: log deletion only allowed in IDLE
    if (getMode() != ares::OperatingMode::IDLE)
    {
        sendError(client, 409, "log deletion requires idle mode");
        LOG_W(TAG, "DELETE /api/logs 409: not idle");
        return;
    }

    if (storage_ == nullptr)
    {
        sendError(client, 500, "storage not available");
        return;
    }

    // List and delete each file
    memset(g_logEntries, 0, sizeof(g_logEntries));
    uint8_t count = 0;

    StorageStatus st = storage_->listFiles(
        ares::LOG_DIR, g_logEntries, ares::MAX_LOG_FILES, count);

    if (st == StorageStatus::NOT_FOUND)
    {
        sendNoContent(client, 204);
        return;
    }
    if (st != StorageStatus::OK)
    {
        sendError(client, 500, "storage error");
        return;
    }

    ARES_ASSERT(count <= ares::MAX_LOG_FILES);

    uint8_t deleted = 0;
    for (uint8_t i = 0; i < count; i++)
    {
        ARES_ASSERT(g_logEntries[i].name[0] == '/');
        if (storage_->removeFile(g_logEntries[i].name) == StorageStatus::OK)
        {
            deleted++;
        }
    }

    sendNoContent(client, 204);
    LOG_I(TAG, "DELETE /api/logs 204: %u/%u deleted",
          static_cast<uint32_t>(deleted),
          static_cast<uint32_t>(count));
}

void ApiServer::handleStorageHealth(WiFiClient& client)
{
    if (storage_ == nullptr)
    {
        sendError(client, 500, "storage not available");
        LOG_E(TAG, "GET /api/storage/health 500: no storage");
        return;
    }

    StorageInfo info = {};
    StorageHealth health = {};

    const StorageStatus infoSt = storage_->info(info);
    const StorageStatus healthSt = storage_->health(health);

    if (infoSt != StorageStatus::OK || healthSt != StorageStatus::OK)
    {
        JsonDocument err;
        err["ok"] = false;
        err["info_status"] = static_cast<uint32_t>(infoSt);
        err["health_status"] = static_cast<uint32_t>(healthSt);
        memset(g_jsonBuf, 0, sizeof(g_jsonBuf));
        const uint32_t errLen = serializeJson(err, g_jsonBuf, sizeof(g_jsonBuf));
        sendJson(client, 500, g_jsonBuf, errLen);  // REST-2.3: 503 not in allowed set
        LOG_W(TAG, "GET /api/storage/health 500: info=%u health=%u",
              static_cast<uint32_t>(infoSt),
              static_cast<uint32_t>(healthSt));
        return;
    }

    JsonDocument doc;
    doc["ok"] = true;
    doc["mounted"] = health.mounted;
    doc["total"] = info.totalBytes;
    doc["used"] = info.usedBytes;
    doc["free"] = info.freeBytes;
    doc["recovery_scanned"] = health.recoveryScanned;
    doc["recovered_from_bak"] = health.recoveredFromBak;
    doc["removed_tmp"] = health.removedTmp;
    doc["removed_bak"] = health.removedBak;
    doc["recovery_errors"] = health.recoveryErrors;
    doc["format_count"] = health.formatCount;

    memset(g_jsonBuf, 0, sizeof(g_jsonBuf));
    const size_t len = serializeJson(doc, g_jsonBuf, sizeof(g_jsonBuf));
    ARES_ASSERT(len < sizeof(g_jsonBuf));
    sendJson(client, 200U, g_jsonBuf, static_cast<uint32_t>(len));
    LOG_D(TAG, "GET /api/storage/health 200");
}

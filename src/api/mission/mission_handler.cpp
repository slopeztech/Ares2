/**
 * @file  mission_handler.cpp
 * @brief AMS mission endpoints.
 *
 * Endpoints:
 *   GET  /api/mission              -> active script snapshot
 *   GET  /api/missions             -> list available .ams files
 *   POST /api/mission/activate     -> {"file":"flight.ams"}
 *   POST /api/mission/deactivate   -> {}
 *   POST /api/mission/command      -> {"command":"LAUNCH"}
 */

#include "api/api_server.h"
#include "api/api_common.h"
#include "debug/ares_log.h"

#include <ArduinoJson.h>
#include <WiFiClient.h>
#include <cinttypes>
#include <cstring>
#include <cstdio>

static constexpr const char* TAG = "API.MSN";
static constexpr uint8_t MAX_COMMAND_TEXT = 16;

// API server handles one request at a time; keep large temporaries off stack.
static FileEntry g_missionEntries[ares::MAX_LOG_FILES] = {};
static char g_missionJsonBuf[ares::API_MAX_RESPONSE_BODY] = {};

static bool isValidMissionFilename(const char* name)
{
    if (name == nullptr || name[0] == '\0' || name[0] == '.')
    {
        return false;
    }

    const uint32_t len = static_cast<uint32_t>(
        strnlen(name, ares::MISSION_FILENAME_MAX + 1U));
    if (len == 0U || len > ares::MISSION_FILENAME_MAX)
    {
        return false;
    }

    for (uint32_t i = 0; i < len; i++)
    {
        const char c = name[i];
        const bool ok = (c >= 'a' && c <= 'z')
                     || (c >= 'A' && c <= 'Z')
                     || (c >= '0' && c <= '9')
                     || c == '_' || c == '-' || c == '.';
        if (!ok)
        {
            return false;
        }
    }

    return strstr(name, "..") == nullptr;
}

static bool buildMissionPath(const char* filename, char* out, uint32_t outSize)
{
    const int32_t written = static_cast<int32_t>(snprintf(out, outSize, "%s/%s",
                                 ares::MISSION_DIR, filename));
    return (written > 0 && static_cast<uint32_t>(written) < outSize);
}

static const char* toStatusText(ares::ams::EngineStatus st)
{
    switch (st)
    {
    case ares::ams::EngineStatus::IDLE:     return "idle";
    case ares::ams::EngineStatus::LOADED:   return "loaded";
    case ares::ams::EngineStatus::RUNNING:  return "running";
    case ares::ams::EngineStatus::COMPLETE: return "complete";
    case ares::ams::EngineStatus::ERROR:    return "error";
    default:                                return "unknown";
    }
}

void ApiServer::handleMissionStatus(WiFiClient& client)
{
    if (mission_ == nullptr)
    {
        sendError(client, 500, "mission engine unavailable");
        return;
    }

    ares::ams::EngineSnapshot snap = {};
    mission_->getSnapshot(snap);

    JsonDocument doc;
    doc["status"] = toStatusText(snap.status);
    doc["activeFile"] = snap.activeFile;
    doc["state"] = snap.stateName;
    doc["error"] = snap.lastError;

    memset(g_missionJsonBuf, 0, sizeof(g_missionJsonBuf));
    const size_t len = serializeJson(doc, g_missionJsonBuf,
                                       sizeof(g_missionJsonBuf));
    ARES_ASSERT(len < sizeof(g_missionJsonBuf));
    sendJson(client, 200U, g_missionJsonBuf, static_cast<uint32_t>(len));
}

void ApiServer::handleMissionList(WiFiClient& client)
{
    if (mission_ == nullptr)
    {
        sendError(client, 500, "mission engine unavailable");
        return;
    }

    memset(g_missionEntries, 0, sizeof(g_missionEntries));
    uint8_t count = 0;

    if (!mission_->listScripts(g_missionEntries, ares::MAX_LOG_FILES, count))
    {
        sendError(client, 500, "failed to list mission scripts");
        return;
    }

    ARES_ASSERT(count <= ares::MAX_LOG_FILES);

    JsonDocument doc;
    JsonArray arr = doc.to<JsonArray>();

    for (uint8_t i = 0; i < count; i++)
    {
        JsonObject item = arr.add<JsonObject>();
        item["name"] = g_missionEntries[i].name;
        item["size"] = g_missionEntries[i].size;
    }

    memset(g_missionJsonBuf, 0, sizeof(g_missionJsonBuf));
    const size_t len = serializeJson(doc, g_missionJsonBuf,
                                       sizeof(g_missionJsonBuf));
    ARES_ASSERT(len < sizeof(g_missionJsonBuf));
    sendJson(client, 200U, g_missionJsonBuf, static_cast<uint32_t>(len));
}

void ApiServer::handleMissionDownload(WiFiClient& client,
                                      const char* filename)
{
    if (storage_ == nullptr)
    {
        sendError(client, 500, "storage not available");
        return;
    }

    if (!isValidMissionFilename(filename))
    {
        sendError(client, 400, "invalid filename");
        return;
    }

    char path[ares::STORAGE_MAX_PATH] = {};
    if (!buildMissionPath(filename, path, sizeof(path)))
    {
        sendError(client, 400, "path too long");
        return;
    }

    uint32_t fileSize = 0;
    const StorageStatus st = storage_->fileSize(path, fileSize);
    if (st == StorageStatus::NOT_FOUND)
    {
        sendError(client, 404, "file not found");
        return;
    }
    if (st != StorageStatus::OK)
    {
        sendError(client, 500, "storage error");
        return;
    }

    sendFileChunked(client, path, filename, "text/plain", fileSize);
}

void ApiServer::handleMissionUpload(WiFiClient& client,
                                    const char* filename,
                                    const char* body,
                                    uint32_t bodyLen)
{
    if (storage_ == nullptr)
    {
        sendError(client, 500, "storage not available");
        return;
    }

    if (!isValidMissionFilename(filename))
    {
        sendError(client, 400, "invalid filename");
        return;
    }

    if (body == nullptr || bodyLen == 0U)
    {
        sendError(client, 400, "empty file body");
        return;
    }

    if (bodyLen > ares::AMS_MAX_SCRIPT_BYTES)
    {
        sendError(client, 413, "script too large");
        return;
    }

    char path[ares::STORAGE_MAX_PATH] = {};
    if (!buildMissionPath(filename, path, sizeof(path)))
    {
        sendError(client, 400, "path too long");
        return;
    }

    const StorageStatus st = storage_->writeFile(path,
                                                 reinterpret_cast<const uint8_t*>(body),
                                                 bodyLen);
    if (st == StorageStatus::NO_SPACE)
    {
        sendError(client, 409, "no storage space");
        return;
    }
    if (st != StorageStatus::OK)
    {
        sendError(client, 500, "storage write error");
        return;
    }

    sendNoContent(client, 204);
}

void ApiServer::handleMissionDelete(WiFiClient& client,
                                    const char* filename)
{
    if (storage_ == nullptr)
    {
        sendError(client, 500, "storage not available");
        return;
    }

    if (!isValidMissionFilename(filename))
    {
        sendError(client, 400, "invalid filename");
        return;
    }

    char path[ares::STORAGE_MAX_PATH] = {};
    if (!buildMissionPath(filename, path, sizeof(path)))
    {
        sendError(client, 400, "path too long");
        return;
    }

    const StorageStatus st = storage_->removeFile(path);
    if (st == StorageStatus::NOT_FOUND)
    {
        sendError(client, 404, "file not found");
        return;
    }
    if (st != StorageStatus::OK)
    {
        sendError(client, 500, "storage error");
        return;
    }

    sendNoContent(client, 204);
}

void ApiServer::handleMissionActivate(WiFiClient& client,
                                      const char* body,
                                      uint32_t bodyLen)
{
    if (mission_ == nullptr)
    {
        sendError(client, 500, "mission engine unavailable");
        return;
    }

    JsonDocument doc;
    const DeserializationError err = deserializeJson(doc, body, bodyLen);
    if (err != DeserializationError::Ok)
    {
        sendError(client, 400, "invalid JSON");
        return;
    }

    if (!doc["file"].is<const char*>())
    {
        sendError(client, 400, "file field required (string)");
        return;
    }

    const char* file = nullptr;
    file = doc["file"].as<const char*>();
    const uint32_t fileLen = static_cast<uint32_t>(
        strnlen(file, ares::MISSION_FILENAME_MAX + 1U));
    if (fileLen == 0U || fileLen > ares::MISSION_FILENAME_MAX)
    {
        sendError(client, 400, "invalid file name length");
        return;
    }

    if (!mission_->activate(file))
    {
        sendError(client, 409, "failed to activate mission script");
        LOG_W(TAG, "POST /api/mission/activate 409: file=%s", file);
        return;
    }

    // Script is loaded and paused (LOADED state). Execution starts on POST /api/arm.
    // Do NOT set FLIGHT mode here — that happens on arm.
    handleMissionStatus(client);
    LOG_I(TAG, "POST /api/mission/activate 200: %s (awaiting arm)", file);
}

void ApiServer::handleMissionDeactivate(WiFiClient& client)
{
    if (mission_ == nullptr)
    {
        sendError(client, 500, "mission engine unavailable");
        return;
    }

    mission_->deactivate();
    setMode(ares::OperatingMode::IDLE);    // back to solid green
    handleMissionStatus(client);
    LOG_I(TAG, "POST /api/mission/deactivate 200");
}

void ApiServer::handleMissionCommand(WiFiClient& client,
                                     const char* body,
                                     uint32_t bodyLen)
{
    if (mission_ == nullptr)
    {
        sendError(client, 500, "mission engine unavailable");
        return;
    }

    JsonDocument doc;
    const DeserializationError err = deserializeJson(doc, body, bodyLen);
    if (err != DeserializationError::Ok)
    {
        sendError(client, 400, "invalid JSON");
        return;
    }

    if (!doc["command"].is<const char*>())
    {
        sendError(client, 400, "command field required (string)");
        return;
    }

    const char* command = nullptr;
    command = doc["command"].as<const char*>();
    const uint32_t cmdLen = static_cast<uint32_t>(
        strnlen(command, MAX_COMMAND_TEXT + 1U));
    if (cmdLen == 0U || cmdLen > MAX_COMMAND_TEXT)
    {
        sendError(client, 400, "invalid command length");
        return;
    }

    if (!mission_->injectTcCommand(command))
    {
        sendError(client, 400, "unsupported command");
        return;
    }

    sendNoContent(client, 204);
    LOG_I(TAG, "POST /api/mission/command 204: %s", command);
}

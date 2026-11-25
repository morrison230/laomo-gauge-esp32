/* Read back closed trip files over the existing USB serial port using SDX. */
#include <SD.h>
#include <mbedtls/sha256.h>
#include "v2.h"

enum class TripExportMode { idle, listing, reading };

static TripExportMode exportMode = TripExportMode::idle;
static File exportFile;
static mbedtls_sha256_context exportSha;
static size_t exportOffset;
static size_t exportSize;
static unsigned exportEntries;
static constexpr size_t EXPORT_CHUNK = 256;
static constexpr char EXPORT_HEX[] = "0123456789abcdef";

static bool isTripName(const char *name)
{
    if (strlen(name) != 14 || strncmp(name, "trip-", 5) || strcmp(name + 8, ".jsonl")) return false;
    for (int i = 5; i < 8; ++i) if (name[i] < '0' || name[i] > '9') return false;
    return strncmp(name + 5, "000", 3) != 0;
}

static void closeTripExport()
{
    exportFile.close();
    if (exportMode == TripExportMode::reading) mbedtls_sha256_free(&exportSha);
    exportMode = TripExportMode::idle;
}

void tripExportCommand(const char *args)
{
    while (*args == ' ') ++args;
    if (!strcmp(args, "PING")) { Serial.println(F("SDX PONG R5")); return; }
    if (!strcmp(args, "INFO")) {
        Serial.printf("SDX INFO mounted=%u recording=%u\n", (unsigned)storeOk(),
                      (unsigned)storeIsRecording());
        return;
    }
    if (!strcmp(args, "CANCEL")) {
        if (exportMode != TripExportMode::idle) closeTripExport();
        Serial.println(F("SDX ERR CANCEL"));
        return;
    }
    const bool list = !strcmp(args, "LS " TRIPS_VFS);
    const bool get = !strncmp(args, "GET " TRIPS_VFS "/", sizeof("GET " TRIPS_VFS "/") - 1);
    if (!list && !get) { Serial.println(F("SDX ERR ARG")); return; }
    if (exportMode != TripExportMode::idle || storeIsRecording()) {
        Serial.println(F("SDX ERR BUSY"));
        return;
    }
    if (!storeOk()) { Serial.println(F("SDX ERR MOUNT")); return; }
    if (list) {
        exportFile = SD.open(TRIPS_VFS, FILE_READ);
        if (!exportFile || !exportFile.isDirectory()) {
            exportFile.close();
            Serial.println(F("SDX ERR OPEN"));
            return;
        }
        exportEntries = 0;
        exportMode = TripExportMode::listing;
        Serial.println(F("SDX LIST " TRIPS_VFS));
        return;
    }
    const char *path = args + 4;
    const char *name = path + sizeof(TRIPS_VFS "/") - 1;
    if (!isTripName(name)) { Serial.println(F("SDX ERR PATH")); return; }
    exportFile = SD.open(path, FILE_READ);
    if (!exportFile || exportFile.isDirectory()) {
        exportFile.close();
        Serial.println(F("SDX ERR OPEN"));
        return;
    }
    exportSize = exportFile.size();
    exportOffset = 0;
    mbedtls_sha256_init(&exportSha);
    mbedtls_sha256_starts(&exportSha, 0);
    exportMode = TripExportMode::reading;
    Serial.printf("SDX BEGIN %lu\n", (unsigned long)exportSize);
}

void tripExportTick()
{
    if (exportMode == TripExportMode::idle) return;
    if (exportMode == TripExportMode::listing) {
        File entry = exportFile.openNextFile(FILE_READ);
        if (!entry) {
            closeTripExport();
            Serial.printf("SDX LISTEND %u\n", exportEntries);
            return;
        }
        const char *name = entry.name();
        if (!entry.isDirectory() && isTripName(name)) {
            Serial.printf("SDX ENTRY %lu\tf\t%s\n", (unsigned long)entry.size(), name);
            ++exportEntries;
        }
        entry.close();
        return;
    }
    if (exportOffset == exportSize) {
        unsigned char digest[32];
        char hex[65];
        mbedtls_sha256_finish(&exportSha, digest);
        for (size_t i = 0; i < sizeof(digest); ++i) {
            hex[2 * i] = EXPORT_HEX[digest[i] >> 4];
            hex[2 * i + 1] = EXPORT_HEX[digest[i] & 15];
        }
        hex[64] = 0;
        closeTripExport();
        Serial.printf("SDX END %lu %s\n", (unsigned long)exportOffset, hex);
        return;
    }
    uint8_t bytes[EXPORT_CHUNK];
    const size_t remaining = exportSize - exportOffset;
    const size_t requested = remaining < sizeof(bytes) ? remaining : sizeof(bytes);
    const size_t count = exportFile.read(bytes, requested);
    if (!count || count > requested) {
        closeTripExport();
        Serial.println(F("SDX ERR READ"));
        return;
    }
    mbedtls_sha256_update(&exportSha, bytes, count);
    char frame[EXPORT_CHUNK * 2 + 48];
    int len = snprintf(frame, sizeof(frame), "SDX DATA %lu ", (unsigned long)exportOffset);
    for (size_t i = 0; i < count; ++i) {
        frame[len++] = EXPORT_HEX[bytes[i] >> 4];
        frame[len++] = EXPORT_HEX[bytes[i] & 15];
    }
    frame[len++] = '\n';
    Serial.write((const uint8_t *)frame, (size_t)len);
    exportOffset += count;
}

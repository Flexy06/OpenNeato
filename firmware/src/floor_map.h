#ifndef FLOOR_MAP_H
#define FLOOR_MAP_H

#include <Arduino.h>
#include <SPIFFS.h>
#include <cmath>
#include <memory>
#include <vector>
#include "config.h"
#include "data_logger.h"

// Floor map: a pinned cleaning session path plus user-drawn rooms and no-go
// zones, all in the map frame of that session (millimetres, dock = origin).
//
// The frontend builds the map (decimates a recorded history session, lets the
// user draw polygons) and uploads it as a compact line-based text document.
// The firmware stores it verbatim at FLOORMAP_FILE and parses it only when a
// guided clean starts, so it costs no RAM while idle. The pinned path is a
// copy, so it survives history rotation (enforceLimits) of the source session.
//
// Format "OFM1" (one record per line, fields separated by single spaces,
// points as "x,y" integer millimetres):
//
//   OFM1
//   S <source session filename>            (optional, informational)
//   P x,y x,y x,y ...                      (path points, may repeat; order kept)
//   R <id> <percent-encoded name> x,y ...  (room polygon, 3..FLOORMAP_MAX_VERTICES)
//   N <id> x,y x,y ...                     (no-go polygon)
//
// Unknown record types are ignored so newer frontends can add data without
// breaking older firmware.

struct MapPoint {
    int16_t x = 0; // mm
    int16_t y = 0; // mm
};

struct MapZone {
    int id = 0;
    String name; // Percent-encoded as stored (display only, decoded by the frontend)
    std::vector<MapPoint> poly;
};

// Localization map ("likelihood field") built by the frontend from a mapping
// run: one byte per cell, 255 on walls/furniture seen by the LIDAR, falling
// off with distance. Scan matching scores a pose by summing the field under
// the scan points. Binary layout (little endian):
//   "OFL1" | int32 originX mm | int32 originY mm | uint16 cell mm | uint16 w | uint16 h
//   | int8 lidarSign (+1 CCW / -1 CW) | int8 reserved | int16 lidarAngleOffset (0.1°)
//   | int16 lidarOffsetX mm | int16 reserved | w*h bytes (row-major, y up)
struct LocField {
    int originX = 0;
    int originY = 0;
    int cell = 50;
    int w = 0;
    int h = 0;
    int lidarSign = 1;
    float lidarAngleOffsetRad = 0.0f;
    int lidarOffsetX = 0;
    std::vector<uint8_t> data;

    bool valid() const { return w > 0 && h > 0 && data.size() == static_cast<size_t>(w) * static_cast<size_t>(h); }
    // Field value at a world point (0 outside)
    bool inside(float x, float y) const {
        return x >= static_cast<float>(originX) && y >= static_cast<float>(originY) &&
               x < static_cast<float>(originX + w * cell) && y < static_cast<float>(originY + h * cell);
    }
    uint8_t at(float x, float y) const {
        int cx = static_cast<int>(floorf((x - static_cast<float>(originX)) / static_cast<float>(cell)));
        int cy = static_cast<int>(floorf((y - static_cast<float>(originY)) / static_cast<float>(cell)));
        if (cx < 0 || cy < 0 || cx >= w || cy >= h)
            return 0;
        return data[static_cast<size_t>(cy) * static_cast<size_t>(w) + static_cast<size_t>(cx)];
    }
};

struct FloorMapData {
    String source;
    std::vector<MapPoint> path;
    std::vector<MapZone> rooms;
    std::vector<MapZone> nogos;

    const MapZone *findRoom(int id) const;
};

// -- Geometry helpers (map frame, millimetres) --------------------------------

namespace mapgeo {
    // Distance from point p to segment a-b
    float distPointSegment(float px, float py, float ax, float ay, float bx, float by);
    // Even-odd point-in-polygon test
    bool pointInPolygon(float x, float y, const std::vector<MapPoint>& poly);
    // Shortest distance from a point to the polygon outline
    float distToPolygonEdge(float x, float y, const std::vector<MapPoint>& poly);
    // True if the point lies inside the polygon or within `margin` of its outline
    bool pointNearPolygon(float x, float y, const std::vector<MapPoint>& poly, float margin);
    // True if the straight segment a→b passes inside or within `margin` of the polygon
    bool segmentNearPolygon(float ax, float ay, float bx, float by, const std::vector<MapPoint>& poly, float margin);
} // namespace mapgeo

class FloorMap {
public:
    explicit FloorMap(DataLogger& logger);

    void begin();

    bool hasMap() const;
    bool hasField() const;
    size_t fileSize() const;

    // Streaming reader for GET /api/floormap (nullptr if no map stored)
    std::shared_ptr<LogReader> read();

    // Remove the stored map. Returns false if none existed.
    bool remove();

    // -- Upload (PUT /api/floormap, chunked body) ------------------------------
    // beginUpload() opens a temp file; writeChunk() appends; endUpload()
    // validates the full document by parsing it and atomically replaces the
    // stored map. Any failure leaves the previous map untouched.
    // target: false = map document, true = localization field
    bool beginUpload(size_t totalBytes, bool field = false);
    bool writeChunk(const uint8_t *data, size_t len);
    bool endUpload();
    void abortUpload();
    bool isUploading() const { return uploading; }
    const String& getUploadError() const { return uploadError; }

    // Parse the stored map into memory. Returns false (with `err`) if missing or invalid.
    bool load(FloorMapData& out, String& err) const;

    // Load the localization field (false if none / invalid)
    bool loadField(LocField& out) const;

    // Streaming reader for GET /api/floormap/field (nullptr if none)
    std::shared_ptr<LogReader> readField();

private:
    DataLogger& dataLogger;

    bool uploading = false;
    bool uploadingField = false;
    File uploadFile;
    size_t uploadReceived = 0;
    String uploadError;

    static bool parseFile(const char *path, FloorMapData& out, String& err);
    static bool parseLine(const String& line, FloorMapData& out, String& err);
    static bool parsePoints(const String& line, int start, std::vector<MapPoint>& out, size_t maxPoints);
};

#endif // FLOOR_MAP_H

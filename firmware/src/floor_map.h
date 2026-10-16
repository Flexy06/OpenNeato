#ifndef FLOOR_MAP_H
#define FLOOR_MAP_H

#include <Arduino.h>
#include <SPIFFS.h>
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
    size_t fileSize() const;

    // Streaming reader for GET /api/floormap (nullptr if no map stored)
    std::shared_ptr<LogReader> read();

    // Remove the stored map. Returns false if none existed.
    bool remove();

    // -- Upload (PUT /api/floormap, chunked body) ------------------------------
    // beginUpload() opens a temp file; writeChunk() appends; endUpload()
    // validates the full document by parsing it and atomically replaces the
    // stored map. Any failure leaves the previous map untouched.
    bool beginUpload(size_t totalBytes);
    bool writeChunk(const uint8_t *data, size_t len);
    bool endUpload();
    void abortUpload();
    bool isUploading() const { return uploading; }
    const String& getUploadError() const { return uploadError; }

    // Parse the stored map into memory. Returns false (with `err`) if missing or invalid.
    bool load(FloorMapData& out, String& err) const;

private:
    DataLogger& dataLogger;

    bool uploading = false;
    File uploadFile;
    size_t uploadReceived = 0;
    String uploadError;

    static bool parseFile(const char *path, FloorMapData& out, String& err);
    static bool parseLine(const String& line, FloorMapData& out, String& err);
    static bool parsePoints(const String& line, int start, std::vector<MapPoint>& out, size_t maxPoints);
};

#endif // FLOOR_MAP_H

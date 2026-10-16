#include "floor_map.h"
#include <cmath>

// -- FloorMapData ---------------------------------------------------------------

const MapZone *FloorMapData::findRoom(int id) const {
    for (const auto& r: rooms) {
        if (r.id == id)
            return &r;
    }
    return nullptr;
}

// -- Geometry -------------------------------------------------------------------

namespace mapgeo {

    bool pointInPolygon(float x, float y, const std::vector<MapPoint>& poly) {
        bool inside = false;
        size_t n = poly.size();
        if (n < 3)
            return false;
        for (size_t i = 0, j = n - 1; i < n; j = i++) {
            float xi = poly[i].x;
            float yi = poly[i].y;
            float xj = poly[j].x;
            float yj = poly[j].y;
            bool crosses = (yi > y) != (yj > y);
            if (crosses && x < (xj - xi) * (y - yi) / (yj - yi) + xi)
                inside = !inside;
        }
        return inside;
    }

    float distPointSegment(float px, float py, float ax, float ay, float bx, float by) {
        float dx = bx - ax;
        float dy = by - ay;
        float len2 = dx * dx + dy * dy;
        float t = 0.0f;
        if (len2 > 0.0f) {
            t = ((px - ax) * dx + (py - ay) * dy) / len2;
            t = t < 0.0f ? 0.0f : (t > 1.0f ? 1.0f : t);
        }
        float cx = ax + t * dx - px;
        float cy = ay + t * dy - py;
        return sqrtf(cx * cx + cy * cy);
    }

    float distToPolygonEdge(float x, float y, const std::vector<MapPoint>& poly) {
        float best = 1e9f;
        size_t n = poly.size();
        for (size_t i = 0, j = n - 1; i < n; j = i++) {
            float d = distPointSegment(x, y, poly[j].x, poly[j].y, poly[i].x, poly[i].y);
            if (d < best)
                best = d;
        }
        return best;
    }

    bool pointNearPolygon(float x, float y, const std::vector<MapPoint>& poly, float margin) {
        if (poly.size() < 3)
            return false;
        return pointInPolygon(x, y, poly) || distToPolygonEdge(x, y, poly) <= margin;
    }

    // Sampled test — cheap and robust for the small polygons drawn in the UI.
    // Sample spacing is a fraction of the margin so a thin corner can't slip
    // between two samples.
    bool segmentNearPolygon(float ax, float ay, float bx, float by, const std::vector<MapPoint>& poly, float margin) {
        if (poly.size() < 3)
            return false;
        float dx = bx - ax;
        float dy = by - ay;
        float len = sqrtf(dx * dx + dy * dy);
        float step = margin > 40.0f ? margin / 2.0f : 20.0f;
        int samples = static_cast<int>(len / step) + 1;
        for (int i = 0; i <= samples; i++) {
            float t = static_cast<float>(i) / static_cast<float>(samples);
            if (pointNearPolygon(ax + dx * t, ay + dy * t, poly, margin))
                return true;
        }
        return false;
    }

} // namespace mapgeo

// -- FloorMap -------------------------------------------------------------------

FloorMap::FloorMap(DataLogger& logger) : dataLogger(logger) {}

void FloorMap::begin() {
    // SPIFFS is flat (directories are path prefixes), nothing to create.
    // Clean up a temp file left behind by an interrupted upload.
    if (SPIFFS.exists(FLOORMAP_TMP_FILE))
        SPIFFS.remove(FLOORMAP_TMP_FILE);
    if (SPIFFS.exists(FLOORMAP_FIELD_TMP_FILE))
        SPIFFS.remove(FLOORMAP_FIELD_TMP_FILE);
}

bool FloorMap::hasField() const {
    return SPIFFS.exists(FLOORMAP_FIELD_FILE);
}

std::shared_ptr<LogReader> FloorMap::readField() {
    File f = SPIFFS.open(FLOORMAP_FIELD_FILE, FILE_READ);
    if (!f)
        return nullptr;
    return std::make_shared<PlainLogReader>(std::move(f));
}

static int32_t rd32(const uint8_t *p) {
    return static_cast<int32_t>(static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) |
                                (static_cast<uint32_t>(p[2]) << 16) | (static_cast<uint32_t>(p[3]) << 24));
}

static int16_t rd16(const uint8_t *p) {
    return static_cast<int16_t>(static_cast<uint16_t>(p[0]) | (static_cast<uint16_t>(p[1]) << 8));
}

static const size_t FIELD_HEADER = 26;

// Parse header + body; on success `out` holds the field
static bool parseField(File& f, LocField& out) {
    uint8_t hdr[FIELD_HEADER];
    if (f.read(hdr, FIELD_HEADER) != static_cast<int>(FIELD_HEADER))
        return false;
    if (hdr[0] != 'O' || hdr[1] != 'F' || hdr[2] != 'L' || hdr[3] != '1')
        return false;
    out.originX = rd32(hdr + 4);
    out.originY = rd32(hdr + 8);
    out.cell = static_cast<uint16_t>(rd16(hdr + 12));
    out.w = static_cast<uint16_t>(rd16(hdr + 14));
    out.h = static_cast<uint16_t>(rd16(hdr + 16));
    out.lidarSign = static_cast<int8_t>(hdr[18]) < 0 ? -1 : 1;
    out.lidarAngleOffsetRad = static_cast<float>(rd16(hdr + 20)) * 0.1f * 3.14159265f / 180.0f;
    out.lidarOffsetX = rd16(hdr + 22);
    size_t n = static_cast<size_t>(out.w) * static_cast<size_t>(out.h);
    if (out.cell < 10 || out.w == 0 || out.h == 0 || n + FIELD_HEADER > FLOORMAP_FIELD_MAX_BYTES)
        return false;
    out.data.resize(n);
    if (f.read(out.data.data(), n) != static_cast<int>(n)) {
        std::vector<uint8_t>().swap(out.data);
        return false;
    }
    return true;
}

bool FloorMap::loadField(LocField& out) const {
    File f = SPIFFS.open(FLOORMAP_FIELD_FILE, FILE_READ);
    if (!f)
        return false;
    bool ok = parseField(f, out);
    f.close();
    return ok;
}

bool FloorMap::hasMap() const {
    return SPIFFS.exists(FLOORMAP_FILE);
}

size_t FloorMap::fileSize() const {
    File f = SPIFFS.open(FLOORMAP_FILE, FILE_READ);
    if (!f)
        return 0;
    size_t sz = f.size();
    f.close();
    return sz;
}

std::shared_ptr<LogReader> FloorMap::read() {
    File f = SPIFFS.open(FLOORMAP_FILE, FILE_READ);
    if (!f)
        return nullptr;
    return std::make_shared<PlainLogReader>(std::move(f));
}

bool FloorMap::remove() {
    if (!SPIFFS.exists(FLOORMAP_FILE))
        return false;
    SPIFFS.remove(FLOORMAP_FILE);
    if (SPIFFS.exists(FLOORMAP_FIELD_FILE))
        SPIFFS.remove(FLOORMAP_FIELD_FILE);
    dataLogger.logGuided("map_deleted");
    LOG("MAP", "Floor map deleted");
    return true;
}

// -- Upload ---------------------------------------------------------------------

bool FloorMap::beginUpload(size_t totalBytes, bool field) {
    uploadError = "";
    if (uploading)
        abortUpload();
    size_t cap = field ? FLOORMAP_FIELD_MAX_BYTES : FLOORMAP_MAX_BYTES;
    if (totalBytes == 0 || totalBytes > cap) {
        uploadError = "map too large";
        return false;
    }
    uploadingField = field;
    uploadFile = SPIFFS.open(field ? FLOORMAP_FIELD_TMP_FILE : FLOORMAP_TMP_FILE, FILE_WRITE);
    if (!uploadFile) {
        uploadError = "cannot open temp file";
        return false;
    }
    uploading = true;
    uploadReceived = 0;
    return true;
}

bool FloorMap::writeChunk(const uint8_t *data, size_t len) {
    if (!uploading)
        return false;
    if (uploadReceived + len > (uploadingField ? FLOORMAP_FIELD_MAX_BYTES : FLOORMAP_MAX_BYTES)) {
        uploadError = "map too large";
        abortUpload();
        return false;
    }
    if (uploadFile.write(data, len) != len) {
        uploadError = "write failed (filesystem full?)";
        abortUpload();
        return false;
    }
    uploadReceived += len;
    return true;
}

bool FloorMap::endUpload() {
    if (!uploading)
        return false;
    uploadFile.close();
    uploading = false;

    if (uploadingField) {
        LocField field;
        File f = SPIFFS.open(FLOORMAP_FIELD_TMP_FILE, FILE_READ);
        bool ok = f && parseField(f, field);
        if (f)
            f.close();
        if (!ok) {
            uploadError = "invalid localization field";
            SPIFFS.remove(FLOORMAP_FIELD_TMP_FILE);
            return false;
        }
        if (SPIFFS.exists(FLOORMAP_FIELD_FILE))
            SPIFFS.remove(FLOORMAP_FIELD_FILE);
        if (!SPIFFS.rename(FLOORMAP_FIELD_TMP_FILE, FLOORMAP_FIELD_FILE)) {
            uploadError = "rename failed";
            SPIFFS.remove(FLOORMAP_FIELD_TMP_FILE);
            return false;
        }
        LOG("MAP", "Localization field saved (%dx%d @%dmm)", field.w, field.h, field.cell);
        dataLogger.logGuided("field_saved", {{"w", String(field.w), FIELD_INT},
                                             {"h", String(field.h), FIELD_INT},
                                             {"cell", String(field.cell), FIELD_INT}});
        return true;
    }

    // Validate before replacing the current map
    FloorMapData parsed;
    String err;
    if (!parseFile(FLOORMAP_TMP_FILE, parsed, err)) {
        uploadError = err;
        SPIFFS.remove(FLOORMAP_TMP_FILE);
        dataLogger.logGuided("map_rejected", {{"error", err, FIELD_STRING}});
        return false;
    }

    if (SPIFFS.exists(FLOORMAP_FILE))
        SPIFFS.remove(FLOORMAP_FILE);
    if (!SPIFFS.rename(FLOORMAP_TMP_FILE, FLOORMAP_FILE)) {
        uploadError = "rename failed";
        SPIFFS.remove(FLOORMAP_TMP_FILE);
        return false;
    }

    LOG("MAP", "Floor map saved (%u bytes, %u path pts, %u rooms, %u no-go)", static_cast<unsigned>(uploadReceived),
        static_cast<unsigned>(parsed.path.size()), static_cast<unsigned>(parsed.rooms.size()),
        static_cast<unsigned>(parsed.nogos.size()));
    dataLogger.logGuided("map_saved", {{"bytes", String(static_cast<unsigned long>(uploadReceived)), FIELD_INT},
                                       {"points", String(static_cast<unsigned>(parsed.path.size())), FIELD_INT},
                                       {"rooms", String(static_cast<unsigned>(parsed.rooms.size())), FIELD_INT},
                                       {"nogos", String(static_cast<unsigned>(parsed.nogos.size())), FIELD_INT}});
    return true;
}

void FloorMap::abortUpload() {
    if (uploadFile)
        uploadFile.close();
    uploading = false;
    SPIFFS.remove(uploadingField ? FLOORMAP_FIELD_TMP_FILE : FLOORMAP_TMP_FILE);
}

// -- Parsing --------------------------------------------------------------------

bool FloorMap::load(FloorMapData& out, String& err) const {
    if (!hasMap()) {
        err = "no floor map";
        return false;
    }
    return parseFile(FLOORMAP_FILE, out, err);
}

bool FloorMap::parseFile(const char *path, FloorMapData& out, String& err) {
    File f = SPIFFS.open(path, FILE_READ);
    if (!f) {
        err = "cannot open map";
        return false;
    }

    out = FloorMapData();
    String header = f.readStringUntil('\n');
    header.trim();
    if (header != "OFM1") {
        f.close();
        err = "bad header (expected OFM1)";
        return false;
    }

    while (f.available()) {
        String line = f.readStringUntil('\n');
        line.trim();
        if (line.isEmpty())
            continue;
        if (!parseLine(line, out, err)) {
            f.close();
            return false;
        }
    }
    f.close();

    if (out.path.size() < 2) {
        err = "map has no path";
        return false;
    }
    return true;
}

bool FloorMap::parsePoints(const String& line, int start, std::vector<MapPoint>& out, size_t maxPoints) {
    int pos = start;
    int len = static_cast<int>(line.length());
    while (pos < len) {
        while (pos < len && line[pos] == ' ')
            pos++;
        if (pos >= len)
            break;
        int end = line.indexOf(' ', pos);
        if (end < 0)
            end = len;
        int comma = line.indexOf(',', pos);
        if (comma < 0 || comma >= end)
            return false;
        long x = line.substring(pos, comma).toInt();
        long y = line.substring(comma + 1, end).toInt();
        if (x < -32000 || x > 32000 || y < -32000 || y > 32000)
            return false;
        if (out.size() >= maxPoints)
            return false;
        MapPoint p;
        p.x = static_cast<int16_t>(x);
        p.y = static_cast<int16_t>(y);
        out.push_back(p);
        pos = end + 1;
    }
    return true;
}

// Split off the next space-delimited token starting at `pos`; advances `pos`.
static String nextToken(const String& line, int& pos) {
    int len = static_cast<int>(line.length());
    while (pos < len && line[pos] == ' ')
        pos++;
    int end = line.indexOf(' ', pos);
    if (end < 0)
        end = len;
    String tok = line.substring(pos, end);
    pos = end;
    return tok;
}

bool FloorMap::parseLine(const String& line, FloorMapData& out, String& err) {
    char kind = line[0];
    if (line.length() < 2 || line[1] != ' ')
        return true; // Unknown/malformed record type — ignore for forward compatibility

    if (kind == 'S') {
        out.source = line.substring(2);
        return true;
    }

    if (kind == 'P') {
        if (!parsePoints(line, 2, out.path, FLOORMAP_MAX_PATH_POINTS)) {
            err = "invalid or too many path points";
            return false;
        }
        return true;
    }

    if (kind == 'R' || kind == 'N') {
        std::vector<MapZone>& list = kind == 'R' ? out.rooms : out.nogos;
        if (list.size() >= FLOORMAP_MAX_ZONES) {
            err = "too many zones";
            return false;
        }
        int pos = 2;
        MapZone zone;
        zone.id = static_cast<int>(nextToken(line, pos).toInt());
        if (zone.id <= 0) {
            err = "invalid zone id";
            return false;
        }
        if (kind == 'R')
            zone.name = nextToken(line, pos);
        if (!parsePoints(line, pos, zone.poly, FLOORMAP_MAX_VERTICES) || zone.poly.size() < 3) {
            err = "invalid zone polygon";
            return false;
        }
        for (const auto& z: list) {
            if (z.id == zone.id) {
                err = "duplicate zone id";
                return false;
            }
        }
        list.push_back(zone);
        return true;
    }

    return true; // Unknown record — ignore
}

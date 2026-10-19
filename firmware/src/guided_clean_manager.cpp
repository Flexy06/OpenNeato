#include "guided_clean_manager.h"
#include <algorithm>
#include <memory>
#include <cmath>
#include <cstring>
#include "json_fields.h"
#include "manual_clean_manager.h"
#include "system_manager.h"

static const float DEG = 3.14159265f / 180.0f;

GuidedCleanManager::GuidedCleanManager(NeatoSerial& serial, FloorMap& floorMap, ManualCleanManager& manual,
                                       DataLogger& logger, SystemManager& sys) :
    LoopTask(GUIDED_TICK_MS), serial(serial), floorMap(floorMap), manual(manual), dataLogger(logger), sysMgr(sys) {
    TaskRegistry::add(this);
}

// -- Helpers ----------------------------------------------------------------------

float GuidedCleanManager::wrapAngle(float a) {
    while (a > 3.14159265f)
        a -= 2.0f * 3.14159265f;
    while (a < -3.14159265f)
        a += 2.0f * 3.14159265f;
    return a;
}

const char *GuidedCleanManager::phaseName(Phase p) {
    switch (p) {
        case Phase::Idle:
            return "idle";
        case Phase::Starting:
            return "starting";
        case Phase::Localizing:
            return "localizing";
        case Phase::Running:
            return "running";
        case Phase::Returning:
            return "returning";
        case Phase::Docking:
            return "docking";
        case Phase::Finishing:
            return "finishing";
        case Phase::Done:
            return "done";
        case Phase::Stopped:
            return "stopped";
        case Phase::Failed:
            return "failed";
    }
    return "idle";
}

bool GuidedCleanManager::isActive() const {
    return phase == Phase::Starting || phase == Phase::Localizing || phase == Phase::Running ||
           phase == Phase::Returning || phase == Phase::Docking || phase == Phase::Finishing;
}

// -- Coverage grid ------------------------------------------------------------------

void GuidedCleanManager::CoverGrid::init(int minX, int minY, int maxX, int maxY) {
    originX = minX;
    originY = minY;
    w = (maxX - minX) / GUIDED_COVER_CELL_MM + 1;
    h = (maxY - minY) / GUIDED_COVER_CELL_MM + 1;
    if (w > GUIDED_COVER_MAX_CELLS)
        w = GUIDED_COVER_MAX_CELLS;
    if (h > GUIDED_COVER_MAX_CELLS)
        h = GUIDED_COVER_MAX_CELLS;
    bits.assign((static_cast<size_t>(w) * static_cast<size_t>(h) + 7) / 8, 0);
}

void GuidedCleanManager::CoverGrid::stamp(float x, float y, float radius) {
    int r = static_cast<int>(radius / GUIDED_COVER_CELL_MM);
    int cx = static_cast<int>(floorf((x - static_cast<float>(originX)) / GUIDED_COVER_CELL_MM));
    int cy = static_cast<int>(floorf((y - static_cast<float>(originY)) / GUIDED_COVER_CELL_MM));
    for (int dy = -r; dy <= r; dy++) {
        for (int dx = -r; dx <= r; dx++) {
            if (dx * dx + dy * dy > r * r)
                continue;
            int gx = cx + dx;
            int gy = cy + dy;
            if (gx < 0 || gy < 0 || gx >= w || gy >= h)
                continue;
            size_t i = static_cast<size_t>(gy) * static_cast<size_t>(w) + static_cast<size_t>(gx);
            bits[i >> 3] |= static_cast<uint8_t>(1u << (i & 7));
        }
    }
}

bool GuidedCleanManager::CoverGrid::covered(float x, float y) const {
    int gx = static_cast<int>(floorf((x - static_cast<float>(originX)) / GUIDED_COVER_CELL_MM));
    int gy = static_cast<int>(floorf((y - static_cast<float>(originY)) / GUIDED_COVER_CELL_MM));
    if (gx < 0 || gy < 0 || gx >= w || gy >= h)
        return false;
    size_t i = static_cast<size_t>(gy) * static_cast<size_t>(w) + static_cast<size_t>(gx);
    return (bits[i >> 3] & (1u << (i & 7))) != 0;
}

void GuidedCleanManager::CoverGrid::clear() {
    std::vector<uint8_t>().swap(bits);
    w = 0;
    h = 0;
}

// -- Plan ---------------------------------------------------------------------------

bool GuidedCleanManager::nearNogo(float x, float y) const {
    return std::any_of(nogos.begin(), nogos.end(), [x, y](const MapZone& z) {
        return mapgeo::pointNearPolygon(x, y, z.poly, GUIDED_NOGO_MARGIN_MM);
    });
}

bool GuidedCleanManager::nearObstacle(float x, float y) const {
    if (dyn.blocked(x, y))
        return true;
    return std::any_of(obstacles.begin(), obstacles.end(), [x, y](const Obstacle& o) {
        float dx = x - static_cast<float>(o.x);
        float dy = y - static_cast<float>(o.y);
        return o.live() && sqrtf(dx * dx + dy * dy) < static_cast<float>(o.r + GUIDED_ROBOT_RADIUS_MM);
    });
}

bool GuidedCleanManager::segmentClear(float ax, float ay, float bx, float by, bool requireCover) const {
    for (const auto& z: nogos) {
        // Already inside the safety band (pose correction, arc): any line from here
        // would fail the full margin. Allow lines that do not get closer to the zone
        // than we already are, never into it.
        float margin = GUIDED_NOGO_MARGIN_MM;
        while (margin > 0.0f && mapgeo::pointNearPolygon(ax, ay, z.poly, margin))
            margin -= 20.0f;
        margin = margin > 30.0f ? margin - 10.0f : 20.0f;
        if (mapgeo::segmentNearPolygon(ax, ay, bx, by, z.poly, margin))
            return false;
    }
    for (const auto& o: obstacles) {
        if (o.live() &&
            mapgeo::distPointSegment(o.x, o.y, ax, ay, bx, by) < static_cast<float>(o.r + GUIDED_ROBOT_RADIUS_MM))
            return false;
    }
    if (!requireCover)
        return true;
    float dx = bx - ax;
    float dy = by - ay;
    float len = sqrtf(dx * dx + dy * dy);
    int samples = static_cast<int>(len / 50.0f) + 1;
    for (int i = 0; i <= samples; i++) {
        float t = static_cast<float>(i) / static_cast<float>(samples);
        if (!cover.covered(ax + dx * t, ay + dy * t))
            return false;
    }
    return true;
}

// Parse "1,3,4" into a list of room ids. Empty = all rooms.
static std::vector<int> parseRoomIds(const String& rooms) {
    std::vector<int> ids;
    int pos = 0;
    int len = static_cast<int>(rooms.length());
    while (pos < len) {
        int comma = rooms.indexOf(',', pos);
        if (comma < 0)
            comma = len;
        int id = static_cast<int>(rooms.substring(pos, comma).toInt());
        if (id > 0)
            ids.push_back(id);
        pos = comma + 1;
    }
    return ids;
}

String GuidedCleanManager::buildPlan(const FloorMapData& map, const String& rooms) {
    freePlan();

    std::vector<int> ids = parseRoomIds(rooms);
    std::vector<const MapZone *> selected;
    if (ids.empty()) {
        for (const auto& r: map.rooms)
            selected.push_back(&r);
    } else {
        for (int id: ids) {
            const MapZone *r = map.findRoom(id);
            if (!r)
                return "unknown room id " + String(id);
            selected.push_back(r);
        }
    }
    if (selected.empty())
        return "no rooms defined on the map";

    nogos = map.nogos;
    // The localization field doubles as the free-space map for lanes and planning.
    // Without it the robot would drive blind: required.
    fieldLoadFailed = false;
    if (!floorMap.loadField(field)) {
        fieldLoadFailed = floorMap.hasField();
        field = LocField();
        freePlan();
        return fieldLoadFailed ? "the LIDAR map could not be loaded (not enough memory) - restart the ESP32"
                               : "no LIDAR map: pin a mapping run first";
    }

    // Dock pose in the map: learned on earlier runs, else the map origin (the
    // mapping run started on the dock)
    dockKnown = floorMap.loadDock(dockX, dockY, dockT, dockRuns);
    if (!dockKnown) {
        dockX = 0.0f;
        dockY = 0.0f;
        dockT = 0.0f;
        dockRuns = 0;
    }
    if (nearNogo(dockX, dockY))
        return "the dock lies inside a no-go zone";

    // Coverage grid over the recorded path and the selected rooms (+ margin)
    int minX = 0;
    int minY = 0;
    int maxX = 0;
    int maxY = 0;
    for (const auto& p: map.path) {
        minX = p.x < minX ? p.x : minX;
        minY = p.y < minY ? p.y : minY;
        maxX = p.x > maxX ? p.x : maxX;
        maxY = p.y > maxY ? p.y : maxY;
    }
    for (const MapZone *r: selected) {
        for (const auto& v: r->poly) {
            minX = v.x < minX ? v.x : minX;
            minY = v.y < minY ? v.y : minY;
            maxX = v.x > maxX ? v.x : maxX;
            maxY = v.y > maxY ? v.y : maxY;
        }
    }
    int margin = GUIDED_COVER_STAMP_MM + GUIDED_COVER_CELL_MM;
    cover.init(minX - margin, minY - margin, maxX + margin, maxY + margin);

    // Stamp recorded poses and the straight hops between consecutive snapshots
    // (snapshots are ~2s apart; the robot drives mostly straight in between).
    float px = 0.0f;
    float py = 0.0f;
    cover.stamp(px, py, GUIDED_COVER_STAMP_MM);
    for (const auto& p: map.path) {
        float dx = static_cast<float>(p.x) - px;
        float dy = static_cast<float>(p.y) - py;
        float len = sqrtf(dx * dx + dy * dy);
        if (len < 1200.0f) {
            int samples = static_cast<int>(len / 50.0f) + 1;
            for (int i = 1; i <= samples; i++) {
                float t = static_cast<float>(i) / static_cast<float>(samples);
                cover.stamp(px + dx * t, py + dy * t, GUIDED_COVER_STAMP_MM);
            }
        } else {
            cover.stamp(p.x, p.y, GUIDED_COVER_STAMP_MM);
        }
        px = p.x;
        py = p.y;
    }

    buildGrid();
    initDyn();

    // Cleaning runs: parallel lanes over the free floor of each selected room
    // (structured coverage, independent of how the recording wandered). Without a
    // localization field the free floor is what the recording covered; if no lane
    // fits at all, fall back to the recorded points inside the rooms.
    // Runs are visited nearest-first in either direction (→ serpentine order);
    // transit between them is planned on the grid.
    buildLanes(selected);
    if (cleanPts.empty()) {
        LOG("GUIDED", "Plan: no lanes fit - following the recorded path instead");
        planRecorded(map, selected);
    }

    if (cleanPts.empty()) {
        freePlan();
        return "no free floor found in the selected rooms";
    }
    cleanTotal = static_cast<int>(cleanPts.size());
    cleanPts.shrink_to_fit();

    LOG("GUIDED", "Plan: %d clean points in %u runs, %u no-go zones, grid %dx%d @%dmm", cleanTotal,
        static_cast<unsigned>(runs.size()), static_cast<unsigned>(nogos.size()), grid.w, grid.h, grid.cell);
    return "";
}


// -- Cleaning runs ----------------------------------------------------------------

// Free floor for a lane point at (x, y): ≥ ~170 mm from mapped returns in the
// LIDAR map (or driven over during the recording), reachable from the dock and
// clear of no-go zones.
bool GuidedCleanManager::floorFree(float x, float y) const {
    int cx = 0;
    int cy = 0;
    grid.toCell(x, y, cx, cy);
    if (!grid.inside(cx, cy) || !grid.free[static_cast<size_t>(grid.index(cx, cy))])
        return false;
    if (nearNogo(x, y))
        return false;
    if (field.valid() && field.inside(x, y))
        return field.at(x, y) <= GUIDED_FIELD_LANE_MAX || cover.covered(x, y);
    return cover.covered(x, y);
}

void GuidedCleanManager::addRun(const std::vector<Waypoint>& pts) {
    if (pts.empty() || cleanPts.size() + pts.size() > 60000)
        return;
    Run r;
    r.first = static_cast<uint16_t>(cleanPts.size());
    for (const Waypoint& w: pts)
        cleanPts.push_back(w);
    r.last = static_cast<uint16_t>(cleanPts.size() - 1);
    runs.push_back(r);
}

void GuidedCleanManager::buildLanes(const std::vector<const MapZone *>& selected) {
    const float spacing = GUIDED_LANE_SPACING_MM;
    const float sample = 50.0f;
    for (const MapZone *room: selected) {
        float minX = 1e9f;
        float minY = 1e9f;
        float maxX = -1e9f;
        float maxY = -1e9f;
        for (const auto& v: room->poly) {
            minX = fminf(minX, v.x);
            minY = fminf(minY, v.y);
            maxX = fmaxf(maxX, v.x);
            maxY = fmaxf(maxY, v.y);
        }
        // Lanes along the longer side: fewer turns
        bool alongX = (maxX - minX) >= (maxY - minY);
        float a0 = alongX ? minX : minY;
        float a1 = alongX ? maxX : maxY;
        float b0 = alongX ? minY : minX;
        float b1 = alongX ? maxY : maxX;
        int lanes = static_cast<int>((b1 - b0) / spacing);
        if (lanes < 1)
            lanes = 1;
        // Centre the lanes in the room
        float first = b0 + ((b1 - b0) - static_cast<float>(lanes - 1) * spacing) / 2.0f;
        for (int l = 0; l < lanes; l++) {
            float b = first + static_cast<float>(l) * spacing;
            std::vector<Waypoint> pts;
            float runStart = 0.0f;
            float lastPt = 0.0f;
            bool inRun = false;
            float prevA = a0;
            auto flush = [&](float endA) {
                if (inRun && endA - runStart >= GUIDED_LANE_MIN_MM) {
                    if (endA - lastPt > 60.0f) {
                        Waypoint w;
                        w.x = static_cast<int16_t>(alongX ? endA : b);
                        w.y = static_cast<int16_t>(alongX ? b : endA);
                        w.clean = true;
                        w.room = static_cast<uint8_t>(room->id);
                        pts.push_back(w);
                    }
                    addRun(pts);
                }
                pts.clear();
                inRun = false;
            };
            for (float a = a0; a <= a1; a += sample) {
                float x = alongX ? a : b;
                float y = alongX ? b : a;
                bool ok = mapgeo::pointInPolygon(x, y, room->poly) && floorFree(x, y);
                if (!ok) {
                    flush(prevA);
                    prevA = a;
                    continue;
                }
                if (!inRun) {
                    inRun = true;
                    runStart = a;
                    lastPt = a - GUIDED_LANE_POINT_MM;
                }
                if (a - lastPt >= GUIDED_LANE_POINT_MM) {
                    Waypoint w;
                    w.x = static_cast<int16_t>(x);
                    w.y = static_cast<int16_t>(y);
                    w.clean = true;
                    w.room = static_cast<uint8_t>(room->id);
                    pts.push_back(w);
                    lastPt = a;
                }
                prevA = a;
            }
            flush(prevA);
        }
    }
}

void GuidedCleanManager::planRecorded(const FloorMapData& map, const std::vector<const MapZone *>& selected) {
    cleanPts.reserve(map.path.size() / 2);
    bool inRun = false;
    for (const auto& p: map.path) {
        const MapZone *room = nullptr;
        if (!nearNogo(p.x, p.y)) {
            for (const MapZone *r: selected) {
                if (mapgeo::pointInPolygon(p.x, p.y, r->poly)) {
                    room = r;
                    break;
                }
            }
        }
        if (!room) {
            inRun = false;
            continue;
        }
        if (inRun) {
            const Waypoint& prev = cleanPts.back();
            auto dx = static_cast<float>(p.x - prev.x);
            auto dy = static_cast<float>(p.y - prev.y);
            float d = sqrtf(dx * dx + dy * dy);
            if (d < GUIDED_MIN_POINT_SPACING_MM)
                continue;
            if (d > GUIDED_RUN_GAP_MM)
                inRun = false;
        }
        Waypoint w;
        w.x = p.x;
        w.y = p.y;
        w.clean = true;
        w.room = static_cast<uint8_t>(room->id);
        if (!inRun) {
            Run r;
            r.first = static_cast<uint16_t>(cleanPts.size());
            runs.push_back(r);
            inRun = true;
        }
        cleanPts.push_back(w);
        runs.back().last = static_cast<uint16_t>(cleanPts.size() - 1);
    }
}

// -- Planning grid ------------------------------------------------------------------

void GuidedCleanManager::PlanGrid::toCell(float x, float y, int& cx, int& cy) const {
    cx = static_cast<int>(floorf((x - static_cast<float>(originX)) / static_cast<float>(cell)));
    cy = static_cast<int>(floorf((y - static_cast<float>(originY)) / static_cast<float>(cell)));
}

void GuidedCleanManager::PlanGrid::clear() {
    std::vector<uint8_t>().swap(free);
    w = 0;
    h = 0;
}

void GuidedCleanManager::buildGrid() {
    int spanX = cover.w * GUIDED_COVER_CELL_MM;
    int spanY = cover.h * GUIDED_COVER_CELL_MM;
    int cell = GUIDED_PLAN_CELL_MM;
    while (spanX / cell + 1 > GUIDED_PLAN_MAX_CELLS || spanY / cell + 1 > GUIDED_PLAN_MAX_CELLS)
        cell *= 2;
    grid.cell = cell;
    grid.originX = cover.originX;
    grid.originY = cover.originY;
    grid.w = spanX / cell + 1;
    grid.h = spanY / cell + 1;
    grid.free.assign(static_cast<size_t>(grid.w) * static_cast<size_t>(grid.h), 0);

    // A planning cell is free when at least half of its coverage sub-cells were
    // driven over during the recording and its centre is clear of no-go zones.
    int sub = cell / GUIDED_COVER_CELL_MM;
    for (int cy = 0; cy < grid.h; cy++) {
        for (int cx = 0; cx < grid.w; cx++) {
            int hits = 0;
            for (int sy = 0; sy < sub; sy++) {
                for (int sx = 0; sx < sub; sx++) {
                    float x = static_cast<float>(grid.originX + cx * cell + sx * GUIDED_COVER_CELL_MM) +
                              GUIDED_COVER_CELL_MM / 2.0f;
                    float y = static_cast<float>(grid.originY + cy * cell + sy * GUIDED_COVER_CELL_MM) +
                              GUIDED_COVER_CELL_MM / 2.0f;
                    if (cover.covered(x, y))
                        hits++;
                }
            }
            float gx = grid.centerX(cx);
            float gy = grid.centerY(cy);
            // Driven over during the recording, or clear of walls in the LIDAR map
            bool driven = hits * 2 >= sub * sub;
            bool mapped = field.valid() && field.inside(gx, gy) && field.at(gx, gy) <= GUIDED_FIELD_FREE_MAX;
            bool ok = (driven || mapped) && !nearNogo(gx, gy);
            grid.free[static_cast<size_t>(grid.index(cx, cy))] = ok ? 1 : 0;
        }
    }

    // The robot leaves the dock straight ahead (+x of the dock frame): keep the
    // dock axis open up to the staging point even if the map is thin there.
    for (int x = 0; x <= GUIDED_DOCK_STAGING_MM + cell; x += cell / 2) {
        float wx = 0.0f;
        float wy = 0.0f;
        fromDock(static_cast<float>(x), 0.0f, wx, wy);
        int cx = 0;
        int cy = 0;
        grid.toCell(wx, wy, cx, cy);
        if (grid.inside(cx, cy) && !nearNogo(wx, wy))
            grid.free[static_cast<size_t>(grid.index(cx, cy))] = 1;
    }

    // Keep only floor reachable from the dock: open space outside the walls (the
    // field is "far from walls" there too) or behind closed doors is not floor.
    std::vector<uint8_t> seen(grid.free.size(), 0);
    std::vector<uint16_t> queue;
    queue.reserve(256);
    int sx = 0;
    int sy = 0;
    float seedX = 0.0f;
    float seedY = 0.0f;
    fromDock(static_cast<float>(GUIDED_DOCK_STAGING_MM) / 2.0f, 0.0f, seedX, seedY);
    grid.toCell(seedX, seedY, sx, sy);
    if (grid.inside(sx, sy) && grid.free[static_cast<size_t>(grid.index(sx, sy))]) {
        seen[static_cast<size_t>(grid.index(sx, sy))] = 1;
        queue.push_back(static_cast<uint16_t>(grid.index(sx, sy)));
    }
    for (size_t q = 0; q < queue.size(); q++) {
        int cx = queue[q] % grid.w;
        int cy = queue[q] / grid.w;
        static const int8_t DX[4] = {1, -1, 0, 0};
        static const int8_t DY[4] = {0, 0, 1, -1};
        for (int k = 0; k < 4; k++) {
            int nx = cx + DX[k];
            int ny = cy + DY[k];
            if (!grid.inside(nx, ny))
                continue;
            auto ni = static_cast<size_t>(grid.index(nx, ny));
            if (seen[ni] || !grid.free[ni])
                continue;
            seen[ni] = 1;
            queue.push_back(static_cast<uint16_t>(ni));
        }
    }
    grid.free.swap(seen);
}

bool GuidedCleanManager::cellBlocked(int cx, int cy) const {
    if (!grid.inside(cx, cy))
        return true;
    if (!planMask.empty())
        return planMask[static_cast<size_t>(grid.index(cx, cy))] != 0;
    return cellBlockedRaw(cx, cy);
}

// Transit planning keeps more room than lanes: a path is only useful if every
// straight piece of it also passes the executor's clearance check, so cells
// close to walls or seen obstacles are not used (built once per plan).
void GuidedCleanManager::buildPlanMask() const {
    planMask.assign(grid.free.size(), 1);
    for (int cy = 0; cy < grid.h; cy++) {
        for (int cx = 0; cx < grid.w; cx++) {
            if (cellBlockedRaw(cx, cy))
                continue;
            float x = grid.centerX(cx);
            float y = grid.centerY(cy);
            if (field.inside(x, y) && wallDist(x, y) < GUIDED_PLAN_CLEARANCE_MM)
                continue;
            if (dynNearest(x, y) < GUIDED_PLAN_CLEARANCE_MM)
                continue;
            planMask[static_cast<size_t>(grid.index(cx, cy))] = 0;
        }
    }
}

bool GuidedCleanManager::cellBlockedRaw(int cx, int cy) const {
    if (!grid.free[static_cast<size_t>(grid.index(cx, cy))])
        return true;
    float x = grid.centerX(cx);
    float y = grid.centerY(cy);
    if (dyn.blocked(x, y))
        return true; // Seen by the LIDAR: real, and cleared again when it goes away
    if (planIgnoreObstacles)
        return false;
    // Bump memory is placed generously ahead of the robot; for planning only
    // block its core so a bump against a wall does not seal a narrow corridor.
    return std::any_of(obstacles.begin(), obstacles.end(), [x, y](const Obstacle& o) {
        float dx = x - static_cast<float>(o.x);
        float dy = y - static_cast<float>(o.y);
        return o.live() && sqrtf(dx * dx + dy * dy) < static_cast<float>(o.r + GUIDED_ROBOT_RADIUS_MM / 2);
    });
}

// Real clearance along a straight line from the map and the LIDAR obstacle layer
// (planning cells are coarse: a line can graze a wall or a table leg while every
// cell centre it touches is free). Near the start we may already stand close to
// something (after a bump): there it may stay as close, never get closer.
bool GuidedCleanManager::lineClearance(float ax, float ay, float bx, float by) const {
    float dx = bx - ax;
    float dy = by - ay;
    float len = sqrtf(dx * dx + dy * dy);
    int fine = static_cast<int>(len / 50.0f) + 1;
    const float need = GUIDED_CLEARANCE_MM;
    // Near the start and the goal the line may be as tight as those points are
    // (the robot stands there / the planner accepted the goal), never tighter
    bool useField = field.valid();
    float startWall = useField && field.inside(ax, ay) ? wallDist(ax, ay) : 1e9f;
    float endWall = useField && field.inside(bx, by) ? wallDist(bx, by) : 1e9f;
    float startDyn = dynNearest(ax, ay);
    float endDyn = dynNearest(bx, by);
    for (int i = 1; i <= fine; i++) {
        float t = static_cast<float>(i) / static_cast<float>(fine);
        float x = ax + dx * t;
        float y = ay + dy * t;
        bool nearStart = t * len < 250.0f;
        bool nearEnd = (1.0f - t) * len < 250.0f;
        if (useField && field.inside(x, y)) {
            float req = need;
            if (nearStart && startWall < req)
                req = startWall - 10.0f;
            if (nearEnd && endWall < req)
                req = endWall - 10.0f;
            if (wallDist(x, y) < req)
                return false;
        }
        if ((nearStart && startDyn < need + 100.0f) || (nearEnd && endDyn < need + 100.0f)) {
            float req = need;
            if (nearStart && startDyn < req)
                req = startDyn - 10.0f;
            if (nearEnd && endDyn < req)
                req = endDyn - 10.0f;
            if (dynNearest(x, y) < req)
                return false;
            continue;
        }
        if (dyn.blocked(x, y))
            return false;
    }
    return true;
}

// Distance (mm) to the nearest mapped return, from the likelihood field (300 mm falloff)
float GuidedCleanManager::wallDist(float x, float y) const {
    return 300.0f * (1.0f - static_cast<float>(field.at(x, y)) / 255.0f);
}

// Distance (mm) to the nearest occupied LIDAR-layer cell within ~400 mm (1e9 if none)
float GuidedCleanManager::dynNearest(float x, float y) const {
    if (dyn.w == 0)
        return 1e9f;
    const auto cell = static_cast<float>(dyn.cell);
    int cx = static_cast<int>(floorf((x - static_cast<float>(dyn.originX)) / cell));
    int cy = static_cast<int>(floorf((y - static_cast<float>(dyn.originY)) / cell));
    int r = static_cast<int>(400.0f / cell) + 1;
    float best = 1e9f;
    for (int ky = -r; ky <= r; ky++) {
        for (int kx = -r; kx <= r; kx++) {
            if (!dyn.inside(cx + kx, cy + ky) ||
                dyn.hits[static_cast<size_t>((cy + ky) * dyn.w + cx + kx)] < GUIDED_DYN_HITS)
                continue;
            float ox = static_cast<float>(dyn.originX) + (static_cast<float>(cx + kx) + 0.5f) * cell;
            float oy = static_cast<float>(dyn.originY) + (static_cast<float>(cy + ky) + 0.5f) * cell;
            float d = sqrtf((ox - x) * (ox - x) + (oy - y) * (oy - y));
            if (d < best)
                best = d;
        }
    }
    return best;
}

bool GuidedCleanManager::gridLineFree(float ax, float ay, float bx, float by) const {
    float dx = bx - ax;
    float dy = by - ay;
    float len = sqrtf(dx * dx + dy * dy);
    float step = static_cast<float>(grid.cell) / 3.0f;
    int samples = static_cast<int>(len / step) + 1;
    for (int i = 0; i <= samples; i++) {
        float t = static_cast<float>(i) / static_cast<float>(samples);
        int cx = 0;
        int cy = 0;
        grid.toCell(ax + dx * t, ay + dy * t, cx, cy);
        if (cellBlocked(cx, cy))
            return false;
    }
    if (!lineClearance(ax, ay, bx, by))
        return false;
    if (planIgnoreObstacles) {
        for (const auto& z: nogos) {
            if (mapgeo::segmentNearPolygon(ax, ay, bx, by, z.poly, GUIDED_NOGO_MARGIN_MM))
                return false;
        }
        return true;
    }
    return segmentClear(ax, ay, bx, by, false);
}

// Shortest path from the current pose to (gx, gy) over free cells: BFS from the
// goal (8-connected, no corner cutting), walk downhill from the start, then pull
// the string so the result is a few straight legs instead of a staircase.
// Uses a transient uint16 distance map (≤ GUIDED_PLAN_MAX_CELLS² × 2 bytes).
bool GuidedCleanManager::planPath(float gx, float gy, std::vector<Waypoint>& out) const {
    if (planPathOnce(gx, gy, out))
        return true;
    if (obstacles.empty())
        return false;
    // Remembered obstacles may be phantom (drift made us touch a wall) or have
    // moved: try again without them rather than giving up on the goal.
    LOG("GUIDED", "Plan: retrying without obstacle memory");
    planIgnoreObstacles = true;
    bool ok = planPathOnce(gx, gy, out);
    planIgnoreObstacles = false;
    return ok;
}

bool GuidedCleanManager::planPathOnce(float gx, float gy, std::vector<Waypoint>& out) const {
    out.clear();
    if (grid.w == 0)
        return false;
    buildPlanMask();
    struct MaskGuard {
        std::vector<uint8_t>& m;
        ~MaskGuard() { std::vector<uint8_t>().swap(m); }
    } guard{planMask};

    auto snap = [this](float x, float y, int& cx, int& cy) {
        grid.toCell(x, y, cx, cy);
        if (!cellBlocked(cx, cy))
            return true;
        // Drift or an obstacle put us on a blocked cell: use the nearest free one
        for (int r = 1; r <= 4; r++) {
            int best = -1;
            int bx = 0;
            int by = 0;
            for (int dy = -r; dy <= r; dy++) {
                for (int dx = -r; dx <= r; dx++) {
                    if (abs(dx) != r && abs(dy) != r)
                        continue;
                    if (cellBlocked(cx + dx, cy + dy))
                        continue;
                    int d = dx * dx + dy * dy;
                    if (best < 0 || d < best) {
                        best = d;
                        bx = cx + dx;
                        by = cy + dy;
                    }
                }
            }
            if (best >= 0) {
                cx = bx;
                cy = by;
                return true;
            }
        }
        return false;
    };

    int sx = 0;
    int sy = 0;
    int tx = 0;
    int ty = 0;
    if (!snap(poseX, poseY, sx, sy) || !snap(gx, gy, tx, ty)) {
        LOG("GUIDED", "Plan: no free cell near %s", snap(poseX, poseY, sx, sy) ? "goal" : "robot");
        return false;
    }

    const uint16_t INF = 0xFFFF;
    std::vector<uint16_t> dist(static_cast<size_t>(grid.w) * static_cast<size_t>(grid.h), INF);
    std::vector<uint16_t> frontier;
    std::vector<uint16_t> next;
    dist[static_cast<size_t>(grid.index(tx, ty))] = 0;
    frontier.push_back(static_cast<uint16_t>(grid.index(tx, ty)));
    const int DX[8] = {1, -1, 0, 0, 1, 1, -1, -1};
    const int DY[8] = {0, 0, 1, -1, 1, -1, 1, -1};
    uint16_t level = 0;
    size_t startIdx = static_cast<size_t>(grid.index(sx, sy));
    while (!frontier.empty() && dist[startIdx] == INF && level < INF - 1) {
        level++;
        next.clear();
        for (uint16_t idx: frontier) {
            int cx = idx % grid.w;
            int cy = idx / grid.w;
            for (int k = 0; k < 8; k++) {
                int nx = cx + DX[k];
                int ny = cy + DY[k];
                if (cellBlocked(nx, ny))
                    continue;
                if (k >= 4 && (cellBlocked(cx + DX[k], cy) || cellBlocked(cx, cy + DY[k])))
                    continue; // No diagonal squeeze past a corner
                auto ni = static_cast<size_t>(grid.index(nx, ny));
                if (dist[ni] != INF)
                    continue;
                dist[ni] = level;
                next.push_back(static_cast<uint16_t>(ni));
            }
        }
        frontier.swap(next);
    }
    if (dist[startIdx] == INF) {
        LOG("GUIDED", "Plan: goal %.0f,%.0f not reachable from %.0f,%.0f", gx, gy, poseX, poseY);
        return false;
    }

    // Walk downhill from start to goal
    std::vector<Waypoint> cells;
    int cx = sx;
    int cy = sy;
    while (dist[static_cast<size_t>(grid.index(cx, cy))] > 0) {
        uint16_t here = dist[static_cast<size_t>(grid.index(cx, cy))];
        int bx = cx;
        int by = cy;
        for (int k = 0; k < 8; k++) {
            int nx = cx + DX[k];
            int ny = cy + DY[k];
            if (!grid.inside(nx, ny))
                continue;
            if (dist[static_cast<size_t>(grid.index(nx, ny))] == here - 1) {
                bx = nx;
                by = ny;
                if (k < 4)
                    break; // Prefer straight moves
            }
        }
        if (bx == cx && by == cy)
            return false; // Should not happen
        cx = bx;
        cy = by;
        Waypoint w;
        w.x = static_cast<int16_t>(grid.centerX(cx));
        w.y = static_cast<int16_t>(grid.centerY(cy));
        cells.push_back(w);
    }
    std::vector<uint16_t>().swap(dist);
    if (!cells.empty()) {
        cells.back().x = static_cast<int16_t>(gx);
        cells.back().y = static_cast<int16_t>(gy);
    } else {
        Waypoint w;
        w.x = static_cast<int16_t>(gx);
        w.y = static_cast<int16_t>(gy);
        cells.push_back(w);
    }

    // String pulling: from each anchor jump to the farthest cell still in line of sight
    float ax = poseX;
    float ay = poseY;
    size_t i = 0;
    while (i < cells.size()) {
        size_t far = i;
        for (size_t j = cells.size(); j-- > i;) {
            if (gridLineFree(ax, ay, cells[j].x, cells[j].y)) {
                far = j;
                break;
            }
        }
        out.push_back(cells[far]);
        ax = cells[far].x;
        ay = cells[far].y;
        i = far + 1;
    }
    return true;
}

// -- Legs -----------------------------------------------------------------------------

void GuidedCleanManager::setLeg(std::vector<Waypoint>& pts, bool transit) {
    route.clear();
    Waypoint here;
    here.x = static_cast<int16_t>(poseX);
    here.y = static_cast<int16_t>(poseY);
    route.push_back(here); // Index 0 = where the leg starts (already "reached")
    for (Waypoint w: pts) {
        if (transit)
            w.clean = false;
        route.push_back(w);
    }
    legTransit = transit;
    legReplans = 0;
    escapes = 0;
    needEscape = false;
    if (!route.empty())
        tracePose("leg", {{"k", transit ? "transit" : "clean", FIELD_STRING},
                          {"run", String(curRun), FIELD_INT},
                          {"n", String(static_cast<unsigned>(route.size() - 1)), FIELD_INT},
                          {"gx", String(route.back().x), FIELD_INT},
                          {"gy", String(route.back().y), FIELD_INT}});
    routeIdx = 0;
    targetIdx = 0;
    obstacleStreak = 0;
    selectNextTarget();
}

void GuidedCleanManager::startNextLeg() {
    // Nearest unfinished run end (runs may be cleaned in either direction)
    int best = -1;
    bool bestRev = false;
    float bestD = 1e12f;
    for (size_t r = 0; r < runs.size(); r++) {
        if (runs[r].done)
            continue;
        const Waypoint& a = cleanPts[runs[r].first];
        const Waypoint& b = cleanPts[runs[r].last];
        float da = (a.x - poseX) * (a.x - poseX) + (a.y - poseY) * (a.y - poseY);
        float db = (b.x - poseX) * (b.x - poseX) + (b.y - poseY) * (b.y - poseY);
        if (da < bestD) {
            bestD = da;
            best = static_cast<int>(r);
            bestRev = false;
        }
        if (db < bestD) {
            bestD = db;
            best = static_cast<int>(r);
            bestRev = true;
        }
    }
    if (best < 0) {
        beginReturn("done");
        return;
    }
    curRun = best;
    curReversed = bestRev;
    const Waypoint& entry = cleanPts[bestRev ? runs[best].last : runs[best].first];
    legGoalX = entry.x;
    legGoalY = entry.y;

    if (sqrtf(bestD) < 400.0f && lineClearance(poseX, poseY, legGoalX, legGoalY) &&
        segmentClear(poseX, poseY, legGoalX, legGoalY, false)) {
        loadRunLeg();
        return;
    }
    std::vector<Waypoint> path;
    if (planPath(legGoalX, legGoalY, path)) {
        LOG("GUIDED", "Transit to run %d (%u legs)", best, static_cast<unsigned>(path.size()));
        setLeg(path, true);
        return;
    }
    skipCurrentRun("unreachable");
}

void GuidedCleanManager::loadRunLeg() {
    const Run& r = runs[static_cast<size_t>(curRun)];
    std::vector<Waypoint> pts;
    pts.reserve(r.last - r.first + 1u);
    for (int i = r.first; i <= r.last; i++)
        pts.push_back(cleanPts[static_cast<size_t>(i)]);
    if (curReversed) {
        for (size_t i = 0, j = pts.size() - 1; i < j; i++, j--) {
            Waypoint t = pts[i];
            pts[i] = pts[j];
            pts[j] = t;
        }
    }
    setLeg(pts, false);
}

void GuidedCleanManager::skipCurrentRun(const char *why) {
    if (curRun < 0)
        return;
    Run& r = runs[static_cast<size_t>(curRun)];
    if (!r.done) {
        // Count the remaining points as processed so progress stays honest-ish
        int remaining = r.last - r.first + 1;
        if (!legTransit) {
            remaining = 0;
            for (size_t i = routeIdx + 1; i < route.size(); i++)
                if (route[i].clean)
                    remaining++;
        }
        cleanDone += remaining;
        r.done = true;
    }
    LOG("GUIDED", "Skipping run %d (%s)", curRun, why);
    event("skip_run", {{"run", String(curRun), FIELD_INT}, {"reason", why, FIELD_STRING}});
    curRun = -1;
    setCleaningMotors(false);
    startNextLeg();
}

void GuidedCleanManager::onLegComplete() {
    if (phase == Phase::Returning) {
        beginDocking();
        return;
    }
    if (legTransit) {
        loadRunLeg();
        return;
    }
    if (curRun >= 0)
        runs[static_cast<size_t>(curRun)].done = true;
    curRun = -1;
    startNextLeg();
}

// An obstacle blocks a transit leg: plan around it (it is now in obstacle memory)
void GuidedCleanManager::onTransitBlocked() {
    legReplans++;
    std::vector<Waypoint> path;
    // Getting home matters most: many more attempts on the way back
    int maxReplans = phase == Phase::Returning ? GUIDED_MAX_REPLANS_HOME : GUIDED_MAX_REPLANS;
    if (legReplans <= maxReplans && planPath(legGoalX, legGoalY, path)) {
        int replans = legReplans;
        setLeg(path, true);
        legReplans = replans;
        return;
    }
    if (phase == Phase::Running) {
        skipCurrentRun("blocked");
    } else if (!fallbackHome) {
        fallbackReturn();
    } else {
        // Blocked twice on the way home: the robot's own navigation finds the dock
        LOG("GUIDED", "Way home blocked - handing over to the robot's own docking");
        fallbackDock = true;
        finish(Phase::Done, "way home blocked - the robot docks itself");
    }
}

void GuidedCleanManager::pushCrumb() {
    if (!crumbs.empty()) {
        float dx = static_cast<float>(crumbs.back().x) - poseX;
        float dy = static_cast<float>(crumbs.back().y) - poseY;
        if (sqrtf(dx * dx + dy * dy) < 400.0f)
            return;
    }
    if (crumbs.size() >= 600) {
        // Thin out: keep every second crumb
        size_t k = 0;
        for (size_t i = 0; i < crumbs.size(); i += 2)
            crumbs[k++] = crumbs[i];
        crumbs.resize(k);
    }
    MapPoint p;
    p.x = static_cast<int16_t>(poseX);
    p.y = static_cast<int16_t>(poseY);
    crumbs.push_back(p);
}

// Planning failed: retrace the breadcrumbs (shortcut over covered floor)
void GuidedCleanManager::fallbackReturn() {
    fallbackHome = true;
    LOG("GUIDED", "Planner could not find a way home - retracing %u crumbs", static_cast<unsigned>(crumbs.size()));
    std::vector<Waypoint> back;
    back.reserve(crumbs.size() + 1);
    for (size_t i = crumbs.size(); i-- > 0;) {
        Waypoint w;
        w.x = crumbs[i].x;
        w.y = crumbs[i].y;
        back.push_back(w);
    }
    fromDock(GUIDED_DOCK_STAGING_MM, 0.0f, legGoalX, legGoalY);
    Waypoint staging;
    staging.x = static_cast<int16_t>(legGoalX);
    staging.y = static_cast<int16_t>(legGoalY);
    back.push_back(staging);
    setLeg(back, true);
    legReplans = GUIDED_MAX_REPLANS_HOME + 1; // No further re-planning on this leg
}

void GuidedCleanManager::freePlan() {
    std::vector<Waypoint>().swap(route);
    std::vector<Waypoint>().swap(cleanPts);
    std::vector<Run>().swap(runs);
    std::vector<MapPoint>().swap(crumbs);
    grid.clear();
    field = LocField();
    curRun = -1;
    fallbackHome = false;
    std::vector<MapZone>().swap(nogos);
    std::vector<Obstacle>().swap(obstacles);
    cover.clear();
    dyn.clear();
    pf.clear();
    routeIdx = 0;
    targetIdx = 0;
    cleanTotal = 0;
    cleanDone = 0;
}

// -- Commands -----------------------------------------------------------------------

String GuidedCleanManager::command(const String& action, const String& rooms) {
    if (action == "start") {
        if (isActive())
            return "guided clean already running";
        if (manual.isActive())
            return "manual mode is active";

        FloorMapData map;
        String err;
        if (!floorMap.load(map, err))
            return err;
        err = buildPlan(map, rooms);
        if (!err.isEmpty())
            return err;

        runGen++;
        waitGen++;
        wait = Wait::None;
        afterDelay = nullptr;
        scanJob = nullptr;
        onMoveDone = nullptr;
        phase = Phase::Starting;
        lastError = "";
        roomsLabel = rooms;
        startedMs = millis();
        startedEpoch = sysMgr.now();
        finishedMs = 0;
        finishingSince = 0;
        timeoutCount = 0;
        obstacleStreak = 0;
        obstacleTotal = 0;
        fallbackDock = false;
        dockWatchSince = 0;
        dockAttempts = 0;
        dockIcpFails = 0;
        dockScanTries = 0;
        refScanValid = false;
        ldsOn = false;
        locFixes = 0;
        locRejected = 0;
        locScore = 0.0f;
        locSpread = 0.0f;
        lostCount = 0;
        relocTries = 0;
        globalTried = false;
        needFix = false;
        needEscape = false;
        escapes = 0;
        move = MoveKind::None;
        moveEnding = false;
        motorsOn = false;
        pollPending = false;
        encValid = false;
        odoX = odoY = odoT = 0.0f;
        // The robot stands on the dock: start from the learned dock pose (or the origin)
        poseX = dockX;
        poseY = dockY;
        poseTheta = dockT;
        fixX = poseX;
        fixY = poseY;
        fixT = poseTheta;
        pf.clear();
        progress();
        traceBegin();
        event("start", {{"rooms", rooms.isEmpty() ? String("all") : rooms, FIELD_STRING},
                        {"field", field.valid() ? "true" : "false", FIELD_BOOL},
                        {"nogos", String(static_cast<unsigned>(nogos.size())), FIELD_INT},
                        {"runs", String(static_cast<unsigned>(runs.size())), FIELD_INT},
                        {"clean", String(cleanTotal), FIELD_INT},
                        {"dock", dockKnown ? "learned" : "unknown", FIELD_STRING}});
        traceFlush(); // Evidence even if the ESP32 dies during the run
        beginStart();
        return "";
    }

    if (action == "home") {
        switch (phase) {
            case Phase::Starting:
                finish(Phase::Stopped, "stopped by user");
                return "";
            case Phase::Localizing:
            case Phase::Running:
                beginReturn("returned on request");
                return "";
            case Phase::Returning:
            case Phase::Docking:
            case Phase::Finishing:
                return ""; // Already on the way (the watchdog ends any stall)
            default:
                return "guided clean is not running";
        }
    }

    if (action == "stop") {
        if (!isActive())
            return "guided clean is not running";
        finish(Phase::Stopped, "stopped by user");
        return "";
    }

    return "unknown action";
}

// -- Waits + watchdog -------------------------------------------------------------------
// The controller does exactly one thing at a time and every wait has a deadline.
// A missed answer can therefore never leave the robot hanging in TestMode: the
// wait times out, the step is treated as failed, and repeated timeouts or a run
// without progress end the run (motors off, TestMode left).

void GuidedCleanManager::beginWait(Wait w, unsigned long ms, const char *what) {
    cmdFallback = nullptr;
    wait = w;
    waitDeadline = millis() + ms;
    waitWhat = what;
    waitGen++;
}

void GuidedCleanManager::endWait() {
    cmdFallback = nullptr;
    wait = Wait::None;
    waitWhat = "";
    waitGen++;
    progress();
}

void GuidedCleanManager::delayThen(unsigned long ms, const char *what, std::function<void()> fn) {
    afterDelay = std::move(fn);
    beginWait(Wait::Delay, ms, what);
}

// Stand still briefly, then read the encoders once more: a move that ended by a
// stop, timeout or bump rolled on after the last poll, and the scan must be
// matched from where the robot really is.
void GuidedCleanManager::settleThen(std::function<void()> fn) {
    auto next = std::make_shared<std::function<void()>>(std::move(fn));
    delayThen(GUIDED_SETTLE_MS, "settle", [this, next] {
        beginWait(Wait::Cmd, GUIDED_CMD_TIMEOUT_MS, "encoders");
        cmdFallback = [next] { (*next)(); };
        uint32_t gen = runGen;
        uint32_t wg = waitGen;
        serial.getMotors(
                [this, gen, wg, next](bool ok, const MotorData& m) {
                    if (gen != runGen || wg != waitGen)
                        return;
                    endWait();
                    if (ok)
                        integrateEncoders(m.leftWheelPositionMM, m.rightWheelPositionMM);
                    (*next)();
                },
                PRIORITY_HIGH);
    });
}

void GuidedCleanManager::onWaitTimeout() {
    Wait w = wait;
    const char *what = waitWhat;
    auto fb = std::move(cmdFallback);
    cmdFallback = nullptr;
    timeoutCount++;
    LOG("GUIDED", "Timeout waiting for %s (%d)", what, timeoutCount);
    event("timeout", {{"what", what, FIELD_STRING}, {"n", String(timeoutCount), FIELD_INT}});
    if (timeoutCount >= GUIDED_MAX_TIMEOUTS) {
        abortRun(String("robot stopped answering (") + what + ")");
        return;
    }
    endWait();
    if (w == Wait::Motion) {
        // Robot did not finish the move in time: stop and treat as blocked
        serial.setMotorWheels(0, 0, 0, nullptr);
        endMove(MoveResult::Blocked);
        return;
    }
    if (w == Wait::Scan) {
        auto job = std::move(scanJob);
        scanJob = nullptr;
        scanReady = false;
        if (job)
            job(false, scanBuf);
        return;
    }
    // Cmd with a safe way to carry on without the answer
    if (fb) {
        fb();
        return;
    }
    // Cmd: the owner's callback will not come (waitGen changed). While cleaning,
    // re-decide from here; elsewhere the sequence cannot continue safely.
    if (phase != Phase::Running && phase != Phase::Returning) {
        abortRun(String("no answer from the robot (") + what + ")");
        return;
    }
    move = MoveKind::None;
    moveEnding = false;
    onMoveDone = nullptr;
    needFix = true;
}

void GuidedCleanManager::abortRun(const String& reason) {
    LOG("GUIDED", "ABORT: %s", reason.c_str());
    finish(Phase::Failed, reason);
}

// -- Start --------------------------------------------------------------------------

void GuidedCleanManager::beginStart() {
    uint32_t gen = runGen;
    beginWait(Wait::Cmd, GUIDED_CMD_TIMEOUT_MS, "state");
    uint32_t wg = waitGen;
    serial.getState([this, gen, wg](bool ok, const RobotState& state) {
        if (gen != runGen || wg != waitGen)
            return;
        if (!ok) {
            fail("robot not responding");
            return;
        }
        if (!state.isIdle()) {
            fail("robot is busy (" + state.uiState + ")");
            return;
        }
        beginWait(Wait::Cmd, GUIDED_CMD_TIMEOUT_MS, "charger");
        uint32_t wg2 = waitGen;
        serial.getCharger([this, gen, wg2](bool ok, const ChargerData& c) {
            if (gen != runGen || wg2 != waitGen)
                return;
            if (!ok) {
                fail("cannot read charger state");
                return;
            }
            if (!c.extPwrPresent) {
                fail("start the robot on its dock");
                return;
            }
            if (c.fuelPercent >= 0 && c.fuelPercent < GUIDED_MIN_BATTERY_PCT + 10) {
                fail("battery too low (" + String(c.fuelPercent) + "%)");
                return;
            }
            beginWait(Wait::Cmd, GUIDED_CMD_TIMEOUT_MS, "TestMode On");
            uint32_t wg3 = waitGen;
            inTestMode = true; // From here on, finishing always sends TestMode Off
            serial.testMode(true, [this, gen, wg3](bool ok) {
                if (gen != runGen || wg3 != waitGen)
                    return;
                if (!ok) {
                    fail("TestMode On failed");
                    return;
                }
                serial.setGuidedCleanActive(true);
                beginWait(Wait::Cmd, GUIDED_CMD_TIMEOUT_MS, "LIDAR on");
                uint32_t wg4 = waitGen;
                serial.setLdsRotation(true, [this, gen, wg4](bool ok) {
                    if (gen != runGen || wg4 != waitGen)
                        return;
                    ldsOn = ok;
                    if (!ok) {
                        fail("cannot start the LIDAR");
                        return;
                    }
                    beginWait(Wait::Cmd, GUIDED_CMD_TIMEOUT_MS, "encoders");
                    uint32_t wg5 = waitGen;
                    serial.getMotors(
                            [this, gen, wg5](bool ok, const MotorData& m) {
                                if (gen != runGen || wg5 != waitGen)
                                    return;
                                endWait();
                                if (!ok) {
                                    fail("cannot read wheel encoders");
                                    return;
                                }
                                encL = m.leftWheelPositionMM;
                                encR = m.rightWheelPositionMM;
                                encValid = true;
                                phase = Phase::Localizing;
                                LOG("GUIDED", "Localizing (%u runs planned)", static_cast<unsigned>(runs.size()));
                                delayThen(GUIDED_LDS_SPINUP_MS, "LIDAR spin-up", [this] { startDockScan(); });
                            },
                            PRIORITY_HIGH);
                });
            });
        });
    });
}

// On the dock: the LIDAR needs a few seconds after spin-up before its scans are
// complete. The first full scan becomes the dock reference (for the final
// approach) and gives the first map fix.
void GuidedCleanManager::startDockScan() {
    scanThen("dock scan", [this](bool ok, const LdsScanData& scan) {
        int valid = ok ? validScanPoints(scan) : 0;
        if (valid < GUIDED_LOC_FIRST_MIN_POINTS) {
            dockScanTries++;
            tracePose("dock_wait", {{"valid", String(valid), FIELD_INT}, {"try", String(dockScanTries), FIELD_INT}});
            if (dockScanTries >= GUIDED_LOC_FIRST_TRIES) {
                fail("the LIDAR delivers no usable scans (is the turret spinning?)");
                return;
            }
            delayThen(800, "LIDAR retry", [this] { startDockScan(); });
            return;
        }
        onDockScan(scan);
    });
}

void GuidedCleanManager::onDockScan(const LdsScanData& scan) {
    traceDockScan(scan, "dockscan", 1500);
    // Dock reference: raw distances by angle index
    for (int i = 0; i < 360; i++)
        refScan[i] = 0;
    int refN = 0;
    for (const LdsScanPoint& p: scan.points) {
        if (p.errorCode == 0 && p.distMM >= 150 && p.distMM <= 6000 && p.angleDeg >= 0 && p.angleDeg < 360) {
            refScan[p.angleDeg] = static_cast<int16_t>(p.distMM);
            refN++;
        }
    }
    refScanValid = refN >= GUIDED_LOC_FIRST_MIN_POINTS;

    // First map fix: around the learned dock pose, otherwise a wide search around
    // the map origin (the mapping run started on the dock)
    extractBeams(scan, field, 2, beams);
    float bx = 0.0f;
    float by = 0.0f;
    float bt = 0.0f;
    float score = 0.0f;
    if (dockKnown) {
        score = correlativeMatch(field, beams, dockX, dockY, dockT, GUIDED_LOC_START_RADIUS_MM,
                                 GUIDED_LOC_START_ANGLE_DEG, bx, by, bt, floorTest);
    }
    if (score < GUIDED_START_MIN_SCORE) {
        float gx = 0.0f;
        float gy = 0.0f;
        float gt = 0.0f;
        float gs = correlativeMatch(field, beams, 0.0f, 0.0f, 0.0f, GUIDED_LOC_GLOBAL_RADIUS_MM,
                                    GUIDED_LOC_GLOBAL_ANGLE_DEG, gx, gy, gt, floorTest);
        if (gs > score) {
            score = gs;
            bx = gx;
            by = gy;
            bt = gt;
        }
    }
    tracePose("dock_match", {{"mx", String(static_cast<int>(bx)), FIELD_INT},
                             {"my", String(static_cast<int>(by)), FIELD_INT},
                             {"mt", String(static_cast<int>(bt / DEG)), FIELD_INT},
                             {"s", String(score, 2), FIELD_FLOAT}});
    poseX = bx;
    poseY = by;
    poseTheta = bt;
    fixX = bx;
    fixY = by;
    fixT = bt;
    // Generous cloud: the turn-around scans narrow it down
    pf.seed(bx, by, bt, GUIDED_PF_SEED_MM, GUIDED_PF_SEED_DEG * DEG);
    applyFix(scan, "dock");

    // Undock straight ahead (the robot sits rear-first on the dock), then turn around
    startMove(MoveKind::Drive, GUIDED_UNDOCK_MM, GUIDED_UNDOCK_MM, GUIDED_SPEED_SLOW, [this](MoveResult) {
        // A bump while undocking is no reason to stop: localize from where we are
        beginSpin(true);
    });
}

// Turn once around on the spot in four stops, scanning at each: the LIDAR sees
// the room from all sides while the robot stays put, so the particle filter
// converges on the one pose that explains all four views.
void GuidedCleanManager::beginSpin(bool forStart) {
    spinForStart = forStart;
    spinLeft = 4;
    setCleaningMotors(false);
    tracePose("spin", {{"start", forStart ? "true" : "false", FIELD_BOOL}, {"try", String(relocTries), FIELD_INT}});
    // Scan here first, then turn
    settleThen([this] {
        scanThen("spin scan", [this](bool ok, const LdsScanData& scan) {
            if (ok)
                applyFix(scan, "spin");
            spinStep();
        });
    });
}

void GuidedCleanManager::spinStep() {
    if (spinLeft <= 0) {
        onSpinDone();
        return;
    }
    spinLeft--;
    turnBy(90.0f * DEG, [this](MoveResult) {
        settleThen([this] {
            scanThen("spin scan", [this](bool ok, const LdsScanData& scan) {
                if (ok)
                    applyFix(scan, "spin");
                spinStep();
            });
        });
    });
}

void GuidedCleanManager::onSpinDone() {
    float x = 0.0f;
    float y = 0.0f;
    float t = 0.0f;
    float sxy = 0.0f;
    float st = 0.0f;
    pf.estimate(x, y, t, sxy, st);
    bool converged =
            sxy <= GUIDED_START_SPREAD_MM && st <= GUIDED_START_SPREAD_DEG * DEG && locScore >= GUIDED_LOST_QUALITY;
    tracePose("spin_done", {{"sp", String(static_cast<int>(sxy)), FIELD_INT},
                            {"spt", String(st / DEG, 1), FIELD_FLOAT},
                            {"s", String(locScore, 2), FIELD_FLOAT},
                            {"ok", converged ? "true" : "false", FIELD_BOOL}});
    if (converged) {
        lostCount = 0;
        if (spinForStart) {
            learnDockPose(true);
            phase = Phase::Running;
            batteryTicker.reset();
            batteryTicker.elapsed(GUIDED_BATTERY_POLL_MS); // Arm: first check after one interval
            LOG("GUIDED", "Localized (±%.0f mm) - cleaning %u runs", sxy, static_cast<unsigned>(runs.size()));
            startNextLeg();
            return;
        }
        relocTries = 0;
        globalTried = false;
        // Re-localized during the run: plan the current leg again from the corrected pose
        if (phase == Phase::Running || phase == Phase::Returning) {
            if (legTransit || phase == Phase::Returning) {
                std::vector<Waypoint> path;
                if (planPath(legGoalX, legGoalY, path)) {
                    int replans = legReplans;
                    setLeg(path, true);
                    legReplans = replans;
                }
            } else {
                selectNextTarget();
            }
        }
        return;
    }
    relocTries++;
    if (relocTries > GUIDED_RELOC_TRIES) {
        if (spinForStart || phase == Phase::Localizing) {
            // Still right in front of the dock: back onto it with the reference scan.
            // The dock frame follows from the short odometry since leaving it.
            learnDockPose(false);
            lastError = "position not found on the map - record a new mapping run";
            phase = Phase::Returning;
            beginDocking();
            return;
        }
        if (!globalTried) {
            // Last resort: search the whole floor (the robot was carried, slid far,
            // or the cloud collapsed on a look-alike spot)
            globalTried = true;
            if (globalRelocalize()) {
                relocTries = GUIDED_RELOC_TRIES;
                spinLeft = 4;
                spinStep();
                return;
            }
        }
        abortRun("lost: the position no longer matches the map");
        return;
    }
    // Not sure yet: re-seed from an exhaustive search around the estimate, turn again
    reseedAround(x, y, t, GUIDED_RELOC_RADIUS_MM * relocTries, GUIDED_RELOC_ANGLE_DEG * relocTries, GUIDED_PF_SEED_MM);
    spinLeft = 4;
    spinStep();
}

bool GuidedCleanManager::globalRelocalize() {
    if (beams.n < 30 || grid.w == 0)
        return false;
    const auto cell = static_cast<float>(field.cell);
    int stride = beams.n > 40 ? beams.n / 40 : 1;
    float bestS = -1.0f;
    float bx = 0.0f;
    float by = 0.0f;
    float bt = 0.0f;
    for (int a = 0; a < 360; a += 6) {
        float th = static_cast<float>(a) * DEG;
        float c = cosf(th);
        float sn = sinf(th);
        for (int cy = 0; cy < grid.h; cy++) {
            for (int cx = 0; cx < grid.w; cx++) {
                if (!grid.free[static_cast<size_t>(grid.index(cx, cy))])
                    continue;
                float x = grid.centerX(cx);
                float y = grid.centerY(cy);
                int sum = 0;
                int n = 0;
                for (int i = 0; i < beams.n; i += stride) {
                    int fx = static_cast<int>(
                            floorf((x + c * beams.x[i] - sn * beams.y[i] - static_cast<float>(field.originX)) / cell));
                    int fy = static_cast<int>(
                            floorf((y + sn * beams.x[i] + c * beams.y[i] - static_cast<float>(field.originY)) / cell));
                    sum += field.cellv(fx, fy);
                    n++;
                }
                float sc = static_cast<float>(sum) / static_cast<float>(n * 255);
                if (sc > bestS) {
                    bestS = sc;
                    bx = x;
                    by = y;
                    bt = th;
                }
            }
        }
    }
    float ox = bx;
    float oy = by;
    float ot = bt;
    float sc = correlativeMatch(field, beams, bx, by, bt, 300, 8, ox, oy, ot, floorTest);
    tracePose("global", {{"mx", String(static_cast<int>(ox)), FIELD_INT},
                         {"my", String(static_cast<int>(oy)), FIELD_INT},
                         {"mt", String(static_cast<int>(ot / DEG)), FIELD_INT},
                         {"s", String(sc, 2), FIELD_FLOAT}});
    if (sc < GUIDED_GLOBAL_MIN_SCORE)
        return false;
    poseX = fixX = ox;
    poseY = fixY = oy;
    poseTheta = fixT = ot;
    pf.seed(ox, oy, ot, 80.0f, 4.0f * DEG);
    return true;
}

// The dock pose in the map follows from the converged pose and the (short, precise)
// odometry since leaving the dock. Learned per map and refined every run.
void GuidedCleanManager::learnDockPose(bool save) {
    // D = P ⊕ inverse(O): P = map pose now, O = odometry pose in the dock frame
    float c = cosf(odoT);
    float s = sinf(odoT);
    float ix = -(c * odoX + s * odoY);
    float iy = -(-s * odoX + c * odoY);
    float fx = poseX + cosf(poseTheta) * ix - sinf(poseTheta) * iy;
    float fy = poseY + sinf(poseTheta) * ix + cosf(poseTheta) * iy;
    float ft = wrapAngle(poseTheta - odoT);
    // Turning in place slips (a full turn can be off by several degrees), so the
    // odometry back to the dock is only a guess: match the dock reference scan
    // against the map around it for the real dock pose.
    if (refScanValid) {
        ScanBeams rb;
        auto sgn = static_cast<float>(field.lidarSign);
        for (int i = 0; i < 360 && rb.n < ScanBeams::MAX; i += 2) {
            if (refScan[i] <= 0 || refScan[i] > 5000)
                continue;
            float a = sgn * static_cast<float>(i) * DEG + field.lidarAngleOffsetRad;
            rb.x[rb.n] = static_cast<float>(field.lidarOffsetX) + static_cast<float>(refScan[i]) * cosf(a);
            rb.y[rb.n] = static_cast<float>(refScan[i]) * sinf(a);
            rb.n++;
        }
        float mx = fx;
        float my = fy;
        float mt = ft;
        float sc = correlativeMatch(field, rb, fx, fy, ft, GUIDED_DOCK_REMATCH_MM, GUIDED_DOCK_REMATCH_DEG, mx, my, mt,
                                    floorTest);
        tracePose("dock_rematch", {{"mx", String(static_cast<int>(mx)), FIELD_INT},
                                   {"my", String(static_cast<int>(my)), FIELD_INT},
                                   {"mt", String(mt / DEG, 1), FIELD_FLOAT},
                                   {"ot", String(ft / DEG, 1), FIELD_FLOAT},
                                   {"s", String(sc, 2), FIELD_FLOAT}});
        if (sc >= GUIDED_RESET_MIN_SCORE) {
            fx = mx;
            fy = my;
            ft = mt;
        }
    }
    if (!save) {
        dockX = fx;
        dockY = fy;
        dockT = ft;
        return;
    }
    const char *src = "measured";
    if (dockKnown) {
        float dx = fx - dockX;
        float dy = fy - dockY;
        float dt = wrapAngle(ft - dockT);
        if (sqrtf(dx * dx + dy * dy) < GUIDED_DOCK_AGREE_MM && fabsf(dt) < GUIDED_DOCK_AGREE_DEG * DEG) {
            float w = 1.0f / static_cast<float>((dockRuns < 5 ? dockRuns : 5) + 1);
            fx = dockX + w * dx;
            fy = dockY + w * dy;
            ft = wrapAngle(dockT + w * dt);
            dockRuns++;
            src = "refined";
        } else {
            dockRuns = 1; // Dock moved, or the stored pose was wrong: this converged fix wins
            src = "replaced";
        }
    } else {
        dockRuns = 1;
    }
    dockX = fx;
    dockY = fy;
    dockT = ft;
    dockKnown = true;
    floorMap.saveDock(dockX, dockY, dockT, dockRuns);
    tracePose("dock_pose", {{"src", src, FIELD_STRING},
                            {"dx", String(static_cast<int>(dockX)), FIELD_INT},
                            {"dy", String(static_cast<int>(dockY)), FIELD_INT},
                            {"dt", String(dockT / DEG, 1), FIELD_FLOAT},
                            {"runs", String(dockRuns), FIELD_INT}});
}

// -- Main loop ----------------------------------------------------------------------

void GuidedCleanManager::tick() {
    // Once after boot: if the ESP32 restarted during a guided run (crash, power
    // loss, OTA), the robot is still in TestMode with the LIDAR spinning and
    // ignores its own buttons/schedule. Release it.
    if (!bootChecked && phase == Phase::Idle && serial.hasSKey() && millis() > 5000) {
        bootChecked = true;
        serial.getState([this](bool ok, const RobotState& st) {
            if (!ok || st.uiState.indexOf("TESTMODE") < 0 || manual.isActive() || isActive())
                return;
            LOG("GUIDED", "Robot still in TestMode after an ESP32 restart - leaving TestMode");
            serial.setMotorWheels(0, 0, 0, nullptr);
            serial.setLdsRotation(false, [this](bool) { serial.testMode(false); });
        });
    }

    // After handing docking over to the robot: verify it really arrives
    if (dockWatchSince != 0 && !isActive() && dockWatchTicker.elapsed(5000)) {
        unsigned long since = millis() - dockWatchSince;
        serial.getCharger([this, since](bool ok, const ChargerData& c) {
            if (dockWatchSince == 0)
                return;
            if (ok && c.extPwrPresent) {
                dockWatchSince = 0;
                event("dock_result", {{"docked", "true", FIELD_BOOL}, {"after_s", String(since / 1000), FIELD_INT}});
                traceFlush();
                return;
            }
            if (since > 60000 && !dockWatchRetried) {
                dockWatchRetried = true;
                serial.clean("dock", nullptr);
                return;
            }
            if (since > 150000) {
                dockWatchSince = 0;
                lastError = "the robot did not reach the dock";
                event("dock_result", {{"docked", "false", FIELD_BOOL}, {"after_s", String(since / 1000), FIELD_INT}});
                traceFlush();
            }
        });
    }

    // TestMode Off got no answer while finishing (serial outage): keep trying
    if (releaseSince != 0 && !isActive() && releaseTicker.elapsed(GUIDED_RELEASE_RETRY_MS)) {
        if (millis() - releaseSince > GUIDED_RELEASE_GIVEUP_MS) {
            releaseSince = 0;
        } else {
            serial.setMotorWheels(0, 0, 0, nullptr);
            serial.setLdsRotation(false, nullptr);
            serial.testMode(false, [this](bool ok) {
                if (ok && releaseSince != 0) {
                    LOG("GUIDED", "TestMode left after retry");
                    releaseSince = 0;
                    inTestMode = false;
                }
            });
        }
    }

    if (!isActive())
        return;

    if (phase == Phase::Finishing) {
        // TestMode Off must not hang either
        if (finishingSince != 0 && millis() - finishingSince > GUIDED_FINISH_TIMEOUT_MS) {
            LOG("GUIDED", "Finishing timed out - retrying TestMode Off in the background");
            finishingSince = 0;
            releaseSince = millis(); // Keep trying: the robot must not stay in TestMode
            serial.setGuidedCleanActive(false);
            phase = lastError.isEmpty() ? Phase::Done : Phase::Failed;
            freePlan();
        }
        return;
    }

    // Watchdogs: a wait past its deadline, or no progress at all for too long
    if (wait != Wait::None && static_cast<long>(millis() - waitDeadline) > 0) {
        if (wait == Wait::Delay) {
            auto fn = std::move(afterDelay);
            afterDelay = nullptr;
            endWait();
            if (fn)
                fn();
            return;
        }
        onWaitTimeout();
        return;
    }
    if (millis() - lastProgressMs > GUIDED_STALL_MS) {
        abortRun(String("no progress for ") + String(GUIDED_STALL_MS / 1000) + " s (" + waitWhat + ")");
        return;
    }
    if (millis() - startedMs > GUIDED_MAX_RUNTIME_MS + 900000UL) {
        abortRun("run took far too long");
        return;
    }

    // Scan results are processed here, not inside the serial callback chain
    // (shallow stack; see scanThen)
    if (scanReady && wait == Wait::Scan) {
        auto job = std::move(scanJob);
        scanJob = nullptr;
        scanReady = false;
        endWait();
        job(scanJobOk, scanBuf);
        return;
    }

    // Sensor polling while the wheels move (odometry + safety)
    if (pollPending && millis() - pollSentMs > 3000)
        pollPending = false; // Never let a lost poll block the next one
    if (wait == Wait::Motion && !pollPending && pollTicker.elapsed(GUIDED_POLL_MS) &&
        serial.queueDepth() <= NEATO_QUEUE_MAX_SIZE / 2)
        pollSensors();

    if (wait != Wait::None)
        return;

    // Runtime + battery guards (only while cleaning; returning always continues)
    if (phase == Phase::Running) {
        if (millis() - startedMs >= GUIDED_MAX_RUNTIME_MS) {
            beginReturn("max runtime reached");
            return;
        }
        if (batteryTicker.elapsed(GUIDED_BATTERY_POLL_MS)) {
            uint32_t gen = runGen;
            serial.getCharger([this, gen](bool ok, const ChargerData& c) {
                if (gen != runGen || phase != Phase::Running)
                    return;
                if (ok && c.fuelPercent >= 0 && c.fuelPercent < GUIDED_MIN_BATTERY_PCT)
                    beginReturn("battery low");
            });
        }
    }

    step();
}

// -- Sensor polling -------------------------------------------------------------------

void GuidedCleanManager::pollSensors() {
    uint32_t gen = runGen;
    pollPending = true;
    pollSentMs = millis();
    // Motors twice per cycle: odometry + move completion matter most
    uint8_t slot = pollSlot++ % 4;
    if (slot == 0 || slot == 2) {
        serial.getMotors(
                [this, gen](bool ok, const MotorData& m) {
                    if (gen != runGen)
                        return;
                    pollPending = false;
                    if (ok)
                        onMotors(m);
                },
                PRIORITY_HIGH);
    } else if (slot == 1) {
        serial.getDigitalSensors(
                [this, gen](bool ok, const DigitalSensorData& d) {
                    if (gen != runGen)
                        return;
                    pollPending = false;
                    if (ok)
                        onDigital(d);
                },
                PRIORITY_HIGH);
    } else {
        serial.getBatteryAnalog(
                [this, gen](bool ok, const BatteryAnalogData& a) {
                    if (gen != runGen)
                        return;
                    pollPending = false;
                    if (ok)
                        onAnalog(a);
                },
                PRIORITY_HIGH);
    }
}

void GuidedCleanManager::integrateEncoders(int newL, int newR) {
    if (!encValid) {
        encL = newL;
        encR = newR;
        encValid = true;
        return;
    }
    int dl = newL - encL;
    int dr = newR - encR;
    encL = newL;
    encR = newR;
    // Ignore implausible jumps (counter reset after wheel disable/enable, glitches)
    if (abs(dl) > 400 || abs(dr) > 400) {
        LOG("GUIDED", "Encoder jump ignored (dl=%d dr=%d)", dl, dr);
        moveStartL += dl;
        moveStartR += dr;
        return;
    }
    float dTheta = static_cast<float>(dr - dl) / GUIDED_WHEEL_BASE_MM;
    float ds = static_cast<float>(dl + dr) / 2.0f;
    float mid = poseTheta + dTheta / 2.0f;
    poseX += ds * cosf(mid);
    poseY += ds * sinf(mid);
    poseTheta = wrapAngle(poseTheta + dTheta);
    float omid = odoT + dTheta / 2.0f;
    odoX += ds * cosf(omid);
    odoY += ds * sinf(omid);
    odoT = wrapAngle(odoT + dTheta);
    traceDist += fabsf(ds);
    traceTurn += fabsf(dTheta);
    if (traceDist >= GUIDED_TRACE_EVERY_MM || traceTurn >= GUIDED_TRACE_EVERY_DEG * DEG) {
        traceDist = 0.0f;
        traceTurn = 0.0f;
        tracePose("p", {{"c", motorsOn ? "true" : "false", FIELD_BOOL}});
    }
}

void GuidedCleanManager::onMotors(const MotorData& m) {
    int prevL = encL;
    int prevR = encR;
    integrateEncoders(m.leftWheelPositionMM, m.rightWheelPositionMM);
    if (move == MoveKind::None || moveEnding || wait != Wait::Motion)
        return;

    bool moved = (encL != prevL) || (encR != prevR);
    moveStillPolls = moved ? 0 : moveStillPolls + 1;
    if (moved)
        progress();

    int progL = abs(encL - moveStartL);
    int progR = abs(encR - moveStartR);
    if (progL >= abs(moveCmdL) - 15 && progR >= abs(moveCmdR) - 15) {
        endMove(MoveResult::Ok);
        return;
    }

    // Wheel overload while driving = pushing against something
    if (move == MoveKind::Drive || move == MoveKind::Turn) {
        bool overloaded = m.leftWheelLoad >= GUIDED_STALL_LOAD_PCT || m.rightWheelLoad >= GUIDED_STALL_LOAD_PCT;
        moveStallPolls = overloaded ? moveStallPolls + 1 : 0;
        if (moveStallPolls >= 2) {
            onContact("stall", false);
            return;
        }
    }

    // Stopped short (blocked without bumper contact, or the command was dropped)
    if (millis() - moveStartMs > 800 && moveStillPolls >= 3) {
        int expected = abs(moveCmdL) + abs(moveCmdR);
        int done = progL + progR;
        if (move == MoveKind::DockBack) {
            endMove(MoveResult::Ok); // Stopped by the dock: verify the contacts
            return;
        }
        if (done * 2 < expected && (move == MoveKind::Drive || move == MoveKind::Reverse)) {
            onContact("blocked", false);
            return;
        }
        endMove(MoveResult::Ok);
    }
}

void GuidedCleanManager::onDigital(const DigitalSensorData& d) {
    if (d.leftWheelExtended || d.rightWheelExtended) {
        LOG("GUIDED", "SAFETY: wheel lifted - aborting");
        serial.setMotorWheels(0, 0, 0, nullptr);
        fail("robot lifted or wheel dropped");
        return;
    }
    if (move == MoveKind::None || moveEnding || wait != Wait::Motion)
        return;
    if (move == MoveKind::DockBack && d.dcJackIn) {
        // On the contacts: stop pushing and verify
        serial.setMotorWheels(0, 0, 0, nullptr);
        endMove(MoveResult::Contact);
        return;
    }
    if (move == MoveKind::Backoff || move == MoveKind::DockBack || move == MoveKind::Reverse)
        return;
    bool bump = d.lFrontBit || d.rFrontBit || d.lLdsBit || d.rLdsBit || d.lSideBit || d.rSideBit;
    if (bump)
        onContact("bumper", false);
}

void GuidedCleanManager::onAnalog(const BatteryAnalogData& a) {
    bool cliff = (a.dropLeftMM > GUIDED_DROP_MM) || (a.dropRightMM > GUIDED_DROP_MM);
    if (!cliff || move == MoveKind::None || moveEnding || wait != Wait::Motion)
        return;
    if (move == MoveKind::Backoff || move == MoveKind::Reverse)
        return;
    if (move == MoveKind::DockBack) {
        // Backing over the dock ramp lifts the front sensors briefly — stop and verify instead
        serial.setMotorWheels(0, 0, 0, nullptr);
        endMove(MoveResult::Ok);
        return;
    }
    LOG("GUIDED", "SAFETY: cliff detected (L=%d R=%d mm)", a.dropLeftMM, a.dropRightMM);
    onContact("cliff", true);
}

// -- Motion executor ------------------------------------------------------------------

void GuidedCleanManager::startMove(MoveKind kind, int leftMM, int rightMM, int speed,
                                   std::function<void(MoveResult)> done) {
    move = kind;
    moveCmdL = leftMM;
    moveCmdR = rightMM;
    moveStartL = encL;
    moveStartR = encR;
    moveStartMs = millis();
    moveStillPolls = 0;
    moveStallPolls = 0;
    moveEnding = false;
    onMoveDone = std::move(done);
    int longest = abs(leftMM) > abs(rightMM) ? abs(leftMM) : abs(rightMM);
    unsigned long expect =
            static_cast<unsigned long>(longest) * 1000UL / static_cast<unsigned long>(speed > 0 ? speed : 1);
    beginWait(Wait::Motion, expect + GUIDED_SEGMENT_SLACK_MS, "move");
    pollTicker.reset(); // Poll promptly after a new command
    uint32_t gen = runGen;
    serial.setMotorWheels(leftMM, rightMM, speed, [this, gen](bool ok) {
        if (gen != runGen || ok || wait != Wait::Motion)
            return;
        // Command rejected (queue full / timeout): nothing moves — report as blocked
        LOG("GUIDED", "SetMotor failed");
        endMove(MoveResult::Blocked);
    });
}

void GuidedCleanManager::turnBy(float rad, std::function<void(MoveResult)> done) {
    int arc = static_cast<int>(wrapAngle(rad) * GUIDED_WHEEL_BASE_MM / 2.0f);
    startMove(MoveKind::Turn, -arc, arc, GUIDED_SPEED_TURN, std::move(done));
}

void GuidedCleanManager::endMove(MoveResult r) {
    if (move == MoveKind::None)
        return;
    move = MoveKind::None;
    moveEnding = false;
    if (wait == Wait::Motion)
        endWait();
    auto done = std::move(onMoveDone);
    onMoveDone = nullptr;
    if (done)
        done(r);
}

void GuidedCleanManager::onContact(const char *reason, bool cliff) {
    obstacleTotal++;
    LOG("GUIDED", "Obstacle (%s) at %.0f,%.0f → back off", reason, poseX, poseY);
    event("obstacle", {{"reason", reason, FIELD_STRING},
                       {"x", String(static_cast<int>(poseX)), FIELD_INT},
                       {"y", String(static_cast<int>(poseY)), FIELD_INT}});
    bool behind = move == MoveKind::Reverse || move == MoveKind::DockBack || move == MoveKind::Backoff;
    rememberObstacle(GUIDED_ROBOT_RADIUS_MM + (cliff ? 60 : GUIDED_OBSTACLE_RADIUS_MM),
                     cliff ? GUIDED_CLIFF_RADIUS_MM : GUIDED_OBSTACLE_RADIUS_MM, behind);
    moveEnding = true;
    blockReason = reason;
    auto owner = std::move(onMoveDone);
    onMoveDone = nullptr;
    MoveResult result = cliff ? MoveResult::Cliff : MoveResult::Blocked;
    // Stop (disable/enable sequence), then reverse a little, then tell the move's owner
    beginWait(Wait::Cmd, GUIDED_CMD_TIMEOUT_MS, "stop");
    auto backoff = [this, owner, result, behind] {
        move = MoveKind::None;
        int back = behind ? GUIDED_BACKOFF_MM : -GUIDED_BACKOFF_MM;
        startMove(MoveKind::Backoff, back, back, GUIDED_SPEED_SLOW, [owner, result](MoveResult) {
            if (owner)
                owner(result);
        });
    };
    cmdFallback = backoff;
    uint32_t gen = runGen;
    uint32_t wg = waitGen;
    serial.setMotorWheels(0, 0, 0, [this, gen, wg, backoff](bool) {
        if (gen != runGen || wg != waitGen)
            return;
        endWait();
        backoff();
    });
}

// -- Localization -------------------------------------------------------------------

// GetLDSScan with deferred processing: the result is copied into scanBuf and the
// handler runs from tick(). Matching inside the serial callback chain sat on top
// of the ~6 KB parse frame and overflowed the 8 KB loop stack on the ESP32.
void GuidedCleanManager::scanThen(const char *what, std::function<void(bool, const LdsScanData&)> handler) {
    beginWait(Wait::Scan, GUIDED_SCAN_TIMEOUT_MS, what);
    scanJob = std::move(handler); // Runs from tick() on the answer, or with ok=false on timeout
    scanReady = false;
    uint32_t gen = runGen;
    uint32_t wg = waitGen;
    serial.getLdsScanFresh([this, gen, wg](bool ok, const LdsScanData& scan) {
        if (gen != runGen || wg != waitGen || wait != Wait::Scan)
            return;
        if (ok)
            scanBuf = scan; // Trivially copyable: in-place copy, no stack temporary
        scanJobOk = ok;
        scanReady = true;
    });
}

bool GuidedCleanManager::applyFix(const LdsScanData& scan, const char *tag) {
    if (extractBeams(scan, field, 2, beams) < 40) {
        locRejected++;
        tracePose("loc", {{"tag", tag, FIELD_STRING}, {"n", String(beams.n), FIELD_INT}, {"ok", "false", FIELD_BOOL}});
        return false;
    }
    // Motion model: odometry since the last update, in the frame of that pose
    float dx = poseX - fixX;
    float dy = poseY - fixY;
    float c = cosf(fixT);
    float s = sinf(fixT);
    float dxl = c * dx + s * dy;
    float dyl = -s * dx + c * dy;
    float dth = wrapAngle(poseTheta - fixT);
    pf.predict(dxl, dyl, dth);
    pf.update(field, beams, floorTest);
    float x = 0.0f;
    float y = 0.0f;
    float t = 0.0f;
    float sxy = 0.0f;
    float st = 0.0f;
    pf.estimate(x, y, t, sxy, st);
    float before = poseQuality(field, beams, poseX, poseY, poseTheta);
    float q = poseQuality(field, beams, x, y, t);
    float shift = sqrtf((x - poseX) * (x - poseX) + (y - poseY) * (y - poseY));
    poseX = x;
    poseY = y;
    poseTheta = t;
    fixX = x;
    fixY = y;
    fixT = t;
    locScore = q;
    locSpread = sxy;
    bool good = q >= GUIDED_LOST_QUALITY && sxy <= GUIDED_LOST_SPREAD_MM;
    if (good)
        locFixes++;
    else
        locRejected++;
    tracePose("loc", {{"tag", tag, FIELD_STRING},
                      {"mx", String(static_cast<int>(x)), FIELD_INT},
                      {"my", String(static_cast<int>(y)), FIELD_INT},
                      {"mt", String(static_cast<int>(t / DEG)), FIELD_INT},
                      {"s", String(q, 2), FIELD_FLOAT},
                      {"s0", String(before, 2), FIELD_FLOAT},
                      {"sp", String(static_cast<int>(sxy)), FIELD_INT},
                      {"d", String(static_cast<int>(shift)), FIELD_INT},
                      {"ok", good ? "true" : "false", FIELD_BOOL}});
    return good;
}

void GuidedCleanManager::reseedAround(float x, float y, float t, int radiusMM, int angleDeg, float sigmaXY) {
    float ox = x;
    float oy = y;
    float ot = t;
    float sc =
            beams.n >= 30 ? correlativeMatch(field, beams, x, y, t, radiusMM, angleDeg, ox, oy, ot, floorTest) : 0.0f;
    tracePose("reseed", {{"mx", String(static_cast<int>(ox)), FIELD_INT},
                         {"my", String(static_cast<int>(oy)), FIELD_INT},
                         {"mt", String(static_cast<int>(ot / DEG)), FIELD_INT},
                         {"s", String(sc, 2), FIELD_FLOAT}});
    poseX = ox;
    poseY = oy;
    poseTheta = ot;
    fixX = ox;
    fixY = oy;
    fixT = ot;
    pf.seed(ox, oy, ot, sigmaXY, GUIDED_PF_SEED_DEG * DEG);
}

// Stationary fix after a move: settle, scan, filter update; two bad fixes in a
// row → stop and re-localize by turning on the spot
void GuidedCleanManager::trackingFix() {
    needFix = false;
    settleThen([this] {
        scanThen("scan", [this](bool ok, const LdsScanData& scan) {
            float px = poseX;
            float py = poseY;
            bool good = ok && applyFix(scan, "track");
            if (!ok)
                return; // Scan failed: carry on on odometry, next stop scans again
            if (!good && beams.n >= 30) {
                // Sensor reset: the cloud may have locked onto a wrong pose (more slip than
                // the motion model expects). An exhaustive match nearby finds the right one.
                float ox = 0.0f;
                float oy = 0.0f;
                float ot = 0.0f;
                float sc = correlativeMatch(field, beams, poseX, poseY, poseTheta, GUIDED_RESET_RADIUS_MM,
                                            GUIDED_RESET_ANGLE_DEG, ox, oy, ot, floorTest);
                tracePose("reset", {{"mx", String(static_cast<int>(ox)), FIELD_INT},
                                    {"my", String(static_cast<int>(oy)), FIELD_INT},
                                    {"mt", String(static_cast<int>(ot / DEG)), FIELD_INT},
                                    {"s", String(sc, 2), FIELD_FLOAT},
                                    {"q", String(locScore, 2), FIELD_FLOAT}});
                if (sc >= GUIDED_RESET_MIN_SCORE && sc > locScore + 0.15f) {
                    poseX = fixX = ox;
                    poseY = fixY = oy;
                    poseTheta = fixT = ot;
                    pf.seed(ox, oy, ot, 50.0f, 3.0f * DEG);
                    locScore = sc;
                    good = true;
                }
            }
            lostCount = good ? 0 : lostCount + 1;
            if (good) {
                markScan();
                timeoutCount = 0; // Only repeated timeouts without a good step in between abort
            }
            if (lostCount >= 2) {
                lostCount = 0;
                LOG("GUIDED", "Localization weak (score %.2f, spread %.0f mm) - re-localizing", locScore, locSpread);
                beginSpin(false);
                return;
            }
            // A large correction on a transit leg: the planned path may no longer fit
            float jump = sqrtf((poseX - px) * (poseX - px) + (poseY - py) * (poseY - py));
            if (jump > GUIDED_REPLAN_JUMP_MM && legTransit && !route.empty()) {
                std::vector<Waypoint> path;
                if (planPath(legGoalX, legGoalY, path)) {
                    int replans = legReplans;
                    setLeg(path, true);
                    legReplans = replans;
                }
            }
        });
    });
}

// -- Navigation -------------------------------------------------------------------------

void GuidedCleanManager::markReached(size_t idx) {
    if (route.empty())
        return;
    if (idx >= route.size())
        idx = route.size() - 1;
    if (phase == Phase::Running) {
        for (size_t i = routeIdx + 1; i <= idx; i++) {
            if (route[i].clean)
                cleanDone++;
        }
    }
    if (idx > routeIdx)
        routeIdx = idx;
    pushCrumb();
}

void GuidedCleanManager::selectNextTarget() {
    if (route.empty())
        return;
    size_t last = route.size() - 1;
    size_t next = routeIdx + 1;
    if (next > last) {
        targetIdx = last;
        return;
    }

    // Shortcut transit stretches: jump as far ahead as a straight line stays on
    // known-free floor and clear of no-go zones. Never skip a clean waypoint.
    if (!route[next].clean) {
        size_t limit = next + GUIDED_SHORTCUT_WINDOW;
        if (limit > last)
            limit = last;
        for (size_t i = next; i <= limit; i++) {
            if (route[i].clean) {
                limit = i;
                break;
            }
        }
        for (size_t j = limit; j > next; j--) {
            if (nearObstacle(route[j].x, route[j].y))
                continue;
            if (gridLineFree(poseX, poseY, route[j].x, route[j].y)) {
                markReached(j - 1);
                targetIdx = j;
                return;
            }
        }
    }

    // Default: next waypoint, but never drive a line through a no-go zone
    for (size_t j = next; j <= last && j < next + GUIDED_SHORTCUT_WINDOW; j++) {
        if (nearObstacle(route[j].x, route[j].y) && j < last)
            continue;
        bool clear = (j == next ? segmentClear(poseX, poseY, route[j].x, route[j].y, false)
                                : segmentClear(poseX, poseY, route[j].x, route[j].y, true)) &&
                     lineClearance(poseX, poseY, route[j].x, route[j].y);
        if (clear) {
            if (j > next)
                markReached(j - 1);
            targetIdx = j;
            return;
        }
    }

    // Nothing reachable on a straight line. In transit (and when boxed in) step
    // out first; on a cleaning lane skip the waypoint.
    if (legTransit || phase == Phase::Returning ||
        clearanceAt(poseX, poseY) < static_cast<float>(GUIDED_ROBOT_RADIUS_MM) + 10.0f) {
        LOG("GUIDED", "No clear line from %.0f,%.0f - stepping out", poseX, poseY);
        needEscape = true;
        targetIdx = next;
        return;
    }
    LOG("GUIDED", "No clear line from %.0f,%.0f - skipping waypoint %u", poseX, poseY, static_cast<unsigned>(next));
    obstacleStreak++;
    markReached(next);
    targetIdx = next < last ? next + 1 : last;
}

// Known floor: a free planning cell (reachable from the dock), or right next to one
bool GuidedCleanManager::onFloor(float x, float y) const {
    if (grid.w == 0)
        return true;
    int cx = 0;
    int cy = 0;
    grid.toCell(x, y, cx, cy);
    if (grid.inside(cx, cy) && grid.free[static_cast<size_t>(grid.index(cx, cy))])
        return true;
    // Next to free floor (close to a wall, on the dock): only if no wall lies in between
    for (int dy = -1; dy <= 1; dy++) {
        for (int dx = -1; dx <= 1; dx++) {
            if (!grid.inside(cx + dx, cy + dy) || !grid.free[static_cast<size_t>(grid.index(cx + dx, cy + dy))])
                continue;
            float tx = grid.centerX(cx + dx);
            float ty = grid.centerY(cy + dy);
            bool crosses = false;
            for (int k = 0; k <= 8 && !crosses; k++) {
                float f = static_cast<float>(k) / 8.0f;
                float px = x + (tx - x) * f;
                float py = y + (ty - y) * f;
                crosses = field.valid() && field.inside(px, py) && field.at(px, py) >= GUIDED_FIELD_WALL_MIN;
            }
            if (!crosses)
                return true;
        }
    }
    return false;
}

// Free space around a point: distance to the nearest wall (map), seen obstacle
// (LIDAR layer) or remembered bump
float GuidedCleanManager::clearanceAt(float x, float y) const {
    float c = field.valid() && field.inside(x, y) ? wallDist(x, y) : 300.0f;
    float d = dynNearest(x, y);
    if (d < c)
        c = d;
    for (const auto& o: obstacles) {
        if (!o.live())
            continue;
        float od = sqrtf((x - o.x) * (x - o.x) + (y - o.y) * (y - o.y)) - static_cast<float>(o.r);
        if (od < c)
            c = od;
    }
    return c;
}

// Recovery when boxed in (table legs, chairs, a corner): of 16 headings take the
// one with the most room over the next 30 cm (ties towards the goal), turn,
// drive 25 cm, check the pose, plan again. Bounded per leg.
void GuidedCleanManager::escapeMove() {
    needEscape = false;
    escapes++;
    if (escapes > GUIDED_MAX_ESCAPES) {
        escapes = 0;
        onTransitBlocked();
        return;
    }
    float gx = legGoalX;
    float gy = legGoalY;
    if (!route.empty() && targetIdx < route.size()) {
        gx = route[targetIdx].x;
        gy = route[targetIdx].y;
    }
    float toGoal = atan2f(gy - poseY, gx - poseX);
    float bestScore = -1e9f;
    float bestHeading = poseTheta;
    float bestClear = 0.0f;
    for (int k = 0; k < 16; k++) {
        float hd = static_cast<float>(k) * 22.5f * DEG;
        float minClear = 1e9f;
        for (int d = 50; d <= 300; d += 50) {
            float c = clearanceAt(poseX + static_cast<float>(d) * cosf(hd), poseY + static_cast<float>(d) * sinf(hd));
            if (c < minClear)
                minClear = c;
        }
        if (nearNogo(poseX + 300.0f * cosf(hd), poseY + 300.0f * sinf(hd)))
            continue;
        float score = (minClear > 260.0f ? 260.0f : minClear) + 60.0f * cosf(wrapAngle(hd - toGoal));
        // Do not undo the previous step-out (that oscillates between two spots)
        if (escapes > 1 && cosf(wrapAngle(hd - lastEscapeHeading)) < -0.5f)
            score -= 120.0f;
        if (score > bestScore) {
            bestScore = score;
            bestHeading = hd;
            bestClear = minClear;
        }
    }
    tracePose("escape", {{"h", String(static_cast<int>(bestHeading / DEG)), FIELD_INT},
                         {"c", String(static_cast<int>(bestClear)), FIELD_INT},
                         {"n", String(escapes), FIELD_INT}});
    lastEscapeHeading = bestHeading;
    float turn = wrapAngle(bestHeading - poseTheta);
    turnBy(turn, [this](MoveResult) {
        startMove(MoveKind::Drive, GUIDED_ESCAPE_MM, GUIDED_ESCAPE_MM, GUIDED_SPEED_SLOW, [this](MoveResult r) {
            needFix = true;
            if (r == MoveResult::Blocked || r == MoveResult::Cliff) {
                needEscape = true; // Try another heading (the bump is remembered now)
                return;
            }
            // Plan the leg again from the new spot
            if (legTransit || phase == Phase::Returning) {
                std::vector<Waypoint> path;
                if (planPath(legGoalX, legGoalY, path)) {
                    int replans = legReplans;
                    int esc = escapes;
                    setLeg(path, true);
                    legReplans = replans;
                    escapes = esc;
                }
            } else {
                selectNextTarget();
            }
        });
    });
}

void GuidedCleanManager::step() {
    if (phase != Phase::Running && phase != Phase::Returning)
        return; // Starting/Localizing/Docking are driven by their own callbacks

    if (needFix) {
        trackingFix();
        return;
    }
    if (needEscape) {
        escapeMove();
        return;
    }
    if (route.empty()) {
        if (phase == Phase::Returning)
            beginDocking();
        else
            startNextLeg();
        return;
    }

    // Every waypoint of this leg processed (reached or skipped)
    if (routeIdx >= route.size() - 1 && targetIdx <= routeIdx) {
        onLegComplete();
        return;
    }

    const Waypoint& t = route[targetIdx];
    float dx = static_cast<float>(t.x) - poseX;
    float dy = static_cast<float>(t.y) - poseY;
    float dist = sqrtf(dx * dx + dy * dy);
    if (dist < GUIDED_REACHED_MM) {
        obstacleStreak = 0;
        markReached(targetIdx);
        if (routeIdx >= route.size() - 1) {
            onLegComplete();
            return;
        }
        selectNextTarget();
        return;
    }

    if (phase == Phase::Running)
        setCleaningMotors(t.clean);
    int speed = (phase == Phase::Running && t.clean) ? GUIDED_SPEED_CLEAN : GUIDED_SPEED_TRANSIT;
    moveTowards(t.x, t.y, speed);
}

void GuidedCleanManager::moveTowards(float tx, float ty, int speed) {
    float dx = tx - poseX;
    float dy = ty - poseY;
    float dist = sqrtf(dx * dx + dy * dy);
    float err = wrapAngle(atan2f(dy, dx) - poseTheta);
    if (fabsf(err) > GUIDED_HEADING_TOL_DEG * DEG) {
        float turned = err;
        turnBy(err, [this, turned](MoveResult r) { onNavMoveDone(MoveKind::Turn, turned, r); });
        return;
    }
    // Straight, at most one step: then stand still and check the pose against the map
    int stepMM = static_cast<int>(dist < GUIDED_STEP_MM ? dist : GUIDED_STEP_MM);
    // Small heading errors are steered out over the step (gentle arc, bounded bulge)
    int l = stepMM;
    int r = stepMM;
    if (fabsf(err) > 1.0f * DEG && stepMM > 100) {
        float radius = static_cast<float>(stepMM) / (2.0f * sinf(fabsf(err)));
        float arcAng = 2.0f * fabsf(err);
        float inner = arcAng * (radius - GUIDED_WHEEL_BASE_MM / 2.0f);
        float outer = arcAng * (radius + GUIDED_WHEEL_BASE_MM / 2.0f);
        l = static_cast<int>(err > 0 ? inner : outer);
        r = static_cast<int>(err > 0 ? outer : inner);
    }
    startMove(MoveKind::Drive, l, r, speed, [this](MoveResult res) { onNavMoveDone(MoveKind::Drive, 0.0f, res); });
}

void GuidedCleanManager::onNavMoveDone(MoveKind kind, float turned, MoveResult r) {
    if (phase != Phase::Running && phase != Phase::Returning)
        return;
    if (r == MoveResult::Blocked || r == MoveResult::Cliff) {
        needFix = true; // Bumps knock the pose: check it before deciding
        onBlocked();
        return;
    }
    if (kind == MoveKind::Drive || fabsf(turned) > GUIDED_TURN_FIX_DEG * DEG)
        needFix = true;
}

void GuidedCleanManager::onBlocked() {
    if (legTransit || phase == Phase::Returning) {
        onTransitBlocked();
        return;
    }
    // Cleaning run: skip past the obstructed point and carry on
    obstacleStreak++;
    if (obstacleStreak >= GUIDED_MAX_OBSTACLE_STREAK) {
        skipCurrentRun("too many obstacles");
        return;
    }
    if (route.empty())
        return;
    const Waypoint& blocked = route[targetIdx];
    size_t i = targetIdx;
    while (i + 1 < route.size()) {
        auto dx = static_cast<float>(route[i].x - blocked.x);
        auto dy = static_cast<float>(route[i].y - blocked.y);
        if (sqrtf(dx * dx + dy * dy) >= GUIDED_SKIP_AFTER_OBSTACLE_MM)
            break;
        i++;
    }
    markReached(i > targetIdx ? i - 1 : targetIdx);
    selectNextTarget();
}

void GuidedCleanManager::setCleaningMotors(bool on) {
    if (on == motorsOn)
        return;
    motorsOn = on;
    if (on) {
        serial.setMotorBrush(MANUAL_BRUSH_RPM, nullptr);
        serial.setMotorVacuum(true, MANUAL_VACUUM_SPEED_PCT, nullptr);
        serial.setMotorSideBrush(true, MANUAL_SIDE_BRUSH_POWER_MW, nullptr);
    } else {
        serial.setMotorBrush(0, nullptr);
        serial.setMotorVacuum(false, 0, nullptr);
        serial.setMotorSideBrush(false, 0, nullptr);
    }
}

void GuidedCleanManager::beginReturn(const String& reason) {
    LOG("GUIDED", "Returning to dock (%s)", reason.c_str());
    event("return", {{"reason", reason, FIELD_STRING},
                     {"cleaned", String(cleanDone), FIELD_INT},
                     {"total", String(cleanTotal), FIELD_INT}});
    lastError = reason == "done" ? String("") : reason;
    setCleaningMotors(false);
    curRun = -1;
    bool wasLocalizing = phase == Phase::Localizing;
    phase = Phase::Returning;
    if (move != MoveKind::None || wait != Wait::None) {
        // Interrupt whatever is going on (a "home" request mid-move)
        waitGen++;
        wait = Wait::None;
        afterDelay = nullptr;
        scanJob = nullptr;
        onMoveDone = nullptr;
        move = MoveKind::None;
        moveEnding = false;
        serial.setMotorWheels(0, 0, 0, nullptr);
        progress();
    }
    if (wasLocalizing || !refScanValid) {
        // Still next to the dock (or no reference): line up from here
        route.clear();
        beginDocking();
        return;
    }
    needFix = true;
    fromDock(GUIDED_DOCK_STAGING_MM, 0.0f, legGoalX, legGoalY);
    std::vector<Waypoint> path;
    if (planPath(legGoalX, legGoalY, path)) {
        setLeg(path, true);
        return;
    }
    fallbackReturn();
}

// -- Docking ------------------------------------------------------------------------------
// The robot docks rear-first while facing away from the dock (+x of the dock
// frame). From the staging point it turns to the dock axis, scans and matches
// against the reference scan taken on the dock at the start: same place, same
// view, so the match is accurate to about a centimetre — independent of the map.
// Then it reverses in short steps along the axis, re-matching after each, and
// finally backs straight onto the contacts. The charger confirms.

void GuidedCleanManager::beginDocking() {
    LOG("GUIDED", "Docking from %.0f,%.0f", poseX, poseY);
    phase = Phase::Docking;
    setCleaningMotors(false);
    tracePose("dock", {{"stage", "begin", FIELD_STRING}, {"try", String(dockAttempts), FIELD_INT}});
    dockAlign();
}

void GuidedCleanManager::dockAlign() {
    if (!refScanValid) {
        dockRetry("no dock reference scan");
        return;
    }
    float lx = 0.0f;
    float ly = 0.0f;
    toDock(poseX, poseY, lx, ly);
    float lt = wrapAngle(poseTheta - dockT);
    // Face the dock axis first (heading 0 in the dock frame) so the views match
    if (fabsf(lt) > 6.0f * DEG) {
        turnBy(-lt, [this](MoveResult) { dockAlign(); });
        return;
    }
    settleThen([this] { scanThen("dock scan", [this](bool ok, const LdsScanData& scan) { onDockScan2(ok, scan); }); });
}

void GuidedCleanManager::onDockScan2(bool ok, const LdsScanData& scan) {
    float lx = 0.0f;
    float ly = 0.0f;
    toDock(poseX, poseY, lx, ly);
    float lt = wrapAngle(poseTheta - dockT);
    if (ok)
        applyFix(scan, "dock");
    toDock(poseX, poseY, lx, ly);
    lt = wrapAngle(poseTheta - dockT);
    float mx = lx;
    float my = ly;
    float mt = lt;
    if (ok && matchReference(scan, mx, my, mt)) {
        dockIcpFails = 0;
        // Trust the reference match: it is the dock frame by construction
        fromDock(mx, my, poseX, poseY);
        poseTheta = wrapAngle(mt + dockT);
        fixX = poseX;
        fixY = poseY;
        fixT = poseTheta;
        pf.seed(poseX, poseY, poseTheta, 20.0f, 1.0f * DEG);
        tracePose("dock_icp", {{"lx", String(static_cast<int>(mx)), FIELD_INT},
                               {"ly", String(static_cast<int>(my)), FIELD_INT},
                               {"lt", String(mt / DEG, 1), FIELD_FLOAT}});
        dockDecide(mx, my, mt);
        return;
    }
    dockIcpFails++;
    tracePose("dock_icp", {{"ok", "false", FIELD_BOOL}, {"n", String(dockIcpFails), FIELD_INT}});
    if (dockIcpFails >= 3) {
        dockRetry("dock reference does not match");
        return;
    }
    // Probably too far off: get onto the staging point via the map pose and try again
    float sx = 0.0f;
    float sy = 0.0f;
    fromDock(GUIDED_DOCK_STAGING_MM, 0.0f, sx, sy);
    float dx = sx - poseX;
    float dy = sy - poseY;
    if (sqrtf(dx * dx + dy * dy) > 120.0f) {
        float err = wrapAngle(atan2f(dy, dx) - poseTheta);
        float dist = sqrtf(dx * dx + dy * dy);
        turnBy(err, [this, dist](MoveResult) {
            int d = static_cast<int>(dist);
            startMove(MoveKind::Drive, d, d, GUIDED_SPEED_SLOW, [this](MoveResult) { dockAlign(); });
        });
        return;
    }
    dockAlign();
}

// Next docking move from a precise dock-frame pose (lx > 0 = in front of the dock)
void GuidedCleanManager::dockDecide(float lx, float ly, float lt) {
    const float preback = GUIDED_DOCK_PREBACK_MM;
    if (lx > preback + 40.0f) {
        // Reverse pursuit: rear towards a point on the axis behind us, ≤ 250 mm per step
        float tx = lx - 350.0f;
        if (tx < preback)
            tx = preback;
        float want = atan2f(ly, lx - tx); // Heading with the rear towards (tx, 0)
        float err = wrapAngle(want - lt);
        float dist = sqrtf((lx - tx) * (lx - tx) + ly * ly);
        int stepMM = static_cast<int>(dist < 250.0f ? dist : 250.0f);
        auto back = [this, stepMM](MoveResult) {
            startMove(MoveKind::Reverse, -stepMM, -stepMM, GUIDED_SPEED_SLOW, [this](MoveResult) {
                settleThen([this] {
                    scanThen("dock scan", [this](bool ok, const LdsScanData& scan) { onDockScan2(ok, scan); });
                });
            });
        };
        if (fabsf(err) > 2.0f * DEG) {
            turnBy(err, back);
            return;
        }
        back(MoveResult::Ok);
        return;
    }
    // At the pre-back point: aligned well enough for a straight back-in?
    if (fabsf(ly) > GUIDED_DOCK_LATERAL_TOL_MM) {
        dockRetry("off the dock axis");
        return;
    }
    if (fabsf(lt) > GUIDED_DOCK_HEADING_TOL_DEG * DEG) {
        turnBy(-lt, [this](MoveResult) {
            settleThen([this] {
                scanThen("dock scan", [this](bool ok, const LdsScanData& scan) { onDockScan2(ok, scan); });
            });
        });
        return;
    }
    tracePose("dock", {{"stage", "backin", FIELD_STRING},
                       {"lx", String(static_cast<int>(lx)), FIELD_INT},
                       {"ly", String(static_cast<int>(ly)), FIELD_INT}});
    int back = static_cast<int>(lx) + 60; // Overshoot slightly: the dock stops us
    startMove(MoveKind::DockBack, -back, -back, GUIDED_SPEED_SLOW,
              [this](MoveResult) { delayThen(1500, "dock settle", [this] { checkDocked(); }); });
}

// Missed: pull forward to the staging point and line up again; after the last
// attempt, hand over to the robot's own docking
void GuidedCleanManager::dockRetry(const char *why) {
    dockAttempts++;
    tracePose("dock_miss", {{"why", why, FIELD_STRING}, {"try", String(dockAttempts), FIELD_INT}});
    if (dockAttempts >= GUIDED_DOCK_ATTEMPTS || !refScanValid) {
        LOG("GUIDED", "Docking failed (%s) - handing over to the robot's own docking", why);
        fallbackDock = true;
        finish(Phase::Done, lastError.isEmpty() ? String("docking handed over to the robot") : lastError);
        return;
    }
    dockIcpFails = 0;
    float lx = 0.0f;
    float ly = 0.0f;
    toDock(poseX, poseY, lx, ly);
    float lt = wrapAngle(poseTheta - dockT);
    // Straight out along our heading (≈ the dock axis) to the staging distance
    int fwd = static_cast<int>(GUIDED_DOCK_STAGING_MM - lx);
    if (fwd < 100)
        fwd = 100;
    turnBy(-lt, [this, fwd](MoveResult) {
        startMove(MoveKind::Drive, fwd, fwd, GUIDED_SPEED_SLOW, [this](MoveResult) { dockAlign(); });
    });
}

void GuidedCleanManager::checkDocked() {
    uint32_t gen = runGen;
    beginWait(Wait::Cmd, GUIDED_CMD_TIMEOUT_MS, "dock check");
    cmdFallback = [this] { dockRetry("no answer from the charger"); };
    uint32_t wg = waitGen;
    serial.getCharger([this, gen, wg](bool ok, const ChargerData& c) {
        if (gen != runGen || wg != waitGen)
            return;
        endWait();
        if (ok && c.extPwrPresent) {
            event("dock_result", {{"docked", "true", FIELD_BOOL}, {"try", String(dockAttempts), FIELD_INT}});
            finish(Phase::Done, lastError);
            return;
        }
        dockRetry("no charger contact");
    });
}

// Point-to-point ICP between the current scan (placed at the guess pose in the
// dock frame) and the reference scan taken on the dock (pose 0,0,0). Both use
// the LIDAR mounting from the map calibration, so the result is the robot
// centre in the dock frame.
bool GuidedCleanManager::matchReference(const LdsScanData& scan, float& lx, float& ly, float& lt) const {
    struct P {
        int32_t x;
        int32_t y;
    };
    std::vector<P> ref;
    std::vector<P> cur;
    ref.reserve(360);
    cur.reserve(180);
    auto sgn = static_cast<float>(field.lidarSign);
    auto off = static_cast<float>(field.lidarOffsetX);
    for (int i = 0; i < 360; i++) {
        if (refScan[i] <= 0)
            continue;
        float a = sgn * static_cast<float>(i) * DEG + field.lidarAngleOffsetRad;
        auto d = static_cast<float>(refScan[i]);
        if (d > 4000.0f)
            continue;
        ref.push_back({static_cast<int32_t>(off + d * cosf(a)), static_cast<int32_t>(d * sinf(a))});
    }
    float c = cosf(lt);
    float s = sinf(lt);
    for (int i = 0; i < 360; i += 2) {
        const LdsScanPoint& p = scan.points[i];
        if (p.errorCode != 0 || p.distMM < 150 || p.distMM > 4000)
            continue;
        float a = sgn * static_cast<float>(p.angleDeg) * DEG + field.lidarAngleOffsetRad;
        float px = off + static_cast<float>(p.distMM) * cosf(a);
        float py = static_cast<float>(p.distMM) * sinf(a);
        cur.push_back({static_cast<int32_t>(lx + c * px - s * py), static_cast<int32_t>(ly + s * px + c * py)});
    }
    if (ref.size() < 60 || cur.size() < 40)
        return false;

    // Accumulated transform cur → ref: rotate by tt around origin, then translate (tx, ty)
    float tt = 0.0f;
    float tx = 0.0f;
    float ty = 0.0f;
    float rms = 0.0f;
    size_t inliers = 0;
    int32_t gate = 300;
    struct Pair {
        float qx;
        float qy;
        float rx;
        float ry;
    };
    std::vector<Pair> pairs;
    pairs.reserve(cur.size());
    for (int iter = 0; iter < 25; iter++) {
        float ct = cosf(tt);
        float st = sinf(tt);
        float err2 = 0.0f;
        pairs.clear();
        int32_t gate2 = gate * gate;
        for (const P& q: cur) {
            auto qx = static_cast<int32_t>(ct * static_cast<float>(q.x) - st * static_cast<float>(q.y) + tx);
            auto qy = static_cast<int32_t>(st * static_cast<float>(q.x) + ct * static_cast<float>(q.y) + ty);
            int32_t best = gate2;
            const P *nn = nullptr;
            for (const P& r: ref) {
                int32_t dx = r.x - qx;
                int32_t dy = r.y - qy;
                if (dx > gate || dx < -gate || dy > gate || dy < -gate)
                    continue;
                int32_t d2 = dx * dx + dy * dy;
                if (d2 < best) {
                    best = d2;
                    nn = &r;
                }
            }
            if (!nn)
                continue;
            pairs.push_back({static_cast<float>(qx), static_cast<float>(qy), static_cast<float>(nn->x),
                             static_cast<float>(nn->y)});
            err2 += static_cast<float>(best);
        }
        inliers = pairs.size();
        if (inliers < 30)
            return false;
        auto n = static_cast<float>(inliers);
        float mx = 0.0f;
        float my = 0.0f;
        float mrx = 0.0f;
        float mry = 0.0f;
        for (const Pair& pr: pairs) {
            mx += pr.qx;
            my += pr.qy;
            mrx += pr.rx;
            mry += pr.ry;
        }
        mx /= n;
        my /= n;
        mrx /= n;
        mry /= n;
        float cxx = 0.0f;
        float cxy = 0.0f;
        float cyx = 0.0f;
        float cyy = 0.0f;
        for (const Pair& pr: pairs) {
            float ax = pr.qx - mx;
            float ay = pr.qy - my;
            float bx = pr.rx - mrx;
            float by = pr.ry - mry;
            cxx += ax * bx;
            cxy += ax * by;
            cyx += ay * bx;
            cyy += ay * by;
        }
        float dTheta = atan2f(cxy - cyx, cxx + cyy);
        float cd = cosf(dTheta);
        float sd = sinf(dTheta);
        float stepX = mrx - (cd * mx - sd * my);
        float stepY = mry - (sd * mx + cd * my);
        float ntx = cd * tx - sd * ty + stepX;
        float nty = sd * tx + cd * ty + stepY;
        tt += dTheta;
        tx = ntx;
        ty = nty;
        rms = sqrtf(err2 / n);
        if (gate > 100)
            gate -= 40;
        if (fabsf(dTheta) < 0.0005f && fabsf(stepX) < 1.0f && fabsf(stepY) < 1.0f && gate <= 100)
            break;
    }
    float ratio = static_cast<float>(inliers) / static_cast<float>(cur.size());
    if (ratio < 0.5f || rms > 50.0f)
        return false;
    float ct = cosf(tt);
    float st = sinf(tt);
    float nx = ct * lx - st * ly + tx;
    float ny = st * lx + ct * ly + ty;
    float cx = nx - lx;
    float cy = ny - ly;
    if (sqrtf(cx * cx + cy * cy) > GUIDED_ICP_MAX_CORR_MM || fabsf(tt) > 20.0f * DEG)
        return false;
    lx = nx;
    ly = ny;
    lt = wrapAngle(lt + tt);
    return true;
}

// -- Finish -------------------------------------------------------------------------------

void GuidedCleanManager::finish(Phase result, const String& reason) {
    Phase from = phase;
    runGen++; // Invalidate all in-flight callbacks of this run
    waitGen++;
    wait = Wait::None;
    afterDelay = nullptr;
    scanJob = nullptr;
    onMoveDone = nullptr;
    move = MoveKind::None;
    moveEnding = false;
    pollPending = false;
    lastError = reason;
    finishedMs = millis();

    LOG("GUIDED", "Finish: %s → %s (%s)", phaseName(from), phaseName(result), reason.c_str());
    event(phaseName(result), {{"reason", reason, FIELD_STRING},
                              {"cleaned", String(cleanDone), FIELD_INT},
                              {"total", String(cleanTotal), FIELD_INT},
                              {"obstacles", String(obstacleTotal), FIELD_INT}});
    traceFlush();

    bool requestDock = fallbackDock && result == Phase::Done;
    if (!inTestMode) {
        phase = result;
        serial.setGuidedCleanActive(false);
        freePlan();
        return;
    }

    phase = Phase::Finishing;
    finishingSince = millis();
    serial.setMotorWheels(0, 0, 0, nullptr);
    motorsOn = true; // Force the off commands
    setCleaningMotors(false);
    serial.setLdsRotation(false, nullptr);
    ldsOn = false;
    uint32_t gen = runGen;
    // TestMode Off, retried: the robot must never be left in TestMode
    auto attempt = std::make_shared<std::function<void(int)>>();
    *attempt = [this, gen, result, requestDock, attempt](int n) {
        serial.testMode(false, [this, gen, result, requestDock, attempt, n](bool ok) {
            if (gen != runGen) {
                *attempt = nullptr; // Break the self-reference
                return;
            }
            if (!ok && n < 3) {
                LOG("GUIDED", "TestMode Off failed - retrying");
                (*attempt)(n + 1);
                return;
            }
            finishingSince = 0;
            if (ok)
                inTestMode = false;
            else
                releaseSince = millis(); // Still failing: keep trying in the background
            serial.setGuidedCleanActive(false);
            phase = result;
            if (requestDock) {
                serial.clean("dock", nullptr);
                dockWatchSince = millis();
                dockWatchRetried = false;
            }
            freePlan();
            *attempt = nullptr; // Break the self-reference
        });
    };
    (*attempt)(1);
}

// -- LIDAR obstacle layer -------------------------------------------------------------

bool GuidedCleanManager::DynGrid::blocked(float x, float y) const {
    if (w == 0)
        return false;
    int cx = static_cast<int>(floorf((x - static_cast<float>(originX)) / static_cast<float>(cell)));
    int cy = static_cast<int>(floorf((y - static_cast<float>(originY)) / static_cast<float>(cell)));
    return inside(cx, cy) && infl[static_cast<size_t>(cy * w + cx)] > 0;
}

void GuidedCleanManager::DynGrid::clear() {
    std::vector<uint8_t>().swap(hits);
    std::vector<uint8_t>().swap(infl);
    w = 0;
    h = 0;
}

void GuidedCleanManager::initDyn() {
    dyn.clear();
    int spanX = cover.w * GUIDED_COVER_CELL_MM;
    int spanY = cover.h * GUIDED_COVER_CELL_MM;
    int cell = GUIDED_COVER_CELL_MM;
    while ((spanX / cell + 1) * (spanY / cell + 1) > GUIDED_DYN_MAX_CELLS)
        cell *= 2;
    dyn.cell = cell;
    dyn.originX = cover.originX;
    dyn.originY = cover.originY;
    dyn.w = spanX / cell + 1;
    dyn.h = spanY / cell + 1;
    dyn.hits.assign(static_cast<size_t>(dyn.w) * static_cast<size_t>(dyn.h), 0);
    dyn.infl.assign(dyn.hits.size(), 0);
}

// A cell crossing the occupied threshold adds/removes its keep-out disc
void GuidedCleanManager::dynSet(int cx, int cy, bool occupied) {
    int r = (static_cast<int>(GUIDED_CLEARANCE_MM) + dyn.cell / 2) / dyn.cell;
    for (int dy = -r; dy <= r; dy++) {
        for (int dx = -r; dx <= r; dx++) {
            if (dx * dx + dy * dy > r * r || !dyn.inside(cx + dx, cy + dy))
                continue;
            uint8_t& v = dyn.infl[static_cast<size_t>((cy + dy) * dyn.w + cx + dx)];
            if (occupied && v < 255)
                v++;
            else if (!occupied && v > 0)
                v--;
        }
    }
}

// Called after a good fix while standing still: returns the map does not explain
// mark cells, rays through cells clear them (chairs get moved back, people leave)
void GuidedCleanManager::markScan() {
    if (dyn.w == 0 || beams.n == 0)
        return;
    float c = cosf(poseTheta);
    float s = sinf(poseTheta);
    const auto cell = static_cast<float>(dyn.cell);
    auto cellOf = [this, cell](float x, float y, int& cx, int& cy) {
        cx = static_cast<int>(floorf((x - static_cast<float>(dyn.originX)) / cell));
        cy = static_cast<int>(floorf((y - static_cast<float>(dyn.originY)) / cell));
        return dyn.inside(cx, cy);
    };
    int added = 0;
    for (int i = 0; i < beams.n; i++) {
        float bx = beams.x[i];
        float by = beams.y[i];
        float r = sqrtf(bx * bx + by * by);
        if (r > GUIDED_DYN_RANGE_MM + 1500.0f)
            continue;
        float wx = poseX + c * bx - s * by;
        float wy = poseY + s * bx + c * by;
        // Clear along the ray (stop short of the return)
        int steps = static_cast<int>((r - 150.0f) / (cell / 2.0f));
        for (int k = 1; k < steps; k++) {
            float t = static_cast<float>(k) * (cell / 2.0f) / r;
            int cx = 0;
            int cy = 0;
            if (!cellOf(poseX + (wx - poseX) * t, poseY + (wy - poseY) * t, cx, cy))
                continue;
            uint8_t& hv = dyn.hits[static_cast<size_t>(cy * dyn.w + cx)];
            if (hv > 0) {
                bool was = hv >= GUIDED_DYN_HITS;
                hv--;
                if (was && hv < GUIDED_DYN_HITS)
                    dynSet(cx, cy, false);
            }
        }
        if (r > GUIDED_DYN_RANGE_MM)
            continue;
        // Return far from anything mapped: something new stands there
        if (field.inside(wx, wy) && field.at(wx, wy) > GUIDED_DYN_FIELD_MAX)
            continue;
        int cx = 0;
        int cy = 0;
        if (!cellOf(wx, wy, cx, cy))
            continue;
        uint8_t& hv = dyn.hits[static_cast<size_t>(cy * dyn.w + cx)];
        bool was = hv >= GUIDED_DYN_HITS;
        hv = hv + 2 > 8 ? 8 : static_cast<uint8_t>(hv + 2);
        if (!was && hv >= GUIDED_DYN_HITS) {
            dynSet(cx, cy, true);
            added++;
        }
    }
    if (added > 0)
        tracePose("seen", {{"n", String(added), FIELD_INT}});
}

// -- Obstacle memory ----------------------------------------------------------------------

void GuidedCleanManager::rememberObstacle(float aheadMM, int radius, bool behind) {
    if (obstacles.size() >= GUIDED_MAX_OBSTACLES)
        obstacles.erase(obstacles.begin());
    float dir = behind ? -1.0f : 1.0f;
    Obstacle o;
    o.x = static_cast<int16_t>(poseX + dir * aheadMM * cosf(poseTheta));
    o.y = static_cast<int16_t>(poseY + dir * aheadMM * sinf(poseTheta));
    o.r = static_cast<int16_t>(radius);
    o.atMs = millis();
    obstacles.push_back(o);
}

// -- Status -----------------------------------------------------------------------------

String GuidedCleanManager::getStatusJson() const {
    unsigned long endMs = finishedMs > 0 && !isActive() ? finishedMs : millis();
    unsigned long elapsed = startedMs > 0 ? (endMs - startedMs) / 1000UL : 0;
    int progressPct = cleanTotal > 0 ? (cleanDone * 100) / cleanTotal : 0;
    if (phase == Phase::Done && lastError.isEmpty())
        progressPct = 100;

    std::vector<Field> fields = {
            {"phase", phaseName(phase), FIELD_STRING},
            {"active", isActive() ? "true" : "false", FIELD_BOOL},
            {"hasMap", floorMap.hasMap() ? "true" : "false", FIELD_BOOL},
            {"locMap",
             !floorMap.hasField() ? "none"
             : fieldLoadFailed    ? "failed"
                                  : "ready",
             FIELD_STRING},
            {"rooms", roomsLabel, FIELD_STRING},
            {"x", String(poseX / 1000.0f, 3), FIELD_FLOAT},
            {"y", String(poseY / 1000.0f, 3), FIELD_FLOAT},
            {"t", String(wrapAngle(poseTheta) / DEG, 1), FIELD_FLOAT},
            {"progress", String(progressPct), FIELD_INT},
            {"cleanDone", String(cleanDone), FIELD_INT},
            {"cleanTotal", String(cleanTotal), FIELD_INT},
            {"obstacles", String(obstacleTotal), FIELD_INT},
            {"cleaning", motorsOn ? "true" : "false", FIELD_BOOL},
            {"elapsed", String(elapsed), FIELD_INT},
            {"startedAt", String(static_cast<long>(startedEpoch)), FIELD_INT},
            {"message", lastError, FIELD_STRING},
            {"compass", "off", FIELD_STRING},
            {"compassFixes", "0", FIELD_INT},
            {"localization", field.valid() ? "map" : "odometry", FIELD_STRING},
            {"locFixes", String(locFixes), FIELD_INT},
            {"locScore", String(locScore, 2), FIELD_FLOAT},
            {"locSpread", String(static_cast<int>(locSpread)), FIELD_INT},
            {"waiting", waitWhat, FIELD_STRING},
    };
    if (isActive() && !route.empty() && targetIdx < route.size()) {
        fields.push_back({"targetX", String(static_cast<float>(route[targetIdx].x) / 1000.0f, 3), FIELD_FLOAT});
        fields.push_back({"targetY", String(static_cast<float>(route[targetIdx].y) / 1000.0f, 3), FIELD_FLOAT});
    }
    return fieldsToJson(fields);
}

// -- Run trace (always on, /map/trace.jsonl, served by GET /api/guided/trace) -------------------
// One JSON object per line: {"e":<event>,"ms":<since start>,"x","y","t"(deg), ...}. Pose lines
// ("p") every GUIDED_TRACE_EVERY_MM, localization fixes ("loc": estimate mx/my/mt, quality s
// vs quality s0 at the odometry pose, spread sp, correction d), legs, obstacles, timeouts,
// dock stages and the result. Buffered in RAM and appended in chunks (rare flash writes).

void GuidedCleanManager::traceBegin() {
    traceBuf = "";
    traceBytes = 0;
    traceDist = 0.0f;
    traceTurn = 0.0f;
    traceStartMs = millis();
    SPIFFS.remove(GUIDED_TRACE_FILE);
}

void GuidedCleanManager::event(const String& name, const std::vector<Field>& fields) {
    dataLogger.logGuided(name, fields);
    tracePose(name, fields);
}

void GuidedCleanManager::tracePose(const String& name, const std::vector<Field>& fields) {
    // Full: pose/fix lines stop first, then other events; the run result always fits
    bool routine = name == "p" || name == "loc" || name == "seen";
    bool result = name == "done" || name == "failed" || name == "stopped" || name == "dock_result";
    size_t limit = GUIDED_TRACE_MAX_BYTES - (routine ? 8192 : result ? 0 : 2048);
    if (traceBytes + traceBuf.length() > limit)
        return;
    std::vector<Field> all = {{"e", name, FIELD_STRING},
                              {"ms", String(millis() - traceStartMs), FIELD_INT},
                              {"x", String(static_cast<int>(poseX)), FIELD_INT},
                              {"y", String(static_cast<int>(poseY)), FIELD_INT},
                              {"t", String(static_cast<int>(poseTheta / DEG)), FIELD_INT}};
    all.insert(all.end(), fields.begin(), fields.end());
    traceBuf += fieldsToJson(all);
    traceBuf += '\n';
    if (traceBuf.length() >= GUIDED_TRACE_FLUSH_BYTES)
        traceFlush();
}

void GuidedCleanManager::traceFlush() {
    if (traceBuf.isEmpty())
        return;
    File f = SPIFFS.open(GUIDED_TRACE_FILE, FILE_APPEND);
    if (f) {
        f.print(traceBuf);
        f.close();
    }
    traceBytes += traceBuf.length();
    traceBuf = "";
}

bool GuidedCleanManager::hasTrace() const {
    return SPIFFS.exists(GUIDED_TRACE_FILE);
}

// On the dock: keep the close-range LIDAR returns (angle, distance, intensity)
// in the trace — data for checking the dock reference.
void GuidedCleanManager::traceDockScan(const LdsScanData& scan, const char *name, int maxDistMM) {
    String pts;
    pts.reserve(2200);
    for (int i = 0; i < 360; i++) {
        const LdsScanPoint& p = scan.points[i];
        if (p.errorCode != 0 || p.distMM <= 0 || p.distMM > maxDistMM)
            continue;
        if (pts.length() > 2000)
            break;
        pts += String(p.angleDeg) + ":" + String(p.distMM) + ":" + String(p.intensity) + " ";
    }
    tracePose(name, {{"pts", pts, FIELD_STRING}});
}

// Dock frame: origin on the dock contacts, +x = the robot's heading when docked
void GuidedCleanManager::toDock(float x, float y, float& lx, float& ly) const {
    float c = cosf(dockT);
    float sn = sinf(dockT);
    float dx = x - dockX;
    float dy = y - dockY;
    lx = c * dx + sn * dy;
    ly = -sn * dx + c * dy;
}

void GuidedCleanManager::fromDock(float lx, float ly, float& x, float& y) const {
    float c = cosf(dockT);
    float sn = sinf(dockT);
    x = dockX + c * lx - sn * ly;
    y = dockY + sn * lx + c * ly;
}

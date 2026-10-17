#include "guided_clean_manager.h"
#include <algorithm>
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
    return phase == Phase::Starting || phase == Phase::Running || phase == Phase::Returning ||
           phase == Phase::Docking || phase == Phase::Finishing;
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
    return std::any_of(obstacles.begin(), obstacles.end(), [x, y](const Obstacle& o) {
        float dx = x - static_cast<float>(o.x);
        float dy = y - static_cast<float>(o.y);
        return sqrtf(dx * dx + dy * dy) < static_cast<float>(o.r + GUIDED_ROBOT_RADIUS_MM);
    });
}

bool GuidedCleanManager::segmentClear(float ax, float ay, float bx, float by, bool requireCover) const {
    for (const auto& z: nogos) {
        if (mapgeo::segmentNearPolygon(ax, ay, bx, by, z.poly, GUIDED_NOGO_MARGIN_MM))
            return false;
    }
    for (const auto& o: obstacles) {
        if (mapgeo::distPointSegment(o.x, o.y, ax, ay, bx, by) < static_cast<float>(o.r + GUIDED_ROBOT_RADIUS_MM))
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
    // The localization field doubles as the free-space map for lanes and planning
    if (!floorMap.loadField(field))
        field = LocField();

    // The dock is the map origin; refuse plans that start inside a no-go zone
    if (nearNogo(0.0f, 0.0f))
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

    // Cleaning runs: parallel lanes over the free floor of each selected room
    // (structured coverage, independent of how the recording wandered). Without a
    // localization field the free floor is what the recording covered; if no lane
    // fits at all, fall back to the recorded points inside the rooms.
    // Runs are visited nearest-first in either direction (→ serpentine order);
    // transit between them is planned on the grid.
    buildLanes(selected);
    if (cleanPts.empty()) {
        LOG("GUIDED", "Plan: no lanes fit — following the recorded path instead");
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

    // The robot leaves the dock straight ahead (+x): keep the dock axis open up
    // to the staging point even if the recording sampled it sparsely.
    for (int x = 0; x <= GUIDED_DOCK_STAGING_MM; x += cell / 2) {
        int cx = 0;
        int cy = 0;
        grid.toCell(static_cast<float>(x), 0.0f, cx, cy);
        if (grid.inside(cx, cy) && !nearNogo(static_cast<float>(x), 0.0f))
            grid.free[static_cast<size_t>(grid.index(cx, cy))] = 1;
    }

    // Keep only floor reachable from the dock: open space outside the walls (the
    // field is "far from walls" there too) or behind closed doors is not floor.
    std::vector<uint8_t> seen(grid.free.size(), 0);
    std::vector<uint16_t> queue;
    queue.reserve(256);
    int sx = 0;
    int sy = 0;
    grid.toCell(static_cast<float>(GUIDED_DOCK_STAGING_MM) / 2.0f, 0.0f, sx, sy);
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
    if (!grid.free[static_cast<size_t>(grid.index(cx, cy))])
        return true;
    if (planIgnoreObstacles)
        return false;
    // Obstacles are remembered generously ahead of the robot; for planning only
    // block their core so a bump against a wall does not seal a narrow corridor.
    float x = grid.centerX(cx);
    float y = grid.centerY(cy);
    return std::any_of(obstacles.begin(), obstacles.end(), [x, y](const Obstacle& o) {
        float dx = x - static_cast<float>(o.x);
        float dy = y - static_cast<float>(o.y);
        return sqrtf(dx * dx + dy * dy) < static_cast<float>(o.r + GUIDED_ROBOT_RADIUS_MM / 2);
    });
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

    if (sqrtf(bestD) < 400.0f) {
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
    if (legReplans <= GUIDED_MAX_REPLANS && planPath(legGoalX, legGoalY, path)) {
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
        LOG("GUIDED", "Way home blocked — handing over to the robot's own docking");
        fallbackDock = true;
        finish(Phase::Done, "way home blocked — the robot docks itself");
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
    LOG("GUIDED", "Planner could not find a way home — retracing %u crumbs", static_cast<unsigned>(crumbs.size()));
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
    legReplans = GUIDED_MAX_REPLANS + 1; // No further re-planning on this leg
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
        scanJob = nullptr;
        phase = Phase::Starting;
        lastError = "";
        roomsLabel = rooms;
        startedMs = millis();
        startedEpoch = sysMgr.now();
        finishedMs = 0;
        obstacleStreak = 0;
        obstacleTotal = 0;
        fallbackDock = false;
        dockWatchSince = 0;
        dockX = 0.0f;
        dockY = 0.0f;
        dockT = 0.0f;
        inTestMode = false;
        dockStage = 0;
        dockAttempts = 0;
        dockIcpNearDone = false;
        ldsOn = false;
        compassTried = false;
        compassRef = false;
        locInit = false;
        locMovingPending = false;
        chaining = false;
        locFixes = 0;
        locScore = 0.0f;
        refScanValid = false;
        dockIcpTries = 0;
        compassSign = 0;
        compassFixes = 0;
        lastFixDeg = 0.0f;
        distSinceFix = 0.0f;
        turnSinceFix = 0.0f;
        calStage = 0;
        calAttempts = 0;
        lastScanMs = 0;
        seg = SegKind::None;
        segStopping = false;
        motorsOn = false;
        pollPending = false;
        encValid = false;
        poseX = 0.0f;
        poseY = 0.0f;
        poseTheta = 0.0f;
        traceBegin();
        event("start", {{"rooms", rooms.isEmpty() ? String("all") : rooms, FIELD_STRING},
                        {"field", field.valid() ? "true" : "false", FIELD_BOOL},
                        {"nogos", String(static_cast<unsigned>(nogos.size())), FIELD_INT},
                        {"runs", String(static_cast<unsigned>(runs.size())), FIELD_INT},
                        {"clean", String(cleanTotal), FIELD_INT}});
        traceFlush(); // Evidence even if the ESP32 dies during the run
        beginStart();
        return "";
    }

    if (action == "home") {
        if (phase == Phase::Running) {
            beginReturn("returned on request");
            return "";
        }
        if (phase == Phase::Returning || phase == Phase::Docking)
            return "";
        return "guided clean is not running";
    }

    if (action == "stop") {
        if (!isActive())
            return "guided clean is not running";
        finish(Phase::Stopped, "stopped by user");
        return "";
    }

    return "unknown action";
}

void GuidedCleanManager::beginStart() {
    uint32_t gen = runGen;
    waitingCmd = true;
    serial.getState([this, gen](bool ok, const RobotState& state) {
        if (gen != runGen)
            return;
        if (!ok) {
            fail("robot not responding");
            return;
        }
        if (!state.isIdle()) {
            fail("robot is busy (" + state.uiState + ")");
            return;
        }
        serial.getCharger([this, gen](bool ok, const ChargerData& c) {
            if (gen != runGen)
                return;
            if (!ok) {
                fail("cannot read charger state");
                return;
            }
            if (!c.extPwrPresent) {
                fail("start the robot on its dock (the dock is the map origin)");
                return;
            }
            if (c.fuelPercent >= 0 && c.fuelPercent < GUIDED_MIN_BATTERY_PCT + 10) {
                fail("battery too low (" + String(c.fuelPercent) + "%)");
                return;
            }
            serial.testMode(true, [this, gen](bool ok) {
                if (gen != runGen)
                    return;
                if (!ok) {
                    fail("TestMode On failed");
                    return;
                }
                inTestMode = true;
                serial.setGuidedCleanActive(true);
                // LIDAR for the wall compass (best effort — runs without it if this fails)
                serial.setLdsRotation(true, [this, gen](bool ok) {
                    if (gen != runGen)
                        return;
                    ldsOn = ok;
                    ldsReadyAtMs = millis() + GUIDED_LDS_SPINUP_MS;
                });
                serial.getMotors(
                        [this, gen](bool ok, const MotorData& m) {
                            if (gen != runGen)
                                return;
                            if (!ok) {
                                fail("cannot read wheel encoders");
                                return;
                            }
                            encL = m.leftWheelPositionMM;
                            encR = m.rightWheelPositionMM;
                            encValid = true;
                            waitingCmd = false;
                            phase = Phase::Running;
                            routeIdx = 0;
                            targetIdx = 0;
                            batteryTicker.reset();
                            batteryTicker.elapsed(GUIDED_BATTERY_POLL_MS); // Arm: first check after one interval
                            LOG("GUIDED", "Running (%u runs)", static_cast<unsigned>(runs.size()));
                            startNextLeg();
                        },
                        PRIORITY_HIGH);
            });
        });
    });
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
            LOG("GUIDED", "Robot still in TestMode after an ESP32 restart — leaving TestMode");
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
                LOG("GUIDED", "Robot not on the dock after 60 s — asking again");
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

    if (!isActive() || phase == Phase::Starting || phase == Phase::Finishing)
        return;

    // Scan results are processed here, not inside the serial callback chain
    // (shallow stack; see fetchScan)
    if (scanJob) {
        auto job = std::move(scanJob);
        scanJob = nullptr;
        job(scanJobOk, scanBuf);
        return;
    }

    // Sensor polling runs continuously while active (odometry + safety)
    if (!pollPending && pollTicker.elapsed(GUIDED_POLL_MS) && serial.queueDepth() <= NEATO_QUEUE_MAX_SIZE / 2)
        pollSensors();

    if (waitingCmd || segStopping)
        return;

    // Localization on the move (continuous driving rarely stops)
    if (seg == SegKind::Drive && field.valid() && ldsOn && locInit && !locMovingPending && !scanJob &&
        distSinceFix >= GUIDED_LOC_MOVING_EVERY_MM && millis() - lastScanMs >= 1600 &&
        (phase == Phase::Running || phase == Phase::Returning)) {
        requestLocScanMoving();
    }

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

    if (phase == Phase::Docking && dockStage == 2) {
        if (millis() >= dockCheckAtMs)
            checkDocked();
        return;
    }

    if (seg == SegKind::None)
        step();
}

// -- Sensor polling -------------------------------------------------------------------

void GuidedCleanManager::pollSensors() {
    uint32_t gen = runGen;
    pollPending = true;
    // Motors twice per cycle: odometry + segment completion matter most
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
        segStartL += dl;
        segStartR += dr;
        return;
    }
    float dTheta = static_cast<float>(dr - dl) / GUIDED_WHEEL_BASE_MM;
    float ds = static_cast<float>(dl + dr) / 2.0f;
    float mid = poseTheta + dTheta / 2.0f;
    poseX += ds * cosf(mid);
    poseY += ds * sinf(mid);
    poseTheta = wrapAngle(poseTheta + dTheta);
    distSinceFix += fabsf(ds);
    turnSinceFix += fabsf(dTheta);
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

    if (seg == SegKind::None || segStopping)
        return;

    bool moved = (encL != prevL) || (encR != prevR);
    segStillPolls = moved ? 0 : segStillPolls + 1;

    int progL = abs(encL - segStartL);
    int progR = abs(encR - segStartR);
    // Keep rolling: plan the next command before this one runs out
    if (seg == SegKind::Drive && (phase == Phase::Running || phase == Phase::Returning) && !waitingCmd) {
        int left = (abs(segCmdL) - progL + abs(segCmdR) - progR) / 2;
        if (left <= GUIDED_CHAIN_MM && progL + progR > 0) {
            chainNext();
            return;
        }
    }
    bool reached = progL >= abs(segCmdL) - 15 && progR >= abs(segCmdR) - 15;
    if (reached) {
        endSegment(false);
        return;
    }

    unsigned long elapsed = millis() - segStartMs;

    // Wheel overload while driving forward = pushing against something
    if (seg == SegKind::Drive || seg == SegKind::Turn) {
        bool overloaded = m.leftWheelLoad >= GUIDED_STALL_LOAD_PCT || m.rightWheelLoad >= GUIDED_STALL_LOAD_PCT;
        segStallPolls = overloaded ? segStallPolls + 1 : 0;
        if (segStallPolls >= 2) {
            onObstacle("stall");
            return;
        }
    }

    // Stopped short of the target (blocked without bumper contact, or command dropped)
    if (elapsed > 800 && segStillPolls >= 3) {
        int expected = abs(segCmdL) + abs(segCmdR);
        int done = progL + progR;
        bool tooShort = (seg == SegKind::Drive) && done * 2 < expected;
        endSegment(tooShort);
        return;
    }

    if (elapsed > segTimeoutMs) {
        LOG("GUIDED", "Segment timeout (prog L=%d R=%d of %d/%d)", progL, progR, segCmdL, segCmdR);
        serial.setMotorWheels(0, 0, 0, nullptr);
        endSegment(seg == SegKind::Drive);
    }
}

void GuidedCleanManager::onDigital(const DigitalSensorData& d) {
    if (d.leftWheelExtended || d.rightWheelExtended) {
        LOG("GUIDED", "SAFETY: wheel lifted — aborting");
        serial.setMotorWheels(0, 0, 0, nullptr);
        fail("robot lifted or wheel dropped");
        return;
    }
    if (seg == SegKind::DockBack && d.dcJackIn && !segStopping) {
        // On the contacts: stop pushing and verify
        serial.setMotorWheels(0, 0, 0, nullptr);
        endSegment(false);
        return;
    }
    if (seg == SegKind::None || seg == SegKind::Backoff || seg == SegKind::DockBack || seg == SegKind::Reverse ||
        segStopping)
        return;
    bool bump = d.lFrontBit || d.rFrontBit || d.lLdsBit || d.rLdsBit || d.lSideBit || d.rSideBit;
    if (bump)
        onObstacle("bumper");
}

void GuidedCleanManager::onAnalog(const BatteryAnalogData& a) {
    bool cliff = (a.dropLeftMM > GUIDED_DROP_MM) || (a.dropRightMM > GUIDED_DROP_MM);
    if (!cliff)
        return;
    if (seg == SegKind::Backoff || segStopping)
        return;
    LOG("GUIDED", "SAFETY: cliff detected (L=%d R=%d mm)", a.dropLeftMM, a.dropRightMM);
    if (seg == SegKind::DockBack) {
        // Backing over the dock ramp can lift the front sensors briefly — stop and verify instead
        serial.setMotorWheels(0, 0, 0, nullptr);
        endSegment(false);
        return;
    }
    onObstacle("cliff");
}

// -- Segment executor ------------------------------------------------------------------

void GuidedCleanManager::startSegment(SegKind kind, int leftMM, int rightMM, int speed) {
    seg = kind;
    segCmdL = leftMM;
    segCmdR = rightMM;
    segStartL = encL;
    segStartR = encR;
    segStartMs = millis();
    int longest = abs(leftMM) > abs(rightMM) ? abs(leftMM) : abs(rightMM);
    segTimeoutMs = static_cast<unsigned long>(longest) * 1000UL / static_cast<unsigned long>(speed > 0 ? speed : 1) +
                   GUIDED_SEGMENT_SLACK_MS;
    segStillPolls = 0;
    segStallPolls = 0;
    segStopping = false;
    pollTicker.reset(); // Poll promptly after a new command
    serial.setMotorWheels(leftMM, rightMM, speed, nullptr);
}

void GuidedCleanManager::endSegment(bool obstacle) {
    SegKind finished = seg;
    seg = SegKind::None;

    if (finished == SegKind::Backoff) {
        if (phase == Phase::Docking) {
            // Bumped while lining up: start the approach over from the staging point
            dockAttempts++;
            dockStage = dockAttempts >= GUIDED_DOCK_ATTEMPTS ? 10 : 4;
            return;
        }
        if (phase != Phase::Running && phase != Phase::Returning)
            return;
        if (legTransit) {
            onTransitBlocked();
            return;
        }
        // Cleaning run: skip past the obstructed point and carry on
        obstacleStreak++;
        if (obstacleStreak >= GUIDED_MAX_OBSTACLE_STREAK) {
            skipCurrentRun("too many obstacles");
            return;
        }
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
        return;
    }

    if (finished == SegKind::DockBack) {
        dockStage = 2;
        dockCheckAtMs = millis() + 1500;
        return;
    }

    if (obstacle)
        onObstacle("blocked");
}

void GuidedCleanManager::onObstacle(const char *reason) {
    obstacleTotal++;
    LOG("GUIDED", "Obstacle (%s) at %.0f,%.0f → back off", reason, poseX, poseY);
    event("obstacle", {{"reason", reason, FIELD_STRING},
                       {"x", String(static_cast<int>(poseX)), FIELD_INT},
                       {"y", String(static_cast<int>(poseY)), FIELD_INT}});
    bool isCliff = strcmp(reason, "cliff") == 0;
    rememberObstacle(GUIDED_ROBOT_RADIUS_MM + (isCliff ? 60 : GUIDED_OBSTACLE_RADIUS_MM),
                     isCliff ? GUIDED_CLIFF_RADIUS_MM : GUIDED_OBSTACLE_RADIUS_MM);
    // Stop first (disable/enable sequence), then reverse a little
    segStopping = true;
    seg = SegKind::None;
    uint32_t gen = runGen;
    serial.setMotorWheels(0, 0, 0, [this, gen](bool) {
        if (gen != runGen)
            return;
        segStopping = false;
        startSegment(SegKind::Backoff, -GUIDED_BACKOFF_MM, -GUIDED_BACKOFF_MM, GUIDED_SPEED_SLOW);
    });
}

// -- Navigation -------------------------------------------------------------------------

void GuidedCleanManager::markReached(size_t idx) {
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
            if (segmentClear(poseX, poseY, route[j].x, route[j].y, true)) {
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
        bool clear = j == next ? segmentClear(poseX, poseY, route[j].x, route[j].y, false)
                               : segmentClear(poseX, poseY, route[j].x, route[j].y, true);
        if (clear) {
            if (j > next)
                markReached(j - 1);
            targetIdx = j;
            return;
        }
    }

    // Nothing reachable without crossing a no-go zone: treat as obstacle and skip on
    LOG("GUIDED", "No clear line from %.0f,%.0f — skipping waypoint %u", poseX, poseY, static_cast<unsigned>(next));
    obstacleStreak++;
    markReached(next);
    targetIdx = next < last ? next + 1 : last;
}

void GuidedCleanManager::step() {
    // Map localization (preferred): first fix on the dock once the LIDAR spins,
    // then a scan match at every stop after some travel or turning. While the robot
    // keeps rolling (chaining) fixes are taken on the move from tick() instead.
    if (field.valid() && ldsOn && !chaining) {
        if (!locInit) {
            if (millis() < ldsReadyAtMs)
                return;
            locInit = true;
            dockScanPending = true;
            requestLocScan(GUIDED_LOC_START_RADIUS_MM, GUIDED_LOC_START_ANGLE_DEG, 0xFF);
            return;
        }
        if ((distSinceFix >= GUIDED_LOC_EVERY_MM || turnSinceFix >= GUIDED_LOC_EVERY_DEG * DEG) &&
            millis() - lastScanMs >= 1600 && phase != Phase::Docking) {
            requestLocScan(GUIDED_LOC_RADIUS_MM, GUIDED_LOC_ANGLE_DEG, 0xFF);
            return;
        }
    }

    // Wall compass (fallback without a map): reference scan on the dock once the
    // LIDAR has spun up, then periodic re-alignment while stopped between segments.
    if (chaining && !field.valid()) {
        // Compass fallback still re-aligns at stops; skip it while rolling
    } else if (ldsOn && !compassTried && !field.valid()) {
        if (millis() < ldsReadyAtMs)
            return;
        compassTried = true;
        requestScan(true);
        return;
    }
    // Direction calibration: once clear of the dock, turn to a heading of
    // ~22.5° (mod 45°) where CW and CCW LIDAR angle conventions disagree most,
    // scan, and lock the sign. Retried up to 3 times along the route.
    if (!chaining && !field.valid() && compassRef && compassSign == 0 && phase == Phase::Running && calAttempts < 3 &&
        sqrtf(poseX * poseX + poseY * poseY) > 300.0f) {
        if (calStage == 0) {
            float target = roundf((poseTheta / DEG - 22.5f) / 45.0f) * 45.0f + 22.5f;
            float err = wrapAngle(target * DEG - poseTheta);
            calStage = 1;
            if (fabsf(err) > 3.0f * DEG) {
                int arc = static_cast<int>(err * GUIDED_WHEEL_BASE_MM / 2.0f);
                startSegment(SegKind::Turn, -arc, arc, GUIDED_SPEED_TURN);
                return;
            }
        }
        if (calStage == 1) {
            calStage = 0;
            calAttempts++;
            requestScan(false);
            return;
        }
    }
    if (!chaining && !field.valid() && compassRef && compassSign != 0 &&
        (distSinceFix >= GUIDED_COMPASS_EVERY_MM || turnSinceFix >= GUIDED_COMPASS_EVERY_DEG * DEG) &&
        millis() - lastScanMs >= 2000) {
        requestScan(false);
        return;
    }

    if (phase == Phase::Docking) {
        stepDocking();
        return;
    }

    if (route.empty())
        return;

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

    // Cleaning motors follow the waypoint type while cleaning
    if (phase == Phase::Running)
        setCleaningMotors(t.clean);

    int speed = (phase == Phase::Running && t.clean) ? GUIDED_SPEED_CLEAN : GUIDED_SPEED_TRANSIT;
    driveTowards(t.x, t.y, speed, false, 0);
}

bool GuidedCleanManager::driveTowards(float tx, float ty, int speed, bool allowReverse, int tolMM) {
    float dx = tx - poseX;
    float dy = ty - poseY;
    float dist = sqrtf(dx * dx + dy * dy);
    if (dist <= static_cast<float>(tolMM))
        return false;
    float heading = atan2f(dy, dx);
    float err = wrapAngle(heading - poseTheta);
    float dir = 1.0f;
    // Short hops to a target behind the robot: back up instead of turning around
    if (allowReverse && fabsf(err) > 90.0f * DEG && dist < 500.0f) {
        dir = -1.0f;
        err = wrapAngle(err + 3.14159265f);
    }
    // Forward with a moderate heading error: drive an arc straight onto the
    // target (pure pursuit) instead of stopping to turn in place first.
    if (dir > 0 && !allowReverse && fabsf(err) <= GUIDED_ARC_MAX_DEG * DEG) {
        float look = dist < GUIDED_MAX_STEP_MM ? dist : static_cast<float>(GUIDED_MAX_STEP_MM);
        // The arc bulges away from the straight line that was checked against walls and
        // no-go zones (sagitta = look/2 · tan(err/2)): keep that bulge small
        float half = tanf(fabsf(err) / 2.0f);
        if (half > 0.001f && look / 2.0f * half > GUIDED_ARC_MAX_BULGE_MM)
            look = 2.0f * GUIDED_ARC_MAX_BULGE_MM / half;
        if (look < 80.0f) {
            int arc = static_cast<int>(err * GUIDED_WHEEL_BASE_MM / 2.0f);
            startSegment(SegKind::Turn, -arc, arc, GUIDED_SPEED_TURN);
            return true;
        }
        float l = look;
        float r = look;
        if (fabsf(err) > 1.0f * DEG) {
            // Circle through the pose, tangent to the heading, through the lookahead
            // point at bearing err: chord = look, arc angle = 2·err
            float radius = look / (2.0f * sinf(fabsf(err)));
            float arcAng = 2.0f * fabsf(err);
            float inner = arcAng * (radius - GUIDED_WHEEL_BASE_MM / 2.0f);
            float outer = arcAng * (radius + GUIDED_WHEEL_BASE_MM / 2.0f);
            l = err > 0 ? inner : outer;
            r = err > 0 ? outer : inner;
        }
        startSegment(SegKind::Drive, static_cast<int>(l), static_cast<int>(r), speed);
        return true;
    }
    if (fabsf(err) > GUIDED_HEADING_TOL_DEG * DEG / (allowReverse ? 3.0f : 1.0f)) {
        int arc = static_cast<int>(err * GUIDED_WHEEL_BASE_MM / 2.0f);
        startSegment(SegKind::Turn, -arc, arc, GUIDED_SPEED_TURN);
        return true;
    }
    int stepMM = static_cast<int>(dist < GUIDED_MAX_STEP_MM ? dist : GUIDED_MAX_STEP_MM) * static_cast<int>(dir);
    startSegment(dir > 0 ? SegKind::Drive : SegKind::Reverse, stepMM, stepMM, speed);
    return true;
}

// Called while a forward segment is about to end: decide the next command now and
// send it right away, so the robot keeps rolling instead of stopping at every
// waypoint (a new SetMotor replaces the running one).
void GuidedCleanManager::chainNext() {
    seg = SegKind::None; // No stop command: the next SetMotor takes over
    if (route.empty() || phase == Phase::Docking)
        return;
    chaining = true;
    const Waypoint& t = route[targetIdx];
    float dx = static_cast<float>(t.x) - poseX;
    float dy = static_cast<float>(t.y) - poseY;
    if (sqrtf(dx * dx + dy * dy) < GUIDED_CHAIN_REACH_MM && routeIdx < route.size() - 1) {
        obstacleStreak = 0;
        markReached(targetIdx);
        if (routeIdx < route.size() - 1)
            selectNextTarget();
    }
    step();
    chaining = false;
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

    phase = Phase::Returning;
    curRun = -1;
    if (seg != SegKind::None && seg != SegKind::Backoff) {
        seg = SegKind::None;
        serial.setMotorWheels(0, 0, 0, nullptr);
    }

    // Shortest known-free path to the staging point on the dock axis
    fromDock(GUIDED_DOCK_STAGING_MM, 0.0f, legGoalX, legGoalY);
    std::vector<Waypoint> path;
    if (planPath(legGoalX, legGoalY, path)) {
        setLeg(path, true);
        return;
    }
    fallbackReturn();
}

void GuidedCleanManager::beginDocking() {
    LOG("GUIDED", "At staging point (%.0f,%.0f) — docking", poseX, poseY);
    phase = Phase::Docking;
    dockStage = 0;
    dockIcpNearDone = false;
    distSinceFix = GUIDED_COMPASS_EVERY_MM; // Re-align heading before lining up
    dockIcpTries = 0;
}

// Deterministic dock approach. The dock is the map origin and the robot docks
// rear-first while facing +x, so the final approach is a straight reverse along
// the x axis:
//   0  turn to heading 0 at the staging point
//   3  scan-match against the dock reference scan (same heading → the LIDAR
//      mounting offset cancels) and correct the pose
//   4  (re)drive to the staging point
//   6  reverse towards the pre-back point, steering the rear onto the axis
//   7  re-align heading 0, final scan match close to the dock
//   5  straight back-in onto the contacts → 1 (moving) → 2 (verify)
//   10 give up: hand over to the robot's own SEND_TO_BASE
void GuidedCleanManager::stepDocking() {
    if (dockStage != tracedDockStage) {
        tracedDockStage = dockStage;
        tracePose("dock", {{"stage", String(dockStage), FIELD_INT}, {"try", String(dockAttempts), FIELD_INT}});
    }
    const float staging = GUIDED_DOCK_STAGING_MM;
    const float preback = GUIDED_DOCK_PREBACK_MM;
    bool canMatch = field.valid() || (refScanValid && compassSign != 0);
    // Work in the dock frame (dock pose measured on the dock at the start; the
    // map origin is where the recording started, which is not exactly the dock)
    float lx = 0.0f;
    float ly = 0.0f;
    toDock(poseX, poseY, lx, ly);
    float lt = wrapAngle(poseTheta - dockT);
    float sx = 0.0f;
    float sy = 0.0f;
    fromDock(staging, 0.0f, sx, sy);

    auto faceZero = [this, lt](float tolDeg) {
        float err = wrapAngle(0.0f - lt);
        if (fabsf(err) > tolDeg * DEG) {
            int arc = static_cast<int>(err * GUIDED_WHEEL_BASE_MM / 2.0f);
            startSegment(SegKind::Turn, -arc, arc, GUIDED_SPEED_TURN);
            return true;
        }
        return false;
    };

    switch (dockStage) {
        case 0:
            if (faceZero(3.0f))
                return;
            if (canMatch && dockIcpTries < 2) {
                dockIcpTries++;
                requestIcpScan(4);
                return;
            }
            dockStage = 6;
            return;
        case 4:
            // After a correction: only re-drive if we are clearly off the staging point
            if (fabsf(ly) > 150.0f || fabsf(lx - staging) > 250.0f) {
                if (driveTowards(sx, sy, GUIDED_SPEED_SLOW, true, 60))
                    return;
                dockStage = 0;
                return;
            }
            dockStage = 6;
            return;
        case 6: {
            if (lx <= preback + 20.0f) {
                dockStage = 7;
                return;
            }
            // Reverse pure-pursuit: aim the rear at a point on the axis behind us
            float tx = lx - 350.0f;
            if (tx < preback)
                tx = preback;
            float want = atan2f(ly - 0.0f, lx - tx); // Heading with the rear towards (tx, 0)
            float err = wrapAngle(want - lt);
            if (fabsf(err) > 3.0f * DEG) {
                int arc = static_cast<int>(err * GUIDED_WHEEL_BASE_MM / 2.0f);
                startSegment(SegKind::Turn, -arc, arc, GUIDED_SPEED_TURN);
                return;
            }
            float dx = lx - tx;
            float dy = ly;
            int stepMM = static_cast<int>(fminf(sqrtf(dx * dx + dy * dy), 250.0f));
            if (stepMM < 20) {
                dockStage = 7;
                return;
            }
            startSegment(SegKind::Reverse, -stepMM, -stepMM, GUIDED_SPEED_SLOW);
            return;
        }
        case 7:
            if (faceZero(2.0f))
                return;
            if (canMatch && !dockIcpNearDone) {
                dockIcpNearDone = true;
                requestIcpScan(7);
                return;
            }
            if (fabsf(ly) > GUIDED_DOCK_LATERAL_TOL_MM) {
                // Too far off the axis for a straight back-in: pull forward and redo the line-up
                dockAttempts++;
                if (dockAttempts >= GUIDED_DOCK_ATTEMPTS) {
                    dockStage = 5; // Best effort
                    return;
                }
                dockIcpNearDone = false;
                dockIcpTries = 0;
                dockStage = 4;
                return;
            }
            dockStage = 5;
            return;
        case 5: {
            dockStage = 1;
            int back = static_cast<int>(lx) + 60; // Overshoot slightly: the dock stops us
            startSegment(SegKind::DockBack, -back, -back, GUIDED_SPEED_SLOW);
            return;
        }
        case 10:
            fallbackDock = true;
            finish(Phase::Done, lastError.isEmpty() ? String("docked by the robot") : lastError);
            return;
        default:
            return;
    }
}

void GuidedCleanManager::checkDocked() {
    uint32_t gen = runGen;
    waitingCmd = true;
    serial.getDigitalSensors(
            [this, gen](bool ok, const DigitalSensorData& d) {
                if (gen != runGen)
                    return;
                if (ok && d.dcJackIn) {
                    waitingCmd = false;
                    finish(Phase::Done, lastError);
                    return;
                }
                // Contacts bit not set: the charger may still see dock power
                serial.getCharger([this, gen](bool cok, const ChargerData& c) {
                    if (gen != runGen)
                        return;
                    waitingCmd = false;
                    if (cok && c.extPwrPresent) {
                        finish(Phase::Done, lastError);
                        return;
                    }
                    onDockMissed();
                });
            },
            PRIORITY_HIGH);
}

void GuidedCleanManager::onDockMissed() {
    tracePose("dock_miss", {{"try", String(dockAttempts), FIELD_INT}});
    dockAttempts++;
    if (dockAttempts < GUIDED_DOCK_ATTEMPTS) {
        // Missed the contacts: drive back out to the staging point and line up again
        LOG("GUIDED", "Dock contacts not detected — retrying approach");
        dockStage = 4;
        dockIcpTries = 0;
        dockIcpNearDone = false;
        return;
    }
    // Contacts still not found: hand over to the robot's own docking
    LOG("GUIDED", "Dock contacts not detected — requesting robot docking");
    fallbackDock = true;
    finish(Phase::Done, lastError);
}

void GuidedCleanManager::finish(Phase result, const String& reason) {
    Phase from = phase;
    runGen++; // Invalidate all in-flight callbacks of this run
    scanJob = nullptr;
    uint32_t gen = runGen;
    seg = SegKind::None;
    segStopping = false;
    pollPending = false;
    waitingCmd = false;
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
    serial.setMotorWheels(0, 0, 0, nullptr);
    motorsOn = true; // Force the off commands
    setCleaningMotors(false);
    if (ldsOn)
        serial.setLdsRotation(false, nullptr);
    ldsOn = false;
    serial.testMode(false, [this, gen, result, requestDock](bool ok) {
        if (gen != runGen)
            return;
        inTestMode = false;
        serial.setGuidedCleanActive(false);
        phase = result;
        if (!ok)
            LOG("GUIDED", "TestMode Off failed");
        if (requestDock) {
            serial.clean("dock", nullptr);
            dockWatchSince = millis();
            dockWatchRetried = false;
        }
        freePlan();
    });
}

// -- Obstacle memory ----------------------------------------------------------------------

void GuidedCleanManager::rememberObstacle(float aheadMM, int radius) {
    if (obstacles.size() >= GUIDED_MAX_OBSTACLES)
        obstacles.erase(obstacles.begin());
    // Turning in place / backing up: the contact is beside or behind — use a smaller offset
    float dir = (seg == SegKind::Backoff || seg == SegKind::DockBack || seg == SegKind::Reverse) ? -1.0f : 1.0f;
    Obstacle o;
    o.x = static_cast<int16_t>(poseX + dir * aheadMM * cosf(poseTheta));
    o.y = static_cast<int16_t>(poseY + dir * aheadMM * sinf(poseTheta));
    o.r = static_cast<int16_t>(radius);
    obstacles.push_back(o);
}

// -- LIDAR wall compass ------------------------------------------------------------------

// Dominant wall direction (mod 90°) in the robot frame, as the circular mean of
// 4× the local line angles between neighbouring scan points. conf = resultant
// length (1 = all segments axis-aligned with each other, 0 = no structure).
bool GuidedCleanManager::dominantWallAngle(const LdsScanData& scan, float& phi, float& conf) {
    float sumC = 0.0f;
    float sumS = 0.0f;
    float sumW = 0.0f;
    int used = 0;
    for (int i = 0; i < 360; i++) {
        const LdsScanPoint& a = scan.points[i];
        const LdsScanPoint& b = scan.points[(i + 2) % 360];
        if (a.errorCode != 0 || b.errorCode != 0 || a.distMM < 150 || b.distMM < 150 || a.distMM > 5000 ||
            b.distMM > 5000)
            continue;
        float ra = static_cast<float>(a.angleDeg) * DEG;
        float rb = static_cast<float>(b.angleDeg) * DEG;
        float ax = static_cast<float>(a.distMM) * cosf(ra);
        float ay = static_cast<float>(a.distMM) * sinf(ra);
        float bx = static_cast<float>(b.distMM) * cosf(rb);
        float by = static_cast<float>(b.distMM) * sinf(rb);
        float dx = bx - ax;
        float dy = by - ay;
        float len = sqrtf(dx * dx + dy * dy);
        if (len < 20.0f || len > 400.0f)
            continue; // Gap between objects, not a surface
        float ang = atan2f(dy, dx);
        sumC += len * cosf(4.0f * ang);
        sumS += len * sinf(4.0f * ang);
        sumW += len;
        used++;
    }
    if (used < 40 || sumW <= 0.0f)
        return false;
    conf = sqrtf(sumC * sumC + sumS * sumS) / sumW;
    phi = atan2f(sumS, sumC) / 4.0f;
    return true;
}

// Map an angle to [-45°, 45°)
static float wrapQuarter(float a) {
    const float q = 3.14159265f / 2.0f;
    while (a >= q / 2.0f)
        a -= q;
    while (a < -q / 2.0f)
        a += q;
    return a;
}

void GuidedCleanManager::requestScan(bool reference) {
    uint32_t gen = runGen;
    waitingCmd = true;
    lastScanMs = millis();
    fetchScan([this, gen, reference](bool ok, const LdsScanData& scan) {
        if (gen != runGen)
            return;
        waitingCmd = false;
        distSinceFix = 0.0f;
        turnSinceFix = 0.0f;
        if (ok)
            applyCompass(scan, reference);
    });
}

void GuidedCleanManager::applyCompass(const LdsScanData& scan, bool reference) {
    float phi = 0.0f;
    float conf = 0.0f;
    if (reference) {
        for (int i = 0; i < 360; i++) {
            const LdsScanPoint& p = scan.points[i];
            bool valid = p.errorCode == 0 && p.distMM >= 150 && p.distMM <= 6000;
            refScan[i] = valid ? static_cast<int16_t>(p.distMM) : static_cast<int16_t>(0);
        }
        refScanValid = true;
    }
    if (!dominantWallAngle(scan, phi, conf) || conf < GUIDED_COMPASS_MIN_CONF) {
        if (reference)
            LOG("GUIDED", "Compass: no clear wall direction at the dock (conf %.2f) — odometry only", conf);
        return;
    }

    if (reference) {
        compassRef = true;
        compassPhi0 = phi;
        LOG("GUIDED", "Compass reference %.1f° (conf %.2f)", phi / DEG, conf);
        return;
    }

    // World wall direction W is constant. With LDS angles CCW (sign +1):
    // W = theta + phi, so theta = phi0 - phi (dock heading is 0). With CW
    // angles the robot-frame angle flips sign.
    if (compassSign == 0) {
        // The two hypotheses predict theta and -theta (mod 90°): they can only be
        // told apart away from multiples of 45°, hence the calibration turn to
        // ~22.5° (mod 45°) right after leaving the dock.
        float predCcw = compassPhi0 - phi;
        float predCw = phi - compassPhi0;
        float separation = fabsf(wrapQuarter(predCcw - predCw));
        float resCcw = fabsf(wrapQuarter(predCcw - poseTheta));
        float resCw = fabsf(wrapQuarter(predCw - poseTheta));
        if (separation < 20.0f * DEG)
            return; // Indistinguishable at this heading
        if (resCcw < 10.0f * DEG && resCcw < resCw) {
            compassSign = 1;
        } else if (resCw < 10.0f * DEG && resCw < resCcw) {
            compassSign = -1;
        } else {
            return; // Inconsistent — leave uncalibrated
        }
        LOG("GUIDED", "Compass calibrated: LDS angles %s", compassSign > 0 ? "CCW" : "CW");
        event("compass", {{"direction", compassSign > 0 ? "ccw" : "cw", FIELD_STRING}});
    }

    float predicted = compassSign > 0 ? compassPhi0 - phi : phi - compassPhi0;
    float delta = wrapQuarter(predicted - poseTheta);
    if (fabsf(delta) > GUIDED_COMPASS_MAX_FIX_DEG * DEG) {
        LOG("GUIDED", "Compass: rejected %.1f° correction (conf %.2f)", delta / DEG, conf);
        return;
    }
    poseTheta = wrapAngle(poseTheta + delta);
    compassFixes++;
    lastFixDeg = delta / DEG;
    if (fabsf(lastFixDeg) > 1.0f)
        LOG("GUIDED", "Compass: heading corrected by %.1f° (conf %.2f)", lastFixDeg, conf);
}

// -- Map localization -----------------------------------------------------------------------

void GuidedCleanManager::requestLocScan(int radiusMM, int angleDeg, uint8_t resumeStage) {
    uint32_t gen = runGen;
    waitingCmd = true;
    lastScanMs = millis();
    mcX = poseX;
    mcY = poseY;
    mcT = poseTheta;
    fetchScan([this, gen, radiusMM, angleDeg, resumeStage](bool ok, const LdsScanData& scan) {
        if (gen != runGen)
            return;
        waitingCmd = false;
        distSinceFix = 0.0f;
        turnSinceFix = 0.0f;
        if (resumeStage != 0xFF)
            dockStage = resumeStage;
        bool first = ok && dockScanPending;
        if (first) {
            dockScanPending = false;
            traceDockScan(scan);
        }
        if (ok && matchField(scan, radiusMM, angleDeg) && first) {
            // The robot sits on the dock: this fix is where the dock is in the map
            dockX = poseX;
            dockY = poseY;
            dockT = poseTheta;
            tracePose("dock_pose", {});
        }
    });
}

// Scan match while the robot keeps driving: the scan reflects (roughly) the pose
// at request time, so the search is centred there and the correction is applied
// as a delta to the then-current pose. Does not block the segment executor.
void GuidedCleanManager::requestLocScanMoving() {
    uint32_t gen = runGen;
    locMovingPending = true;
    lastScanMs = millis();
    mcX = poseX;
    mcY = poseY;
    mcT = poseTheta;
    distSinceFix = 0.0f;
    turnSinceFix = 0.0f;
    fetchScan([this, gen](bool ok, const LdsScanData& scan) {
        if (gen != runGen)
            return;
        locMovingPending = false;
        if (ok)
            matchField(scan, GUIDED_LOC_RADIUS_MM, GUIDED_LOC_ANGLE_DEG);
    });
}

// Correlative scan matching: score candidate poses by summing the likelihood
// field under the scan points. Coarse grid search (field cells × 2° steps),
// then a bilinear refinement around the best candidate. Integer cell offsets
// in the coarse stage keep this to a few ms on the ESP32.
bool GuidedCleanManager::matchField(const LdsScanData& scan, int radiusMM, int angleDeg) {
    // Scan points in the robot frame (LIDAR mounting from the map calibration)
    // Scratch buffers are members: this runs on the 8 KB loop stack
    float *px = mfPx;
    float *py = mfPy;
    int n = 0;
    auto sgn = static_cast<float>(field.lidarSign);
    for (int i = 0; i < 360 && n < 180; i += 2) {
        const LdsScanPoint& p = scan.points[i];
        if (p.errorCode != 0 || p.distMM < 150 || p.distMM > 5000)
            continue;
        float a = sgn * static_cast<float>(p.angleDeg) * DEG + field.lidarAngleOffsetRad;
        px[n] = static_cast<float>(field.lidarOffsetX) + static_cast<float>(p.distMM) * cosf(a);
        py[n] = static_cast<float>(p.distMM) * sinf(a);
        n++;
    }
    if (n < 40) {
        LOG("GUIDED", "Loc: too few scan points (%d)", n);
        return false;
    }

    const auto cell = static_cast<float>(field.cell);
    const int steps = radiusMM / field.cell;
    float bestScore = -1.0f;
    // Search around the pose at scan time (the robot may have moved since)
    const float cX = mcX;
    const float cY = mcY;
    const float cT = mcT;
    float bestT = cT;
    float bestX = cX;
    float bestY = cY;
    int *bx = mfBx;
    int *by = mfBy;
    for (int da = -angleDeg; da <= angleDeg; da += 2) {
        float th = cT + static_cast<float>(da) * DEG;
        float c = cosf(th);
        float s = sinf(th);
        for (int i = 0; i < n; i++) {
            float wx = cX + c * px[i] - s * py[i];
            float wy = cY + s * px[i] + c * py[i];
            bx[i] = static_cast<int>(floorf((wx - static_cast<float>(field.originX)) / cell));
            by[i] = static_cast<int>(floorf((wy - static_cast<float>(field.originY)) / cell));
        }
        for (int ky = -steps; ky <= steps; ky++) {
            for (int kx = -steps; kx <= steps; kx++) {
                int sum = 0;
                for (int i = 0; i < n; i++) {
                    int cx = bx[i] + kx;
                    int cy = by[i] + ky;
                    if (cx < 0 || cy < 0 || cx >= field.w || cy >= field.h)
                        continue;
                    sum += field.data[static_cast<size_t>(cy) * static_cast<size_t>(field.w) + static_cast<size_t>(cx)];
                }
                // Slight preference for small corrections breaks ties on repetitive walls
                float score = static_cast<float>(sum) - static_cast<float>(abs(kx) + abs(ky) + abs(da) / 2) * 2.0f;
                if (score > bestScore) {
                    bestScore = score;
                    bestT = th;
                    bestX = cX + static_cast<float>(kx) * cell;
                    bestY = cY + static_cast<float>(ky) * cell;
                }
            }
        }
    }

    // Refinement with bilinear sampling (sub-cell precision)
    auto sample = [this](float x, float y) {
        float fx = (x - static_cast<float>(field.originX)) / static_cast<float>(field.cell) - 0.5f;
        float fy = (y - static_cast<float>(field.originY)) / static_cast<float>(field.cell) - 0.5f;
        int x0 = static_cast<int>(floorf(fx));
        int y0 = static_cast<int>(floorf(fy));
        float tx = fx - static_cast<float>(x0);
        float ty = fy - static_cast<float>(y0);
        auto v = [this](int cx, int cy) -> float {
            if (cx < 0 || cy < 0 || cx >= field.w || cy >= field.h)
                return 0.0f;
            return field.data[static_cast<size_t>(cy) * static_cast<size_t>(field.w) + static_cast<size_t>(cx)];
        };
        return (v(x0, y0) * (1 - tx) + v(x0 + 1, y0) * tx) * (1 - ty) +
               (v(x0, y0 + 1) * (1 - tx) + v(x0 + 1, y0 + 1) * tx) * ty;
    };
    auto scoreAt = [&](float x, float y, float th) {
        float c = cosf(th);
        float s = sinf(th);
        float sum = 0.0f;
        for (int i = 0; i < n; i++)
            sum += sample(x + c * px[i] - s * py[i], y + s * px[i] + c * py[i]);
        return sum;
    };
    float refined = scoreAt(bestX, bestY, bestT);
    float step = cell / 2.0f;
    float astep = 1.0f * DEG;
    for (int iter = 0; iter < 6; iter++) {
        bool improved = false;
        const float cand[6][3] = {{step, 0, 0},  {-step, 0, 0}, {0, step, 0},
                                  {0, -step, 0}, {0, 0, astep}, {0, 0, -astep}};
        for (const auto& d: cand) {
            float sc = scoreAt(bestX + d[0], bestY + d[1], bestT + d[2]);
            if (sc > refined) {
                refined = sc;
                bestX += d[0];
                bestY += d[1];
                bestT += d[2];
                improved = true;
            }
        }
        if (!improved) {
            step /= 2.0f;
            astep /= 2.0f;
        }
    }

    float norm = refined / (static_cast<float>(n) * 255.0f);
    float before = scoreAt(cX, cY, cT) / (static_cast<float>(n) * 255.0f);
    locScore = norm;
    if (norm < GUIDED_LOC_MIN_SCORE || norm + 0.02f < before) {
        LOG("GUIDED", "Loc: rejected (score %.2f, at odometry pose %.2f)", norm, before);
        tracePose("loc", {{"mx", String(static_cast<int>(bestX)), FIELD_INT},
                          {"my", String(static_cast<int>(bestY)), FIELD_INT},
                          {"mt", String(static_cast<int>(bestT / DEG)), FIELD_INT},
                          {"s", String(norm, 2), FIELD_FLOAT},
                          {"s0", String(before, 2), FIELD_FLOAT},
                          {"ok", "false", FIELD_BOOL}});
        return false;
    }
    tracePose("loc", {{"mx", String(static_cast<int>(bestX)), FIELD_INT},
                      {"my", String(static_cast<int>(bestY)), FIELD_INT},
                      {"mt", String(static_cast<int>(bestT / DEG)), FIELD_INT},
                      {"s", String(norm, 2), FIELD_FLOAT},
                      {"s0", String(before, 2), FIELD_FLOAT},
                      {"ok", "true", FIELD_BOOL}});
    // Apply the correction found at scan time to the current pose
    float dx = bestX - cX;
    float dy = bestY - cY;
    float dt = wrapAngle(bestT - cT);
    poseX += dx;
    poseY += dy;
    poseTheta = wrapAngle(poseTheta + dt);
    locFixes++;
    if (fabsf(dx) + fabsf(dy) > 60.0f || fabsf(dt) > 2.0f * DEG)
        LOG("GUIDED", "Loc: corrected %.0f,%.0f mm %.1f° (score %.2f)", dx, dy, dt / DEG, norm);
    return true;
}

// -- Dock homing (ICP against the reference scan) ------------------------------------------

void GuidedCleanManager::requestIcpScan(uint8_t resumeStage) {
    if (field.valid()) {
        dockStage = 3;
        requestLocScan(GUIDED_LOC_RADIUS_MM, GUIDED_LOC_ANGLE_DEG, resumeStage);
        return;
    }
    uint32_t gen = runGen;
    waitingCmd = true;
    dockStage = 3;
    lastScanMs = millis();
    fetchScan([this, gen, resumeStage](bool ok, const LdsScanData& scan) {
        if (gen != runGen)
            return;
        waitingCmd = false;
        dockStage = resumeStage;
        float cx = 0.0f;
        float cy = 0.0f;
        float ct = 0.0f;
        if (ok && matchReference(scan, cx, cy, ct)) {
            poseX += cx;
            poseY += cy;
            poseTheta = wrapAngle(poseTheta + ct);
            LOG("GUIDED", "Dock ICP: corrected by %.0f,%.0f mm %.1f° → pose %.0f,%.0f", cx, cy, ct / DEG, poseX, poseY);
            event("dock_match", {{"dx", String(static_cast<int>(cx)), FIELD_INT},
                                 {"dy", String(static_cast<int>(cy)), FIELD_INT},
                                 {"x", String(static_cast<int>(poseX)), FIELD_INT},
                                 {"y", String(static_cast<int>(poseY)), FIELD_INT}});
            return;
        }
        LOG("GUIDED", "Dock ICP: no reliable match — continuing on odometry");
    });
}

// Point-to-point ICP between the current scan (placed with the current pose
// estimate) and the reference scan taken on the dock (pose 0,0,0). Both scans
// are taken at heading ~0, so the LIDAR mounting offset cancels out. Integer
// nearest-neighbour search keeps it fast on the FPU-less ESP32-C3.
// Returns the pose correction to apply (world frame).
bool GuidedCleanManager::matchReference(const LdsScanData& scan, float& corrX, float& corrY, float& corrT) const {
    struct P {
        int32_t x;
        int32_t y;
    };
    std::vector<P> ref;
    std::vector<P> cur;
    ref.reserve(360);
    cur.reserve(180);
    auto sgn = static_cast<float>(compassSign);
    for (int i = 0; i < 360; i++) {
        if (refScan[i] <= 0)
            continue;
        float a = sgn * static_cast<float>(i) * DEG;
        auto d = static_cast<float>(refScan[i]);
        ref.push_back({static_cast<int32_t>(d * cosf(a)), static_cast<int32_t>(d * sinf(a))});
    }
    float c = cosf(poseTheta);
    float s = sinf(poseTheta);
    for (int i = 0; i < 360; i += 2) {
        const LdsScanPoint& p = scan.points[i];
        if (p.errorCode != 0 || p.distMM < 150 || p.distMM > 6000)
            continue;
        float a = sgn * static_cast<float>(p.angleDeg) * DEG;
        float lx = static_cast<float>(p.distMM) * cosf(a);
        float ly = static_cast<float>(p.distMM) * sinf(a);
        cur.push_back({static_cast<int32_t>(poseX + c * lx - s * ly), static_cast<int32_t>(poseY + s * lx + c * ly)});
    }
    if (ref.size() < 60 || cur.size() < 40)
        return false;

    // Accumulated transform cur → ref: rotate by tt around origin, then translate (tx, ty)
    float tt = 0.0f;
    float tx = 0.0f;
    float ty = 0.0f;
    float rms = 0.0f;
    size_t inliers = 0;
    int32_t gate = 400;
    struct Pair {
        float qx;
        float qy;
        float rx;
        float ry;
    };
    std::vector<Pair> pairs;
    pairs.reserve(cur.size());
    for (int iter = 0; iter < 20; iter++) {
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
        // Cross-covariance of the centred point sets (two-pass for float precision)
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
        // Incremental step: rotate the (already transformed) cur set about the origin, then translate
        float stepX = mrx - (cd * mx - sd * my);
        float stepY = mry - (sd * mx + cd * my);
        // Compose: new = step ∘ old
        float ntx = cd * tx - sd * ty + stepX;
        float nty = sd * tx + cd * ty + stepY;
        tt += dTheta;
        tx = ntx;
        ty = nty;
        rms = sqrtf(err2 / n);
        if (gate > 150)
            gate -= 50;
        if (fabsf(dTheta) < 0.0005f && fabsf(stepX) < 2.0f && fabsf(stepY) < 2.0f && gate <= 150)
            break;
    }

    float ratio = static_cast<float>(inliers) / static_cast<float>(cur.size());
    if (ratio < 0.5f || rms > 60.0f)
        return false;

    // Apply the cur→ref transform to the robot pose
    float ct = cosf(tt);
    float st = sinf(tt);
    float nx = ct * poseX - st * poseY + tx;
    float ny = st * poseX + ct * poseY + ty;
    corrX = nx - poseX;
    corrY = ny - poseY;
    corrT = tt;
    if (sqrtf(corrX * corrX + corrY * corrY) > GUIDED_ICP_MAX_CORR_MM || fabsf(tt) > 20.0f * DEG)
        return false;
    return true;
}

// -- Status -----------------------------------------------------------------------------

String GuidedCleanManager::getStatusJson() const {
    unsigned long endMs = finishedMs > 0 && !isActive() ? finishedMs : millis();
    unsigned long elapsed = startedMs > 0 ? (endMs - startedMs) / 1000UL : 0;
    int progress = cleanTotal > 0 ? (cleanDone * 100) / cleanTotal : 0;
    if (phase == Phase::Done && lastError.isEmpty())
        progress = 100;

    std::vector<Field> fields = {
            {"phase", phaseName(phase), FIELD_STRING},
            {"active", isActive() ? "true" : "false", FIELD_BOOL},
            {"hasMap", floorMap.hasMap() ? "true" : "false", FIELD_BOOL},
            {"rooms", roomsLabel, FIELD_STRING},
            {"x", String(poseX / 1000.0f, 3), FIELD_FLOAT},
            {"y", String(poseY / 1000.0f, 3), FIELD_FLOAT},
            {"t", String(wrapAngle(poseTheta) / DEG, 1), FIELD_FLOAT},
            {"progress", String(progress), FIELD_INT},
            {"cleanDone", String(cleanDone), FIELD_INT},
            {"cleanTotal", String(cleanTotal), FIELD_INT},
            {"obstacles", String(obstacleTotal), FIELD_INT},
            {"cleaning", motorsOn ? "true" : "false", FIELD_BOOL},
            {"elapsed", String(elapsed), FIELD_INT},
            {"startedAt", String(static_cast<long>(startedEpoch)), FIELD_INT},
            {"message", lastError, FIELD_STRING},
            {"compass", compassRef ? (compassSign != 0 ? "locked" : "reference") : (ldsOn ? "pending" : "off"),
             FIELD_STRING},
            {"compassFixes", String(compassFixes), FIELD_INT},
            {"localization", field.valid() ? "map" : (compassRef ? "compass" : "odometry"), FIELD_STRING},
            {"locFixes", String(locFixes), FIELD_INT},
            {"locScore", String(locScore, 2), FIELD_FLOAT},
    };
    if (isActive() && !route.empty() && targetIdx < route.size()) {
        fields.push_back({"targetX", String(static_cast<float>(route[targetIdx].x) / 1000.0f, 3), FIELD_FLOAT});
        fields.push_back({"targetY", String(static_cast<float>(route[targetIdx].y) / 1000.0f, 3), FIELD_FLOAT});
    }
    return fieldsToJson(fields);
}

// -- Run trace (always on, /map/trace.jsonl, served by GET /api/guided/trace) -------------------
// One JSON object per line: {"e":<event>,"ms":<since start>,"x","y","t"(deg), ...}. Pose lines
// ("p") every GUIDED_TRACE_EVERY_MM, localization fixes ("loc": matched pose mx/my/mt, score
// s vs score s0 at the odometry pose), legs, obstacles, dock stages and the result. Buffered
// in RAM and appended in chunks to keep flash writes rare.

void GuidedCleanManager::traceBegin() {
    traceBuf = "";
    traceBytes = 0;
    traceDist = 0.0f;
    traceTurn = 0.0f;
    tracedDockStage = 255;
    traceStartMs = millis();
    SPIFFS.remove(GUIDED_TRACE_FILE);
}

void GuidedCleanManager::event(const String& name, const std::vector<Field>& fields) {
    dataLogger.logGuided(name, fields);
    tracePose(name, fields);
}

void GuidedCleanManager::tracePose(const String& name, const std::vector<Field>& fields) {
    bool important = name != "p" && name != "loc";
    if (traceBytes + traceBuf.length() > GUIDED_TRACE_MAX_BYTES - (important ? 0 : 4096))
        return; // Full: keep room for the remaining events
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

// GetLDSScan with deferred processing: the result is copied into scanBuf and the
// handler runs from tick(). Matching inside the serial callback chain sat on top
// of the ~6 KB parse frame and overflowed the 8 KB loop stack on the ESP32.
void GuidedCleanManager::fetchScan(std::function<void(bool, const LdsScanData&)> handler) {
    uint32_t gen = runGen;
    serial.getLdsScanFresh([this, gen, handler](bool ok, const LdsScanData& scan) {
        if (gen != runGen)
            return;
        if (ok)
            scanBuf = scan; // Trivially copyable: in-place copy, no stack temporary
        scanJobOk = ok;
        scanJob = handler;
    });
}

// On the dock: keep the close-range LIDAR returns (angle, distance, intensity)
// in the trace. The dock's reflectors should stand out by intensity — data for
// a LIDAR-guided final approach.
void GuidedCleanManager::traceDockScan(const LdsScanData& scan) {
    String pts;
    pts.reserve(1600);
    for (int i = 0; i < 360; i++) {
        const LdsScanPoint& p = scan.points[i];
        if (p.errorCode != 0 || p.distMM <= 0 || p.distMM > 1200)
            continue;
        if (pts.length() > 1500)
            break;
        pts += String(p.angleDeg) + ":" + String(p.distMM) + ":" + String(p.intensity) + " ";
    }
    tracePose("dockscan", {{"pts", pts, FIELD_STRING}});
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

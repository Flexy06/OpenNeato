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

    // The dock is the map origin; refuse plans that start inside a no-go zone
    if (nearNogo(0.0f, 0.0f))
        return "the dock lies inside a no-go zone";

    // Coverage grid over the recorded path (+ margin)
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

    // Cleaning runs: recorded points inside the selected rooms (and clear of
    // no-go zones), split wherever the recording left the rooms or jumped.
    // Runs are visited nearest-first; transit between them is planned on the
    // coverage grid, so the robot no longer replays detours through other rooms.
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

    if (cleanPts.empty()) {
        freePlan();
        return "the pinned path never enters the selected rooms";
    }
    cleanTotal = static_cast<int>(cleanPts.size());
    cleanPts.shrink_to_fit();

    buildGrid();

    LOG("GUIDED", "Plan: %d clean points in %u runs, %u no-go zones, grid %dx%d @%dmm", cleanTotal,
        static_cast<unsigned>(runs.size()), static_cast<unsigned>(nogos.size()), grid.w, grid.h, grid.cell);
    return "";
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
            bool ok = hits * 2 >= sub * sub && !nearNogo(grid.centerX(cx), grid.centerY(cy));
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
    dataLogger.logGuided("skip_run", {{"run", String(curRun), FIELD_INT}, {"reason", why, FIELD_STRING}});
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
        fail("path back to the dock is blocked");
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
    Waypoint staging;
    staging.x = GUIDED_DOCK_STAGING_MM;
    staging.y = 0;
    back.push_back(staging);
    legGoalX = GUIDED_DOCK_STAGING_MM;
    legGoalY = 0.0f;
    setLeg(back, true);
    legReplans = GUIDED_MAX_REPLANS + 1; // No further re-planning on this leg
}

void GuidedCleanManager::freePlan() {
    std::vector<Waypoint>().swap(route);
    std::vector<Waypoint>().swap(cleanPts);
    std::vector<Run>().swap(runs);
    std::vector<MapPoint>().swap(crumbs);
    grid.clear();
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
        phase = Phase::Starting;
        lastError = "";
        roomsLabel = rooms;
        startedMs = millis();
        startedEpoch = sysMgr.now();
        finishedMs = 0;
        obstacleStreak = 0;
        obstacleTotal = 0;
        fallbackDock = false;
        inTestMode = false;
        dockStage = 0;
        dockAttempts = 0;
        dockIcpNearDone = false;
        ldsOn = false;
        compassTried = false;
        compassRef = false;
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
        dataLogger.logGuided("start", {{"rooms", rooms.isEmpty() ? String("all") : rooms, FIELD_STRING},
                                       {"runs", String(static_cast<unsigned>(runs.size())), FIELD_INT},
                                       {"clean", String(cleanTotal), FIELD_INT}});
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
    if (!isActive() || phase == Phase::Starting || phase == Phase::Finishing)
        return;

    // Sensor polling runs continuously while active (odometry + safety)
    if (!pollPending && pollTicker.elapsed(GUIDED_POLL_MS) && serial.queueDepth() <= NEATO_QUEUE_MAX_SIZE / 2)
        pollSensors();

    if (waitingCmd || segStopping)
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
    dataLogger.logGuided("obstacle", {{"reason", reason, FIELD_STRING},
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
    // Wall compass: reference scan on the dock once the LIDAR has spun up,
    // then periodic re-alignment while stopped between segments.
    if (ldsOn && !compassTried) {
        if (millis() < ldsReadyAtMs)
            return;
        compassTried = true;
        requestScan(true);
        return;
    }
    // Direction calibration: once clear of the dock, turn to a heading of
    // ~22.5° (mod 45°) where CW and CCW LIDAR angle conventions disagree most,
    // scan, and lock the sign. Retried up to 3 times along the route.
    if (compassRef && compassSign == 0 && phase == Phase::Running && calAttempts < 3 &&
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
    if (compassRef && compassSign != 0 &&
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
    if (fabsf(err) > GUIDED_HEADING_TOL_DEG * DEG / (allowReverse ? 3.0f : 1.0f)) {
        int arc = static_cast<int>(err * GUIDED_WHEEL_BASE_MM / 2.0f);
        startSegment(SegKind::Turn, -arc, arc, GUIDED_SPEED_TURN);
        return true;
    }
    int stepMM = static_cast<int>(dist < GUIDED_MAX_STEP_MM ? dist : GUIDED_MAX_STEP_MM) * static_cast<int>(dir);
    startSegment(dir > 0 ? SegKind::Drive : SegKind::Reverse, stepMM, stepMM, speed);
    return true;
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
    dataLogger.logGuided("return", {{"reason", reason, FIELD_STRING},
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
    legGoalX = GUIDED_DOCK_STAGING_MM;
    legGoalY = 0.0f;
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
    const float staging = GUIDED_DOCK_STAGING_MM;
    const float preback = GUIDED_DOCK_PREBACK_MM;
    bool canMatch = refScanValid && compassSign != 0;

    auto faceZero = [this](float tolDeg) {
        float err = wrapAngle(0.0f - poseTheta);
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
            if (fabsf(poseY) > 150.0f || fabsf(poseX - staging) > 250.0f) {
                if (driveTowards(staging, 0.0f, GUIDED_SPEED_SLOW, true, 60))
                    return;
                dockStage = 0;
                return;
            }
            dockStage = 6;
            return;
        case 6: {
            if (poseX <= preback + 20.0f) {
                dockStage = 7;
                return;
            }
            // Reverse pure-pursuit: aim the rear at a point on the axis behind us
            float tx = poseX - 350.0f;
            if (tx < preback)
                tx = preback;
            float want = atan2f(poseY - 0.0f, poseX - tx); // Heading with the rear towards (tx, 0)
            float err = wrapAngle(want - poseTheta);
            if (fabsf(err) > 3.0f * DEG) {
                int arc = static_cast<int>(err * GUIDED_WHEEL_BASE_MM / 2.0f);
                startSegment(SegKind::Turn, -arc, arc, GUIDED_SPEED_TURN);
                return;
            }
            float dx = poseX - tx;
            float dy = poseY;
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
            if (fabsf(poseY) > GUIDED_DOCK_LATERAL_TOL_MM) {
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
            int back = static_cast<int>(poseX) + 60; // Overshoot slightly: the dock stops us
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
                waitingCmd = false;
                if (ok && d.dcJackIn) {
                    finish(Phase::Done, lastError);
                    return;
                }
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
            },
            PRIORITY_HIGH);
}

void GuidedCleanManager::finish(Phase result, const String& reason) {
    Phase from = phase;
    runGen++; // Invalidate all in-flight callbacks of this run
    uint32_t gen = runGen;
    seg = SegKind::None;
    segStopping = false;
    pollPending = false;
    waitingCmd = false;
    lastError = reason;
    finishedMs = millis();

    LOG("GUIDED", "Finish: %s → %s (%s)", phaseName(from), phaseName(result), reason.c_str());
    dataLogger.logGuided(phaseName(result), {{"reason", reason, FIELD_STRING},
                                             {"cleaned", String(cleanDone), FIELD_INT},
                                             {"total", String(cleanTotal), FIELD_INT},
                                             {"obstacles", String(obstacleTotal), FIELD_INT}});

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
        if (requestDock)
            serial.clean("dock", nullptr);
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
    serial.getLdsScan([this, gen, reference](bool ok, const LdsScanData& scan) {
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
        dataLogger.logGuided("compass", {{"direction", compassSign > 0 ? "ccw" : "cw", FIELD_STRING}});
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

// -- Dock homing (ICP against the reference scan) ------------------------------------------

void GuidedCleanManager::requestIcpScan(uint8_t resumeStage) {
    uint32_t gen = runGen;
    waitingCmd = true;
    dockStage = 3;
    lastScanMs = millis();
    serial.getLdsScan([this, gen, resumeStage](bool ok, const LdsScanData& scan) {
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
            dataLogger.logGuided("dock_match", {{"dx", String(static_cast<int>(cx)), FIELD_INT},
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
    };
    if (isActive() && !route.empty() && targetIdx < route.size()) {
        fields.push_back({"targetX", String(static_cast<float>(route[targetIdx].x) / 1000.0f, 3), FIELD_FLOAT});
        fields.push_back({"targetY", String(static_cast<float>(route[targetIdx].y) / 1000.0f, 3), FIELD_FLOAT});
    }
    return fieldsToJson(fields);
}

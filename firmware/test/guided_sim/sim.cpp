#include <fstream>
#include <cctype>
// Host-side simulation of GuidedCleanManager against a physics model.
// Build + run: ./run.sh   (see README.md in this folder)
#include <algorithm>
#include <cassert>
#include <cstring>
#include <set>
#include <string>
#include <unistd.h>
#include "floor_map.h"
#include "guided_clean_manager.h"
#include "manual_clean_manager.h"
#include "system_manager.h"

unsigned long g_simMillis = 1;
FakeSerial Serial;
FakeFS SPIFFS;
static std::string g_fsDir;
std::string fakeFsPath(const char *p) {
    std::string s = p;
    for (auto& c: s) if (c == '/') c = '_';
    return g_fsDir + "/" + s;
}
static bool g_quietLog = true;
void DataLogger::logGuided(const String& ev, const std::vector<Field>& extra) {
    if (g_quietLog && ev.str() != "obstacle" && ev.str() != "timeout" && ev.str() != "return" && ev.str() != "done" &&
        ev.str() != "failed" && ev.str() != "stopped" && ev.str() != "dock_result")
        return;
    std::string line = "  [log] guided " + ev.str();
    for (auto& f: extra) line += " " + f.key.str() + "=" + f.value.str();
    std::printf("%s\n", line.c_str());
}

struct Pt { double x, y; };
struct Rect { double x0, y0, x1, y1; };

// -- Worlds ---------------------------------------------------------------------------
// 0: two rooms joined by a 1 m door, one no-go zone in room 2
// 1: flat like the real one: narrow hallway ("Flur") with the dock at one end and a
//    cabinet right next to it, 80 cm door into a living room with table legs, a sofa,
//    a carpet (slip + heading kicks) and chairs that were not there when mapping
static int g_world = 0;

static void buildWorld(NeatoSerial& s) {
    if (g_world == 0) {
        s.walls = {{-170, -2000, -170, 2000}, {6000, -2000, 6000, 2000}, {-170, -2000, 6000, -2000},
                   {-170, 2000, 6000, 2000},  {3000, -2000, 3000, -500}, {3000, 500, 3000, 2000}};
        return;
    }
    s.walls = {{-170, -600, -170, 600},   {-170, -600, 4500, -600}, {4500, -600, 4500, 3500},
               {-170, 600, 1500, 600},    {2300, 600, 4500, 600},   {1000, 600, 1000, 3500},
               {1000, 3500, 4500, 3500},
               // Sofa against the top wall
               {1100, 3000, 2600, 3000},  {2600, 3000, 2600, 3500}, {1100, 3000, 1100, 3500}};
    s.boxes = {{330, 470, 110}, // Cabinet next to the dock (mapped)
               {3000, 1800, 25}, {3800, 1800, 25}, {3000, 2600, 25}, {3800, 2600, 25}}; // Table legs
    s.carpetX0 = 2400;
    s.carpetY0 = 1300;
    s.carpetX1 = 4300;
    s.carpetY1 = 2900;
}

// Obstacles placed after mapping (not in the field)
static void addUnmapped(NeatoSerial& s) {
    if (g_world == 1) {
        s.boxes.push_back({3400, 1450, 150}); // Chair pulled out from the table
        s.boxes.push_back({2600, 2200, 150});
    }
}

static std::vector<Pt> recordedPath() {
    std::vector<Pt> p;
    auto lane = [&](double x0, double x1, double y) {
        double step = 500 * (x1 > x0 ? 1 : -1);
        for (double x = x0; (step > 0 ? x <= x1 : x >= x1); x += step) p.push_back({x, y});
        p.push_back({x1, y});
    };
    if (g_world == 0) {
        p.push_back({300, 0});
        bool dir = true;
        for (double y = -1700; y <= 1700; y += 300, dir = !dir) dir ? lane(300, 2700, y) : lane(2700, 300, y);
        p.push_back({2700, 0});
        p.push_back({3300, 0});
        dir = true;
        for (double y = -1700; y <= 1700; y += 300, dir = !dir) dir ? lane(3300, 5700, y) : lane(5700, 3300, y);
        p.push_back({3300, 0});
        p.push_back({2700, 0});
        p.push_back({1500, 0});
        p.push_back({300, 0});
        return p;
    }
    p.push_back({300, 0});
    lane(300, 4200, 0);
    p.push_back({1900, 0});
    p.push_back({1900, 900});
    bool dir = true;
    for (double y = 900; y <= 2800; y += 400, dir = !dir) dir ? lane(1300, 4200, y) : lane(4200, 1300, y);
    p.push_back({1900, 900});
    p.push_back({1900, 0});
    p.push_back({300, 0});
    return p;
}

static Rect nogoRect() { return g_world == 0 ? Rect{4200, 600, 4900, 1400} : Rect{3950, 3000, 4500, 3500}; }

// Map frame = world + offset: the recording's origin is not exactly the dock
static int g_mapOffX = 0;
static int g_mapOffY = 0;

static std::string mapPt(int x, int y) {
    return std::to_string(x + g_mapOffX) + "," + std::to_string(y + g_mapOffY);
}
static std::string rectZone(char kind, int id, const char *name, Rect r) {
    std::string d(1, kind);
    d += " " + std::to_string(id) + (name ? std::string(" ") + name : std::string());
    d += " " + mapPt(static_cast<int>(r.x0), static_cast<int>(r.y0)) + " " +
         mapPt(static_cast<int>(r.x1), static_cast<int>(r.y0)) + " " +
         mapPt(static_cast<int>(r.x1), static_cast<int>(r.y1)) + " " +
         mapPt(static_cast<int>(r.x0), static_cast<int>(r.y1)) + "\n";
    return d;
}

static std::string buildMapDoc(const std::vector<Pt>& path) {
    std::string d = "OFM1\nS sim.jsonl\n";
    for (size_t i = 0; i < path.size(); i += 50) {
        d += "P";
        for (size_t j = i; j < path.size() && j < i + 50; j++)
            d += " " + mapPt(static_cast<int>(path[j].x), static_cast<int>(path[j].y));
        d += "\n";
    }
    if (g_world == 0) {
        d += rectZone('R', 1, "Wohnzimmer", {-170, -2000, 3000, 2000});
        d += rectZone('R', 2, "K%C3%BCche", {3000, -2000, 6000, 2000});
    } else {
        d += rectZone('R', 1, "Flur", {-170, -600, 4500, 600});
        d += rectZone('R', 2, "Wohnzimmer", {1000, 600, 4500, 3500});
    }
    Rect n = nogoRect();
    d += rectZone('N', 1, nullptr, n);
    return d;
}

struct Result {
    std::string phase, message;
    bool docked;
    bool testMode; // Robot left in TestMode at the end (must never happen)
    double minNogoDist; // min distance robot centre ↔ no-go rectangle (negative = inside)
    double cover; // fraction of the scored room's cells passed with brush on
    double maxX;
    double simMinutes;
    int collisions;
    double maxErr;
};

static double rectDist(double x, double y, Rect r) {
    double dx = std::max({r.x0 - x, 0.0, x - r.x1}), dy = std::max({r.y0 - y, 0.0, y - r.y1});
    if (dx == 0 && dy == 0) return -std::min({x - r.x0, r.x1 - x, y - r.y0, r.y1 - y});
    return std::hypot(dx, dy);
}

// Likelihood field like the frontend builds from a mapping run. The robot's own
// pose drifts during mapping, so walls come out smeared: every wall sample is
// jittered sideways (g_smearMM) and boxes are rasterized as rings.
static double g_smearMM = 40;
static std::string buildField(const NeatoSerial& s) {
    const int cell = 50;
    double minX = 1e9, minY = 1e9, maxX = -1e9, maxY = -1e9;
    for (auto& w: s.walls) {
        minX = std::min({minX, w.ax, w.bx});
        maxX = std::max({maxX, w.ax, w.bx});
        minY = std::min({minY, w.ay, w.by});
        maxY = std::max({maxY, w.ay, w.by});
    }
    int ox = static_cast<int>(minX) - 500, oy = static_cast<int>(minY) - 500;
    int w = static_cast<int>((maxX - minX + 1000) / cell) + 1, h = static_cast<int>((maxY - minY + 1000) / cell) + 1;
    std::vector<std::pair<int, int>> occ;
    std::vector<uint8_t> mark(static_cast<size_t>(w * h), 0);
    unsigned rs = 777;
    auto jit = [&rs]() { rs = rs * 1103515245u + 12345u; return ((rs >> 8) & 0xFFFF) / 65536.0 - 0.5; };
    auto put = [&](double x, double y) {
        int cx = static_cast<int>((x - ox) / cell);
        int cy = static_cast<int>((y - oy) / cell);
        if (cx >= 0 && cy >= 0 && cx < w && cy < h && !mark[cy * w + cx]) {
            mark[cy * w + cx] = 1;
            occ.push_back({cx, cy});
        }
    };
    for (auto& wl: s.walls) {
        double len = std::hypot(wl.bx - wl.ax, wl.by - wl.ay);
        double nx = -(wl.by - wl.ay) / len, ny = (wl.bx - wl.ax) / len;
        for (double t = 0; t <= len; t += 20) {
            double j = jit() * 2 * g_smearMM;
            put(wl.ax + (wl.bx - wl.ax) * t / len + nx * j, wl.ay + (wl.by - wl.ay) * t / len + ny * j);
        }
    }
    for (auto& b: s.boxes)
        for (double a = 0; a < 2 * M_PI; a += 0.1) put(b.x + b.r * std::cos(a), b.y + b.r * std::sin(a));
    std::string out = "OFL1";
    auto put32 = [&](int32_t v) { for (int i = 0; i < 4; i++) out += static_cast<char>((v >> (8 * i)) & 255); };
    auto put16 = [&](int16_t v) { out += static_cast<char>(v & 255); out += static_cast<char>((v >> 8) & 255); };
    put32(ox + g_mapOffX);
    put32(oy + g_mapOffY);
    put16(cell);
    put16(static_cast<int16_t>(w));
    put16(static_cast<int16_t>(h));
    out += static_cast<char>(s.ldsSign);
    out += '\0';
    put16(0);
    put16(static_cast<int16_t>(s.ldsOffset));
    put16(0);
    for (int cy = 0; cy < h; cy++)
        for (int cx = 0; cx < w; cx++) {
            double best = 1e9;
            for (auto& o: occ) {
                double dx = (o.first - cx) * cell, dy = (o.second - cy) * cell;
                if (std::fabs(dx) > 320 || std::fabs(dy) > 320) continue;
                double d = std::hypot(dx, dy);
                if (d < best) best = d;
            }
            out += static_cast<char>(static_cast<uint8_t>(std::max(0.0, 255.0 * (1.0 - best / 300.0))));
        }
    return out;
}

static int g_traceFailures = 0;
static FILE *g_traj = nullptr;
static bool g_keepMap = false; // Next run on the same stored map (learned dock pose survives)

using TickHook = std::function<void(GuidedCleanManager&, NeatoSerial&)>;

static Result runScenario(const char *name, const char *rooms, std::function<void(NeatoSerial&)> tweak,
                          Rect coverRect = {3300, -1700, 5700, 1700}, TickHook hook = nullptr) {
    std::printf("\n=== %s (rooms=%s, world %d) ===\n", name, rooms, g_world);
    if (g_traj) { std::fclose(g_traj); g_traj = nullptr; }
    if (const char *dir = std::getenv("SIM_TRAJ_DIR")) {
        std::string out = std::string(dir) + "/";
        for (const char *p = name; *p; p++) out += std::isalnum(static_cast<unsigned char>(*p)) ? *p : '-';
        g_traj = std::fopen((out + ".csv").c_str(), "w");
    }
    g_simMillis = 1;
    NeatoSerial serial;
    buildWorld(serial);
    if (const char *seed = std::getenv("SIM_SEED")) serial.rngState += static_cast<unsigned>(std::atoi(seed)) * 7919u;
    std::srand(serial.rngState); // Scan noise: reproducible per scenario
    tweak(serial);
    DataLogger logger;
    FloorMap map(logger);
    ManualCleanManager manual;
    SystemManager sys;
    GuidedCleanManager guided(serial, map, manual, logger, sys);

    if (!g_keepMap) {
        std::string doc = buildMapDoc(recordedPath());
        assert(map.beginUpload(doc.size()));
        size_t half = doc.size() / 2;
        assert(map.writeChunk(reinterpret_cast<const uint8_t *>(doc.data()), half));
        assert(map.writeChunk(reinterpret_cast<const uint8_t *>(doc.data()) + half, doc.size() - half));
        if (!map.endUpload()) {
            std::printf("upload rejected: %s\n", map.getUploadError().c_str());
            std::exit(1);
        }
        SPIFFS.remove("/map/field.bin");
        if (!serial.walls.empty()) {
            std::string fld = buildField(serial);
            assert(map.beginUpload(fld.size(), true));
            assert(map.writeChunk(reinterpret_cast<const uint8_t *>(fld.data()), fld.size()));
            if (!map.endUpload()) {
                std::printf("field rejected: %s\n", map.getUploadError().c_str());
                std::exit(1);
            }
        }
    }
    addUnmapped(serial);

    String err = guided.command("start", rooms);
    if (!err.isEmpty()) {
        std::printf("start refused: %s\n", err.c_str());
        return {"refused", err.str(), serial.docked(), serial.inTestMode, 0, 0, 0, 0, 0, 0};
    }

    std::set<long> brushCells;
    Rect ng = nogoRect();
    double minNogo = 1e9, maxErr = 0, sumErr = 0, maxX = -1e9;
    long errSamples = 0;
    unsigned long lastPrint = 0;
    while (g_simMillis < 3UL * 3600 * 1000) {
        g_simMillis += 10;
        serial.physics(0.010);
        serial.runDue();
        TaskRegistry::tickAll();
        if (hook) hook(guided, serial);
        if (g_traj && g_simMillis % 200 == 1) {
            std::string st = guided.getStatusJson().str();
            auto num = [&](const char *k) { auto p = st.find(std::string("\"") + k + "\":"); return p == std::string::npos ? 0.0 : std::atof(st.c_str() + p + std::strlen(k) + 3); };
            std::fprintf(g_traj, "%lu,%.0f,%.0f,%.3f,%.0f,%.0f,%d,%d\n", g_simMillis, serial.tx, serial.ty, serial.tth,
                         num("x") * 1000 - g_mapOffX, num("y") * 1000 - g_mapOffY, serial.brush ? 1 : 0, serial.stalled ? 1 : 0);
        }
        double nd = rectDist(serial.tx, serial.ty, ng);
        if (nd < 0 && minNogo >= 0)
            std::printf("  !! entered no-go at %.0f,%.0f: %s\n", serial.tx, serial.ty, guided.getStatusJson().c_str());
        minNogo = std::min(minNogo, nd);
        maxX = std::max(maxX, serial.tx);
        if (guided.isActive() && (g_simMillis - 1) % 1000 == 0) {
            std::string st = guided.getStatusJson().str();
            auto num = [&](const char *k) { auto p = st.find(std::string("\"") + k + "\":"); return p == std::string::npos ? 0.0 : std::atof(st.c_str() + p + std::strlen(k) + 3); };
            if (st.find("\"phase\":\"running\"") != std::string::npos || st.find("\"phase\":\"returning\"") != std::string::npos) {
                double e = std::hypot(num("x") * 1000 - g_mapOffX - serial.tx, num("y") * 1000 - g_mapOffY - serial.ty);
                maxErr = std::max(maxErr, e); sumErr += e; errSamples++;
            }
        }
        if (serial.brush && serial.tx >= coverRect.x0 - 200 && serial.tx <= coverRect.x1 + 200) {
            for (int dx = -150; dx <= 150; dx += 50)
                for (int dy = -150; dy <= 150; dy += 50) {
                    long cx = std::lround((serial.tx + dx) / 100), cy = std::lround((serial.ty + dy) / 100);
                    brushCells.insert(cx * 100000 + cy);
                }
        }
        if (g_simMillis - lastPrint >= 120000) {
            lastPrint = g_simMillis;
            std::printf("  t=%3lus %s\n", g_simMillis / 1000, guided.getStatusJson().c_str());
        }
        if (!guided.isActive() && !guided.releasing() && serial.evs.empty())
            break;
    }
    // Scored room cells: inside coverRect, away from the no-go zone and from boxes
    int total = 0, hit = 0;
    for (int x = static_cast<int>(coverRect.x0 / 100); x <= static_cast<int>(coverRect.x1 / 100); x++)
        for (int y = static_cast<int>(coverRect.y0 / 100); y <= static_cast<int>(coverRect.y1 / 100); y++) {
            if (rectDist(x * 100, y * 100, ng) < 200) continue;
            bool nearBox = false;
            for (auto& b: serial.boxes) if (std::hypot(x * 100 - b.x, y * 100 - b.y) < b.r + 250) nearBox = true;
            if (nearBox) continue;
            total++;
            if (brushCells.count(static_cast<long>(x) * 100000 + y)) hit++;
        }
    String st = guided.getStatusJson();
    std::printf("  final %s\n", st.c_str());
    guided.flushTrace();
    {
        std::ifstream tf(fakeFsPath("/map/trace.jsonl"));
        std::string line, last;
        int lines = 0, locs = 0;
        while (std::getline(tf, line)) {
            lines++;
            if (line.find("\"e\":\"loc\"") != std::string::npos) locs++;
            if (line.find("\"e\":\"done\"") != std::string::npos || line.find("\"e\":\"failed\"") != std::string::npos ||
                line.find("\"e\":\"stopped\"") != std::string::npos)
                last = line;
        }
        bool ok = lines >= 2 && !last.empty();
        std::printf("  trace lines=%d loc=%d %s\n", lines, locs, ok ? "ok" : "BAD");
        if (!ok) g_traceFailures++;
        if (const char *dir = std::getenv("GUIDED_SIM_TRACE_DIR")) {
            std::string out = std::string(dir) + "/";
            for (const char *p = name; *p; p++) out += std::isalnum(static_cast<unsigned char>(*p)) ? *p : '-';
            std::ifstream in(fakeFsPath("/map/trace.jsonl"), std::ios::binary);
            std::ofstream(out + ".jsonl", std::ios::binary) << in.rdbuf();
        }
    }
    std::printf("  true pose %.0f,%.0f θ=%.1f° docked=%d testMode=%d collisions(ticks)=%d scans=%d\n", serial.tx,
                serial.ty, serial.tth * 180 / M_PI, serial.docked(), serial.inTestMode, serial.collisions, serial.scans);
    Result r;
    auto fieldOf = [&](const char *k) {
        std::string s = st.str(), key = std::string("\"") + k + "\":\"";
        auto p = s.find(key);
        if (p == std::string::npos) return std::string();
        p += key.size();
        return s.substr(p, s.find('"', p) - p);
    };
    r.phase = fieldOf("phase");
    r.message = fieldOf("message");
    r.docked = serial.docked();
    r.testMode = serial.inTestMode;
    r.minNogoDist = minNogo;
    r.cover = total ? static_cast<double>(hit) / total : 0;
    r.simMinutes = g_simMillis / 60000.0;
    r.collisions = serial.collisions;
    r.maxX = maxX;
    r.maxErr = maxErr;
    std::printf("  → phase=%s msg='%s' docked=%d minNoGoDist=%.0fmm coverage=%.0f%% time=%.1fmin "
                "poseErr avg=%.0f max=%.0fmm\n",
                r.phase.c_str(), r.message.c_str(), r.docked, r.minNogoDist, r.cover * 100, r.simMinutes,
                errSamples ? sumErr / errSamples : 0.0, maxErr);
    return r;
}

int main(int argc, char **argv) {
#ifdef _WIN32
    const char *tmpBase = std::getenv("TEMP"); // Native Windows build (MinGW): no /tmp
#else
    const char *tmpBase = std::getenv("TMPDIR");
#endif
    std::string tmpl = std::string(tmpBase && *tmpBase ? tmpBase : "/tmp") + "/guidedsimXXXXXX";
    if (!mkdtemp(&tmpl[0])) {
        std::perror("mkdtemp");
        return 2;
    }
    g_fsDir = tmpl;
    Serial.enabled = argc > 1 && std::strcmp(argv[1], "-v") == 0;
    g_quietLog = !Serial.enabled;
    const char *only = argc > 2 ? argv[2] : nullptr;
    int failures = 0;
    auto expect = [&](bool ok, const char *what) {
        std::printf("  %s %s\n", ok ? "PASS" : "FAIL", what);
        failures += ok ? 0 : 1;
    };
    auto want = [&](const char *group) { return !only || std::strcmp(only, group) == 0; };
    auto clean = [&](const Result& r, const char *what) {
        expect(r.phase == "done" && r.message.empty() && r.docked && !r.testMode, what);
    };

    if (want("basic")) {
        g_world = 0;
        Result a = runScenario("room 2 with no-go", "2", [](NeatoSerial&) {});
        clean(a, "completes and ends on the dock");
        expect(a.minNogoDist > 0, "robot centre never enters the no-go zone");
        expect(a.cover > 0.8, "room 2 coverage > 80%");

        Result b = runScenario("chair in the way", "2", [](NeatoSerial& s) { s.boxes.push_back({1500, 0, 200}); });
        clean(b, "completes and docks despite an unmapped chair");

        Result c = runScenario("low battery", "1,2", [](NeatoSerial& s) {
            s.fuel = 31;
            s.drainPerMin = 3;
        });
        expect(c.phase == "done" && c.message == "battery low" && c.docked, "returns early on low battery and docks");

        Result g = runScenario("clockwise LIDAR", "2", [](NeatoSerial& s) { s.ldsSign = -1; });
        clean(g, "completes and docks with CW LIDAR");

        Result d = runScenario("battery too low", "2", [](NeatoSerial& s) { s.fuel = 22; });
        expect(d.phase == "failed" && !d.testMode, "refuses to start on low battery");

        Result e = runScenario("not docked", "2", [](NeatoSerial& s) { s.tx = 800; });
        expect(e.phase == "failed" && e.message.find("dock") != std::string::npos, "refuses to start off the dock");

        Result f = runScenario("cliff in room 2", "2", [](NeatoSerial& s) {
            s.cliffMinX = 5800;
            s.cliffMinY = -2000;
            s.cliffMaxY = 2000;
        });
        clean(f, "completes with a cliff present");

        Result m = runScenario("room 1 only", "1", [](NeatoSerial&) {});
        clean(m, "room 1: completes and docks");
        expect(m.maxX < 3000 - 100, "never drives into room 2");

        Result hd = runScenario("heavy drift", "2", [](NeatoSerial& s) {
            s.slipR = 1.03;
            s.wheelBase = 260;
            s.moveNoise = 0.03;
        });
        clean(hd, "completes and docks under heavy drift");
        expect(hd.minNogoDist > 0, "avoids no-go under heavy drift");

        Result k = runScenario("lidar offset + furniture", "1,2", [](NeatoSerial& s) {
            s.ldsOffset = -90;
            s.boxes.push_back({5800, -1850, 120});
            s.boxes.push_back({-20, 1800, 120});
        });
        clean(k, "completes and docks with offset LIDAR");

        g_mapOffX = 290;
        g_mapOffY = 40;
        Result o = runScenario("map origin off the dock", "2", [](NeatoSerial& s) { s.ldsWarmup = 3; });
        clean(o, "docks although the map origin is not the dock (slow LIDAR start)");
        g_mapOffX = 0;
        g_mapOffY = 0;

        Result nl = runScenario("LIDAR never delivers", "2", [](NeatoSerial& s) { s.ldsWarmup = 1000; });
        expect(nl.phase == "failed" && !nl.testMode && nl.docked, "no LIDAR: refuses cleanly, stays on the dock");
    }

    if (want("robust")) {
        g_world = 0;
        // Serial outage mid-run (cable glitch / robot busy): the watchdog must end the run
        Result so = runScenario("serial outage", "2", [](NeatoSerial& s) {
            s.outageFrom = 120000;
            s.outageUntil = 200000;
        });
        expect(so.phase == "failed" && !so.testMode, "outage: run aborted and TestMode left (no hang)");

        // Some answers never come at all
        Result dr = runScenario("lost answers", "2", [](NeatoSerial& s) { s.dropRate = 0.004; });
        expect(!dr.testMode && (dr.phase == "done" || dr.phase == "failed"), "lost answers: never hangs");
        expect(dr.phase == "done" && dr.docked, "lost answers: still completes and docks");

        // "Home" pressed while cleaning
        Result hm = runScenario("home pressed", "2", [](NeatoSerial&) {}, {3300, -1700, 5700, 1700},
                                [](GuidedCleanManager& g, NeatoSerial&) {
                                    if (g_simMillis == 150001) g.command("home", "");
                                });
        expect(hm.phase == "done" && hm.docked && !hm.testMode, "home mid-run: returns and docks");

        // "Stop" pressed mid-move
        Result st = runScenario("stop pressed", "2", [](NeatoSerial&) {}, {3300, -1700, 5700, 1700},
                                [](GuidedCleanManager& g, NeatoSerial&) {
                                    if (g_simMillis == 100001) g.command("stop", "");
                                });
        expect(st.phase == "stopped" && !st.testMode, "stop mid-run: stops and leaves TestMode");

        // Robot shoved 40 cm sideways and 25° (kicked, or slid off a threshold)
        Result kd = runScenario("kidnapped", "2", [](NeatoSerial&) {}, {3300, -1700, 5700, 1700},
                                [](GuidedCleanManager&, NeatoSerial& s) {
                                    if (g_simMillis == 200001) {
                                        // Shove it 40 cm towards the room centre and turn it 25°
                                        double ny = s.ty + (s.ty < 0 ? 400 : -400);
                                        if (!s.collides(s.tx, ny)) {
                                            s.ty = ny;
                                            s.tth += 0.44;
                                        }
                                    }
                                });
        clean(kd, "kidnapped: re-localizes and still docks");
        expect(kd.minNogoDist > 0, "kidnapped: avoids the no-go zone");

        // Very smeared map (old firmware mapping run)
        g_smearMM = 90;
        Result fm = runScenario("smeared map", "2", [](NeatoSerial&) {});
        g_smearMM = 40;
        clean(fm, "smeared map: completes and docks");
    }

    if (want("flat")) {
        g_world = 1;
        Rect flur{100, -400, 4300, 400};
        Rect wohn{1200, 800, 4300, 2900};
        Result f1 = runScenario("Flur", "1", [](NeatoSerial&) {}, flur);
        clean(f1, "Flur: completes and docks");
        expect(f1.cover > 0.75, "Flur coverage > 75%");

        Result w1 = runScenario("Wohnzimmer", "2", [](NeatoSerial&) {}, wohn);
        clean(w1, "Wohnzimmer: completes and docks (door, carpet, chairs)");
        expect(w1.cover > 0.6, "Wohnzimmer coverage > 60%");
        expect(w1.minNogoDist > 0, "Wohnzimmer: avoids the no-go zone");

        // Ten runs in a row on the same map: must keep working without recalibration
        int ok = 0;
        g_keepMap = false;
        for (int i = 0; i < 10; i++) {
            char name[64];
            std::snprintf(name, sizeof(name), "repeat %d", i + 1);
            Result r = runScenario(name, i % 2 ? "2" : "1", [i](NeatoSerial& s) { s.rngState = 1000 + i * 77; },
                                   i % 2 ? wohn : flur);
            g_keepMap = true;
            if (r.phase == "done" && r.docked && !r.testMode) ok++;
        }
        g_keepMap = false;
        std::printf("\n  repeat runs docked: %d/10\n", ok);
        expect(ok == 10, "10 runs in a row all complete and dock");
    }

    expect(g_traceFailures == 0, "every started run leaves a complete trace");
    std::printf("\n%s (%d failure%s)\n", failures ? "FAILED" : "ALL PASSED", failures, failures == 1 ? "" : "s");
    return failures ? 1 : 0;
}

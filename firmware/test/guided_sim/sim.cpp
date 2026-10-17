#include <fstream>
#include <cctype>
// Host-side simulation of GuidedCleanManager against a simple physics model.
// Build + run: ./run.sh   (see README.md in this folder)
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
void DataLogger::logGuided(const String& ev, const std::vector<Field>& extra) {
    std::string line = "  [log] guided " + ev.str();
    for (auto& f: extra) line += " " + f.key.str() + "=" + f.value.str();
    std::printf("%s\n", line.c_str());
}

struct Pt { double x, y; };
static bool inRect(double x, double y, double x0, double y0, double x1, double y1) {
    return x >= x0 && x <= x1 && y >= y0 && y <= y1;
}

// House: left wall (dock) at x=-170, right wall x=6000, y=-2000..2000, inner
// wall at x=3000 with a 1m door at y=-500..500.
static void buildWorld(NeatoSerial& s) {
    s.walls = {{-170, -2000, -170, 2000}, {6000, -2000, 6000, 2000}, {-170, -2000, 6000, -2000},
               {-170, 2000, 6000, 2000},  {3000, -2000, 3000, -500}, {3000, 500, 3000, 2000}};
}

// Recorded house clean: boustrophedon in room A, through the door, room B, back.
static std::vector<Pt> recordedPath() {
    std::vector<Pt> p;
    auto lane = [&](double x0, double x1, double y) {
        double step = 500 * (x1 > x0 ? 1 : -1);
        for (double x = x0; (step > 0 ? x <= x1 : x >= x1); x += step) p.push_back({x, y});
        p.push_back({x1, y});
    };
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

// Map frame = world + offset: the recording's origin is not exactly the dock
// (seen on the real robot: dock ~29 cm off the map origin)
static int g_mapOffX = 0;
static int g_mapOffY = 0;

static std::string mapPt(int x, int y) {
    return std::to_string(x + g_mapOffX) + "," + std::to_string(y + g_mapOffY);
}

static std::string buildMapDoc(const std::vector<Pt>& path) {
    std::string d = "OFM1\nS sim.jsonl\n";
    for (size_t i = 0; i < path.size(); i += 50) {
        d += "P";
        for (size_t j = i; j < path.size() && j < i + 50; j++)
            d += " " + mapPt(static_cast<int>(path[j].x), static_cast<int>(path[j].y));
        d += "\n";
    }
    d += "R 1 Wohnzimmer " + mapPt(-170, -2000) + " " + mapPt(3000, -2000) + " " + mapPt(3000, 2000) + " " +
         mapPt(-170, 2000) + "\n";
    d += "R 2 K%C3%BCche " + mapPt(3000, -2000) + " " + mapPt(6000, -2000) + " " + mapPt(6000, 2000) + " " +
         mapPt(3000, 2000) + "\n";
    d += "N 1 " + mapPt(4200, 600) + " " + mapPt(4900, 600) + " " + mapPt(4900, 1400) + " " + mapPt(4200, 1400) + "\n";
    return d;
}

struct Result {
    std::string phase, message;
    bool docked;
    double minNogoDist; // min distance robot centre ↔ no-go rectangle (negative = inside)
    double coverB; // fraction of room-B cells (outside no-go) passed with brush on
    double maxX; // furthest the robot centre got in +x (room 2 starts at x = 3000)
    double simMinutes;
    int collisions;
};

static double rectDist(double x, double y, double x0, double y0, double x1, double y1) {
    double dx = std::max({x0 - x, 0.0, x - x1}), dy = std::max({y0 - y, 0.0, y - y1});
    if (dx == 0 && dy == 0) return -std::min({x - x0, x1 - x, y - y0, y1 - y});
    return std::hypot(dx, dy);
}

// Likelihood field from the world walls, like the frontend builds from a mapping run
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
    for (auto& wl: s.walls) {
        double len = std::hypot(wl.bx - wl.ax, wl.by - wl.ay);
        for (double t = 0; t <= len; t += 20) {
            int cx = static_cast<int>((wl.ax + (wl.bx - wl.ax) * t / len - ox) / cell);
            int cy = static_cast<int>((wl.ay + (wl.by - wl.ay) * t / len - oy) / cell);
            if (cx >= 0 && cy >= 0 && cx < w && cy < h && !mark[cy * w + cx]) {
                mark[cy * w + cx] = 1;
                occ.push_back({cx, cy});
            }
        }
    }
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
                double d = std::hypot(o.first - cx, o.second - cy) * cell;
                if (d < best) best = d;
            }
            out += static_cast<char>(static_cast<uint8_t>(std::max(0.0, 255.0 * (1.0 - best / 300.0))));
        }
    return out;
}

static bool g_useField = true;

static int g_traceFailures = 0;

static Result runScenario(const char *name, const char *rooms, std::function<void(NeatoSerial&)> tweak) {
    std::printf("\n=== %s (rooms=%s) ===\n", name, rooms);
    g_simMillis = 1;
    NeatoSerial serial;
    buildWorld(serial);
    tweak(serial);
    DataLogger logger;
    FloorMap map(logger);
    ManualCleanManager manual;
    SystemManager sys;
    GuidedCleanManager guided(serial, map, manual, logger, sys);

    std::string doc = buildMapDoc(recordedPath());
    assert(map.beginUpload(doc.size()));
    // Upload in two chunks like the web server would
    size_t half = doc.size() / 2;
    assert(map.writeChunk(reinterpret_cast<const uint8_t *>(doc.data()), half));
    assert(map.writeChunk(reinterpret_cast<const uint8_t *>(doc.data()) + half, doc.size() - half));
    if (!map.endUpload()) {
        std::printf("upload rejected: %s\n", map.getUploadError().c_str());
        std::exit(1);
    }
    SPIFFS.remove("/map/field.bin");
    if (g_useField && !serial.walls.empty()) {
        std::string fld = buildField(serial);
        assert(map.beginUpload(fld.size(), true));
        assert(map.writeChunk(reinterpret_cast<const uint8_t *>(fld.data()), fld.size()));
        if (!map.endUpload()) {
            std::printf("field rejected: %s\n", map.getUploadError().c_str());
            std::exit(1);
        }
    }

    String err = guided.command("start", rooms);
    if (!err.isEmpty()) {
        std::printf("start refused: %s\n", err.c_str());
        return {"refused", err.str(), false, 0, 0, 0, 0, 0};
    }

    std::set<long> brushCells;
    double minNogo = 1e9, maxErr = 0, sumErr = 0, maxX = -1e9;
    long errSamples = 0;
    unsigned long lastPrint = 0;
    while (g_simMillis < 3UL * 3600 * 1000) {
        g_simMillis += 10;
        serial.physics(0.010);
        serial.runDue();
        TaskRegistry::tickAll();
        {
            double nd = rectDist(serial.tx, serial.ty, 4200, 600, 4900, 1400);
            if (nd < 0 && minNogo >= 0)
                std::printf("  !! entered no-go at %.0f,%.0f: %s\n", serial.tx, serial.ty, guided.getStatusJson().c_str());
            minNogo = std::min(minNogo, nd);
        }
        maxX = std::max(maxX, serial.tx);
        if (guided.isActive() && (g_simMillis - 1) % 1000 == 0) {
            std::string st = guided.getStatusJson().str();
            auto num = [&](const char *k) { auto p = st.find(std::string("\"") + k + "\":"); return p == std::string::npos ? 0.0 : std::atof(st.c_str() + p + std::strlen(k) + 3); };
            double e = std::hypot(num("x") * 1000 - g_mapOffX - serial.tx, num("y") * 1000 - g_mapOffY - serial.ty);
            maxErr = std::max(maxErr, e); sumErr += e; errSamples++;
        }
        if (serial.brush && serial.tx > 3000) {
            // Mark 100mm cells within the 300mm cleaning width
            for (int dx = -150; dx <= 150; dx += 50)
                for (int dy = -150; dy <= 150; dy += 50) {
                    long cx = std::lround((serial.tx + dx) / 100), cy = std::lround((serial.ty + dy) / 100);
                    brushCells.insert(cx * 100000 + cy);
                }
        }
        if (g_simMillis - lastPrint >= 60000) {
            lastPrint = g_simMillis;
            std::printf("  t=%3lus %s\n", g_simMillis / 1000, guided.getStatusJson().c_str());
        }
        if (!guided.isActive() && serial.evs.empty())
            break;
    }
    // Room B reachable cells: x 3300..5700, y -1700..1700 minus no-go (+robot radius)
    int total = 0, hit = 0;
    for (int x = 33; x <= 57; x++)
        for (int y = -17; y <= 17; y++) {
            if (rectDist(x * 100, y * 100, 4200, 600, 4900, 1400) < 200) continue;
            total++;
            if (brushCells.count(static_cast<long>(x) * 100000 + y)) hit++;
        }
    String st = guided.getStatusJson();
    std::printf("  final %s\n", st.c_str());
    // Run trace: must exist and end with the result event
    guided.flushTrace();
    {
        std::ifstream tf(fakeFsPath("/map/trace.jsonl"));
        std::string line, last;
        int lines = 0, locs = 0;
        while (std::getline(tf, line)) {
            lines++;
            if (line.find("\"e\":\"loc\"") != std::string::npos) locs++;
            last = line;
        }
        bool ok = lines >= 2 && (last.find("\"e\":\"done\"") != std::string::npos ||
                                  last.find("\"e\":\"failed\"") != std::string::npos ||
                                  last.find("\"e\":\"stopped\"") != std::string::npos);
        std::printf("  trace lines=%d loc=%d %s\n", lines, locs, ok ? "ok" : "BAD");
        if (!ok) g_traceFailures++;
        if (const char *dir = std::getenv("GUIDED_SIM_TRACE_DIR")) {
            std::string out = std::string(dir) + "/";
            for (const char *p = name; *p; p++) out += std::isalnum(static_cast<unsigned char>(*p)) ? *p : '-';
            std::ifstream in(fakeFsPath("/map/trace.jsonl"), std::ios::binary);
            std::ofstream(out + ".jsonl", std::ios::binary) << in.rdbuf();
        }
    }
    std::printf("  true pose %.0f,%.0f θ=%.1f° docked=%d collisions(ticks)=%d dockCmds=%zu scans=%d\n", serial.tx,
                serial.ty, serial.tth * 180 / M_PI, serial.docked(), serial.collisions, serial.cleanCmds.size(),
                serial.scans);
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
    r.minNogoDist = minNogo;
    r.coverB = total ? static_cast<double>(hit) / total : 0;
    r.simMinutes = g_simMillis / 60000.0;
    r.collisions = serial.collisions;
    r.maxX = maxX;
    std::printf("  → phase=%s msg='%s' docked=%d minNoGoDist=%.0fmm roomB coverage=%.0f%% time=%.1fmin "
                "poseErr avg=%.0f max=%.0fmm maxX=%.0f\n",
                r.phase.c_str(), r.message.c_str(), r.docked, r.minNogoDist, r.coverB * 100, r.simMinutes,
                errSamples ? sumErr / errSamples : 0.0, maxErr, maxX);
    return r;
}

int main(int argc, char **argv) {
    char tmpl[] = "/tmp/guidedsimXXXXXX";
    g_fsDir = mkdtemp(tmpl);
    Serial.enabled = argc > 1 && std::strcmp(argv[1], "-v") == 0;
    int failures = 0;
    auto expect = [&](bool ok, const char *what) {
        std::printf("  %s %s\n", ok ? "PASS" : "FAIL", what);
        failures += ok ? 0 : 1;
    };

    // 1. Clean only room 2 (Küche), with drift; must avoid the no-go zone and dock
    Result a = runScenario("room 2 with no-go", "2", [](NeatoSerial&) {});
    expect(a.phase == "done", "run completes");
    expect(a.docked, "robot ends on the dock");
    expect(a.minNogoDist > 0, "robot centre never enters the no-go zone");
    expect(a.coverB > 0.8, "room 2 coverage > 80%");

    // 2. Unrecorded obstacle (chair) on the transit line through room 1
    Result b = runScenario("chair in the way", "2", [](NeatoSerial& s) { s.boxes.push_back({1500, 0, 200}); });
    expect(b.phase == "done", "run completes despite obstacle");
    expect(b.docked, "robot docks after detour");

    // 3. Battery runs low mid-run: return home early
    Result c = runScenario("low battery", "1,2", [](NeatoSerial& s) {
        s.fuel = 31;
        s.drainPerMin = 3;
    });
    expect(c.phase == "done" && c.message == "battery low", "returns early on low battery");
    expect(c.docked, "docks after low-battery return");

    // 7. LDS angles reported clockwise: compass must auto-calibrate
    Result g = runScenario("clockwise LIDAR", "2", [](NeatoSerial& s) { s.ldsSign = -1; });
    expect(g.phase == "done" && g.docked, "completes and docks with CW LIDAR");
    expect(g.minNogoDist > 0, "avoids no-go with CW LIDAR");

    // 8. No LIDAR structure (compass unavailable): odometry only, smaller drift
    Result h = runScenario("no compass, low drift", "2", [](NeatoSerial& s) {
        s.slipR = 1.001;
        s.walls.clear(); // nothing to see (and nothing to hit) — pure odometry check
    });
    expect(h.phase == "done", "odometry-only run completes");

    // 4. Battery too low to start
    Result d = runScenario("battery too low", "2", [](NeatoSerial& s) { s.fuel = 22; });
    expect(d.phase == "failed", "refuses to start on low battery");

    // 5. Not on the dock
    Result e = runScenario("not docked", "2", [](NeatoSerial& s) { s.tx = 800; });
    expect(e.phase == "failed" && e.message.find("dock") != std::string::npos, "refuses to start off the dock");

    // 6. Cliff (stairs) at the far end of room 2 that the recording got close to
    Result f = runScenario("cliff in room 2", "2", [](NeatoSerial& s) {
        s.cliffMinX = 5800;
        s.cliffMinY = -2000;
        s.cliffMaxY = 2000;
    });
    expect(f.phase == "done", "completes with cliff present");

    // 10. Only room 1: the recording detours through room 2 before finishing
    //     room 1 — the robot must not follow that detour
    Result m = runScenario("room 1 only (recording detours via room 2)", "1", [](NeatoSerial&) {});
    expect(m.phase == "done" && m.docked, "completes and docks");
    expect(m.maxX < 3000 - 100, "never drives into room 2");

    // 11. Heavy drift (2% wheel slip, 4% wheelbase error): map localization must hold
    Result hd = runScenario("heavy drift with map", "2", [](NeatoSerial& s) {
        s.slipR = 1.02;
        s.wheelBase = 258;
    });
    expect(hd.phase == "done" && hd.docked, "completes and docks under heavy drift");
    expect(hd.minNogoDist > 0, "avoids no-go under heavy drift");

    // 12. Same scenario without a map: compass fallback still works for the basics
    g_useField = false;
    Result nf = runScenario("no map field (compass fallback)", "2", [](NeatoSerial&) {});
    expect(nf.phase == "done", "compass fallback completes");
    g_useField = true;

    // 9. LIDAR mounted behind the centre + furniture the recording never touched
    Result k = runScenario("lidar offset + furniture", "1,2", [](NeatoSerial& s) {
        s.ldsOffset = -90;
        s.boxes.push_back({5800, -1850, 120}); // plant in a corner
        s.boxes.push_back({-20, 1800, 120}); // shelf next to the dock wall
    });
    expect(k.phase == "done" && k.docked, "completes and docks with offset LIDAR");
    expect(k.minNogoDist > 0, "avoids no-go with offset LIDAR");

    expect(g_traceFailures == 0, "every started run leaves a complete trace");

    // The recording's origin is 29 cm in front of the dock: docking must use the
    // dock pose measured by the first scan match, not the map origin
    g_mapOffX = 290;
    g_mapOffY = 40;
    Result o = runScenario("map origin off the dock", "2", [](NeatoSerial&) {});
    g_mapOffX = 0;
    g_mapOffY = 0;
    expect(o.phase == "done" && o.docked, "docks although the map origin is not the dock");
    expect(o.minNogoDist > 0, "avoids no-go with an offset map origin");

    std::printf("\n%s (%d failure%s)\n", failures ? "FAILED" : "ALL PASSED", failures, failures == 1 ? "" : "s");
    return failures ? 1 : 0;
}

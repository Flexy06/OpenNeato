// Host-side physics stand-in for NeatoSerial: differential-drive robot in a 2D
// world with walls, cliffs and a dock. Serial latency is modelled by deferring
// every callback by a fixed delay.
#ifndef NEATO_SERIAL_H
#define NEATO_SERIAL_H
#include <Arduino.h>
#include <functional>
#include <vector>
#include "config.h"
#include "loop_task.h"
#include "neato_commands.h"

enum CommandPriority : uint8_t { PRIORITY_CRITICAL = 1, PRIORITY_HIGH = 2, PRIORITY_MEDIUM = 3, PRIORITY_NORMAL = 4 };

struct SimSeg { double ax, ay, bx, by; };
struct SimCircle { double x, y, r; };

class NeatoSerial {
public:
    // -- World ------------------------------------------------------------------
    std::vector<SimSeg> walls;
    std::vector<SimCircle> boxes; // Unrecorded obstacles (chairs, ...)
    std::vector<SimSeg> cliffEdges; // Cliff region = x > cliffX inside these bounds (simple model below)
    double cliffMinX = 1e9, cliffMinY = 0, cliffMaxY = 0; // Region x >= cliffMinX, y in [minY,maxY] is a drop
    double radius = 165.0;
    double wheelBase = 248.0;
    double slipL = 1.0, slipR = 1.004; // actual travel per encoder mm
    double fuel = 90;
    double drainPerMin = 0.5; // % per minute while in TestMode
    int ldsSign = 1; // +1: LDS angles CCW (ROS convention), -1: CW
    // Realism knobs
    double moveNoise = 0.01; // Per-move random wheel slip (1σ, fraction)
    double failRate = 0.02; // Share of answers that come back as failed (timeouts)
    double dropRate = 0.0; // Share of answers that never come (exercises the watchdogs)
    unsigned long outageFrom = 0, outageUntil = 0; // No answers at all in this window
    double spuriousRate = 0.04; // Share of LIDAR points hitting something unmapped (legs, cables)
    unsigned long scanLatency = 700; // GetLDSScan round trip at 115200 baud
    double carpetX0 = 1e9, carpetY0 = 0, carpetX1 = 0, carpetY1 = 0; // Extra slip + heading kicks here
    unsigned rngState = 12345;
    double rnd() { rngState = rngState * 1103515245u + 12345u; return ((rngState >> 8) & 0xFFFF) / 65536.0; }
    double gauss() { return (rnd() + rnd() + rnd() + rnd() - 2.0) * 1.7320508; }
    double moveSlipL = 1.0, moveSlipR = 1.0;
    double ldsOffset = 0; // LIDAR turret position along the heading (mm, negative = behind centre)
    bool ldsOn = false;
    int scans = 0;

    // -- True state -----------------------------------------------------------------
    double tx = 0, ty = 0, tth = 0; // mm, rad
    double encL = 1000, encR = 2000; // arbitrary non-zero counters
    double remL = 0, remR = 0, spd = 0;
    double acc = 0, vel = 0; // Ramp (mm/s²) and current speed of the faster wheel
    double odoKm = 0; // Distance driven (for stops per metre)
    int stops = 0; // Times the robot came to a standstill from cruising (TestMode only)
    bool cruising = false;
    // True poses of the last 0.5 s (10 ms physics steps): the LIDAR scan is taken
    // over the turret turn before the request, not at one instant
    struct PoseAt { unsigned long t; double x, y, th; };
    std::vector<PoseAt> poseHist;
    unsigned long sentAt = 0; // Like NeatoSerial::lastSentAt(): set before each answer callback
    unsigned long lastSentAt() const { return sentAt; }
    bool inTestMode = false, guided = false;
    bool bump = false, stalled = false;
    int collisions = 0;
    bool brush = false;
    std::vector<std::string> cleanCmds;

    // -- Event queue -------------------------------------------------------------------
    struct Ev { unsigned long at; std::function<void()> fn; };
    std::vector<Ev> evs;
    unsigned long lat = 60;
    bool answerLost() {
        unsigned long now = millis();
        if (now >= outageFrom && now < outageUntil) return true;
        return dropRate > 0 && rnd() < dropRate;
    }
    void laterIn(unsigned long ms, std::function<void()> fn) {
        if (answerLost()) return;
        evs.push_back({millis() + ms, std::move(fn)});
    }
    void later(std::function<void()> fn) { laterIn(lat, std::move(fn)); }
    // Answer callback for a command sent now: lastSentAt() = the send time while it runs
    void answer(unsigned long ms, std::function<void()> fn) {
        unsigned long t0 = millis();
        laterIn(ms, [this, t0, fn] { sentAt = t0; fn(); });
    }
    bool failNow() { return failRate > 0 && rnd() < failRate; }
    void runDue() {
        for (size_t i = 0; i < evs.size();) {
            if (evs[i].at <= millis()) { auto fn = evs[i].fn; evs.erase(evs.begin() + i); fn(); i = 0; }
            else i++;
        }
    }
    int queueDepth() const { return static_cast<int>(evs.size()); }
    bool hasSKey() const { return true; }

    // -- Physics -----------------------------------------------------------------------
    static double dseg(double px, double py, const SimSeg& s) {
        double dx = s.bx - s.ax, dy = s.by - s.ay, l2 = dx * dx + dy * dy;
        double t = l2 > 0 ? ((px - s.ax) * dx + (py - s.ay) * dy) / l2 : 0;
        t = t < 0 ? 0 : (t > 1 ? 1 : t);
        double cx = s.ax + t * dx - px, cy = s.ay + t * dy - py;
        return std::sqrt(cx * cx + cy * cy);
    }
    double clearance(double x, double y) const {
        double c = 1e9;
        for (auto& w: walls) c = std::min(c, dseg(x, y, w));
        for (auto& b: boxes) c = std::min(c, std::hypot(x - b.x, y - b.y) - b.r);
        return c;
    }
    bool collides(double x, double y) const {
        for (auto& w: walls) if (dseg(x, y, w) < radius) return true;
        for (auto& b: boxes) if (std::hypot(x - b.x, y - b.y) < radius + b.r) return true;
        return false;
    }
    bool overCliff() const {
        // Front drop sensors ~130mm ahead of centre
        double fx = tx + 130 * std::cos(tth), fy = ty + 130 * std::sin(tth);
        return fx >= cliffMinX && fy >= cliffMinY && fy <= cliffMaxY;
    }
    void physics(double dtS) {
        poseHist.push_back({millis(), tx, ty, tth});
        if (poseHist.size() > 50) poseHist.erase(poseHist.begin());
        if (inTestMode) fuel -= drainPerMin * dtS / 60.0;
        if (!inTestMode || (remL == 0 && remR == 0)) {
            stalled = false;
            if (vel > 0) vel = 0;
            if (cruising) { cruising = false; stops++; }
            return;
        }
        double big = std::max(std::fabs(remL), std::fabs(remR));
        if (big < 0.5) {
            remL = remR = 0;
            vel = 0;
            if (cruising) { cruising = false; stops++; }
            return;
        }
        // Trapezoid profile: ramp up at acc, brake so the distance ends at zero speed
        double a = acc > 0 ? acc : spd;
        double brakeV = std::sqrt(2 * a * big);
        double want = std::min(spd, brakeV);
        if (vel < want) vel = std::min(want, vel + a * dtS);
        else vel = want;
        if (vel > 60) cruising = true;
        double step = std::min(big, std::max(vel, 5.0) * dtS);
        double f = step / big;
        double ml = remL * f, mr = remR * f;
        bool onCarpet = tx >= carpetX0 && tx <= carpetX1 && ty >= carpetY0 && ty <= carpetY1;
        double carpet = onCarpet ? 0.97 : 1.0;
        double al = ml * slipL * moveSlipL * carpet, ar = mr * slipR * moveSlipR * carpet; // actual travel
        double dth = (ar - al) / wheelBase, ds = (al + ar) / 2;
        double nx = tx + ds * std::cos(tth + dth / 2), ny = ty + ds * std::sin(tth + dth / 2);
        bool forward = ds > 0.01;
        // Blocked when moving into something; when already touching, only moves
        // that do not go deeper are possible (never through a wall)
        if (collides(nx, ny) && (!collides(tx, ty) || clearance(nx, ny) < clearance(tx, ty))) {
            if (forward) bump = true;
            if (std::getenv("SIM_DEBUG") && !stalled) {
                std::string what = "wall";
                for (auto& b: boxes) if (std::hypot(nx - b.x, ny - b.y) < radius + b.r) what = "box " + std::to_string((int)b.x) + "," + std::to_string((int)b.y);
                std::printf("  [sim] contact at %.0f,%.0f th=%.0f with %s (t=%lus)\n", tx, ty, tth * 57.3, what.c_str(), millis() / 1000);
            }
            stalled = true;
            collisions++;
            vel = 0;
            return; // wheels blocked — encoders do not advance
        }
        stalled = false;
        bool nowCarpet = nx >= carpetX0 && nx <= carpetX1 && ny >= carpetY0 && ny <= carpetY1;
        if (nowCarpet != onCarpet) tth += gauss() * 0.035; // Carpet edge knocks the heading (~2°)
        tx = nx; ty = ny; tth += dth;
        odoKm += std::fabs(ds) / 1e6;
        encL += ml; encR += mr; remL -= ml; remR -= mr;
    }
    // On the contacts: centre within a few cm of the docked pose, facing out
    bool docked() const {
        return tx > -15 && tx < 35 && std::fabs(ty) < 22 && std::fabs(std::remainder(tth, 2 * M_PI)) < 6 * M_PI / 180;
    }

    // -- API used by guided clean ----------------------------------------------------------
    void setGuidedCleanActive(bool a) { guided = a; }
    void getState(std::function<void(bool, const RobotState&)> cb) {
        later([this, cb] { RobotState s; s.uiState = inTestMode ? "UIMGR_STATE_TESTMODE" : "UIMGR_STATE_IDLE"; s.robotState = inTestMode ? "ST_C_TestMode" : "ST_C_Idle"; cb(true, s); });
    }
    void getCharger(std::function<void(bool, const ChargerData&)> cb) {
        later([this, cb] { ChargerData c; c.fuelPercent = static_cast<int>(fuel); c.extPwrPresent = docked(); cb(true, c); });
    }
    bool testMode(bool on, std::function<void(bool)> cb = nullptr) {
        later([this, on, cb] { inTestMode = on; remL = remR = 0; if (cb) cb(true); }); return true;
    }
    void getMotors(std::function<void(bool, const MotorData&)> cb, CommandPriority = PRIORITY_NORMAL) {
        if (failNow()) { answer(lat, [cb] { cb(false, MotorData()); }); return; }
        // The robot reads its encoders when the command arrives, the answer comes later
        MotorData m;
        m.leftWheelPositionMM = static_cast<int>(std::lround(encL));
        m.rightWheelPositionMM = static_cast<int>(std::lround(encR));
        m.leftWheelLoad = m.rightWheelLoad = stalled ? 90 : 20;
        answer(lat, [cb, m] { cb(true, m); });
    }
    void getDigitalSensors(std::function<void(bool, const DigitalSensorData&)> cb, CommandPriority = PRIORITY_NORMAL) {
        if (failNow()) { later([cb] { cb(false, DigitalSensorData()); }); return; }
        later([this, cb] { DigitalSensorData d; d.lFrontBit = bump; d.rFrontBit = bump; d.dcJackIn = docked(); bump = false; cb(true, d); });
    }
    void getBatteryAnalog(std::function<void(bool, const BatteryAnalogData&)> cb, CommandPriority = PRIORITY_NORMAL) {
        later([this, cb] { BatteryAnalogData a; a.dropLeftMM = a.dropRightMM = overCliff() ? 90 : 19; cb(true, a); });
    }
    bool setMotorWheels(int l, int r, int s, std::function<void(bool)> cb = nullptr, int accel = 0) {
        if (failNow() && (l != 0 || r != 0)) { later([cb] { if (cb) cb(false); }); return true; }
        later([this, l, r, s, cb, accel] {
            if (l == 0 && r == 0) vel = 0; // Wheel disable: immediate stop
            // A new command replaces the running one; the wheels keep their speed
            remL = l; remR = r; spd = s; acc = accel;
            moveSlipL = 1.0 + gauss() * moveNoise;
            moveSlipR = 1.0 + gauss() * moveNoise;
            if (cb) cb(true);
        });
        return true;
    }
    bool setMotorBrush(int rpm, std::function<void(bool)> cb = nullptr) { later([this, rpm, cb] { brush = rpm > 0; if (cb) cb(true); }); return true; }
    bool setMotorVacuum(bool, int = 80, std::function<void(bool)> cb = nullptr) { later([cb] { if (cb) cb(true); }); return true; }
    bool setMotorSideBrush(bool, int = 5000, std::function<void(bool)> cb = nullptr) { later([cb] { if (cb) cb(true); }); return true; }
    bool setLdsRotation(bool on, std::function<void(bool)> cb = nullptr) { later([this, on, cb] { ldsOn = on; if (cb) cb(true); }); return true; }
    // Ray-cast scan from the robot centre against walls + boxes, ±8 mm noise
    double ray(double ang) const {
        double best = 0, dx = std::cos(ang), dy = std::sin(ang);
        double tx = this->tx + ldsOffset * std::cos(tth), ty = this->ty + ldsOffset * std::sin(tth);
        for (auto& w: walls) {
            double ex = w.bx - w.ax, ey = w.by - w.ay, den = dx * ey - dy * ex;
            if (std::fabs(den) < 1e-9) continue;
            double t = ((w.ax - tx) * ey - (w.ay - ty) * ex) / den, u = ((w.ax - tx) * dy - (w.ay - ty) * dx) / den;
            if (t > 0 && u >= 0 && u <= 1 && (best == 0 || t < best)) best = t;
        }
        for (auto& b: boxes) {
            double fx = tx - b.x, fy = ty - b.y, bb = fx * dx + fy * dy, cc = fx * fx + fy * fy - b.r * b.r, disc = bb * bb - cc;
            if (disc < 0) continue;
            double t = -bb - std::sqrt(disc);
            if (t > 0 && (best == 0 || t < best)) best = t;
        }
        return best;
    }
    int ldsWarmup = 0; // First N scans come back empty (LIDAR still spinning up)
    void getLdsScanFresh(std::function<void(bool, const LdsScanData&)> cb) { getLdsScan(cb); }
    // Pose at time t from the physics history (the turret turn before a request)
    PoseAt poseAt(unsigned long t) const {
        for (size_t k = poseHist.size(); k-- > 0;)
            if (poseHist[k].t <= t) return poseHist[k];
        return poseHist.empty() ? PoseAt{t, tx, ty, tth} : poseHist.front();
    }
    void getLdsScan(std::function<void(bool, const LdsScanData&)> cb) {
        if (failNow()) { answer(scanLatency, [cb] { auto* d = new LdsScanData(); cb(false, *d); delete d; }); return; }
        // The last full turret turn (0.2 s) before the request; index 0 first
        auto* d = new LdsScanData();
        unsigned long t0 = millis();
        {
            for (int i = 0; i < 360; i++) {
                PoseAt at = poseAt(t0 - 200 + static_cast<unsigned long>(i) * 200 / 360);
                double sx = this->tx, sy = this->ty, sth = this->tth;
                this->tx = at.x; this->ty = at.y; this->tth = at.th;
                double ang = tth + ldsSign * i * M_PI / 180.0;
                double r = ldsOn && scans >= ldsWarmup ? ray(ang) : 0;
                this->tx = sx; this->ty = sy; this->tth = sth;
                if (r > 0 && rnd() < spuriousRate) r = 200 + rnd() * r; // Something unmapped in between
                d->points[i].angleDeg = i;
                d->points[i].distMM = r > 0 && r < 6000 ? static_cast<int>(r + (std::rand() % 17) - 8) : 0;
                d->points[i].errorCode = d->points[i].distMM > 0 ? 0 : 1;
            }
            scans++;
        }
        answer(scanLatency, [cb, d] { cb(true, *d); delete d; });
    }
    bool clean(const String& a, std::function<void(bool)> cb = nullptr) { cleanCmds.push_back(a.str()); if (cb) cb(true); return true; }
};
#endif

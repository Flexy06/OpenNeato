#ifndef GUIDED_CLEAN_MANAGER_H
#define GUIDED_CLEAN_MANAGER_H

#include <Arduino.h>
#include <functional>
#include <memory>
#include <vector>
#include "config.h"
#include "data_logger.h"
#include "floor_map.h"
#include "loop_task.h"
#include "neato_serial.h"
#include "particle_filter.h"

class ManualCleanManager;
class SystemManager;

// Guided clean: room cleaning and no-go zones on top of a pinned floor map.
//
// The robot firmware exposes no zone or map API over serial, so the ESP32
// drives the robot itself (TestMode + SetMotor). Design (v2):
//
//   1. Map      — the LIDAR likelihood field of a pinned mapping run is required.
//                 Lanes over the free floor of each selected room, transit planned
//                 on a coarse grid (reachable, clear of inflated no-go zones).
//   2. Start    — like the robot's own undocking: on the dock, wait until the
//                 LIDAR delivers full scans, keep that scan as the dock reference,
//                 drive 30 cm off the dock and turn once around in four stops,
//                 scanning at each. A particle filter must converge before any
//                 cleaning starts; the dock pose in the map is learned from this.
//   3. Drive    — continuous: lanes and transit lines are driven in one go, the
//                 next command is sent before the current one ends, and only
//                 sharp corners, obstacles and bumps stop the robot (gentle
//                 ramps). LIDAR scans are taken on the move and paired with the
//                 odometry of the moment they were taken; each one updates the
//                 particle filter and re-steers the drive with a gentle arc. The
//                 filter's spread says when the robot is unsure — then it stops
//                 and re-localizes by turning on the spot instead of driving on.
//   4. Dock     — drive to a staging point in front of the dock, then align by
//                 matching scans against the dock reference scan (ICP, cm-level:
//                 same place, same view) and back straight onto the contacts.
//                 Verified by the charger; retried; the robot's own docking is
//                 only the very last resort.
//   5. Safety   — every wait (serial answer, motion, scan) has a deadline and a
//                 run without progress is aborted: motors off, TestMode left, so
//                 the robot never hangs in TestMode. "Home" and "Stop" always act.
//
// Everything runs from tick() via serial callbacks — never blocks loop().
class GuidedCleanManager : public LoopTask {
public:
    GuidedCleanManager(NeatoSerial& serial, FloorMap& floorMap, ManualCleanManager& manual, DataLogger& logger,
                       SystemManager& sys);

    // Synchronous command entry point for POST /api/guided.
    //   action "start": rooms = comma-separated room ids ("" = all rooms)
    //   action "home":  stop cleaning and drive back to the dock
    //   action "stop":  stop in place and leave TestMode
    // Returns an empty string if accepted, otherwise a human-readable reason.
    String command(const String& action, const String& rooms);

    bool isActive() const;

    String getStatusJson() const;

    // Trace of the last run (see traceBegin); flushes pending lines first
    bool hasTrace() const;
    // Still trying to leave TestMode after a run (serial outage while finishing)
    bool releasing() const { return releaseSince != 0; }
    void flushTrace() { traceFlush(); }

private:
    enum class Phase : uint8_t {
        Idle, // Never run since boot
        Starting, // Pre-flight checks, TestMode, LIDAR spin-up
        Localizing, // Dock reference scan, undocking, turn-around scans until the filter converges
        Running, // Cleaning lanes / transit between them
        Returning, // Driving back to the dock staging point
        Docking, // Aligning on the dock reference scan, backing onto the contacts
        Finishing, // Leaving TestMode
        Done,
        Stopped,
        Failed,
    };

    // The one thing the controller is waiting for (each with a deadline)
    enum class Wait : uint8_t { None, Cmd, Motion, Delay, Scan };
    enum class MoveKind : uint8_t { None, Turn, Drive, Reverse, Backoff, DockBack };
    enum class MoveResult : uint8_t { Ok, Blocked, Cliff, Contact };

    struct Waypoint {
        int16_t x = 0;
        int16_t y = 0;
        bool clean = false;
        uint8_t room = 0; // Room id (0 = transit)
    };

    // Coverage bitmap of floor the robot crossed during the recording
    struct CoverGrid {
        int originX = 0; // mm, world coordinate of cell (0,0)
        int originY = 0;
        int w = 0;
        int h = 0;
        std::vector<uint8_t> bits;

        void init(int minX, int minY, int maxX, int maxY);
        void stamp(float x, float y, float radius);
        bool covered(float x, float y) const;
        void clear();
    };

    void tick() override;

    NeatoSerial& serial;
    FloorMap& floorMap;
    ManualCleanManager& manual;
    DataLogger& dataLogger;
    SystemManager& sysMgr;

    Phase phase = Phase::Idle;
    String lastError; // Failure reason (Failed) or return reason (Returning/Done)
    String roomsLabel; // Selected room ids as requested, for status
    unsigned long startedMs = 0;
    time_t startedEpoch = 0;
    unsigned long finishedMs = 0;
    uint32_t runGen = 0; // Incremented per run/finish; stale serial callbacks compare against it

    // Coarse planning grid: free = floor clear of walls in the LIDAR map (or
    // covered during the recording), minus inflated no-go zones, reachable from
    // the dock. Obstacles found during the run are checked on top.
    struct PlanGrid {
        int originX = 0;
        int originY = 0;
        int cell = GUIDED_PLAN_CELL_MM;
        int w = 0;
        int h = 0;
        std::vector<uint8_t> free;

        bool inside(int cx, int cy) const { return cx >= 0 && cy >= 0 && cx < w && cy < h; }
        int index(int cx, int cy) const { return cy * w + cx; }
        void toCell(float x, float y, int& cx, int& cy) const;
        float centerX(int cx) const { return static_cast<float>(originX + cx * cell + cell / 2); }
        float centerY(int cy) const { return static_cast<float>(originY + cy * cell + cell / 2); }
        void clear();
    };

    // A cleaning run: consecutive lane points inside the selected rooms
    struct Run {
        uint16_t first = 0; // Index into cleanPts
        uint16_t last = 0;
        bool done = false;
    };

    // -- Plan -----------------------------------------------------------------
    // `route` is the current leg: either a planned transit path (to the start of
    // the next run, or home) or the points of one cleaning run.
    std::vector<Waypoint> route;
    std::vector<Waypoint> cleanPts;
    std::vector<Run> runs;
    int curRun = -1;
    bool curReversed = false;
    bool legTransit = false;
    float legGoalX = 0.0f;
    float legGoalY = 0.0f;
    int legReplans = 0;
    bool fallbackHome = false; // Returning along breadcrumbs because planning failed
    std::vector<MapPoint> crumbs; // Sparse trail of reached positions (fallback way home)
    PlanGrid grid;
    std::vector<MapZone> nogos;
    CoverGrid cover;
    size_t routeIdx = 0; // Last reached (or skipped) waypoint
    size_t targetIdx = 0; // Waypoint currently driven to
    int cleanTotal = 0;
    int cleanDone = 0;
    // Per room: lane points planned / reached (a room never entered is reported)
    struct RoomProgress {
        uint8_t id;
        uint16_t total;
        uint16_t reached;
    };
    std::vector<RoomProgress> roomProg;
    String unreachedRooms() const;
    int obstacleStreak = 0;
    int obstacleTotal = 0;

    String buildPlan(const FloorMapData& map, const String& rooms);
    bool floorFree(float x, float y) const;
    void addRun(const std::vector<Waypoint>& pts);
    void buildLanes(const std::vector<const MapZone *>& selected);
    void planRecorded(const FloorMapData& map, const std::vector<const MapZone *>& selected);
    void freePlan();

    // -- Pose (mm / radians, map frame) ------------------------------------------
    // pose = particle filter estimate at the last scan + wheel odometry since.
    float poseX = 0.0f;
    float poseY = 0.0f;
    float poseTheta = 0.0f;
    // Filter estimate at the last update's scan, and the odometry at that moment
    // (odometry since = motion model input; scans on the move are older than "now")
    float fixX = 0.0f;
    float fixY = 0.0f;
    float fixT = 0.0f;
    float fixOdoX = 0.0f;
    float fixOdoY = 0.0f;
    float fixOdoT = 0.0f;
    // Pure odometry since leaving the dock, in the dock frame (learns the dock pose)
    float odoX = 0.0f;
    float odoY = 0.0f;
    float odoT = 0.0f;
    bool encValid = false;
    int encL = 0;
    int encR = 0;
    void integrateEncoders(int newL, int newR);
    // Map pose (x, y, t) held at the odometry pose (ox, oy, ot): pose now follows
    void setFix(float x, float y, float t, float ox, float oy, float ot);
    void resetPose(float x, float y, float t) { setFix(x, y, t, odoX, odoY, odoT); }
    // Odometry over time (time = when the encoder query went out) to place scans
    struct OdoSample {
        unsigned long t = 0;
        float x = 0.0f;
        float y = 0.0f;
        float th = 0.0f;
    };
    static const int ODO_HIST = 24;
    OdoSample odoHist[ODO_HIST];
    int odoHistN = 0;
    int odoHistPos = 0;
    void pushOdo(unsigned long t);
    void odoAt(unsigned long t, float& x, float& y, float& th) const;
    unsigned long odoNewest() const;
    float distSinceFix = 0.0f; // Odometry since the last fix attempt (mm / rad)
    float turnSinceFix = 0.0f;
    float distSinceGood = 0.0f; // ... since the last good fix

    // -- Waits + watchdog -----------------------------------------------------------
    Wait wait = Wait::None;
    unsigned long waitDeadline = 0;
    const char *waitWhat = "";
    uint32_t waitGen = 0; // Bumped on every wait start/end: late answers are dropped
    std::function<void()> afterDelay;
    std::function<void()> cmdFallback; // How to carry on if a Cmd wait times out (else: abort / re-decide)
    unsigned long lastProgressMs = 0;
    int timeoutCount = 0;
    unsigned long finishingSince = 0;
    unsigned long releaseSince = 0;
    Ticker releaseTicker;
    void beginWait(Wait w, unsigned long ms, const char *what);
    void endWait();
    void progress() { lastProgressMs = millis(); }
    void delayThen(unsigned long ms, const char *what, std::function<void()> fn);
    void settleThen(std::function<void()> fn);
    void onWaitTimeout();
    void abortRun(const String& reason);

    // -- Motion executor ------------------------------------------------------------
    MoveKind move = MoveKind::None;
    int moveCmdL = 0;
    int moveCmdR = 0;
    int moveStartL = 0;
    int moveStartR = 0;
    unsigned long moveStartMs = 0;
    int moveStillPolls = 0;
    int moveStallPolls = 0;
    bool moveEnding = false;
    bool navMove = false; // Navigation drive: may be chained, re-steered or softly stopped
    int moveSpeed = 0;
    std::function<void(MoveResult)> onMoveDone;
    const char *blockReason = "";
    void startMove(MoveKind kind, int leftMM, int rightMM, int speed, std::function<void(MoveResult)> done);
    void sendMove(int leftMM, int rightMM, int speed, bool retry);
    void turnBy(float rad, std::function<void(MoveResult)> done);
    // Replace the running drive command without stopping (the robot keeps rolling)
    void redirect(int leftMM, int rightMM);
    // Ramp down to a stop within the braking distance instead of cutting the wheels
    void softStop();
    void endMove(MoveResult r);
    // Bump/stall/cliff: stop, back off, then report Blocked/Cliff to the move's owner
    void onContact(const char *reason, bool cliff);

    // -- Sensor polling ------------------------------------------------------------
    Ticker pollTicker;
    Ticker batteryTicker;
    uint8_t pollSlot = 0;
    bool pollPending = false;
    unsigned long pollSentMs = 0;
    void pollSensors();
    void onMotors(const MotorData& m);
    void onDigital(const DigitalSensorData& d);
    void onAnalog(const BatteryAnalogData& a);

    // -- Obstacle memory (bumps, stalls, cliffs seen during this run) ---------------
    // Bump memory: what the LIDAR cannot see (low obstacles, glass) — forgotten after
    // a while, since the robot's own pose error can make it bump a mapped wall
    struct Obstacle {
        int16_t x = 0;
        int16_t y = 0;
        int16_t r = 0;
        unsigned long atMs = 0;
        bool live() const { return millis() - atMs < GUIDED_BUMP_MEMORY_MS; }
    };
    std::vector<Obstacle> obstacles;
    // LIDAR obstacle layer: returns at stops that the map does not explain (chairs,
    // bags, a moved table) — avoided before the bumper finds them. Hit counts per
    // cell (rays passing through clear them again) + an inflated "keep out" count.
    struct DynGrid {
        int originX = 0;
        int originY = 0;
        int cell = 100;
        int w = 0;
        int h = 0;
        std::vector<uint8_t> hits;
        std::vector<uint8_t> infl;
        bool inside(int cx, int cy) const { return cx >= 0 && cy >= 0 && cx < w && cy < h; }
        bool blocked(float x, float y) const;
        void clear();
    };
    DynGrid dyn;
    void initDyn();
    void markScan(float px, float py, float pt);
    void dynSet(int cx, int cy, bool occupied);
    void rememberObstacle(float aheadMM, int radius, bool behind);
    bool nearNogo(float x, float y) const;
    bool nearObstacle(float x, float y) const;
    bool segmentClear(float ax, float ay, float bx, float by, bool requireCover) const;

    // -- Localization ---------------------------------------------------------------
    LocField field;
    bool fieldLoadFailed = false; // A field exists but could not be loaded (memory)
    ParticleFilter pf{GUIDED_PF_PARTICLES};
    ScanBeams beams;
    bool ldsOn = false;
    bool bootChecked = false;
    int locFixes = 0;
    int locRejected = 0;
    float locScore = 0.0f; // Quality of the last fix (mean field value under the scan, 0..1)
    float locSpread = 0.0f; // Filter 1σ position spread (mm)
    int lostCount = 0;
    int relocTries = 0;
    bool needFix = false;
    bool needSpin = false; // Lost on the move: stop softly, then re-localize turning on the spot
    bool reselect = false; // Braked for something seen on the line: choose the next target again
    // Scan on the move: requested, its answer (in scanBuf) and when it was taken
    bool mscanPending = false;
    bool mscanReady = false;
    unsigned long mscanAt = 0;
    uint32_t mscanGen = 0;
    bool wantMoveScan() const;
    void startMoveScan();
    void onMoveScan();
    // After a fix: what the scan says about the run (lost count, obstacles, re-plans)
    void afterFix(bool good, float scanX, float scanY, float scanT, float jumpMM, bool moving);
    // Deferred scan processing (see fetchScan): keeps big buffers off the loop stack
    LdsScanData scanBuf;
    bool scanJobOk = false;
    bool scanReady = false;
    bool inTestMode = false; // TestMode On was sent by this run (finishing must leave it)
    std::function<void(bool, const LdsScanData&)> scanJob;
    void scanThen(const char *what, std::function<void(bool, const LdsScanData&)> handler);
    // Filter update from a scan taken at odometry pose (ox, oy, ot) — standing still:
    // the current one. Returns true for a good fix; the map pose at the scan is
    // left in fixX/fixY/fixT.
    bool applyFix(const LdsScanData& scan, const char *tag) { return applyFixAt(scan, tag, odoX, odoY, odoT); }
    bool applyFixAt(const LdsScanData& scan, const char *tag, float ox, float oy, float ot);
    // Exhaustive match around the pose at the scan when the filter's fix is bad:
    // the cloud may have locked onto a wrong pose (more slip than the motion model)
    bool sensorReset();
    void reseedAround(float x, float y, float t, int radiusMM, int angleDeg, float sigmaXY);
    void trackingFix();

    // -- Start / re-localization -------------------------------------------------------
    int dockScanTries = 0;
    bool dockKnown = false; // Learned dock pose loaded / measured this run
    int dockRuns = 0;
    int spinLeft = 0; // Turn-around stops still to do
    bool spinForStart = false;
    void beginStart();
    void startDockScan();
    void onDockScan(const LdsScanData& scan);
    void beginSpin(bool forStart);
    void spinStep();
    void onSpinDone();
    void learnDockPose(bool save);
    bool globalTried = false;
    bool globalRelocalize();

    // -- Cleaning motors ----------------------------------------------------------
    bool motorsOn = false;
    void setCleaningMotors(bool on);

    // -- Navigation ------------------------------------------------------------------
    void step(); // Decide the next action when idle
    void selectNextTarget();
    // Turn towards / drive straight to (tx, ty)
    void moveTowards(float tx, float ty, int speed);
    // Wheel distances for a drive from the pose to (tx, ty): straight, or a gentle
    // arc that removes the heading error. False when the error exceeds maxErrDeg
    // (a turn in place comes first).
    bool driveCommand(float tx, float ty, int& l, int& r, float maxErrDeg) const;
    // After a fix on the move: re-steer the running drive (or brake for a turn)
    void steer();
    // Near the end of a drive: send the next one now if it bends only slightly
    void chainNext();
    // Waypoints passed on a long drive count as reached
    void passWaypoints();
    void onNavMoveDone(MoveKind kind, float turned, MoveResult r);
    void markReached(size_t idx);
    void buildGrid();
    bool cellBlocked(int cx, int cy) const;
    bool cellBlockedRaw(int cx, int cy) const;
    void buildPlanMask() const;
    mutable std::vector<uint8_t> planMask; // Per plan: 1 = not usable for transit
    bool gridLineFree(float ax, float ay, float bx, float by) const;
    bool lineClearance(float ax, float ay, float bx, float by) const;
    float wallDist(float x, float y) const;
    float dynNearest(float x, float y) const;
    float clearanceAt(float x, float y) const;
    bool onFloor(float x, float y) const;
    FloorTest floorTest = [this](float x, float y) { return onFloor(x, y); };
    bool needEscape = false;
    int escapes = 0;
    float lastEscapeHeading = 0.0f;
    void escapeMove();
    bool planPath(float gx, float gy, std::vector<Waypoint>& out) const;
    bool planPathOnce(float gx, float gy, std::vector<Waypoint>& out) const;
    mutable bool planIgnoreObstacles = false;
    mutable float planClearance = GUIDED_PLAN_CLEARANCE_MM; // Room around transit cells (lowered as a last try)
    void setLeg(std::vector<Waypoint>& pts, bool transit);
    void startNextLeg();
    void loadRunLeg();
    void skipCurrentRun(const char *why);
    bool backtrackOut();
    bool backtracking = false; // The current leg retraces the trail out of a pocket
    int backtracks = 0; // ... in a row without a successful plan
    void onLegComplete();
    void onTransitBlocked();
    void onBlocked();
    void pushCrumb();
    void fallbackReturn();
    void beginReturn(const String& reason);

    // -- Docking ------------------------------------------------------------------------
    // Dock pose in the map frame (learned at the start)
    float dockX = 0.0f;
    float dockY = 0.0f;
    float dockT = 0.0f;
    int16_t refScan[360] = {}; // Dock reference scan (mm per LIDAR angle index, 0 = invalid)
    bool refScanValid = false;
    int dockAttempts = 0;
    int dockIcpFails = 0;
    void toDock(float x, float y, float& lx, float& ly) const;
    void fromDock(float lx, float ly, float& x, float& y) const;
    void beginDocking();
    void dockAlign(); // Face the dock axis, scan, match against the reference
    void onDockScan2(bool ok, const LdsScanData& scan);
    void dockDecide(float lx, float ly, float lt);
    void dockRetry(const char *why);
    void checkDocked();
    // ICP of a scan against the dock reference; guess and result in the dock frame
    bool matchReference(const LdsScanData& scan, float& lx, float& ly, float& lt) const;
    // Robot's own docking (last resort): verified for a while after the hand-over
    bool fallbackDock = false;
    unsigned long dockWatchSince = 0;
    bool dockWatchRetried = false;
    Ticker dockWatchTicker;

    void finish(Phase result, const String& reason);
    void fail(const String& reason) { finish(Phase::Failed, reason); }

    // -- Run trace ---------------------------------------------------------------------
    String traceBuf;
    size_t traceBytes = 0;
    float traceDist = 0.0f;
    float traceTurn = 0.0f;
    unsigned long traceStartMs = 0;
    void traceBegin();
    void traceFlush();
    void tracePose(const String& name, const std::vector<Field>& fields);
    void traceDockScan(const LdsScanData& scan, const char *name, int maxDistMM);
    // Log to the data logger (if enabled) and to the run trace (always)
    void event(const String& name, const std::vector<Field>& fields);

    static const char *phaseName(Phase p);
    static float wrapAngle(float a);
};

#endif // GUIDED_CLEAN_MANAGER_H

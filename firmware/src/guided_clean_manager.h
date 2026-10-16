#ifndef GUIDED_CLEAN_MANAGER_H
#define GUIDED_CLEAN_MANAGER_H

#include <Arduino.h>
#include <functional>
#include <vector>
#include "config.h"
#include "data_logger.h"
#include "floor_map.h"
#include "loop_task.h"
#include "neato_serial.h"

class ManualCleanManager;
class SystemManager;

// Guided clean: room cleaning and no-go zones on top of a pinned floor map.
//
// The robot firmware exposes no zone or map API over serial, so the ESP32
// drives the robot itself (TestMode + SetMotor), replaying the recorded path
// of the pinned session ("line tracer", see issue #68 upstream):
//
//   1. Route   — the pinned path from the dock up to the last point that lies
//                in a selected room. Points inside (or within the robot radius
//                of) a no-go zone are removed. Points inside a selected room
//                are "clean" waypoints (brush/vacuum on), the rest "transit".
//   2. Drive   — stop-and-go segments: turn in place towards the next
//                waypoint, then drive straight (max GUIDED_MAX_STEP_MM), then
//                re-plan. Transit waypoints may be shortcut when the straight
//                line stays on floor the robot already covered during the
//                recording (coverage bitmap) and clear of every no-go zone.
//   3. Pose    — dead reckoning from the wheel encoders (GetMotors
//                Left/RightWheel_PositionInMM), starting at the dock = map
//                origin (0,0, heading 0). There is no relocalization, so drift
//                grows with distance; keep runs short and pin a clean run.
//   4. Safety  — bumpers, wheel stall (load), cliff sensors (DropSensor*) and
//                wheel lift are polled continuously. Contact → stop, back off,
//                skip ahead. Wheel lift → abort. Low battery / too many
//                obstacles / max runtime → return to the dock.
//   5. Return  — retrace the travelled route back to an approach point in
//                front of the dock, face away from it and back onto the
//                contacts (the robot docks rear-first). If the contacts are not
//                detected, fall back to the robot's own SEND_TO_BASE event.
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

private:
    enum class Phase : uint8_t {
        Idle, // Never run since boot
        Starting, // Pre-flight checks + TestMode entry
        Running, // Replaying the route
        Returning, // Driving back to the dock approach point
        Docking, // Turning + backing onto the contacts
        Finishing, // Leaving TestMode
        Done,
        Stopped,
        Failed,
    };

    enum class SegKind : uint8_t { None, Turn, Drive, Reverse, Backoff, DockBack };

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

    // -- Plan -----------------------------------------------------------------
    std::vector<Waypoint> route;
    std::vector<MapZone> nogos;
    CoverGrid cover;
    size_t routeIdx = 0; // Last reached (or skipped) waypoint
    size_t targetIdx = 0; // Waypoint currently driven to
    int cleanTotal = 0;
    int cleanDone = 0;
    int obstacleStreak = 0;
    int obstacleTotal = 0;
    bool fallbackDock = false;
    // 0 = face away from dock, 3 = scan matching, 4 = re-approach, 5 = start back-in,
    // 1 = backing in, 2 = verifying contacts
    uint8_t dockStage = 0;
    unsigned long dockCheckAtMs = 0;
    uint32_t runGen = 0; // Incremented per run/finish; stale serial callbacks compare against it

    String buildPlan(const FloorMapData& map, const String& rooms);
    void freePlan();

    // -- Pose (dead reckoning, mm / radians) -------------------------------------
    float poseX = 0.0f;
    float poseY = 0.0f;
    float poseTheta = 0.0f;
    bool encValid = false;
    int encL = 0;
    int encR = 0;
    void integrateEncoders(int newL, int newR);

    // -- Segment executor ---------------------------------------------------------
    SegKind seg = SegKind::None;
    int segCmdL = 0;
    int segCmdR = 0;
    int segStartL = 0;
    int segStartR = 0;
    unsigned long segStartMs = 0;
    unsigned long segTimeoutMs = 0;
    int segStillPolls = 0;
    int segStallPolls = 0;
    bool segStopping = false; // Stop command in flight; ignore further triggers
    bool waitingCmd = false; // A setup/teardown serial chain is in flight
    bool inTestMode = false; // TestMode entered by this run (must be left on finish)

    void startSegment(SegKind kind, int leftMM, int rightMM, int speed);
    void endSegment(bool obstacle);
    void onObstacle(const char *reason);

    // -- Sensor polling ------------------------------------------------------------
    Ticker pollTicker;
    Ticker batteryTicker;
    uint8_t pollSlot = 0;
    bool pollPending = false;
    void pollSensors();
    void onMotors(const MotorData& m);
    void onDigital(const DigitalSensorData& d);
    void onAnalog(const BatteryAnalogData& a);

    // -- Obstacle memory (bumps, stalls, cliffs seen during this run) ---------------
    struct Obstacle {
        int16_t x = 0;
        int16_t y = 0;
        int16_t r = 0;
    };
    std::vector<Obstacle> obstacles;
    void rememberObstacle(float aheadMM, int radius);

    // -- LIDAR wall compass ---------------------------------------------------------------
    // Heading drift is the dominant odometry error. Rooms are mostly rectangular, so
    // the dominant wall direction seen by the LIDAR (mod 90°) is fixed in the world.
    // A reference is taken on the dock; later scans correct the heading estimate.
    // The LDS angle direction (CW/CCW) is auto-detected from the first turns.
    bool ldsOn = false;
    unsigned long ldsReadyAtMs = 0;
    bool compassTried = false;
    bool compassRef = false;
    float compassPhi0 = 0.0f; // Dominant wall angle in the robot frame at the dock (rad, mod 90°)
    int compassSign = 0; // +1 = LDS angles CCW, -1 = CW, 0 = not yet calibrated
    uint8_t calStage = 0; // Direction calibration: 0 = pending, 1 = turning, 2 = scanned/done
    int calAttempts = 0;
    float distSinceFix = 0.0f; // mm
    float turnSinceFix = 0.0f; // rad
    unsigned long lastScanMs = 0;
    int compassFixes = 0;
    float lastFixDeg = 0.0f;
    void requestScan(bool reference);
    int16_t refScan[360] = {}; // Reference scan on the dock (mm per degree, 0 = invalid)
    bool refScanValid = false;
    int dockIcpTries = 0;
    void requestIcpScan();
    bool matchReference(const LdsScanData& scan, float& corrX, float& corrY, float& corrT) const;
    void applyCompass(const LdsScanData& scan, bool reference);
    static bool dominantWallAngle(const LdsScanData& scan, float& phi, float& conf);

    // -- Cleaning motors ----------------------------------------------------------
    bool motorsOn = false;
    void setCleaningMotors(bool on);

    // -- Phase logic -----------------------------------------------------------------
    void beginStart();
    void step(); // Decide the next segment when idle
    void selectNextTarget();
    // Start a turn or straight segment towards (tx, ty). Returns false if already within tolMM.
    bool driveTowards(float tx, float ty, int speed, bool allowReverse, int tolMM);
    bool segmentClear(float ax, float ay, float bx, float by, bool requireCover) const;
    bool nearNogo(float x, float y) const;
    bool nearObstacle(float x, float y) const;
    void markReached(size_t idx);
    void beginReturn(const String& reason);
    void beginDocking();
    void checkDocked();
    void finish(Phase result, const String& reason);
    void fail(const String& reason) { finish(Phase::Failed, reason); }

    static const char *phaseName(Phase p);
    static float wrapAngle(float a);
};

#endif // GUIDED_CLEAN_MANAGER_H

#ifndef PARTICLE_FILTER_H
#define PARTICLE_FILTER_H

#include <Arduino.h>
#include <functional>
#include <vector>
#include "floor_map.h"
#include "neato_commands.h"

// Monte Carlo localization (particle filter) against the LIDAR likelihood
// field of the mapping run.
//
// Why a particle filter instead of "take the best scan match": a single best
// match is wrong now and then (repetitive walls, clutter, a smeared scan) and
// the robot then drives on with a wrong pose and never recovers. The filter
// keeps many pose hypotheses, weighs each by how well the scan fits the map,
// and moves them with the wheel odometry (with noise). Its spread says how
// sure it is, so the controller can stop and re-localize instead of driving
// on blind.
//
// Units: mm and radians, map frame. Scans are only fed while the robot stands
// still (the 115200 baud link needs ~0.7 s per scan; scans on the move smear).

// Scan points in the robot frame (LIDAR mounting from the field calibration)
struct ScanBeams {
    static const int MAX = 180;
    float x[MAX];
    float y[MAX];
    int n = 0;
};

// stride: use every n-th degree. Returns the number of beams.
int extractBeams(const LdsScanData& scan, const LocField& field, int stride, ScanBeams& out);
// Valid returns in a raw scan (spin-up check)
int validScanPoints(const LdsScanData& scan);
// Mean field value (0..1) under the beams at a pose: how well a pose explains the scan
float poseQuality(const LocField& field, const ScanBeams& b, float x, float y, float t);

// Where the robot can be at all (known floor): poses elsewhere are never taken,
// however well the scan fits (the map's walls also "fit" from outside the house)
using FloorTest = std::function<bool(float, float)>;

// Exhaustive correlative search around (cx, cy, ct) within ±radius / ±angle, then a
// local refinement. Used to (re)seed the filter. Returns the normalized score (0..1).
float correlativeMatch(const LocField& field, const ScanBeams& b, float cx, float cy, float ct, int radiusMM,
                       int angleDeg, float& ox, float& oy, float& ot, const FloorTest& onFloor = nullptr);

class ParticleFilter {
public:
    struct Particle {
        float x;
        float y;
        float t;
        float w;
    };

    explicit ParticleFilter(int count = 240) : count(count) {}

    bool active() const { return !p.empty(); }
    void clear();
    // Gaussian cloud around a pose
    void seed(float x, float y, float t, float sigmaXY, float sigmaT);
    // Odometry step, given in the robot frame of the previous estimate
    void predict(float dxl, float dyl, float dth);
    // Weigh with a scan; resamples when the weights degenerate
    void update(const LocField& field, const ScanBeams& b, const FloorTest& onFloor = nullptr);
    // Weighted mean pose and spread (1σ position in mm, heading in rad)
    void estimate(float& x, float& y, float& t, float& spreadXY, float& spreadT) const;

private:
    int count;
    std::vector<Particle> p;
    uint32_t rng = 0x9E3779B9u;
    float logLut[256] = {};
    bool lutReady = false;

    float uniform();
    float gauss();
    void resample();
};

#endif // PARTICLE_FILTER_H

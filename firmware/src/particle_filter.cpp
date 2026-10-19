#include "particle_filter.h"
#include <cmath>

static const float PF_DEG = 3.14159265f / 180.0f;

static float wrapPi(float a) {
    while (a > 3.14159265f)
        a -= 2.0f * 3.14159265f;
    while (a < -3.14159265f)
        a += 2.0f * 3.14159265f;
    return a;
}

int validScanPoints(const LdsScanData& scan) {
    int n = 0;
    for (const LdsScanPoint& pt: scan.points)
        if (pt.errorCode == 0 && pt.distMM >= 150 && pt.distMM <= 5000)
            n++;
    return n;
}

int extractBeams(const LdsScanData& scan, const LocField& field, int stride, ScanBeams& out) {
    out.n = 0;
    auto sgn = static_cast<float>(field.lidarSign);
    for (int i = 0; i < 360 && out.n < ScanBeams::MAX; i += stride) {
        const LdsScanPoint& pt = scan.points[i];
        if (pt.errorCode != 0 || pt.distMM < 150 || pt.distMM > 5000)
            continue;
        float a = sgn * static_cast<float>(pt.angleDeg) * PF_DEG + field.lidarAngleOffsetRad;
        out.x[out.n] = static_cast<float>(field.lidarOffsetX) + static_cast<float>(pt.distMM) * cosf(a);
        out.y[out.n] = static_cast<float>(pt.distMM) * sinf(a);
        out.n++;
    }
    return out.n;
}

static inline int cellOf(float v, int origin, float cell) {
    return static_cast<int>(floorf((v - static_cast<float>(origin)) / cell));
}

float poseQuality(const LocField& field, const ScanBeams& b, float x, float y, float t) {
    if (b.n == 0)
        return 0.0f;
    float c = cosf(t);
    float s = sinf(t);
    auto cell = static_cast<float>(field.cell);
    int sum = 0;
    for (int i = 0; i < b.n; i++) {
        sum += field.cellv(cellOf(x + c * b.x[i] - s * b.y[i], field.originX, cell),
                           cellOf(y + s * b.x[i] + c * b.y[i], field.originY, cell));
    }
    return static_cast<float>(sum) / (static_cast<float>(b.n) * 255.0f);
}

// Bilinear field sample (sub-cell precision for the refinement)
static float sampleField(const LocField& field, float x, float y) {
    float fx = (x - static_cast<float>(field.originX)) / static_cast<float>(field.cell) - 0.5f;
    float fy = (y - static_cast<float>(field.originY)) / static_cast<float>(field.cell) - 0.5f;
    int x0 = static_cast<int>(floorf(fx));
    int y0 = static_cast<int>(floorf(fy));
    float tx = fx - static_cast<float>(x0);
    float ty = fy - static_cast<float>(y0);
    auto v = [&field](int cx, int cy) -> float { return static_cast<float>(field.cellv(cx, cy)); };
    return (v(x0, y0) * (1 - tx) + v(x0 + 1, y0) * tx) * (1 - ty) +
           (v(x0, y0 + 1) * (1 - tx) + v(x0 + 1, y0 + 1) * tx) * ty;
}

float correlativeMatch(const LocField& field, const ScanBeams& b, float cx, float cy, float ct, int radiusMM,
                       int angleDeg, float& ox, float& oy, float& ot, const FloorTest& onFloor) {
    // Scratch in static storage: this runs on the small loop stack
    static int bx[ScanBeams::MAX];
    static int by[ScanBeams::MAX];
    ox = cx;
    oy = cy;
    ot = ct;
    if (b.n < 30)
        return 0.0f;
    const auto cell = static_cast<float>(field.cell);
    const int steps = radiusMM / field.cell;
    const int kstep = radiusMM > 400 ? 2 : 1;
    const int degStep = angleDeg > 15 ? 3 : 2;
    float bestScore = -1.0f;
    for (int da = -angleDeg; da <= angleDeg; da += degStep) {
        float th = ct + static_cast<float>(da) * PF_DEG;
        float c = cosf(th);
        float s = sinf(th);
        for (int i = 0; i < b.n; i++) {
            bx[i] = cellOf(cx + c * b.x[i] - s * b.y[i], field.originX, cell);
            by[i] = cellOf(cy + s * b.x[i] + c * b.y[i], field.originY, cell);
        }
        for (int ky = -steps; ky <= steps; ky += kstep) {
            for (int kx = -steps; kx <= steps; kx += kstep) {
                int sum = 0;
                for (int i = 0; i < b.n; i++)
                    sum += field.cellv(bx[i] + kx, by[i] + ky);
                // Slight preference for small corrections breaks ties on repetitive walls
                float score = static_cast<float>(sum) - static_cast<float>(abs(kx) + abs(ky) + abs(da) / 2) * 2.0f;
                if (score > bestScore && onFloor &&
                    !onFloor(cx + static_cast<float>(kx) * cell, cy + static_cast<float>(ky) * cell))
                    continue;
                if (score > bestScore) {
                    bestScore = score;
                    ot = th;
                    ox = cx + static_cast<float>(kx) * cell;
                    oy = cy + static_cast<float>(ky) * cell;
                }
            }
        }
    }
    auto scoreAt = [&](float x, float y, float th) {
        float c = cosf(th);
        float s = sinf(th);
        float sum = 0.0f;
        for (int i = 0; i < b.n; i++)
            sum += sampleField(field, x + c * b.x[i] - s * b.y[i], y + s * b.x[i] + c * b.y[i]);
        return sum;
    };
    float refined = scoreAt(ox, oy, ot);
    float step = cell * static_cast<float>(kstep) / 2.0f;
    float astep = static_cast<float>(degStep) / 2.0f * PF_DEG;
    for (int iter = 0; iter < 12; iter++) {
        bool improved = false;
        const float cand[6][3] = {{step, 0, 0},  {-step, 0, 0}, {0, step, 0},
                                  {0, -step, 0}, {0, 0, astep}, {0, 0, -astep}};
        for (const auto& d: cand) {
            float sc = scoreAt(ox + d[0], oy + d[1], ot + d[2]);
            if (sc > refined) {
                refined = sc;
                ox += d[0];
                oy += d[1];
                ot += d[2];
                improved = true;
            }
        }
        if (!improved) {
            step /= 2.0f;
            astep /= 2.0f;
        }
    }
    ot = wrapPi(ot);
    if (onFloor && !onFloor(ox, oy))
        return 0.0f; // Refined off the floor: no valid match
    return refined / (static_cast<float>(b.n) * 255.0f);
}

// -- Filter ---------------------------------------------------------------------

float ParticleFilter::uniform() {
    // xorshift32: deterministic, no libc state
    rng ^= rng << 13;
    rng ^= rng >> 17;
    rng ^= rng << 5;
    return static_cast<float>(rng >> 8) / 16777216.0f;
}

float ParticleFilter::gauss() {
    // Sum of 4 uniforms ≈ normal (σ = 1 after scaling), cheap and bounded
    float s = uniform() + uniform() + uniform() + uniform() - 2.0f;
    return s * 1.7320508f;
}

void ParticleFilter::clear() {
    std::vector<Particle>().swap(p);
}

void ParticleFilter::seed(float x, float y, float t, float sigmaXY, float sigmaT) {
    p.assign(static_cast<size_t>(count), Particle{x, y, t, 1.0f / static_cast<float>(count)});
    for (size_t i = 1; i < p.size(); i++) {
        p[i].x += gauss() * sigmaXY;
        p[i].y += gauss() * sigmaXY;
        p[i].t = wrapPi(p[i].t + gauss() * sigmaT);
    }
}

void ParticleFilter::predict(float dxl, float dyl, float dth) {
    if (p.empty())
        return;
    float dist = sqrtf(dxl * dxl + dyl * dyl);
    // Odometry error model: slip along the travel, a little sideways, and heading
    // error from turning (wheel slip on carpet / thresholds) and from driving
    float sAlong = 0.05f * dist + 3.0f;
    float sSide = 0.02f * dist + 3.0f;
    float sTurn = 0.04f * fabsf(dth) + 0.025f * dist / 1000.0f + 0.4f * PF_DEG;
    for (Particle& q: p) {
        float ax = dxl + gauss() * sAlong;
        float ay = dyl + gauss() * sSide;
        float c = cosf(q.t);
        float s = sinf(q.t);
        q.x += c * ax - s * ay;
        q.y += s * ax + c * ay;
        q.t = wrapPi(q.t + dth + gauss() * sTurn);
    }
}

void ParticleFilter::update(const LocField& field, const ScanBeams& b, const FloorTest& onFloor) {
    if (p.empty() || b.n < 20)
        return;
    if (!lutReady) {
        // A beam far from any mapped wall is not impossible (furniture moved,
        // people, open doors): floor of 0.08 keeps one outlier from killing a particle
        for (int v = 0; v < 256; v++)
            logLut[v] = logf(0.08f + 0.92f * static_cast<float>(v) / 255.0f);
        lutReady = true;
    }
    // Use at most ~60 beams: plenty for the weighting, keeps it to a few ms
    int stride = b.n > 60 ? b.n / 60 : 1;
    const auto cell = static_cast<float>(field.cell);
    float best = -1e30f;
    std::vector<float> ll(p.size());
    int used = 0;
    for (size_t k = 0; k < p.size(); k++) {
        const Particle& q = p[k];
        float c = cosf(q.t);
        float s = sinf(q.t);
        float sum = 0.0f;
        used = 0;
        for (int i = 0; i < b.n; i += stride) {
            int cx = cellOf(q.x + c * b.x[i] - s * b.y[i], field.originX, cell);
            int cy = cellOf(q.y + s * b.x[i] + c * b.y[i], field.originY, cell);
            sum += logLut[field.cellv(cx, cy)];
            used++;
        }
        if (onFloor && !onFloor(q.x, q.y))
            sum -= 1000.0f; // Off the floor (inside a wall, outside the house): impossible
        ll[k] = sum;
        if (sum > best)
            best = sum;
    }
    // Neighbouring beams are strongly correlated: temper the likelihood so the
    // cloud does not collapse onto one particle after a single scan
    float tau = static_cast<float>(used) / 8.0f;
    if (tau < 1.0f)
        tau = 1.0f;
    float total = 0.0f;
    for (size_t k = 0; k < p.size(); k++) {
        p[k].w *= expf((ll[k] - best) / tau);
        total += p[k].w;
    }
    if (total <= 0.0f) {
        for (Particle& q: p)
            q.w = 1.0f / static_cast<float>(p.size());
        return;
    }
    float sq = 0.0f;
    for (Particle& q: p) {
        q.w /= total;
        sq += q.w * q.w;
    }
    float neff = 1.0f / sq;
    if (neff < static_cast<float>(p.size()) / 2.0f)
        resample();
}

void ParticleFilter::resample() {
    // Low-variance (systematic) resampling, then slight roughening against depletion
    std::vector<Particle> out;
    out.reserve(p.size());
    auto n = static_cast<float>(p.size());
    float r = uniform() / n;
    float c = p[0].w;
    size_t i = 0;
    for (size_t m = 0; m < p.size(); m++) {
        float u = r + static_cast<float>(m) / n;
        while (u > c && i + 1 < p.size()) {
            i++;
            c += p[i].w;
        }
        Particle q = p[i];
        q.x += gauss() * 4.0f;
        q.y += gauss() * 4.0f;
        q.t = wrapPi(q.t + gauss() * 0.3f * PF_DEG);
        q.w = 1.0f / n;
        out.push_back(q);
    }
    p.swap(out);
}

void ParticleFilter::estimate(float& x, float& y, float& t, float& spreadXY, float& spreadT) const {
    x = y = t = 0.0f;
    spreadXY = spreadT = 1e6f;
    if (p.empty())
        return;
    float sw = 0.0f;
    float sc = 0.0f;
    float ss = 0.0f;
    for (const Particle& q: p) {
        x += q.w * q.x;
        y += q.w * q.y;
        sc += q.w * cosf(q.t);
        ss += q.w * sinf(q.t);
        sw += q.w;
    }
    if (sw <= 0.0f)
        return;
    x /= sw;
    y /= sw;
    t = atan2f(ss, sc);
    float vxy = 0.0f;
    float vt = 0.0f;
    for (const Particle& q: p) {
        float dx = q.x - x;
        float dy = q.y - y;
        float dt = wrapPi(q.t - t);
        vxy += q.w * (dx * dx + dy * dy);
        vt += q.w * dt * dt;
    }
    spreadXY = sqrtf(vxy / sw);
    spreadT = sqrtf(vt / sw);
}

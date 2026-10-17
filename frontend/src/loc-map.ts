// Localization map ("likelihood field") built from a mapping run.
//
// A mapping run is a normal house clean during which the firmware also stores
// LIDAR scans together with the robot's own SLAM pose. Here the scans are
// projected into a 5 cm occupancy grid (walls + furniture), the LIDAR mounting
// (angle direction, angle offset, distance from the robot centre) is
// calibrated by picking the variant that gives the sharpest map, and a
// distance-based likelihood field is derived for the firmware's scan matcher.
// Binary layout: see firmware/src/floor_map.h (LocField).

import type { MapScan } from "./types";

export interface LidarMount {
    sign: 1 | -1; // +1: angles counter-clockwise
    angleOffsetDeg: number;
    offsetX: number; // mm along the robot heading
}

export interface LocMap {
    originX: number; // mm
    originY: number;
    cell: number; // mm
    w: number;
    h: number;
    mount: LidarMount;
    field: Uint8Array; // w*h, 255 = wall
}

const BIN_COUNT = 180;
const UNIT_MM = 20;
const MIN_RANGE_MM = 150;
const MAX_RANGE_MM = 4500;
const FALLOFF_MM = 300;
const MAX_CELLS = 72000; // keeps the field ≤ 72 KB in ESP32 RAM

function scanPoints(scan: MapScan, mount: LidarMount, step = 1): [number, number][] {
    const pts: [number, number][] = [];
    const th = (scan.t * Math.PI) / 180;
    const c = Math.cos(th);
    const s = Math.sin(th);
    const rx = scan.x * 1000;
    const ry = scan.y * 1000;
    for (let b = 0; b < BIN_COUNT; b += step) {
        const d = scan.bins[b] * UNIT_MM;
        if (d < MIN_RANGE_MM || d > MAX_RANGE_MM) continue;
        const a = mount.sign * (((b * 2 + 0.5) * Math.PI) / 180) + (mount.angleOffsetDeg * Math.PI) / 180;
        const lx = mount.offsetX + d * Math.cos(a);
        const ly = d * Math.sin(a);
        pts.push([rx + c * lx - s * ly, ry + s * lx + c * ly]);
    }
    return pts;
}

// Fewer distinct occupied cells for the same scans = walls line up = right mount
function sharpness(scans: MapScan[], mount: LidarMount): number {
    const cells = new Set<number>();
    for (const sc of scans) {
        for (const [x, y] of scanPoints(sc, mount, 2)) {
            cells.add(Math.floor(x / 60) * 100003 + Math.floor(y / 60));
        }
    }
    return cells.size;
}

export function calibrateMount(scans: MapScan[]): LidarMount {
    const sample = scans.filter((_, i) => i % Math.max(1, Math.floor(scans.length / 120)) === 0);
    let best: LidarMount = { sign: 1, angleOffsetDeg: 0, offsetX: 0 };
    let bestScore = Infinity;
    for (const sign of [1, -1] as const) {
        for (const angleOffsetDeg of [0, 90, 180, 270]) {
            const score = sharpness(sample, { sign, angleOffsetDeg, offsetX: 0 });
            if (score < bestScore) {
                bestScore = score;
                best = { sign, angleOffsetDeg, offsetX: 0 };
            }
        }
    }
    // Fine angle trim, then the turret position along the heading axis
    let fine = best;
    for (let a = -8; a <= 8; a += 2) {
        const m = { ...best, angleOffsetDeg: best.angleOffsetDeg + a };
        const score = sharpness(sample, m);
        if (score < bestScore) {
            bestScore = score;
            fine = m;
        }
    }
    best = fine;
    for (let off = -150; off <= 150; off += 25) {
        const m = { ...fine, offsetX: off };
        const score = sharpness(sample, m);
        if (score < bestScore) {
            bestScore = score;
            best = m;
        }
    }
    return best;
}

export function buildLocMap(scans: MapScan[]): LocMap | null {
    if (scans.length < 10) return null;
    const mount = calibrateMount(scans);

    const pts: [number, number][] = [];
    for (const sc of scans) pts.push(...scanPoints(sc, mount));
    if (pts.length < 200) return null;
    let minX = Infinity;
    let minY = Infinity;
    let maxX = -Infinity;
    let maxY = -Infinity;
    for (const [x, y] of pts) {
        minX = Math.min(minX, x);
        minY = Math.min(minY, y);
        maxX = Math.max(maxX, x);
        maxY = Math.max(maxY, y);
    }
    const pad = 400;
    let cell = 50;
    let w = 0;
    let h = 0;
    for (;;) {
        w = Math.ceil((maxX - minX + 2 * pad) / cell);
        h = Math.ceil((maxY - minY + 2 * pad) / cell);
        if (w * h <= MAX_CELLS) break;
        cell += 10;
    }
    const originX = Math.floor(minX - pad);
    const originY = Math.floor(minY - pad);

    // Occupancy: a cell is a wall when hit by at least two scans
    const hits = new Uint16Array(w * h);
    for (const [x, y] of pts) {
        const cx = Math.floor((x - originX) / cell);
        const cy = Math.floor((y - originY) / cell);
        if (cx >= 0 && cy >= 0 && cx < w && cy < h) hits[cy * w + cx]++;
    }

    // Distance transform (two-pass chamfer, in mm)
    const INF = 1e9;
    const dist = new Float32Array(w * h);
    // Walls: hit by at least two scans and not an isolated speck (a single cell
    // without any occupied neighbour is mostly noise: people, cables, smeared returns)
    const occ = (x: number, y: number) => x >= 0 && y >= 0 && x < w && y < h && hits[y * w + x] >= 2;
    for (let y = 0; y < h; y++) {
        for (let x = 0; x < w; x++) {
            let wall = occ(x, y);
            if (wall) {
                let n = 0;
                for (let dy = -1; dy <= 1; dy++)
                    for (let dx = -1; dx <= 1; dx++) if ((dx || dy) && occ(x + dx, y + dy)) n++;
                wall = n > 0;
            }
            dist[y * w + x] = wall ? 0 : INF;
        }
    }
    const d1 = cell;
    const d2 = cell * Math.SQRT2;
    for (let y = 0; y < h; y++) {
        for (let x = 0; x < w; x++) {
            const i = y * w + x;
            let v = dist[i];
            if (x > 0) v = Math.min(v, dist[i - 1] + d1);
            if (y > 0) {
                v = Math.min(v, dist[i - w] + d1);
                if (x > 0) v = Math.min(v, dist[i - w - 1] + d2);
                if (x < w - 1) v = Math.min(v, dist[i - w + 1] + d2);
            }
            dist[i] = v;
        }
    }
    for (let y = h - 1; y >= 0; y--) {
        for (let x = w - 1; x >= 0; x--) {
            const i = y * w + x;
            let v = dist[i];
            if (x < w - 1) v = Math.min(v, dist[i + 1] + d1);
            if (y < h - 1) {
                v = Math.min(v, dist[i + w] + d1);
                if (x < w - 1) v = Math.min(v, dist[i + w + 1] + d2);
                if (x > 0) v = Math.min(v, dist[i + w - 1] + d2);
            }
            dist[i] = v;
        }
    }
    const field = new Uint8Array(w * h);
    for (let i = 0; i < w * h; i++) field[i] = Math.max(0, Math.round(255 * (1 - dist[i] / FALLOFF_MM)));

    return { originX, originY, cell, w, h, mount, field };
}

export function serializeLocMap(m: LocMap): Uint8Array {
    const out = new Uint8Array(26 + m.field.length);
    const dv = new DataView(out.buffer);
    out.set([0x4f, 0x46, 0x4c, 0x31]); // "OFL1"
    dv.setInt32(4, m.originX, true);
    dv.setInt32(8, m.originY, true);
    dv.setUint16(12, m.cell, true);
    dv.setUint16(14, m.w, true);
    dv.setUint16(16, m.h, true);
    dv.setInt8(18, m.mount.sign);
    dv.setInt16(20, Math.round(m.mount.angleOffsetDeg * 10), true);
    dv.setInt16(22, Math.round(m.mount.offsetX), true);
    out.set(m.field, 26);
    return out;
}

export function parseLocMap(buf: ArrayBuffer): LocMap | null {
    if (buf.byteLength < 26) return null;
    const b = new Uint8Array(buf);
    if (b[0] !== 0x4f || b[1] !== 0x46 || b[2] !== 0x4c || b[3] !== 0x31) return null;
    const dv = new DataView(buf);
    const w = dv.getUint16(14, true);
    const h = dv.getUint16(16, true);
    if (buf.byteLength < 26 + w * h) return null;
    return {
        originX: dv.getInt32(4, true),
        originY: dv.getInt32(8, true),
        cell: dv.getUint16(12, true),
        w,
        h,
        mount: {
            sign: dv.getInt8(18) < 0 ? -1 : 1,
            angleOffsetDeg: dv.getInt16(20, true) / 10,
            offsetX: dv.getInt16(22, true),
        },
        field: b.slice(26, 26 + w * h),
    };
}

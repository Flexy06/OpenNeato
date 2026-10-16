// Floor map model shared by the map view and the history "use as floor map"
// action. The firmware stores the map as a compact line-based "OFM1" text
// document (see firmware/src/floor_map.h); this module converts between that
// format and the in-memory model (metres, map frame of the pinned session,
// dock = origin).

import type { LocMap } from "./loc-map";
import type { MapData } from "./types";

export type MapVertex = [number, number];

export interface FloorMapZone {
    id: number;
    name: string;
    poly: MapVertex[];
}

export interface FloorMap {
    source: string;
    path: MapVertex[];
    rooms: FloorMapZone[];
    nogos: FloorMapZone[];
}

export interface MapBox {
    minX: number;
    maxX: number;
    minY: number;
    maxY: number;
}

// Firmware limits (config.h FLOORMAP_*)
export const MAX_PATH_POINTS = 3000;
export const MAX_ZONES = 16;
export const MAX_VERTICES = 24;
const MIN_POINT_SPACING_M = 0.1;
const POINTS_PER_LINE = 60;

// Distinct, theme-neutral room tints (index = room id modulo length)
export const ROOM_COLORS = ["#3b9eff", "#bf5af2", "#30d5c8", "#e5972a", "#ff6b9d", "#5ac8fa", "#34c759", "#f9ebb2"];

export function roomColor(id: number): string {
    return ROOM_COLORS[(id - 1 + ROOM_COLORS.length) % ROOM_COLORS.length];
}

function mm(v: number): number {
    return Math.round(v * 1000);
}

function formatPoints(points: MapVertex[]): string {
    return points.map(([x, y]) => `${mm(x)},${mm(y)}`).join(" ");
}

function parsePoints(tokens: string[]): MapVertex[] | null {
    const out: MapVertex[] = [];
    for (const tok of tokens) {
        const [xs, ys] = tok.split(",");
        const x = Number(xs);
        const y = Number(ys);
        if (!Number.isFinite(x) || !Number.isFinite(y) || ys === undefined) return null;
        out.push([x / 1000, y / 1000]);
    }
    return out;
}

/** Serialize to the firmware's OFM1 text format */
export function serializeFloorMap(map: FloorMap): string {
    const lines = ["OFM1"];
    if (map.source) lines.push(`S ${map.source}`);
    for (let i = 0; i < map.path.length; i += POINTS_PER_LINE) {
        lines.push(`P ${formatPoints(map.path.slice(i, i + POINTS_PER_LINE))}`);
    }
    for (const r of map.rooms) {
        lines.push(`R ${r.id} ${encodeURIComponent(r.name || `Room ${r.id}`)} ${formatPoints(r.poly)}`);
    }
    for (const n of map.nogos) lines.push(`N ${n.id} ${formatPoints(n.poly)}`);
    return `${lines.join("\n")}\n`;
}

/** Parse an OFM1 document. Returns null when the header is missing. */
export function parseFloorMap(text: string): FloorMap | null {
    const lines = text.split("\n").map((l) => l.trim());
    if (lines[0] !== "OFM1") return null;
    const map: FloorMap = { source: "", path: [], rooms: [], nogos: [] };
    for (const line of lines.slice(1)) {
        if (line.length < 2 || line[1] !== " ") continue;
        const kind = line[0];
        const tokens = line.slice(2).split(" ").filter(Boolean);
        if (kind === "S") {
            map.source = line.slice(2);
        } else if (kind === "P") {
            const pts = parsePoints(tokens);
            if (pts) map.path.push(...pts);
        } else if (kind === "R" && tokens.length >= 5) {
            const poly = parsePoints(tokens.slice(2));
            let name = tokens[1];
            try {
                name = decodeURIComponent(name);
            } catch {
                // keep raw name
            }
            if (poly) map.rooms.push({ id: Number(tokens[0]), name, poly });
        } else if (kind === "N" && tokens.length >= 4) {
            const poly = parsePoints(tokens.slice(1));
            if (poly) map.nogos.push({ id: Number(tokens[0]), name: "", poly });
        }
    }
    return map;
}

/** Thin a recorded session path to the firmware's point budget */
export function decimatePath(points: MapVertex[]): MapVertex[] {
    let spacing = MIN_POINT_SPACING_M;
    for (;;) {
        const out: MapVertex[] = [];
        for (const p of points) {
            const prev = out[out.length - 1];
            if (!prev || Math.hypot(p[0] - prev[0], p[1] - prev[1]) >= spacing) out.push(p);
        }
        if (out.length <= MAX_PATH_POINTS) return out;
        spacing *= 1.3;
    }
}

/** Build a floor map from a recorded session, keeping existing zones (same dock frame) */
export function floorMapFromSession(name: string, session: MapData, previous: FloorMap | null): FloorMap {
    const path = decimatePath(session.path.map((p) => [p.x, p.y] as MapVertex));
    return {
        source: name,
        path,
        rooms: previous?.rooms ?? [],
        nogos: previous?.nogos ?? [],
    };
}

export function nextZoneId(zones: FloorMapZone[]): number {
    let id = 1;
    while (zones.some((z) => z.id === id)) id++;
    return id;
}

export function rectPolygon(a: MapVertex, b: MapVertex): MapVertex[] {
    const x0 = Math.min(a[0], b[0]);
    const x1 = Math.max(a[0], b[0]);
    const y0 = Math.min(a[1], b[1]);
    const y1 = Math.max(a[1], b[1]);
    return [
        [x0, y0],
        [x1, y0],
        [x1, y1],
        [x0, y1],
    ];
}

export function pointInPolygon(x: number, y: number, poly: MapVertex[]): boolean {
    let inside = false;
    for (let i = 0, j = poly.length - 1; i < poly.length; j = i++) {
        const [xi, yi] = poly[i];
        const [xj, yj] = poly[j];
        if (yi > y !== yj > y && x < ((xj - xi) * (y - yi)) / (yj - yi) + xi) inside = !inside;
    }
    return inside;
}

export function polygonArea(poly: MapVertex[]): number {
    let a = 0;
    for (let i = 0, j = poly.length - 1; i < poly.length; j = i++) {
        a += (poly[j][0] + poly[i][0]) * (poly[j][1] - poly[i][1]);
    }
    return Math.abs(a / 2);
}

export function polygonCentroid(poly: MapVertex[]): MapVertex {
    let x = 0;
    let y = 0;
    for (const p of poly) {
        x += p[0];
        y += p[1];
    }
    return [x / poly.length, y / poly.length];
}

/** Share of recorded path points that fall inside the polygon */
export function pathShareInside(path: MapVertex[], poly: MapVertex[]): number {
    if (path.length === 0) return 0;
    let n = 0;
    for (const [x, y] of path) if (pointInPolygon(x, y, poly)) n++;
    return n / path.length;
}

export function floorMapBounds(map: FloorMap, loc?: LocMap | null): MapBox | null {
    const all: MapVertex[] = [[0, 0], ...map.path];
    if (loc) {
        // Wall extent (the field is padded by 0.4 m on each side)
        all.push([(loc.originX + 400) / 1000, (loc.originY + 400) / 1000]);
        all.push([(loc.originX + loc.w * loc.cell - 400) / 1000, (loc.originY + loc.h * loc.cell - 400) / 1000]);
    }
    for (const z of [...map.rooms, ...map.nogos]) all.push(...z.poly);
    if (all.length === 0) return null;
    const pad = 0.4;
    return {
        minX: Math.min(...all.map((p) => p[0])) - pad,
        maxX: Math.max(...all.map((p) => p[0])) + pad,
        minY: Math.min(...all.map((p) => p[1])) - pad,
        maxY: Math.max(...all.map((p) => p[1])) + pad,
    };
}

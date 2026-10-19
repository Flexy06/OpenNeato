// Floor map canvas: view transform (pan / zoom / 90° rotation) and renderer.
// Independent from the history renderer because the editor needs a stable
// world<->screen mapping for hit testing and drawing zones.

import type { FloorMap, FloorMapZone, MapBox, MapVertex } from "../../floor-map";
import type { GuidedTrace } from "../../guided-trace";
import type { LocMap } from "../../loc-map";
import { polygonCentroid, roomColor } from "../../floor-map";

export interface MapView {
    cx: number; // world centre (m)
    cy: number;
    scale: number; // px per metre
    rot: number; // 0 / 90 / 180 / 270, clockwise on screen
}

export interface Screen {
    w: number;
    h: number;
}

export function toScreen(view: MapView, s: Screen, x: number, y: number): MapVertex {
    const r = (view.rot * Math.PI) / 180;
    const dx = x - view.cx;
    const dy = y - view.cy;
    const u = Math.cos(r) * dx + Math.sin(r) * dy;
    const v = -Math.sin(r) * dx + Math.cos(r) * dy;
    return [s.w / 2 + u * view.scale, s.h / 2 - v * view.scale];
}

export function toWorld(view: MapView, s: Screen, sx: number, sy: number): MapVertex {
    const r = (view.rot * Math.PI) / 180;
    const u = (sx - s.w / 2) / view.scale;
    const v = -(sy - s.h / 2) / view.scale;
    return [view.cx + Math.cos(r) * u - Math.sin(r) * v, view.cy + Math.sin(r) * u + Math.cos(r) * v];
}

export function fitView(box: MapBox, s: Screen, rot: number): MapView {
    const w = box.maxX - box.minX;
    const h = box.maxY - box.minY;
    const sideways = rot === 90 || rot === 270;
    const sw = sideways ? h : w;
    const sh = sideways ? w : h;
    const scale = Math.min(s.w / Math.max(sw, 0.5), s.h / Math.max(sh, 0.5)) * 0.92;
    return { cx: (box.minX + box.maxX) / 2, cy: (box.minY + box.maxY) / 2, scale, rot };
}

/** Zoom by `factor` keeping the world point under (sx, sy) fixed */
export function zoomAt(view: MapView, s: Screen, sx: number, sy: number, factor: number): MapView {
    const scale = Math.max(8, Math.min(600, view.scale * factor));
    const [wx, wy] = toWorld(view, s, sx, sy);
    const next = { ...view, scale };
    const [nx, ny] = toWorld(next, s, sx, sy);
    return { ...next, cx: view.cx + (wx - nx), cy: view.cy + (wy - ny) };
}

/** Pan by a screen-space delta */
export function panBy(view: MapView, s: Screen, dx: number, dy: number): MapView {
    const [ax, ay] = toWorld(view, s, 0, 0);
    const [bx, by] = toWorld(view, s, dx, dy);
    return { ...view, cx: view.cx - (bx - ax), cy: view.cy - (by - ay) };
}

// -- Rendering ------------------------------------------------------------------

export interface RobotPose {
    x: number;
    y: number;
    t: number; // degrees, CCW
}

export interface RenderState {
    map: FloorMap;
    view: MapView;
    cleanSelection: Set<number>; // Room ids selected for cleaning
    editZone: { kind: "room" | "nogo"; id: number } | null;
    activeVertex: number | null;
    draft: { kind: "room" | "nogo"; a: MapVertex; b: MapVertex } | null;
    robot: RobotPose | null;
    target: MapVertex | null;
    trail: MapVertex[];
    showPath: boolean;
    loc: LocMap | null;
    trace: GuidedTrace | null; // Last guided run (analysis overlay)
}

interface Palette {
    surface: string;
    grid: string;
    coverage: string;
    path: string;
    text: string;
    textDim: string;
    red: string;
    green: string;
    blue: string;
    accent: string;
    dark: boolean;
}

function palette(canvas: HTMLCanvasElement): Palette {
    const cs = getComputedStyle(canvas);
    const v = (name: string, fb: string) => cs.getPropertyValue(name).trim() || fb;
    const surface = v("--surface", "#1a1a1c");
    const dark = surface.startsWith("#1");
    return {
        surface,
        grid: dark ? "rgba(255, 255, 255, 0.04)" : "rgba(0, 0, 0, 0.06)",
        coverage: dark ? "rgba(124, 199, 156, 0.12)" : "rgba(36, 77, 67, 0.12)",
        path: dark ? "rgba(222, 205, 160, 0.35)" : "rgba(150, 120, 60, 0.35)",
        text: v("--text", "#f0f0f2"),
        textDim: v("--text-dim", "#8a8a8e"),
        red: v("--red", "#ff453a"),
        green: v("--green", "#34c759"),
        blue: v("--accent", "#9cc8b2"),
        accent: v("--accent", "#f9ebb2"),
        dark,
    };
}

function withAlpha(hex: string, alpha: number): string {
    const m = hex.match(/^#([0-9a-f]{6})$/i);
    if (!m) return hex;
    const n = Number.parseInt(m[1], 16);
    return `rgba(${(n >> 16) & 255}, ${(n >> 8) & 255}, ${n & 255}, ${alpha})`;
}

function tracePolygon(ctx: CanvasRenderingContext2D, pts: MapVertex[]) {
    ctx.beginPath();
    pts.forEach(([x, y], i) => (i === 0 ? ctx.moveTo(x, y) : ctx.lineTo(x, y)));
    ctx.closePath();
}

function drawGrid(ctx: CanvasRenderingContext2D, st: RenderState, s: Screen, pal: Palette) {
    const corners = [
        toWorld(st.view, s, 0, 0),
        toWorld(st.view, s, s.w, 0),
        toWorld(st.view, s, 0, s.h),
        toWorld(st.view, s, s.w, s.h),
    ];
    const minX = Math.min(...corners.map((c) => c[0]));
    const maxX = Math.max(...corners.map((c) => c[0]));
    const minY = Math.min(...corners.map((c) => c[1]));
    const maxY = Math.max(...corners.map((c) => c[1]));
    const step = st.view.scale < 25 ? 1 : 0.5;
    ctx.strokeStyle = pal.grid;
    ctx.lineWidth = 1;
    ctx.beginPath();
    for (let gx = Math.floor(minX / step) * step; gx <= maxX; gx += step) {
        const a = toScreen(st.view, s, gx, minY);
        const b = toScreen(st.view, s, gx, maxY);
        ctx.moveTo(a[0], a[1]);
        ctx.lineTo(b[0], b[1]);
    }
    for (let gy = Math.floor(minY / step) * step; gy <= maxY; gy += step) {
        const a = toScreen(st.view, s, minX, gy);
        const b = toScreen(st.view, s, maxX, gy);
        ctx.moveTo(a[0], a[1]);
        ctx.lineTo(b[0], b[1]);
    }
    ctx.stroke();
}

function drawPath(ctx: CanvasRenderingContext2D, st: RenderState, s: Screen, pal: Palette) {
    const pts = st.map.path.map(([x, y]) => toScreen(st.view, s, x, y));
    if (pts.length < 2) return;
    ctx.lineJoin = "round";
    ctx.lineCap = "round";
    // Coverage band: the robot's ~30 cm cleaning width
    ctx.beginPath();
    pts.forEach(([x, y], i) => (i === 0 ? ctx.moveTo(x, y) : ctx.lineTo(x, y)));
    ctx.strokeStyle = pal.coverage;
    ctx.lineWidth = Math.max(2, 0.3 * st.view.scale);
    ctx.stroke();
    if (!st.showPath) return;
    ctx.strokeStyle = pal.path;
    ctx.lineWidth = 1;
    ctx.stroke();
}

// Walls from the LIDAR map, rasterized once per field into an offscreen image
const wallImages = new WeakMap<LocMap, Map<string, HTMLCanvasElement>>();

function wallImage(loc: LocMap, color: string): HTMLCanvasElement {
    let byColor = wallImages.get(loc);
    if (!byColor) {
        byColor = new Map();
        wallImages.set(loc, byColor);
    }
    const cached = byColor.get(color);
    if (cached) return cached;
    const img = document.createElement("canvas");
    img.width = loc.w;
    img.height = loc.h;
    const ctx = img.getContext("2d");
    if (ctx) {
        const data = ctx.createImageData(loc.w, loc.h);
        const m = color.match(/^#([0-9a-f]{6})$/i);
        const n = m ? Number.parseInt(m[1], 16) : 0xffffff;
        for (let cy = 0; cy < loc.h; cy++) {
            for (let cx = 0; cx < loc.w; cx++) {
                const v = loc.field[cy * loc.w + cx];
                if (v < 200) continue;
                const o = ((loc.h - 1 - cy) * loc.w + cx) * 4; // image rows run downwards = world y decreasing
                data.data[o] = (n >> 16) & 255;
                data.data[o + 1] = (n >> 8) & 255;
                data.data[o + 2] = n & 255;
                data.data[o + 3] = v >= 245 ? 235 : 90;
            }
        }
        ctx.putImageData(data, 0, 0);
    }
    byColor.set(color, img);
    return img;
}

function drawWalls(ctx: CanvasRenderingContext2D, st: RenderState, s: Screen, pal: Palette, dpr: number) {
    const loc = st.loc;
    if (!loc) return;
    const img = wallImage(loc, pal.dark ? "#d6d0c4" : "#4a4e4a");
    const r = (st.view.rot * Math.PI) / 180;
    const k = loc.cell / 1000;
    const S = st.view.scale;
    const x0 = loc.originX / 1000 - st.view.cx;
    const y1 = (loc.originY + loc.h * loc.cell) / 1000 - st.view.cy;
    const cos = Math.cos(r);
    const sin = Math.sin(r);
    // Affine image→screen transform equivalent to toScreen()
    const a = S * k * cos;
    const b = S * k * sin;
    const c = -S * k * sin;
    const d = S * k * cos;
    const e = s.w / 2 + S * (cos * x0 + sin * y1);
    const f = s.h / 2 - S * (-sin * x0 + cos * y1);
    ctx.save();
    ctx.setTransform(dpr * a, dpr * b, dpr * c, dpr * d, dpr * e, dpr * f);
    ctx.imageSmoothingEnabled = false;
    ctx.drawImage(img, 0, 0);
    ctx.restore();
}

function hatch(ctx: CanvasRenderingContext2D, pts: MapVertex[], color: string) {
    const xs = pts.map((p) => p[0]);
    const ys = pts.map((p) => p[1]);
    const x0 = Math.min(...xs);
    const x1 = Math.max(...xs);
    const y0 = Math.min(...ys);
    const y1 = Math.max(...ys);
    ctx.save();
    tracePolygon(ctx, pts);
    ctx.clip();
    ctx.strokeStyle = color;
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    for (let d = x0 - (y1 - y0); d < x1; d += 10) {
        ctx.moveTo(d, y1);
        ctx.lineTo(d + (y1 - y0), y0);
    }
    ctx.stroke();
    ctx.restore();
}

function drawLabel(ctx: CanvasRenderingContext2D, text: string, x: number, y: number, pal: Palette, bold: boolean) {
    ctx.font = `${bold ? 600 : 500} 12px -apple-system, BlinkMacSystemFont, system-ui, sans-serif`;
    ctx.textAlign = "center";
    ctx.textBaseline = "middle";
    const w = ctx.measureText(text).width + 12;
    ctx.fillStyle = pal.dark ? "rgba(0, 0, 0, 0.55)" : "rgba(255, 255, 255, 0.8)";
    ctx.beginPath();
    ctx.roundRect(x - w / 2, y - 10, w, 20, 10);
    ctx.fill();
    ctx.fillStyle = pal.text;
    ctx.fillText(text, x, y + 0.5);
}

function drawZone(
    ctx: CanvasRenderingContext2D,
    st: RenderState,
    s: Screen,
    pal: Palette,
    zone: FloorMapZone,
    kind: "room" | "nogo",
) {
    const pts = zone.poly.map(([x, y]) => toScreen(st.view, s, x, y));
    const editing = st.editZone?.kind === kind && st.editZone.id === zone.id;
    if (kind === "room") {
        const color = roomColor(zone.id);
        const picked = st.cleanSelection.has(zone.id);
        tracePolygon(ctx, pts);
        ctx.fillStyle = withAlpha(color, picked ? 0.32 : 0.12);
        ctx.fill();
        ctx.strokeStyle = withAlpha(color, picked || editing ? 0.95 : 0.6);
        ctx.lineWidth = picked || editing ? 2.5 : 1.5;
        ctx.setLineDash([]);
        ctx.stroke();
    } else {
        tracePolygon(ctx, pts);
        ctx.fillStyle = withAlpha(pal.red, 0.12);
        ctx.fill();
        hatch(ctx, pts, withAlpha(pal.red, 0.35));
        tracePolygon(ctx, pts);
        ctx.strokeStyle = withAlpha(pal.red, editing ? 1 : 0.8);
        ctx.lineWidth = editing ? 2.5 : 1.5;
        ctx.setLineDash([6, 4]);
        ctx.stroke();
        ctx.setLineDash([]);
    }
    if (editing) {
        pts.forEach(([x, y], i) => {
            ctx.beginPath();
            ctx.arc(x, y, i === st.activeVertex ? 8 : 6, 0, Math.PI * 2);
            ctx.fillStyle = pal.surface;
            ctx.fill();
            ctx.lineWidth = 2;
            ctx.strokeStyle = kind === "room" ? roomColor(zone.id) : pal.red;
            ctx.stroke();
        });
    }
}

function drawDock(ctx: CanvasRenderingContext2D, st: RenderState, s: Screen, pal: Palette) {
    // Dock sits behind the robot's start pose (robot docks rear-first, faces +x)
    const back = toScreen(st.view, s, -0.2, 0);
    const [ox, oy] = toScreen(st.view, s, 0, 0);
    const ang = Math.atan2(oy - back[1], ox - back[0]);
    ctx.save();
    ctx.translate(back[0], back[1]);
    ctx.rotate(ang);
    ctx.fillStyle = pal.accent;
    ctx.beginPath();
    ctx.roundRect(-4, -10, 8, 20, 3);
    ctx.fill();
    ctx.restore();
    ctx.beginPath();
    ctx.arc(ox, oy, 4, 0, Math.PI * 2);
    ctx.fillStyle = withAlpha(pal.green, 0.9);
    ctx.fill();
}

function drawRobot(ctx: CanvasRenderingContext2D, st: RenderState, s: Screen, pal: Palette) {
    if (st.trail.length > 1) {
        ctx.beginPath();
        st.trail.forEach(([x, y], i) => {
            const [px, py] = toScreen(st.view, s, x, y);
            if (i === 0) ctx.moveTo(px, py);
            else ctx.lineTo(px, py);
        });
        ctx.strokeStyle = withAlpha(pal.blue, 0.7);
        ctx.lineWidth = 2;
        ctx.stroke();
    }
    if (st.target) {
        const [tx, ty] = toScreen(st.view, s, st.target[0], st.target[1]);
        ctx.beginPath();
        ctx.arc(tx, ty, 5, 0, Math.PI * 2);
        ctx.strokeStyle = withAlpha(pal.blue, 0.9);
        ctx.lineWidth = 1.5;
        ctx.stroke();
    }
    if (!st.robot) return;
    const [rx, ry] = toScreen(st.view, s, st.robot.x, st.robot.y);
    const [hx, hy] = toScreen(
        st.view,
        s,
        st.robot.x + Math.cos((st.robot.t * Math.PI) / 180),
        st.robot.y + Math.sin((st.robot.t * Math.PI) / 180),
    );
    const radius = Math.max(7, 0.165 * st.view.scale);
    ctx.save();
    ctx.translate(rx, ry);
    ctx.rotate(Math.atan2(hy - ry, hx - rx));
    ctx.shadowColor = withAlpha(pal.blue, 0.6);
    ctx.shadowBlur = 10;
    ctx.beginPath();
    ctx.arc(0, 0, radius, 0, Math.PI * 2);
    ctx.fillStyle = withAlpha(pal.blue, 0.95);
    ctx.fill();
    ctx.shadowBlur = 0;
    ctx.beginPath();
    ctx.moveTo(radius + 5, 0);
    ctx.lineTo(radius * 0.4, -radius * 0.55);
    ctx.lineTo(radius * 0.4, radius * 0.55);
    ctx.closePath();
    ctx.fillStyle = "rgba(255, 255, 255, 0.9)";
    ctx.fill();
    ctx.restore();
}

// Last guided run: driven path (odometry estimate, solid while cleaning), each
// localization fix as a tick from the odometry pose to the matched pose (green =
// accepted, red = rejected) and obstacles as crosses.
function drawTrace(ctx: CanvasRenderingContext2D, st: RenderState, s: Screen, pal: Palette) {
    const tr = st.trace;
    if (!tr) return;
    const amber = "#e5972a";
    for (let i = 1; i < tr.poses.length; i++) {
        const a = tr.poses[i - 1];
        const b = tr.poses[i];
        if (Math.hypot(b.x - a.x, b.y - a.y) > 0.8) continue; // Fix jump — do not draw a long line
        const [ax, ay] = toScreen(st.view, s, a.x, a.y);
        const [bx, by] = toScreen(st.view, s, b.x, b.y);
        ctx.beginPath();
        ctx.moveTo(ax, ay);
        ctx.lineTo(bx, by);
        ctx.strokeStyle = b.cleaning ? withAlpha(amber, 0.85) : withAlpha(amber, 0.4);
        ctx.lineWidth = b.cleaning ? 2.5 : 1.5;
        ctx.setLineDash(b.cleaning ? [] : [4, 3]);
        ctx.stroke();
    }
    ctx.setLineDash([]);
    for (const f of tr.fixes) {
        const [ox, oy] = toScreen(st.view, s, f.x, f.y);
        const [mx, my] = toScreen(st.view, s, f.mx, f.my);
        const color = f.ok ? pal.green : pal.red;
        ctx.beginPath();
        ctx.moveTo(ox, oy);
        ctx.lineTo(mx, my);
        ctx.strokeStyle = withAlpha(color, 0.9);
        ctx.lineWidth = 1.5;
        ctx.stroke();
        ctx.beginPath();
        ctx.arc(mx, my, 2.5, 0, Math.PI * 2);
        ctx.fillStyle = color;
        ctx.fill();
    }
    for (const ev of tr.events) {
        if (ev.e !== "obstacle") continue;
        const [px, py] = toScreen(st.view, s, ev.x, ev.y);
        ctx.beginPath();
        ctx.moveTo(px - 5, py - 5);
        ctx.lineTo(px + 5, py + 5);
        ctx.moveTo(px + 5, py - 5);
        ctx.lineTo(px - 5, py + 5);
        ctx.strokeStyle = pal.red;
        ctx.lineWidth = 2;
        ctx.stroke();
    }
}

export function renderFloorMap(canvas: HTMLCanvasElement, st: RenderState, labels: (zone: FloorMapZone) => string) {
    const ctx = canvas.getContext("2d");
    if (!ctx) return;
    const dpr = window.devicePixelRatio || 1;
    const s: Screen = { w: canvas.clientWidth, h: canvas.clientHeight };
    if (canvas.width !== Math.round(s.w * dpr) || canvas.height !== Math.round(s.h * dpr)) {
        canvas.width = Math.round(s.w * dpr);
        canvas.height = Math.round(s.h * dpr);
    }
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    const pal = palette(canvas);
    ctx.fillStyle = pal.surface;
    ctx.fillRect(0, 0, s.w, s.h);

    drawGrid(ctx, st, s, pal);
    drawPath(ctx, st, s, pal);
    drawWalls(ctx, st, s, pal, dpr);
    for (const r of st.map.rooms) drawZone(ctx, st, s, pal, r, "room");
    for (const n of st.map.nogos) drawZone(ctx, st, s, pal, n, "nogo");

    if (st.draft) {
        const [ax, ay] = st.draft.a;
        const [bx, by] = st.draft.b;
        const pts: MapVertex[] = [
            [ax, ay],
            [bx, ay],
            [bx, by],
            [ax, by],
        ].map(([x, y]) => toScreen(st.view, s, x, y));
        tracePolygon(ctx, pts);
        const color = st.draft.kind === "room" ? pal.blue : pal.red;
        ctx.fillStyle = withAlpha(color, 0.18);
        ctx.fill();
        ctx.strokeStyle = color;
        ctx.lineWidth = 2;
        ctx.setLineDash([5, 4]);
        ctx.stroke();
        ctx.setLineDash([]);
    }

    drawTrace(ctx, st, s, pal);
    drawDock(ctx, st, s, pal);
    drawRobot(ctx, st, s, pal);

    for (const r of st.map.rooms) {
        const [cx, cy] = polygonCentroid(r.poly);
        const [lx, ly] = toScreen(st.view, s, cx, cy);
        drawLabel(ctx, labels(r), lx, ly, pal, st.cleanSelection.has(r.id));
    }
}

// -- Hit testing ------------------------------------------------------------------

export function hitVertex(view: MapView, s: Screen, zone: FloorMapZone, sx: number, sy: number): number | null {
    let best: number | null = null;
    let bestD = 18;
    zone.poly.forEach(([x, y], i) => {
        const [px, py] = toScreen(view, s, x, y);
        const d = Math.hypot(px - sx, py - sy);
        if (d < bestD) {
            bestD = d;
            best = i;
        }
    });
    return best;
}

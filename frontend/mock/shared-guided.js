// Mock floor map + guided (room) clean. Shared by the Vite dev server and the
// Cloudflare demo Worker. The guided run is simulated from wall-clock time so
// it needs no timers: the robot follows the pinned path inside the selected
// rooms (demo speed), skips no-go zones, then retraces its way to the dock.

const DEMO_SESSION = "mapdata-house-05.jsonl";
const NOGO_MARGIN = 0.28;
const SPEED_CLEAN = 0.55; // m/s (sped up for the demo)
const SPEED_TRANSIT = 0.9;
const START_S = 3;
const DOCK_S = 4;

const DEMO_ROOMS = [
    { id: 1, name: "Living room", poly: "-200,-2300 4600,-2300 4600,2400 -200,2400" },
    { id: 2, name: "Kitchen", poly: "4600,400 7800,400 7800,2400 4600,2400" },
    { id: 3, name: "Bedroom", poly: "4600,-3600 7800,-3600 7800,-600 4600,-600" },
];
const DEMO_NOGOS = [{ id: 1, poly: "2700,1300 3700,1300 3700,2300 2700,2300" }];

const mm = (v) => Math.round(v * 1000);

function buildDemoFloorMap(historySessions) {
    const lines = historySessions.get(DEMO_SESSION);
    if (!lines) return null;
    const path = [];
    for (const line of lines) {
        if (!line.includes('"x":')) continue;
        const p = JSON.parse(line);
        if (p.x === 0 && p.y === 0) continue;
        const prev = path[path.length - 1];
        if (!prev || Math.hypot(p.x - prev[0], p.y - prev[1]) >= 0.1) path.push([p.x, p.y]);
    }
    const out = ["OFM1", `S ${DEMO_SESSION}`];
    for (let i = 0; i < path.length; i += 60) {
        out.push(
            `P ${path
                .slice(i, i + 60)
                .map(([x, y]) => `${mm(x)},${mm(y)}`)
                .join(" ")}`,
        );
    }
    for (const r of DEMO_ROOMS) out.push(`R ${r.id} ${encodeURIComponent(r.name)} ${r.poly}`);
    for (const n of DEMO_NOGOS) out.push(`N ${n.id} ${n.poly}`);
    return `${out.join("\n")}\n`;
}

function parsePts(tokens) {
    return tokens.map((t) => {
        const [x, y] = t.split(",").map(Number);
        return [x / 1000, y / 1000];
    });
}

function parseFloorMap(doc) {
    const map = { path: [], rooms: [], nogos: [] };
    const lines = doc.split("\n").map((l) => l.trim());
    if (lines[0] !== "OFM1") return null;
    for (const line of lines.slice(1)) {
        const tok = line.slice(2).split(" ").filter(Boolean);
        if (line.startsWith("P ")) map.path.push(...parsePts(tok));
        else if (line.startsWith("R ")) map.rooms.push({ id: Number(tok[0]), poly: parsePts(tok.slice(2)) });
        else if (line.startsWith("N ")) map.nogos.push({ id: Number(tok[0]), poly: parsePts(tok.slice(1)) });
    }
    return map;
}

function validateFloorMap(doc) {
    if (doc.length > 65536) return "map too large";
    const map = parseFloorMap(doc);
    if (!map) return "bad header (expected OFM1)";
    if (map.path.length < 2) return "map has no path";
    if (map.path.length > 3000) return "invalid or too many path points";
    if (map.rooms.length > 16 || map.nogos.length > 16) return "too many zones";
    if ([...map.rooms, ...map.nogos].some((z) => z.poly.length < 3 || z.poly.length > 24)) {
        return "invalid zone polygon";
    }
    return null;
}

function inside([x, y], poly) {
    let c = false;
    for (let i = 0, j = poly.length - 1; i < poly.length; j = i++) {
        const [xi, yi] = poly[i];
        const [xj, yj] = poly[j];
        if (yi > y !== yj > y && x < ((xj - xi) * (y - yi)) / (yj - yi) + xi) c = !c;
    }
    return c;
}

function distToPoly(p, poly) {
    let best = Infinity;
    for (let i = 0, j = poly.length - 1; i < poly.length; j = i++) {
        const [ax, ay] = poly[j];
        const [bx, by] = poly[i];
        const dx = bx - ax;
        const dy = by - ay;
        const l2 = dx * dx + dy * dy;
        const t = l2 > 0 ? Math.max(0, Math.min(1, ((p[0] - ax) * dx + (p[1] - ay) * dy) / l2)) : 0;
        best = Math.min(best, Math.hypot(ax + t * dx - p[0], ay + t * dy - p[1]));
    }
    return best;
}

// Route like the firmware: dock → recorded path up to the last point in a
// selected room, minus points near no-go zones; then the way back.
function planRoute(map, roomIds) {
    const rooms = roomIds.length ? map.rooms.filter((r) => roomIds.includes(r.id)) : map.rooms;
    if (rooms.length === 0) return { error: "no rooms defined on the map" };
    const route = [{ p: [0, 0], clean: false }];
    let last = -1;
    for (const p of map.path) {
        if (map.nogos.some((z) => inside(p, z.poly) || distToPoly(p, z.poly) < NOGO_MARGIN)) continue;
        const clean = rooms.some((r) => inside(p, r.poly));
        route.push({ p, clean });
        if (clean) last = route.length - 1;
    }
    if (last < 0) return { error: "the pinned path never enters the selected rooms" };
    // Thin long transit stretches (the firmware shortcuts them over covered floor)
    const kept = route.slice(0, last + 1).filter((r, i, all) => {
        if (r.clean || i === 0 || i === all.length - 1) return true;
        return all[i - 1].clean || all[i + 1].clean || i % 6 === 0;
    });
    return { route: kept };
}

// Timeline segments with start times; `phase` tells which part of the run a segment belongs to
function buildTimeline(route, t0, phaseOut = "running") {
    const segs = [];
    let t = t0;
    for (let i = 1; i < route.length; i++) {
        const a = route[i - 1].p;
        const b = route[i].p;
        const len = Math.hypot(b[0] - a[0], b[1] - a[1]);
        const clean = phaseOut === "running" && route[i].clean;
        const dur = len / (clean ? SPEED_CLEAN : SPEED_TRANSIT);
        segs.push({ a, b, t0: t, t1: t + dur, clean, phase: phaseOut, idx: i });
        t += dur;
    }
    return { segs, end: t };
}

function returnRoute(route, idx, pos) {
    const back = [{ p: pos, clean: false }];
    for (let i = idx; i >= 0; i--) back.push({ p: route[i].p, clean: false });
    back[back.length - 1] = { p: [0.45, 0], clean: false };
    back.push({ p: [0, 0], clean: false });
    return back;
}

function startGuided(state, doc, roomsParam, now) {
    const map = parseFloorMap(doc);
    const roomIds = (roomsParam || "")
        .split(",")
        .map(Number)
        .filter((n) => n > 0);
    const plan = planRoute(map, roomIds);
    if (plan.error) return plan.error;
    const tStart = START_S;
    const run = buildTimeline(plan.route, tStart);
    const lastIdx = plan.route.length - 1;
    const back = buildTimeline(returnRoute(plan.route, lastIdx, plan.route[lastIdx].p), run.end, "returning");
    state.guided = {
        startMs: now,
        startedAt: Math.floor(now / 1000),
        rooms: roomsParam || "",
        route: plan.route,
        segs: [...run.segs, ...back.segs],
        runEnd: run.end,
        returnEnd: back.end,
        cleanTotal: plan.route.filter((r) => r.clean).length,
        message: "",
        final: null,
    };
    return null;
}

function poseAt(g, t) {
    let seg = g.segs[0];
    for (const s of g.segs) {
        if (t >= s.t0) seg = s;
        else break;
    }
    if (!seg) return { x: 0, y: 0, t: 0, seg: null };
    const f = seg.t1 > seg.t0 ? Math.min(1, Math.max(0, (t - seg.t0) / (seg.t1 - seg.t0))) : 1;
    const x = seg.a[0] + (seg.b[0] - seg.a[0]) * f;
    const y = seg.a[1] + (seg.b[1] - seg.a[1]) * f;
    const heading = (Math.atan2(seg.b[1] - seg.a[1], seg.b[0] - seg.a[0]) * 180) / Math.PI;
    return { x, y, t: seg.phase === "returning" && t >= g.returnEnd ? 0 : heading, seg };
}

function guidedStatus(state, hasMap, now) {
    const g = state.guided;
    const base = {
        phase: "idle",
        active: false,
        hasMap,
        locMap: state.floorField ? "ready" : "none",
        rooms: "",
        x: 0,
        y: 0,
        t: 0,
        progress: 0,
        cleanDone: 0,
        cleanTotal: 0,
        obstacles: 0,
        cleaning: false,
        elapsed: 0,
        startedAt: 0,
        message: "",
        compass: "off",
        compassFixes: 0,
        localization: state.floorField ? "map" : "odometry",
        locFixes: 0,
        locScore: 0,
        locSpread: 0,
        waiting: "",
    };
    if (!g) return base;
    if (g.final) return { ...base, ...g.final, hasMap };
    const t = (now - g.startMs) / 1000;
    const pose = poseAt(g, t);
    let phase = "running";
    if (t < START_S / 2) phase = "starting";
    else if (t < START_S) phase = "localizing";
    else if (t >= g.returnEnd + DOCK_S) phase = "done";
    else if (t >= g.returnEnd) phase = "docking";
    else if (pose.seg?.phase === "returning") phase = "returning";
    const doneIdx = pose.seg?.phase === "running" ? pose.seg.idx : g.route.length;
    const cleanDone = g.route.slice(0, doneIdx).filter((r) => r.clean).length;
    const status = {
        ...base,
        phase,
        active: phase !== "done",
        rooms: g.rooms,
        x: Number(pose.x.toFixed(3)),
        y: Number(pose.y.toFixed(3)),
        t: Number(pose.t.toFixed(1)),
        progress: phase === "done" ? 100 : Math.round((cleanDone * 100) / Math.max(1, g.cleanTotal)),
        cleanDone,
        cleanTotal: g.cleanTotal,
        obstacles: Math.floor(t / 47) % 4,
        cleaning: phase === "running" && (pose.seg?.clean ?? false),
        elapsed: Math.floor(t),
        startedAt: g.startedAt,
        message: g.message,
        locFixes: Math.floor(t / 4),
        locScore: 0.9,
        locSpread: 45,
        waiting: "scan",
    };
    if (pose.seg && status.active) {
        status.targetX = pose.seg.b[0];
        status.targetY = pose.seg.b[1];
    }
    if (phase === "done") g.final = { ...status, active: false };
    return status;
}

function guidedHome(state, now) {
    const g = state.guided;
    if (!g || g.final) return "guided clean is not running";
    const t = (now - g.startMs) / 1000;
    const pose = poseAt(g, t);
    if (pose.seg?.phase !== "running") return null; // already returning
    const idx = Math.max(0, pose.seg.idx - 1);
    const kept = g.segs.filter((s) => s.phase === "running" && s.t1 <= t);
    const back = buildTimeline(returnRoute(g.route, idx, [pose.x, pose.y]), t, "returning");
    g.segs = [...kept, ...back.segs];
    g.returnEnd = back.end;
    g.message = "returned on request";
    return null;
}

function guidedStop(state, now) {
    const g = state.guided;
    if (!g || g.final) return "guided clean is not running";
    const s = guidedStatus(state, true, now);
    g.final = { ...s, phase: "stopped", active: false, cleaning: false, message: "stopped by user" };
    return null;
}

function isGuidedActive(state, now) {
    return !!state.guided && !state.guided.final && guidedStatus(state, true, now).active;
}

export { buildDemoFloorMap, guidedHome, guidedStatus, guidedStop, isGuidedActive, startGuided, validateFloorMap };

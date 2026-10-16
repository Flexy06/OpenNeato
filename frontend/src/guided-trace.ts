// Trace of the last guided clean, recorded by the firmware in /map/trace.jsonl
// (see GuidedCleanManager::tracePose). One JSON object per line:
//   {"e": event, "ms": since start, "x","y" (mm), "t" (deg), ...extra}
// Pose lines ("p") carry "c" (cleaning motors on). Localization fixes ("loc")
// carry the matched pose mx/my/mt, its score s, the score s0 at the odometry
// pose and whether the fix was accepted.

export interface TracePose {
    x: number; // metres
    y: number;
    t: number; // degrees
    ms: number;
    cleaning: boolean;
}

export interface TraceFix {
    x: number; // odometry pose before the fix (metres)
    y: number;
    mx: number; // matched pose (metres)
    my: number;
    ms: number;
    score: number;
    ok: boolean;
    shift: number; // mm between odometry and matched pose
}

export interface TraceEvent {
    e: string;
    ms: number;
    x: number;
    y: number;
    fields: Record<string, unknown>;
}

export interface TraceSummary {
    rooms: string;
    withField: boolean;
    result: string; // done / failed / stopped / running (no end yet)
    reason: string;
    durationS: number;
    distanceM: number;
    fixesOk: number;
    fixesRejected: number;
    avgScore: number | null;
    maxShiftMm: number;
    obstacles: number;
    cleaned: number;
    total: number;
}

export interface GuidedTrace {
    poses: TracePose[];
    fixes: TraceFix[];
    events: TraceEvent[];
    summary: TraceSummary;
}

const END_EVENTS = new Set(["done", "failed", "stopped"]);

function num(v: unknown, fallback = 0): number {
    return typeof v === "number" && Number.isFinite(v) ? v : fallback;
}

export function parseGuidedTrace(text: string): GuidedTrace | null {
    const poses: TracePose[] = [];
    const fixes: TraceFix[] = [];
    const events: TraceEvent[] = [];
    let start: Record<string, unknown> | null = null;
    let end: Record<string, unknown> | null = null;
    let lastMs = 0;
    for (const line of text.split("\n")) {
        if (!line.trim()) continue;
        let o: Record<string, unknown>;
        try {
            o = JSON.parse(line) as Record<string, unknown>;
        } catch {
            continue; // Truncated last line (power loss) — skip
        }
        const e = String(o.e ?? "");
        const ms = num(o.ms);
        const x = num(o.x) / 1000;
        const y = num(o.y) / 1000;
        lastMs = Math.max(lastMs, ms);
        if (e === "p") {
            poses.push({ x, y, t: num(o.t), ms, cleaning: o.c === true });
        } else if (e === "loc") {
            const mx = num(o.mx) / 1000;
            const my = num(o.my) / 1000;
            fixes.push({
                x,
                y,
                mx,
                my,
                ms,
                score: num(o.s),
                ok: o.ok === true,
                shift: Math.hypot(mx - x, my - y) * 1000,
            });
        } else {
            if (e === "start") start = o;
            if (END_EVENTS.has(e)) end = o;
            const { e: _e, ms: _ms, x: _x, y: _y, t: _t, ...fields } = o;
            events.push({ e, ms, x, y, fields });
        }
    }
    if (!start && poses.length === 0) return null;

    let distance = 0;
    for (let i = 1; i < poses.length; i++) {
        distance += Math.hypot(poses[i].x - poses[i - 1].x, poses[i].y - poses[i - 1].y);
    }
    const ok = fixes.filter((f) => f.ok);
    return {
        poses,
        fixes,
        events,
        summary: {
            rooms: String(start?.rooms ?? ""),
            withField: start?.field === true,
            result: end ? String(end.e) : "running",
            reason: String(end?.reason ?? ""),
            durationS: Math.round(lastMs / 1000),
            distanceM: distance,
            fixesOk: ok.length,
            fixesRejected: fixes.length - ok.length,
            avgScore: ok.length ? ok.reduce((a, f) => a + f.score, 0) / ok.length : null,
            maxShiftMm: ok.reduce((a, f) => Math.max(a, f.shift), 0),
            obstacles: events.filter((ev) => ev.e === "obstacle").length,
            cleaned: num(end?.cleaned),
            total: num(end?.total ?? start?.clean),
        },
    };
}

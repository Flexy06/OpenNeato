// Diagnostics bundle: one JSON file with everything needed to analyse a
// connection or cleaning problem (identification + serial link state, system,
// robot state, guided status + trace of the last room clean, floor map, recent
// logs). Settings are left out on purpose (they hold WiFi / notification secrets).

const TIMEOUT_MS = 8000;
const MAX_LOG_BYTES = 200_000;

async function grab(url: string, asText = false): Promise<unknown> {
    const ctrl = new AbortController();
    const timer = setTimeout(() => ctrl.abort(), TIMEOUT_MS);
    try {
        const res = await fetch(url, { signal: ctrl.signal });
        const body = await res.text();
        if (!res.ok) return { error: `${res.status}`, body: body.slice(0, 500) };
        if (asText) return body;
        try {
            return JSON.parse(body);
        } catch {
            return body;
        }
    } catch (e) {
        return { error: e instanceof Error ? e.message : String(e) };
    } finally {
        clearTimeout(timer);
    }
}

export async function collectDiagnostics(): Promise<Record<string, unknown>> {
    const out: Record<string, unknown> = {
        generatedAt: new Date().toISOString(),
        page: location.href,
        userAgent: navigator.userAgent,
    };
    // Sequential on purpose: the ESP32 serves requests one at a time anyway, and
    // a burst would queue serial commands behind each other.
    const json: [string, string][] = [
        ["diag", "/api/diag"],
        ["firmware", "/api/firmware/version"],
        ["system", "/api/system"],
        ["state", "/api/state"],
        ["error", "/api/error"],
        ["charger", "/api/charger"],
        ["robotVersion", "/api/version"],
        ["guidedStatus", "/api/guided/status"],
        ["history", "/api/history"],
        ["logs", "/api/logs"],
    ];
    for (const [key, url] of json) out[key] = await grab(url);
    out.guidedTrace = await grab("/api/guided/trace", true);
    out.floorMap = await grab("/api/floormap", true);

    // Newest log file (names sort chronologically)
    const logs = out.logs;
    if (Array.isArray(logs) && logs.length > 0) {
        const names = logs.map((l: { name?: string }) => String(l.name ?? "")).sort();
        const newest = names[names.length - 1];
        const text = await grab(`/api/logs/${newest}`, true);
        out.newestLog = {
            name: newest,
            text: typeof text === "string" ? text.slice(-MAX_LOG_BYTES) : text,
        };
    }
    out.diagAfter = await grab("/api/diag");
    return out;
}

export async function downloadDiagnostics(): Promise<void> {
    const data = await collectDiagnostics();
    const blob = new Blob([JSON.stringify(data, null, 2)], { type: "application/json" });
    const url = URL.createObjectURL(blob);
    const a = document.createElement("a");
    const stamp = new Date().toISOString().replace(/[:.]/g, "-").slice(0, 19);
    a.href = url;
    a.download = `openneato-diagnose-${stamp}.json`;
    document.body.appendChild(a);
    a.click();
    a.remove();
    setTimeout(() => URL.revokeObjectURL(url), 5000);
}

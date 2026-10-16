// Shown on the dashboard while the robot is not identified yet: explains what
// the ESP32 sees on the serial link instead of a bare "Connecting to robot...",
// and offers a retry and a diagnostics download.
import { useEffect, useState } from "preact/hooks";
import { api } from "../api";
import { downloadDiagnostics } from "../diagnostics";
import { usePolling } from "../hooks/use-polling";
import { T, useI18n } from "../i18n";
import type { DiagData } from "../types";

// Seconds without identification before the details open by themselves
const AUTO_OPEN_S = 20;

function hint(d: DiagData, t: ReturnType<typeof useI18n>["t"]): string {
    const s = d.serial;
    if (s.rxBytes === 0 && d.identify.attempts > 0) {
        return t(
            "No data has arrived from the robot yet (0 bytes received). Check that the robot is switched on (press a button to wake it), the cable to the debug port and the UART pins (TX/RX swapped?).",
        );
    }
    if (d.identify.lastError) return t("Last attempt: {error}", { error: d.identify.lastError });
    if (d.identify.inFlight) return t("Waiting for the robot's answer...");
    return "";
}

export function ConnectionDiag() {
    const { t } = useI18n();
    const diag = usePolling<DiagData>(api.getDiag, 2000);
    const [open, setOpen] = useState(false);
    const [busy, setBusy] = useState(false);
    const [msg, setMsg] = useState<string | null>(null);
    const d = diag.data;

    useEffect(() => {
        if (d && d.esp.uptimeS > AUTO_OPEN_S && d.identify.pending) setOpen(true);
    }, [d]);

    const retry = async () => {
        setMsg(null);
        try {
            await api.reidentify();
            setMsg(t("Identification restarted."));
        } catch (e) {
            setMsg(e instanceof Error ? e.message : String(e));
        }
    };

    const download = async () => {
        setBusy(true);
        try {
            await downloadDiagnostics();
        } finally {
            setBusy(false);
        }
    };

    return (
        <div class="conn-diag">
            <p class="gate-message">
                <T>Connecting to robot...</T>
            </p>
            {diag.error && !d && (
                <p class="conn-diag-hint">
                    <T>The ESP32 does not answer diagnostics requests.</T> ({diag.error})
                </p>
            )}
            {d && (
                <>
                    <p class="conn-diag-hint">
                        {t("Attempt {n}", { n: d.identify.attempts })}
                        {hint(d, t) && ` · ${hint(d, t)}`}
                    </p>
                    <div class="conn-diag-actions">
                        <button type="button" class="floormap-btn" onClick={retry}>
                            <T>Retry now</T>
                        </button>
                        <button type="button" class="floormap-btn" onClick={download} disabled={busy}>
                            {busy ? t("Collecting...") : t("Download diagnostics")}
                        </button>
                        <button type="button" class="floormap-btn" onClick={() => setOpen(!open)}>
                            {open ? t("Hide details") : t("Details")}
                        </button>
                    </div>
                    {msg && <p class="conn-diag-hint">{msg}</p>}
                    {open && (
                        <dl class="conn-diag-details">
                            <dt>
                                <T>ESP32 uptime</T>
                            </dt>
                            <dd>{t("{s} s", { s: d.esp.uptimeS })}</dd>
                            <dt>
                                <T>Bytes from robot</T>
                            </dt>
                            <dd>
                                {d.serial.rxBytes}
                                {d.serial.lastRxAgoS >= 0 && ` (${t("{s} s ago", { s: d.serial.lastRxAgoS })})`}
                            </dd>
                            <dt>
                                <T>Commands ok / timeout / desync</T>
                            </dt>
                            <dd>
                                {d.serial.ok} / {d.serial.timeouts} / {d.serial.desyncs}
                            </dd>
                            <dt>
                                <T>Serial queue</T>
                            </dt>
                            <dd>
                                {d.serial.queueDepth} · {d.serial.state}
                                {d.serial.current && ` · ${d.serial.current}`}
                            </dd>
                            {d.serial.lastFailCmd && (
                                <>
                                    <dt>
                                        <T>Last failed command</T>
                                    </dt>
                                    <dd>
                                        {d.serial.lastFailCmd} ({d.serial.lastFailStatus},{" "}
                                        {t("{s} s ago", { s: d.serial.lastFailAgoS })})
                                    </dd>
                                </>
                            )}
                            {d.identify.lastRaw && (
                                <>
                                    <dt>
                                        <T>Last answer to GetVersion</T>
                                    </dt>
                                    <dd>
                                        <pre>{d.identify.lastRaw}</pre>
                                    </dd>
                                </>
                            )}
                        </dl>
                    )}
                </>
            )}
        </div>
    );
}

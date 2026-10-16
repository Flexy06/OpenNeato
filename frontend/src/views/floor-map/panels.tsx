import { useEffect, useState } from "preact/hooks";
import { api } from "../../api";
import { T, useI18n } from "../../i18n";
import type { FloorMap, FloorMapZone } from "../../floor-map";
import { MAX_ZONES, roomColor } from "../../floor-map";
import type { GuidedStatus, HistoryFileInfo } from "../../types";
import { normalizeError } from "../../utils";

// -- Session picker (pin a recorded cleaning run as the floor map) ------------

interface SessionPickerProps {
    current: string | null;
    busy: boolean;
    onPick: (file: HistoryFileInfo) => void;
    onError: (msg: string) => void;
}

export function SessionPicker({ current, busy, onPick, onError }: SessionPickerProps) {
    const { formatDateTime, formatDuration, formatNumber } = useI18n();
    const [files, setFiles] = useState<HistoryFileInfo[] | null>(null);

    useEffect(() => {
        api.getHistoryList()
            .then((list) =>
                setFiles(
                    list
                        .filter((f) => !f.recording && f.session?.mode === "house" && (f.summary?.snapshots ?? 0) >= 20)
                        .sort((a, b) => (b.session?.time ?? 0) - (a.session?.time ?? 0)),
                ),
            )
            .catch((e: unknown) => {
                setFiles([]);
                onError(normalizeError(e, "Failed to load cleaning history"));
            });
    }, [onError]);

    if (files === null) {
        return (
            <div class="history-empty">
                <T>Loading...</T>
            </div>
        );
    }
    if (files.length === 0) {
        return (
            <div class="floormap-note">
                <T>No finished house clean in the history yet. Run a normal house clean first, then pin it here.</T>
            </div>
        );
    }
    return (
        <div class="floormap-sessions">
            {files.map((f) => (
                <button
                    type="button"
                    key={f.name}
                    class={`floormap-session${current === f.name ? " current" : ""}${busy ? " pending" : ""}`}
                    disabled={busy}
                    onClick={() => onPick(f)}
                >
                    <span class="floormap-session-date">
                        {f.session ? formatDateTime(f.session.time) : f.name}
                        {f.session?.scans && (
                            <span class="floormap-badge">
                                <T>LIDAR map</T>
                            </span>
                        )}
                    </span>
                    <span class="floormap-session-stats">
                        {f.summary ? formatDuration(f.summary.duration) : ""}
                        {f.summary ? ` · ${formatNumber(f.summary.areaCovered, { maximumFractionDigits: 1 })} m²` : ""}
                    </span>
                </button>
            ))}
        </div>
    );
}

// -- Mapping run (house clean that records LIDAR scans) -------------------------

interface MappingCardProps {
    started: boolean;
    robotIdle: boolean;
    onStart: () => void;
}

export function MappingCard({ started, robotIdle, onStart }: MappingCardProps) {
    return (
        <div class="floormap-panel floormap-mapping">
            <div class="floormap-panel-title">
                <T>Record a LIDAR map</T>
            </div>
            <div class="floormap-note">
                <T>
                    For reliable room cleaning the robot needs a wall map: start a mapping run from the dock. It is a
                    normal house clean that also records LIDAR scans. When it is back on the dock, pick it below.
                </T>
            </div>
            {started ? (
                <div class="floormap-note small">
                    <T>Mapping run started. Come back when the robot has returned to the dock.</T>
                </div>
            ) : (
                <button type="button" class="floormap-btn primary wide" onClick={onStart} disabled={!robotIdle}>
                    <T>Start mapping run</T>
                </button>
            )}
        </div>
    );
}

// -- Clean panel (pick rooms, start/stop, live progress) -----------------------

interface CleanPanelProps {
    map: FloorMap;
    selection: Set<number>;
    status: GuidedStatus | null;
    canStart: boolean;
    startHint: string | null;
    pending: boolean;
    onToggle: (id: number) => void;
    onSelectAll: (all: boolean) => void;
    onStart: () => void;
    onHome: () => void;
    onStop: () => void;
}

const PHASE_LABELS: Record<string, string> = {
    idle: "Ready",
    starting: "Starting...",
    running: "Cleaning rooms",
    returning: "Returning to dock",
    docking: "Docking",
    finishing: "Finishing...",
    done: "Finished",
    stopped: "Stopped",
    failed: "Failed",
};

export function CleanPanel({
    map,
    selection,
    status,
    canStart,
    startHint,
    pending,
    onToggle,
    onSelectAll,
    onStart,
    onHome,
    onStop,
}: CleanPanelProps) {
    const { t, formatDuration } = useI18n();
    const active = status?.active ?? false;
    const showStatus = status && status.phase !== "idle";
    const allSelected = map.rooms.length > 0 && map.rooms.every((r) => selection.has(r.id));

    return (
        <div class="floormap-panel">
            {map.rooms.length === 0 ? (
                <div class="floormap-note">
                    <T>No rooms yet. Switch to Edit and draw your rooms on the map.</T>
                </div>
            ) : (
                <>
                    <div class="floormap-panel-head">
                        <span class="floormap-panel-title">
                            <T>Rooms</T>
                        </span>
                        <button type="button" class="floormap-link-btn" onClick={() => onSelectAll(!allSelected)}>
                            {t(allSelected ? "Select none" : "Select all")}
                        </button>
                    </div>
                    <div class="floormap-chips">
                        {map.rooms.map((r) => (
                            <button
                                type="button"
                                key={r.id}
                                class={`floormap-chip${selection.has(r.id) ? " on" : ""}`}
                                style={{ "--chip-color": roomColor(r.id) }}
                                onClick={() => onToggle(r.id)}
                                disabled={active}
                                aria-pressed={selection.has(r.id)}
                            >
                                <span class="floormap-chip-dot" />
                                {r.name}
                            </button>
                        ))}
                    </div>
                </>
            )}

            {showStatus && (
                <div class={`floormap-status ${status.phase}`}>
                    <div class="floormap-status-row">
                        <span class="floormap-status-phase">{t(PHASE_LABELS[status.phase] ?? status.phase)}</span>
                        <span class="floormap-status-meta">
                            {formatDuration(status.elapsed)}
                            {status.obstacles > 0 && ` · ${t("{count} obstacles", { count: status.obstacles })}`}
                            {status.localization === "map" &&
                                ` · ${t("Map lock {score}%", { score: Math.round(status.locScore * 100) })}`}
                        </span>
                    </div>
                    {(active || status.phase === "done") && (
                        <div class="floormap-progress">
                            <div class="floormap-progress-bar" style={{ width: `${status.progress}%` }} />
                        </div>
                    )}
                    {status.message && <div class="floormap-status-msg">{t(status.message)}</div>}
                </div>
            )}

            {active ? (
                <div class="floormap-actions">
                    <button
                        type="button"
                        class={`floormap-btn${pending ? " pending" : ""}`}
                        onClick={onHome}
                        disabled={pending || status?.phase !== "running"}
                    >
                        <T>Return to dock</T>
                    </button>
                    <button
                        type="button"
                        class={`floormap-btn danger${pending ? " pending" : ""}`}
                        onClick={onStop}
                        disabled={pending}
                    >
                        <T>Stop</T>
                    </button>
                </div>
            ) : (
                map.rooms.length > 0 && (
                    <>
                        <button
                            type="button"
                            class={`floormap-btn primary wide${pending ? " pending" : ""}`}
                            onClick={onStart}
                            disabled={!canStart || pending}
                        >
                            {selection.size > 0
                                ? t("Clean {count} selected rooms", { count: selection.size })
                                : t("Select rooms to clean")}
                        </button>
                        {startHint && <div class="floormap-note small">{t(startHint)}</div>}
                    </>
                )
            )}
        </div>
    );
}

// -- Edit panel (draw / reshape / rename zones) --------------------------------

export type EditTarget = { kind: "room" | "nogo"; id: number } | null;

interface EditPanelProps {
    map: FloorMap;
    target: EditTarget;
    armed: "room" | "nogo" | null;
    dirty: boolean;
    saving: boolean;
    onArm: (kind: "room" | "nogo" | null) => void;
    onSelect: (t: EditTarget) => void;
    onRename: () => void;
    onAddCorner: () => void;
    onRemoveCorner: () => void;
    onDelete: () => void;
    onSave: () => void;
    onDiscard: () => void;
    onReplace: () => void;
    onDeleteMap: () => void;
}

function zoneLabel(t: (s: string, v?: Record<string, string | number>) => string, z: FloorMapZone, kind: string) {
    return kind === "room" ? z.name : t("No-go zone {id}", { id: z.id });
}

export function EditPanel(props: EditPanelProps) {
    const { t } = useI18n();
    const { map, target, armed } = props;
    const zone = target
        ? ((target.kind === "room" ? map.rooms : map.nogos).find((z) => z.id === target.id) ?? null)
        : null;

    return (
        <div class="floormap-panel">
            <div class="floormap-actions">
                <button
                    type="button"
                    class={`floormap-btn${armed === "room" ? " armed" : ""}`}
                    onClick={() => props.onArm(armed === "room" ? null : "room")}
                    disabled={map.rooms.length >= MAX_ZONES}
                >
                    <T>+ Room</T>
                </button>
                <button
                    type="button"
                    class={`floormap-btn${armed === "nogo" ? " armed" : ""}`}
                    onClick={() => props.onArm(armed === "nogo" ? null : "nogo")}
                    disabled={map.nogos.length >= MAX_ZONES}
                >
                    <T>+ No-go zone</T>
                </button>
            </div>
            {armed && (
                <div class="floormap-note small">
                    <T>Drag on the map to draw a rectangle. Reshape it afterwards by dragging its corners.</T>
                </div>
            )}

            {zone && target && (
                <div class="floormap-zone-editor">
                    <div class="floormap-panel-head">
                        <span class="floormap-panel-title">{zoneLabel(t, zone, target.kind)}</span>
                        <button type="button" class="floormap-link-btn" onClick={() => props.onSelect(null)}>
                            <T>Done</T>
                        </button>
                    </div>
                    <div class="floormap-actions">
                        {target.kind === "room" && (
                            <button type="button" class="floormap-btn small" onClick={props.onRename}>
                                <T>Rename</T>
                            </button>
                        )}
                        <button type="button" class="floormap-btn small" onClick={props.onAddCorner}>
                            <T>Add corner</T>
                        </button>
                        <button
                            type="button"
                            class="floormap-btn small"
                            onClick={props.onRemoveCorner}
                            disabled={zone.poly.length <= 3}
                        >
                            <T>Remove corner</T>
                        </button>
                        <button type="button" class="floormap-btn small danger" onClick={props.onDelete}>
                            <T>Delete</T>
                        </button>
                    </div>
                    <div class="floormap-note small">
                        <T>Drag a corner to reshape, drag inside to move. Tap elsewhere to deselect.</T>
                    </div>
                </div>
            )}

            {!zone && (map.rooms.length > 0 || map.nogos.length > 0) && (
                <div class="floormap-chips">
                    {map.rooms.map((r) => (
                        <button
                            type="button"
                            key={`r${r.id}`}
                            class="floormap-chip"
                            style={{ "--chip-color": roomColor(r.id) }}
                            onClick={() => props.onSelect({ kind: "room", id: r.id })}
                        >
                            <span class="floormap-chip-dot" />
                            {r.name}
                        </button>
                    ))}
                    {map.nogos.map((n) => (
                        <button
                            type="button"
                            key={`n${n.id}`}
                            class="floormap-chip nogo"
                            onClick={() => props.onSelect({ kind: "nogo", id: n.id })}
                        >
                            <span class="floormap-chip-dot" />
                            {zoneLabel(t, n, "nogo")}
                        </button>
                    ))}
                </div>
            )}

            {props.dirty && (
                <div class="floormap-actions">
                    <button
                        type="button"
                        class={`floormap-btn primary${props.saving ? " pending" : ""}`}
                        onClick={props.onSave}
                        disabled={props.saving}
                    >
                        <T>Save map</T>
                    </button>
                    <button type="button" class="floormap-btn" onClick={props.onDiscard} disabled={props.saving}>
                        <T>Discard</T>
                    </button>
                </div>
            )}

            <div class="floormap-manage">
                <button type="button" class="floormap-link-btn" onClick={props.onReplace}>
                    <T>Use another cleaning run</T>
                </button>
                <button type="button" class="floormap-link-btn danger" onClick={props.onDeleteMap}>
                    <T>Delete floor map</T>
                </button>
            </div>
        </div>
    );
}

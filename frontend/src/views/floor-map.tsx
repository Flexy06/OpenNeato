import { useCallback, useEffect, useMemo, useRef, useState } from "preact/hooks";
import { api } from "../api";
import backSvg from "../assets/icons/back.svg?raw";
import rotateLeftSvg from "../assets/icons/rotate-left.svg?raw";
import rotateRightSvg from "../assets/icons/rotate-right.svg?raw";
import { ConfirmDialog } from "../components/confirm-dialog";
import { ErrorBannerStack, useErrorStack } from "../components/error-banner";
import { Icon } from "../components/icon";
import {
    type FloorMap,
    type FloorMapZone,
    floorMapBounds,
    type MapVertex,
    nextZoneId,
    parseFloorMap,
    pointInPolygon,
    polygonArea,
    rectPolygon,
    serializeFloorMap,
    MAX_VERTICES,
} from "../floor-map";
import { pinSession } from "../floor-map-pin";
import { useDirtyGuard } from "../hooks/use-dirty-guard";
import { type LocMap, parseLocMap } from "../loc-map";
import { usePolling } from "../hooks/use-polling";
import { T, useI18n } from "../i18n";
import type { GuidedStatus, HistoryFileInfo, StateData } from "../types";
import { normalizeError } from "../utils";
import { fitView, hitVertex, type MapView, renderFloorMap, type Screen } from "./floor-map/canvas";
import { CleanPanel, EditPanel, type EditTarget, MappingCard, SessionPicker } from "./floor-map/panels";
import { useEditorGestures } from "./floor-map/use-editor-gestures";

interface FloorMapViewProps {
    state: StateData | null;
    robotReady: boolean;
}

type Tab = "clean" | "edit";

function loadRotation(): number {
    const raw = Number(localStorage.getItem("floorMapRotation"));
    return Number.isFinite(raw) ? (((Math.round(raw / 90) * 90) % 360) + 360) % 360 : 0;
}

function loadSelection(): Set<number> {
    try {
        const raw = JSON.parse(localStorage.getItem("floorMapRooms") ?? "[]");
        return new Set(Array.isArray(raw) ? raw.filter((n) => typeof n === "number") : []);
    } catch {
        return new Set();
    }
}

// Topmost zone under a world point: no-go zones first, then the smallest room
function zoneAt(map: FloorMap, [x, y]: MapVertex, includeNogo: boolean): EditTarget {
    if (includeNogo) {
        const n = map.nogos.find((z) => pointInPolygon(x, y, z.poly));
        if (n) return { kind: "nogo", id: n.id };
    }
    const rooms = map.rooms.filter((z) => pointInPolygon(x, y, z.poly));
    rooms.sort((a, b) => polygonArea(a.poly) - polygonArea(b.poly));
    return rooms.length > 0 ? { kind: "room", id: rooms[0].id } : null;
}

function updateZone(map: FloorMap, target: EditTarget, fn: (z: FloorMapZone) => FloorMapZone): FloorMap {
    if (!target) return map;
    const key = target.kind === "room" ? "rooms" : "nogos";
    return { ...map, [key]: map[key].map((z) => (z.id === target.id ? fn(z) : z)) };
}

export function FloorMapView({ state, robotReady }: FloorMapViewProps) {
    const { t } = useI18n();
    const [errors, errorStack] = useErrorStack();

    const [loading, setLoading] = useState(true);
    const [map, setMap] = useState<FloorMap | null>(null);
    const [savedDoc, setSavedDoc] = useState<string | null>(null);
    const [saving, setSaving] = useState(false);
    const [picking, setPicking] = useState(false);
    const [pinning, setPinning] = useState(false);
    const [tab, setTab] = useState<Tab>("clean");
    const [selection, setSelection] = useState<Set<number>>(loadSelection);
    const [editTarget, setEditTarget] = useState<EditTarget>(null);
    const [activeVertex, setActiveVertex] = useState<number | null>(null);
    const [armed, setArmed] = useState<"room" | "nogo" | null>(null);
    const [draft, setDraft] = useState<{ kind: "room" | "nogo"; a: MapVertex; b: MapVertex } | null>(null);
    const [rotation, setRotation] = useState(loadRotation);
    const [confirm, setConfirm] = useState<"deleteZone" | "deleteMap" | "rename" | null>(null);
    const [pendingAction, setPendingAction] = useState(false);
    const [trail, setTrail] = useState<MapVertex[]>([]);
    const [loc, setLoc] = useState<LocMap | null>(null);
    const [mappingStarted, setMappingStarted] = useState(false);

    const doc = useMemo(() => (map ? serializeFloorMap(map) : null), [map]);
    const dirty = map !== null && doc !== savedDoc;
    const { guardedGoBack, showDiscardConfirm, setShowDiscardConfirm, handleDiscard } = useDirtyGuard(dirty);

    // -- Load -----------------------------------------------------------------
    useEffect(() => {
        api.getFloorMap()
            .then((text) => {
                const parsed = text ? parseFloorMap(text) : null;
                setMap(parsed);
                setSavedDoc(parsed ? serializeFloorMap(parsed) : null);
                setPicking(!parsed);
            })
            .catch((e: unknown) => errorStack.push(normalizeError(e, "Failed to load floor map")))
            .finally(() => setLoading(false));
        api.getLocField()
            .then((buf) => setLoc(buf ? parseLocMap(buf) : null))
            .catch(() => setLoc(null));
    }, [errorStack]);

    useEffect(() => {
        try {
            localStorage.setItem("floorMapRooms", JSON.stringify([...selection]));
        } catch {
            // storage unavailable — selection just isn't remembered
        }
    }, [selection]);

    useEffect(() => {
        localStorage.setItem("floorMapRotation", String(rotation));
    }, [rotation]);

    // Drop selections for rooms that no longer exist
    useEffect(() => {
        if (!map) return;
        setSelection((prev) => {
            const next = new Set([...prev].filter((id) => map.rooms.some((r) => r.id === id)));
            return next.size === prev.size ? prev : next;
        });
    }, [map]);

    // -- Guided status ------------------------------------------------------------
    const [fastPoll, setFastPoll] = useState(false);
    const guided = usePolling<GuidedStatus>(api.getGuidedStatus, fastPoll ? 1000 : 4000);
    const status = guided.data;
    const active = status?.active ?? false;
    useEffect(() => setFastPoll(active), [active]);

    const lastStart = useRef<number>(0);
    useEffect(() => {
        if (!status) return;
        if (status.startedAt !== lastStart.current) {
            lastStart.current = status.startedAt;
            setTrail([]);
        }
        if (status.active && status.phase !== "starting") {
            setTrail((prev) => {
                const last = prev[prev.length - 1];
                if (last && Math.hypot(last[0] - status.x, last[1] - status.y) < 0.05) return prev;
                return [...prev.slice(-1500), [status.x, status.y]];
            });
        }
    }, [status]);

    // -- Canvas view -----------------------------------------------------------------
    const canvasRef = useRef<HTMLCanvasElement | null>(null);
    const [canvasEl, setCanvasEl] = useState<HTMLCanvasElement | null>(null);
    const bindCanvas = useCallback((el: HTMLCanvasElement | null) => {
        canvasRef.current = el;
        setCanvasEl(el);
    }, []);
    const viewRef = useRef<MapView>({ cx: 0, cy: 0, scale: 40, rot: rotation });
    const [view, setView] = useState<MapView>(viewRef.current);
    const screen = (): Screen => ({
        w: canvasRef.current?.clientWidth ?? 300,
        h: canvasRef.current?.clientHeight ?? 300,
    });

    const fit = useCallback(() => {
        if (!map) return;
        const box = floorMapBounds(map, loc);
        if (!box) return;
        const v = fitView(box, screen(), rotation);
        viewRef.current = v;
        setView(v);
    }, [map, rotation, loc]);

    // Fit once when a map first appears (and on rotation changes)
    const fittedFor = useRef<string | null>(null);
    useEffect(() => {
        if (!map || picking || !canvasEl) return;
        const key = `${map.source}:${rotation}`;
        if (fittedFor.current === key) return;
        fittedFor.current = key;
        fit();
    }, [map, picking, rotation, fit, canvasEl]);

    const labelFor = useCallback((z: FloorMapZone) => z.name, []);

    const redraw = useCallback(() => {
        const canvas = canvasRef.current;
        if (!canvas || !map) return;
        renderFloorMap(
            canvas,
            {
                map,
                view: viewRef.current,
                cleanSelection: tab === "clean" ? selection : new Set(),
                editZone: tab === "edit" ? editTarget : null,
                activeVertex,
                draft,
                robot: status && status.active ? { x: status.x, y: status.y, t: status.t } : null,
                target:
                    status?.active && status.targetX !== undefined && status.targetY !== undefined
                        ? [status.targetX, status.targetY]
                        : null,
                trail,
                showPath: tab === "edit",
                loc,
            },
            labelFor,
        );
    }, [map, tab, selection, editTarget, activeVertex, draft, status, trail, labelFor, loc]);

    useEffect(() => {
        const id = requestAnimationFrame(redraw);
        return () => cancelAnimationFrame(id);
    }, [redraw, view, canvasEl]);

    useEffect(() => {
        const onResize = () => redraw();
        window.addEventListener("resize", onResize);
        return () => window.removeEventListener("resize", onResize);
    }, [redraw]);

    // -- Editing gestures ----------------------------------------------------------------
    const dragRef = useRef<{ kind: "draw" | "vertex" | "move"; last: MapVertex; vertex: number } | null>(null);

    useEditorGestures(canvasEl, viewRef, setView, {
        onTap: (w) => {
            if (!map) return;
            if (tab === "clean") {
                const hit = zoneAt(map, w, false);
                if (hit && !active) {
                    setSelection((prev) => {
                        const next = new Set(prev);
                        if (next.has(hit.id)) next.delete(hit.id);
                        else next.add(hit.id);
                        return next;
                    });
                }
                return;
            }
            if (armed) return;
            setEditTarget(zoneAt(map, w, true));
            setActiveVertex(null);
        },
        onDragStart: (w, sx, sy) => {
            if (!map || tab !== "edit") return false;
            if (armed) {
                dragRef.current = { kind: "draw", last: w, vertex: -1 };
                setDraft({ kind: armed, a: w, b: w });
                return true;
            }
            if (!editTarget) return false;
            const zone = (editTarget.kind === "room" ? map.rooms : map.nogos).find((z) => z.id === editTarget.id);
            if (!zone) return false;
            const v = hitVertex(viewRef.current, screen(), zone, sx, sy);
            if (v !== null) {
                dragRef.current = { kind: "vertex", last: w, vertex: v };
                setActiveVertex(v);
                return true;
            }
            if (pointInPolygon(w[0], w[1], zone.poly)) {
                dragRef.current = { kind: "move", last: w, vertex: -1 };
                return true;
            }
            return false;
        },
        onDragMove: (w) => {
            const d = dragRef.current;
            if (!d) return;
            if (d.kind === "draw") {
                setDraft((prev) => (prev ? { ...prev, b: w } : prev));
            } else if (d.kind === "vertex") {
                setMap((m) =>
                    m
                        ? updateZone(m, editTarget, (z) => ({
                              ...z,
                              poly: z.poly.map((p, i) => (i === d.vertex ? w : p)),
                          }))
                        : m,
                );
            } else {
                const dx = w[0] - d.last[0];
                const dy = w[1] - d.last[1];
                setMap((m) =>
                    m
                        ? updateZone(m, editTarget, (z) => ({ ...z, poly: z.poly.map(([x, y]) => [x + dx, y + dy]) }))
                        : m,
                );
            }
            d.last = w;
        },
        onDragEnd: () => {
            const d = dragRef.current;
            dragRef.current = null;
            if (d?.kind !== "draw" || !draft || !map) {
                setDraft(null);
                return;
            }
            const poly = rectPolygon(draft.a, draft.b);
            setDraft(null);
            setArmed(null);
            if (polygonArea(poly) < 0.1) return; // Ignore accidental tiny drags
            if (draft.kind === "room") {
                const id = nextZoneId(map.rooms);
                setMap({ ...map, rooms: [...map.rooms, { id, name: t("Room {id}", { id }), poly }] });
                setEditTarget({ kind: "room", id });
                setConfirm("rename");
            } else {
                const id = nextZoneId(map.nogos);
                setMap({ ...map, nogos: [...map.nogos, { id, name: "", poly }] });
                setEditTarget({ kind: "nogo", id });
            }
        },
        onDoubleTap: fit,
    });

    // -- Actions -------------------------------------------------------------------------
    const save = useCallback(async () => {
        if (!map || !doc) return;
        setSaving(true);
        try {
            await api.saveFloorMap(doc);
            setSavedDoc(doc);
        } catch (e: unknown) {
            errorStack.push(normalizeError(e, "Failed to save floor map"));
        } finally {
            setSaving(false);
        }
    }, [map, doc, errorStack]);

    const pin = useCallback(
        async (file: HistoryFileInfo) => {
            setPinning(true);
            try {
                const sessions = await api.getHistorySession(file.name);
                if (sessions.length === 0 || sessions[0].path.length < 2) throw new Error(t("Session has no path"));
                const result = await pinSession(file.name, sessions[0], map);
                const next = result.map;
                setMap(next);
                setSavedDoc(result.doc);
                if (result.loc) setLoc(result.loc);
                setPicking(false);
                fittedFor.current = null;
                if (next.rooms.length === 0) setTab("edit");
            } catch (e: unknown) {
                errorStack.push(normalizeError(e, "Failed to pin session"));
            } finally {
                setPinning(false);
            }
        },
        [map, errorStack, t],
    );

    const startMapping = useCallback(async () => {
        try {
            await api.startMappingRun();
            setMappingStarted(true);
        } catch (e: unknown) {
            errorStack.push(normalizeError(e, "Action failed"));
        }
    }, [errorStack]);

    const deleteMap = useCallback(async () => {
        setConfirm(null);
        try {
            await api.deleteFloorMap();
            setMap(null);
            setLoc(null);
            setSavedDoc(null);
            setPicking(true);
            setEditTarget(null);
        } catch (e: unknown) {
            errorStack.push(normalizeError(e, "Failed to delete floor map"));
        }
    }, [errorStack]);

    const runGuided = useCallback(
        async (fn: () => Promise<void>) => {
            setPendingAction(true);
            try {
                await fn();
                setFastPoll(true);
            } catch (e: unknown) {
                errorStack.push(normalizeError(e, "Action failed"));
            } finally {
                setPendingAction(false);
            }
        },
        [errorStack],
    );

    const editZone =
        map && editTarget
            ? ((editTarget.kind === "room" ? map.rooms : map.nogos).find((z) => z.id === editTarget.id) ?? null)
            : null;

    const addCorner = () => {
        if (!map || !editZone || editZone.poly.length >= MAX_VERTICES) return;
        // Split the longest edge — lets rectangles become L-shapes
        let best = 0;
        let bestLen = -1;
        editZone.poly.forEach((p, i) => {
            const q = editZone.poly[(i + 1) % editZone.poly.length];
            const len = Math.hypot(q[0] - p[0], q[1] - p[1]);
            if (len > bestLen) {
                bestLen = len;
                best = i;
            }
        });
        const a = editZone.poly[best];
        const b = editZone.poly[(best + 1) % editZone.poly.length];
        const mid: MapVertex = [(a[0] + b[0]) / 2, (a[1] + b[1]) / 2];
        setMap(
            updateZone(map, editTarget, (z) => ({
                ...z,
                poly: [...z.poly.slice(0, best + 1), mid, ...z.poly.slice(best + 1)],
            })),
        );
        setActiveVertex(best + 1);
    };

    const removeCorner = () => {
        if (!map || !editZone || editZone.poly.length <= 3) return;
        const idx = activeVertex ?? editZone.poly.length - 1;
        setMap(updateZone(map, editTarget, (z) => ({ ...z, poly: z.poly.filter((_, i) => i !== idx) })));
        setActiveVertex(null);
    };

    const deleteZone = () => {
        setConfirm(null);
        if (!map || !editTarget) return;
        const key = editTarget.kind === "room" ? "rooms" : "nogos";
        setMap({ ...map, [key]: map[key].filter((z) => z.id !== editTarget.id) });
        setEditTarget(null);
    };

    const renameZone = (name?: string) => {
        setConfirm(null);
        const trimmed = name?.trim();
        if (!map || !editTarget || !trimmed) return;
        setMap(updateZone(map, editTarget, (z) => ({ ...z, name: trimmed.slice(0, 32) })));
    };

    // -- Start conditions ---------------------------------------------------------------
    const ui = state?.uiState ?? "";
    const robotIdle = ui.includes("IDLE") || ui.includes("STANDBY");
    let startHint: string | null = null;
    if (!robotReady) startHint = "Robot not ready";
    else if (dirty) startHint = "Save the map first";
    else if (selection.size === 0) startHint = null;
    else if (!robotIdle) startHint = "The robot is busy";
    else startHint = "The robot must start on its dock: the dock is the origin of the map.";
    const canStart = robotReady && !dirty && selection.size > 0 && robotIdle && !active;

    const handleBack = () => {
        if (picking && map) {
            setPicking(false);
            return;
        }
        guardedGoBack("/");
    };

    const rotate = (delta: number) => {
        setRotation((r) => (((r + delta) % 360) + 360) % 360);
    };

    return (
        <>
            <div class="header">
                <button type="button" class="header-back-btn" onClick={handleBack} aria-label={t("Back")}>
                    <Icon svg={backSvg} />
                </button>
                <h1>
                    <T>Floor Map</T>
                </h1>
                <div class="header-right-spacer" />
            </div>

            <ErrorBannerStack errors={errors} />

            <div class="history-page floormap-page">
                {loading && (
                    <div class="history-empty">
                        <T>Loading...</T>
                    </div>
                )}

                {!loading && picking && (
                    <div class="floormap-intro">
                        <h2 class="floormap-intro-title">
                            <T>Pin a cleaning run as your floor map</T>
                        </h2>
                        <p class="floormap-note">
                            <T>
                                Pick a complete house clean that started on the dock. Its path becomes the map you draw
                                rooms and no-go zones on, and the route the robot follows when it cleans single rooms.
                            </T>
                        </p>
                        <MappingCard started={mappingStarted} robotIdle={robotIdle} onStart={startMapping} />
                        <SessionPicker
                            current={map?.source ?? null}
                            busy={pinning}
                            onPick={pin}
                            onError={errorStack.push}
                        />
                    </div>
                )}

                {!loading && !picking && map && (
                    <>
                        <div class="floormap-tabs" role="tablist">
                            <button
                                type="button"
                                role="tab"
                                aria-selected={tab === "clean"}
                                class={`floormap-tab${tab === "clean" ? " active" : ""}`}
                                onClick={() => {
                                    setTab("clean");
                                    setArmed(null);
                                    setEditTarget(null);
                                }}
                            >
                                <T>Clean</T>
                            </button>
                            <button
                                type="button"
                                role="tab"
                                aria-selected={tab === "edit"}
                                class={`floormap-tab${tab === "edit" ? " active" : ""}`}
                                onClick={() => setTab("edit")}
                                disabled={active}
                            >
                                <T>Edit</T>
                                {dirty && <span class="floormap-dirty-dot" />}
                            </button>
                        </div>

                        <div class="history-canvas-wrap">
                            <canvas
                                ref={bindCanvas}
                                class={`history-canvas floormap-canvas${armed ? " drawing" : ""}`}
                            />
                            <button
                                type="button"
                                class="history-rotate-btn left"
                                onClick={() => rotate(-90)}
                                aria-label={t("Rotate map counter-clockwise")}
                            >
                                <Icon svg={rotateLeftSvg} />
                            </button>
                            <button
                                type="button"
                                class="history-rotate-btn right"
                                onClick={() => rotate(90)}
                                aria-label={t("Rotate map clockwise")}
                            >
                                <Icon svg={rotateRightSvg} />
                            </button>
                        </div>
                        <div class="history-map-hint">
                            {t(
                                tab === "clean"
                                    ? "Tap rooms to select them. Pinch or scroll to zoom, double-tap to fit."
                                    : "Tap a room or no-go zone to edit it. Pinch or scroll to zoom, double-tap to fit.",
                            )}
                        </div>

                        {tab === "clean" ? (
                            <CleanPanel
                                map={map}
                                selection={selection}
                                status={status}
                                canStart={canStart}
                                startHint={startHint}
                                pending={pendingAction}
                                onToggle={(id) =>
                                    setSelection((prev) => {
                                        const next = new Set(prev);
                                        if (next.has(id)) next.delete(id);
                                        else next.add(id);
                                        return next;
                                    })
                                }
                                onSelectAll={(all) => setSelection(new Set(all ? map.rooms.map((r) => r.id) : []))}
                                onStart={() => runGuided(() => api.guidedStart([...selection].sort((a, b) => a - b)))}
                                onHome={() => runGuided(api.guidedHome)}
                                onStop={() => runGuided(api.guidedStop)}
                            />
                        ) : (
                            <EditPanel
                                map={map}
                                target={editTarget}
                                armed={armed}
                                dirty={dirty}
                                saving={saving}
                                onArm={(k) => {
                                    setArmed(k);
                                    setEditTarget(null);
                                }}
                                onSelect={(target) => {
                                    setEditTarget(target);
                                    setActiveVertex(null);
                                }}
                                onRename={() => setConfirm("rename")}
                                onAddCorner={addCorner}
                                onRemoveCorner={removeCorner}
                                onDelete={() => setConfirm("deleteZone")}
                                onSave={save}
                                onDiscard={() => {
                                    const restored = savedDoc ? parseFloorMap(savedDoc) : null;
                                    if (restored) setMap(restored);
                                    setEditTarget(null);
                                }}
                                onReplace={() => setPicking(true)}
                                onDeleteMap={() => setConfirm("deleteMap")}
                            />
                        )}
                    </>
                )}

                {confirm === "rename" && editZone && (
                    <ConfirmDialog
                        message={t("Room name")}
                        confirmLabel={t("Save")}
                        destructive={false}
                        inputType="text"
                        inputPlaceholder={editZone.name}
                        inputRequired
                        onConfirm={renameZone}
                        onCancel={() => setConfirm(null)}
                    />
                )}
                {confirm === "deleteZone" && editZone && (
                    <ConfirmDialog
                        message={t(editTarget?.kind === "room" ? "Delete this room?" : "Delete this no-go zone?")}
                        confirmLabel={t("Delete")}
                        onConfirm={deleteZone}
                        onCancel={() => setConfirm(null)}
                    />
                )}
                {confirm === "deleteMap" && (
                    <ConfirmDialog
                        message={t("Delete the floor map with all rooms and no-go zones?")}
                        confirmLabel={t("Delete")}
                        onConfirm={deleteMap}
                        onCancel={() => setConfirm(null)}
                    />
                )}
                {showDiscardConfirm && (
                    <ConfirmDialog
                        message={t("You have unsaved changes. Discard them?")}
                        confirmLabel={t("Discard")}
                        onConfirm={handleDiscard}
                        onCancel={() => setShowDiscardConfirm(false)}
                    />
                )}
            </div>
        </>
    );
}

import { useEffect, useRef } from "preact/hooks";
import type { MapVertex } from "../../floor-map";
import { type MapView, panBy, type Screen, toWorld, zoomAt } from "./canvas";

export interface EditorHandlers {
    onTap: (world: MapVertex, sx: number, sy: number) => void;
    // Return true when the editor takes over this drag (drawing, moving a
    // vertex or zone). Otherwise the drag pans the map.
    onDragStart: (world: MapVertex, sx: number, sy: number) => boolean;
    onDragMove: (world: MapVertex) => void;
    onDragEnd: () => void;
    onDoubleTap: () => void;
}

const TAP_SLOP_PX = 6;
const DOUBLE_TAP_MS = 300;

interface Pt {
    x: number;
    y: number;
}

// Pointer handling for the floor map editor: one finger / mouse drag pans or
// edits (decided by the editor on drag start), two fingers pinch-zoom and
// pan, wheel zooms at the cursor, double tap re-fits the map.
export function useEditorGestures(
    canvas: HTMLCanvasElement | null,
    viewRef: { current: MapView },
    setView: (v: MapView) => void,
    handlers: EditorHandlers,
) {
    const handlersRef = useRef(handlers);
    handlersRef.current = handlers;

    useEffect(() => {
        if (!canvas) return;

        const pointers = new Map<number, Pt>();
        let start: Pt | null = null;
        let last: Pt | null = null;
        let mode: "idle" | "pending" | "pan" | "edit" | "pinch" = "idle";
        let pinchDist = 0;
        let pinchMid: Pt = { x: 0, y: 0 };
        let lastTap = 0;

        const screen = (): Screen => ({ w: canvas.clientWidth, h: canvas.clientHeight });
        const local = (e: PointerEvent | WheelEvent): Pt => {
            const r = canvas.getBoundingClientRect();
            return { x: e.clientX - r.left, y: e.clientY - r.top };
        };
        const world = (p: Pt): MapVertex => toWorld(viewRef.current, screen(), p.x, p.y);
        const commit = (v: MapView) => {
            viewRef.current = v;
            setView(v);
        };
        const pinchState = () => {
            const [a, b] = [...pointers.values()];
            return { dist: Math.hypot(a.x - b.x, a.y - b.y), mid: { x: (a.x + b.x) / 2, y: (a.y + b.y) / 2 } };
        };

        const onDown = (e: PointerEvent) => {
            if (e.pointerType === "mouse" && e.button !== 0) return;
            canvas.setPointerCapture(e.pointerId);
            const p = local(e);
            pointers.set(e.pointerId, p);
            if (pointers.size === 1) {
                start = p;
                last = p;
                mode = "pending";
            } else if (pointers.size === 2) {
                if (mode === "edit") handlersRef.current.onDragEnd();
                const ps = pinchState();
                pinchDist = ps.dist;
                pinchMid = ps.mid;
                mode = "pinch";
            }
        };

        const onMove = (e: PointerEvent) => {
            if (!pointers.has(e.pointerId)) return;
            const p = local(e);
            pointers.set(e.pointerId, p);
            if (mode === "pinch" && pointers.size >= 2) {
                const ps = pinchState();
                let v = viewRef.current;
                if (pinchDist > 0) v = zoomAt(v, screen(), ps.mid.x, ps.mid.y, ps.dist / pinchDist);
                v = panBy(v, screen(), ps.mid.x - pinchMid.x, ps.mid.y - pinchMid.y);
                pinchDist = ps.dist;
                pinchMid = ps.mid;
                commit(v);
                return;
            }
            if (!start || !last) return;
            if (mode === "pending" && Math.hypot(p.x - start.x, p.y - start.y) > TAP_SLOP_PX) {
                mode = handlersRef.current.onDragStart(world(start), start.x, start.y) ? "edit" : "pan";
            }
            if (mode === "pan") {
                commit(panBy(viewRef.current, screen(), p.x - last.x, p.y - last.y));
            } else if (mode === "edit") {
                handlersRef.current.onDragMove(world(p));
            }
            last = p;
        };

        const onUp = (e: PointerEvent) => {
            if (!pointers.has(e.pointerId)) return;
            pointers.delete(e.pointerId);
            if (mode === "pinch") {
                if (pointers.size === 0) mode = "idle";
                // Lifting one finger of a pinch must not start a pan jump
                start = null;
                last = null;
                return;
            }
            if (mode === "pending" && start) {
                const now = Date.now();
                if (now - lastTap < DOUBLE_TAP_MS) {
                    lastTap = 0;
                    handlersRef.current.onDoubleTap();
                } else {
                    lastTap = now;
                    handlersRef.current.onTap(world(start), start.x, start.y);
                }
            } else if (mode === "edit") {
                handlersRef.current.onDragEnd();
            }
            mode = "idle";
            start = null;
            last = null;
        };

        const onWheel = (e: WheelEvent) => {
            e.preventDefault();
            const p = local(e);
            commit(zoomAt(viewRef.current, screen(), p.x, p.y, Math.exp(-e.deltaY * 0.0015)));
        };

        canvas.addEventListener("pointerdown", onDown);
        canvas.addEventListener("pointermove", onMove);
        canvas.addEventListener("pointerup", onUp);
        canvas.addEventListener("pointercancel", onUp);
        canvas.addEventListener("wheel", onWheel, { passive: false });
        return () => {
            canvas.removeEventListener("pointerdown", onDown);
            canvas.removeEventListener("pointermove", onMove);
            canvas.removeEventListener("pointerup", onUp);
            canvas.removeEventListener("pointercancel", onUp);
            canvas.removeEventListener("wheel", onWheel);
        };
    }, [canvas, viewRef, setView]);
}

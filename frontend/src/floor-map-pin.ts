// Pin a recorded session as the floor map: upload the map document and, for
// mapping runs, the LIDAR localization field built from its scans.

import { api } from "./api";
import { type FloorMap, floorMapFromSession, parseFloorMap, serializeFloorMap } from "./floor-map";
import { buildLocMap, type LocMap, serializeLocMap } from "./loc-map";
import type { MapData } from "./types";

export interface PinResult {
    map: FloorMap;
    doc: string;
    loc: LocMap | null;
}

export async function pinSession(name: string, session: MapData, previous?: FloorMap | null): Promise<PinResult> {
    let prev = previous;
    if (prev === undefined) {
        const existing = await api.getFloorMap();
        prev = existing ? parseFloorMap(existing) : null;
    }
    const map = floorMapFromSession(name, session, prev ?? null);
    const doc = serializeFloorMap(map);
    // Building the field can take a moment on phones; yield first so the UI updates
    await new Promise((r) => setTimeout(r, 0));
    const loc = session.scans.length > 0 ? buildLocMap(session.scans) : null;
    await api.saveFloorMap(doc);
    if (loc) await api.saveLocField(serializeLocMap(loc));
    return { map, doc, loc };
}

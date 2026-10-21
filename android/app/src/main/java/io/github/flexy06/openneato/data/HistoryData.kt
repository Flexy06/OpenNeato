package io.github.flexy06.openneato.data

import java.util.Base64
import kotlinx.serialization.json.Json
import kotlinx.serialization.json.JsonObject
import kotlinx.serialization.json.doubleOrNull
import kotlinx.serialization.json.jsonObject
import kotlinx.serialization.json.jsonPrimitive
import kotlin.math.ceil
import kotlin.math.max
import kotlin.math.roundToInt

// Port of frontend/src/history-data.ts: a session JSONL (pose snapshots, session /
// summary lines, recharge markers, mapping-run LIDAR scans) as path, coverage and
// bounds for drawing. Units: metres, degrees, seconds since the first pose.

data class PathPoint(val x: Double, val y: Double, val t: Double, val ts: Double)

// Earliest time a 5 cm cell was under the robot (for the replay)
data class CoverCell(val cx: Int, val cy: Int, val ts: Double)

data class Recharge(val x: Double, val y: Double, val ts: Double, val endTs: Double)

data class Bounds(val minX: Double, val maxX: Double, val minY: Double, val maxY: Double)

// Mapping-run scan: robot pose (m, deg) + 180 bins of 2° in 20 mm units (0 = no return)
class Scan(val x: Double, val y: Double, val t: Double, val bins: ByteArray)

data class SessionInfo(val mode: String, val time: Long, val battery: Int?)

data class SummaryInfo(
    val duration: Double,
    val distance: Double,
    val area: Double,
    val recharges: Int,
    val batteryEnd: Int?,
)

class SessionMap(
    val session: SessionInfo?,
    val summary: SummaryInfo?,
    val path: List<PathPoint>,
    val coverage: List<CoverCell>,
    val recharges: List<Recharge>,
    val bounds: Bounds?,
    val scans: List<Scan>,
) {
    // Summary value when written, else the last pose time
    val duration: Double get() = summary?.duration?.takeIf { it > 0 } ?: path.lastOrNull()?.ts ?: 0.0

    companion object {
        const val CELL_M = 0.05
        const val ROBOT_DIAMETER_M = 0.33
    }
}

object HistoryData {
    private val json = Json { ignoreUnknownKeys = true }

    // Heatshrink decompression can corrupt single bytes in numeric tokens ('.' -> ':'):
    // match the pose skeleton permissively and repair the numbers
    private val POSE_RE = Regex(
        """^\{.x.:\s*(-?[\d.eE:"\w-]+)\s*,.y.:\s*(-?[\d.eE:"\w-]+)\s*,.t.:\s*([\d.eE:"\w-]+)\s*,.ts.:\s*([\d.eE:"\w-]+)\s*\}$""",
    )

    private fun repairNumber(raw: String): Double? {
        val cleaned = raw.replace(Regex("[^0-9.eE-]"), ".")
        val parts = cleaned.split(".")
        val fixed = if (parts.size > 2) parts[0] + "." + parts.drop(1).joinToString("") else cleaned
        return fixed.toDoubleOrNull()?.takeIf { it.isFinite() }
    }

    private fun repairPose(line: String): PathPoint? {
        val m = POSE_RE.matchEntire(line) ?: return null
        val v = m.groupValues.drop(1).map { repairNumber(it) ?: return null }
        return PathPoint(v[0], v[1], v[2], v[3])
    }

    /** Raw JSONL (possibly several sessions) -> sessions with at least one pose */
    fun parse(raw: String): List<SessionMap> {
        val chunks = mutableListOf<MutableList<String>>()
        var current = mutableListOf<String>()
        for (line in raw.trim().lineSequence().map { it.trim() }.filter { it.isNotEmpty() }) {
            if (line.contains("\"type\":\"session\"") && current.isNotEmpty()) {
                chunks += current
                current = mutableListOf()
            }
            current += line
        }
        if (current.isNotEmpty()) chunks += current
        return chunks.map(::build).filter { it.path.isNotEmpty() }
    }

    private fun JsonObject.num(key: String): Double? = this[key]?.jsonPrimitive?.doubleOrNull
    private fun JsonObject.str(key: String): String? = this[key]?.jsonPrimitive?.content

    private fun build(lines: List<String>): SessionMap {
        var session: SessionInfo? = null
        var summary: SummaryInfo? = null
        val scans = mutableListOf<Scan>()
        val raw = mutableListOf<PathPoint>()
        val rawRecharges = mutableListOf<Triple<Double, Double, Double>>() // x, y, last pose ts
        var lastPoseTs = 0.0

        for (l in lines) {
            val obj = runCatching { json.parseToJsonElement(l).jsonObject }.getOrNull()
            if (obj == null) {
                repairPose(l)?.let {
                    raw += it
                    lastPoseTs = it.ts
                }
                continue
            }
            when (obj.str("type")) {
                "session" -> if (session == null) {
                    session = SessionInfo(
                        mode = obj.str("mode").orEmpty(),
                        time = obj.num("time")?.toLong() ?: 0L,
                        battery = obj.num("battery")?.roundToInt(),
                    )
                }
                "summary" -> if (summary == null) {
                    summary = SummaryInfo(
                        duration = obj.num("duration") ?: 0.0,
                        distance = obj.num("distanceTraveled") ?: 0.0,
                        area = obj.num("areaCovered") ?: 0.0,
                        recharges = obj.num("recharges")?.roundToInt() ?: 0,
                        batteryEnd = obj.num("batteryEnd")?.roundToInt(),
                    )
                }
                "recharge" -> {
                    val x = obj.num("x")
                    val y = obj.num("y")
                    if (x != null && y != null) rawRecharges += Triple(x, y, lastPoseTs)
                }
                "scan" -> decodeScan(obj)?.let { scans += it }
                null -> {
                    val x = obj.num("x")
                    val y = obj.num("y")
                    val t = obj.num("t")
                    val ts = obj.num("ts")
                    if (x != null && y != null && t != null && ts != null) {
                        raw += PathPoint(x, y, t, ts)
                        lastPoseTs = ts
                    }
                }
            }
        }

        // Before the first real snapshot the robot reports 0,0,0
        val kept = raw.filter { it.x != 0.0 || it.y != 0.0 || it.t != 0.0 }
        if (kept.isEmpty()) return SessionMap(session, summary, emptyList(), emptyList(), emptyList(), null, scans)

        // `ts` is seconds since boot: make it session-relative
        val origin = kept.first().ts
        val poses = kept.map { it.copy(ts = max(0.0, it.ts - origin)) }

        // Charge windows: long gaps in the pose timeline, paired with the nearest marker
        val gaps = mutableListOf<Pair<Double, Double>>()
        for (i in 1 until poses.size) {
            if (poses[i].ts - poses[i - 1].ts >= GAP_MIN_S) gaps += poses[i - 1].ts to poses[i].ts
        }
        val used = mutableSetOf<Int>()
        val recharges = rawRecharges.map { (x, y, prevTs) ->
            val marker = max(0.0, prevTs - origin)
            var best = -1
            var bestScore = Double.MAX_VALUE
            gaps.forEachIndexed { i, g ->
                if (i in used) return@forEachIndexed
                val d = if (g.first >= marker) g.first - marker else (marker - g.first) * 4
                if (d < bestScore) {
                    bestScore = d
                    best = i
                }
            }
            if (best >= 0) {
                used += best
                Recharge(x, y, gaps[best].first, gaps[best].second)
            } else {
                val next = poses.firstOrNull { it.ts > marker }
                Recharge(x, y, marker, next?.ts ?: (marker + POSE_INTERVAL_S))
            }
        }

        // Coverage: robot footprint stamped at each pose, earliest time per cell
        val r = ceil(SessionMap.ROBOT_DIAMETER_M / 2 / SessionMap.CELL_M).toInt()
        val first = LinkedHashMap<Long, CoverCell>()
        for (p in poses) {
            val cx = (p.x / SessionMap.CELL_M).roundToInt()
            val cy = (p.y / SessionMap.CELL_M).roundToInt()
            for (dx in -r..r) {
                for (dy in -r..r) {
                    if (dx * dx + dy * dy > r * r) continue
                    val key = (cx + dx).toLong() shl 32 or ((cy + dy).toLong() and 0xffffffffL)
                    if (key !in first) first[key] = CoverCell(cx + dx, cy + dy, p.ts)
                }
            }
        }

        val pad = SessionMap.ROBOT_DIAMETER_M / 2 + 0.1
        val bounds = Bounds(
            poses.minOf { it.x } - pad,
            poses.maxOf { it.x } + pad,
            poses.minOf { it.y } - pad,
            poses.maxOf { it.y } + pad,
        )
        return SessionMap(session, summary, poses, first.values.toList(), recharges, bounds, scans)
    }

    private fun decodeScan(obj: JsonObject): Scan? {
        val d = obj.str("d") ?: return null
        val bins = runCatching { Base64.getMimeDecoder().decode(d) }.getOrNull() ?: return null
        if (bins.size != 180) return null
        return Scan(obj.num("x") ?: return null, obj.num("y") ?: return null, obj.num("t") ?: return null, bins)
    }

    /** The session line of a file, null when the line is not one */
    fun parseHeader(line: String): SessionInfo? {
        val obj = runCatching { json.parseToJsonElement(line).jsonObject }.getOrNull() ?: return null
        if (obj.str("type") != "session") return null
        return SessionInfo(obj.str("mode").orEmpty(), obj.num("time")?.toLong() ?: 0L, obj.num("battery")?.roundToInt())
    }

    /** Pose at a session time: linear for x/y, shortest arc for the heading */
    fun poseAt(path: List<PathPoint>, ts: Double): PathPoint? {
        if (path.isEmpty()) return null
        if (ts <= path.first().ts) return path.first()
        if (ts >= path.last().ts) return path.last()
        var lo = 0
        var hi = path.size - 1
        while (hi - lo > 1) {
            val mid = (lo + hi) ushr 1
            if (path[mid].ts <= ts) lo = mid else hi = mid
        }
        val a = path[lo]
        val b = path[hi]
        val span = b.ts - a.ts
        val f = if (span > 0) (ts - a.ts) / span else 0.0
        val dt = ((b.t - a.t + 540.0) % 360.0) - 180.0
        return PathPoint(a.x + (b.x - a.x) * f, a.y + (b.y - a.y) * f, a.t + dt * f, ts)
    }

    private const val POSE_INTERVAL_S = 2.0 // Firmware samples every ~2 s
    private const val GAP_MIN_S = 30.0 // Longer pauses are charges
}

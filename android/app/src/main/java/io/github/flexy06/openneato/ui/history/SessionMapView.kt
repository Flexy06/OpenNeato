package io.github.flexy06.openneato.ui.history

import androidx.compose.foundation.Canvas
import androidx.compose.foundation.gestures.detectTapGestures
import androidx.compose.foundation.gestures.detectTransformGestures
import androidx.compose.runtime.Composable
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableFloatStateOf
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.setValue
import androidx.compose.ui.Modifier
import androidx.compose.ui.geometry.Offset
import androidx.compose.ui.geometry.Size
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.graphics.Path
import androidx.compose.ui.graphics.PointMode
import androidx.compose.ui.graphics.StrokeCap
import androidx.compose.ui.graphics.StrokeJoin
import androidx.compose.ui.graphics.drawscope.DrawScope
import androidx.compose.ui.graphics.drawscope.Stroke
import androidx.compose.ui.graphics.drawscope.rotate
import androidx.compose.ui.graphics.drawscope.withTransform
import androidx.compose.ui.input.pointer.pointerInput
import androidx.compose.ui.unit.dp
import io.github.flexy06.openneato.data.Bounds
import io.github.flexy06.openneato.data.HistoryData
import io.github.flexy06.openneato.data.PathPoint
import io.github.flexy06.openneato.data.SessionMap
import kotlin.math.cos
import kotlin.math.floor
import kotlin.math.min
import kotlin.math.sin

data class MapColors(
    val background: Color,
    val grid: Color,
    val walls: Color,
    val coverage: Color,
    val path: Color,
    val start: Color,
    val end: Color,
    val robot: Color,
    val recharge: Color,
)

// World (metres, y up) -> canvas, the session centred with a margin
private class Projection(size: Size, b: Bounds, pad: Float) {
    val scale: Float
    private val offX: Float
    private val offY: Float
    private val minX = b.minX
    private val maxY = b.maxY

    init {
        val w = (b.maxX - b.minX).toFloat()
        val h = (b.maxY - b.minY).toFloat()
        val availW = size.width - 2 * pad
        val availH = size.height - 2 * pad
        scale = min(availW / w, availH / h)
        offX = pad + (availW - w * scale) / 2
        offY = pad + (availH - h * scale) / 2
    }

    fun x(wx: Double) = offX + ((wx - minX) * scale).toFloat()
    fun y(wy: Double) = offY + ((maxY - wy) * scale).toFloat()
}

/**
 * A session on a map: LIDAR walls of a mapping run, coverage (the path drawn at
 * robot width), the path, start/end and recharges. With [time] only the part up
 * to that session time is drawn plus the robot at its interpolated pose (replay).
 * Pinch to zoom, drag to pan, double tap to reset; [rotation] turns the view
 * in 90° steps (like the web UI).
 */
@Composable
fun SessionMapView(
    map: SessionMap,
    colors: MapColors,
    rotation: Int,
    recording: Boolean,
    time: Double?,
    modifier: Modifier = Modifier,
) {
    var zoom by remember { mutableFloatStateOf(1f) }
    var pan by remember { mutableStateOf(Offset.Zero) }
    // LIDAR returns in world coordinates, computed once per session
    val walls = remember(map) { wallPoints(map) }
    val bounds = remember(map, walls) { viewBounds(map.bounds, walls) }

    Canvas(
        modifier
            .pointerInput(Unit) {
                detectTransformGestures { centroid, delta, z, _ ->
                    val nz = (zoom * z).coerceIn(1f, 12f)
                    val f = nz / zoom
                    pan = centroid - (centroid - pan) * f + delta
                    zoom = nz
                }
            }
            .pointerInput(Unit) {
                detectTapGestures(onDoubleTap = { at ->
                    if (zoom > 1.5f) {
                        zoom = 1f
                        pan = Offset.Zero
                    } else {
                        pan = at - (at - pan) * 2.5f
                        zoom *= 2.5f
                    }
                })
            },
    ) {
        drawRect(colors.background)
        if (bounds == null) return@Canvas
        // A quarter turn swaps the room's width and height on screen
        val turned = rotation % 180 != 0
        val area = if (turned) Size(size.height, size.width) else size
        val proj = Projection(area, bounds, 20.dp.toPx())
        withTransform({
            translate(pan.x, pan.y)
            scale(zoom, zoom, Offset.Zero)
            rotate(rotation.toFloat(), center)
            translate((size.width - area.width) / 2, (size.height - area.height) / 2)
        }) {
            drawSession(map, bounds, proj, colors, walls, recording, time, zoom)
        }
    }
}

// The path's bounds grown to the walls of a mapping run (without the few stray
// far returns through doors and windows)
private fun viewBounds(path: Bounds?, walls: List<Pair<Double, Double>>): Bounds? {
    if (path == null || walls.size < 50) return path
    val xs = walls.map { it.first }.sorted()
    val ys = walls.map { it.second }.sorted()
    val lo = walls.size / 200
    val hi = walls.size - 1 - lo
    return Bounds(
        minOf(path.minX, xs[lo] - 0.1),
        maxOf(path.maxX, xs[hi] + 0.1),
        minOf(path.minY, ys[lo] - 0.1),
        maxOf(path.maxY, ys[hi] + 0.1),
    )
}

private fun wallPoints(map: SessionMap): List<Pair<Double, Double>> {
    val out = ArrayList<Pair<Double, Double>>(map.scans.size * 120)
    for (s in map.scans) {
        val th = Math.toRadians(s.t)
        val c = cos(th)
        val sn = sin(th)
        for (i in 0 until 180) {
            val v = s.bins[i].toInt() and 0xff
            if (v == 0) continue
            val d = v * 0.02
            val a = Math.toRadians(i * 2.0 + 1.0) // Counter-clockwise turret (the recorded runs)
            val lx = d * cos(a)
            val ly = d * sin(a)
            out += (s.x + c * lx - sn * ly) to (s.y + sn * lx + c * ly)
        }
    }
    return out
}

private fun DrawScope.drawSession(
    map: SessionMap,
    b: Bounds,
    p: Projection,
    colors: MapColors,
    walls: List<Pair<Double, Double>>,
    recording: Boolean,
    time: Double?,
    zoom: Float,
) {
    // Grid every 0.5 m
    val gridW = 1f / zoom
    var gx = floor(b.minX / 0.5) * 0.5
    while (gx <= b.maxX) {
        drawLine(colors.grid, Offset(p.x(gx), p.y(b.minY)), Offset(p.x(gx), p.y(b.maxY)), gridW.dp.toPx())
        gx += 0.5
    }
    var gy = floor(b.minY / 0.5) * 0.5
    while (gy <= b.maxY) {
        drawLine(colors.grid, Offset(p.x(b.minX), p.y(gy)), Offset(p.x(b.maxX), p.y(gy)), gridW.dp.toPx())
        gy += 0.5
    }

    if (walls.isNotEmpty()) {
        drawPoints(
            walls.map { Offset(p.x(it.first), p.y(it.second)) },
            PointMode.Points,
            colors.walls,
            strokeWidth = (2.5f / zoom).dp.toPx(),
            cap = StrokeCap.Round,
        )
    }

    // The part of the path up to `time` (+ the interpolated head while replaying)
    val tNow = time ?: Double.MAX_VALUE
    val shown = ArrayList<PathPoint>(map.path.size + 1)
    for (pt in map.path) {
        if (pt.ts > tNow) break
        shown += pt
    }
    val head = time?.let { HistoryData.poseAt(map.path, it) }
    if (head != null && (shown.isEmpty() || shown.last().ts != head.ts)) shown += head

    if (shown.size > 1) {
        val path = Path().apply {
            moveTo(p.x(shown[0].x), p.y(shown[0].y))
            for (i in 1 until shown.size) lineTo(p.x(shown[i].x), p.y(shown[i].y))
        }
        // Coverage: the path at the robot's width, one translucent layer
        drawPath(
            path,
            colors.coverage,
            style = Stroke(
                (SessionMap.ROBOT_DIAMETER_M * p.scale).toFloat(),
                cap = StrokeCap.Round,
                join = StrokeJoin.Round,
            ),
        )
        drawPath(
            path,
            colors.path,
            style = Stroke((2f / zoom).dp.toPx(), cap = StrokeCap.Round, join = StrokeJoin.Round),
        )
    }

    val dot = (5f / zoom).dp.toPx()
    map.path.firstOrNull()?.let { drawCircle(colors.start, dot, Offset(p.x(it.x), p.y(it.y))) }

    for (r in map.recharges) {
        if (r.ts > tNow) continue
        drawBolt(Offset(p.x(r.x), p.y(r.y)), (10f / zoom).dp.toPx(), colors.recharge)
    }

    if (head != null) {
        drawRobot(Offset(p.x(head.x), p.y(head.y)), head.t, (7f / zoom).dp.toPx(), colors.robot)
    } else if (map.path.size > 1) {
        val e = map.path.last()
        val at = Offset(p.x(e.x), p.y(e.y))
        if (recording) {
            drawCircle(colors.start, dot * 1.2f, at, style = Stroke((2.5f / zoom).dp.toPx()))
        } else {
            drawCircle(colors.end, dot, at)
        }
    }
}

// Robot sprite: body with a nose in the heading direction (degrees, CCW, y up)
private fun DrawScope.drawRobot(at: Offset, headingDeg: Double, radius: Float, color: Color) {
    rotate(-headingDeg.toFloat(), at) {
        val nose = Path().apply {
            moveTo(at.x + radius + radius * 0.7f, at.y)
            lineTo(at.x + radius * 0.6f, at.y - radius * 0.7f)
            lineTo(at.x + radius * 0.6f, at.y + radius * 0.7f)
            close()
        }
        drawPath(nose, color)
        drawCircle(color, radius, at)
        drawCircle(Color.White.copy(alpha = 0.9f), radius * 0.35f, at)
    }
}

private fun DrawScope.drawBolt(at: Offset, s: Float, color: Color) {
    val bolt = Path().apply {
        moveTo(at.x + s * 0.15f, at.y - s)
        lineTo(at.x - s * 0.55f, at.y + s * 0.05f)
        lineTo(at.x - s * 0.05f, at.y + s * 0.05f)
        lineTo(at.x - s * 0.15f, at.y + s)
        lineTo(at.x + s * 0.55f, at.y - s * 0.05f)
        lineTo(at.x + s * 0.05f, at.y - s * 0.05f)
        close()
    }
    drawPath(bolt, color)
    drawPath(bolt, Color.Black.copy(alpha = 0.35f), style = Stroke(s * 0.12f))
}

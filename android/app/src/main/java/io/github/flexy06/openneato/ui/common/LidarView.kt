package io.github.flexy06.openneato.ui.common

import androidx.compose.foundation.Canvas
import androidx.compose.material3.MaterialTheme
import androidx.compose.runtime.Composable
import androidx.compose.runtime.LaunchedEffect
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.setValue
import androidx.compose.ui.Modifier
import androidx.compose.ui.geometry.Offset
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.graphics.Path
import androidx.compose.ui.graphics.StrokeCap
import androidx.compose.ui.graphics.StrokeJoin
import androidx.compose.ui.graphics.drawscope.Stroke
import androidx.compose.ui.unit.dp
import io.github.flexy06.openneato.api.models.LidarScan
import kotlin.math.cos
import kotlin.math.min
import kotlin.math.sin

private val WallColor = Color(0xFF7CB496) // Same sage as the web LIDAR map

fun LidarScan.readings(): List<LidarReading> =
    points.map { LidarReading(it.angle.toInt(), it.dist.toInt(), it.error.toInt()) }

// Live LIDAR view centred on the robot (forward = up), 1 m rings, walls from the last scans
@Composable
fun LidarView(scan: LidarScan?, moving: Boolean, modifier: Modifier = Modifier) {
    val acc = remember { LidarAccumulator() }
    var segments by remember { mutableStateOf(emptyList<WallSegment>()) }
    LaunchedEffect(scan, moving) {
        acc.merge(scan?.readings(), moving)
        segments = acc.segments()
    }
    val bg = MaterialTheme.colorScheme.surfaceContainer
    val grid = MaterialTheme.colorScheme.outlineVariant
    val robot = MaterialTheme.colorScheme.onSurfaceVariant

    Canvas(modifier) {
        val size = min(this.size.width, this.size.height)
        val cx = this.size.width / 2
        val cy = this.size.height / 2
        val scale = (size / 2 - 8.dp.toPx()) / LIDAR_MAX_RANGE_MM
        drawRect(bg)
        for (r in 1000..LIDAR_MAX_RANGE_MM step 1000) {
            drawCircle(grid, radius = r * scale, center = Offset(cx, cy), style = Stroke(1.dp.toPx()))
        }
        drawLine(grid, Offset(cx, cy - size / 2 + 4), Offset(cx, cy + size / 2 - 4), 1.dp.toPx())
        drawLine(grid, Offset(cx - size / 2 + 4, cy), Offset(cx + size / 2 - 4, cy), 1.dp.toPx())

        fun toCanvas(angle: Int, dist: Double): Offset {
            val rad = Math.toRadians(90.0 + angle)
            return Offset((cx + dist * scale * cos(rad)).toFloat(), (cy - dist * scale * sin(rad)).toFloat())
        }
        val stroke = Stroke(2.5.dp.toPx(), cap = StrokeCap.Round, join = StrokeJoin.Round)
        for (seg in segments) {
            val path = Path()
            seg.points.forEachIndexed { i, p ->
                val o = toCanvas(p.angle, p.dist)
                if (i == 0) path.moveTo(o.x, o.y) else path.lineTo(o.x, o.y)
            }
            drawPath(path, WallColor.copy(alpha = seg.alpha), style = stroke)
        }

        // Robot: triangle pointing forward
        val u = 1.dp.toPx()
        drawPath(
            Path().apply {
                moveTo(cx, cy - 8 * u)
                lineTo(cx - 5 * u, cy + 4 * u)
                lineTo(cx + 5 * u, cy + 4 * u)
                close()
            },
            robot,
        )
    }
}

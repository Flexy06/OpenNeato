package io.github.flexy06.openneato.ui.common

import kotlin.math.abs
import kotlin.math.max
import kotlin.math.min

// Port of frontend/src/components/lidar-map.tsx: merges the last few scans per angle,
// splits the 360 readings into wall segments and smooths them for drawing.

const val LIDAR_MAX_RANGE_MM = 5000
private const val MAX_DIST_MM = 6000 // Reject readings above this (LDS max range + margin)
private const val MAX_SCAN_AGE = 5 // Keep points from the last N scans
private const val MAX_BRIDGE_GAP = 5 // Bridge up to N missing angles between valid points
private const val MAX_DIST_JUMP_PCT = 0.3 // Max distance jump as fraction of the average distance
private const val SMOOTH_WINDOW = 5 // Moving-average half-window
private const val MIN_SEGMENT_LEN = 3 // Shorter segments are not drawn

data class LidarReading(val angle: Int, val dist: Int, val error: Int)

data class SegmentPoint(val angle: Int, val dist: Double, val age: Int)

// A smoothed wall polyline and its opacity (older scans fade out)
data class WallSegment(val points: List<SegmentPoint>, val alpha: Float)

class LidarAccumulator {
    private class Slot(var dist: Int, var age: Int)

    private val slots = arrayOfNulls<Slot>(360)

    // moving: drop older scans, they would draw ghost walls from earlier positions
    fun merge(readings: List<LidarReading>?, moving: Boolean) {
        if (moving) {
            slots.fill(null)
        } else {
            for (i in 0 until 360) {
                val s = slots[i] ?: continue
                s.age++
                if (s.age > MAX_SCAN_AGE) slots[i] = null
            }
        }
        readings?.forEach { p ->
            if (p.error != 0 || p.dist == 0 || p.dist > MAX_DIST_MM || p.angle !in 0 until 360) return@forEach
            slots[p.angle] = Slot(p.dist, 0)
        }
    }

    fun segments(): List<WallSegment> {
        fun same(a: Double, b: Double) = abs(a - b) < (a + b) / 2 * MAX_DIST_JUMP_PCT

        val segs = mutableListOf<MutableList<SegmentPoint>>()
        var cur = mutableListOf<SegmentPoint>()
        for (i in 0 until 360) {
            val pt = slots[i]
            if (pt == null) {
                val bridged = cur.isNotEmpty() && (1..MAX_BRIDGE_GAP).any { gap ->
                    val next = slots[(i + gap) % 360]
                    next != null && same(cur.last().dist, next.dist.toDouble())
                }
                if (!bridged && cur.isNotEmpty()) {
                    segs += cur
                    cur = mutableListOf()
                }
                continue
            }
            if (cur.isNotEmpty() && !same(cur.last().dist, pt.dist.toDouble())) {
                segs += cur
                cur = mutableListOf()
            }
            cur += SegmentPoint(i, pt.dist.toDouble(), pt.age)
        }
        if (cur.isNotEmpty()) segs += cur

        // Join the segments that meet at 0°/360°
        if (segs.size >= 2) {
            val first = segs.first()
            val last = segs.last()
            if (last.last().angle >= 355 && first.first().angle <= 4 && same(last.last().dist, first.first().dist)) {
                segs[0] = (last + first).toMutableList()
                segs.removeAt(segs.size - 1)
            }
        }

        return segs.filter { it.size >= MIN_SEGMENT_LEN }.map { seg ->
            val smoothed = seg.mapIndexed { i, p ->
                val from = max(0, i - SMOOTH_WINDOW)
                val to = min(seg.size - 1, i + SMOOTH_WINDOW)
                p.copy(dist = (from..to).sumOf { seg[it].dist } / (to - from + 1))
            }
            val avgAge = seg.sumOf { it.age }.toDouble() / seg.size
            WallSegment(smoothed, ((1.0 - avgAge / MAX_SCAN_AGE * 0.7) * 0.7).toFloat())
        }
    }
}

package io.github.flexy06.openneato.ui.manual

import io.github.flexy06.openneato.R
import io.github.flexy06.openneato.api.models.ManualStatus
import kotlin.math.abs
import kotlin.math.roundToInt

// Same rules as frontend/src/views/manual.tsx.

// The robot keeps moving until the next command: a long fixed distance, the firmware
// watchdog stops the wheels when no API request arrives for 2 s.
const val MOVE_DIST_MM = 10000
const val MAX_SPEED_MM_S = 200

data class WheelCommand(val left: Int, val right: Int, val speed: Int) {
    val isStop get() = left == 0 && right == 0
}

// x: -1 (left) .. 1 (right), y: -1 (back) .. 1 (forward), magnitude 0..1
fun joystickToWheels(x: Float, y: Float, magnitude: Float): WheelCommand {
    if (magnitude == 0f) return WheelCommand(0, 0, 0)
    val speed = (magnitude * MAX_SPEED_MM_S).roundToInt()
    val fwd = y * MOVE_DIST_MM
    val turn = x * MOVE_DIST_MM
    val left = (fwd + turn).coerceIn(-MOVE_DIST_MM.toFloat(), MOVE_DIST_MM.toFloat()).roundToInt()
    val right = (fwd - turn).coerceIn(-MOVE_DIST_MM.toFloat(), MOVE_DIST_MM.toFloat()).roundToInt()
    return WheelCommand(left, right, speed)
}

// Skip a move that hardly differs from the last one sent (~5 % of the distance range)
fun worthSending(next: WheelCommand, prev: WheelCommand?): Boolean =
    prev == null || abs(next.left - prev.left) >= 500 || abs(next.right - prev.right) >= 500 ||
        abs(next.speed - prev.speed) >= 10

sealed interface SafetyWarning {
    data class Simple(val text: Int) : SafetyWarning
    data class Bumper(val sides: List<Int>) : SafetyWarning
}

fun safetyWarning(s: ManualStatus?): SafetyWarning? {
    if (s == null) return null
    if (s.lifted) return SafetyWarning.Simple(R.string.warn_lifted)
    if (s.stallFront) return SafetyWarning.Simple(R.string.warn_stall_front)
    if (s.stallRear) return SafetyWarning.Simple(R.string.warn_stall_rear)
    val sides = buildList {
        if (s.bumperFrontLeft) add(R.string.bumper_front_left)
        if (s.bumperFrontRight) add(R.string.bumper_front_right)
        if (s.bumperSideLeft) add(R.string.bumper_side_left)
        if (s.bumperSideRight) add(R.string.bumper_side_right)
    }
    return if (sides.isEmpty()) null else SafetyWarning.Bumper(sides)
}

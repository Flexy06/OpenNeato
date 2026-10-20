package io.github.flexy06.openneato.ui.manual

import io.github.flexy06.openneato.R
import io.github.flexy06.openneato.api.models.ManualStatus
import io.github.flexy06.openneato.ui.common.LidarAccumulator
import io.github.flexy06.openneato.ui.common.LidarReading
import io.github.flexy06.openneato.ui.dashboard.Action
import io.github.flexy06.openneato.ui.dashboard.RobotFlags
import io.github.flexy06.openneato.ui.dashboard.actionButtons
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test

class ManualLogicTest {
    @Test
    fun joystickMapsToDifferentialWheels() {
        assertEquals(WheelCommand(10000, 10000, 200), joystickToWheels(0f, 1f, 1f))
        assertEquals(WheelCommand(5000, -5000, 100), joystickToWheels(0.5f, 0f, 0.5f))
        assertEquals(WheelCommand(10000, 0, 141), joystickToWheels(0.5f, 0.5f, 0.707f))
        assertTrue(joystickToWheels(0.3f, 0.3f, 0f).isStop)
    }

    @Test
    fun smallChangesAreNotResent() {
        val a = WheelCommand(10000, 10000, 200)
        assertTrue(worthSending(a, null))
        assertFalse(worthSending(WheelCommand(9800, 10000, 195), a))
        assertTrue(worthSending(WheelCommand(9000, 10000, 200), a))
    }

    private fun status(lifted: Boolean = false, stallRear: Boolean = false, bumperFrontLeft: Boolean = false) =
        ManualStatus(
            active = true, brush = false, vacuum = false, sideBrush = false, lifted = lifted,
            bumperFrontLeft = bumperFrontLeft, bumperFrontRight = false, bumperSideLeft = false,
            bumperSideRight = false, stallFront = false, stallRear = stallRear,
        )

    @Test
    fun safetyWarningsInPriorityOrder() {
        assertNull(safetyWarning(status()))
        assertEquals(SafetyWarning.Simple(R.string.warn_lifted), safetyWarning(status(lifted = true, stallRear = true)))
        assertEquals(SafetyWarning.Simple(R.string.warn_stall_rear), safetyWarning(status(stallRear = true)))
        assertEquals(
            SafetyWarning.Bumper(listOf(R.string.bumper_front_left)),
            safetyWarning(status(bumperFrontLeft = true)),
        )
    }

    @Test
    fun manualModeOnlyOffersTheDrivingScreen() {
        val b = actionButtons(RobotFlags.of("UIMGR_STATE_MANUALCLEANING"), false, true, false, false)
        assertEquals(listOf(Action.MANUAL), b.map { it.action })
    }

    @Test
    fun lidarWallBecomesOneSegmentAcrossSmallGaps() {
        val acc = LidarAccumulator()
        // A wall 1 m away from 10° to 40°, with two missing angles in between
        val wall = (10..40).filter { it != 20 && it != 21 }.map { LidarReading(it, 1000, 0) }
        acc.merge(wall + LidarReading(200, 0, 0) + LidarReading(201, 1500, 1), moving = false)
        val segs = acc.segments()
        assertEquals(1, segs.size)
        assertEquals(10, segs[0].points.first().angle)
        assertEquals(40, segs[0].points.last().angle)
    }

    @Test
    fun lidarOldScansFadeAndExpire() {
        val acc = LidarAccumulator()
        acc.merge((0..9).map { LidarReading(it, 2000, 0) }, moving = false)
        val fresh = acc.segments().single().alpha
        acc.merge(emptyList(), moving = false)
        assertTrue(acc.segments().single().alpha < fresh)
        repeat(5) { acc.merge(emptyList(), moving = false) }
        assertTrue(acc.segments().isEmpty())
    }
}

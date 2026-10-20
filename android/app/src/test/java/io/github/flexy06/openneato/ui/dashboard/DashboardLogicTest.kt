package io.github.flexy06.openneato.ui.dashboard

import io.github.flexy06.openneato.R
import io.github.flexy06.openneato.data.RobotClient
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertTrue
import org.junit.Test

class DashboardLogicTest {
    @Test
    fun statusFollowsTheWebRules() {
        assertEquals(R.string.status_ready, statusInfo("UIMGR_STATE_STANDBY").label)
        assertEquals(R.string.status_paused, statusInfo("UIMGR_STATE_HOUSECLEANINGPAUSED").label)
        assertEquals(R.string.status_room_cleaning, statusInfo("UIMGR_STATE_GUIDEDCLEANING").label)
        assertEquals(R.string.status_docking, statusInfo("UIMGR_STATE_DOCKING").label)
    }

    @Test
    fun idleOnDockOffersHouseSpotButNotDock() {
        val b = actionButtons(RobotFlags.of("UIMGR_STATE_IDLE"), docked = true, ready = true, busy = false, robotError = false)
        assertEquals(listOf(Action.HOUSE, Action.SPOT, Action.DOCK), b.map { it.action })
        assertTrue(b[0].enabled)
        assertFalse(b[2].enabled)
    }

    @Test
    fun pausedOffersResume() {
        val b = actionButtons(
            RobotFlags.of("UIMGR_STATE_HOUSECLEANINGPAUSED"),
            docked = false,
            ready = true,
            busy = false,
            robotError = false,
        )
        assertEquals(listOf(Action.RESUME, Action.DOCK, Action.STOP), b.map { it.action })
    }

    @Test
    fun robotErrorBlocksStartingButNotDocking() {
        val b = actionButtons(RobotFlags.of("UIMGR_STATE_IDLE"), docked = false, ready = true, busy = false, robotError = true)
        assertFalse(b[0].enabled)
        assertTrue(b[2].enabled)
    }

    @Test
    fun guidedRunGoesThroughTheGuidedEndpoints() {
        val b = actionButtons(RobotFlags.of("UIMGR_STATE_GUIDEDCLEANING"), false, true, false, false)
        assertEquals(listOf(Action.GUIDED_HOME, Action.GUIDED_STOP), b.map { it.action })
    }

    @Test
    fun batteryAndWifiThresholds() {
        assertEquals(Tone.RED, batteryTone(25))
        assertEquals(Tone.AMBER, batteryTone(50))
        assertEquals(Tone.GREEN, batteryTone(51))
        assertEquals(R.string.wifi_good, wifiStrength(-55))
    }

    @Test
    fun clockAndUrlParsing() {
        assertEquals("08:12", clockOf("2026-10-20 08:12:03"))
        assertEquals("http://192.168.1.42", RobotClient.normalizeUrl(" 192.168.1.42/ "))
        assertEquals("http://neato.local:8080", RobotClient.normalizeUrl("http://neato.local:8080"))
    }
}

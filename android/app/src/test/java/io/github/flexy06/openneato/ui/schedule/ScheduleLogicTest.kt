package io.github.flexy06.openneato.ui.schedule

import kotlinx.serialization.json.Json
import kotlinx.serialization.json.JsonPrimitive
import kotlinx.serialization.json.jsonObject
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertTrue
import org.junit.Test

class ScheduleLogicTest {
    private val settings = Json.parseToJsonElement(
        """{"tz":"CET-1CEST","scheduleEnabled":true,
            "sched0Hour":9,"sched0Min":30,"sched0On":true,
            "sched0Slot1Hour":15,"sched0Slot1Min":0,"sched0Slot1On":true,
            "sched1Hour":8,"sched1Min":0,"sched1On":false,
            "sched1Slot1Hour":18,"sched1Slot1Min":0,"sched1Slot1On":true}""",
    ).jsonObject

    @Test
    fun readsBothSlotsAndDropsASecondSlotWithoutFirst() {
        val s = readSchedule(settings)
        assertTrue(s.enabled)
        assertEquals(Slot(9, 30, true), s.days[0].slots[0])
        assertEquals(Slot(15, 0, true), s.days[0].slots[1])
        assertFalse("slot 1 needs slot 0", s.days[1].slots[1].on)
        assertEquals(Slot(), s.days[6].slots[0]) // Missing keys default to off
    }

    @Test
    fun patchHoldsOnlyChangedKeys() {
        val server = readSchedule(settings)
        assertTrue(schedulePatch(server, server).isEmpty())

        val edited = server.updateSlot(0, 0) { it.copy(hour = 10) }.copy(enabled = false)
        val patch = schedulePatch(edited, server)
        assertEquals(setOf("sched0Hour", "scheduleEnabled"), patch.keys)
        assertEquals(JsonPrimitive(10), patch["sched0Hour"])
    }

    @Test
    fun switchingADayOffAlsoTurnsOffItsSecondSlot() {
        val server = readSchedule(settings)
        val edited = server.updateSlot(0, 0) { it.copy(on = false) }
        val patch = schedulePatch(edited, server)
        assertEquals(setOf("sched0On", "sched0Slot1On"), patch.keys)
    }
}

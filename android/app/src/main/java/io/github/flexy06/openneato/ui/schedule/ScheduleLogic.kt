package io.github.flexy06.openneato.ui.schedule

import kotlinx.serialization.json.JsonObject
import kotlinx.serialization.json.JsonPrimitive
import kotlinx.serialization.json.booleanOrNull
import kotlinx.serialization.json.buildJsonObject
import kotlinx.serialization.json.intOrNull
import kotlinx.serialization.json.jsonPrimitive

// Same rules as frontend/src/views/schedule.tsx: 7 days (0 = Monday), two slots per day,
// the second slot only counts while the first is on. Settings keys: sched<d>Hour/Min/On
// and sched<d>Slot1Hour/Min/On.

const val SLOTS_PER_DAY = 2

data class Slot(val hour: Int = 0, val minute: Int = 0, val on: Boolean = false)

data class Day(val slots: List<Slot> = List(SLOTS_PER_DAY) { Slot() })

data class Schedule(val enabled: Boolean = false, val days: List<Day> = List(7) { Day() })

private fun prefix(day: Int, slot: Int) = if (slot == 0) "sched$day" else "sched${day}Slot$slot"

fun normalize(days: List<Day>): List<Day> = days.map { d ->
    val (s0, s1) = d.slots
    Day(listOf(s0, s1.copy(on = s0.on && s1.on)))
}

fun readSchedule(settings: JsonObject): Schedule {
    fun int(key: String) = settings[key]?.jsonPrimitive?.intOrNull ?: 0
    fun bool(key: String) = settings[key]?.jsonPrimitive?.booleanOrNull ?: false
    val days = (0 until 7).map { d ->
        Day((0 until SLOTS_PER_DAY).map { s -> Slot(int(prefix(d, s) + "Hour"), int(prefix(d, s) + "Min"), bool(prefix(d, s) + "On")) })
    }
    return Schedule(bool("scheduleEnabled"), normalize(days))
}

// Only the keys that differ from what the robot has (a settings save writes NVS)
fun schedulePatch(edited: Schedule, server: Schedule): JsonObject = buildJsonObject {
    val days = normalize(edited.days)
    for (d in 0 until 7) {
        for (s in 0 until SLOTS_PER_DAY) {
            val cur = days[d].slots[s]
            val srv = server.days[d].slots[s]
            val p = prefix(d, s)
            if (cur.hour != srv.hour) put(p + "Hour", JsonPrimitive(cur.hour))
            if (cur.minute != srv.minute) put(p + "Min", JsonPrimitive(cur.minute))
            if (cur.on != srv.on) put(p + "On", JsonPrimitive(cur.on))
        }
    }
    if (edited.enabled != server.enabled) put("scheduleEnabled", JsonPrimitive(edited.enabled))
}

fun Schedule.updateSlot(day: Int, slot: Int, f: (Slot) -> Slot): Schedule = copy(
    days = normalize(
        days.mapIndexed { i, d -> if (i != day) d else Day(d.slots.mapIndexed { si, s -> if (si == slot) f(s) else s }) },
    ),
)

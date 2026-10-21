package io.github.flexy06.openneato.data

import io.github.flexy06.openneato.ui.history.HistoryImport
import io.github.flexy06.openneato.ui.history.ImportProblem
import org.junit.Assert.assertEquals
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test

class HistoryDataTest {
    private val session = """
        {"type":"session","mode":"house","time":1792240008,"battery":86,"scans":true}
        {"x":0.000,"y":0.000,"t":0.0,"ts":60185.0}
        {"x":0.100,"y":0.000,"t":0.0,"ts":60187.0}
        {"type":"scan","x":0.100,"y":0.000,"t":0.0,"d":"${"A".repeat(240)}"}
        {"x":0.300,"y":0.000,"t":90.0,"ts":60189.0}
        {"x":0:500,"y":0.000,"t":90.0,"ts":60191.0}
        {"type":"recharge","x":0.5,"y":0.0}
        {"x":0.500,"y":0.200,"t":90.0,"ts":60300.0}
        {"type":"summary","time":1792240400,"duration":115,"mode":"house","recharges":1,"snapshots":5,"distanceTraveled":0.7,"maxDistFromOrigin":0.54,"totalRotation":90,"areaCovered":0.3,"errorsDuringClean":0,"batteryEnd":70}
    """.trimIndent()

    @Test
    fun parsesPosesRelativeToTheFirstAndRepairsDamagedNumbers() {
        val m = HistoryData.parse(session).single()
        // 0,0,0 is dropped (no pose yet), the damaged "0:500" is repaired to 0.5
        assertEquals(listOf(0.1, 0.3, 0.5, 0.5), m.path.map { it.x })
        assertEquals(0.0, m.path.first().ts, 1e-9)
        assertEquals(113.0, m.path.last().ts, 1e-9)
        assertEquals(115.0, m.duration, 1e-9)
        assertEquals(1, m.scans.size)
        assertEquals("house", m.session?.mode)
        assertEquals(70, m.summary?.batteryEnd)
    }

    @Test
    fun rechargeTakesTheLongGapAfterItsMarker() {
        val r = HistoryData.parse(session).single().recharges.single()
        assertEquals(4.0, r.ts, 1e-9) // Last pose before the 109 s pause
        assertEquals(113.0, r.endTs, 1e-9)
    }

    @Test
    fun poseAtInterpolatesAlongTheShortArc() {
        val path = listOf(PathPoint(0.0, 0.0, 350.0, 0.0), PathPoint(1.0, 0.0, 10.0, 2.0))
        val p = HistoryData.poseAt(path, 1.0)!!
        assertEquals(0.5, p.x, 1e-9)
        assertEquals(360.0, p.t, 1e-9) // Through 0°, not back through 180°
    }

    @Test
    fun splitsSeveralSessionsAndSkipsEmptyOnes() {
        val two = session + "\n" + """{"type":"session","mode":"spot","time":1,"battery":50}"""
        assertEquals(1, HistoryData.parse(two).size)
    }

    @Test
    fun importChecksLikeTheWebUi() {
        val bytes = session.toByteArray()
        assertNull(HistoryImport.check("1792240008.jsonl", bytes, emptyList()))
        assertEquals(ImportProblem.NOT_JSONL, HistoryImport.check("x.txt", bytes, emptyList()))
        assertEquals(ImportProblem.NAME_MISMATCH, HistoryImport.check("1.jsonl", bytes, emptyList()))
        assertEquals(
            ImportProblem.EXISTS,
            HistoryImport.check("1792240008.jsonl", bytes, listOf("1792240008.jsonl.hs")),
        )
        assertEquals(ImportProblem.NO_HEADER, HistoryImport.check("1792240008.jsonl", "{}".toByteArray(), emptyList()))
        assertNotNull(HistoryData.parseHeader(session.lineSequence().first()))
        assertTrue(HistoryData.parseHeader("""{"x":1}""") == null)
    }
}

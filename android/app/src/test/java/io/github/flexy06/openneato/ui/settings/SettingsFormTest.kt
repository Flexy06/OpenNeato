package io.github.flexy06.openneato.ui.settings

import kotlinx.serialization.json.Json
import kotlinx.serialization.json.JsonPrimitive
import kotlinx.serialization.json.jsonObject
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test

class SettingsFormTest {
    private val server = BridgeSettings.from(
        Json.parseToJsonElement(
            """{"tz":"CET-1CEST,M3.5.0,M10.5.0/3","logLevel":0,"hostname":"neato","uartTxPin":3,"uartRxPin":4,
                "maxGpioPin":21,"brushRpm":1200,"ntfyTopic":"","ntfyEnabled":false,"sched0Hour":9}""",
        ).jsonObject,
    )

    @Test
    fun readsKnownKeysAndDefaultsTheRest() {
        assertEquals("CET-1CEST,M3.5.0,M10.5.0/3", server.tz)
        assertEquals(80, server.vacuumSpeed) // Missing: default
        assertEquals(21, server.maxGpioPin)
    }

    @Test
    fun patchHoldsOnlyChangesAndKnowsWhenARestartIsNeeded() {
        assertTrue(settingsPatch(server, server).isEmpty())
        val edited = server.copy(brushRpm = 1400, ntfyEnabled = true, ntfyTopic = " my-topic ")
        val patch = settingsPatch(edited, server)
        assertEquals(setOf("brushRpm", "ntfyEnabled", "ntfyTopic"), patch.keys)
        assertEquals(JsonPrimitive("my-topic"), patch["ntfyTopic"]) // Trimmed
        assertFalse(needsReboot(edited, server))
        assertTrue(needsReboot(server.copy(hostname = "kitchen"), server))
        assertTrue(needsReboot(server.copy(uartRxPin = 5), server))
    }

    @Test
    fun validatesLikeTheWebUi() {
        assertNull(server.problem())
        assertEquals(SettingsProblem.PINS_SAME, server.copy(uartRxPin = 3).problem())
        assertEquals(SettingsProblem.PIN_RANGE, server.copy(uartTxPin = 22).problem())
        assertEquals(SettingsProblem.HOSTNAME_CHARS, server.copy(hostname = "my robot").problem())
        assertEquals(SettingsProblem.HOSTNAME_EMPTY, server.copy(hostname = "").problem())
        assertEquals(SettingsProblem.SYSLOG_IP_MISSING, server.copy(syslogEnabled = true).problem())
        assertEquals(SettingsProblem.SYSLOG_IP_INVALID, server.copy(syslogEnabled = true, syslogIp = "192.168.1.300").problem())
        assertNull(server.copy(syslogEnabled = true, syslogIp = "192.168.1.30").problem())
    }

    @Test
    fun firmwareFileChecks() {
        val bin = ByteArray(32).also { it[12] = 5 }
        assertEquals("ESP32-C3", FirmwareFile.chip(bin))
        assertNull(FirmwareFile.chip(ByteArray(32).also { it[12] = 99 }))
        assertEquals("openneato-esp32-c3-firmware.bin", FirmwareFile.releaseName("ESP32-C3"))
        val sums = FirmwareFile.parseChecksums(
            "${"a".repeat(64)}  openneato-esp32-firmware.bin\n${"B".repeat(64)}  OpenNeato-android.apk\n",
        )
        assertEquals("a".repeat(64), sums["openneato-esp32-firmware.bin"])
        assertEquals("b".repeat(64), sums["openneato-android.apk"])
    }
}

package io.github.flexy06.openneato.ui.settings

import kotlinx.serialization.json.JsonObject
import kotlinx.serialization.json.JsonPrimitive
import kotlinx.serialization.json.booleanOrNull
import kotlinx.serialization.json.buildJsonObject
import kotlinx.serialization.json.contentOrNull
import kotlinx.serialization.json.intOrNull
import kotlinx.serialization.json.jsonPrimitive

// Bridge settings as edited in the app (same fields, defaults and checks as
// frontend/src/views/settings/use-settings-form.ts). Saving sends only what changed.
data class BridgeSettings(
    val tz: String = "UTC0",
    val logLevel: Int = 0,
    val apFallbackOnDisconnect: Boolean = true,
    val syslogEnabled: Boolean = false,
    val syslogIp: String = "",
    val wifiTxPower: Int = 60,
    val uartTxPin: Int = 3,
    val uartRxPin: Int = 4,
    val maxGpioPin: Int = 21,
    val hostname: String = "neato",
    val navMode: String = "Normal",
    val stallThreshold: Int = 60,
    val brushRpm: Int = 1200,
    val vacuumSpeed: Int = 80,
    val sideBrushPower: Int = 1500,
    val ntfyTopic: String = "",
    val ntfyEnabled: Boolean = false,
    val ntfyOnDone: Boolean = true,
    val ntfyOnError: Boolean = true,
    val ntfyOnAlert: Boolean = true,
    val ntfyOnDocking: Boolean = true,
    val autoRestartEnabled: Boolean = false,
    val autoRestartHour: Int = 3,
    val autoRestartMinute: Int = 0,
    val restartBeforeClean: Boolean = false,
) {
    companion object {
        fun from(json: JsonObject): BridgeSettings {
            val d = BridgeSettings()
            fun int(k: String, def: Int) = json[k]?.jsonPrimitive?.intOrNull ?: def
            fun bool(k: String, def: Boolean) = json[k]?.jsonPrimitive?.booleanOrNull ?: def
            fun str(k: String, def: String) = json[k]?.jsonPrimitive?.contentOrNull ?: def
            return BridgeSettings(
                tz = str("tz", d.tz),
                logLevel = int("logLevel", d.logLevel),
                apFallbackOnDisconnect = bool("apFallbackOnDisconnect", d.apFallbackOnDisconnect),
                syslogEnabled = bool("syslogEnabled", d.syslogEnabled),
                syslogIp = str("syslogIp", d.syslogIp),
                wifiTxPower = int("wifiTxPower", d.wifiTxPower),
                uartTxPin = int("uartTxPin", d.uartTxPin),
                uartRxPin = int("uartRxPin", d.uartRxPin),
                maxGpioPin = int("maxGpioPin", d.maxGpioPin),
                hostname = str("hostname", d.hostname),
                navMode = str("navMode", d.navMode),
                stallThreshold = int("stallThreshold", d.stallThreshold),
                brushRpm = int("brushRpm", d.brushRpm),
                vacuumSpeed = int("vacuumSpeed", d.vacuumSpeed),
                sideBrushPower = int("sideBrushPower", d.sideBrushPower),
                ntfyTopic = str("ntfyTopic", d.ntfyTopic),
                ntfyEnabled = bool("ntfyEnabled", d.ntfyEnabled),
                ntfyOnDone = bool("ntfyOnDone", d.ntfyOnDone),
                ntfyOnError = bool("ntfyOnError", d.ntfyOnError),
                ntfyOnAlert = bool("ntfyOnAlert", d.ntfyOnAlert),
                ntfyOnDocking = bool("ntfyOnDocking", d.ntfyOnDocking),
                autoRestartEnabled = bool("autoRestartEnabled", d.autoRestartEnabled),
                autoRestartHour = int("autoRestartHour", d.autoRestartHour),
                autoRestartMinute = int("autoRestartMinute", d.autoRestartMinute),
                restartBeforeClean = bool("restartBeforeClean", d.restartBeforeClean),
            )
        }
    }
}

enum class SettingsProblem { PINS_SAME, PIN_RANGE, HOSTNAME_EMPTY, HOSTNAME_LONG, HOSTNAME_CHARS, SYSLOG_IP_MISSING, SYSLOG_IP_INVALID }

private val HOSTNAME_RE = Regex("^[a-zA-Z0-9-]+$")
private val IPV4_RE = Regex("""^(\d{1,3}\.){3}\d{1,3}$""")

fun BridgeSettings.pinProblem(): SettingsProblem? = when {
    uartTxPin == uartRxPin -> SettingsProblem.PINS_SAME
    uartTxPin !in 0..maxGpioPin || uartRxPin !in 0..maxGpioPin -> SettingsProblem.PIN_RANGE
    else -> null
}

fun BridgeSettings.hostnameProblem(): SettingsProblem? = when {
    hostname.isEmpty() -> SettingsProblem.HOSTNAME_EMPTY
    hostname.length > 32 -> SettingsProblem.HOSTNAME_LONG
    !HOSTNAME_RE.matches(hostname) -> SettingsProblem.HOSTNAME_CHARS
    else -> null
}

fun BridgeSettings.syslogProblem(): SettingsProblem? {
    if (!syslogEnabled) return null
    val ip = syslogIp.trim()
    return when {
        ip.isEmpty() -> SettingsProblem.SYSLOG_IP_MISSING
        !IPV4_RE.matches(ip) || ip.split(".").any { it.toInt() > 255 } -> SettingsProblem.SYSLOG_IP_INVALID
        else -> null
    }
}

fun BridgeSettings.problem(): SettingsProblem? = pinProblem() ?: hostnameProblem() ?: syslogProblem()

// UART pins and the hostname are only applied by a restart of the bridge
fun needsReboot(edited: BridgeSettings, server: BridgeSettings) =
    edited.uartTxPin != server.uartTxPin || edited.uartRxPin != server.uartRxPin || edited.hostname != server.hostname

/** The keys that differ from what the bridge has (maxGpioPin is read-only) */
fun settingsPatch(edited: BridgeSettings, server: BridgeSettings): JsonObject = buildJsonObject {
    fun <T> diff(key: String, a: T, b: T, v: (T) -> JsonPrimitive) {
        if (a != b) put(key, v(a))
    }
    val s = { x: String -> JsonPrimitive(x) }
    val i = { x: Int -> JsonPrimitive(x) }
    val b = { x: Boolean -> JsonPrimitive(x) }
    diff("tz", edited.tz, server.tz, s)
    diff("logLevel", edited.logLevel, server.logLevel, i)
    diff("apFallbackOnDisconnect", edited.apFallbackOnDisconnect, server.apFallbackOnDisconnect, b)
    diff("syslogEnabled", edited.syslogEnabled, server.syslogEnabled, b)
    diff("syslogIp", edited.syslogIp.trim(), server.syslogIp, s)
    diff("wifiTxPower", edited.wifiTxPower, server.wifiTxPower, i)
    diff("uartTxPin", edited.uartTxPin, server.uartTxPin, i)
    diff("uartRxPin", edited.uartRxPin, server.uartRxPin, i)
    diff("hostname", edited.hostname, server.hostname, s)
    diff("navMode", edited.navMode, server.navMode, s)
    diff("stallThreshold", edited.stallThreshold, server.stallThreshold, i)
    diff("brushRpm", edited.brushRpm, server.brushRpm, i)
    diff("vacuumSpeed", edited.vacuumSpeed, server.vacuumSpeed, i)
    diff("sideBrushPower", edited.sideBrushPower, server.sideBrushPower, i)
    diff("ntfyTopic", edited.ntfyTopic.trim(), server.ntfyTopic, s)
    diff("ntfyEnabled", edited.ntfyEnabled, server.ntfyEnabled, b)
    diff("ntfyOnDone", edited.ntfyOnDone, server.ntfyOnDone, b)
    diff("ntfyOnError", edited.ntfyOnError, server.ntfyOnError, b)
    diff("ntfyOnAlert", edited.ntfyOnAlert, server.ntfyOnAlert, b)
    diff("ntfyOnDocking", edited.ntfyOnDocking, server.ntfyOnDocking, b)
    diff("autoRestartEnabled", edited.autoRestartEnabled, server.autoRestartEnabled, b)
    diff("autoRestartHour", edited.autoRestartHour, server.autoRestartHour, i)
    diff("autoRestartMinute", edited.autoRestartMinute, server.autoRestartMinute, i)
    diff("restartBeforeClean", edited.restartBeforeClean, server.restartBeforeClean, b)
}

// -- Choices (frontend/src/views/settings/constants.ts) --------------------------------

data class Choice<T>(val value: T, val label: String, val note: ChoiceNote? = null)

enum class ChoiceNote { RECOMMENDED, ECO, HIGH, TURBO, LOW_POWER, MAX_RANGE, VERY_SENSITIVE, LESS_SENSITIVE }

val TX_POWER_CHOICES = listOf(
    Choice(34, "8.5 dBm", ChoiceNote.LOW_POWER),
    Choice(44, "11 dBm"),
    Choice(52, "13 dBm"),
    Choice(60, "15 dBm", ChoiceNote.RECOMMENDED),
    Choice(68, "17 dBm"),
    Choice(78, "19.5 dBm", ChoiceNote.MAX_RANGE),
)

val STALL_CHOICES = listOf(
    Choice(30, "30 %", ChoiceNote.VERY_SENSITIVE),
    Choice(40, "40 %"),
    Choice(50, "50 %"),
    Choice(60, "60 %", ChoiceNote.RECOMMENDED),
    Choice(70, "70 %"),
    Choice(80, "80 %", ChoiceNote.LESS_SENSITIVE),
)

val BRUSH_CHOICES = listOf(
    Choice(800, "800 RPM", ChoiceNote.ECO),
    Choice(1000, "1000 RPM"),
    Choice(1200, "1200 RPM", ChoiceNote.RECOMMENDED),
    Choice(1400, "1400 RPM"),
    Choice(1600, "1600 RPM", ChoiceNote.HIGH),
)

val VACUUM_CHOICES = listOf(
    Choice(40, "40 %", ChoiceNote.ECO),
    Choice(60, "60 %"),
    Choice(80, "80 %", ChoiceNote.RECOMMENDED),
    Choice(90, "90 %"),
    Choice(100, "100 %", ChoiceNote.TURBO),
)

val SIDE_BRUSH_CHOICES = listOf(
    Choice(700, "700 mW"),
    Choice(1000, "1000 mW"),
    Choice(1200, "1200 mW"),
    Choice(1500, "1500 mW", ChoiceNote.RECOMMENDED),
)

// Navigation modes the robot knows (label resources in the screen)
val NAV_MODES = listOf("Normal", "Gentle", "Deep", "Quick")

// POSIX TZ strings; the labels are string resources in the same order (tz_labels)
val TIMEZONES = listOf(
    "UTC0",
    "HST10",
    "AKST9AKDT,M3.2.0,M11.1.0",
    "PST8PDT,M3.2.0,M11.1.0",
    "MST7MDT,M3.2.0,M11.1.0",
    "CST6CDT,M3.2.0,M11.1.0",
    "EST5EDT,M3.2.0,M11.1.0",
    "GMT0BST,M3.5.0/1,M10.5.0",
    "CET-1CEST,M3.5.0,M10.5.0/3",
    "EET-2EEST,M3.5.0/3,M10.5.0/4",
    "<+03>-3",
    "IST-5:30",
    "CST-8",
    "JST-9",
    "AEST-10AEDT,M10.1.0,M4.1.0/3",
    "NZST-12NZDT,M9.5.0,M4.1.0/3",
)

// -- Firmware file checks (frontend/src/views/settings/use-firmware-upload.ts) ---------

object FirmwareFile {
    private val CHIP_IDS = mapOf(
        0 to "ESP32",
        2 to "ESP32-S2",
        5 to "ESP32-C3",
        9 to "ESP32-S3",
        12 to "ESP32-C2",
        13 to "ESP32-C6",
        16 to "ESP32-H2",
    )

    /** Chip from the ESP image header (extended header, byte 12), null when unknown */
    fun chip(bin: ByteArray): String? = if (bin.size < 16) null else CHIP_IDS[bin[12].toInt() and 0xff]

    /** Release asset name for a chip, as listed in checksums.txt */
    fun releaseName(chip: String) = "openneato-${chip.lowercase()}-firmware.bin"

    /** GoReleaser checksums.txt: "<sha256>  <file>" per line -> file (lowercase) to hash */
    fun parseChecksums(text: String): Map<String, String> = text.lineSequence()
        .mapNotNull { Regex("""^([0-9a-fA-F]{64})\s+(.+)$""").matchEntire(it.trim()) }
        .associate { it.groupValues[2].trim().lowercase() to it.groupValues[1].lowercase() }

    fun hex(bytes: ByteArray) = bytes.joinToString("") { "%02x".format(it) }
}

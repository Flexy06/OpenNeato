package io.github.flexy06.openneato.ui.dashboard

import io.github.flexy06.openneato.R

// Same rules as frontend/src/views/dashboard.tsx, kept free of Android types so it can be unit tested.

enum class Tone { GREEN, AMBER, RED, BLUE }

enum class StatusIcon { CHECK, SPARKLE, ALERT, BOLT, MANUAL, MAP, SPOT, HOUSE, IDLE }

data class Badge(val label: Int, val tone: Tone, val icon: StatusIcon)

fun statusInfo(uiState: String): Badge = when {
    "IDLE" in uiState || "STANDBY" in uiState -> Badge(R.string.status_ready, Tone.GREEN, StatusIcon.CHECK)
    "CLEANINGRUNNING" in uiState -> Badge(R.string.status_cleaning, Tone.GREEN, StatusIcon.SPARKLE)
    "CLEANINGPAUSED" in uiState -> Badge(R.string.status_paused, Tone.AMBER, StatusIcon.ALERT)
    "CLEANINGSUSPENDED" in uiState -> Badge(R.string.status_recharging, Tone.AMBER, StatusIcon.BOLT)
    "MANUALCLEANING" in uiState -> Badge(R.string.status_cleaning, Tone.GREEN, StatusIcon.SPARKLE)
    "GUIDEDCLEANING" in uiState -> Badge(R.string.status_room_cleaning, Tone.GREEN, StatusIcon.SPARKLE)
    "DOCKING" in uiState -> Badge(R.string.status_docking, Tone.AMBER, StatusIcon.BOLT)
    else -> Badge(R.string.status_active, Tone.GREEN, StatusIcon.CHECK)
}

data class RobotFlags(
    val running: Boolean = false,
    val paused: Boolean = false,
    val docking: Boolean = false,
    val suspended: Boolean = false,
    val spot: Boolean = false,
    val guided: Boolean = false,
    val manual: Boolean = false,
) {
    val cleaning get() = running || paused || suspended

    companion object {
        fun of(uiState: String?): RobotFlags {
            val s = uiState ?: return RobotFlags()
            return RobotFlags(
                running = "CLEANINGRUNNING" in s,
                paused = "CLEANINGPAUSED" in s,
                docking = "DOCKING" in s,
                suspended = "CLEANINGSUSPENDED" in s,
                spot = "SPOT" in s,
                guided = "GUIDEDCLEANING" in s,
                manual = "MANUALCLEANING" in s,
            )
        }
    }
}

fun modeInfo(charging: Boolean, docked: Boolean, f: RobotFlags): Badge = when {
    f.manual -> Badge(R.string.mode_manual, Tone.BLUE, StatusIcon.MANUAL)
    f.guided -> Badge(R.string.mode_rooms, Tone.BLUE, StatusIcon.MAP)
    charging -> Badge(R.string.mode_charging, Tone.AMBER, StatusIcon.BOLT)
    docked -> Badge(R.string.mode_docked, Tone.AMBER, StatusIcon.BOLT)
    f.spot -> Badge(R.string.mode_spot, Tone.BLUE, StatusIcon.SPOT)
    f.cleaning -> Badge(R.string.mode_house, Tone.BLUE, StatusIcon.HOUSE)
    else -> Badge(R.string.mode_idle, Tone.GREEN, StatusIcon.IDLE)
}

fun batteryTone(pct: Int): Tone = when {
    pct <= 25 -> Tone.RED
    pct <= 50 -> Tone.AMBER
    else -> Tone.GREEN
}

fun wifiStrength(rssi: Int): Int = when {
    rssi >= -50 -> R.string.wifi_excellent
    rssi >= -60 -> R.string.wifi_good
    rssi >= -70 -> R.string.wifi_fair
    else -> R.string.wifi_weak
}

// "2026-10-20 08:12:03" / "Mon Oct 20 08:12:03 2026" -> "08:12"
fun clockOf(localTime: String): String =
    Regex("""(\d{1,2}):(\d{2})""").find(localTime)?.let { "${it.groupValues[1].padStart(2, '0')}:${it.groupValues[2]}" }
        ?: localTime

// The three buttons of the bottom bar, depending on what the robot does
// MANUAL opens the driving screen (the robot is already in manual mode)
enum class Action { HOUSE, SPOT, PAUSE, RESUME, DOCK, STOP, GUIDED_HOME, GUIDED_STOP, MANUAL }

data class ActionButton(val action: Action, val enabled: Boolean, val primary: Boolean = false)

fun actionButtons(f: RobotFlags, docked: Boolean, ready: Boolean, busy: Boolean, robotError: Boolean): List<ActionButton> {
    val ok = ready && !busy
    return when {
        f.manual -> listOf(ActionButton(Action.MANUAL, ready, primary = true))
        f.guided -> listOf(
            ActionButton(Action.GUIDED_HOME, ok, primary = true),
            ActionButton(Action.GUIDED_STOP, ok),
        )
        f.cleaning -> listOf(
            ActionButton(if (f.paused) Action.RESUME else Action.PAUSE, ok, primary = true),
            ActionButton(Action.DOCK, ok),
            ActionButton(Action.STOP, ok),
        )
        else -> {
            val canStart = ok && !f.docking && !f.manual && !robotError
            listOf(
                ActionButton(Action.HOUSE, canStart, primary = true),
                ActionButton(Action.SPOT, canStart),
                if (f.docking) ActionButton(Action.STOP, ok) else ActionButton(Action.DOCK, ok && !docked && !f.manual),
            )
        }
    }
}

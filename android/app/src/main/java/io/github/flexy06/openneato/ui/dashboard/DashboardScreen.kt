package io.github.flexy06.openneato.ui.dashboard

import androidx.compose.foundation.Image
import androidx.compose.foundation.background
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.navigationBarsPadding
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.size
import androidx.compose.foundation.layout.width
import androidx.compose.foundation.rememberScrollState
import androidx.compose.foundation.text.TextAutoSize
import androidx.compose.foundation.shape.CircleShape
import androidx.compose.foundation.verticalScroll
import androidx.compose.material.icons.Icons
import androidx.compose.material.icons.filled.AutoAwesome
import androidx.compose.material.icons.filled.Bolt
import androidx.compose.material.icons.filled.CenterFocusStrong
import androidx.compose.material.icons.filled.CheckCircle
import androidx.compose.material.icons.filled.EvStation
import androidx.compose.material.icons.filled.Gamepad
import androidx.compose.material.icons.filled.History
import androidx.compose.material.icons.filled.Settings
import androidx.compose.material.icons.filled.Home
import androidx.compose.material.icons.filled.Map
import androidx.compose.material.icons.filled.MoreVert
import androidx.compose.material.icons.filled.Pause
import androidx.compose.material.icons.filled.PlayArrow
import androidx.compose.material.icons.filled.Schedule
import androidx.compose.material.icons.filled.Stop
import androidx.compose.material.icons.filled.Warning
import androidx.compose.material.icons.outlined.Bedtime
import androidx.compose.material.icons.outlined.WifiOff
import androidx.compose.material3.Button
import androidx.compose.material3.ButtonDefaults
import androidx.compose.material3.Card
import androidx.compose.material3.CardDefaults
import androidx.compose.material3.DropdownMenu
import androidx.compose.material3.DropdownMenuItem
import androidx.compose.material3.FilledTonalButton
import androidx.compose.material3.Icon
import androidx.compose.material3.IconButton
import androidx.compose.material3.LinearWavyProgressIndicator
import androidx.compose.material3.LoadingIndicator
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.OutlinedButton
import androidx.compose.material3.Scaffold
import androidx.compose.material3.SnackbarHost
import androidx.compose.material3.SnackbarHostState
import androidx.compose.material3.Surface
import androidx.compose.material3.Text
import androidx.compose.material3.TextButton
import androidx.compose.material3.TopAppBar
import androidx.compose.runtime.Composable
import androidx.compose.runtime.LaunchedEffect
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.graphics.ColorFilter
import androidx.compose.ui.graphics.vector.ImageVector
import androidx.compose.ui.platform.LocalContext
import androidx.compose.ui.res.painterResource
import androidx.compose.ui.res.stringResource
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import androidx.lifecycle.compose.collectAsStateWithLifecycle
import androidx.lifecycle.viewmodel.compose.viewModel
import io.github.flexy06.openneato.R
import io.github.flexy06.openneato.api.models.ErrorData
import io.github.flexy06.openneato.api.models.ScheduleNextData
import io.github.flexy06.openneato.api.models.ScheduleTime
import io.github.flexy06.openneato.api.models.SystemData
import io.github.flexy06.openneato.ui.theme.LocalStatusColors

@Composable
fun DashboardScreen(
    onChangeRobot: () -> Unit,
    onOpenManual: () -> Unit,
    onOpenSchedule: () -> Unit,
    onOpenHistory: () -> Unit,
    onOpenSettings: () -> Unit,
    vm: DashboardViewModel = viewModel(),
) {
    val ui by vm.ui.collectAsStateWithLifecycle()
    val snackbar = remember { SnackbarHostState() }
    val context = LocalContext.current
    LaunchedEffect(Unit) {
        vm.actionErrors.collect { snackbar.showSnackbar(context.getString(R.string.action_failed, it)) }
    }
    val d = ui.data
    val flags = RobotFlags.of(d.state.data?.uiState)
    val offline = d.state.error != null && d.charger.error != null && d.state.data == null && d.charger.data == null
    val docked = d.charger.data?.extPwrPresent ?: false
    val robotError = d.error.data?.takeIf { it.hasError }
    val ready = ui.firmwareReady && !offline

    Scaffold(
        topBar = { DashboardTopBar(d.firmware.data?.hostname, onChangeRobot = { vm.forgetRobot(onChangeRobot) }, onOpenSettings = onOpenSettings) },
        snackbarHost = { SnackbarHost(snackbar) },
        bottomBar = {
            if (ready) {
                ActionBar(
                    actionButtons(
                        flags,
                        docked,
                        ready = true,
                        busy = ui.pending,
                        robotError = robotError?.kind == ErrorData.Kind.ERROR,
                    ),
                    onAction = { if (it == Action.MANUAL) onOpenManual() else vm.run(it) },
                )
            }
        },
    ) { padding ->
        Column(
            Modifier
                .fillMaxSize()
                .padding(padding)
                .verticalScroll(rememberScrollState())
                .padding(horizontal = 16.dp, vertical = 8.dp),
            verticalArrangement = Arrangement.spacedBy(12.dp),
        ) {
            when {
                offline && d.firmware.data == null -> Unreachable(onChangeRobot = { vm.forgetRobot(onChangeRobot) })
                !ui.firmwareReady -> Gate(ui, onRetry = vm::reidentify)
                else -> {
                    robotError?.let { RobotErrorBanner(it) }
                    Hero(ui, flags, docked)
                    d.schedule.data?.takeIf { it.enabled }?.let {
                        ScheduleCard(it, ui.skipPending, vm::toggleSkipNextClean, onOpenSchedule)
                    }
                    Shortcuts(onOpenManual, onOpenSchedule, onOpenHistory, onOpenSettings)
                    d.system.data?.let { StatusRow(it, d.firmware.data?.version) }
                }
            }
        }
    }
}

@Composable
private fun DashboardTopBar(hostname: String?, onChangeRobot: () -> Unit, onOpenSettings: () -> Unit) {
    var menu by remember { mutableStateOf(false) }
    TopAppBar(
        title = {
            Row(verticalAlignment = Alignment.Bottom) {
                Text("OpenNeato", fontWeight = FontWeight.SemiBold)
                if (!hostname.isNullOrBlank()) {
                    Text(
                        " ($hostname)",
                        style = MaterialTheme.typography.bodyMedium,
                        color = MaterialTheme.colorScheme.onSurfaceVariant,
                    )
                }
            }
        },
        actions = {
            IconButton(onClick = onOpenSettings) {
                Icon(Icons.Filled.Settings, contentDescription = stringResource(R.string.settings))
            }
            Box {
                IconButton(onClick = { menu = true }) { Icon(Icons.Filled.MoreVert, contentDescription = null) }
                DropdownMenu(expanded = menu, onDismissRequest = { menu = false }) {
                    DropdownMenuItem(
                        text = { Text(stringResource(R.string.change_robot)) },
                        onClick = {
                            menu = false
                            onChangeRobot()
                        },
                    )
                }
            }
        },
    )
}

@Composable
private fun toneColor(tone: Tone): Color {
    val c = LocalStatusColors.current
    return when (tone) {
        Tone.GREEN -> c.green
        Tone.AMBER -> c.amber
        Tone.RED -> c.red
        Tone.BLUE -> c.blue
    }
}

private fun iconOf(icon: StatusIcon): ImageVector = when (icon) {
    StatusIcon.CHECK -> Icons.Filled.CheckCircle
    StatusIcon.SPARKLE -> Icons.Filled.AutoAwesome
    StatusIcon.ALERT -> Icons.Filled.Warning
    StatusIcon.BOLT -> Icons.Filled.Bolt
    StatusIcon.MANUAL -> Icons.Filled.Gamepad
    StatusIcon.MAP -> Icons.Filled.Map
    StatusIcon.SPOT -> Icons.Filled.CenterFocusStrong
    StatusIcon.HOUSE -> Icons.Filled.Home
    StatusIcon.IDLE -> Icons.Outlined.Bedtime
}

@Composable
private fun Hero(ui: DashboardUi, flags: RobotFlags, docked: Boolean) {
    val d = ui.data
    val status = d.state.data?.let { statusInfo(it.uiState) }
        ?: Badge(if (d.state.error != null) R.string.status_error else R.string.status_active, Tone.AMBER, StatusIcon.ALERT)
    val charger = d.charger.data
    val mode = if (d.state.data == null && d.state.error != null) {
        Badge(R.string.status_error, Tone.RED, StatusIcon.ALERT)
    } else {
        modeInfo(charger?.chargingActive ?: false, docked, flags)
    }
    val pct = charger?.fuelPercent?.toInt()?.coerceIn(0, 100)

    Card(
        colors = CardDefaults.cardColors(containerColor = MaterialTheme.colorScheme.surfaceContainer),
        shape = MaterialTheme.shapes.large,
        modifier = Modifier.fillMaxWidth(),
    ) {
        Column(Modifier.padding(20.dp), horizontalAlignment = Alignment.CenterHorizontally) {
            Box(
                Modifier
                    .size(132.dp)
                    .background(MaterialTheme.colorScheme.primaryContainer, CircleShape),
                contentAlignment = Alignment.Center,
            ) {
                Image(
                    painterResource(R.drawable.ic_launcher_foreground),
                    contentDescription = null,
                    colorFilter = ColorFilter.tint(MaterialTheme.colorScheme.onPrimaryContainer),
                    modifier = Modifier.size(150.dp),
                )
            }
            Spacer(Modifier.height(16.dp))
            Text(
                stringResource(R.string.status).uppercase(),
                style = MaterialTheme.typography.labelMedium,
                color = MaterialTheme.colorScheme.onSurfaceVariant,
            )
            Row(verticalAlignment = Alignment.CenterVertically) {
                Icon(iconOf(status.icon), contentDescription = null, tint = toneColor(status.tone))
                Spacer(Modifier.width(8.dp))
                Text(
                    stringResource(status.label),
                    style = MaterialTheme.typography.headlineMedium,
                    fontWeight = FontWeight.SemiBold,
                    color = toneColor(status.tone),
                )
            }
            Spacer(Modifier.height(12.dp))
            Row(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
                Chip(iconOf(mode.icon), stringResource(mode.label), toneColor(mode.tone))
                val battTone = when {
                    pct != null -> batteryTone(pct)
                    d.charger.error != null -> Tone.RED
                    else -> Tone.AMBER
                }
                Chip(
                    if (charger?.chargingActive == true) Icons.Filled.Bolt else Icons.Filled.EvStation,
                    pct?.let { "$it %" } ?: if (d.charger.error != null) stringResource(R.string.status_error) else "…",
                    toneColor(battTone),
                )
            }
            if (pct != null) {
                Spacer(Modifier.height(16.dp))
                val battColor = toneColor(batteryTone(pct))
                if (charger.chargingActive) {
                    // Wavy while charging: Material 3 Expressive progress
                    LinearWavyProgressIndicator(
                        progress = { pct / 100f },
                        color = battColor,
                        modifier = Modifier.fillMaxWidth(),
                    )
                } else {
                    androidx.compose.material3.LinearProgressIndicator(
                        progress = { pct / 100f },
                        color = battColor,
                        modifier = Modifier.fillMaxWidth(),
                    )
                }
            }
        }
    }
}

@Composable
private fun Chip(icon: ImageVector, text: String, color: Color) {
    Surface(shape = CircleShape, color = color.copy(alpha = 0.14f)) {
        Row(Modifier.padding(horizontal = 12.dp, vertical = 6.dp), verticalAlignment = Alignment.CenterVertically) {
            Icon(icon, contentDescription = null, tint = color, modifier = Modifier.size(18.dp))
            Spacer(Modifier.width(6.dp))
            Text(text, style = MaterialTheme.typography.labelLarge, color = color)
        }
    }
}

@Composable
private fun RobotErrorBanner(err: ErrorData) {
    val warning = err.kind == ErrorData.Kind.WARNING
    val color = if (warning) LocalStatusColors.current.amber else MaterialTheme.colorScheme.error
    Card(
        colors = CardDefaults.cardColors(containerColor = color.copy(alpha = 0.14f)),
        modifier = Modifier.fillMaxWidth(),
    ) {
        Row(Modifier.padding(16.dp), verticalAlignment = Alignment.CenterVertically) {
            Icon(Icons.Filled.Warning, contentDescription = null, tint = color)
            Spacer(Modifier.width(12.dp))
            Column {
                Text(
                    stringResource(if (warning) R.string.robot_notice else R.string.robot_attention),
                    style = MaterialTheme.typography.titleSmall,
                    color = color,
                )
                Text(
                    err.displayMessage.ifBlank { stringResource(R.string.robot_error_code, err.errorCode.toInt()) },
                    style = MaterialTheme.typography.bodyMedium,
                )
            }
        }
    }
}

@Composable
private fun dayName(day: ScheduleTime.Day): String = stringResource(
    when (day) {
        ScheduleTime.Day.MON -> R.string.day_mon
        ScheduleTime.Day.TUE -> R.string.day_tue
        ScheduleTime.Day.WED -> R.string.day_wed
        ScheduleTime.Day.THU -> R.string.day_thu
        ScheduleTime.Day.FRI -> R.string.day_fri
        ScheduleTime.Day.SAT -> R.string.day_sat
        ScheduleTime.Day.SUN -> R.string.day_sun
    },
)

@Composable
private fun ScheduleCard(s: ScheduleNextData, busy: Boolean, onToggleSkip: () -> Unit, onOpen: () -> Unit) {
    val next = s.next
    val label = if (next != null) {
        val day = when (next.dayOffset) {
            0 -> stringResource(R.string.today)
            1 -> stringResource(R.string.tomorrow)
            else -> dayName(next.day)
        }
        stringResource(R.string.next_clean, "%s %02d:%02d".format(day, next.hour, next.minute))
    } else {
        stringResource(R.string.schedule_enabled)
    }
    Card(
        onClick = onOpen,
        colors = CardDefaults.cardColors(containerColor = MaterialTheme.colorScheme.surfaceContainer),
        modifier = Modifier.fillMaxWidth(),
    ) {
        Row(Modifier.padding(start = 16.dp, end = 8.dp, top = 8.dp, bottom = 8.dp), verticalAlignment = Alignment.CenterVertically) {
            Icon(Icons.Filled.Schedule, contentDescription = null, tint = MaterialTheme.colorScheme.secondary)
            Spacer(Modifier.width(12.dp))
            Text(label, style = MaterialTheme.typography.bodyLarge, modifier = Modifier.weight(1f))
            if (next != null) {
                TextButton(onClick = onToggleSkip, enabled = !busy) {
                    Text(stringResource(if (s.skipNextClean) R.string.cancel_skip else R.string.skip_clean))
                }
            }
        }
    }
}

@Composable
private fun StatusRow(sys: SystemData, firmware: String?) {
    val storagePct = if (sys.fsTotal.toLong() > 0) sys.fsUsed.toLong() * 100 / sys.fsTotal.toLong() else 0
    Row(Modifier.fillMaxWidth(), horizontalArrangement = Arrangement.SpaceBetween) {
        StatusItem(stringResource(R.string.wifi), stringResource(wifiStrength(sys.rssi.toInt())))
        StatusItem(stringResource(R.string.time), clockOf(sys.localTime))
        StatusItem(stringResource(R.string.storage), "$storagePct %")
        if (firmware != null) StatusItem(stringResource(R.string.firmware), firmware)
    }
}

@Composable
private fun StatusItem(label: String, value: String) {
    Column(horizontalAlignment = Alignment.CenterHorizontally) {
        Text(label, style = MaterialTheme.typography.labelSmall, color = MaterialTheme.colorScheme.onSurfaceVariant)
        Text(value, style = MaterialTheme.typography.bodyMedium)
    }
}

// Robot not identified yet (or unsupported): what the ESP32 sees on the serial link
@Composable
private fun Gate(ui: DashboardUi, onRetry: () -> Unit) {
    val fw = ui.data.firmware.data
    Card(
        colors = CardDefaults.cardColors(containerColor = MaterialTheme.colorScheme.surfaceContainer),
        modifier = Modifier.fillMaxWidth(),
    ) {
        Column(Modifier.padding(20.dp).fillMaxWidth(), horizontalAlignment = Alignment.CenterHorizontally) {
            if (fw != null && !fw.identifying && !fw.supported) {
                Icon(Icons.Filled.Warning, contentDescription = null, tint = MaterialTheme.colorScheme.error)
                Spacer(Modifier.height(8.dp))
                Text(stringResource(R.string.unsupported_title), style = MaterialTheme.typography.titleLarge)
                Text(stringResource(R.string.unsupported_text), style = MaterialTheme.typography.bodyMedium)
            } else {
                LoadingIndicator(Modifier.size(56.dp))
                Spacer(Modifier.height(12.dp))
                Text(stringResource(R.string.connecting), style = MaterialTheme.typography.titleMedium)
                ui.data.diag.data?.identify?.let { id ->
                    Text(
                        stringResource(R.string.attempt, id.attempts) + if (id.lastError.isNotBlank()) " · ${id.lastError}" else "",
                        style = MaterialTheme.typography.bodySmall,
                        color = MaterialTheme.colorScheme.onSurfaceVariant,
                    )
                }
                Spacer(Modifier.height(12.dp))
                OutlinedButton(onClick = onRetry) { Text(stringResource(R.string.retry)) }
            }
        }
    }
}

@Composable
private fun Unreachable(onChangeRobot: () -> Unit) {
    Column(Modifier.fillMaxWidth().padding(top = 48.dp), horizontalAlignment = Alignment.CenterHorizontally) {
        Icon(
            Icons.Outlined.WifiOff,
            contentDescription = null,
            tint = MaterialTheme.colorScheme.error,
            modifier = Modifier.size(48.dp),
        )
        Spacer(Modifier.height(12.dp))
        Text(stringResource(R.string.unreachable), style = MaterialTheme.typography.titleMedium)
        Spacer(Modifier.height(12.dp))
        OutlinedButton(onClick = onChangeRobot) { Text(stringResource(R.string.change_robot)) }
    }
}

private fun actionLabel(a: Action): Int = when (a) {
    Action.HOUSE -> R.string.action_house
    Action.SPOT -> R.string.action_spot
    Action.PAUSE -> R.string.action_pause
    Action.RESUME -> R.string.action_resume
    Action.DOCK, Action.GUIDED_HOME -> R.string.action_dock
    Action.STOP, Action.GUIDED_STOP -> R.string.action_stop
    Action.MANUAL -> R.string.manual
}

private fun actionIcon(a: Action): ImageVector = when (a) {
    Action.HOUSE -> Icons.Filled.Home
    Action.SPOT -> Icons.Filled.CenterFocusStrong
    Action.PAUSE -> Icons.Filled.Pause
    Action.RESUME -> Icons.Filled.PlayArrow
    Action.DOCK, Action.GUIDED_HOME -> Icons.Filled.EvStation
    Action.STOP, Action.GUIDED_STOP -> Icons.Filled.Stop
    Action.MANUAL -> Icons.Filled.Gamepad
}

// Entry points to the other screens (more follow as they are built)
@Composable
private fun Shortcuts(
    onOpenManual: () -> Unit,
    onOpenSchedule: () -> Unit,
    onOpenHistory: () -> Unit,
    onOpenSettings: () -> Unit,
) {
    Column(verticalArrangement = Arrangement.spacedBy(8.dp)) {
        Row(Modifier.fillMaxWidth(), horizontalArrangement = Arrangement.spacedBy(8.dp)) {
            Shortcut(Icons.Filled.Gamepad, stringResource(R.string.manual), onOpenManual, Modifier.weight(1f))
            Shortcut(Icons.Filled.Schedule, stringResource(R.string.schedule), onOpenSchedule, Modifier.weight(1f))
        }
        Row(Modifier.fillMaxWidth(), horizontalArrangement = Arrangement.spacedBy(8.dp)) {
            Shortcut(Icons.Filled.History, stringResource(R.string.history), onOpenHistory, Modifier.weight(1f))
            Shortcut(Icons.Filled.Settings, stringResource(R.string.settings), onOpenSettings, Modifier.weight(1f))
        }
    }
}

@Composable
private fun Shortcut(icon: ImageVector, label: String, onClick: () -> Unit, modifier: Modifier) {
    Card(
        onClick = onClick,
        colors = CardDefaults.cardColors(containerColor = MaterialTheme.colorScheme.surfaceContainer),
        modifier = modifier,
    ) {
        Row(Modifier.padding(16.dp), verticalAlignment = Alignment.CenterVertically) {
            Icon(icon, contentDescription = null, tint = MaterialTheme.colorScheme.secondary)
            Spacer(Modifier.width(12.dp))
            Text(label, style = MaterialTheme.typography.titleSmall)
        }
    }
}

@Composable
private fun ActionBar(buttons: List<ActionButton>, onAction: (Action) -> Unit) {
    Surface(color = MaterialTheme.colorScheme.surfaceContainer) {
        Row(
            Modifier
                .fillMaxWidth()
                .navigationBarsPadding()
                .padding(horizontal = 16.dp, vertical = 12.dp),
            horizontalArrangement = Arrangement.spacedBy(8.dp),
        ) {
            for (b in buttons) {
                val content: @Composable () -> Unit = {
                    Column(horizontalAlignment = Alignment.CenterHorizontally) {
                        Icon(actionIcon(b.action), contentDescription = null)
                        // Long labels ("Zur Ladestation") shrink instead of being cut off
                        Text(
                            stringResource(actionLabel(b.action)),
                            maxLines = 1,
                            autoSize = TextAutoSize.StepBased(minFontSize = 10.sp, maxFontSize = 14.sp),
                        )
                    }
                }
                val mod = Modifier.weight(1f).height(72.dp)
                val pad = ButtonDefaults.ContentPadding
                if (b.primary) {
                    Button(onClick = { onAction(b.action) }, enabled = b.enabled, modifier = mod, contentPadding = pad) {
                        content()
                    }
                } else {
                    FilledTonalButton(onClick = { onAction(b.action) }, enabled = b.enabled, modifier = mod, contentPadding = pad) {
                        content()
                    }
                }
            }
        }
    }
}

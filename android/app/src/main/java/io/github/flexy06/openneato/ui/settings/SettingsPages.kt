package io.github.flexy06.openneato.ui.settings

import android.app.LocaleManager
import android.content.Intent
import android.os.Build
import android.os.LocaleList
import android.text.format.DateFormat
import androidx.activity.compose.rememberLauncherForActivityResult
import androidx.activity.result.contract.ActivityResultContracts
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.text.KeyboardOptions
import androidx.compose.material.icons.Icons
import androidx.compose.material.icons.filled.Battery5Bar
import androidx.compose.material.icons.filled.DeleteForever
import androidx.compose.material.icons.filled.Description
import androidx.compose.material.icons.filled.ErrorOutline
import androidx.compose.material.icons.filled.FileOpen
import androidx.compose.material.icons.filled.Lock
import androidx.compose.material.icons.filled.OpenInNew
import androidx.compose.material.icons.filled.PowerSettingsNew
import androidx.compose.material.icons.filled.RestartAlt
import androidx.compose.material.icons.filled.Search
import androidx.compose.material.icons.filled.SdCard
import androidx.compose.material.icons.filled.SystemUpdateAlt
import androidx.compose.material.icons.filled.Wifi
import androidx.compose.material.icons.filled.WifiOff
import androidx.compose.material3.AlertDialog
import androidx.compose.material3.Button
import androidx.compose.material3.Icon
import androidx.compose.material3.LinearWavyProgressIndicator
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.OutlinedButton
import androidx.compose.material3.OutlinedTextField
import androidx.compose.material3.SegmentedButton
import androidx.compose.material3.SegmentedButtonDefaults
import androidx.compose.material3.SingleChoiceSegmentedButtonRow
import androidx.compose.material3.Text
import androidx.compose.material3.TextButton
import androidx.compose.material3.TimePicker
import androidx.compose.material3.rememberTimePickerState
import androidx.compose.runtime.Composable
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.setValue
import androidx.compose.ui.Modifier
import androidx.compose.ui.platform.LocalContext
import androidx.compose.ui.res.stringArrayResource
import androidx.compose.ui.res.stringResource
import androidx.compose.ui.text.input.KeyboardType
import androidx.compose.ui.text.input.PasswordVisualTransformation
import androidx.compose.ui.unit.dp
import androidx.core.net.toUri
import io.github.flexy06.openneato.BuildConfig
import io.github.flexy06.openneato.R
import io.github.flexy06.openneato.api.models.UserSettingsData
import io.github.flexy06.openneato.api.models.WiFiNetwork
import io.github.flexy06.openneato.data.AppTheme
import io.github.flexy06.openneato.ui.history.formatDuration
import io.github.flexy06.openneato.ui.theme.LocalStatusColors
import java.util.Locale

private const val RELEASES_URL = "https://github.com/Flexy06/OpenNeato/releases"

@Composable
private fun noteLabel(n: ChoiceNote?): String? = n?.let {
    stringResource(
        when (it) {
            ChoiceNote.RECOMMENDED -> R.string.note_recommended
            ChoiceNote.ECO -> R.string.note_eco
            ChoiceNote.HIGH -> R.string.note_high
            ChoiceNote.TURBO -> R.string.note_turbo
            ChoiceNote.LOW_POWER -> R.string.note_low_power
            ChoiceNote.MAX_RANGE -> R.string.note_max_range
            ChoiceNote.VERY_SENSITIVE -> R.string.note_very_sensitive
            ChoiceNote.LESS_SENSITIVE -> R.string.note_less_sensitive
        },
    )
}

@Composable
private fun IntChoiceRow(title: String, value: Int, choices: List<Choice<Int>>, desc: String? = null, enabled: Boolean, onPick: (Int) -> Unit) {
    val byValue = choices.associateBy { it.value }
    ChoiceRow(
        title = title,
        value = value,
        options = choices.map { it.value },
        label = { v ->
            val c = byValue[v]
            val note = noteLabel(c?.note)
            if (c == null) v.toString() else if (note != null) "${c.label} ($note)" else c.label
        },
        desc = desc,
        enabled = enabled,
        onPick = onPick,
    )
}

// -- App ---------------------------------------------------------------------------------

@Composable
fun AppPage(theme: AppTheme, robotUrl: String?, onTheme: (AppTheme) -> Unit, onChangeRobot: () -> Unit) {
    val context = LocalContext.current
    SettingsGroup(stringResource(R.string.theme)) {
        val options = listOf(AppTheme.SYSTEM to R.string.theme_system, AppTheme.LIGHT to R.string.theme_light, AppTheme.DARK to R.string.theme_dark)
        SingleChoiceSegmentedButtonRow(Modifier.fillMaxWidth().padding(16.dp)) {
            options.forEachIndexed { i, (t, label) ->
                SegmentedButton(
                    selected = theme == t,
                    onClick = { onTheme(t) },
                    shape = SegmentedButtonDefaults.itemShape(i, options.size),
                ) { Text(stringResource(label)) }
            }
        }
    }
    SettingsGroup(stringResource(R.string.language)) {
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.TIRAMISU) {
            val lm = context.getSystemService(LocaleManager::class.java)
            var current by remember { mutableStateOf(lm.applicationLocales.toLanguageTags()) }
            val options = listOf("", "de", "en")
            ChoiceRow(
                title = stringResource(R.string.language),
                value = options.firstOrNull { it.isNotEmpty() && current.startsWith(it) } ?: "",
                options = options,
                label = { tag ->
                    if (tag.isEmpty()) stringResource(R.string.language_system) else Locale.forLanguageTag(tag).let { it.getDisplayName(it) }
                        .replaceFirstChar { it.titlecase() }
                },
                onPick = { tag ->
                    // Recreates the activity in the new language
                    lm.applicationLocales = if (tag.isEmpty()) LocaleList.getEmptyLocaleList() else LocaleList.forLanguageTags(tag)
                    current = tag
                },
            )
        } else {
            InfoRow(stringResource(R.string.language_old_android))
        }
    }
    SettingsGroup(stringResource(R.string.robot_address)) {
        InfoRow(robotUrl ?: "–", MaterialTheme.colorScheme.onSurface)
        RowDivider()
        ActionRow(stringResource(R.string.change_robot), Icons.Filled.Search, onClick = onChangeRobot)
    }
    SettingsGroup {
        InfoRow(stringResource(R.string.app_version, BuildConfig.VERSION_NAME))
    }
}

// -- Device ------------------------------------------------------------------------------

@Composable
fun DevicePage(ui: SettingsUi, vm: SettingsViewModel, onAskRestart: () -> Unit) {
    val e = ui.edited
    val busy = ui.saving
    val tzLabels = stringArrayResource(R.array.tz_labels)
    SettingsGroup {
        ui.system?.let { InfoRow(stringResource(R.string.uptime, formatDuration(it.uptime.toDouble() / 1000))) }
        TextRow(
            label = stringResource(R.string.hostname),
            value = e.hostname,
            onChange = { v -> vm.edit { it.copy(hostname = v.take(32)) } },
            error = problemText(e.hostnameProblem(), e.maxGpioPin),
            hint = stringResource(R.string.hostname_hint),
            enabled = !busy,
            placeholder = "neato",
        )
        IntChoiceRow(stringResource(R.string.tx_power), e.wifiTxPower, TX_POWER_CHOICES, stringResource(R.string.tx_power_hint), !busy) { v ->
            vm.edit { it.copy(wifiTxPower = v) }
        }
        RowDivider()
        ChoiceRow(
            title = stringResource(R.string.timezone),
            value = e.tz,
            options = TIMEZONES,
            label = { tz -> TIMEZONES.indexOf(tz).let { i -> if (i >= 0) tzLabels[i] else stringResource(R.string.timezone_custom, tz) } },
            desc = ui.system?.localTime,
            enabled = !busy,
        ) { tz -> vm.edit { it.copy(tz = tz) } }
    }
    SettingsGroup(stringResource(R.string.uart_pins)) {
        Row(Modifier.fillMaxWidth().padding(horizontal = 8.dp), horizontalArrangement = Arrangement.spacedBy(0.dp)) {
            Column(Modifier.weight(1f)) {
                PinField(stringResource(R.string.uart_tx), e.uartTxPin, !busy) { p -> vm.edit { it.copy(uartTxPin = p) } }
            }
            Column(Modifier.weight(1f)) {
                PinField(stringResource(R.string.uart_rx), e.uartRxPin, !busy) { p -> vm.edit { it.copy(uartRxPin = p) } }
            }
        }
        problemText(e.pinProblem(), e.maxGpioPin)?.let { InfoRow(it, MaterialTheme.colorScheme.error) }
    }
    SettingsGroup {
        SwitchRow(
            stringResource(R.string.auto_restart),
            stringResource(R.string.auto_restart_desc),
            e.autoRestartEnabled,
            enabled = !busy && ui.supported,
        ) { on -> vm.edit { it.copy(autoRestartEnabled = on) } }
        if (e.autoRestartEnabled) {
            var picking by remember { mutableStateOf(false) }
            ActionRow(
                stringResource(R.string.auto_restart_time),
                desc = "%02d:%02d".format(e.autoRestartHour, e.autoRestartMinute),
                enabled = !busy,
            ) { picking = true }
            InfoRow(stringResource(R.string.auto_restart_hint))
            if (ui.user?.melodies == true) InfoRow(stringResource(R.string.auto_restart_melodies), LocalStatusColors.current.amber)
            if (picking) {
                TimeDialog(e.autoRestartHour, e.autoRestartMinute, onDismiss = { picking = false }) { h, m ->
                    picking = false
                    vm.edit { it.copy(autoRestartHour = h, autoRestartMinute = m) }
                }
            }
        }
        RowDivider()
        SwitchRow(
            stringResource(R.string.restart_before_clean),
            stringResource(R.string.restart_before_clean_desc),
            e.restartBeforeClean,
            enabled = !busy && ui.supported,
        ) { on -> vm.edit { it.copy(restartBeforeClean = on) } }
    }
    SettingsGroup {
        ActionRow(stringResource(R.string.restart_device), Icons.Filled.RestartAlt, enabled = ui.busy == Busy.NONE, onClick = onAskRestart)
    }
}

@Composable
private fun PinField(label: String, value: Int, enabled: Boolean, onChange: (Int) -> Unit) {
    var text by remember(value) { mutableStateOf(value.toString()) }
    OutlinedTextField(
        value = text,
        onValueChange = { v ->
            text = v.filter { it.isDigit() }.take(2)
            text.toIntOrNull()?.let(onChange)
        },
        label = { Text(label) },
        singleLine = true,
        enabled = enabled,
        keyboardOptions = KeyboardOptions(keyboardType = KeyboardType.Number),
        modifier = Modifier.fillMaxWidth().padding(8.dp),
    )
}

@Composable
private fun TimeDialog(hour: Int, minute: Int, onDismiss: () -> Unit, onConfirm: (Int, Int) -> Unit) {
    val state = rememberTimePickerState(hour, minute, is24Hour = DateFormat.is24HourFormat(LocalContext.current))
    AlertDialog(
        onDismissRequest = onDismiss,
        text = { TimePicker(state) },
        confirmButton = { TextButton(onClick = { onConfirm(state.hour, state.minute) }) { Text(stringResource(R.string.ok)) } },
        dismissButton = { TextButton(onClick = onDismiss) { Text(stringResource(R.string.cancel)) } },
    )
}

@Composable
fun problemText(p: SettingsProblem?, maxPin: Int): String? = p?.let {
    when (it) {
        SettingsProblem.PINS_SAME -> stringResource(R.string.problem_pins_same)
        SettingsProblem.PIN_RANGE -> stringResource(R.string.problem_pin_range, maxPin)
        SettingsProblem.HOSTNAME_EMPTY -> stringResource(R.string.problem_hostname_empty)
        SettingsProblem.HOSTNAME_LONG -> stringResource(R.string.problem_hostname_long)
        SettingsProblem.HOSTNAME_CHARS -> stringResource(R.string.problem_hostname_chars)
        SettingsProblem.SYSLOG_IP_MISSING -> stringResource(R.string.problem_syslog_missing)
        SettingsProblem.SYSLOG_IP_INVALID -> stringResource(R.string.problem_syslog_invalid)
    }
}

// -- WiFi --------------------------------------------------------------------------------

@Composable
fun WifiPage(ui: SettingsUi, vm: SettingsViewModel, onAskForget: () -> Unit) {
    val w = ui.wifi
    var connectTo by remember { mutableStateOf<WiFiNetwork?>(null) }
    SettingsGroup {
        if (w == null) {
            InfoRow("…")
        } else {
            Row(Modifier.padding(16.dp)) {
                Icon(if (w.staConnected) Icons.Filled.Wifi else Icons.Filled.WifiOff, contentDescription = null)
                Column(Modifier.padding(start = 16.dp)) {
                    Text(
                        if (w.staConnected) stringResource(R.string.wifi_connected, w.ssid) else stringResource(R.string.wifi_not_connected),
                        style = MaterialTheme.typography.bodyLarge,
                    )
                    if (w.staConnected) {
                        Text(
                            stringResource(R.string.wifi_details, w.ip, w.rssi),
                            style = MaterialTheme.typography.bodySmall,
                            color = MaterialTheme.colorScheme.onSurfaceVariant,
                        )
                    }
                    if (w.apActive) {
                        Text(
                            stringResource(R.string.wifi_ap_active, w.apSsid, w.apIp),
                            style = MaterialTheme.typography.bodySmall,
                            color = LocalStatusColors.current.amber,
                        )
                    }
                    if (w.lastError.isNotEmpty()) {
                        Text(
                            stringResource(R.string.wifi_last_error, w.lastError),
                            style = MaterialTheme.typography.bodySmall,
                            color = MaterialTheme.colorScheme.error,
                        )
                    }
                }
            }
            if (w.staConnected) {
                RowDivider()
                ActionRow(stringResource(R.string.wifi_forget), Icons.Filled.WifiOff, danger = true, enabled = ui.busy == Busy.NONE, onClick = onAskForget)
            }
        }
    }
    SettingsGroup {
        SwitchRow(
            stringResource(R.string.wifi_ap_fallback),
            stringResource(R.string.wifi_ap_fallback_desc),
            ui.edited.apFallbackOnDisconnect,
            enabled = !ui.saving,
        ) { on -> vm.edit { it.copy(apFallbackOnDisconnect = on) } }
    }
    SettingsGroup {
        ActionRow(stringResource(R.string.wifi_scan), Icons.Filled.Search, enabled = !ui.scanning) { vm.scanWifi() }
        if (ui.scanning) LinearWavyProgressIndicator(Modifier.fillMaxWidth().padding(16.dp))
        ui.networks?.forEach { n ->
            RowDivider()
            val bars = when {
                n.rssi >= -55 -> "▂▄▆█"
                n.rssi >= -65 -> "▂▄▆"
                n.rssi >= -75 -> "▂▄"
                else -> "▂"
            }
            ActionRow(
                n.ssid,
                icon = if (n.open == true) Icons.Filled.Wifi else Icons.Filled.Lock,
                desc = "$bars  ${n.rssi} dBm" + if (n.open == true) " · " + stringResource(R.string.wifi_open) else "",
                enabled = ui.busy == Busy.NONE,
            ) { connectTo = n }
        }
    }
    connectTo?.let { n ->
        var password by remember { mutableStateOf("") }
        AlertDialog(
            onDismissRequest = { connectTo = null },
            title = { Text(stringResource(R.string.wifi_connect_title, n.ssid)) },
            text = {
                Column {
                    if (n.open != true) {
                        OutlinedTextField(
                            value = password,
                            onValueChange = { password = it },
                            label = { Text(stringResource(R.string.wifi_password)) },
                            singleLine = true,
                            visualTransformation = PasswordVisualTransformation(),
                            keyboardOptions = KeyboardOptions(keyboardType = KeyboardType.Password),
                            modifier = Modifier.fillMaxWidth(),
                        )
                    }
                    Text(
                        stringResource(R.string.wifi_connect_hint),
                        style = MaterialTheme.typography.bodySmall,
                        modifier = Modifier.padding(top = 8.dp),
                    )
                }
            },
            confirmButton = {
                TextButton(
                    onClick = {
                        connectTo = null
                        vm.connectWifi(n.ssid, password)
                    },
                    enabled = n.open == true || password.length >= 8,
                ) { Text(stringResource(R.string.wifi_connect)) }
            },
            dismissButton = { TextButton(onClick = { connectTo = null }) { Text(stringResource(R.string.cancel)) } },
        )
    }
}

// -- Notifications -----------------------------------------------------------------------

@Composable
fun NotificationsPage(ui: SettingsUi, vm: SettingsViewModel) {
    val e = ui.edited
    val busy = ui.saving
    SettingsGroup {
        SwitchRow(stringResource(R.string.ntfy_enable), stringResource(R.string.ntfy_enable_desc), e.ntfyEnabled, enabled = !busy) { on ->
            vm.edit { it.copy(ntfyEnabled = on) }
        }
        if (e.ntfyEnabled) {
            Row(Modifier.fillMaxWidth().padding(end = 16.dp), verticalAlignment = androidx.compose.ui.Alignment.CenterVertically) {
                Column(Modifier.weight(1f)) {
                    TextRow(
                        label = stringResource(R.string.ntfy_topic),
                        value = e.ntfyTopic,
                        onChange = { v -> vm.edit { it.copy(ntfyTopic = v) } },
                        enabled = !busy,
                        placeholder = "my-robot-alerts",
                    )
                }
                OutlinedButton(
                    onClick = vm::testNotification,
                    enabled = e.ntfyTopic.isNotBlank() && !ui.testingNotification,
                ) { Text(stringResource(R.string.ntfy_test)) }
            }
            RowDivider()
            SwitchRow(stringResource(R.string.ntfy_done), stringResource(R.string.ntfy_done_desc), e.ntfyOnDone, enabled = !busy) { on ->
                vm.edit { it.copy(ntfyOnDone = on) }
            }
            SwitchRow(stringResource(R.string.ntfy_error), stringResource(R.string.ntfy_error_desc), e.ntfyOnError, enabled = !busy) { on ->
                vm.edit { it.copy(ntfyOnError = on) }
            }
            SwitchRow(stringResource(R.string.ntfy_alert), stringResource(R.string.ntfy_alert_desc), e.ntfyOnAlert, enabled = !busy) { on ->
                vm.edit { it.copy(ntfyOnAlert = on) }
            }
            SwitchRow(stringResource(R.string.ntfy_docking), stringResource(R.string.ntfy_docking_desc), e.ntfyOnDocking, enabled = !busy) { on ->
                vm.edit { it.copy(ntfyOnDocking = on) }
            }
        }
    }
}

// -- Cleaning ----------------------------------------------------------------------------

@Composable
fun CleaningPage(ui: SettingsUi, vm: SettingsViewModel) {
    val e = ui.edited
    val busy = ui.saving
    SettingsGroup(stringResource(R.string.mode_house)) {
        ChoiceRow(
            title = stringResource(R.string.nav_mode),
            value = e.navMode,
            options = NAV_MODES,
            label = { m ->
                when (m) {
                    "Gentle" -> stringResource(R.string.nav_gentle)
                    "Deep" -> stringResource(R.string.nav_deep)
                    "Quick" -> stringResource(R.string.nav_quick)
                    else -> stringResource(R.string.nav_normal)
                }
            },
            desc = stringResource(R.string.nav_mode_hint),
            enabled = !busy && ui.supported,
        ) { m -> vm.edit { it.copy(navMode = m) } }
    }
    SettingsGroup(stringResource(R.string.manual_cleaning)) {
        IntChoiceRow(stringResource(R.string.brush_speed), e.brushRpm, BRUSH_CHOICES, enabled = !busy) { v -> vm.edit { it.copy(brushRpm = v) } }
        RowDivider()
        IntChoiceRow(stringResource(R.string.vacuum_power), e.vacuumSpeed, VACUUM_CHOICES, enabled = !busy) { v -> vm.edit { it.copy(vacuumSpeed = v) } }
        RowDivider()
        IntChoiceRow(stringResource(R.string.side_brush_power), e.sideBrushPower, SIDE_BRUSH_CHOICES, stringResource(R.string.side_brush_hint), !busy) { v ->
            vm.edit { it.copy(sideBrushPower = v) }
        }
        RowDivider()
        IntChoiceRow(stringResource(R.string.stall_detection), e.stallThreshold, STALL_CHOICES, stringResource(R.string.stall_hint), !busy) { v ->
            vm.edit { it.copy(stallThreshold = v) }
        }
    }
}

// -- Robot (applied at once) ---------------------------------------------------------------

@Composable
fun RobotPage(ui: SettingsUi, vm: SettingsViewModel, onAskRestart: () -> Unit, onAskShutdown: () -> Unit) {
    val u = ui.user
    val enabled = u != null && !ui.userPending && ui.supported
    @Composable
    fun Toggle(title: Int, desc: Int, key: String, value: Boolean?, set: (UserSettingsData, Boolean) -> UserSettingsData) {
        SwitchRow(stringResource(title), stringResource(desc), value == true, enabled = enabled) { on ->
            vm.setRobotSetting(key, { set(it, on) }, on)
        }
    }
    SettingsGroup(stringResource(R.string.robot_sound)) {
        Toggle(R.string.button_clicks, R.string.button_clicks_desc, "ButtonClick", u?.buttonClick) { s, on -> s.copy(buttonClick = on) }
        Toggle(R.string.melodies, R.string.melodies_desc, "Melodies", u?.melodies) { s, on -> s.copy(melodies = on) }
        Toggle(R.string.warnings, R.string.warnings_desc, "Warnings", u?.warnings) { s, on -> s.copy(warnings = on) }
    }
    SettingsGroup(stringResource(R.string.robot_cleaning)) {
        Toggle(R.string.eco_mode, R.string.eco_mode_desc, "EcoMode", u?.ecoMode) { s, on -> s.copy(ecoMode = on) }
        Toggle(R.string.intense_clean, R.string.intense_clean_desc, "IntenseClean", u?.intenseClean) { s, on -> s.copy(intenseClean = on) }
        Toggle(R.string.bin_full, R.string.bin_full_desc, "BinFullDetect", u?.binFullDetect) { s, on -> s.copy(binFullDetect = on) }
        Toggle(R.string.wall_follow, R.string.wall_follow_desc, "WallEnable", u?.wallEnable) { s, on -> s.copy(wallEnable = on) }
    }
    SettingsGroup(stringResource(R.string.robot_power)) {
        Toggle(R.string.robot_wifi, R.string.robot_wifi_desc, "WiFi", u?.wifi) { s, on -> s.copy(wifi = on) }
        Toggle(R.string.stealth_leds, R.string.stealth_leds_desc, "StealthLED", u?.stealthLed) { s, on -> s.copy(stealthLed = on) }
    }
    SettingsGroup(stringResource(R.string.robot_power_control)) {
        ActionRow(stringResource(R.string.restart_robot), Icons.Filled.RestartAlt, enabled = ui.busy == Busy.NONE && ui.supported, onClick = onAskRestart)
        RowDivider()
        ActionRow(stringResource(R.string.shutdown_robot), Icons.Filled.PowerSettingsNew, danger = true, enabled = ui.supported, onClick = onAskShutdown)
    }
}

// -- Firmware ------------------------------------------------------------------------------

@Composable
fun FirmwarePage(ui: SettingsUi, vm: SettingsViewModel, onAskUploadUnverified: () -> Unit) {
    val context = LocalContext.current
    val pickBin = rememberLauncherForActivityResult(ActivityResultContracts.OpenDocument()) { it?.let(vm::pickFirmware) }
    val pickSums = rememberLauncherForActivityResult(ActivityResultContracts.OpenDocument()) { it?.let(vm::pickChecksums) }
    val up = ui.upload
    SettingsGroup {
        InfoRow(stringResource(R.string.fw_version) + ": " + (ui.firmware?.version ?: "…"), MaterialTheme.colorScheme.onSurface)
        InfoRow(stringResource(R.string.fw_chip) + ": " + (ui.firmware?.chip ?: "…"), MaterialTheme.colorScheme.onSurface)
        RowDivider()
        ActionRow(stringResource(R.string.fw_releases), Icons.Filled.OpenInNew) {
            context.startActivity(Intent(Intent.ACTION_VIEW, RELEASES_URL.toUri()))
        }
    }
    SettingsGroup {
        val idle = up.status == FwStatus.IDLE
        ActionRow(
            if (up.selected) up.name else stringResource(R.string.fw_select),
            Icons.Filled.FileOpen,
            desc = if (up.selected) "%.0f KB".format(up.size / 1024.0) + (up.chip?.let { " · $it" } ?: "") else null,
            enabled = idle,
        ) { pickBin.launch(arrayOf("*/*")) }
        if (up.selected && up.chip == null) InfoRow(stringResource(R.string.fw_chip_unknown), MaterialTheme.colorScheme.error)
        if (up.chipMismatch) {
            InfoRow(stringResource(R.string.fw_chip_mismatch, up.chip.orEmpty(), ui.firmware?.chip.orEmpty()), MaterialTheme.colorScheme.error)
        }
        if (up.selected && up.chip != null && !up.chipMismatch) {
            RowDivider()
            ActionRow(stringResource(R.string.fw_select_checksums), Icons.Filled.Description, enabled = idle) { pickSums.launch(arrayOf("*/*")) }
            when (up.checksum) {
                ChecksumResult.MATCH -> InfoRow(stringResource(R.string.fw_checksum_ok), LocalStatusColors.current.green)
                ChecksumResult.MISMATCH -> InfoRow(stringResource(R.string.fw_checksum_bad), MaterialTheme.colorScheme.error)
                ChecksumResult.NOT_FOUND -> InfoRow(stringResource(R.string.fw_checksum_missing), LocalStatusColors.current.amber)
                null -> {}
            }
        }
        when (up.status) {
            FwStatus.IDLE -> if (up.canUpload) {
                Button(
                    onClick = { if (up.checksum == ChecksumResult.MATCH) vm.uploadFirmware() else onAskUploadUnverified() },
                    modifier = Modifier.fillMaxWidth().padding(16.dp),
                ) {
                    Icon(Icons.Filled.SystemUpdateAlt, contentDescription = null)
                    Text(stringResource(R.string.fw_upload), Modifier.padding(start = 8.dp))
                }
            }
            FwStatus.UPLOADING, FwStatus.DONE -> {
                LinearWavyProgressIndicator(progress = { up.progress }, modifier = Modifier.fillMaxWidth().padding(16.dp))
                InfoRow(
                    when {
                        up.status == FwStatus.DONE -> stringResource(R.string.fw_done)
                        up.progress >= 0.9f -> stringResource(R.string.fw_writing)
                        else -> stringResource(R.string.fw_uploading, (up.progress * 100).toInt())
                    },
                )
            }
        }
    }
}

// -- Diagnostics ---------------------------------------------------------------------------

@Composable
fun DiagnosticsPage(
    ui: SettingsUi,
    vm: SettingsViewModel,
    onOpenLogs: () -> Unit,
    onAskClearErrors: () -> Unit,
    onAskNewBattery: () -> Unit,
) {
    val e = ui.edited
    val busy = ui.saving
    SettingsGroup {
        ChoiceRow(
            title = stringResource(R.string.log_level),
            value = e.logLevel,
            options = listOf(0, 1, 2),
            label = { l ->
                when (l) {
                    1 -> stringResource(if (e.syslogEnabled) R.string.log_info else R.string.log_info_auto)
                    2 -> stringResource(if (e.syslogEnabled) R.string.log_debug else R.string.log_debug_auto)
                    else -> stringResource(R.string.log_off)
                }
            },
            desc = stringResource(if (e.syslogEnabled) R.string.log_hint_syslog else R.string.log_hint_flash),
            enabled = !busy,
        ) { l -> vm.edit { it.copy(logLevel = l) } }
        RowDivider()
        SwitchRow(stringResource(R.string.syslog), stringResource(R.string.syslog_desc), e.syslogEnabled, enabled = !busy) { on ->
            vm.edit { it.copy(syslogEnabled = on) }
        }
        if (e.syslogEnabled) {
            TextRow(
                label = stringResource(R.string.syslog_ip),
                value = e.syslogIp,
                onChange = { v -> vm.edit { it.copy(syslogIp = v) } },
                error = problemText(e.syslogProblem(), e.maxGpioPin),
                enabled = !busy,
                placeholder = "192.168.1.100",
            )
        }
    }
    BatteryGroup(ui, onAskNewBattery)
    SettingsGroup {
        ActionRow(stringResource(R.string.logs), Icons.Filled.Description, chevron = true, onClick = onOpenLogs)
        RowDivider()
        ActionRow(
            stringResource(R.string.clear_errors),
            Icons.Filled.ErrorOutline,
            enabled = ui.busy == Busy.NONE && ui.supported,
            onClick = onAskClearErrors,
        )
    }
}

@Composable
private fun BatteryGroup(ui: SettingsUi, onAskNewBattery: () -> Unit) {
    val b = ui.battery
    SettingsGroup(stringResource(R.string.battery_title)) {
        val c = b.charger
        val a = b.analog
        val w = b.warranty
        if (c == null || a == null || w == null) {
            InfoRow("…")
        } else {
            val yes = stringResource(R.string.yes)
            val no = stringResource(R.string.no)
            val rows = listOf(
                R.string.battery_charge to "${c.fuelPercent.toInt()} %",
                R.string.battery_state to stringResource(
                    when {
                        c.chargingActive -> R.string.battery_charging
                        c.extPwrPresent -> R.string.battery_docked
                        else -> R.string.battery_on_battery
                    },
                ),
                R.string.battery_voltage to "%.2f V".format(c.vBattV.toDouble()),
                R.string.battery_temp to "%.1f °C".format(a.batteryTemperatureC.toDouble()),
                R.string.battery_current to "${a.batteryCurrentMA.toInt()} mA",
                R.string.battery_ext_voltage to "%.2f V".format(a.externalVoltageV.toDouble()),
                R.string.battery_cycles to w.cumulativeBatteryCycles.toInt().toString(),
                R.string.battery_clean_time to formatDuration(w.cumulativeCleaningTimeSeconds.toDouble()),
                R.string.battery_pack to (b.version?.smartBatteryManufacturerName?.takeIf { it.isNotBlank() } ?: stringResource(R.string.unknown)),
                R.string.battery_charging_enabled to if (c.chargingEnabled) yes else no,
                R.string.battery_fuel_conf to if (c.confidOnFuel) yes else no,
            )
            for ((label, value) in rows) {
                Row(Modifier.fillMaxWidth().padding(horizontal = 16.dp, vertical = 6.dp)) {
                    Text(stringResource(label), Modifier.weight(1f), style = MaterialTheme.typography.bodyMedium)
                    Text(value, style = MaterialTheme.typography.bodyMedium, color = MaterialTheme.colorScheme.primary)
                }
            }
        }
        RowDivider()
        ActionRow(stringResource(R.string.new_battery), Icons.Filled.Battery5Bar, enabled = ui.busy == Busy.NONE && ui.supported, onClick = onAskNewBattery)
    }
}

// -- About / danger zone -------------------------------------------------------------------

@Composable
fun AboutPage(ui: SettingsUi) {
    val context = LocalContext.current
    val fw = ui.firmware
    SettingsGroup {
        Column(Modifier.padding(16.dp)) {
            Text(fw?.name ?: "OpenNeato", style = MaterialTheme.typography.titleLarge)
            Text(stringResource(R.string.about_desc), style = MaterialTheme.typography.bodyMedium)
            Text("Copyright © 2026 Soner Köksal", style = MaterialTheme.typography.bodySmall, modifier = Modifier.padding(top = 8.dp))
            fw?.license?.let { Text(stringResource(R.string.about_license, it), style = MaterialTheme.typography.bodySmall) }
            Text(stringResource(R.string.app_version, BuildConfig.VERSION_NAME), style = MaterialTheme.typography.bodySmall)
        }
        fw?.repositoryUrl?.takeIf { it.isNotBlank() }?.let { url ->
            RowDivider()
            ActionRow(stringResource(R.string.about_github), Icons.Filled.OpenInNew) {
                context.startActivity(Intent(Intent.ACTION_VIEW, url.toUri()))
            }
        }
    }
}

@Composable
fun DangerPage(ui: SettingsUi, onAskFormat: () -> Unit, onAskReset: () -> Unit) {
    SettingsGroup {
        ActionRow(stringResource(R.string.format_storage), Icons.Filled.SdCard, danger = true, enabled = ui.busy == Busy.NONE, onClick = onAskFormat)
        RowDivider()
        ActionRow(stringResource(R.string.factory_reset), Icons.Filled.DeleteForever, danger = true, enabled = ui.busy == Busy.NONE, onClick = onAskReset)
    }
}

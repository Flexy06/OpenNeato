package io.github.flexy06.openneato.ui.settings

import androidx.activity.compose.BackHandler
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
import androidx.compose.foundation.rememberScrollState
import androidx.compose.foundation.verticalScroll
import androidx.compose.material.icons.Icons
import androidx.compose.material.icons.automirrored.filled.ArrowBack
import androidx.compose.material.icons.filled.CleaningServices
import androidx.compose.material.icons.filled.Info
import androidx.compose.material.icons.filled.Memory
import androidx.compose.material.icons.filled.MonitorHeart
import androidx.compose.material.icons.filled.Notifications
import androidx.compose.material.icons.filled.Palette
import androidx.compose.material.icons.filled.Router
import androidx.compose.material.icons.filled.SmartToy
import androidx.compose.material.icons.filled.Warning
import androidx.compose.material.icons.filled.Wifi
import androidx.compose.material3.AlertDialog
import androidx.compose.material3.Button
import androidx.compose.material3.Icon
import androidx.compose.material3.IconButton
import androidx.compose.material3.LoadingIndicator
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.OutlinedButton
import androidx.compose.material3.OutlinedTextField
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
import androidx.compose.runtime.saveable.rememberSaveable
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.platform.LocalContext
import androidx.compose.ui.res.stringResource
import androidx.compose.ui.unit.dp
import androidx.compose.ui.window.Dialog
import androidx.compose.ui.window.DialogProperties
import androidx.lifecycle.compose.collectAsStateWithLifecycle
import androidx.lifecycle.viewmodel.compose.viewModel
import io.github.flexy06.openneato.R

// Confirmations that ask before something drastic happens
private enum class Ask { NONE, DISCARD, SAVE_REBOOT, RESTART, ROBOT_RESTART, SHUTDOWN, CLEAR_ERRORS, NEW_BATTERY, FORGET_WIFI, UPLOAD, FORMAT, RESET }

@Composable
fun SettingsScreen(
    onBack: () -> Unit,
    onOpenLogs: () -> Unit,
    onChangeRobot: () -> Unit,
    vm: SettingsViewModel = viewModel(),
) {
    val ui by vm.ui.collectAsStateWithLifecycle()
    val theme by vm.theme.collectAsStateWithLifecycle()
    val robotUrl by vm.robotUrl.collectAsStateWithLifecycle()
    val snackbar = remember { SnackbarHostState() }
    val context = LocalContext.current
    var page by rememberSaveable { mutableStateOf(SettingsPage.HUB) }
    var ask by remember { mutableStateOf(Ask.NONE) }
    var leaveAfterDiscard by remember { mutableStateOf(false) }

    LaunchedEffect(page) { vm.open(page) }
    LaunchedEffect(Unit) {
        vm.messages.collect { e ->
            val text = when (e) {
                SettingsEvent.Saved -> context.getString(R.string.settings_saved)
                SettingsEvent.Back -> context.getString(R.string.bridge_back)
                SettingsEvent.RebootTimeout -> context.getString(R.string.reboot_timeout)
                SettingsEvent.RobotBack -> context.getString(R.string.robot_back)
                SettingsEvent.RobotTimeout -> context.getString(R.string.robot_timeout)
                SettingsEvent.NotificationSent -> context.getString(R.string.ntfy_sent)
                SettingsEvent.ErrorsCleared -> context.getString(R.string.errors_cleared)
                SettingsEvent.NewBatterySet -> context.getString(R.string.new_battery_done)
                SettingsEvent.WifiConnecting -> context.getString(R.string.wifi_connecting)
                SettingsEvent.Shutdown -> context.getString(R.string.shutdown_done)
                SettingsEvent.Reset -> {
                    onChangeRobot() // The bridge is gone from this network: pick it again
                    context.getString(R.string.factory_reset_done)
                }
                is SettingsEvent.Failed -> context.getString(R.string.failed_with, e.message)
            }
            snackbar.showSnackbar(text)
        }
    }

    // Back: sub page -> hub -> leave (asking first about unsaved changes)
    val back = {
        when {
            page != SettingsPage.HUB -> page = SettingsPage.HUB
            ui.dirty -> {
                leaveAfterDiscard = true
                ask = Ask.DISCARD
            }
            else -> onBack()
        }
    }
    BackHandler(onBack = back)

    Scaffold(
        topBar = {
            TopAppBar(
                title = { Text(stringResource(pageTitle(page))) },
                navigationIcon = {
                    IconButton(onClick = back) {
                        Icon(Icons.AutoMirrored.Filled.ArrowBack, contentDescription = stringResource(R.string.back))
                    }
                },
            )
        },
        snackbarHost = { SnackbarHost(snackbar) },
        bottomBar = {
            if (ui.dirty) {
                SaveBar(
                    saving = ui.saving,
                    reboot = ui.needsReboot,
                    valid = ui.edited.problem() == null,
                    onDiscard = vm::discard,
                    onSave = { if (ui.needsReboot) ask = Ask.SAVE_REBOOT else vm.save() },
                )
            }
        },
    ) { padding ->
        when {
            ui.loading -> Box(Modifier.fillMaxSize().padding(padding), contentAlignment = Alignment.Center) {
                LoadingIndicator(Modifier.size(56.dp))
            }
            ui.loadError != null && page != SettingsPage.APP -> Column(
                Modifier.fillMaxSize().padding(padding).padding(24.dp),
                horizontalAlignment = Alignment.CenterHorizontally,
                verticalArrangement = Arrangement.Center,
            ) {
                Text(ui.loadError ?: "", color = MaterialTheme.colorScheme.error)
                Spacer(Modifier.height(12.dp))
                Row(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
                    OutlinedButton(onClick = vm::load) { Text(stringResource(R.string.retry)) }
                    OutlinedButton(onClick = { page = SettingsPage.APP }) { Text(stringResource(R.string.settings_app)) }
                }
            }
            else -> Column(
                Modifier
                    .fillMaxSize()
                    .padding(padding)
                    .verticalScroll(rememberScrollState())
                    .padding(16.dp),
                verticalArrangement = Arrangement.spacedBy(12.dp),
            ) {
                when (page) {
                    SettingsPage.HUB -> Hub { page = it }
                    SettingsPage.APP -> AppPage(theme, robotUrl, vm::setTheme) { vm.forgetRobot(onChangeRobot) }
                    SettingsPage.DEVICE -> DevicePage(ui, vm) { ask = Ask.RESTART }
                    SettingsPage.WIFI -> WifiPage(ui, vm) { ask = Ask.FORGET_WIFI }
                    SettingsPage.NOTIFICATIONS -> NotificationsPage(ui, vm)
                    SettingsPage.CLEANING -> CleaningPage(ui, vm)
                    SettingsPage.ROBOT -> RobotPage(ui, vm, { ask = Ask.ROBOT_RESTART }, { ask = Ask.SHUTDOWN })
                    SettingsPage.FIRMWARE -> FirmwarePage(ui, vm) { ask = Ask.UPLOAD }
                    SettingsPage.DIAGNOSTICS -> DiagnosticsPage(ui, vm, onOpenLogs, { ask = Ask.CLEAR_ERRORS }, { ask = Ask.NEW_BATTERY })
                    SettingsPage.ABOUT -> AboutPage(ui)
                    SettingsPage.DANGER -> DangerPage(ui, { ask = Ask.FORMAT }, { ask = Ask.RESET })
                }
            }
        }
    }

    if (ui.rebooting || ui.robotRestarting) Waiting(ui.robotRestarting)

    val close = { ask = Ask.NONE }
    when (ask) {
        Ask.NONE -> {}
        Ask.DISCARD -> ConfirmDialog(stringResource(R.string.unsaved_changes), stringResource(R.string.discard), onConfirm = {
            close()
            vm.discard()
            if (leaveAfterDiscard) onBack()
        }, onDismiss = close)
        Ask.SAVE_REBOOT -> ConfirmDialog(stringResource(R.string.save_reboot_q), stringResource(R.string.save_reboot), onConfirm = {
            close()
            vm.save()
        }, onDismiss = close)
        Ask.RESTART -> ConfirmDialog(stringResource(R.string.restart_device_q), stringResource(R.string.restart), onConfirm = {
            close()
            vm.restartBridge()
        }, onDismiss = close)
        Ask.ROBOT_RESTART -> ConfirmDialog(stringResource(R.string.restart_robot_q), stringResource(R.string.restart), onConfirm = {
            close()
            vm.restartRobot()
        }, onDismiss = close)
        Ask.SHUTDOWN -> ConfirmDialog(stringResource(R.string.shutdown_robot_q), stringResource(R.string.shutdown_robot), danger = true, onConfirm = {
            close()
            vm.shutdownRobot()
        }, onDismiss = close)
        Ask.CLEAR_ERRORS -> ConfirmDialog(stringResource(R.string.clear_errors_q), stringResource(R.string.clear_errors), onConfirm = {
            close()
            vm.clearErrors()
        }, onDismiss = close)
        Ask.NEW_BATTERY -> ConfirmDialog(stringResource(R.string.new_battery_q), stringResource(R.string.new_battery), onConfirm = {
            close()
            vm.newBattery()
        }, onDismiss = close)
        Ask.FORGET_WIFI -> ConfirmDialog(stringResource(R.string.wifi_forget_q), stringResource(R.string.wifi_forget), danger = true, onConfirm = {
            close()
            vm.forgetWifi()
        }, onDismiss = close)
        Ask.UPLOAD -> ConfirmDialog(stringResource(R.string.fw_no_checksum_q), stringResource(R.string.fw_upload), danger = true, onConfirm = {
            close()
            vm.uploadFirmware()
        }, onDismiss = close)
        Ask.FORMAT -> ConfirmDialog(stringResource(R.string.format_storage_q), stringResource(R.string.format_storage), danger = true, onConfirm = {
            close()
            vm.formatStorage()
        }, onDismiss = close)
        Ask.RESET -> ResetDialog(onDismiss = close) {
            close()
            vm.factoryReset()
        }
    }
}

private fun pageTitle(p: SettingsPage) = when (p) {
    SettingsPage.HUB -> R.string.settings
    SettingsPage.APP -> R.string.settings_app
    SettingsPage.DEVICE -> R.string.settings_device
    SettingsPage.WIFI -> R.string.settings_wifi
    SettingsPage.NOTIFICATIONS -> R.string.settings_notifications
    SettingsPage.CLEANING -> R.string.settings_cleaning
    SettingsPage.ROBOT -> R.string.settings_robot
    SettingsPage.FIRMWARE -> R.string.settings_firmware
    SettingsPage.DIAGNOSTICS -> R.string.settings_diagnostics
    SettingsPage.ABOUT -> R.string.settings_about
    SettingsPage.DANGER -> R.string.settings_danger
}

@Composable
private fun Hub(open: (SettingsPage) -> Unit) {
    SettingsGroup {
        HubRow(Icons.Filled.Palette, stringResource(R.string.settings_app), stringResource(R.string.settings_app_desc)) { open(SettingsPage.APP) }
    }
    SettingsGroup {
        HubRow(Icons.Filled.Router, stringResource(R.string.settings_device), stringResource(R.string.settings_device_desc)) { open(SettingsPage.DEVICE) }
        RowDivider()
        HubRow(Icons.Filled.Wifi, stringResource(R.string.settings_wifi), stringResource(R.string.settings_wifi_desc)) { open(SettingsPage.WIFI) }
        RowDivider()
        HubRow(Icons.Filled.Notifications, stringResource(R.string.settings_notifications), stringResource(R.string.settings_notifications_desc)) {
            open(SettingsPage.NOTIFICATIONS)
        }
        RowDivider()
        HubRow(Icons.Filled.CleaningServices, stringResource(R.string.settings_cleaning), stringResource(R.string.settings_cleaning_desc)) {
            open(SettingsPage.CLEANING)
        }
        RowDivider()
        HubRow(Icons.Filled.SmartToy, stringResource(R.string.settings_robot), stringResource(R.string.settings_robot_desc)) { open(SettingsPage.ROBOT) }
    }
    SettingsGroup {
        HubRow(Icons.Filled.Memory, stringResource(R.string.settings_firmware), stringResource(R.string.settings_firmware_desc)) { open(SettingsPage.FIRMWARE) }
        RowDivider()
        HubRow(Icons.Filled.MonitorHeart, stringResource(R.string.settings_diagnostics), stringResource(R.string.settings_diagnostics_desc)) {
            open(SettingsPage.DIAGNOSTICS)
        }
        RowDivider()
        HubRow(Icons.Filled.Info, stringResource(R.string.settings_about), "OpenNeato") { open(SettingsPage.ABOUT) }
    }
    SettingsGroup {
        HubRow(
            Icons.Filled.Warning,
            stringResource(R.string.settings_danger),
            stringResource(R.string.settings_danger_desc),
            onClick = { open(SettingsPage.DANGER) },
            tint = MaterialTheme.colorScheme.error,
        )
    }
}

@Composable
private fun SaveBar(saving: Boolean, reboot: Boolean, valid: Boolean, onDiscard: () -> Unit, onSave: () -> Unit) {
    Surface(color = MaterialTheme.colorScheme.surfaceContainer) {
        Row(
            Modifier.fillMaxWidth().navigationBarsPadding().padding(16.dp),
            horizontalArrangement = Arrangement.spacedBy(12.dp),
        ) {
            OutlinedButton(onClick = onDiscard, enabled = !saving, modifier = Modifier.weight(1f).height(52.dp)) {
                Text(stringResource(R.string.discard))
            }
            Button(onClick = onSave, enabled = !saving && valid, modifier = Modifier.weight(1.4f).height(52.dp)) {
                Text(stringResource(if (saving) R.string.saving else if (reboot) R.string.save_reboot else R.string.save))
            }
        }
    }
}

@Composable
private fun Waiting(robot: Boolean) {
    Dialog(onDismissRequest = {}, properties = DialogProperties(dismissOnBackPress = false, dismissOnClickOutside = false)) {
        Surface(shape = MaterialTheme.shapes.large, color = MaterialTheme.colorScheme.surfaceContainerHigh) {
            Column(Modifier.padding(28.dp), horizontalAlignment = Alignment.CenterHorizontally) {
                LoadingIndicator(Modifier.size(56.dp))
                Spacer(Modifier.height(12.dp))
                Text(stringResource(if (robot) R.string.robot_restarting else R.string.rebooting), style = MaterialTheme.typography.titleMedium)
                if (!robot) {
                    Text(
                        stringResource(R.string.rebooting_wait),
                        style = MaterialTheme.typography.bodySmall,
                        color = MaterialTheme.colorScheme.onSurfaceVariant,
                    )
                }
            }
        }
    }
}

// Factory reset needs RESET typed, like the web UI
@Composable
private fun ResetDialog(onDismiss: () -> Unit, onConfirm: () -> Unit) {
    var typed by remember { mutableStateOf("") }
    AlertDialog(
        onDismissRequest = onDismiss,
        text = {
            Column {
                Text(stringResource(R.string.factory_reset_q))
                OutlinedTextField(
                    value = typed,
                    onValueChange = { typed = it },
                    singleLine = true,
                    placeholder = { Text("RESET") },
                    modifier = Modifier.fillMaxWidth().padding(top = 12.dp),
                )
            }
        },
        confirmButton = {
            TextButton(onClick = onConfirm, enabled = typed.trim() == "RESET") {
                Text(stringResource(R.string.factory_reset), color = MaterialTheme.colorScheme.error)
            }
        },
        dismissButton = { TextButton(onClick = onDismiss) { Text(stringResource(R.string.cancel)) } },
    )
}

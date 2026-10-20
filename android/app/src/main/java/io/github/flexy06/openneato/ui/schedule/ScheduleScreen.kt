package io.github.flexy06.openneato.ui.schedule

import android.text.format.DateFormat
import androidx.activity.compose.BackHandler
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.PaddingValues
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
import androidx.compose.foundation.verticalScroll
import androidx.compose.material.icons.Icons
import androidx.compose.material.icons.automirrored.filled.ArrowBack
import androidx.compose.material.icons.filled.Add
import androidx.compose.material.icons.filled.Close
import androidx.compose.material3.AlertDialog
import androidx.compose.material3.Button
import androidx.compose.material3.Card
import androidx.compose.material3.CardDefaults
import androidx.compose.material3.FilledTonalButton
import androidx.compose.material3.Icon
import androidx.compose.material3.IconButton
import androidx.compose.material3.LoadingIndicator
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.OutlinedButton
import androidx.compose.material3.Scaffold
import androidx.compose.material3.SnackbarHost
import androidx.compose.material3.SnackbarHostState
import androidx.compose.material3.Surface
import androidx.compose.material3.Switch
import androidx.compose.material3.Text
import androidx.compose.material3.TextButton
import androidx.compose.material3.TimePicker
import androidx.compose.material3.TopAppBar
import androidx.compose.material3.rememberTimePickerState
import androidx.compose.runtime.Composable
import androidx.compose.runtime.LaunchedEffect
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.platform.LocalContext
import androidx.compose.ui.res.stringResource
import androidx.compose.ui.unit.dp
import androidx.lifecycle.compose.collectAsStateWithLifecycle
import androidx.lifecycle.viewmodel.compose.viewModel
import io.github.flexy06.openneato.R
import io.github.flexy06.openneato.ui.dashboard.clockOf
import java.time.DayOfWeek
import java.time.format.TextStyle
import java.util.Locale

private data class Editing(val day: Int, val slot: Int)

@Composable
fun ScheduleScreen(onBack: () -> Unit, vm: ScheduleViewModel = viewModel()) {
    val ui by vm.ui.collectAsStateWithLifecycle()
    val snackbar = remember { SnackbarHostState() }
    val context = LocalContext.current
    var confirmDiscard by remember { mutableStateOf(false) }
    var editing by remember { mutableStateOf<Editing?>(null) }

    LaunchedEffect(Unit) {
        vm.messages.collect { e ->
            snackbar.showSnackbar(
                when (e) {
                    ScheduleEvent.Saved -> context.getString(R.string.schedule_saved)
                    is ScheduleEvent.Failed -> context.getString(R.string.schedule_save_failed, e.message)
                },
            )
        }
    }
    val leave = { if (ui.dirty) confirmDiscard = true else onBack() }
    BackHandler(onBack = leave)

    Scaffold(
        topBar = {
            TopAppBar(
                title = { Text(stringResource(R.string.schedule)) },
                navigationIcon = {
                    IconButton(onClick = leave) {
                        Icon(Icons.AutoMirrored.Filled.ArrowBack, contentDescription = stringResource(R.string.back))
                    }
                },
            )
        },
        snackbarHost = { SnackbarHost(snackbar) },
        bottomBar = {
            if (!ui.loading && ui.loadError == null) {
                Surface(color = MaterialTheme.colorScheme.surfaceContainer) {
                    Button(
                        onClick = vm::save,
                        enabled = ui.dirty && !ui.saving,
                        modifier = Modifier
                            .fillMaxWidth()
                            .navigationBarsPadding()
                            .padding(16.dp)
                            .height(56.dp),
                    ) {
                        Text(stringResource(if (ui.saving) R.string.saving else R.string.save))
                    }
                }
            }
        },
    ) { padding ->
        when {
            ui.loading -> Column(
                Modifier.fillMaxSize().padding(padding),
                horizontalAlignment = Alignment.CenterHorizontally,
                verticalArrangement = Arrangement.Center,
            ) {
                LoadingIndicator(Modifier.size(56.dp))
                Text(stringResource(R.string.schedule_loading))
            }
            ui.loadError != null -> Column(
                Modifier.fillMaxSize().padding(padding).padding(24.dp),
                horizontalAlignment = Alignment.CenterHorizontally,
                verticalArrangement = Arrangement.Center,
            ) {
                Text(ui.loadError ?: "", color = MaterialTheme.colorScheme.error)
                Spacer(Modifier.height(12.dp))
                OutlinedButton(onClick = vm::load) { Text(stringResource(R.string.retry)) }
            }
            else -> Column(
                Modifier
                    .fillMaxSize()
                    .padding(padding)
                    .verticalScroll(rememberScrollState())
                    .padding(16.dp),
                verticalArrangement = Arrangement.spacedBy(10.dp),
            ) {
                MasterSwitch(ui.edited.enabled) { on -> vm.edit { it.copy(enabled = on) } }
                Text(
                    stringResource(R.string.schedule_tz, ui.robotTime?.let(::clockOf) ?: "–", ui.tz),
                    style = MaterialTheme.typography.bodySmall,
                    color = MaterialTheme.colorScheme.onSurfaceVariant,
                    modifier = Modifier.padding(horizontal = 4.dp),
                )
                ui.edited.days.forEachIndexed { d, day ->
                    DayRow(
                        day = d,
                        state = day,
                        scheduleOn = ui.edited.enabled,
                        onToggle = { on -> vm.edit { it.updateSlot(d, 0) { s -> s.copy(on = on) } } },
                        onPick = { slot -> editing = Editing(d, slot) },
                        onAddSecond = { vm.edit { it.updateSlot(d, 1) { Slot(15, 0, true) } } },
                        onRemoveSecond = { vm.edit { it.updateSlot(d, 1) { s -> s.copy(on = false) } } },
                    )
                }
            }
        }
    }

    editing?.let { e ->
        val slot = ui.edited.days[e.day].slots[e.slot]
        TimeDialog(
            hour = slot.hour,
            minute = slot.minute,
            onDismiss = { editing = null },
            onConfirm = { h, m ->
                vm.edit { it.updateSlot(e.day, e.slot) { s -> s.copy(hour = h, minute = m) } }
                editing = null
            },
        )
    }
    if (confirmDiscard) {
        AlertDialog(
            onDismissRequest = { confirmDiscard = false },
            text = { Text(stringResource(R.string.unsaved_changes)) },
            confirmButton = {
                TextButton(onClick = {
                    confirmDiscard = false
                    onBack()
                }) { Text(stringResource(R.string.discard)) }
            },
            dismissButton = { TextButton(onClick = { confirmDiscard = false }) { Text(stringResource(R.string.cancel)) } },
        )
    }
}

@Composable
private fun MasterSwitch(on: Boolean, onChange: (Boolean) -> Unit) {
    Card(colors = CardDefaults.cardColors(containerColor = MaterialTheme.colorScheme.surfaceContainer)) {
        Row(Modifier.padding(16.dp), verticalAlignment = Alignment.CenterVertically) {
            Column(Modifier.weight(1f)) {
                Text(stringResource(R.string.schedule_enabled), style = MaterialTheme.typography.titleMedium)
                Text(
                    stringResource(R.string.schedule_desc),
                    style = MaterialTheme.typography.bodySmall,
                    color = MaterialTheme.colorScheme.onSurfaceVariant,
                )
            }
            Switch(checked = on, onCheckedChange = onChange)
        }
    }
}

private fun time(s: Slot) = "%02d:%02d".format(s.hour, s.minute)

private val TimeWidth = 84.dp

@Composable
private fun TimeButton(slot: Slot, onClick: () -> Unit) {
    FilledTonalButton(
        onClick = onClick,
        contentPadding = PaddingValues(horizontal = 8.dp),
        modifier = Modifier.width(TimeWidth),
    ) { Text(time(slot), maxLines = 1) }
}

@Composable
private fun DayRow(
    day: Int,
    state: Day,
    scheduleOn: Boolean,
    onToggle: (Boolean) -> Unit,
    onPick: (Int) -> Unit,
    onAddSecond: () -> Unit,
    onRemoveSecond: () -> Unit,
) {
    val (s0, s1) = state.slots
    // Monday first, localized (Montag, Dienstag, ...)
    val name = DayOfWeek.of(day + 1).getDisplayName(TextStyle.FULL, Locale.getDefault())
    Card(
        colors = CardDefaults.cardColors(
            containerColor = if (s0.on && scheduleOn) {
                MaterialTheme.colorScheme.surfaceContainer
            } else {
                MaterialTheme.colorScheme.surfaceContainerLow
            },
        ),
    ) {
        Row(
            Modifier.fillMaxWidth().padding(horizontal = 12.dp, vertical = 8.dp),
            verticalAlignment = Alignment.CenterVertically,
        ) {
            Switch(checked = s0.on, onCheckedChange = onToggle)
            Spacer(Modifier.width(12.dp))
            Text(
                name,
                style = MaterialTheme.typography.titleSmall,
                color = if (s0.on) MaterialTheme.colorScheme.onSurface else MaterialTheme.colorScheme.onSurfaceVariant,
                modifier = Modifier.weight(1f),
            )
            if (s0.on) {
                // Fixed widths: the first time stays in its column whether or not there is a second one
                TimeButton(s0) { onPick(0) }
                Spacer(Modifier.width(6.dp))
                if (s1.on) {
                    TimeButton(s1) { onPick(1) }
                    IconButton(onClick = onRemoveSecond) {
                        Icon(Icons.Filled.Close, contentDescription = stringResource(R.string.schedule_remove_slot))
                    }
                } else {
                    Spacer(Modifier.width(TimeWidth))
                    IconButton(onClick = onAddSecond) {
                        Icon(Icons.Filled.Add, contentDescription = stringResource(R.string.schedule_add_slot))
                    }
                }
            }
        }
    }
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

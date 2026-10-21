package io.github.flexy06.openneato.ui.history

import android.content.Context
import android.content.Intent
import android.text.format.DateUtils
import androidx.activity.compose.rememberLauncherForActivityResult
import androidx.activity.result.contract.ActivityResultContracts
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.PaddingValues
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.size
import androidx.compose.foundation.layout.width
import androidx.compose.foundation.lazy.LazyColumn
import androidx.compose.foundation.lazy.items
import androidx.compose.material.icons.Icons
import androidx.compose.material.icons.automirrored.filled.ArrowBack
import androidx.compose.material.icons.automirrored.filled.KeyboardArrowRight
import androidx.compose.material.icons.filled.CenterFocusStrong
import androidx.compose.material.icons.filled.Delete
import androidx.compose.material.icons.filled.DeleteSweep
import androidx.compose.material.icons.filled.FileUpload
import androidx.compose.material.icons.filled.Gamepad
import androidx.compose.material.icons.filled.Home
import androidx.compose.material.icons.filled.Map
import androidx.compose.material.icons.filled.Schedule
import androidx.compose.material3.AlertDialog
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
import androidx.compose.ui.graphics.vector.ImageVector
import androidx.compose.ui.platform.LocalContext
import androidx.compose.ui.res.pluralStringResource
import androidx.compose.ui.res.stringResource
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.dp
import androidx.core.net.toUri
import androidx.lifecycle.compose.collectAsStateWithLifecycle
import androidx.lifecycle.viewmodel.compose.viewModel
import io.github.flexy06.openneato.R
import io.github.flexy06.openneato.api.models.HistoryFileInfo
import io.github.flexy06.openneato.ui.theme.LocalStatusColors

private const val RECOVERY_GUIDE_URL =
    "https://github.com/renjfk/OpenNeato/blob/main/docs/user-guide.md#recovering-corrupted-cleaning-history"

@Composable
fun HistoryListScreen(onBack: () -> Unit, onOpen: (String) -> Unit, vm: HistoryListViewModel = viewModel()) {
    val ui by vm.ui.collectAsStateWithLifecycle()
    val snackbar = remember { SnackbarHostState() }
    val context = LocalContext.current
    var confirm by remember { mutableStateOf<String?>(null) } // File name, or "" = all
    val pick = rememberLauncherForActivityResult(ActivityResultContracts.OpenDocument()) { uri ->
        if (uri != null) vm.import(uri)
    }

    LaunchedEffect(Unit) {
        vm.messages.collect { snackbar.showSnackbar(context.historyMessage(it)) }
    }

    Scaffold(
        topBar = {
            TopAppBar(
                title = { Text(stringResource(R.string.history_title)) },
                navigationIcon = {
                    IconButton(onClick = onBack) {
                        Icon(Icons.AutoMirrored.Filled.ArrowBack, contentDescription = stringResource(R.string.back))
                    }
                },
                actions = {
                    IconButton(onClick = { pick.launch(arrayOf("*/*")) }, enabled = !ui.busy) {
                        Icon(Icons.Filled.FileUpload, contentDescription = stringResource(R.string.history_import))
                    }
                    IconButton(onClick = { confirm = "" }, enabled = !ui.busy && ui.files.isNotEmpty()) {
                        Icon(Icons.Filled.DeleteSweep, contentDescription = stringResource(R.string.history_delete_all))
                    }
                },
            )
        },
        snackbarHost = { SnackbarHost(snackbar) },
    ) { padding ->
        when {
            ui.loading -> Box(Modifier.fillMaxSize().padding(padding), contentAlignment = Alignment.Center) {
                LoadingIndicator(Modifier.size(56.dp))
            }
            ui.corrupted -> Corrupted(Modifier.padding(padding), busy = ui.busy, onDeleteAll = { confirm = "" })
            ui.files.isEmpty() -> Column(
                Modifier.fillMaxSize().padding(padding).padding(24.dp),
                horizontalAlignment = Alignment.CenterHorizontally,
                verticalArrangement = Arrangement.Center,
            ) {
                Text(
                    ui.error ?: stringResource(R.string.history_empty),
                    color = if (ui.error != null) MaterialTheme.colorScheme.error else MaterialTheme.colorScheme.onSurfaceVariant,
                )
            }
            else -> LazyColumn(
                Modifier.fillMaxSize().padding(padding),
                contentPadding = PaddingValues(16.dp),
                verticalArrangement = Arrangement.spacedBy(8.dp),
            ) {
                item {
                    Text(
                        pluralStringResource(R.plurals.history_sessions, ui.files.size, ui.files.size),
                        style = MaterialTheme.typography.labelLarge,
                        color = MaterialTheme.colorScheme.onSurfaceVariant,
                        modifier = Modifier.padding(horizontal = 4.dp, vertical = 4.dp),
                    )
                }
                items(ui.files, key = { it.name }) { f ->
                    SessionCard(f, onOpen = { onOpen(f.name) }, onDelete = { confirm = f.name })
                }
            }
        }
    }

    confirm?.let { target ->
        AlertDialog(
            onDismissRequest = { confirm = null },
            text = {
                Text(stringResource(if (target.isEmpty()) R.string.history_delete_all_q else R.string.history_delete_one_q))
            },
            confirmButton = {
                TextButton(onClick = {
                    confirm = null
                    if (target.isEmpty()) vm.deleteAll() else vm.delete(target)
                }) { Text(stringResource(R.string.history_delete)) }
            },
            dismissButton = { TextButton(onClick = { confirm = null }) { Text(stringResource(R.string.cancel)) } },
        )
    }
}

@Composable
private fun Corrupted(modifier: Modifier, busy: Boolean, onDeleteAll: () -> Unit) {
    val context = LocalContext.current
    Column(modifier.fillMaxSize().padding(24.dp), verticalArrangement = Arrangement.spacedBy(12.dp)) {
        Text(stringResource(R.string.history_corrupt_title), style = MaterialTheme.typography.titleLarge)
        Text(stringResource(R.string.history_corrupt_text), color = MaterialTheme.colorScheme.onSurfaceVariant)
        Row(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
            OutlinedButton(onClick = {
                context.startActivity(Intent(Intent.ACTION_VIEW, RECOVERY_GUIDE_URL.toUri()))
            }) { Text(stringResource(R.string.history_guide)) }
            FilledTonalButton(onClick = onDeleteAll, enabled = !busy) { Text(stringResource(R.string.history_delete_all)) }
        }
    }
}

@Composable
private fun SessionCard(f: HistoryFileInfo, onOpen: () -> Unit, onDelete: () -> Unit) {
    val context = LocalContext.current
    val status = LocalStatusColors.current
    val mode = f.session?.mode.orEmpty()
    Card(
        onClick = onOpen,
        colors = CardDefaults.cardColors(containerColor = MaterialTheme.colorScheme.surfaceContainer),
    ) {
        Row(Modifier.fillMaxWidth().padding(start = 16.dp, top = 12.dp, bottom = 12.dp), verticalAlignment = Alignment.CenterVertically) {
            Surface(shape = MaterialTheme.shapes.small, color = MaterialTheme.colorScheme.secondaryContainer) {
                Icon(
                    modeIcon(mode, f.session?.scans == true),
                    contentDescription = null,
                    tint = MaterialTheme.colorScheme.onSecondaryContainer,
                    modifier = Modifier.padding(8.dp).size(22.dp),
                )
            }
            Spacer(Modifier.width(12.dp))
            Column(Modifier.weight(1f)) {
                Row(verticalAlignment = Alignment.CenterVertically) {
                    Text(
                        modeLabel(mode, f.session?.scans == true),
                        style = MaterialTheme.typography.titleSmall,
                        fontWeight = FontWeight.SemiBold,
                    )
                    if (f.recording) {
                        Spacer(Modifier.width(8.dp))
                        Surface(shape = MaterialTheme.shapes.small, color = status.green.copy(alpha = 0.18f)) {
                            Text(
                                stringResource(R.string.history_running),
                                style = MaterialTheme.typography.labelSmall,
                                color = status.green,
                                modifier = Modifier.padding(horizontal = 6.dp, vertical = 2.dp),
                            )
                        }
                    }
                }
                f.session?.let {
                    Text(
                        formatDateTime(context, it.time.toLong()),
                        style = MaterialTheme.typography.bodySmall,
                        color = MaterialTheme.colorScheme.onSurfaceVariant,
                    )
                }
                f.summary?.let { s ->
                    Spacer(Modifier.height(2.dp))
                    Text(
                        listOf(
                            formatDuration(s.duration.toDouble()),
                            formatDistance(s.distanceTraveled.toDouble()),
                            formatArea(s.areaCovered.toDouble()),
                            "${f.session?.battery?.toInt() ?: "?"} → ${s.batteryEnd?.toInt() ?: "?"} %",
                        ).joinToString(" · "),
                        style = MaterialTheme.typography.bodySmall,
                        color = MaterialTheme.colorScheme.onSurfaceVariant,
                    )
                }
            }
            if (!f.recording) {
                IconButton(onClick = onDelete) {
                    Icon(Icons.Filled.Delete, contentDescription = stringResource(R.string.history_delete))
                }
            } else {
                Icon(
                    Icons.AutoMirrored.Filled.KeyboardArrowRight,
                    contentDescription = null,
                    modifier = Modifier.padding(end = 12.dp),
                )
            }
        }
    }
}

// -- Shared formatting (list and detail) ---------------------------------------------------

fun modeIcon(mode: String, mapping: Boolean): ImageVector = when {
    mapping -> Icons.Filled.Map
    mode.equals("house", true) -> Icons.Filled.Home
    mode.equals("spot", true) -> Icons.Filled.CenterFocusStrong
    mode.equals("manual", true) -> Icons.Filled.Gamepad
    else -> Icons.Filled.Schedule
}

@Composable
fun modeLabel(mode: String, mapping: Boolean): String = when {
    mapping -> stringResource(R.string.history_mapping_run)
    mode.equals("house", true) -> stringResource(R.string.mode_house)
    mode.equals("spot", true) -> stringResource(R.string.mode_spot)
    mode.equals("manual", true) -> stringResource(R.string.mode_manual)
    else -> mode
}

fun formatDateTime(context: Context, epochS: Long): String = DateUtils.formatDateTime(
    context,
    epochS * 1000,
    DateUtils.FORMAT_SHOW_DATE or DateUtils.FORMAT_SHOW_TIME or DateUtils.FORMAT_SHOW_WEEKDAY or DateUtils.FORMAT_ABBREV_ALL,
)

fun formatDuration(seconds: Double): String {
    val s = seconds.toLong().coerceAtLeast(0)
    val h = s / 3600
    val m = (s % 3600) / 60
    return if (h > 0) "%d h %02d min".format(h, m) else if (m > 0) "%d min".format(m) else "%d s".format(s)
}

// Replay clock: m:ss or h:mm:ss
fun formatClock(seconds: Double): String {
    val s = seconds.toLong().coerceAtLeast(0)
    return if (s >= 3600) "%d:%02d:%02d".format(s / 3600, (s % 3600) / 60, s % 60) else "%d:%02d".format(s / 60, s % 60)
}

fun formatDistance(m: Double) = "%.1f m".format(m)

fun formatArea(m2: Double) = "%.1f m²".format(m2)

fun Context.historyMessage(e: HistoryEvent): String = when (e) {
    is HistoryEvent.Failed -> getString(R.string.failed_with, e.message)
    HistoryEvent.Imported -> getString(R.string.history_imported)
    HistoryEvent.Exported -> getString(R.string.history_exported)
    is HistoryEvent.Rejected -> getString(
        when (e.problem) {
            ImportProblem.NOT_JSONL -> R.string.import_not_jsonl
            ImportProblem.TOO_LARGE -> R.string.import_too_large
            ImportProblem.NO_HEADER -> R.string.import_no_header
            ImportProblem.NAME_MISMATCH -> R.string.import_name_mismatch
            ImportProblem.EXISTS -> R.string.import_exists
        },
    )
}

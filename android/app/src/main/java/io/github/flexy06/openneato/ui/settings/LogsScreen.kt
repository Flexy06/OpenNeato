package io.github.flexy06.openneato.ui.settings

import android.app.Application
import android.net.Uri
import androidx.activity.compose.BackHandler
import androidx.activity.compose.rememberLauncherForActivityResult
import androidx.activity.result.contract.ActivityResultContracts
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.PaddingValues
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.size
import androidx.compose.foundation.lazy.LazyColumn
import androidx.compose.foundation.lazy.items
import androidx.compose.material.icons.Icons
import androidx.compose.material.icons.automirrored.filled.ArrowBack
import androidx.compose.material.icons.filled.Delete
import androidx.compose.material.icons.filled.DeleteSweep
import androidx.compose.material.icons.filled.Description
import androidx.compose.material.icons.filled.SaveAlt
import androidx.compose.material3.Icon
import androidx.compose.material3.IconButton
import androidx.compose.material3.LoadingIndicator
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.OutlinedTextField
import androidx.compose.material3.Scaffold
import androidx.compose.material3.SnackbarHost
import androidx.compose.material3.SnackbarHostState
import androidx.compose.material3.Text
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
import androidx.compose.ui.text.font.FontFamily
import androidx.compose.ui.unit.dp
import androidx.lifecycle.AndroidViewModel
import androidx.lifecycle.compose.collectAsStateWithLifecycle
import androidx.lifecycle.viewModelScope
import androidx.lifecycle.viewmodel.compose.viewModel
import io.github.flexy06.openneato.OpenNeatoApp
import io.github.flexy06.openneato.R
import io.github.flexy06.openneato.api.models.LogFileInfo
import kotlinx.coroutines.CancellationException
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.channels.Channel
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.asStateFlow
import kotlinx.coroutines.flow.filterNotNull
import kotlinx.coroutines.flow.first
import kotlinx.coroutines.flow.receiveAsFlow
import kotlinx.coroutines.flow.update
import kotlinx.coroutines.launch
import kotlinx.coroutines.withContext
import kotlinx.serialization.json.Json
import kotlinx.serialization.json.JsonObject
import kotlinx.serialization.json.JsonPrimitive
import java.text.SimpleDateFormat
import java.util.Date
import java.util.Locale

// One log line ({"t":epoch,"typ":"...","d":{...}}) for reading: local time, type, key=value
data class LogLine(val time: String, val type: String, val body: String) {
    companion object {
        private val fmt = SimpleDateFormat("dd.MM. HH:mm:ss", Locale.getDefault())

        fun parse(line: String): LogLine? {
            val o = runCatching { Json.parseToJsonElement(line) as? JsonObject }
                .getOrNull() ?: return null
            val t = (o["t"] as? JsonPrimitive)?.content?.toLongOrNull() ?: return null
            val type = (o["typ"] as? JsonPrimitive)?.content ?: ""
            val d = o["d"] as? JsonObject
            val body = d?.entries?.joinToString("  ") { (k, v) ->
                val text = (v as? JsonPrimitive)?.content ?: v.toString()
                // Serial responses carry their line breaks: show them inline
                "$k=" + text.replace("\r\n", " | ").replace("\n", " | ").trimEnd(' ', '|')
            } ?: ""
            return LogLine(fmt.format(Date(t * 1000)), type, body)
        }
    }
}

data class LogsUi(
    val loading: Boolean = true,
    val files: List<LogFileInfo> = emptyList(),
    val open: String? = null,
    val lines: List<String> = emptyList(),
    val loadingFile: Boolean = false,
    val error: String? = null,
)

class LogsViewModel(app: Application) : AndroidViewModel(app) {
    private val container = app as OpenNeatoApp
    private val client = container.robot
    private val _ui = MutableStateFlow(LogsUi())
    val ui = _ui.asStateFlow()
    private val events = Channel<String>(Channel.BUFFERED)
    val messages = events.receiveAsFlow()
    private var text = ""

    init {
        refresh()
    }

    private suspend fun url() = container.settings.robotUrl.filterNotNull().first()

    private fun run(block: suspend () -> Unit) {
        viewModelScope.launch {
            try {
                block()
            } catch (e: CancellationException) {
                throw e
            } catch (e: Exception) {
                _ui.update { it.copy(loading = false, loadingFile = false) }
                events.trySend(e.message ?: e.javaClass.simpleName)
            }
        }
    }

    fun refresh() = run {
        val files = client.call(url()) { logs.apiLogsGet() }.sortedByDescending { it.name }
        _ui.update { it.copy(loading = false, files = files) }
    }

    fun open(name: String) = run {
        _ui.update { it.copy(open = name, lines = emptyList(), loadingFile = true) }
        text = client.logText(url(), name)
        _ui.update { it.copy(lines = text.lines().filter { l -> l.isNotBlank() }, loadingFile = false) }
    }

    fun close() = _ui.update { it.copy(open = null, lines = emptyList()) }

    fun delete(name: String) = run {
        client.call(url()) { logs.apiLogsFilenameDelete(name) }
        if (_ui.value.open == name) close()
        refresh()
    }

    fun deleteAll() = run {
        client.call(url()) { logs.apiLogsDelete() }
        close()
        refresh()
    }

    fun save(uri: Uri) = run {
        val t = text
        withContext(Dispatchers.IO) {
            getApplication<Application>().contentResolver.openOutputStream(uri)?.use { it.write(t.toByteArray()) }
        }
    }
}

@Composable
fun LogsScreen(onBack: () -> Unit, vm: LogsViewModel = viewModel()) {
    val ui by vm.ui.collectAsStateWithLifecycle()
    val snackbar = remember { SnackbarHostState() }
    val context = LocalContext.current
    var confirm by remember { mutableStateOf<String?>(null) } // name, or "" = all
    var filter by rememberSaveable { mutableStateOf("") }
    val save = rememberLauncherForActivityResult(ActivityResultContracts.CreateDocument("text/plain")) { it?.let(vm::save) }
    LaunchedEffect(Unit) { vm.messages.collect { snackbar.showSnackbar(context.getString(R.string.failed_with, it)) } }
    val back = { if (ui.open != null) vm.close() else onBack() }
    BackHandler(onBack = back)

    Scaffold(
        topBar = {
            TopAppBar(
                title = { Text(ui.open ?: stringResource(R.string.logs_title), maxLines = 1) },
                navigationIcon = {
                    IconButton(onClick = back) { Icon(Icons.AutoMirrored.Filled.ArrowBack, stringResource(R.string.back)) }
                },
                actions = {
                    val open = ui.open
                    if (open != null) {
                        IconButton(onClick = { save.launch(open.removeSuffix(".hs")) }) {
                            Icon(Icons.Filled.SaveAlt, stringResource(R.string.history_export))
                        }
                        IconButton(onClick = { confirm = open }) { Icon(Icons.Filled.Delete, stringResource(R.string.history_delete)) }
                    } else if (ui.files.isNotEmpty()) {
                        IconButton(onClick = { confirm = "" }) { Icon(Icons.Filled.DeleteSweep, stringResource(R.string.history_delete_all)) }
                    }
                },
            )
        },
        snackbarHost = { SnackbarHost(snackbar) },
    ) { padding ->
        val open = ui.open
        when {
            ui.loading || ui.loadingFile -> Box(Modifier.fillMaxSize().padding(padding), contentAlignment = Alignment.Center) {
                LoadingIndicator(Modifier.size(56.dp))
            }
            open != null -> Column(Modifier.fillMaxSize().padding(padding)) {
                OutlinedTextField(
                    value = filter,
                    onValueChange = { filter = it },
                    label = { Text(stringResource(R.string.logs_filter)) },
                    singleLine = true,
                    modifier = Modifier.fillMaxWidth().padding(horizontal = 16.dp, vertical = 8.dp),
                )
                val shown = if (filter.isBlank()) ui.lines else ui.lines.filter { it.contains(filter, ignoreCase = true) }
                Text(
                    stringResource(R.string.logs_lines, shown.size, ui.lines.size),
                    style = MaterialTheme.typography.labelMedium,
                    color = MaterialTheme.colorScheme.onSurfaceVariant,
                    modifier = Modifier.padding(horizontal = 16.dp),
                )
                // Newest last, like the file
                LazyColumn(
                    Modifier.fillMaxSize(),
                    contentPadding = PaddingValues(16.dp),
                    verticalArrangement = Arrangement.spacedBy(2.dp),
                ) {
                    items(shown) { line ->
                        val entry = remember(line) { LogLine.parse(line) }
                        if (entry == null) {
                            Text(line, fontFamily = FontFamily.Monospace, style = MaterialTheme.typography.bodySmall)
                        } else {
                            Column(Modifier.padding(vertical = 3.dp)) {
                                Text(
                                    "${entry.time}  ${entry.type}",
                                    style = MaterialTheme.typography.labelMedium,
                                    color = MaterialTheme.colorScheme.primary,
                                )
                                Text(entry.body, fontFamily = FontFamily.Monospace, style = MaterialTheme.typography.bodySmall)
                            }
                        }
                    }
                }
            }
            ui.files.isEmpty() -> Box(Modifier.fillMaxSize().padding(padding).padding(24.dp), contentAlignment = Alignment.Center) {
                Text(stringResource(R.string.logs_empty), color = MaterialTheme.colorScheme.onSurfaceVariant)
            }
            else -> LazyColumn(
                Modifier.fillMaxSize().padding(padding),
                contentPadding = PaddingValues(16.dp),
                verticalArrangement = Arrangement.spacedBy(8.dp),
            ) {
                item {
                    SettingsGroup {
                        ui.files.forEachIndexed { i, f ->
                            if (i > 0) RowDivider()
                            ActionRow(
                                f.name,
                                Icons.Filled.Description,
                                desc = "%.1f KB".format(f.propertySize.toDouble() / 1024),
                                chevron = true,
                            ) { vm.open(f.name) }
                        }
                    }
                }
            }
        }
    }

    confirm?.let { target ->
        ConfirmDialog(
            stringResource(if (target.isEmpty()) R.string.logs_delete_all_q else R.string.logs_delete_q),
            stringResource(R.string.history_delete),
            danger = true,
            onConfirm = {
                confirm = null
                if (target.isEmpty()) vm.deleteAll() else vm.delete(target)
            },
            onDismiss = { confirm = null },
        )
    }
}

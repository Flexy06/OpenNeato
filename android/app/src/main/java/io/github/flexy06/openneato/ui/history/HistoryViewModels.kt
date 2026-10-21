package io.github.flexy06.openneato.ui.history

import android.app.Application
import android.net.Uri
import androidx.lifecycle.AndroidViewModel
import androidx.lifecycle.viewModelScope
import io.github.flexy06.openneato.OpenNeatoApp
import io.github.flexy06.openneato.api.models.HistoryFileInfo
import io.github.flexy06.openneato.data.HistoryData
import io.github.flexy06.openneato.data.SessionMap
import kotlinx.coroutines.CancellationException
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.Job
import kotlinx.coroutines.channels.Channel
import kotlinx.coroutines.delay
import kotlinx.coroutines.flow.Flow
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow
import kotlinx.coroutines.flow.filterNotNull
import kotlinx.coroutines.flow.first
import kotlinx.coroutines.flow.receiveAsFlow
import kotlinx.coroutines.flow.update
import kotlinx.coroutines.launch
import kotlinx.coroutines.withContext
import kotlinx.serialization.SerializationException

sealed interface HistoryEvent {
    data class Failed(val message: String) : HistoryEvent
    data class Rejected(val problem: ImportProblem) : HistoryEvent
    data object Imported : HistoryEvent
    data object Exported : HistoryEvent
}

private fun Exception.text() = message ?: javaClass.simpleName

// -- List ------------------------------------------------------------------------------

data class HistoryListUi(
    val loading: Boolean = true,
    val files: List<HistoryFileInfo> = emptyList(),
    val error: String? = null,
    // The list response is unparseable (one session's metadata is damaged)
    val corrupted: Boolean = false,
    val busy: Boolean = false,
) {
    val recording get() = files.any { it.recording }
}

class HistoryListViewModel(app: Application) : AndroidViewModel(app) {
    private val container = app as OpenNeatoApp
    private val client = container.robot
    private val _ui = MutableStateFlow(HistoryListUi())
    val ui: StateFlow<HistoryListUi> = _ui.asStateFlow()
    private val events = Channel<HistoryEvent>(Channel.BUFFERED)
    val messages: Flow<HistoryEvent> = events.receiveAsFlow()

    init {
        // Refresh every 5 s while a session is being recorded, otherwise now and then
        viewModelScope.launch {
            while (true) {
                refresh()
                delay(if (_ui.value.recording) 5_000 else 30_000)
            }
        }
    }

    private suspend fun url() = container.settings.robotUrl.filterNotNull().first()

    suspend fun refresh() {
        try {
            val files = client.call(url()) { history.apiHistoryGet() }
                .sortedByDescending { it.session?.time?.toLong() ?: 0L }
            _ui.update { it.copy(loading = false, files = files, error = null, corrupted = false) }
        } catch (e: CancellationException) {
            throw e
        } catch (e: SerializationException) {
            _ui.update { it.copy(loading = false, corrupted = true) }
        } catch (e: Exception) {
            _ui.update { it.copy(loading = false, error = e.text()) }
        }
    }

    private fun busy(block: suspend () -> Unit) {
        if (_ui.value.busy) return
        _ui.update { it.copy(busy = true) }
        viewModelScope.launch {
            try {
                block()
            } catch (e: CancellationException) {
                throw e
            } catch (e: Exception) {
                events.trySend(HistoryEvent.Failed(e.text()))
            } finally {
                _ui.update { it.copy(busy = false) }
                refresh()
            }
        }
    }

    fun delete(name: String) = busy { client.call(url()) { history.apiHistoryFilenameDelete(name) } }

    fun deleteAll() = busy { client.call(url()) { history.apiHistoryDelete() } }

    // A session file exported earlier: checks like the web UI before uploading
    fun import(uri: Uri) = busy {
        val resolver = getApplication<Application>().contentResolver
        val name = resolver.query(uri, null, null, null, null)?.use { c ->
            val i = c.getColumnIndex(android.provider.OpenableColumns.DISPLAY_NAME)
            if (i >= 0 && c.moveToFirst()) c.getString(i) else null
        } ?: uri.lastPathSegment.orEmpty()
        val bytes = withContext(Dispatchers.IO) { resolver.openInputStream(uri)?.use { it.readBytes() } }
            ?: throw IllegalStateException("cannot read $name")
        val problem = HistoryImport.check(name, bytes, _ui.value.files.map { it.name })
        if (problem != null) {
            events.trySend(HistoryEvent.Rejected(problem))
            return@busy
        }
        client.importHistory(url(), name, bytes)
        events.trySend(HistoryEvent.Imported)
    }
}

enum class ImportProblem { NOT_JSONL, TOO_LARGE, NO_HEADER, NAME_MISMATCH, EXISTS }

object HistoryImport {
    const val MAX_BYTES = 256 * 1024 // Firmware HISTORY_IMPORT_MAX_BYTES

    /** Null when the file looks like a session export (same checks as the web UI) */
    fun check(name: String, bytes: ByteArray, existing: List<String>): ImportProblem? {
        if (!name.endsWith(".jsonl")) return ImportProblem.NOT_JSONL
        if (bytes.size > MAX_BYTES) return ImportProblem.TOO_LARGE
        val first = bytes.decodeToString(0, minOf(bytes.size, 512)).lineSequence().firstOrNull().orEmpty()
        val header = HistoryData.parseHeader(first) ?: return ImportProblem.NO_HEADER
        if (header.time.toString() != name.removeSuffix(".jsonl")) return ImportProblem.NAME_MISMATCH
        if (existing.any { it == name || it == "$name.hs" }) return ImportProblem.EXISTS
        return null
    }
}

// -- Detail ----------------------------------------------------------------------------

data class HistoryDetailUi(
    val loading: Boolean = true,
    val file: HistoryFileInfo? = null,
    val map: SessionMap? = null,
    val empty: Boolean = false,
    val error: String? = null,
) {
    val recording get() = file?.recording == true
}

class HistoryDetailViewModel(app: Application) : AndroidViewModel(app) {
    private val container = app as OpenNeatoApp
    private val client = container.robot
    private val _ui = MutableStateFlow(HistoryDetailUi())
    val ui: StateFlow<HistoryDetailUi> = _ui.asStateFlow()
    private val events = Channel<HistoryEvent>(Channel.BUFFERED)
    val messages: Flow<HistoryEvent> = events.receiveAsFlow()
    private var job: Job? = null
    private var raw: String = ""

    private suspend fun url() = container.settings.robotUrl.filterNotNull().first()

    fun open(name: String) {
        if (job != null) return
        job = viewModelScope.launch {
            while (true) {
                try {
                    val u = url()
                    val file = client.call(u) { history.apiHistoryGet() }.firstOrNull { it.name == name }
                    raw = client.historySession(u, name)
                    val map = withContext(Dispatchers.Default) { HistoryData.parse(raw).firstOrNull() }
                    _ui.update {
                        it.copy(loading = false, file = file, map = map, empty = map == null && file?.recording != true, error = null)
                    }
                } catch (e: CancellationException) {
                    throw e
                } catch (e: Exception) {
                    _ui.update { it.copy(loading = false, error = e.text()) }
                }
                // A session still being recorded grows: follow it
                if (!_ui.value.recording) break
                delay(5_000)
            }
        }
    }

    // Save the session file (as the robot stores it, decompressed) to a document
    fun export(uri: Uri) {
        val text = raw
        viewModelScope.launch {
            try {
                withContext(Dispatchers.IO) {
                    getApplication<Application>().contentResolver.openOutputStream(uri)?.use {
                        it.write(text.toByteArray())
                    } ?: throw IllegalStateException("cannot write")
                }
                events.trySend(HistoryEvent.Exported)
            } catch (e: CancellationException) {
                throw e
            } catch (e: Exception) {
                events.trySend(HistoryEvent.Failed(e.text()))
            }
        }
    }
}

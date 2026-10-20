package io.github.flexy06.openneato.ui.schedule

import android.app.Application
import androidx.lifecycle.AndroidViewModel
import androidx.lifecycle.viewModelScope
import io.github.flexy06.openneato.OpenNeatoApp
import kotlinx.coroutines.CancellationException
import kotlinx.coroutines.channels.Channel
import kotlinx.coroutines.flow.Flow
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow
import kotlinx.coroutines.flow.first
import kotlinx.coroutines.flow.filterNotNull
import kotlinx.coroutines.flow.receiveAsFlow
import kotlinx.coroutines.flow.update
import kotlinx.coroutines.launch
import kotlinx.serialization.json.contentOrNull
import kotlinx.serialization.json.jsonPrimitive

data class ScheduleUi(
    val loading: Boolean = true,
    val loadError: String? = null,
    val server: Schedule = Schedule(),
    val edited: Schedule = Schedule(),
    val tz: String = "",
    val robotTime: String? = null,
    val saving: Boolean = false,
) {
    val dirty get() = schedulePatch(edited, server).isNotEmpty()
}

sealed interface ScheduleEvent {
    data object Saved : ScheduleEvent
    data class Failed(val message: String) : ScheduleEvent
}

class ScheduleViewModel(app: Application) : AndroidViewModel(app) {
    private val container = app as OpenNeatoApp
    private val client = container.robot
    private val _ui = MutableStateFlow(ScheduleUi())
    val ui: StateFlow<ScheduleUi> = _ui.asStateFlow()
    private val events = Channel<ScheduleEvent>(Channel.BUFFERED)
    val messages: Flow<ScheduleEvent> = events.receiveAsFlow()

    init {
        load()
    }

    private suspend fun url() = container.settings.robotUrl.filterNotNull().first()

    fun load() {
        _ui.update { it.copy(loading = true, loadError = null) }
        viewModelScope.launch {
            try {
                val u = url()
                val settings = client.settingsJson(u)
                val s = readSchedule(settings)
                val time = runCatching { client.call(u) { system.apiSystemGet() }.localTime }.getOrNull()
                _ui.update {
                    it.copy(
                        loading = false,
                        server = s,
                        edited = s,
                        tz = settings["tz"]?.jsonPrimitive?.contentOrNull.orEmpty(),
                        robotTime = time,
                    )
                }
            } catch (e: CancellationException) {
                throw e
            } catch (e: Exception) {
                _ui.update { it.copy(loading = false, loadError = e.message ?: e.javaClass.simpleName) }
            }
        }
    }

    fun edit(f: (Schedule) -> Schedule) = _ui.update { it.copy(edited = f(it.edited)) }

    fun save() {
        val s = _ui.value
        val patch = schedulePatch(s.edited, s.server)
        if (patch.isEmpty() || s.saving) return
        _ui.update { it.copy(saving = true) }
        viewModelScope.launch {
            try {
                // The answer is the stored settings: show what the robot really kept
                val saved = readSchedule(client.patchSettings(url(), patch))
                _ui.update { it.copy(saving = false, server = saved, edited = saved) }
                events.trySend(ScheduleEvent.Saved)
            } catch (e: CancellationException) {
                throw e
            } catch (e: Exception) {
                _ui.update { it.copy(saving = false) }
                events.trySend(ScheduleEvent.Failed(e.message ?: e.javaClass.simpleName))
            }
        }
    }
}

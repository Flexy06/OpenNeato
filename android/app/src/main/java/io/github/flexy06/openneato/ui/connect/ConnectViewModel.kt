package io.github.flexy06.openneato.ui.connect

import android.app.Application
import androidx.lifecycle.AndroidViewModel
import androidx.lifecycle.viewModelScope
import io.github.flexy06.openneato.OpenNeatoApp
import io.github.flexy06.openneato.data.Discovery
import io.github.flexy06.openneato.data.FoundRobot
import io.github.flexy06.openneato.data.RobotClient
import kotlinx.coroutines.Job
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow
import kotlinx.coroutines.flow.catch
import kotlinx.coroutines.flow.onCompletion
import kotlinx.coroutines.flow.update
import kotlinx.coroutines.launch

data class ConnectUiState(
    val found: List<FoundRobot> = emptyList(),
    val searching: Boolean = true,
    val discoveryError: String? = null,
    val checking: Boolean = false,
    val notFoundAt: String? = null,
)

class ConnectViewModel(app: Application) : AndroidViewModel(app) {
    private val container = app as OpenNeatoApp
    private val discovery = Discovery(app, container.robot)
    private val _state = MutableStateFlow(ConnectUiState())
    val state: StateFlow<ConnectUiState> = _state.asStateFlow()
    private var discoveryJob: Job? = null

    init {
        startDiscovery()
    }

    fun startDiscovery() {
        discoveryJob?.cancel()
        _state.update { it.copy(searching = true, discoveryError = null) }
        discoveryJob = viewModelScope.launch {
            discovery.robots()
                .catch { e -> _state.update { it.copy(discoveryError = e.message) } }
                .onCompletion { _state.update { it.copy(searching = false) } }
                .collect { robot -> _state.update { s -> s.copy(found = (s.found + robot).distinctBy { it.url }) } }
        }
    }

    fun choose(url: String, onDone: () -> Unit) {
        viewModelScope.launch {
            container.settings.setRobotUrl(url)
            onDone()
        }
    }

    // Manual address: only accepted if an OpenNeato bridge answers there
    fun connectManual(input: String, onDone: () -> Unit) {
        if (input.isBlank()) return
        val url = RobotClient.normalizeUrl(input)
        _state.update { it.copy(checking = true, notFoundAt = null) }
        viewModelScope.launch {
            val fw = container.robot.probe(url)
            _state.update { it.copy(checking = false, notFoundAt = if (fw == null) url else null) }
            if (fw != null) choose(url, onDone)
        }
    }
}

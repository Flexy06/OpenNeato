package io.github.flexy06.openneato.ui.manual

import android.app.Application
import androidx.lifecycle.AndroidViewModel
import androidx.lifecycle.viewModelScope
import io.github.flexy06.openneato.OpenNeatoApp
import io.github.flexy06.openneato.api.models.ChargerData
import io.github.flexy06.openneato.api.models.LidarScan
import io.github.flexy06.openneato.api.models.ManualStatus
import io.github.flexy06.openneato.api.models.StateData
import io.github.flexy06.openneato.data.Poll
import io.github.flexy06.openneato.data.poll
import io.github.flexy06.openneato.ui.dashboard.RobotFlags
import kotlinx.coroutines.CancellationException
import kotlinx.coroutines.ExperimentalCoroutinesApi
import kotlinx.coroutines.Job
import kotlinx.coroutines.channels.Channel
import kotlinx.coroutines.delay
import kotlinx.coroutines.flow.Flow
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.SharingStarted
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.combine
import kotlinx.coroutines.flow.distinctUntilChanged
import kotlinx.coroutines.flow.filterNotNull
import kotlinx.coroutines.flow.flatMapLatest
import kotlinx.coroutines.flow.flowOf
import kotlinx.coroutines.flow.map
import kotlinx.coroutines.flow.onEach
import kotlinx.coroutines.flow.receiveAsFlow
import kotlinx.coroutines.flow.shareIn
import kotlinx.coroutines.flow.stateIn
import kotlinx.coroutines.flow.update
import kotlinx.coroutines.launch

data class Motors(val brush: Boolean = false, val vacuum: Boolean = false, val sideBrush: Boolean = false) {
    val all get() = brush && vacuum && sideBrush
}

private data class Local(
    val moving: Boolean = false,
    val motors: Motors = Motors(),
    val motorPending: Boolean = false,
    val starting: Boolean = false,
    val stopping: Boolean = false,
)

data class ManualUi(
    val state: Poll<StateData> = Poll(),
    val status: Poll<ManualStatus> = Poll(),
    val lidar: Poll<LidarScan> = Poll(),
    val charger: Poll<ChargerData> = Poll(),
    val moving: Boolean = false,
    val motors: Motors = Motors(),
    val motorPending: Boolean = false,
    val starting: Boolean = false,
    val stopping: Boolean = false,
) {
    val flags get() = RobotFlags.of(state.data?.uiState)
    val active get() = flags.manual
}

@OptIn(ExperimentalCoroutinesApi::class)
class ManualViewModel(app: Application) : AndroidViewModel(app) {
    private val container = app as OpenNeatoApp
    private val client = container.robot
    private val url: StateFlow<String?> = container.settings.robotUrl.stateIn(viewModelScope, SharingStarted.Eagerly, null)
    private val local = MutableStateFlow(Local())
    private val messages = Channel<String>(Channel.BUFFERED)
    val errors: Flow<String> = messages.receiveAsFlow()

    private val state = container.settings.robotUrl.filterNotNull()
        .flatMapLatest { u -> poll(1000) { client.call(u) { sensors.apiStateGet() } } }
        .shareIn(viewModelScope, SharingStarted.WhileSubscribed(5000), replay = 1)
    private val active = state.map { RobotFlags.of(it.data?.uiState).manual }.distinctUntilChanged()

    // Safety + motor state twice a second and LIDAR once a second, only while driving
    private fun <T> whileActive(ms: Long, fetch: suspend (String) -> T): Flow<Poll<T>> =
        combine(container.settings.robotUrl.filterNotNull(), active) { u, a -> u to a }
            .flatMapLatest { (u, a) -> if (a) poll(ms) { fetch(u) } else flowOf(Poll()) }

    private val status = whileActive(500) { u -> client.call(u) { manual.apiManualStatusGet() } }
        .onEach { p ->
            val s = p.data ?: return@onEach
            if (!local.value.motorPending) local.update { it.copy(motors = Motors(s.brush, s.vacuum, s.sideBrush)) }
        }
    private val lidar = whileActive(1000) { u -> client.call(u) { sensors.apiLidarGet() } }
    private val charger = container.settings.robotUrl.filterNotNull()
        .flatMapLatest { u -> poll(5000) { client.call(u) { sensors.apiChargerGet() } } }

    val ui: StateFlow<ManualUi> = combine(state, status, lidar, charger, local) { st, ms, li, ch, l ->
        // Starting/stopping ends once the robot confirms it
        val manualNow = RobotFlags.of(st.data?.uiState).manual
        if (l.starting && manualNow) local.update { it.copy(starting = false) }
        ManualUi(st, ms, li, ch, l.moving, l.motors, l.motorPending, l.starting && !manualNow, l.stopping)
    }.stateIn(viewModelScope, SharingStarted.WhileSubscribed(5000), ManualUi())

    private suspend fun send(block: suspend (String) -> Unit): Boolean {
        val u = url.value ?: return false
        return try {
            block(u)
            true
        } catch (e: CancellationException) {
            throw e
        } catch (e: Exception) {
            messages.trySend(e.message ?: e.javaClass.simpleName)
            false
        }
    }

    fun start() {
        local.update { it.copy(starting = true) }
        viewModelScope.launch {
            if (!send { u -> client.call(u) { manual.apiManualPost(true) } }) local.update { it.copy(starting = false) }
            delay(10000)
            local.update { it.copy(starting = false) }
        }
    }

    fun stop() {
        if (local.value.stopping) return
        release()
        local.update { it.copy(stopping = true) }
        viewModelScope.launch {
            if (!send { u -> client.call(u) { manual.apiManualPost(false) } }) local.update { it.copy(stopping = false) }
        }
    }

    // -- Driving: at most one move every 200 ms, only when the direction really changed --
    private var latest: WheelCommand? = null
    private var lastSent: WheelCommand? = null
    private var moveJob: Job? = null

    fun move(x: Float, y: Float, magnitude: Float) {
        if (!ui.value.active || local.value.stopping) return
        val cmd = joystickToWheels(x, y, magnitude)
        latest = cmd
        local.update { it.copy(moving = !cmd.isStop) }
        if (moveJob?.isActive == true) return
        moveJob = viewModelScope.launch {
            delay(200)
            val m = latest ?: return@launch
            if (m.isStop || !worthSending(m, lastSent)) return@launch
            lastSent = m
            send { u -> client.call(u) { manual.apiManualMovePost(m.left, m.right, m.speed) } }
        }
    }

    fun release() {
        moveJob?.cancel()
        latest = null
        lastSent = null
        local.update { it.copy(moving = false) }
        if (!ui.value.active) return
        viewModelScope.launch { send { u -> client.call(u) { manual.apiManualMovePost(0, 0, 0) } } }
    }

    fun setMotors(next: Motors) {
        if (local.value.motorPending) return
        local.update { it.copy(motorPending = true) }
        viewModelScope.launch {
            val ok = send { u -> client.call(u) { manual.apiManualMotorsPost(next.brush, next.vacuum, next.sideBrush) } }
            local.update { it.copy(motorPending = false, motors = if (ok) next else it.motors) }
        }
    }
}

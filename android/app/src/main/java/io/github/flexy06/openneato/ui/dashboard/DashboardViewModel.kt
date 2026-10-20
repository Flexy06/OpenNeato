package io.github.flexy06.openneato.ui.dashboard

import android.app.Application
import androidx.lifecycle.AndroidViewModel
import androidx.lifecycle.viewModelScope
import io.github.flexy06.openneato.OpenNeatoApp
import io.github.flexy06.openneato.api.apis.ActionsApi.ActionApiCleanPost
import io.github.flexy06.openneato.api.apis.GuidedApi.ActionApiGuidedPost
import io.github.flexy06.openneato.api.models.ChargerData
import io.github.flexy06.openneato.api.models.DiagData
import io.github.flexy06.openneato.api.models.ErrorData
import io.github.flexy06.openneato.api.models.FirmwareVersion
import io.github.flexy06.openneato.api.models.ScheduleNextData
import io.github.flexy06.openneato.api.models.StateData
import io.github.flexy06.openneato.api.models.SystemData
import io.github.flexy06.openneato.data.Poll
import io.github.flexy06.openneato.data.RobotClient
import io.github.flexy06.openneato.data.poll
import kotlinx.coroutines.CancellationException
import kotlinx.coroutines.ExperimentalCoroutinesApi
import kotlinx.coroutines.channels.Channel
import kotlinx.coroutines.delay
import kotlinx.coroutines.flow.Flow
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.SharingStarted
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.combine
import kotlinx.coroutines.flow.filterNotNull
import kotlinx.coroutines.flow.first
import kotlinx.coroutines.flow.flatMapLatest
import kotlinx.coroutines.flow.flow
import kotlinx.coroutines.flow.onEach
import kotlinx.coroutines.flow.receiveAsFlow
import kotlinx.coroutines.flow.stateIn
import kotlinx.coroutines.launch

data class DashboardData(
    val firmware: Poll<FirmwareVersion> = Poll(),
    val state: Poll<StateData> = Poll(),
    val error: Poll<ErrorData> = Poll(),
    val charger: Poll<ChargerData> = Poll(),
    val schedule: Poll<ScheduleNextData> = Poll(),
    val system: Poll<SystemData> = Poll(),
    val diag: Poll<DiagData?> = Poll(),
)

data class DashboardUi(
    val data: DashboardData = DashboardData(),
    // An action was sent and the robot has not changed state yet (or 10 s passed)
    val pending: Boolean = false,
    val skipPending: Boolean = false,
) {
    val firmwareReady get() = data.firmware.data?.let { !it.identifying && it.supported } ?: false
    val identifying get() = data.firmware.data?.identifying ?: true
}

@OptIn(ExperimentalCoroutinesApi::class)
class DashboardViewModel(app: Application) : AndroidViewModel(app) {
    private val container = app as OpenNeatoApp
    private val client: RobotClient = container.robot
    val robotUrl: StateFlow<String?> = container.settings.robotUrl.stateIn(viewModelScope, SharingStarted.Eagerly, null)

    private val pending = MutableStateFlow(false)
    private val skipPending = MutableStateFlow(false)
    private var pendingFrom: String? = null
    private val messages = Channel<String>(Channel.BUFFERED)
    val actionErrors: Flow<String> = messages.receiveAsFlow()

    // Polling intervals as in the web UI; everything stops 5 s after the screen leaves the foreground
    private val data: Flow<DashboardData> = container.settings.robotUrl.filterNotNull().flatMapLatest { url ->
        fun <T> p(ms: Long, fetch: RobotClient.Apis.() -> T) = poll(ms) { client.call(url, fetch) }
        // Serial diagnostics only matter (and are only polled) while the robot is not identified yet
        var identifying = true
        val firmware = firmwarePoll(url).onEach { identifying = it.data?.identifying != false }
        val diag = poll(2000) { if (identifying) client.call(url) { system.apiDiagGet() } else null }
        val group1 = combine(
            firmware,
            p(2000) { sensors.apiStateGet() },
            p(2000) { sensors.apiErrorGet() },
            p(5000) { sensors.apiChargerGet() },
        ) { fw, st, err, ch -> DashboardData(firmware = fw, state = st, error = err, charger = ch) }
        combine(
            group1,
            p(30000) { actions.apiScheduleNextGet() },
            p(10000) { system.apiSystemGet() },
            diag,
        ) { d, sched, sys, dg -> d.copy(schedule = sched, system = sys, diag = dg) }
    }

    val ui: StateFlow<DashboardUi> = combine(data, pending, skipPending) { d, p, s ->
        // The robot reacted: its state changed since the action was sent
        if (p && d.state.data?.uiState != null && d.state.data.uiState != pendingFrom) pending.value = false
        DashboardUi(d, pending.value, s)
    }.stateIn(viewModelScope, SharingStarted.WhileSubscribed(5000), DashboardUi())

    // Fast while the robot is still being identified, then once a minute
    private fun firmwarePoll(url: String): Flow<Poll<FirmwareVersion>> = flow {
        var last: FirmwareVersion? = null
        while (true) {
            val err = try {
                last = client.call(url) { firmware.apiFirmwareVersionGet() }
                null
            } catch (e: CancellationException) {
                throw e
            } catch (e: Exception) {
                e.message ?: e.javaClass.simpleName
            }
            emit(Poll(last, err))
            delay(if (last == null || last.identifying) 2000 else 60000)
        }
    }

    fun run(action: Action) {
        if (action == Action.MANUAL) return
        val url = robotUrl.value ?: return
        pendingFrom = ui.value.data.state.data?.uiState
        pending.value = true
        viewModelScope.launch {
            try {
                client.call(url) {
                    when (action) {
                        Action.HOUSE, Action.RESUME -> actions.apiCleanPost(ActionApiCleanPost.HOUSE)
                        Action.SPOT -> actions.apiCleanPost(ActionApiCleanPost.SPOT)
                        Action.PAUSE -> actions.apiCleanPost(ActionApiCleanPost.PAUSE)
                        Action.DOCK -> actions.apiCleanPost(ActionApiCleanPost.DOCK)
                        Action.STOP -> actions.apiCleanPost(ActionApiCleanPost.STOP)
                        Action.GUIDED_HOME -> guided.apiGuidedPost(ActionApiGuidedPost.HOME)
                        Action.GUIDED_STOP -> guided.apiGuidedPost(ActionApiGuidedPost.STOP)
                        Action.MANUAL -> Unit // Navigation only, handled by the screen
                    }
                }
                delay(10000)
            } catch (e: CancellationException) {
                throw e
            } catch (e: Exception) {
                messages.trySend(e.message ?: e.javaClass.simpleName)
            }
            pending.value = false
        }
    }

    fun toggleSkipNextClean() {
        val url = robotUrl.value ?: return
        val skipped = ui.value.data.schedule.data?.skipNextClean ?: return
        skipPending.value = true
        viewModelScope.launch {
            try {
                client.call(url) { if (skipped) actions.apiScheduleNextDelete() else actions.apiScheduleNextPost() }
            } catch (e: CancellationException) {
                throw e
            } catch (e: Exception) {
                messages.trySend(e.message ?: e.javaClass.simpleName)
            }
            skipPending.value = false
        }
    }

    fun reidentify() {
        val url = robotUrl.value ?: return
        viewModelScope.launch {
            runCatching { client.call(url) { system.apiDiagIdentifyPost() } }
                .onFailure { e -> if (e !is CancellationException) messages.trySend(e.message ?: "") }
        }
    }

    fun forgetRobot(onDone: () -> Unit) {
        viewModelScope.launch {
            container.settings.setRobotUrl(null)
            container.settings.robotUrl.first { it == null }
            onDone()
        }
    }

}

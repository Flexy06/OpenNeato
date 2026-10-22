package io.github.flexy06.openneato.ui.settings

import android.app.Application
import android.net.Uri
import android.provider.OpenableColumns
import androidx.lifecycle.AndroidViewModel
import androidx.lifecycle.viewModelScope
import io.github.flexy06.openneato.OpenNeatoApp
import io.github.flexy06.openneato.api.apis.ActionsApi
import io.github.flexy06.openneato.api.models.BatteryAnalogData
import io.github.flexy06.openneato.api.models.BatteryWarrantyData
import io.github.flexy06.openneato.api.models.ChargerData
import io.github.flexy06.openneato.api.models.FirmwareVersion
import io.github.flexy06.openneato.api.models.SystemData
import io.github.flexy06.openneato.api.models.UserSettingsData
import io.github.flexy06.openneato.api.models.VersionData
import io.github.flexy06.openneato.api.models.WiFiNetwork
import io.github.flexy06.openneato.api.models.WiFiStatus
import io.github.flexy06.openneato.data.AppTheme
import kotlinx.coroutines.CancellationException
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.channels.Channel
import kotlinx.coroutines.delay
import kotlinx.coroutines.flow.Flow
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.SharingStarted
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow
import kotlinx.coroutines.flow.filterNotNull
import kotlinx.coroutines.flow.first
import kotlinx.coroutines.flow.receiveAsFlow
import kotlinx.coroutines.flow.stateIn
import kotlinx.coroutines.flow.update
import kotlinx.coroutines.launch
import kotlinx.coroutines.withContext
import java.security.MessageDigest

// Settings pages; the hub lists them, each one polls only what it shows
enum class SettingsPage { HUB, APP, DEVICE, WIFI, NOTIFICATIONS, CLEANING, ROBOT, FIRMWARE, DIAGNOSTICS, ABOUT, DANGER }

enum class Busy { NONE, RESTART, FORMAT, RESET, ROBOT_RESTART, CLEAR_ERRORS, NEW_BATTERY, WIFI }

enum class FwStatus { IDLE, UPLOADING, DONE }

enum class ChecksumResult { MATCH, MISMATCH, NOT_FOUND }

data class FirmwareUpload(
    val name: String = "",
    val size: Int = 0,
    val chip: String? = null,
    val chipMismatch: Boolean = false,
    val checksum: ChecksumResult? = null,
    val status: FwStatus = FwStatus.IDLE,
    val progress: Float = 0f,
) {
    val selected get() = size > 0
    val canUpload get() = selected && chip != null && !chipMismatch && checksum != ChecksumResult.MISMATCH && status == FwStatus.IDLE
}

data class Battery(
    val charger: ChargerData? = null,
    val analog: BatteryAnalogData? = null,
    val warranty: BatteryWarrantyData? = null,
    val version: VersionData? = null,
)

data class SettingsUi(
    val loading: Boolean = true,
    val loadError: String? = null,
    val server: BridgeSettings = BridgeSettings(),
    val edited: BridgeSettings = BridgeSettings(),
    val system: SystemData? = null,
    val firmware: FirmwareVersion? = null,
    val user: UserSettingsData? = null,
    val userPending: Boolean = false,
    val wifi: WiFiStatus? = null,
    val networks: List<WiFiNetwork>? = null,
    val scanning: Boolean = false,
    val battery: Battery = Battery(),
    val saving: Boolean = false,
    val busy: Busy = Busy.NONE,
    // Waiting for the bridge (or the robot) to come back after a restart
    val rebooting: Boolean = false,
    val robotRestarting: Boolean = false,
    val testingNotification: Boolean = false,
    val upload: FirmwareUpload = FirmwareUpload(),
) {
    val dirty get() = !loading && settingsPatch(edited, server).isNotEmpty()
    val needsReboot get() = needsReboot(edited, server)
    val supported get() = firmware?.supported != false
}

sealed interface SettingsEvent {
    data object Saved : SettingsEvent
    data object Back : SettingsEvent // The bridge restarted and answers again
    data object RebootTimeout : SettingsEvent
    data object RobotBack : SettingsEvent
    data object RobotTimeout : SettingsEvent
    data object NotificationSent : SettingsEvent
    data object ErrorsCleared : SettingsEvent
    data object NewBatterySet : SettingsEvent
    data object WifiConnecting : SettingsEvent
    data object Reset : SettingsEvent // Factory reset: the bridge is in setup mode, pick it again
    data object Shutdown : SettingsEvent
    data class Failed(val message: String) : SettingsEvent
}

class SettingsViewModel(app: Application) : AndroidViewModel(app) {
    private val container = app as OpenNeatoApp
    private val client = container.robot
    private val _ui = MutableStateFlow(SettingsUi())
    val ui: StateFlow<SettingsUi> = _ui.asStateFlow()
    private val events = Channel<SettingsEvent>(Channel.BUFFERED)
    val messages: Flow<SettingsEvent> = events.receiveAsFlow()
    val theme: StateFlow<AppTheme> = container.settings.theme.stateIn(viewModelScope, SharingStarted.Eagerly, AppTheme.SYSTEM)
    val robotUrl: StateFlow<String?> = container.settings.robotUrl.stateIn(viewModelScope, SharingStarted.Eagerly, null)

    private val page = MutableStateFlow(SettingsPage.HUB)
    private var firmwareBin: ByteArray? = null
    private var checksums: Map<String, String>? = null

    init {
        load()
        // Light polling for what the open page shows
        viewModelScope.launch {
            var tick = 0
            while (true) {
                delay(1_000)
                tick++
                val p = page.value
                if (_ui.value.rebooting || _ui.value.loading) continue
                when {
                    p == SettingsPage.WIFI && tick % 5 == 0 -> quiet { _ui.update { it.copy(wifi = client.call(url()) { wifi.apiWifiStatusGet() }) } }
                    p == SettingsPage.ROBOT && tick % 30 == 0 -> quiet { _ui.update { it.copy(user = client.call(url()) { sensors.apiUserSettingsGet() }) } }
                    p == SettingsPage.DIAGNOSTICS && tick % 30 == 0 -> loadBattery()
                    tick % 10 == 0 -> quiet { _ui.update { it.copy(system = client.call(url()) { system.apiSystemGet() }) } }
                }
            }
        }
    }

    private suspend fun url() = container.settings.robotUrl.filterNotNull().first()

    private suspend fun quiet(block: suspend () -> Unit) {
        try {
            block()
        } catch (e: CancellationException) {
            throw e
        } catch (_: Exception) {
            // A missed poll changes nothing on screen
        }
    }

    private fun fail(e: Exception) = events.trySend(SettingsEvent.Failed(e.message ?: e.javaClass.simpleName))

    fun load() {
        _ui.update { it.copy(loading = true, loadError = null) }
        viewModelScope.launch {
            try {
                val u = url()
                val settings = BridgeSettings.from(client.settingsJson(u))
                val fw = client.call(u) { firmware.apiFirmwareVersionGet() }
                val sys = runCatching { client.call(u) { system.apiSystemGet() } }.getOrNull()
                _ui.update { it.copy(loading = false, server = settings, edited = settings, firmware = fw, system = sys) }
            } catch (e: CancellationException) {
                throw e
            } catch (e: Exception) {
                _ui.update { it.copy(loading = false, loadError = e.message ?: e.javaClass.simpleName) }
            }
        }
    }

    fun open(p: SettingsPage) {
        page.value = p
        viewModelScope.launch {
            when (p) {
                SettingsPage.ROBOT -> quiet { _ui.update { it.copy(user = client.call(url()) { sensors.apiUserSettingsGet() }) } }
                SettingsPage.WIFI -> quiet { _ui.update { it.copy(wifi = client.call(url()) { wifi.apiWifiStatusGet() }) } }
                SettingsPage.DIAGNOSTICS -> loadBattery()
                else -> {}
            }
        }
    }

    fun edit(f: (BridgeSettings) -> BridgeSettings) = _ui.update { it.copy(edited = f(it.edited)) }

    fun discard() = _ui.update { it.copy(edited = it.server) }

    fun setTheme(t: AppTheme) {
        viewModelScope.launch { container.settings.setTheme(t) }
    }

    fun forgetRobot(done: () -> Unit) {
        viewModelScope.launch {
            container.settings.setRobotUrl(null)
            done()
        }
    }

    // -- Bridge settings ------------------------------------------------------------------

    fun save() {
        val s = _ui.value
        val patch = settingsPatch(s.edited, s.server)
        if (patch.isEmpty() || s.saving || s.edited.problem() != null) return
        val reboot = s.needsReboot
        _ui.update { it.copy(saving = true) }
        viewModelScope.launch {
            val uptime = s.system?.uptime?.toLong() ?: Long.MAX_VALUE
            try {
                val saved = BridgeSettings.from(client.patchSettings(url(), patch))
                _ui.update { it.copy(saving = false, server = saved, edited = saved) }
                if (reboot) waitForReboot(uptime) else events.trySend(SettingsEvent.Saved)
            } catch (e: CancellationException) {
                throw e
            } catch (e: Exception) {
                _ui.update { it.copy(saving = false) }
                // The bridge may restart before it answers (new pins / hostname)
                if (reboot) waitForReboot(uptime) else fail(e)
            }
        }
    }

    // The bridge is back when it answers with a smaller uptime than before
    private suspend fun waitForReboot(uptimeBefore: Long) {
        _ui.update { it.copy(rebooting = true) }
        val until = System.currentTimeMillis() + 120_000
        delay(3_000)
        while (System.currentTimeMillis() < until) {
            try {
                val sys = client.call(url()) { system.apiSystemGet() }
                if (sys.uptime.toLong() < uptimeBefore) {
                    _ui.update { it.copy(rebooting = false, system = sys) }
                    events.trySend(SettingsEvent.Back)
                    load()
                    return
                }
            } catch (e: CancellationException) {
                throw e
            } catch (_: Exception) {
                // Still restarting
            }
            delay(2_000)
        }
        _ui.update { it.copy(rebooting = false) }
        events.trySend(SettingsEvent.RebootTimeout)
    }

    fun testNotification() {
        val topic = _ui.value.edited.ntfyTopic.trim()
        if (topic.isEmpty() || _ui.value.testingNotification) return
        _ui.update { it.copy(testingNotification = true) }
        viewModelScope.launch {
            try {
                client.call(url()) { settings.apiNotificationsTestPost(topic) }
                events.trySend(SettingsEvent.NotificationSent)
            } catch (e: CancellationException) {
                throw e
            } catch (e: Exception) {
                fail(e)
            } finally {
                _ui.update { it.copy(testingNotification = false) }
            }
        }
    }

    // -- Robot settings (applied at once, like the web UI) ----------------------------------

    fun setRobotSetting(key: String, update: (UserSettingsData) -> UserSettingsData, on: Boolean) {
        val before = _ui.value.user ?: return
        if (_ui.value.userPending) return
        _ui.update { it.copy(user = update(before), userPending = true) }
        viewModelScope.launch {
            try {
                client.call(url()) { actions.apiUserSettingsPost(key, if (on) "ON" else "OFF") }
            } catch (e: CancellationException) {
                throw e
            } catch (e: Exception) {
                _ui.update { it.copy(user = before) }
                fail(e)
            } finally {
                _ui.update { it.copy(userPending = false) }
            }
        }
    }

    // -- Actions ------------------------------------------------------------------------------

    private fun action(kind: Busy, block: suspend () -> Unit) {
        if (_ui.value.busy != Busy.NONE) return
        _ui.update { it.copy(busy = kind) }
        viewModelScope.launch {
            try {
                block()
            } catch (e: CancellationException) {
                throw e
            } catch (e: Exception) {
                fail(e)
            } finally {
                _ui.update { it.copy(busy = Busy.NONE) }
            }
        }
    }

    // Restart / format: the bridge may drop the connection before answering
    private fun restartingAction(kind: Busy, call: suspend (String) -> Unit) = action(kind) {
        val uptime = _ui.value.system?.uptime?.toLong() ?: Long.MAX_VALUE
        runCatching { call(url()) }
        waitForReboot(uptime)
    }

    fun restartBridge() = restartingAction(Busy.RESTART) { u -> client.call(u) { system.apiSystemRestartPost() } }

    fun formatStorage() = restartingAction(Busy.FORMAT) { u -> client.call(u) { system.apiSystemFormatFsPost() } }

    // Settings and WiFi are gone: the bridge opens its setup hotspot
    fun factoryReset() = action(Busy.RESET) {
        runCatching { client.call(url()) { system.apiSystemResetPost() } }
        container.settings.setRobotUrl(null)
        events.trySend(SettingsEvent.Reset)
    }

    fun clearErrors() = action(Busy.CLEAR_ERRORS) {
        client.call(url()) { actions.apiClearErrorsPost() }
        events.trySend(SettingsEvent.ErrorsCleared)
    }

    fun newBattery() = action(Busy.NEW_BATTERY) {
        client.call(url()) { actions.apiBatteryNewPost() }
        events.trySend(SettingsEvent.NewBatterySet)
        loadBattery()
    }

    // The robot reboots (a few seconds); back once its state answers again
    fun restartRobot() = action(Busy.ROBOT_RESTART) {
        client.call(url()) { actions.apiPowerPost(ActionsApi.ActionApiPowerPost.RESTART) }
        _ui.update { it.copy(robotRestarting = true) }
        val until = System.currentTimeMillis() + 30_000
        delay(3_000)
        while (System.currentTimeMillis() < until) {
            val back = runCatching { client.call(url()) { sensors.apiStateGet() } }.isSuccess
            if (back) {
                _ui.update { it.copy(robotRestarting = false) }
                events.trySend(SettingsEvent.RobotBack)
                return@action
            }
            delay(2_000)
        }
        _ui.update { it.copy(robotRestarting = false) }
        events.trySend(SettingsEvent.RobotTimeout)
    }

    // The bridge loses power with the robot: no answer to wait for
    fun shutdownRobot() {
        viewModelScope.launch {
            runCatching { client.call(url()) { actions.apiPowerPost(ActionsApi.ActionApiPowerPost.SHUTDOWN) } }
            events.trySend(SettingsEvent.Shutdown)
        }
    }

    private suspend fun loadBattery() {
        quiet {
            val u = url()
            val charger = runCatching { client.call(u) { sensors.apiChargerGet() } }.getOrNull()
            val analog = runCatching { client.call(u) { sensors.apiAnalogGet() } }.getOrNull()
            val warranty = runCatching { client.call(u) { sensors.apiWarrantyGet() } }.getOrNull()
            val version = runCatching { client.call(u) { sensors.apiVersionGet() } }.getOrNull()
            _ui.update {
                it.copy(
                    battery = Battery(
                        charger ?: it.battery.charger,
                        analog ?: it.battery.analog,
                        warranty ?: it.battery.warranty,
                        version ?: it.battery.version,
                    ),
                )
            }
        }
    }

    // -- WiFi ----------------------------------------------------------------------------------

    fun scanWifi() {
        if (_ui.value.scanning) return
        _ui.update { it.copy(scanning = true) }
        viewModelScope.launch {
            try {
                val found = client.call(url()) { wifi.apiWifiScanGet() }.networks
                    .filter { it.ssid.isNotEmpty() }
                    .groupBy { it.ssid }
                    .map { (_, same) -> same.maxBy { it.rssi } }
                    .sortedByDescending { it.rssi }
                _ui.update { it.copy(networks = found) }
            } catch (e: CancellationException) {
                throw e
            } catch (e: Exception) {
                fail(e)
            } finally {
                _ui.update { it.copy(scanning = false) }
            }
        }
    }

    // The bridge switches networks: it may answer from a new address afterwards
    fun connectWifi(ssid: String, password: String) = action(Busy.WIFI) {
        client.call(url()) { wifi.apiWifiConnectPost(ssid, password) }
        events.trySend(SettingsEvent.WifiConnecting)
    }

    fun forgetWifi() = action(Busy.WIFI) {
        client.call(url()) { wifi.apiWifiDisconnectPost() }
        val status = runCatching { client.call(url()) { wifi.apiWifiStatusGet() } }.getOrNull()
        _ui.update { it.copy(wifi = status ?: it.wifi) }
    }

    // -- Firmware update ----------------------------------------------------------------------

    fun pickFirmware(uri: Uri) {
        viewModelScope.launch {
            try {
                val resolver = getApplication<Application>().contentResolver
                val name = displayName(uri)
                val bin = withContext(Dispatchers.IO) { resolver.openInputStream(uri)?.use { it.readBytes() } }
                    ?: throw IllegalStateException("cannot read $name")
                firmwareBin = bin
                val chip = FirmwareFile.chip(bin)
                val device = _ui.value.firmware?.chip
                _ui.update {
                    it.copy(
                        upload = FirmwareUpload(
                            name = name,
                            size = bin.size,
                            chip = chip,
                            chipMismatch = chip != null && device != null && !chip.equals(device, ignoreCase = true),
                        ),
                    )
                }
                verifyChecksum()
            } catch (e: CancellationException) {
                throw e
            } catch (e: Exception) {
                fail(e)
            }
        }
    }

    fun pickChecksums(uri: Uri) {
        viewModelScope.launch {
            try {
                val text = withContext(Dispatchers.IO) {
                    getApplication<Application>().contentResolver.openInputStream(uri)?.use { it.readBytes().decodeToString() }
                } ?: throw IllegalStateException("cannot read checksums")
                checksums = FirmwareFile.parseChecksums(text)
                verifyChecksum()
            } catch (e: CancellationException) {
                throw e
            } catch (e: Exception) {
                fail(e)
            }
        }
    }

    // SHA-256 of the image against checksums.txt, under the release asset name of its
    // chip (the file may have been renamed by the download)
    private suspend fun verifyChecksum() {
        val bin = firmwareBin ?: return
        val sums = checksums ?: return
        val chip = _ui.value.upload.chip ?: return
        val expected = sums[FirmwareFile.releaseName(chip)]
        val result = if (expected == null) {
            ChecksumResult.NOT_FOUND
        } else {
            val hash = withContext(Dispatchers.Default) { FirmwareFile.hex(MessageDigest.getInstance("SHA-256").digest(bin)) }
            if (hash == expected) ChecksumResult.MATCH else ChecksumResult.MISMATCH
        }
        _ui.update { it.copy(upload = it.upload.copy(checksum = result)) }
    }

    fun uploadFirmware() {
        val bin = firmwareBin ?: return
        if (!_ui.value.upload.canUpload) return
        _ui.update { it.copy(upload = it.upload.copy(status = FwStatus.UPLOADING, progress = 0f)) }
        viewModelScope.launch {
            val uptime = _ui.value.system?.uptime?.toLong() ?: Long.MAX_VALUE
            try {
                val md5 = withContext(Dispatchers.Default) { FirmwareFile.hex(MessageDigest.getInstance("MD5").digest(bin)) }
                // The last 10 % stand for the bridge writing and checking the flash
                client.uploadFirmware(url(), bin, md5) { p ->
                    _ui.update { it.copy(upload = it.upload.copy(progress = 0.9f * p)) }
                }
                _ui.update { it.copy(upload = it.upload.copy(status = FwStatus.DONE, progress = 1f)) }
                firmwareBin = null
                waitForReboot(uptime)
                _ui.update { it.copy(upload = FirmwareUpload()) }
            } catch (e: CancellationException) {
                throw e
            } catch (e: Exception) {
                _ui.update { it.copy(upload = it.upload.copy(status = FwStatus.IDLE, progress = 0f)) }
                fail(e)
            }
        }
    }

    private fun displayName(uri: Uri): String =
        getApplication<Application>().contentResolver.query(uri, null, null, null, null)?.use { c ->
            val i = c.getColumnIndex(OpenableColumns.DISPLAY_NAME)
            if (i >= 0 && c.moveToFirst()) c.getString(i) else null
        } ?: uri.lastPathSegment.orEmpty()
}

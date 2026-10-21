package io.github.flexy06.openneato.ui.history

import androidx.activity.compose.rememberLauncherForActivityResult
import androidx.activity.result.contract.ActivityResultContracts
import androidx.compose.foundation.Canvas
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.FlowRow
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.navigationBarsPadding
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.size
import androidx.compose.foundation.layout.width
import androidx.compose.foundation.shape.CircleShape
import androidx.compose.material.icons.Icons
import androidx.compose.material.icons.automirrored.filled.ArrowBack
import androidx.compose.material.icons.automirrored.filled.RotateLeft
import androidx.compose.material.icons.automirrored.filled.RotateRight
import androidx.compose.material.icons.filled.Pause
import androidx.compose.material.icons.filled.PlayArrow
import androidx.compose.material.icons.filled.Replay
import androidx.compose.material.icons.filled.SaveAlt
import androidx.compose.material3.Card
import androidx.compose.material3.CardDefaults
import androidx.compose.material3.FilledIconButton
import androidx.compose.material3.FilledTonalIconButton
import androidx.compose.material3.Icon
import androidx.compose.material3.IconButton
import androidx.compose.material3.LoadingIndicator
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.Scaffold
import androidx.compose.material3.Slider
import androidx.compose.material3.SnackbarHost
import androidx.compose.material3.SnackbarHostState
import androidx.compose.material3.Surface
import androidx.compose.material3.Text
import androidx.compose.material3.TextButton
import androidx.compose.material3.TopAppBar
import androidx.compose.runtime.Composable
import androidx.compose.runtime.LaunchedEffect
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableIntStateOf
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.saveable.rememberSaveable
import androidx.compose.runtime.setValue
import androidx.compose.runtime.withFrameNanos
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.clip
import androidx.compose.ui.geometry.Offset
import androidx.compose.ui.geometry.Size
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.platform.LocalContext
import androidx.compose.ui.res.stringResource
import androidx.compose.ui.semantics.contentDescription
import androidx.compose.ui.semantics.semantics
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.dp
import androidx.lifecycle.compose.collectAsStateWithLifecycle
import androidx.lifecycle.viewmodel.compose.viewModel
import io.github.flexy06.openneato.R
import io.github.flexy06.openneato.data.SessionMap
import io.github.flexy06.openneato.ui.theme.LocalStatusColors

private val SPEEDS = listOf(8, 32, 128)

@Composable
fun HistoryDetailScreen(name: String, onBack: () -> Unit, vm: HistoryDetailViewModel = viewModel(key = name)) {
    LaunchedEffect(name) { vm.open(name) }
    val ui by vm.ui.collectAsStateWithLifecycle()
    val snackbar = remember { SnackbarHostState() }
    val context = LocalContext.current
    val save = rememberLauncherForActivityResult(ActivityResultContracts.CreateDocument("application/x-ndjson")) { uri ->
        if (uri != null) vm.export(uri)
    }
    LaunchedEffect(Unit) {
        vm.messages.collect { snackbar.showSnackbar(context.historyMessage(it)) }
    }

    Scaffold(
        topBar = {
            TopAppBar(
                title = { Text(stringResource(R.string.history_map)) },
                navigationIcon = {
                    IconButton(onClick = onBack) {
                        Icon(Icons.AutoMirrored.Filled.ArrowBack, contentDescription = stringResource(R.string.back))
                    }
                },
                actions = {
                    if (ui.map != null && !ui.recording) {
                        IconButton(onClick = { save.launch(name.removeSuffix(".hs")) }) {
                            Icon(Icons.Filled.SaveAlt, contentDescription = stringResource(R.string.history_export))
                        }
                    }
                },
            )
        },
        snackbarHost = { SnackbarHost(snackbar) },
    ) { padding ->
        val map = ui.map
        when {
            map == null && ui.loading -> Box(Modifier.fillMaxSize().padding(padding), contentAlignment = Alignment.Center) {
                LoadingIndicator(Modifier.size(56.dp))
            }
            map == null -> Box(Modifier.fillMaxSize().padding(padding).padding(24.dp), contentAlignment = Alignment.Center) {
                Text(
                    ui.error ?: stringResource(R.string.history_no_data),
                    color = if (ui.error != null) MaterialTheme.colorScheme.error else MaterialTheme.colorScheme.onSurfaceVariant,
                )
            }
            else -> Detail(map, ui, Modifier.padding(padding))
        }
    }
}

@Composable
private fun Detail(map: SessionMap, ui: HistoryDetailUi, modifier: Modifier) {
    val status = LocalStatusColors.current
    val scheme = MaterialTheme.colorScheme
    val colors = MapColors(
        background = scheme.surfaceContainer,
        grid = scheme.outlineVariant.copy(alpha = 0.45f),
        walls = scheme.onSurface.copy(alpha = 0.55f),
        coverage = status.green.copy(alpha = 0.22f),
        path = status.amber.copy(alpha = 0.75f),
        start = status.green,
        end = status.red,
        robot = status.green,
        recharge = Color(0xFFFFCC00),
    )
    var rotation by rememberSaveable { mutableIntStateOf(0) }
    val duration = map.duration
    // Replay: null = the whole session, else the session time shown
    var time by remember { mutableStateOf<Double?>(null) }
    var playing by remember { mutableStateOf(false) }
    var speedIdx by rememberSaveable { mutableIntStateOf(0) }

    LaunchedEffect(playing, speedIdx) {
        if (!playing) return@LaunchedEffect
        var last = withFrameNanos { it }
        if ((time ?: duration) >= duration) time = 0.0
        while (playing) {
            val now = withFrameNanos { it }
            val next = (time ?: 0.0) + (now - last) / 1e9 * SPEEDS[speedIdx]
            last = now
            if (next >= duration) {
                time = null
                playing = false
            } else {
                time = next
            }
        }
    }

    Column(modifier.fillMaxSize().padding(horizontal = 16.dp)) {
        Stats(map, ui)
        Spacer(Modifier.height(8.dp))
        Card(
            colors = CardDefaults.cardColors(containerColor = scheme.surfaceContainer),
            modifier = Modifier.fillMaxWidth().weight(1f),
        ) {
            Box(Modifier.fillMaxSize()) {
                SessionMapView(
                    map,
                    colors,
                    rotation,
                    ui.recording,
                    time,
                    Modifier.fillMaxSize().clip(MaterialTheme.shapes.large),
                )
                Row(Modifier.align(Alignment.TopEnd).padding(8.dp)) {
                    FilledTonalIconButton(onClick = { rotation = (rotation + 270) % 360 }) {
                        Icon(Icons.AutoMirrored.Filled.RotateLeft, stringResource(R.string.history_rotate_left))
                    }
                    FilledTonalIconButton(onClick = { rotation = (rotation + 90) % 360 }) {
                        Icon(Icons.AutoMirrored.Filled.RotateRight, stringResource(R.string.history_rotate_right))
                    }
                }
            }
        }
        Legend(map, ui.recording, colors)
        Text(
            stringResource(R.string.history_hint),
            style = MaterialTheme.typography.bodySmall,
            color = scheme.onSurfaceVariant,
            modifier = Modifier.padding(horizontal = 4.dp),
        )
        if (!ui.recording && duration > 0) {
            Player(
                map = map,
                time = time ?: duration,
                playing = playing,
                speed = SPEEDS[speedIdx],
                onPlay = { playing = !playing },
                onRestart = {
                    time = 0.0
                    playing = true
                },
                onSeek = {
                    playing = false
                    time = it
                },
                onSpeed = { speedIdx = (speedIdx + 1) % SPEEDS.size },
            )
        } else {
            Spacer(Modifier.navigationBarsPadding().height(12.dp))
        }
    }
}

@Composable
private fun Stats(map: SessionMap, ui: HistoryDetailUi) {
    val summary = ui.file?.summary
    val session = ui.file?.session
    val duration = summary?.duration?.toDouble() ?: map.summary?.duration ?: map.duration
    val distance = summary?.distanceTraveled?.toDouble() ?: map.summary?.distance
    val area = summary?.areaCovered?.toDouble() ?: map.summary?.area
    val batteryStart = session?.battery?.toInt() ?: map.session?.battery
    val batteryEnd = summary?.batteryEnd?.toInt() ?: map.summary?.batteryEnd
    val recharges = summary?.recharges?.toInt() ?: map.summary?.recharges ?: 0
    FlowRow(
        Modifier.fillMaxWidth().padding(top = 4.dp),
        horizontalArrangement = Arrangement.spacedBy(8.dp),
        verticalArrangement = Arrangement.spacedBy(8.dp),
    ) {
        Stat(stringResource(R.string.history_duration), formatDuration(duration))
        distance?.let { Stat(stringResource(R.string.history_distance), formatDistance(it)) }
        area?.let { Stat(stringResource(R.string.history_area), formatArea(it)) }
        Stat(stringResource(R.string.history_battery), "${batteryStart ?: "?"} → ${batteryEnd ?: "?"} %")
        if (recharges > 0) Stat(stringResource(R.string.history_recharges), recharges.toString())
    }
}

@Composable
private fun Stat(label: String, value: String) {
    Surface(shape = MaterialTheme.shapes.medium, color = MaterialTheme.colorScheme.surfaceContainerHigh) {
        Column(Modifier.padding(horizontal = 12.dp, vertical = 8.dp)) {
            Text(label, style = MaterialTheme.typography.labelSmall, color = MaterialTheme.colorScheme.onSurfaceVariant)
            Text(value, style = MaterialTheme.typography.titleSmall, fontWeight = FontWeight.SemiBold)
        }
    }
}

@Composable
private fun Legend(map: SessionMap, recording: Boolean, colors: MapColors) {
    FlowRow(
        Modifier.fillMaxWidth().padding(horizontal = 4.dp, vertical = 8.dp),
        horizontalArrangement = Arrangement.spacedBy(14.dp),
    ) {
        LegendItem(colors.start, stringResource(R.string.history_start))
        LegendItem(
            if (recording) colors.start else colors.end,
            stringResource(if (recording) R.string.history_current else R.string.history_end),
        )
        LegendItem(colors.coverage.copy(alpha = 0.5f), stringResource(R.string.history_coverage), square = true)
        if (map.scans.isNotEmpty()) LegendItem(colors.walls, stringResource(R.string.history_walls))
        if (map.recharges.isNotEmpty()) LegendItem(colors.recharge, stringResource(R.string.history_recharge))
    }
}

@Composable
private fun LegendItem(color: Color, label: String, square: Boolean = false) {
    Row(verticalAlignment = Alignment.CenterVertically) {
        Surface(
            shape = if (square) MaterialTheme.shapes.extraSmall else CircleShape,
            color = color,
            modifier = Modifier.size(10.dp),
        ) {}
        Spacer(Modifier.width(6.dp))
        Text(label, style = MaterialTheme.typography.labelMedium)
    }
}

@Composable
private fun Player(
    map: SessionMap,
    time: Double,
    playing: Boolean,
    speed: Int,
    onPlay: () -> Unit,
    onRestart: () -> Unit,
    onSeek: (Double) -> Unit,
    onSpeed: () -> Unit,
) {
    val duration = map.duration
    val status = LocalStatusColors.current
    Column(Modifier.fillMaxWidth().navigationBarsPadding().padding(bottom = 8.dp)) {
        // Timeline: cleaning in green, charging stops in yellow
        Spacer(Modifier.height(8.dp))
        Canvas(Modifier.fillMaxWidth().padding(horizontal = 12.dp).height(4.dp)) {
            drawRect(status.green.copy(alpha = 0.7f))
            for (r in map.recharges) {
                val a = (r.ts / duration).coerceIn(0.0, 1.0).toFloat()
                val b = (r.endTs / duration).coerceIn(0.0, 1.0).toFloat()
                if (b > a) {
                    drawRect(Color(0xFFFFCC00), Offset(a * size.width, 0f), Size((b - a) * size.width, size.height))
                }
            }
        }
        Slider(
            value = time.toFloat().coerceIn(0f, duration.toFloat()),
            onValueChange = { onSeek(it.toDouble()) },
            valueRange = 0f..duration.toFloat(),
        )
        Row(verticalAlignment = Alignment.CenterVertically) {
            FilledIconButton(onClick = onPlay) {
                Icon(
                    if (playing) Icons.Filled.Pause else Icons.Filled.PlayArrow,
                    stringResource(if (playing) R.string.history_pause else R.string.history_play),
                )
            }
            IconButton(onClick = onRestart) {
                Icon(Icons.Filled.Replay, stringResource(R.string.history_restart))
            }
            Text(
                "${formatClock(time)} / ${formatClock(duration)}",
                style = MaterialTheme.typography.bodyMedium,
                modifier = Modifier.weight(1f).padding(start = 4.dp),
            )
            val speedLabel = stringResource(R.string.history_speed)
            TextButton(onClick = onSpeed, modifier = Modifier.semantics { contentDescription = "$speedLabel ${speed}×" }) {
                Text("${speed}×")
            }
        }
    }
}

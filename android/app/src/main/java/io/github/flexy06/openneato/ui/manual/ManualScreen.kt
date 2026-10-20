package io.github.flexy06.openneato.ui.manual

import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.aspectRatio
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
import androidx.compose.material.icons.filled.Air
import androidx.compose.material.icons.filled.AutoAwesome
import androidx.compose.material.icons.filled.Bolt
import androidx.compose.material.icons.filled.Brush
import androidx.compose.material.icons.filled.Cyclone
import androidx.compose.material.icons.filled.Gamepad
import androidx.compose.material.icons.filled.Stop
import androidx.compose.material.icons.filled.Warning
import androidx.compose.material3.Button
import androidx.compose.material3.ButtonDefaults
import androidx.compose.material3.Icon
import androidx.compose.material3.IconButton
import androidx.compose.material3.LoadingIndicator
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.Scaffold
import androidx.compose.material3.SnackbarHost
import androidx.compose.material3.SnackbarHostState
import androidx.compose.material3.Surface
import androidx.compose.material3.Text
import androidx.compose.material3.ToggleButton
import androidx.compose.material3.TopAppBar
import androidx.compose.runtime.Composable
import androidx.compose.runtime.LaunchedEffect
import androidx.compose.runtime.getValue
import androidx.compose.runtime.remember
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.graphics.vector.ImageVector
import androidx.compose.ui.platform.LocalContext
import androidx.compose.ui.res.stringResource
import androidx.compose.ui.text.style.TextAlign
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import androidx.compose.foundation.text.TextAutoSize
import androidx.lifecycle.compose.collectAsStateWithLifecycle
import androidx.lifecycle.viewmodel.compose.viewModel
import io.github.flexy06.openneato.R
import io.github.flexy06.openneato.ui.common.LidarView
import io.github.flexy06.openneato.ui.theme.LocalStatusColors

@Composable
fun ManualScreen(onBack: () -> Unit, vm: ManualViewModel = viewModel()) {
    val ui by vm.ui.collectAsStateWithLifecycle()
    val snackbar = remember { SnackbarHostState() }
    val context = LocalContext.current
    LaunchedEffect(Unit) {
        vm.errors.collect { snackbar.showSnackbar(context.getString(R.string.action_failed, it)) }
    }
    // Leave only after the robot confirmed that manual mode ended
    LaunchedEffect(ui.stopping, ui.active) {
        if (ui.stopping && !ui.active) onBack()
    }

    Scaffold(
        topBar = {
            TopAppBar(
                title = { Text(stringResource(R.string.manual)) },
                navigationIcon = {
                    IconButton(onClick = onBack, enabled = !ui.stopping) {
                        Icon(Icons.AutoMirrored.Filled.ArrowBack, contentDescription = stringResource(R.string.back))
                    }
                },
            )
        },
        snackbarHost = { SnackbarHost(snackbar) },
    ) { padding ->
        Column(
            Modifier
                .fillMaxSize()
                .padding(padding)
                .navigationBarsPadding()
                .padding(16.dp),
            horizontalAlignment = Alignment.CenterHorizontally,
        ) {
            LidarPanel(ui, Modifier.weight(1f, fill = false))
            Spacer(Modifier.height(16.dp))
            if (ui.active) {
                Controls(ui, vm)
            } else {
                StartPanel(ui, vm::start)
            }
        }
    }
}

@Composable
private fun LidarPanel(ui: ManualUi, modifier: Modifier) {
    Box(
        modifier
            .fillMaxWidth()
            .aspectRatio(1f),
    ) {
        Surface(shape = MaterialTheme.shapes.large, modifier = Modifier.fillMaxSize()) {
            LidarView(ui.lidar.data, ui.moving, Modifier.fillMaxSize())
        }
        ui.charger.data?.let { c ->
            val pct = c.fuelPercent.toInt()
            Overlay(Modifier.align(Alignment.TopEnd).padding(10.dp)) {
                if (c.chargingActive || c.extPwrPresent) {
                    Icon(Icons.Filled.Bolt, contentDescription = null, modifier = Modifier.size(16.dp))
                }
                Text("$pct %", style = MaterialTheme.typography.labelLarge)
            }
        }
        safetyWarning(ui.status.data)?.let { w ->
            val text = when (w) {
                is SafetyWarning.Simple -> stringResource(w.text)
                is SafetyWarning.Bumper ->
                    stringResource(R.string.warn_bumper, w.sides.map { stringResource(it) }.joinToString(", "))
            }
            Surface(
                color = MaterialTheme.colorScheme.errorContainer,
                shape = MaterialTheme.shapes.small,
                modifier = Modifier.align(Alignment.TopCenter).padding(top = 48.dp, start = 12.dp, end = 12.dp),
            ) {
                Row(Modifier.padding(10.dp), verticalAlignment = Alignment.CenterVertically) {
                    Icon(Icons.Filled.Warning, contentDescription = null, tint = MaterialTheme.colorScheme.error)
                    Spacer(Modifier.width(8.dp))
                    Text(text, color = MaterialTheme.colorScheme.onErrorContainer)
                }
            }
        }
        val center = when {
            ui.lidar.data == null && !ui.active -> stringResource(R.string.manual_not_active)
            ui.lidar.data == null && ui.lidar.error != null -> stringResource(R.string.lidar_unavailable)
            else -> null
        }
        center?.let {
            Text(
                it,
                color = MaterialTheme.colorScheme.onSurfaceVariant,
                // Below the robot marker in the centre
                modifier = Modifier.align(Alignment.Center).padding(top = 56.dp),
            )
        }
        ui.lidar.data?.let { scan ->
            val points = scan.validPoints.toInt()
            val hz = scan.rotationSpeed.toDouble()
            val notes = buildList {
                if (points < 90) add(stringResource(R.string.lidar_low_quality, points))
                if (hz > 0 && hz < 4.0) add(stringResource(R.string.lidar_slow, hz))
            }
            if (notes.isNotEmpty()) {
                Overlay(Modifier.align(Alignment.BottomCenter).padding(10.dp)) {
                    Text(
                        notes.joinToString(" · "),
                        style = MaterialTheme.typography.labelMedium,
                        color = LocalStatusColors.current.amber,
                    )
                }
            }
        }
    }
}

@Composable
private fun Overlay(modifier: Modifier, content: @Composable () -> Unit) {
    Surface(color = MaterialTheme.colorScheme.surfaceContainerHigh.copy(alpha = 0.9f), shape = CircleShape, modifier = modifier) {
        Row(
            Modifier.padding(horizontal = 10.dp, vertical = 4.dp),
            verticalAlignment = Alignment.CenterVertically,
            horizontalArrangement = Arrangement.spacedBy(4.dp),
        ) { content() }
    }
}

@Composable
private fun StartPanel(ui: ManualUi, onStart: () -> Unit) {
    val busy = ui.flags.cleaning || ui.flags.docking || ui.flags.guided
    if (busy) {
        Text(
            stringResource(R.string.manual_blocked),
            textAlign = TextAlign.Center,
            color = MaterialTheme.colorScheme.onSurfaceVariant,
        )
        Spacer(Modifier.height(12.dp))
    }
    Button(
        onClick = onStart,
        enabled = !busy && !ui.starting && ui.state.data != null,
        modifier = Modifier.fillMaxWidth().height(64.dp),
    ) {
        if (ui.starting) {
            LoadingIndicator(Modifier.size(28.dp), color = MaterialTheme.colorScheme.onPrimary)
            Spacer(Modifier.width(8.dp))
            Text(stringResource(R.string.manual_starting))
        } else {
            Icon(Icons.Filled.Gamepad, contentDescription = null)
            Spacer(Modifier.width(8.dp))
            Text(stringResource(R.string.manual_start))
        }
    }
}

@Composable
private fun Controls(ui: ManualUi, vm: ManualViewModel) {
    val enabled = !ui.stopping
    Joystick(onMove = vm::move, onRelease = vm::release, enabled = enabled)
    Spacer(Modifier.height(16.dp))
    val motorsEnabled = enabled && !ui.motorPending && ui.status.data?.lifted != true
    val m = ui.motors
    Row(Modifier.fillMaxWidth(), horizontalArrangement = Arrangement.spacedBy(6.dp)) {
        MotorToggle(Icons.Filled.Brush, R.string.motor_brush, m.brush, motorsEnabled) { vm.setMotors(m.copy(brush = it)) }
        MotorToggle(Icons.Filled.Air, R.string.motor_vacuum, m.vacuum, motorsEnabled) { vm.setMotors(m.copy(vacuum = it)) }
        MotorToggle(Icons.Filled.Cyclone, R.string.motor_side_brush, m.sideBrush, motorsEnabled) {
            vm.setMotors(m.copy(sideBrush = it))
        }
        MotorToggle(Icons.Filled.AutoAwesome, R.string.motor_all, m.all, motorsEnabled) { vm.setMotors(Motors(it, it, it)) }
    }
    Spacer(Modifier.height(16.dp))
    Button(
        onClick = vm::stop,
        enabled = enabled,
        colors = ButtonDefaults.buttonColors(
            containerColor = MaterialTheme.colorScheme.error,
            contentColor = MaterialTheme.colorScheme.onError,
        ),
        modifier = Modifier.fillMaxWidth().height(56.dp),
    ) {
        Icon(Icons.Filled.Stop, contentDescription = null)
        Spacer(Modifier.width(8.dp))
        Text(stringResource(R.string.action_stop))
    }
}

@Composable
private fun androidx.compose.foundation.layout.RowScope.MotorToggle(
    icon: ImageVector,
    label: Int,
    checked: Boolean,
    enabled: Boolean,
    onChange: (Boolean) -> Unit,
) {
    ToggleButton(
        checked = checked,
        onCheckedChange = onChange,
        enabled = enabled,
        contentPadding = ButtonDefaults.ContentPadding,
        modifier = Modifier.weight(1f).height(64.dp),
    ) {
        Column(horizontalAlignment = Alignment.CenterHorizontally) {
            Icon(icon, contentDescription = null, modifier = Modifier.size(20.dp))
            // "Seitenbürste" does not fit a quarter of a phone width at full size
            Text(
                stringResource(label),
                style = MaterialTheme.typography.labelSmall,
                maxLines = 1,
                autoSize = TextAutoSize.StepBased(minFontSize = 8.sp, maxFontSize = 11.sp),
            )
        }
    }
}

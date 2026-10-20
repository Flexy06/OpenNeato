package io.github.flexy06.openneato.ui.manual

import androidx.compose.animation.core.Animatable
import androidx.compose.animation.core.VectorConverter
import androidx.compose.foundation.Canvas
import androidx.compose.foundation.gestures.awaitEachGesture
import androidx.compose.foundation.gestures.awaitFirstDown
import androidx.compose.foundation.layout.size
import androidx.compose.material3.MaterialTheme
import androidx.compose.runtime.Composable
import androidx.compose.runtime.remember
import androidx.compose.runtime.rememberCoroutineScope
import androidx.compose.ui.Modifier
import androidx.compose.ui.geometry.Offset
import androidx.compose.ui.input.pointer.pointerInput
import androidx.compose.ui.unit.Dp
import androidx.compose.ui.unit.dp
import kotlinx.coroutines.launch
import kotlin.math.min

// Thumb stick: reports x (-1 left .. 1 right), y (-1 back .. 1 forward) and magnitude 0..1
// while pressed, and a release when let go (the knob springs back to the centre).
@Composable
fun Joystick(
    onMove: (x: Float, y: Float, magnitude: Float) -> Unit,
    onRelease: () -> Unit,
    enabled: Boolean,
    modifier: Modifier = Modifier,
    size: Dp = 168.dp,
) {
    val knob = remember { Animatable(Offset.Zero, Offset.VectorConverter) }
    val scope = rememberCoroutineScope()
    val base = MaterialTheme.colorScheme.surfaceContainerHigh
    val ring = MaterialTheme.colorScheme.outlineVariant
    val knobColor = if (enabled) MaterialTheme.colorScheme.primary else MaterialTheme.colorScheme.outline

    Canvas(
        modifier
            .size(size)
            .pointerInput(enabled) {
                if (!enabled) return@pointerInput
                val radius = min(this.size.width, this.size.height) / 2f
                val centre = Offset(this.size.width / 2f, this.size.height / 2f)
                fun update(pos: Offset) {
                    var d = pos - centre
                    val len = d.getDistance()
                    if (len > radius) d = d * (radius / len)
                    scope.launch { knob.snapTo(d) }
                    val mag = (d.getDistance() / radius).coerceIn(0f, 1f)
                    onMove(d.x / radius, -d.y / radius, if (mag < 0.08f) 0f else mag)
                }
                awaitEachGesture {
                    val down = awaitFirstDown()
                    update(down.position)
                    while (true) {
                        val change = awaitPointerEvent().changes.firstOrNull { it.id == down.id } ?: break
                        if (!change.pressed) break
                        update(change.position)
                        change.consume()
                    }
                    scope.launch { knob.animateTo(Offset.Zero) }
                    onRelease()
                }
            },
    ) {
        val r = min(this.size.width, this.size.height) / 2f
        drawCircle(base, r)
        drawCircle(ring, r * 0.55f, style = androidx.compose.ui.graphics.drawscope.Stroke(1.dp.toPx()))
        drawCircle(knobColor, r * 0.32f, center = center + knob.value)
    }
}

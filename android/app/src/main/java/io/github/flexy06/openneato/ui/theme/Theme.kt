package io.github.flexy06.openneato.ui.theme

import androidx.compose.foundation.isSystemInDarkTheme
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material3.MaterialExpressiveTheme
import androidx.compose.material3.MotionScheme
import androidx.compose.material3.Shapes
import androidx.compose.material3.darkColorScheme
import androidx.compose.material3.lightColorScheme
import androidx.compose.runtime.Composable
import androidx.compose.runtime.CompositionLocalProvider
import androidx.compose.runtime.Immutable
import androidx.compose.runtime.staticCompositionLocalOf
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.unit.dp

// Forest/sage palette, same tokens as frontend/src/style.css. Forest/sage = brand and
// actions, terracotta = warnings and destructive actions only, no purple/blue accents.
private val Dark = darkColorScheme(
    primary = Color(0xFF3B7A66),
    onPrimary = Color(0xFFFBF7F2),
    primaryContainer = Color(0xFF223630),
    onPrimaryContainer = Color(0xFF9CC8B2),
    secondary = Color(0xFF9CC8B2),
    onSecondary = Color(0xFF131714),
    secondaryContainer = Color(0xFF232B26),
    onSecondaryContainer = Color(0xFFEEE9E1),
    tertiary = Color(0xFFE2B15C),
    onTertiary = Color(0xFF131714),
    error = Color(0xFFE0877D),
    onError = Color(0xFF131714),
    errorContainer = Color(0xFF3A2523),
    onErrorContainer = Color(0xFFF2C4BE),
    background = Color(0xFF131714),
    onBackground = Color(0xFFEEE9E1),
    surface = Color(0xFF131714),
    onSurface = Color(0xFFEEE9E1),
    surfaceVariant = Color(0xFF232B26),
    onSurfaceVariant = Color(0xFFA9AEA6),
    surfaceContainerLowest = Color(0xFF0F1210),
    surfaceContainerLow = Color(0xFF171C19),
    surfaceContainer = Color(0xFF1A201C),
    surfaceContainerHigh = Color(0xFF232B26),
    surfaceContainerHighest = Color(0xFF2A332D),
    outline = Color(0xFF4A524C),
    outlineVariant = Color(0xFF2A302C),
    inverseSurface = Color(0xFFEEE9E1),
    inverseOnSurface = Color(0xFF292C2A),
    inversePrimary = Color(0xFF244D43),
)

private val Light = lightColorScheme(
    primary = Color(0xFF244D43),
    onPrimary = Color(0xFFFBF7F2),
    primaryContainer = Color(0xFFDCE9E2),
    onPrimaryContainer = Color(0xFF244D43),
    secondary = Color(0xFF2D5D51),
    onSecondary = Color(0xFFFBF7F2),
    secondaryContainer = Color(0xFFEDE4DA),
    onSecondaryContainer = Color(0xFF292C2A),
    tertiary = Color(0xFFA06A10),
    onTertiary = Color(0xFFFBF7F2),
    error = Color(0xFFB95F59),
    onError = Color(0xFFFBF7F2),
    errorContainer = Color(0xFFF4DEDA),
    onErrorContainer = Color(0xFF6E2C27),
    background = Color(0xFFF2EAE2),
    onBackground = Color(0xFF292C2A),
    surface = Color(0xFFF2EAE2),
    onSurface = Color(0xFF292C2A),
    surfaceVariant = Color(0xFFEDE4DA),
    onSurfaceVariant = Color(0xFF737671),
    surfaceContainerLowest = Color(0xFFFFFFFF),
    surfaceContainerLow = Color(0xFFFFFDF9),
    surfaceContainer = Color(0xFFFFFDF9),
    surfaceContainerHigh = Color(0xFFFBF7F2),
    surfaceContainerHighest = Color(0xFFEDE4DA),
    outline = Color(0xFFBDB8B0),
    outlineVariant = Color(0xFFE2DAD0),
    inverseSurface = Color(0xFF292C2A),
    inverseOnSurface = Color(0xFFF2EAE2),
    inversePrimary = Color(0xFF9CC8B2),
)

// Status hues (green success, amber warning, terracotta error, blue connection)
@Immutable
data class StatusColors(val green: Color, val amber: Color, val red: Color, val blue: Color)

private val DarkStatus = StatusColors(Color(0xFF7CC79C), Color(0xFFE2B15C), Color(0xFFE0877D), Color(0xFF86B6D2))
private val LightStatus = StatusColors(Color(0xFF2F7D55), Color(0xFFA06A10), Color(0xFFB95F59), Color(0xFF3A6F92))

val LocalStatusColors = staticCompositionLocalOf { DarkStatus }

private val AppShapes = Shapes(
    small = RoundedCornerShape(12.dp),
    medium = RoundedCornerShape(18.dp),
    large = RoundedCornerShape(24.dp),
)

@Composable
fun OpenNeatoTheme(dark: Boolean = isSystemInDarkTheme(), content: @Composable () -> Unit) {
    CompositionLocalProvider(LocalStatusColors provides if (dark) DarkStatus else LightStatus) {
        MaterialExpressiveTheme(
            colorScheme = if (dark) Dark else Light,
            motionScheme = MotionScheme.expressive(),
            shapes = AppShapes,
            content = content,
        )
    }
}

package io.github.flexy06.openneato.data

import android.content.Context
import androidx.datastore.preferences.core.edit
import androidx.datastore.preferences.core.stringPreferencesKey
import androidx.datastore.preferences.preferencesDataStore
import kotlinx.coroutines.flow.Flow
import kotlinx.coroutines.flow.map

private val Context.dataStore by preferencesDataStore(name = "settings")

class SettingsStore(private val context: Context) {
    private val robotUrlKey = stringPreferencesKey("robot_url")
    private val themeKey = stringPreferencesKey("theme")

    // Base URL of the chosen robot, e.g. "http://192.168.1.42"; null until one is picked
    val robotUrl: Flow<String?> = context.dataStore.data.map { it[robotUrlKey] }

    suspend fun setRobotUrl(url: String?) {
        context.dataStore.edit { prefs ->
            if (url == null) prefs.remove(robotUrlKey) else prefs[robotUrlKey] = url
        }
    }

    // App theme: follow the system, or always light / dark
    val theme: Flow<AppTheme> = context.dataStore.data.map { p ->
        AppTheme.entries.firstOrNull { it.name == p[themeKey] } ?: AppTheme.SYSTEM
    }

    suspend fun setTheme(theme: AppTheme) {
        context.dataStore.edit { it[themeKey] = theme.name }
    }
}

enum class AppTheme { SYSTEM, LIGHT, DARK }

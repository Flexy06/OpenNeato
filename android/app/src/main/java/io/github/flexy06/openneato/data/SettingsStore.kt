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

    // Base URL of the chosen robot, e.g. "http://192.168.1.42"; null until one is picked
    val robotUrl: Flow<String?> = context.dataStore.data.map { it[robotUrlKey] }

    suspend fun setRobotUrl(url: String?) {
        context.dataStore.edit { prefs ->
            if (url == null) prefs.remove(robotUrlKey) else prefs[robotUrlKey] = url
        }
    }
}

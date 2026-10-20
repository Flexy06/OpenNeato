package io.github.flexy06.openneato

import android.app.Application
import io.github.flexy06.openneato.data.RobotClient
import io.github.flexy06.openneato.data.SettingsStore

// Manual dependency container: the app is small enough that a DI framework would cost more than it saves
class OpenNeatoApp : Application() {
    val settings by lazy { SettingsStore(this) }
    val robot by lazy { RobotClient() }
}

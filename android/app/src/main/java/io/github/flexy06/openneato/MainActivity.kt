package io.github.flexy06.openneato

import android.os.Bundle
import androidx.activity.ComponentActivity
import androidx.activity.compose.setContent
import androidx.activity.enableEdgeToEdge
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.Surface
import androidx.compose.runtime.Composable
import androidx.compose.runtime.collectAsState
import androidx.compose.runtime.getValue
import androidx.compose.runtime.remember
import androidx.navigation.NavHostController
import androidx.navigation.compose.NavHost
import androidx.navigation.compose.composable
import androidx.navigation.compose.rememberNavController
import io.github.flexy06.openneato.ui.connect.ConnectScreen
import io.github.flexy06.openneato.ui.dashboard.DashboardScreen
import io.github.flexy06.openneato.ui.theme.OpenNeatoTheme
import kotlinx.coroutines.flow.map
import kotlinx.coroutines.flow.take

private object Routes {
    const val CONNECT = "connect"
    const val DASHBOARD = "dashboard"
}

class MainActivity : ComponentActivity() {
    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        enableEdgeToEdge()
        val settings = (application as OpenNeatoApp).settings
        setContent {
            OpenNeatoTheme {
                Surface(color = MaterialTheme.colorScheme.background) {
                    // null = DataStore not read yet; avoids flashing the robot search on every start
                    val start by remember { settings.robotUrl.take(1).map { if (it == null) Routes.CONNECT else Routes.DASHBOARD } }
                        .collectAsState(initial = null)
                    start?.let { AppNav(rememberNavController(), it) }
                }
            }
        }
    }
}

@Composable
private fun AppNav(nav: NavHostController, start: String) {
    NavHost(nav, startDestination = start) {
        composable(Routes.CONNECT) {
            ConnectScreen(onConnected = {
                nav.navigate(Routes.DASHBOARD) { popUpTo(Routes.CONNECT) { inclusive = true } }
            })
        }
        composable(Routes.DASHBOARD) {
            DashboardScreen(onChangeRobot = {
                nav.navigate(Routes.CONNECT) { popUpTo(Routes.DASHBOARD) { inclusive = true } }
            })
        }
    }
}

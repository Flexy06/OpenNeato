package io.github.flexy06.openneato

import android.net.Uri
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
import androidx.navigation.NavType
import androidx.navigation.compose.NavHost
import androidx.navigation.compose.composable
import androidx.navigation.compose.rememberNavController
import androidx.navigation.navArgument
import io.github.flexy06.openneato.ui.connect.ConnectScreen
import io.github.flexy06.openneato.ui.dashboard.DashboardScreen
import io.github.flexy06.openneato.ui.history.HistoryDetailScreen
import io.github.flexy06.openneato.ui.history.HistoryListScreen
import io.github.flexy06.openneato.ui.manual.ManualScreen
import io.github.flexy06.openneato.ui.schedule.ScheduleScreen
import io.github.flexy06.openneato.ui.theme.OpenNeatoTheme
import kotlinx.coroutines.flow.map
import kotlinx.coroutines.flow.take

private object Routes {
    const val CONNECT = "connect"
    const val DASHBOARD = "dashboard"
    const val MANUAL = "manual"
    const val SCHEDULE = "schedule"
    const val HISTORY = "history"
    const val HISTORY_DETAIL = "history/{name}"
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
            DashboardScreen(
                onChangeRobot = {
                    nav.navigate(Routes.CONNECT) { popUpTo(Routes.DASHBOARD) { inclusive = true } }
                },
                onOpenManual = { nav.navigate(Routes.MANUAL) },
                onOpenSchedule = { nav.navigate(Routes.SCHEDULE) },
                onOpenHistory = { nav.navigate(Routes.HISTORY) },
            )
        }
        composable(Routes.MANUAL) {
            ManualScreen(onBack = { nav.popBackStack() })
        }
        composable(Routes.SCHEDULE) {
            ScheduleScreen(onBack = { nav.popBackStack() })
        }
        composable(Routes.HISTORY) {
            HistoryListScreen(
                onBack = { nav.popBackStack() },
                onOpen = { name -> nav.navigate("history/${Uri.encode(name)}") },
            )
        }
        composable(
            Routes.HISTORY_DETAIL,
            arguments = listOf(navArgument("name") { type = NavType.StringType }),
        ) { entry ->
            HistoryDetailScreen(name = entry.arguments?.getString("name").orEmpty(), onBack = { nav.popBackStack() })
        }
    }
}

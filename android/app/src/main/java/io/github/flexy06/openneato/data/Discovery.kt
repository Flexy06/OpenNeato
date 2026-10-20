package io.github.flexy06.openneato.data

import android.content.Context
import android.net.nsd.NsdManager
import android.net.nsd.NsdServiceInfo
import android.os.Build
import kotlinx.coroutines.CompletableDeferred
import kotlinx.coroutines.channels.Channel
import kotlinx.coroutines.channels.awaitClose
import kotlinx.coroutines.flow.Flow
import kotlinx.coroutines.flow.callbackFlow
import kotlinx.coroutines.launch
import kotlinx.coroutines.withTimeoutOrNull
import java.net.Inet4Address
import java.util.concurrent.Executor

data class FoundRobot(val name: String, val url: String, val model: String, val version: String)

// The firmware announces itself as "_http._tcp" under its hostname (see wifi_manager.cpp).
// Every HTTP service found is resolved and probed, only OpenNeato bridges are reported.
class Discovery(context: Context, private val client: RobotClient) {
    private val nsd = context.getSystemService(Context.NSD_SERVICE) as NsdManager

    fun robots(): Flow<FoundRobot> = callbackFlow {
        val pending = Channel<NsdServiceInfo>(Channel.UNLIMITED)
        val seen = mutableSetOf<String>()
        val listener = object : NsdManager.DiscoveryListener {
            override fun onServiceFound(info: NsdServiceInfo) {
                pending.trySend(info)
            }
            override fun onServiceLost(info: NsdServiceInfo) {}
            override fun onDiscoveryStarted(serviceType: String) {}
            override fun onDiscoveryStopped(serviceType: String) {}
            override fun onStartDiscoveryFailed(serviceType: String, errorCode: Int) {
                close(IllegalStateException("mDNS discovery failed ($errorCode)"))
            }
            override fun onStopDiscoveryFailed(serviceType: String, errorCode: Int) {}
        }
        // Resolve one service at a time: older NsdManager versions reject parallel resolves
        launch {
            for (info in pending) {
                val host = withTimeoutOrNull(5000) { resolve(info) } ?: continue
                val url = "http://$host" + if (info.port == 80 || info.port == 0) "" else ":${info.port}"
                if (!seen.add(url)) continue
                val fw = client.probe(url) ?: continue
                trySend(FoundRobot(fw.hostname.ifBlank { info.serviceName }, url, fw.model, fw.version))
            }
        }
        nsd.discoverServices("_http._tcp", NsdManager.PROTOCOL_DNS_SD, listener)
        awaitClose {
            runCatching { nsd.stopServiceDiscovery(listener) }
            pending.close()
        }
    }

    private suspend fun resolve(info: NsdServiceInfo): String? {
        val result = CompletableDeferred<String?>()
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.UPSIDE_DOWN_CAKE) {
            val cb = object : NsdManager.ServiceInfoCallback {
                override fun onServiceInfoCallbackRegistrationFailed(errorCode: Int) {
                    result.complete(null)
                }
                override fun onServiceUpdated(updated: NsdServiceInfo) {
                    val addr = updated.hostAddresses.firstOrNull { it is Inet4Address } ?: updated.hostAddresses.firstOrNull()
                    if (addr != null) result.complete(addr.hostAddress)
                }
                override fun onServiceLost() {}
                override fun onServiceInfoCallbackUnregistered() {}
            }
            // Direct executor: the callbacks only complete a Deferred, and NsdManager still
            // delivers "unregistered" after we are done (a shut-down pool would crash the app)
            nsd.registerServiceInfoCallback(info, Executor { it.run() }, cb)
            try {
                return result.await()
            } finally {
                runCatching { nsd.unregisterServiceInfoCallback(cb) }
            }
        }
        @Suppress("DEPRECATION")
        nsd.resolveService(
            info,
            object : NsdManager.ResolveListener {
                override fun onResolveFailed(serviceInfo: NsdServiceInfo, errorCode: Int) {
                    result.complete(null)
                }
                override fun onServiceResolved(serviceInfo: NsdServiceInfo) {
                    result.complete(serviceInfo.host?.hostAddress)
                }
            },
        )
        return result.await()
    }
}

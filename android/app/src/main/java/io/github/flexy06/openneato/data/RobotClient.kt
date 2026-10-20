package io.github.flexy06.openneato.data

import io.github.flexy06.openneato.api.apis.ActionsApi
import io.github.flexy06.openneato.api.apis.FirmwareApi
import io.github.flexy06.openneato.api.apis.GuidedApi
import io.github.flexy06.openneato.api.apis.SensorsApi
import io.github.flexy06.openneato.api.apis.SystemApi
import io.github.flexy06.openneato.api.infrastructure.ClientError
import io.github.flexy06.openneato.api.infrastructure.ClientException
import io.github.flexy06.openneato.api.infrastructure.ServerError
import io.github.flexy06.openneato.api.infrastructure.ServerException
import io.github.flexy06.openneato.api.models.FirmwareVersion
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.withContext
import kotlinx.serialization.json.Json
import kotlinx.serialization.json.jsonObject
import kotlinx.serialization.json.jsonPrimitive
import okhttp3.OkHttpClient
import java.util.concurrent.TimeUnit

// Thin wrapper around the generated API classes: one set per robot base URL,
// every call on the IO dispatcher, firmware error bodies ({"error": "..."}) as messages.
class RobotClient {
    // The ESP32 answers one request at a time and a serial timeout takes up to ~12 s
    private val http = OkHttpClient.Builder()
        .connectTimeout(4, TimeUnit.SECONDS)
        .readTimeout(20, TimeUnit.SECONDS)
        .build()

    class Apis(baseUrl: String, http: OkHttpClient) {
        val actions = ActionsApi(baseUrl, http)
        val sensors = SensorsApi(baseUrl, http)
        val system = SystemApi(baseUrl, http)
        val firmware = FirmwareApi(baseUrl, http)
        val guided = GuidedApi(baseUrl, http)
    }

    @Volatile
    private var current: Pair<String, Apis>? = null

    fun apis(baseUrl: String): Apis {
        current?.let { (url, apis) -> if (url == baseUrl) return apis }
        return Apis(baseUrl, http).also { current = baseUrl to it }
    }

    suspend fun <T> call(baseUrl: String, block: Apis.() -> T): T = withContext(Dispatchers.IO) {
        try {
            apis(baseUrl).block()
        } catch (e: ClientException) {
            throw RobotException(errorMessage((e.response as? ClientError<*>)?.body) ?: e.message, e.statusCode)
        } catch (e: ServerException) {
            throw RobotException(errorMessage((e.response as? ServerError<*>)?.body) ?: e.message, e.statusCode)
        }
    }

    // Is there an OpenNeato bridge at this address? Used by discovery and the manual address field.
    suspend fun probe(baseUrl: String): FirmwareVersion? = runCatching {
        withContext(Dispatchers.IO) {
            val quick = http.newBuilder().readTimeout(4, TimeUnit.SECONDS).build()
            FirmwareApi(baseUrl, quick).apiFirmwareVersionGet()
        }
    }.getOrNull()?.takeIf { it.name == "OpenNeato" }

    private fun errorMessage(body: Any?): String? {
        val text = body as? String ?: return null
        return runCatching { Json.parseToJsonElement(text).jsonObject["error"]?.jsonPrimitive?.content }.getOrNull()
            ?: text.takeIf { it.isNotBlank() && it.length < 200 }
    }

    companion object {
        // "192.168.1.42", "neato.local:80/" -> "http://192.168.1.42", "http://neato.local:80"
        fun normalizeUrl(input: String): String {
            var s = input.trim().trimEnd('/')
            if (!s.startsWith("http://") && !s.startsWith("https://")) s = "http://$s"
            return s
        }
    }
}

class RobotException(message: String?, val status: Int) : Exception(message)

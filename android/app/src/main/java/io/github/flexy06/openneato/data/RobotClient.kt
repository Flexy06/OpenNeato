package io.github.flexy06.openneato.data

import android.net.Uri
import io.github.flexy06.openneato.api.apis.ActionsApi
import io.github.flexy06.openneato.api.apis.FirmwareApi
import io.github.flexy06.openneato.api.apis.GuidedApi
import io.github.flexy06.openneato.api.apis.HistoryApi
import io.github.flexy06.openneato.api.apis.LogsApi
import io.github.flexy06.openneato.api.apis.ManualApi
import io.github.flexy06.openneato.api.apis.SensorsApi
import io.github.flexy06.openneato.api.apis.SettingsApi
import io.github.flexy06.openneato.api.apis.SystemApi
import io.github.flexy06.openneato.api.apis.WiFiApi
import io.github.flexy06.openneato.api.infrastructure.ClientError
import io.github.flexy06.openneato.api.infrastructure.ClientException
import io.github.flexy06.openneato.api.infrastructure.ServerError
import io.github.flexy06.openneato.api.infrastructure.ServerException
import io.github.flexy06.openneato.api.models.FirmwareVersion
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.withContext
import kotlinx.serialization.json.Json
import kotlinx.serialization.json.JsonObject
import kotlinx.serialization.json.jsonObject
import kotlinx.serialization.json.jsonPrimitive
import okhttp3.MediaType.Companion.toMediaType
import okhttp3.MultipartBody
import okhttp3.OkHttpClient
import okhttp3.Request
import okhttp3.RequestBody
import okhttp3.RequestBody.Companion.toRequestBody
import okio.BufferedSink
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
        val history = HistoryApi(baseUrl, http)
        val settings = SettingsApi(baseUrl, http)
        val wifi = WiFiApi(baseUrl, http)
        val logs = LogsApi(baseUrl, http)
        val actions = ActionsApi(baseUrl, http)
        val sensors = SensorsApi(baseUrl, http)
        val system = SystemApi(baseUrl, http)
        val firmware = FirmwareApi(baseUrl, http)
        val guided = GuidedApi(baseUrl, http)
        val manual = ManualApi(baseUrl, http)
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

    // Settings as a raw JSON object and partial updates: PUT /api/settings takes only the
    // changed keys (like the web UI), the generated client would send every setting back
    // (needless NVS writes, and it would overwrite changes made meanwhile elsewhere).
    suspend fun settingsJson(baseUrl: String): JsonObject = request(Request.Builder().url("$baseUrl/api/settings").get())

    suspend fun patchSettings(baseUrl: String, patch: JsonObject): JsonObject = request(
        Request.Builder().url("$baseUrl/api/settings").put(patch.toString().toRequestBody("application/json".toMediaType())),
    )

    private suspend fun request(builder: Request.Builder): JsonObject =
        Json.parseToJsonElement(requestText(builder)).jsonObject

    private suspend fun requestText(builder: Request.Builder): String = withContext(Dispatchers.IO) {
        http.newCall(builder.build()).execute().use { res ->
            val text = res.body?.string().orEmpty()
            if (!res.isSuccessful) throw RobotException(errorMessage(text) ?: "HTTP ${res.code}", res.code)
            text
        }
    }

    // A cleaning session as stored (JSONL, decompressed by the firmware)
    suspend fun historySession(baseUrl: String, name: String): String =
        requestText(Request.Builder().url("$baseUrl/api/history/${Uri.encode(name)}").get())

    // A log file as text (decompressed by the firmware)
    suspend fun logText(baseUrl: String, name: String): String =
        requestText(Request.Builder().url("$baseUrl/api/logs/${Uri.encode(name)}").get())

    // OTA update: multipart "file" + MD5 for the bridge to verify; the bridge writes
    // the flash before it answers, so allow for that. Progress 0..1 of the upload.
    suspend fun uploadFirmware(baseUrl: String, bin: ByteArray, md5: String, progress: (Float) -> Unit) {
        val file = object : RequestBody() {
            override fun contentType() = "application/octet-stream".toMediaType()
            override fun contentLength() = bin.size.toLong()
            override fun writeTo(sink: BufferedSink) {
                var sent = 0
                while (sent < bin.size) {
                    val n = minOf(8192, bin.size - sent)
                    sink.write(bin, sent, n)
                    sent += n
                    progress(sent.toFloat() / bin.size)
                }
            }
        }
        val body = MultipartBody.Builder()
            .setType(MultipartBody.FORM)
            .addFormDataPart("file", "firmware.bin", file)
            .build()
        val slow = http.newBuilder().writeTimeout(120, TimeUnit.SECONDS).readTimeout(120, TimeUnit.SECONDS).build()
        withContext(Dispatchers.IO) {
            slow.newCall(Request.Builder().url("$baseUrl/api/firmware/update?hash=$md5").post(body).build()).execute()
                .use { res ->
                    val text = res.body?.string().orEmpty()
                    if (!res.isSuccessful) throw RobotException(errorMessage(text) ?: "HTTP ${res.code}", res.code)
                }
        }
    }

    // Upload a session file exported earlier (multipart field "file", like the web UI)
    suspend fun importHistory(baseUrl: String, fileName: String, bytes: ByteArray) {
        val body = MultipartBody.Builder()
            .setType(MultipartBody.FORM)
            .addFormDataPart("file", fileName, bytes.toRequestBody("application/x-ndjson".toMediaType()))
            .build()
        requestText(Request.Builder().url("$baseUrl/api/history/import").post(body))
    }

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

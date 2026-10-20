package io.github.flexy06.openneato.data

import kotlinx.coroutines.CancellationException
import kotlinx.coroutines.delay
import kotlinx.coroutines.flow.Flow
import kotlinx.coroutines.flow.flow

// Result of a polled endpoint, like usePolling() in the web UI: the last good value
// survives a failed poll so the screen does not flicker on a single timeout.
data class Poll<T>(val data: T? = null, val error: String? = null)

fun <T> poll(intervalMs: Long, fetch: suspend () -> T): Flow<Poll<T>> = flow {
    var last: T? = null
    while (true) {
        val error = try {
            last = fetch()
            null
        } catch (e: CancellationException) {
            throw e
        } catch (e: Exception) {
            e.message ?: e.javaClass.simpleName
        }
        emit(Poll(last, error))
        delay(intervalMs)
    }
}

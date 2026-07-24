package mutiny.transport.websocket

import io.ktor.server.application.install
import io.ktor.server.cio.CIO
import io.ktor.server.engine.EmbeddedServer
import io.ktor.server.engine.embeddedServer
import io.ktor.server.routing.routing
import io.ktor.server.websocket.DefaultWebSocketServerSession
import io.ktor.server.websocket.WebSockets
import io.ktor.server.websocket.webSocket
import io.ktor.websocket.Frame
import io.ktor.websocket.readText
import kotlinx.coroutines.delay
import kotlinx.coroutines.isActive
import kotlinx.coroutines.launch
import kotlinx.serialization.json.Json
import mutiny.relay.NetworkProvider
import mutiny.relay.RobotState
import java.util.concurrent.atomic.AtomicReference

const val DEFAULT_PORT = 5800
const val DEFAULT_HOST = "0.0.0.0"
const val DEFAULT_PUSH_PERIOD_MS = 20L

private const val PATH = "/relay"
private const val STOP_GRACE_MS = 1000L
private const val STOP_TIMEOUT_MS = 2000L

/** Wire codec shared by every connection. */
private val json =
    Json {
        ignoreUnknownKeys = true
        encodeDefaults = false
        classDiscriminator = "type"
    }

/**
 * Immutable description of a WebSocket transport for the relay. Carries the
 * connection to the relay ([networkProvider]) plus listen/push configuration; [engine]
 * is non-null only once [start] has been called. Each connected client sends
 * [ClientMessage]s (actions + a [Subscription]) and receives filtered
 * [ServerMessage.State] snapshots.
 *
 * @param port FRC team-use TCP port (5800–5810).
 */
data class WebSocketRelayServer(
    val networkProvider: NetworkProvider,
    val port: Int = DEFAULT_PORT,
    val host: String = DEFAULT_HOST,
    val pushPeriodMs: Long = DEFAULT_PUSH_PERIOD_MS,
    val engine: EmbeddedServer<*, *>? = null,
)

/**
 * Start [server]: begin listening and return a started copy carrying the engine
 * handle. Does not mutate [server]; pass the returned value to [stop].
 */
fun start(server: WebSocketRelayServer): WebSocketRelayServer {
    val engine =
        embeddedServer(CIO, port = server.port, host = server.host) {
            install(WebSockets)
            routing {
                webSocket(PATH) { handleSession(server, this) }
            }
        }.start(wait = false)
    println(
        "[WebSocketRelayServer] listening on ws://${server.host}:${server.port}$PATH (push every ${server.pushPeriodMs}ms)",
    )
    return server.copy(engine = engine)
}

/** Stop a started [server] (no-op if not running). */
fun stop(server: WebSocketRelayServer) {
    server.engine?.stop(STOP_GRACE_MS, STOP_TIMEOUT_MS)
}

private suspend fun handleSession(
    server: WebSocketRelayServer,
    session: DefaultWebSocketServerSession,
) {
    val subscription = AtomicReference(Subscription.ALL)
    val sender =
        session.launch {
            while (isActive) {
                try {
                    val state = filter(server.networkProvider.latest, subscription.get())
                    session.send(
                        Frame.Text(json.encodeToString(ServerMessage.serializer(), ServerMessage.State(state))),
                    )
                } catch (e: Throwable) {
                    // Client gone or send failed; let the receive loop tear us down.
                    break
                }
                delay(server.pushPeriodMs)
            }
        }
    try {
        for (frame in session.incoming) {
            if (frame is Frame.Text) handleText(server, frame.readText(), subscription)
        }
    } finally {
        sender.cancel()
    }
}

private fun handleText(
    server: WebSocketRelayServer,
    text: String,
    subscription: AtomicReference<Subscription>,
) {
    val message =
        try {
            json.decodeFromString(ClientMessage.serializer(), text)
        } catch (e: Exception) {
            println("[WebSocketRelayServer] dropped malformed message: ${e.message}")
            return
        }
    when (message) {
        is ClientMessage.Action -> server.networkProvider.submit(message.action)
        is ClientMessage.Actions -> server.networkProvider.submitAll(message.actions)
        is ClientMessage.Subscribe -> subscription.set(message.subscription)
    }
}

/** Keep only the state this [subscription] asked for; status + errors always pass through. */
private fun filter(
    state: RobotState,
    subscription: Subscription,
): RobotState =
    state.copy(
        pwmSpeed = keepKeys(state.pwmSpeed, subscription.pwm),
        pwmPosition = keepKeys(state.pwmPosition, subscription.pwm),
        digitalInputs = keepKeys(state.digitalInputs, subscription.digitalInputs),
        digitalOutputs = keepKeys(state.digitalOutputs, subscription.digitalOutputs),
        analogInputs = keepKeys(state.analogInputs, subscription.analogInputs),
        analogOutputs = keepKeys(state.analogOutputs, subscription.analogOutputs),
        canFrames = keepKeys(state.canFrames, subscription.canFrames),
    )

private fun <K, V> keepKeys(
    map: Map<K, V>,
    keys: Set<K>?,
): Map<K, V> = if (keys == null) map else map.filterKeys { it in keys }

package mutiny.relay

import kotlinx.coroutines.CompletableDeferred

/**
 * Implemented by the relay and handed to a network transport (websocket /
 * protobuf / zmq / ...) so that transport can push [RobotAction]s into the relay
 * and pull [RobotState] out, with no knowledge of WPILib.
 *
 * The provider is connection-aware: every WebSocket connection opens a
 * [SessionId] via [openSession] and closes it via [closeSession] (which releases
 * every device that session registered). Registration is an RPC —
 * [submitRegister] returns a [CompletableDeferred] that completes with a
 * [RegisterOutcome] (an opaque [Token] on success, or a structured
 * [ApplyError]); the transport awaits it to reply to the issuing client.
 * Operates remain fire-and-forget via [submitAction].
 *
 * All submit / open / close methods are safe to call from any thread; work is
 * drained and executed on the robot periodic thread. [latest] is refreshed
 * every cycle and safe to read from any thread. The relay ([TimedRobotRelay])
 * is the canonical implementation.
 */
interface NetworkProvider {
    /** Mint and register a new session; its devices are released by [closeSession]. */
    fun openSession(): SessionId

    /** Release every device owned by [id] and drop the session. */
    fun closeSession(id: SessionId)

    /** Enqueue an operate / deregister action for the periodic loop. */
    fun submitAction(
        sessionId: SessionId,
        action: RobotAction,
    )

    /**
     * Enqueue a register action; the returned deferred completes on the periodic
     * loop with the minted token (or a structured error) for the transport to
     * relay back to the issuing client.
     */
    fun submitRegister(
        sessionId: SessionId,
        action: RobotAction,
    ): CompletableDeferred<RegisterOutcome>

    val latest: RobotState
}

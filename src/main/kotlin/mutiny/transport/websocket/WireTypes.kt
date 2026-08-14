package mutiny.transport.websocket

import kotlinx.serialization.SerialName
import kotlinx.serialization.Serializable
import mutiny.relay.RegisterOutcome
import mutiny.relay.RobotAction
import mutiny.relay.RobotState

/**
 * Per-client interest filter for the state stream. Each field selects the device
 * ids to include; `null` (the default) means "include every registered device",
 * while an explicit set means "include only these ids".
 *
 * `pwm` governs both `pwmSpeed` and `pwmPosition`. Status fields
 * (enabled/mode/timestamp/sequence) and `errors` are always sent. On the wire
 * this is JSON, decoded by the client.
 */
@Serializable
data class Subscription(
    val pwm: Set<Int>? = null,
    val digitalInputs: Set<Int>? = null,
    val digitalOutputs: Set<Int>? = null,
    val analogInputs: Set<Int>? = null,
    val analogOutputs: Set<Int>? = null,
    val canFrames: Set<String>? = null,
    val sparkMax: Set<Int>? = null,
) {
    companion object {
        /** Include everything registered — the default for a new connection. */
        val ALL = Subscription()
    }
}

/** Inbound message from a client. Tagged by `type` for the client's decoder. */
@Serializable
sealed interface ClientMessage {
    /** Carry out a single operate / deregister / CAN / SparkMax [RobotAction]. */
    @Serializable
    @SerialName("action")
    data class Action(val action: RobotAction) : ClientMessage

    /** Carry out a batch of [RobotAction]s in order. */
    @Serializable
    @SerialName("actions")
    data class Actions(val actions: List<RobotAction>) : ClientMessage

    /**
     * Register a device as an RPC: the server replies with a matching
     * [ServerMessage.RegisterResult] echoing [requestId] and carrying either the
     * minted token or a structured error.
     */
    @Serializable
    @SerialName("register")
    data class Register(
        val requestId: String,
        val action: RobotAction,
    ) : ClientMessage

    /** Replace this connection's [Subscription]. */
    @Serializable
    @SerialName("subscribe")
    data class Subscribe(val subscription: Subscription) : ClientMessage
}

/** Outbound message to a client. Tagged by `type` for the client's decoder. */
@Serializable
sealed interface ServerMessage {
    /** A filtered [RobotState] snapshot. */
    @Serializable
    @SerialName("state")
    data class State(val state: RobotState) : ServerMessage

    /**
     * Reply to a [ClientMessage.Register], echoing the client's [requestId] and
     * carrying the register [outcome] (minted token or structured error). Sent
     * only to the session that issued the register — never broadcast.
     */
    @Serializable
    @SerialName("registerResult")
    data class RegisterResult(
        val requestId: String,
        val outcome: RegisterOutcome,
    ) : ServerMessage
}

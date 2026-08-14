package mutiny.relay

import kotlinx.serialization.SerialName
import kotlinx.serialization.Serializable

/**
 * An opaque, connection-scoped handle minted by the relay on a successful
 * registration. The token's *type* (which kind entry it keys) encodes the
 * device family + direction/mode; subsequent operate actions carry the token
 * instead of a port number, giving compile-time / kind-safety on both sides.
 *
 * A token is valid only for the lifetime of the [SessionId] that registered it;
 * it does not survive relay process shutdown or a WebSocket reconnect.
 */
@Serializable
data class Token(val value: String) {
    companion object {
        fun random(): Token = Token(java.util.UUID.randomUUID().toString())
    }
}

/**
 * Server-side identity of a single WebSocket session. Registrations are owned
 * by the session that issued them; on disconnect the relay releases every
 * device the session minted a token for, so reconnect is a clean slate.
 */
@Serializable
data class SessionId(val value: String) {
    companion object {
        fun random(): SessionId = SessionId(java.util.UUID.randomUUID().toString())
    }
}

/**
 * Declared mode of a PWM registration. Determines which operate actions are
 * compatible with the resulting token: [MOTOR] accepts `pwm.setSpeed`,
 * [SERVO] accepts `pwm.setPosition`.
 */
@Serializable
enum class PwmMode {
    MOTOR,
    SERVO,
}

/**
 * Outcome of a register RPC, returned via [ServerMessage.RegisterResult].
 * Allocation is allowed while the robot is disabled, so the disabled gate only
 * applies to operate actions — register never produces a "robot disabled" error.
 */
@Serializable
sealed interface RegisterOutcome {
    @Serializable
    @SerialName("ok")
    data class Ok(val token: Token) : RegisterOutcome

    @Serializable
    @SerialName("error")
    data class Error(val error: ApplyError) : RegisterOutcome
}

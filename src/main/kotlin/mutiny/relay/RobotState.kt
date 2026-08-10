package mutiny.relay
import kotlinx.serialization.SerialName
import kotlinx.serialization.Serializable
import mutiny.relay.Snapshot.CanFrameSnapshot
import mutiny.relay.Snapshot.SparkMaxSnapshot

@Serializable
sealed interface SignalStatus {
    // TODO: Name may be confused with Phoenix6's StatusSignal.
    // TODO: Add more statuses. (i.e. stale)
    @Serializable
    @SerialName("ok")
    data object Ok : SignalStatus

    @Serializable
    @SerialName("error")
    data class Error(
        val message: String,
    ) : SignalStatus
}

/**
 * @param timestampSeconds
 * Monotonic timestamp associated with this signal.
 *
 * The clock domain is vendor/source-specific. Timestamps from different
 * vendors are not necessarily directly comparable.
 */
@Serializable
data class SignalSample(
    val value: Double,
    val timestampSeconds: Double,
    val status: SignalStatus,
)

@Serializable
sealed interface Snapshot {
    /** A single sampled CAN frame, published in the [RobotState] snapshot. */
    @Serializable
    data class CanFrameSnapshot(
        val messageId: Int,
        val apiId: Int,
        /** True when a frame matching (messageId, apiId) has arrived since boot. */
        val valid: Boolean,
        /** Payload bytes, each 0..255. Empty when [valid] is false. */
        val data: List<Int>,
        val length: Int,
        /** FPGA timestamp (microseconds) of the last received frame; 0 when invalid. */
        val fpgaTimestampUs: Long,
    ) : Snapshot

    // TODO: Update REV library to 2026. The use of SignalSamples may seem a little clunky
    //      right now, but it becomes much more natural with CTRE and REV 2026.
    @Serializable
    data class SparkMaxSnapshot(
        val position: SignalSample,
        val velocity: SignalSample,
    ) : Snapshot
}

/** Diagnostic for an action that failed to execute: the exact action plus its structured reason. */
@Serializable
data class ActionError(
    val action: RobotAction,
    val error: ApplyError,
)

/**
 * Immutable, transport-agnostic snapshot of the robot produced once per periodic
 * cycle. A network client reads [RobotState] via [NetworkProvider].
 */
@Serializable
data class RobotState(
    val sequence: Long,
    val timestampSec: Double,
    val enabled: Boolean,
    val autonomous: Boolean,
    val teleop: Boolean,
    val test: Boolean,
    val emergencyStopped: Boolean,
    val analogInputs: Map<Int, Double>,
    val analogOutputs: Map<Int, Double>,
    val digitalInputs: Map<Int, Boolean>,
    val digitalOutputs: Map<Int, Boolean>,
    val pwmSpeed: Map<Int, Double>,
    val pwmPosition: Map<Int, Double>,
    val sparkMaxSnapshots: Map<Int, SparkMaxSnapshot>,
    val canFrames: Map<String, CanFrameSnapshot>,
    val errors: List<ActionError>,
) {
    companion object {
        val EMPTY =
            RobotState(
                sequence = 0L,
                timestampSec = 0.0,
                enabled = false,
                autonomous = false,
                teleop = false,
                test = false,
                emergencyStopped = false,
                analogInputs = emptyMap(),
                analogOutputs = emptyMap(),
                digitalInputs = emptyMap(),
                digitalOutputs = emptyMap(),
                pwmSpeed = emptyMap(),
                pwmPosition = emptyMap(),
                sparkMaxSnapshots = emptyMap(),
                canFrames = emptyMap(),
                errors = emptyList(),
            )
    }
}

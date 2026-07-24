package mutiny.relay

import kotlinx.serialization.SerialName
import kotlinx.serialization.Serializable

/** Which family of WPILib device an [ApplyError] concerns. */
@Serializable
enum class DeviceKind {
    PWM,
    DIGITAL_INPUT,
    DIGITAL_OUTPUT,
    ANALOG_INPUT,
    ANALOG_OUTPUT,
    CAN,
}

/**
 * Structured failure of a single [RobotAction], reported as a value
 * (never thrown) so a network client can produce a precise, machine-readable
 * response. Each variant names the [deviceKind] and [id] (port / channel / CAN
 * message id) it concerns and, where relevant, the offending input.
 */
@Serializable
sealed interface ApplyError {
    val deviceKind: DeviceKind

    /** Port / channel / CAN message id involved. */
    val id: Int

    /** The action referenced a device that has not been registered. */
    @Serializable
    @SerialName("notRegistered")
    data class NotRegistered(
        override val deviceKind: DeviceKind,
        override val id: Int,
    ) : ApplyError

    /** A scalar input fell outside its valid range. */
    @Serializable
    @SerialName("outOfRange")
    data class OutOfRange(
        override val deviceKind: DeviceKind,
        override val id: Int,
        /** Which input field was bad, e.g. "speed" or "voltage". */
        val field: String,
        val value: Double,
        val min: Double,
        val max: Double,
    ) : ApplyError

    /** A CAN payload byte was not in 0..255. */
    @Serializable
    @SerialName("invalidCanByte")
    data class InvalidCanByte(
        override val id: Int,
        val index: Int,
        val byte: Int,
        override val deviceKind: DeviceKind = DeviceKind.CAN,
    ) : ApplyError

    /** WPILib refused to allocate the channel (out of range or already in use). */
    @Serializable
    @SerialName("allocationFailed")
    data class AllocationFailed(
        override val deviceKind: DeviceKind,
        override val id: Int,
        /** WPILib's explanation of why allocation failed. */
        val detail: String,
    ) : ApplyError

    /** A WPILib operation failed at runtime (a read/write threw). */
    @Serializable
    @SerialName("hardwareFault")
    data class HardwareFault(
        override val deviceKind: DeviceKind,
        override val id: Int,
        val detail: String,
    ) : ApplyError
}

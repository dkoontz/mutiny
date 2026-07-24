package mutiny.relay

import kotlinx.serialization.SerialName
import kotlinx.serialization.Serializable

/**
 * A single command the external robot application asks the relay to carry out
 * during a [edu.wpi.first.wpilibj.TimedRobot.robotPeriodic] window. All variants
 * are [Serializable] so any transport (websocket/protobuf/zmq/...) can reuse the
 * same model with kotlinx.serialization.
 */
@Serializable
sealed interface RobotAction {
    // ------------------------------------------------------------------ PWM
    @Serializable
    @SerialName("pwm.register")
    data class RegisterPwm(
        val port: Int,
    ) : RobotAction

    @Serializable
    @SerialName("pwm.deregister")
    data class DeregisterPwm(
        val port: Int,
    ) : RobotAction

    /** @param speed -1.0 (full reverse) - 1.0 (full forward). */
    @Serializable
    @SerialName("pwm.setSpeed")
    data class SetPwmSpeed(
        val port: Int,
        val speed: Double,
    ) : RobotAction

    /** @param position 0.0 -- 1.0 (servo-style). */
    @Serializable
    @SerialName("pwm.setPosition")
    data class SetPwmPosition(
        val port: Int,
        val position: Double,
    ) : RobotAction

    @Serializable
    @SerialName("pwm.disable")
    data class DisablePwm(
        val port: Int,
    ) : RobotAction

    // ----------------------------------------------------------- Digital IO
    @Serializable
    @SerialName("dio.registerInput")
    data class RegisterDigitalInput(
        val channel: Int,
    ) : RobotAction

    @Serializable
    @SerialName("dio.registerOutput")
    data class RegisterDigitalOutput(
        val channel: Int,
    ) : RobotAction

    @Serializable
    @SerialName("dio.deregisterInput")
    data class DeregisterDigitalInput(
        val channel: Int,
    ) : RobotAction

    @Serializable
    @SerialName("dio.deregisterOutput")
    data class DeregisterDigitalOutput(
        val channel: Int,
    ) : RobotAction

    @Serializable
    @SerialName("dio.setOutput")
    data class SetDigitalOutput(
        val channel: Int,
        val value: Boolean,
    ) : RobotAction

    // ----------------------------------------------------------- Analog IO
    @Serializable
    @SerialName("aio.registerInput")
    data class RegisterAnalogInput(
        val channel: Int,
    ) : RobotAction

    @Serializable
    @SerialName("aio.registerOutput")
    data class RegisterAnalogOutput(
        val channel: Int,
    ) : RobotAction

    @Serializable
    @SerialName("aio.deregisterInput")
    data class DeregisterAnalogInput(
        val channel: Int,
    ) : RobotAction

    @Serializable
    @SerialName("aio.deregisterOutput")
    data class DeregisterAnalogOutput(
        val channel: Int,
    ) : RobotAction

    @Serializable
    @SerialName("aio.setOutput")
    data class SetAnalogOutput(
        val channel: Int,
        val voltage: Double,
    ) : RobotAction

    // --------------------------------------------------------------- CAN

    /** Begin sampling the latest frame for (messageId, apiId) into the snapshot. */
    @Serializable
    @SerialName("can.registerRx")
    data class RegisterCanRx(
        val messageId: Int,
        val apiId: Int,
    ) : RobotAction

    @Serializable
    @SerialName("can.deregisterRx")
    data class DeregisterCanRx(
        val messageId: Int,
        val apiId: Int,
    ) : RobotAction

    /**
     * Write a CAN frame. [data] bytes are each 0-255; values outside that range
     * are masked to their low 8 bits.
     */
    @Serializable
    @SerialName("can.write")
    data class CanWrite(
        val messageId: Int,
        val apiId: Int,
        val data: List<Int>,
    ) : RobotAction
}

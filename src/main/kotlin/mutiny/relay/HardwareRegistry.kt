package mutiny.relay

import edu.wpi.first.hal.CANData
import edu.wpi.first.wpilibj.AnalogInput
import edu.wpi.first.wpilibj.AnalogOutput
import edu.wpi.first.wpilibj.CAN
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.DigitalOutput
import edu.wpi.first.wpilibj.PWM
import mutiny.relay.ApplyError.AllocationFailed
import mutiny.relay.ApplyError.HardwareFault
import mutiny.relay.ApplyError.InvalidCanByte
import mutiny.relay.ApplyError.NotRegistered
import mutiny.relay.ApplyError.OutOfRange
import mutiny.relay.DeviceKind.ANALOG_INPUT
import mutiny.relay.DeviceKind.ANALOG_OUTPUT
import mutiny.relay.DeviceKind.CAN
import mutiny.relay.DeviceKind.DIGITAL_INPUT
import mutiny.relay.DeviceKind.DIGITAL_OUTPUT
import mutiny.relay.DeviceKind.PWM

/** roboRIO analog output voltage range (per WPILib AnalogOutput spec). */
private const val ANALOG_OUT_MIN = 0.0
private const val ANALOG_OUT_MAX = 5.0

/**
 * Outcome of executing a single [RobotAction] against a [HardwareRegistry].
 * Reported as a value; [execute] never throws.
 */
sealed interface ApplyOutcome {
    data object Applied : ApplyOutcome

    data class Failed(val error: ApplyError) : ApplyOutcome
}

/**
 * Holds the live WPILib handles the relay has allocated. These are native,
 * mutable resources, so this is intentionally a plain class — not a data class,
 * since its generated [copy] would alias handles and risk a double-free — and it
 * is mutated only by the top-level [execute] function on the robot periodic thread.
 */
class HardwareRegistry {
    internal val pwm = HashMap<Int, PWM>()
    internal val digitalInputs = HashMap<Int, DigitalInput>()
    internal val digitalOutputs = HashMap<Int, DigitalOutput>()
    internal val analogInputs = HashMap<Int, AnalogInput>()
    internal val analogOutputs = HashMap<Int, AnalogOutput>()
    internal val canDevices = HashMap<Int, CAN>()
    internal val canRxSubscriptions = ArrayList<Pair<Int, Int>>()
    internal val canBuffer = CANData()
}

/**
 * Execute one [action] against [registry], mutating it in place. All inputs are
 * validated and every WPILib failure is classified into a structured
 * [ApplyError]; this function never throws.
 */
fun execute(
    registry: HardwareRegistry,
    action: RobotAction,
): ApplyOutcome =
    when (action) {
        // PWM
        is RobotAction.RegisterPwm ->
            register(registry.pwm, action.port, PWM) { PWM(action.port) }
        is RobotAction.DeregisterPwm ->
            release(registry.pwm, action.port, PWM)
        is RobotAction.SetPwmSpeed ->
            if (action.speed !in -1.0..1.0) {
                ApplyOutcome.Failed(OutOfRange(PWM, action.port, "speed", action.speed, -1.0, 1.0))
            } else {
                operate(registry.pwm, action.port, PWM) { it.setSpeed(action.speed) }
            }
        is RobotAction.SetPwmPosition ->
            if (action.position !in 0.0..1.0) {
                ApplyOutcome.Failed(OutOfRange(PWM, action.port, "position", action.position, 0.0, 1.0))
            } else {
                operate(registry.pwm, action.port, PWM) { it.setPosition(action.position) }
            }
        is RobotAction.DisablePwm ->
            operate(registry.pwm, action.port, PWM) { it.setDisabled() }

        // Digital IO
        is RobotAction.RegisterDigitalInput ->
            register(registry.digitalInputs, action.channel, DIGITAL_INPUT) { DigitalInput(action.channel) }
        is RobotAction.RegisterDigitalOutput ->
            register(registry.digitalOutputs, action.channel, DIGITAL_OUTPUT) { DigitalOutput(action.channel) }
        is RobotAction.DeregisterDigitalInput ->
            release(registry.digitalInputs, action.channel, DIGITAL_INPUT)
        is RobotAction.DeregisterDigitalOutput ->
            release(registry.digitalOutputs, action.channel, DIGITAL_OUTPUT)
        is RobotAction.SetDigitalOutput ->
            operate(registry.digitalOutputs, action.channel, DIGITAL_OUTPUT) { it.set(action.value) }

        // Analog IO
        is RobotAction.RegisterAnalogInput ->
            register(registry.analogInputs, action.channel, ANALOG_INPUT) { AnalogInput(action.channel) }
        is RobotAction.RegisterAnalogOutput ->
            register(registry.analogOutputs, action.channel, ANALOG_OUTPUT) { AnalogOutput(action.channel) }
        is RobotAction.DeregisterAnalogInput ->
            release(registry.analogInputs, action.channel, ANALOG_INPUT)
        is RobotAction.DeregisterAnalogOutput ->
            release(registry.analogOutputs, action.channel, ANALOG_OUTPUT)
        is RobotAction.SetAnalogOutput ->
            if (action.voltage !in ANALOG_OUT_MIN..ANALOG_OUT_MAX) {
                ApplyOutcome.Failed(
                    OutOfRange(
                        ANALOG_OUTPUT,
                        action.channel,
                        "voltage",
                        action.voltage,
                        ANALOG_OUT_MIN,
                        ANALOG_OUT_MAX,
                    ),
                )
            } else {
                operate(registry.analogOutputs, action.channel, ANALOG_OUTPUT) { it.setVoltage(action.voltage) }
            }

        // CAN
        is RobotAction.RegisterCanRx -> {
            val outcome = register(registry.canDevices, action.messageId, CAN) { CAN(action.messageId) }
            if (outcome is ApplyOutcome.Applied) {
                registry.canRxSubscriptions.add(action.messageId to action.apiId)
            }
            outcome
        }
        is RobotAction.DeregisterCanRx -> {
            registry.canRxSubscriptions.remove(action.messageId to action.apiId)
            ApplyOutcome.Applied
        }
        is RobotAction.CanWrite -> {
            val bad = action.data.withIndex().firstOrNull { it.value !in 0..255 }
            if (bad != null) {
                ApplyOutcome.Failed(InvalidCanByte(action.messageId, bad.index, bad.value))
            } else {
                writeCan(registry, action.messageId, action.apiId, action.data.toWireBytes())
            }
        }
    }

/** Build an immutable snapshot of every input and commanded output in [registry]. */
fun sample(
    registry: HardwareRegistry,
    sequence: Long,
    timestampSec: Double,
    enabled: Boolean,
    autonomous: Boolean,
    teleop: Boolean,
    test: Boolean,
    emergencyStopped: Boolean,
    errors: List<ActionError>,
): RobotState {
    val analogIn = registry.analogInputs.mapValues { it.value.voltage }
    val analogOut = registry.analogOutputs.mapValues { it.value.voltage }
    val digitalIn = registry.digitalInputs.mapValues { it.value.get() }
    val digitalOut = registry.digitalOutputs.mapValues { it.value.get() }
    val pwmSpeed = registry.pwm.mapValues { it.value.speed }
    val pwmPosition = registry.pwm.mapValues { it.value.position }
    val canFrames = HashMap<String, CanFrameSnapshot>()
    for ((messageId, apiId) in registry.canRxSubscriptions) {
        val valid = canFor(registry, messageId).readPacketLatest(apiId, registry.canBuffer)
        val payload =
            if (valid) {
                registry.canBuffer.data.copyOf(registry.canBuffer.length).map { it.toInt() and 0xFF }
            } else {
                emptyList()
            }
        canFrames["$messageId:$apiId"] =
            CanFrameSnapshot(
                messageId = messageId,
                apiId = apiId,
                valid = valid,
                data = payload,
                length = if (valid) registry.canBuffer.length else 0,
                fpgaTimestampUs = if (valid) registry.canBuffer.timestamp else 0L,
            )
    }
    return RobotState(
        sequence = sequence,
        timestampSec = timestampSec,
        enabled = enabled,
        autonomous = autonomous,
        teleop = teleop,
        test = test,
        emergencyStopped = emergencyStopped,
        analogInputs = analogIn,
        analogOutputs = analogOut,
        digitalInputs = digitalIn,
        digitalOutputs = digitalOut,
        pwmSpeed = pwmSpeed,
        pwmPosition = pwmPosition,
        canFrames = canFrames,
        errors = errors,
    )
}

/** Release every allocated WPILib handle in [registry]. */
fun close(registry: HardwareRegistry) {
    registry.pwm.values.forEach { it.close() }
    registry.digitalInputs.values.forEach { it.close() }
    registry.digitalOutputs.values.forEach { it.close() }
    registry.analogInputs.values.forEach { it.close() }
    registry.analogOutputs.values.forEach { it.close() }
    registry.canDevices.values.forEach { it.close() }
    registry.pwm.clear()
    registry.digitalInputs.clear()
    registry.digitalOutputs.clear()
    registry.analogInputs.clear()
    registry.analogOutputs.clear()
    registry.canDevices.clear()
    registry.canRxSubscriptions.clear()
}

private fun canFor(
    registry: HardwareRegistry,
    messageId: Int,
): CAN = registry.canDevices.getOrPut(messageId) { CAN(messageId) }

/** Allocate (idempotently) a device, classifying a WPILib throw as [AllocationFailed]. */
private inline fun <T> register(
    map: MutableMap<Int, T>,
    id: Int,
    deviceKind: DeviceKind,
    factory: () -> T,
): ApplyOutcome =
    try {
        map.getOrPut(id) { factory() }
        ApplyOutcome.Applied
    } catch (e: Exception) {
        ApplyOutcome.Failed(AllocationFailed(deviceKind, id, e.describe()))
    }

/** Run [block] on a registered device, or report it [NotRegistered] / [HardwareFault]. */
private inline fun <T> operate(
    map: Map<Int, T>,
    id: Int,
    deviceKind: DeviceKind,
    block: (T) -> Unit,
): ApplyOutcome {
    val device = map[id] ?: return ApplyOutcome.Failed(NotRegistered(deviceKind, id))
    return try {
        block(device)
        ApplyOutcome.Applied
    } catch (e: Exception) {
        ApplyOutcome.Failed(HardwareFault(deviceKind, id, e.describe()))
    }
}

/** Remove and close a device if present (idempotent). */
private fun <T : AutoCloseable> release(
    map: MutableMap<Int, T>,
    id: Int,
    deviceKind: DeviceKind,
): ApplyOutcome =
    try {
        map.remove(id)?.close()
        ApplyOutcome.Applied
    } catch (e: Exception) {
        ApplyOutcome.Failed(HardwareFault(deviceKind, id, e.describe()))
    }

/** Open (if needed) and write a CAN frame, distinguishing allocation from write faults. */
private fun writeCan(
    registry: HardwareRegistry,
    messageId: Int,
    apiId: Int,
    data: ByteArray,
): ApplyOutcome {
    val can =
        try {
            registry.canDevices.getOrPut(messageId) { CAN(messageId) }
        } catch (e: Exception) {
            return ApplyOutcome.Failed(AllocationFailed(CAN, messageId, e.describe()))
        }
    return try {
        can.writePacket(data, apiId)
        ApplyOutcome.Applied
    } catch (e: Exception) {
        ApplyOutcome.Failed(HardwareFault(CAN, messageId, e.describe()))
    }
}

private fun List<Int>.toWireBytes(): ByteArray = ByteArray(size) { this[it].toByte() }

private fun Throwable.describe(): String = message ?: javaClass.simpleName

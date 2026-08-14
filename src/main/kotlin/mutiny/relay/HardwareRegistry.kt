package mutiny.relay

import com.revrobotics.REVLibError
import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.SparkMax
import edu.wpi.first.hal.CANData
import edu.wpi.first.wpilibj.AnalogInput
import edu.wpi.first.wpilibj.AnalogOutput
import edu.wpi.first.wpilibj.CAN
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.DigitalOutput
import edu.wpi.first.wpilibj.PWM
import edu.wpi.first.wpilibj.RobotController
import mutiny.relay.ApplyError.AllocationFailed
import mutiny.relay.ApplyError.DeviceAlreadyRegistered
import mutiny.relay.ApplyError.HardwareFault
import mutiny.relay.ApplyError.InvalidCanByte
import mutiny.relay.ApplyError.NotRegistered
import mutiny.relay.ApplyError.OutOfRange
import mutiny.relay.ApplyError.RobotDisabled
import mutiny.relay.ApplyError.WrongDeviceMode
import mutiny.relay.DeviceKind.ANALOG_INPUT
import mutiny.relay.DeviceKind.ANALOG_OUTPUT
import mutiny.relay.DeviceKind.CAN
import mutiny.relay.DeviceKind.DIGITAL_INPUT
import mutiny.relay.DeviceKind.DIGITAL_OUTPUT
import mutiny.relay.DeviceKind.PWM
import mutiny.relay.DeviceKind.SPARKMAX
import mutiny.relay.PwmMode.MOTOR
import mutiny.relay.PwmMode.SERVO
import mutiny.relay.RegisterOutcome.Error
import mutiny.relay.RegisterOutcome.Ok
import mutiny.relay.Snapshot.CanFrameSnapshot

/** roboRIO analog output voltage range (per WPILib AnalogOutput spec). */
private const val ANALOG_OUT_MIN = 0.0
private const val ANALOG_OUT_MAX = 5.0

/** Valid SparkMax setOutput input range */
private const val SPARKMAX_OUT_MIN = -1.0
private const val SPARKMAX_OUT_MAX = 1.0

/** Sentinel `id` for an error about a token that resolves to no entry. */
private const val UNKNOWN_ID = -1

/**
 * Outcome of executing a single [RobotAction] against a [HardwareRegistry].
 * Reported as a value; [execute] never throws.
 */
sealed interface ApplyOutcome {
    data object Applied : ApplyOutcome

    data class Failed(val error: ApplyError) : ApplyOutcome
}

/** Token-keyed registration of a PWM output, plus its port/mode. */
internal data class PwmEntry(
    val port: Int,
    val mode: PwmMode,
    val handle: PWM,
)

/** Token-keyed registration of a digital input, plus its channel. */
internal data class DigitalInputEntry(
    val channel: Int,
    val handle: DigitalInput,
)

/** Token-keyed registration of a digital output, plus its channel. */
internal data class DigitalOutputEntry(
    val channel: Int,
    val handle: DigitalOutput,
)

/** Token-keyed registration of an analog input, plus its channel. */
internal data class AnalogInputEntry(
    val channel: Int,
    val handle: AnalogInput,
)

/** Token-keyed registration of an analog output, plus its channel. */
internal data class AnalogOutputEntry(
    val channel: Int,
    val handle: AnalogOutput,
)

/**
 * Holds the live WPILib handles the relay has allocated. Phase-1 device families
 * (PWM / DIO / Analog) are token-keyed and session-owned: each registration
 * mints an opaque [Token], recorded under the [SessionId] that issued it, and
 * is released wholesale on disconnect via [releaseSession]. CAN and SparkMax
 * remain on the legacy idempotent, port/id-keyed path during Phase 1.
 *
 * Native, mutable resources live here, so this is intentionally a plain class
 * (not a data class — its `copy` would alias handles and risk a double-free),
 * mutated only by the top-level [register] / [execute] / [releaseSession] /
 * [close] functions on the robot periodic thread.
 */
class HardwareRegistry {
    internal val pwm = HashMap<Token, PwmEntry>()
    internal val digitalInputs = HashMap<Token, DigitalInputEntry>()
    internal val digitalOutputs = HashMap<Token, DigitalOutputEntry>()
    internal val analogInputs = HashMap<Token, AnalogInputEntry>()
    internal val analogOutputs = HashMap<Token, AnalogOutputEntry>()

    internal val pwmByPort = HashMap<Int, Token>()
    internal val digitalInputsByChannel = HashMap<Int, Token>()
    internal val digitalOutputsByChannel = HashMap<Int, Token>()
    internal val analogInputsByChannel = HashMap<Int, Token>()
    internal val analogOutputsByChannel = HashMap<Int, Token>()

    internal val sessionTokens = HashMap<SessionId, MutableSet<Token>>()

    // CAN + SparkMax stay port/id-based during Phase 1 (mixed model).
    internal val canDevices = HashMap<Int, CAN>()
    internal val canRxSubscriptions = ArrayList<Pair<Int, Int>>()
    internal val canBuffer = CANData()
    internal val sparkMaxDevices = HashMap<Int, SparkMax>()
}

/**
 * Allocate exclusively, mint a token, and record session ownership. Allocation
 * is **not** gated by the disabled state — it is safe to register a device
 * while the DS has the robot disabled (the HAL neutralizes outputs regardless).
 *
 * A second registration of a port/channel already held returns
 * [DeviceAlreadyRegistered]; a WPILib allocation throw returns [AllocationFailed].
 * CAN and SparkMax registration does **not** flow through here (they remain on
 * the legacy idempotent [execute] path during Phase 1).
 */
fun register(
    registry: HardwareRegistry,
    session: SessionId,
    action: RobotAction,
): RegisterOutcome =
    when (action) {
        is RobotAction.RegisterPwm -> {
            if (registry.pwmByPort.containsKey(action.port)) {
                Error(DeviceAlreadyRegistered(PWM, action.port))
            } else {
                val handle =
                    try {
                        PWM(action.port)
                    } catch (e: Exception) {
                        return Error(AllocationFailed(PWM, action.port, e.describe()))
                    }
                installToken(
                    registry,
                    session,
                    action.port,
                    PwmEntry(action.port, action.mode, handle),
                    registry.pwm,
                    registry.pwmByPort,
                )
            }
        }

        is RobotAction.RegisterDigitalInput -> {
            if (registry.digitalInputsByChannel.containsKey(action.channel)) {
                Error(DeviceAlreadyRegistered(DIGITAL_INPUT, action.channel))
            } else {
                val handle =
                    try {
                        DigitalInput(action.channel)
                    } catch (e: Exception) {
                        return Error(AllocationFailed(DIGITAL_INPUT, action.channel, e.describe()))
                    }
                installToken(
                    registry,
                    session,
                    action.channel,
                    DigitalInputEntry(action.channel, handle),
                    registry.digitalInputs,
                    registry.digitalInputsByChannel,
                )
            }
        }

        is RobotAction.RegisterDigitalOutput -> {
            if (registry.digitalOutputsByChannel.containsKey(action.channel)) {
                Error(DeviceAlreadyRegistered(DIGITAL_OUTPUT, action.channel))
            } else {
                val handle =
                    try {
                        DigitalOutput(action.channel)
                    } catch (e: Exception) {
                        return Error(AllocationFailed(DIGITAL_OUTPUT, action.channel, e.describe()))
                    }
                installToken(
                    registry,
                    session,
                    action.channel,
                    DigitalOutputEntry(action.channel, handle),
                    registry.digitalOutputs,
                    registry.digitalOutputsByChannel,
                )
            }
        }

        is RobotAction.RegisterAnalogInput -> {
            if (registry.analogInputsByChannel.containsKey(action.channel)) {
                Error(DeviceAlreadyRegistered(ANALOG_INPUT, action.channel))
            } else {
                val handle =
                    try {
                        AnalogInput(action.channel)
                    } catch (e: Exception) {
                        return Error(AllocationFailed(ANALOG_INPUT, action.channel, e.describe()))
                    }
                installToken(
                    registry,
                    session,
                    action.channel,
                    AnalogInputEntry(action.channel, handle),
                    registry.analogInputs,
                    registry.analogInputsByChannel,
                )
            }
        }

        is RobotAction.RegisterAnalogOutput -> {
            if (registry.analogOutputsByChannel.containsKey(action.channel)) {
                Error(DeviceAlreadyRegistered(ANALOG_OUTPUT, action.channel))
            } else {
                val handle =
                    try {
                        AnalogOutput(action.channel)
                    } catch (e: Exception) {
                        return Error(AllocationFailed(ANALOG_OUTPUT, action.channel, e.describe()))
                    }
                installToken(
                    registry,
                    session,
                    action.channel,
                    AnalogOutputEntry(action.channel, handle),
                    registry.analogOutputs,
                    registry.analogOutputsByChannel,
                )
            }
        }

        // CAN / SparkMax register variants stay on the legacy execute path in Phase 1.
        // Reaching register() with them is a client routing error.
        is RobotAction.RegisterCanRx,
        is RobotAction.RegisterBrushlessSparkMax,
        -> Error(AllocationFailed(CAN, UNKNOWN_ID, "register variant sent via the register path is not supported"))

        // Operate / deregister variants are handled by execute(); they should not
        // arrive here. A misrouted operate is reported as a failure rather than
        // silently dropped.
        else -> Error(AllocationFailed(CAN, UNKNOWN_ID, "non-register action sent to register path"))
    }

/**
 * Mint a fresh token for [entry], store it under [tokenMap] keyed by the new
 * token, index [portOrChannel] under [byPort], and record the token under the
 * session. Returns [Ok] of the minted token.
 */
private fun <E> installToken(
    registry: HardwareRegistry,
    session: SessionId,
    portOrChannel: Int,
    entry: E,
    tokenMap: HashMap<Token, E>,
    byPort: HashMap<Int, Token>,
): RegisterOutcome {
    val token = Token.random()
    tokenMap[token] = entry
    byPort[portOrChannel] = token
    registry.sessionTokens.getOrPut(session) { HashSet() }.add(token)
    return Ok(token)
}

/**
 * Execute one operate / deregister / CAN / SparkMax [action] against [registry],
 * mutating it in place. All inputs are validated and every WPILib failure is
 * classified into a structured [ApplyError]; this function never throws.
 *
 * Operate actions (SetX / Disable / CanWrite / SparkMax set*) are **rejected
 * while the DS has the robot disabled** ([enabled] == false) with a
 * [RobotDisabled] error — no handle is touched. Register / deregister /
 * subscribe-like actions (including CAN-Rx subscribe) are allowed while
 * disabled. Phase-1 token families (PWM / DIO / Analog) resolve their device by
 * [Token]; CAN and SparkMax stay on the legacy port/id path.
 */
fun execute(
    registry: HardwareRegistry,
    enabled: Boolean,
    action: RobotAction,
): ApplyOutcome =
    when (action) {
        // ---------------------------------------------------------- PWM operate
        is RobotAction.SetPwmSpeed -> {
            if (!enabled) {
                ApplyOutcome.Failed(RobotDisabled(PWM, registry.pwm[action.token]?.port ?: UNKNOWN_ID))
            } else {
                val entry = registry.pwm[action.token]
                when {
                    entry == null ->
                        ApplyOutcome.Failed(NotRegistered(PWM, UNKNOWN_ID))
                    entry.mode != MOTOR ->
                        ApplyOutcome.Failed(
                            WrongDeviceMode(PWM, entry.port, expected = MOTOR.name, actual = entry.mode.name),
                        )
                    action.speed !in -1.0..1.0 ->
                        ApplyOutcome.Failed(OutOfRange(PWM, entry.port, "speed", action.speed, -1.0, 1.0))
                    else -> runOperate(PWM, entry.port) { entry.handle.setSpeed(action.speed) }
                }
            }
        }

        is RobotAction.SetPwmPosition -> {
            if (!enabled) {
                ApplyOutcome.Failed(RobotDisabled(PWM, registry.pwm[action.token]?.port ?: UNKNOWN_ID))
            } else {
                val entry = registry.pwm[action.token]
                when {
                    entry == null ->
                        ApplyOutcome.Failed(NotRegistered(PWM, UNKNOWN_ID))
                    entry.mode != SERVO ->
                        ApplyOutcome.Failed(
                            WrongDeviceMode(PWM, entry.port, expected = SERVO.name, actual = entry.mode.name),
                        )
                    action.position !in 0.0..1.0 ->
                        ApplyOutcome.Failed(OutOfRange(PWM, entry.port, "position", action.position, 0.0, 1.0))
                    else -> runOperate(PWM, entry.port) { entry.handle.setPosition(action.position) }
                }
            }
        }

        is RobotAction.DisablePwm -> {
            if (!enabled) {
                ApplyOutcome.Failed(RobotDisabled(PWM, registry.pwm[action.token]?.port ?: UNKNOWN_ID))
            } else {
                val entry = registry.pwm[action.token]
                if (entry == null) {
                    ApplyOutcome.Failed(NotRegistered(PWM, UNKNOWN_ID))
                } else {
                    runOperate(PWM, entry.port) { entry.handle.setDisabled() }
                }
            }
        }

        is RobotAction.DeregisterPwm ->
            releaseToken(registry.pwm, registry.pwmByPort, action.token, PWM) { it.port to it.handle }

        // ---------------------------------------------------- DIO operate/dereg
        is RobotAction.SetDigitalOutput -> {
            if (!enabled) {
                ApplyOutcome.Failed(
                    RobotDisabled(DIGITAL_OUTPUT, registry.digitalOutputs[action.token]?.channel ?: UNKNOWN_ID),
                )
            } else {
                val entry = registry.digitalOutputs[action.token]
                if (entry == null) {
                    ApplyOutcome.Failed(NotRegistered(DIGITAL_OUTPUT, UNKNOWN_ID))
                } else {
                    runOperate(DIGITAL_OUTPUT, entry.channel) { entry.handle.set(action.value) }
                }
            }
        }

        is RobotAction.DeregisterDigitalInput ->
            releaseToken(registry.digitalInputs, registry.digitalInputsByChannel, action.token, DIGITAL_INPUT) {
                it.channel to it.handle
            }

        is RobotAction.DeregisterDigitalOutput ->
            releaseToken(registry.digitalOutputs, registry.digitalOutputsByChannel, action.token, DIGITAL_OUTPUT) {
                it.channel to it.handle
            }

        // ------------------------------------------------ Analog operate/dereg
        is RobotAction.SetAnalogOutput -> {
            if (!enabled) {
                ApplyOutcome.Failed(
                    RobotDisabled(ANALOG_OUTPUT, registry.analogOutputs[action.token]?.channel ?: UNKNOWN_ID),
                )
            } else {
                val entry = registry.analogOutputs[action.token]
                when {
                    entry == null ->
                        ApplyOutcome.Failed(NotRegistered(ANALOG_OUTPUT, UNKNOWN_ID))
                    action.voltage !in ANALOG_OUT_MIN..ANALOG_OUT_MAX ->
                        ApplyOutcome.Failed(
                            OutOfRange(
                                ANALOG_OUTPUT,
                                entry.channel,
                                "voltage",
                                action.voltage,
                                ANALOG_OUT_MIN,
                                ANALOG_OUT_MAX,
                            ),
                        )
                    else -> runOperate(ANALOG_OUTPUT, entry.channel) { entry.handle.setVoltage(action.voltage) }
                }
            }
        }

        is RobotAction.DeregisterAnalogInput ->
            releaseToken(registry.analogInputs, registry.analogInputsByChannel, action.token, ANALOG_INPUT) {
                it.channel to it.handle
            }

        is RobotAction.DeregisterAnalogOutput ->
            releaseToken(registry.analogOutputs, registry.analogOutputsByChannel, action.token, ANALOG_OUTPUT) {
                it.channel to it.handle
            }

        // --------------------------------------------------------------- CAN
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

        is RobotAction.CanWrite ->
            if (!enabled) {
                ApplyOutcome.Failed(RobotDisabled(CAN, action.messageId))
            } else {
                val bad = action.data.withIndex().firstOrNull { it.value !in 0..255 }
                if (bad != null) {
                    ApplyOutcome.Failed(InvalidCanByte(action.messageId, bad.index, bad.value))
                } else {
                    writeCan(registry, action.messageId, action.apiId, action.data.toWireBytes())
                }
            }

        // ---------------------------------------------------------- SPARK MAX
        is RobotAction.RegisterBrushlessSparkMax ->
            registerWithVerify(
                map = registry.sparkMaxDevices,
                id = action.deviceId,
                deviceKind = SPARKMAX,
                factory = { SparkMax(action.deviceId, SparkLowLevel.MotorType.kBrushless) },
                verify = { sparkMaxDevice ->
                    val firmware = sparkMaxDevice.firmwareString
                    val error = sparkMaxDevice.lastError

                    val connected = (error == REVLibError.kOk) && !firmware.isNullOrEmpty()

                    if (connected) {
                        ApplyOutcome.Applied
                    } else {
                        ApplyOutcome.Failed(ApplyError.NotConnected(SPARKMAX, action.deviceId))
                    }
                },
                onRejected = { sparkMaxDevice ->
                    sparkMaxDevice.close()
                },
            )

        is RobotAction.DeregisterSparkMax ->
            release(registry.sparkMaxDevices, action.deviceId, SPARKMAX)

        is RobotAction.SetSparkMaxOutput ->
            if (!enabled) {
                ApplyOutcome.Failed(RobotDisabled(SPARKMAX, action.deviceId))
            } else if (action.output !in SPARKMAX_OUT_MIN..SPARKMAX_OUT_MAX) {
                ApplyOutcome.Failed(
                    OutOfRange(
                        SPARKMAX,
                        action.deviceId,
                        "dutyCycle",
                        action.output,
                        SPARKMAX_OUT_MIN,
                        SPARKMAX_OUT_MAX,
                    ),
                )
            } else {
                operate(registry.sparkMaxDevices, action.deviceId, SPARKMAX) { it.set(action.output) }
            }

        is RobotAction.SetSparkMaxVoltage ->
            if (!enabled) {
                ApplyOutcome.Failed(RobotDisabled(SPARKMAX, action.deviceId))
            } else {
                operate(registry.sparkMaxDevices, action.deviceId, SPARKMAX) { it.setVoltage(action.voltage) }
            }

        // Phase-1 token registers route through register(); reaching execute()
        // with one is a client routing error.
        is RobotAction.RegisterPwm,
        is RobotAction.RegisterDigitalInput,
        is RobotAction.RegisterDigitalOutput,
        is RobotAction.RegisterAnalogInput,
        is RobotAction.RegisterAnalogOutput,
        ->
            ApplyOutcome.Failed(
                AllocationFailed(CAN, UNKNOWN_ID, "register action sent via the operate path"),
            )
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
    // Token-keyed internally, but re-keyed by port/channel for the snapshot so
    // the RobotState shape (and subscription filtering) is unchanged.
    val analogIn = registry.analogInputs.values.associateBy({ it.channel }, { it.handle.voltage })
    val analogOut = registry.analogOutputs.values.associateBy({ it.channel }, { it.handle.voltage })
    val digitalIn = registry.digitalInputs.values.associateBy({ it.channel }, { it.handle.get() })
    val digitalOut = registry.digitalOutputs.values.associateBy({ it.channel }, { it.handle.get() })
    val pwmSpeed = registry.pwm.values.associateBy({ it.port }, { it.handle.speed })
    val pwmPosition = registry.pwm.values.associateBy({ it.port }, { it.handle.position })
    val sparkMaxSnapshots =
        registry.sparkMaxDevices.mapValues {
            val position = it.value.encoder.position
            val positionError = it.value.lastError
            val positionStatus =
                if (positionError == REVLibError.kOk) {
                    SignalStatus.Ok
                } else {
                    SignalStatus.Error(positionError.toString())
                }

            val velocity = it.value.encoder.velocity
            val velocityError = it.value.lastError
            val velocityStatus =
                if (velocityError == REVLibError.kOk) {
                    SignalStatus.Ok
                } else {
                    SignalStatus.Error(velocityError.toString())
                }
            val timestampSeconds = RobotController.getFPGATime() / 1_000_000.0

            Snapshot.SparkMaxSnapshot(
                position =
                    SignalSample(
                        value = position,
                        timestampSeconds = timestampSeconds,
                        status = positionStatus,
                    ),
                velocity =
                    SignalSample(
                        value = velocity,
                        timestampSeconds = timestampSeconds,
                        status = velocityStatus,
                    ),
            )
        }
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
        sparkMaxSnapshots = sparkMaxSnapshots,
        canFrames = canFrames,
        errors = errors,
    )
}

/**
 * Release every device owned by [session] (called on WebSocket disconnect).
 * Closes the WPILib handles, drops the token + port/channel indices. Tokens the
 * session minted become invalid; a fresh registration by any session can now
 * claim the freed ports/channels.
 */
fun releaseSession(
    registry: HardwareRegistry,
    session: SessionId,
) {
    val tokens = registry.sessionTokens.remove(session) ?: return
    for (token in tokens) {
        registry.pwm.remove(token)?.let { entry ->
            runCatching { entry.handle.close() }
            registry.pwmByPort.remove(entry.port)
        }
        registry.digitalInputs.remove(token)?.let { entry ->
            runCatching { entry.handle.close() }
            registry.digitalInputsByChannel.remove(entry.channel)
        }
        registry.digitalOutputs.remove(token)?.let { entry ->
            runCatching { entry.handle.close() }
            registry.digitalOutputsByChannel.remove(entry.channel)
        }
        registry.analogInputs.remove(token)?.let { entry ->
            runCatching { entry.handle.close() }
            registry.analogInputsByChannel.remove(entry.channel)
        }
        registry.analogOutputs.remove(token)?.let { entry ->
            runCatching { entry.handle.close() }
            registry.analogOutputsByChannel.remove(entry.channel)
        }
    }
}

/** Release every allocated WPILib handle in [registry]. */
fun close(registry: HardwareRegistry) {
    registry.pwm.values.forEach { runCatching { it.handle.close() } }
    registry.digitalInputs.values.forEach { runCatching { it.handle.close() } }
    registry.digitalOutputs.values.forEach { runCatching { it.handle.close() } }
    registry.analogInputs.values.forEach { runCatching { it.handle.close() } }
    registry.analogOutputs.values.forEach { runCatching { it.handle.close() } }
    registry.canDevices.values.forEach { runCatching { it.close() } }
    registry.sparkMaxDevices.values.forEach { runCatching { it.close() } }
    registry.pwm.clear()
    registry.digitalInputs.clear()
    registry.digitalOutputs.clear()
    registry.analogInputs.clear()
    registry.analogOutputs.clear()
    registry.pwmByPort.clear()
    registry.digitalInputsByChannel.clear()
    registry.digitalOutputsByChannel.clear()
    registry.analogInputsByChannel.clear()
    registry.analogOutputsByChannel.clear()
    registry.sessionTokens.clear()
    registry.canDevices.clear()
    registry.canRxSubscriptions.clear()
    registry.sparkMaxDevices.clear()
}

private const val WARMUP_PWM_PORT = 0
private const val WARMUP_DIO_INPUT = 0
private const val WARMUP_DIO_OUTPUT = 1
private const val WARMUP_AIO_INPUT = 0
private const val WARMUP_AIO_OUTPUT = 0
private const val WARMUP_CAN_MESSAGE_ID = 0

/**
 * Force the one-time HAL/JNI bring-up for each resource family by briefly
 * allocating and releasing one device. Best-effort: a family that fails to
 * allocate (unavailable on this controller) is skipped, so its init is simply
 * paid on first real use instead. Call once before the timed loop starts so a
 * client's first registration can't stall a periodic cycle.
 */
fun warmUpHal() {
    runCatching { PWM(WARMUP_PWM_PORT).close() }
    runCatching { DigitalInput(WARMUP_DIO_INPUT).close() }
    runCatching { DigitalOutput(WARMUP_DIO_OUTPUT).close() }
    runCatching { AnalogInput(WARMUP_AIO_INPUT).close() }
    runCatching { AnalogOutput(WARMUP_AIO_OUTPUT).close() }
    runCatching { CAN(WARMUP_CAN_MESSAGE_ID).close() }
}

private fun canFor(
    registry: HardwareRegistry,
    messageId: Int,
): CAN = registry.canDevices.getOrPut(messageId) { CAN(messageId) }

/** Allocate (idempotently) a legacy CAN device, classifying a WPILib throw as [AllocationFailed]. */
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

/** Allocate (idempotently) a device if it passes a verification, classifying a WPILib throw as
 * [AllocationFailed] and a verification throw as [HardwareFault]. Allows for a cleanup function
 * to be specified in case of a verification failure. */
private inline fun <T> registerWithVerify(
    map: MutableMap<Int, T>,
    id: Int,
    deviceKind: DeviceKind,
    factory: () -> T,
    verify: (T) -> ApplyOutcome,
    onRejected: (T) -> Unit,
): ApplyOutcome {
    if (map.containsKey(id)) {
        return ApplyOutcome.Applied
    } else {
        val device =
            try {
                factory()
            } catch (e: Exception) {
                return ApplyOutcome.Failed(AllocationFailed(deviceKind, id, e.describe()))
            }

        val outcome =
            try {
                verify(device)
            } catch (e: Exception) {
                ApplyOutcome.Failed(HardwareFault(deviceKind, id, e.describe()))
            }

        when (outcome) {
            is ApplyOutcome.Applied -> {
                map[id] = device
                return outcome
            }

            is ApplyOutcome.Failed -> {
                try {
                    onRejected(device)
                } catch (e: Exception) {
                    println(
                        "Failed to clean up " +
                            "$deviceKind $id: ${e.describe()} that did not pass verification",
                    )
                }
                return outcome
            }
        }
    }
}

/** Run [block] on a registered legacy device, or report it [NotRegistered] / [HardwareFault]. */
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

/** Run [block] on a registered handle, classifying a throw as [HardwareFault]. */
private inline fun runOperate(
    deviceKind: DeviceKind,
    id: Int,
    block: () -> Unit,
): ApplyOutcome =
    try {
        block()
        ApplyOutcome.Applied
    } catch (e: Exception) {
        ApplyOutcome.Failed(HardwareFault(deviceKind, id, e.describe()))
    }

/** Remove, close, and de-index a token-keyed device if present (idempotent). */
private inline fun <E> releaseToken(
    tokenMap: MutableMap<Token, E>,
    byPort: MutableMap<Int, Token>,
    token: Token,
    deviceKind: DeviceKind,
    entryIdAndHandle: (E) -> Pair<Int, AutoCloseable>,
): ApplyOutcome {
    val entry = tokenMap.remove(token) ?: return ApplyOutcome.Applied
    val (id, handle) = entryIdAndHandle(entry)
    byPort.remove(id)
    return try {
        handle.close()
        ApplyOutcome.Applied
    } catch (e: Exception) {
        ApplyOutcome.Failed(HardwareFault(deviceKind, id, e.describe()))
    }
}

/** Remove and close a legacy device if present (idempotent). */
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

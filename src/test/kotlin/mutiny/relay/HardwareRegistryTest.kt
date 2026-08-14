package mutiny.relay

import edu.wpi.first.hal.HAL
import mutiny.relay.ApplyError.AllocationFailed
import mutiny.relay.ApplyError.DeviceAlreadyRegistered
import mutiny.relay.ApplyError.InvalidCanByte
import mutiny.relay.ApplyError.RobotDisabled
import mutiny.relay.ApplyError.WrongDeviceMode
import mutiny.relay.PwmMode.MOTOR
import mutiny.relay.PwmMode.SERVO
import mutiny.relay.RegisterOutcome.Ok
import org.junit.jupiter.api.AfterEach
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Assertions.assertNotEquals
import org.junit.jupiter.api.Assertions.assertTrue
import org.junit.jupiter.api.BeforeEach
import org.junit.jupiter.api.Test

/**
 * Unit coverage for the token registration model: exclusive registration, token
 * minting, session-owned release, mode validation, the disabled-operate gate,
 * and the CAN / SparkMax token paths. The sim HAL is initialized so
 * PWM/DIO/CAN allocation works on the JVM.
 */
class HardwareRegistryTest {
    private lateinit var registry: HardwareRegistry
    private val session = SessionId("session-a")
    private val other = SessionId("session-b")

    @BeforeEach
    fun setUp() {
        assertTrue(HAL.initialize(500, 0), "HAL sim failed to initialize")
        registry = HardwareRegistry()
    }

    @AfterEach
    fun tearDown() {
        close(registry)
        HAL.shutdown()
    }

    @Test
    fun `register mints a token for a free port`() {
        val outcome = register(registry, session, RobotAction.RegisterPwm(PORT_1, MOTOR))

        assertTrue(outcome is Ok)
        val token = (outcome as Ok).token
        assertTrue(token.value.isNotBlank())
    }

    @Test
    fun `a second registration of an in-use port is rejected as DeviceAlreadyRegistered`() {
        register(registry, session, RobotAction.RegisterPwm(PORT_1, MOTOR))

        val second = register(registry, other, RobotAction.RegisterPwm(PORT_1, MOTOR))

        assertTrue(second is RegisterOutcome.Error)
        val err = (second as RegisterOutcome.Error).error
        assertTrue(err is DeviceAlreadyRegistered)
        assertEquals(PORT_1, (err as DeviceAlreadyRegistered).id)
    }

    @Test
    fun `distinct ports mint distinct tokens`() {
        val a = register(registry, session, RobotAction.RegisterPwm(PORT_1, MOTOR))
        val b = register(registry, session, RobotAction.RegisterPwm(PORT_2, MOTOR))

        val ta = (a as Ok).token
        val tb = (b as Ok).token
        assertNotEquals(ta, tb)
    }

    @Test
    fun `releaseSession frees the session ports for re-registration`() {
        val first = register(registry, session, RobotAction.RegisterPwm(PORT_1, MOTOR))
        assertTrue(first is Ok)

        releaseSession(registry, session)

        // After release, any session can claim the freed port.
        val reclaimed = register(registry, other, RobotAction.RegisterPwm(PORT_1, MOTOR))
        assertTrue(reclaimed is Ok)
    }

    @Test
    fun `releaseSession only frees the targeted session`() {
        register(registry, session, RobotAction.RegisterPwm(PORT_1, MOTOR))
        register(registry, other, RobotAction.RegisterPwm(PORT_2, MOTOR))

        releaseSession(registry, session)

        // The other session's port is still held by `other`.
        val claim = register(registry, session, RobotAction.RegisterPwm(PORT_2, MOTOR))
        assertTrue(claim is RegisterOutcome.Error)
        assertTrue((claim as RegisterOutcome.Error).error is DeviceAlreadyRegistered)
    }

    @Test
    fun `SetPwmPosition against a motor-registered token is WrongDeviceMode`() {
        val token = (register(registry, session, RobotAction.RegisterPwm(PORT_1, MOTOR)) as Ok).token

        val outcome =
            execute(registry, enabled = true, RobotAction.SetPwmPosition(token, position = 0.5))

        assertTrue(outcome is ApplyOutcome.Failed)
        val err = (outcome as ApplyOutcome.Failed).error
        assertTrue(err is WrongDeviceMode)
        assertEquals(MOTOR.name, (err as WrongDeviceMode).actual)
        assertEquals(SERVO.name, err.expected)
    }

    @Test
    fun `SetPwmSpeed against a servo-registered token is WrongDeviceMode`() {
        val token = (register(registry, session, RobotAction.RegisterPwm(PORT_1, SERVO)) as Ok).token

        val outcome =
            execute(registry, enabled = true, RobotAction.SetPwmSpeed(token, speed = 0.5))

        assertTrue(outcome is ApplyOutcome.Failed)
        assertTrue((outcome as ApplyOutcome.Failed).error is WrongDeviceMode)
    }

    @Test
    fun `an operate is rejected as RobotDisabled when enabled is false`() {
        val token = (register(registry, session, RobotAction.RegisterPwm(PORT_1, MOTOR)) as Ok).token

        val outcome =
            execute(registry, enabled = false, RobotAction.SetPwmSpeed(token, speed = 0.5))

        assertTrue(outcome is ApplyOutcome.Failed)
        val err = (outcome as ApplyOutcome.Failed).error
        assertTrue(err is RobotDisabled)
        assertEquals(PORT_1, (err as RobotDisabled).id)
    }

    @Test
    fun `registration succeeds regardless of the enabled state`() {
        // register() has no enabled gate (allocation is safe while disabled).
        val outcome = register(registry, session, RobotAction.RegisterPwm(PORT_1, MOTOR))
        assertTrue(outcome is Ok)
    }

    @Test
    fun `an operate with an unknown token is NotRegistered`() {
        val outcome =
            execute(
                registry,
                enabled = true,
                RobotAction.SetPwmSpeed(Token("not-a-real-registration"), speed = 0.5),
            )

        assertTrue(outcome is ApplyOutcome.Failed)
        assertTrue((outcome as ApplyOutcome.Failed).error is ApplyError.NotRegistered)
    }

    @Test
    fun `a misrouted register sent via the operate path is a failure`() {
        val outcome = execute(registry, enabled = true, RobotAction.RegisterPwm(PORT_1, MOTOR))

        assertTrue(outcome is ApplyOutcome.Failed)
        assertTrue((outcome as ApplyOutcome.Failed).error is AllocationFailed)
    }

    // ------------------------------------------------------------------ CAN

    @Test
    fun `RegisterCanRx mints a token for a free frame id`() {
        val outcome = register(registry, session, RobotAction.RegisterCanRx(CAN_MSG, CAN_API))

        assertTrue(outcome is Ok)
        assertTrue((outcome as Ok).token.value.isNotBlank())
    }

    @Test
    fun `a second registration of an in-use frame id is rejected as DeviceAlreadyRegistered`() {
        register(registry, session, RobotAction.RegisterCanRx(CAN_MSG, CAN_API))

        val second = register(registry, other, RobotAction.RegisterCanRx(CAN_MSG, CAN_API))

        assertTrue(second is RegisterOutcome.Error)
        val err = (second as RegisterOutcome.Error).error
        assertTrue(err is DeviceAlreadyRegistered)
        assertEquals(CAN_MSG, (err as DeviceAlreadyRegistered).id)
    }

    @Test
    fun `distinct api ids on the same message id mint coexisting tokens`() {
        val a = register(registry, session, RobotAction.RegisterCanRx(CAN_MSG, CAN_API))
        val b = register(registry, session, RobotAction.RegisterCanRx(CAN_MSG, CAN_API + 1))

        assertTrue(a is Ok)
        assertTrue(b is Ok)
        assertNotEquals((a as Ok).token, (b as Ok).token)
    }

    @Test
    fun `DeregisterCanRx by token frees the frame id for re-registration`() {
        val token = (register(registry, session, RobotAction.RegisterCanRx(CAN_MSG, CAN_API)) as Ok).token

        val outcome = execute(registry, enabled = true, RobotAction.DeregisterCanRx(token))
        assertTrue(outcome is ApplyOutcome.Applied)

        val reclaimed = register(registry, other, RobotAction.RegisterCanRx(CAN_MSG, CAN_API))
        assertTrue(reclaimed is Ok)
    }

    @Test
    fun `DeregisterCanRx with an unknown token is idempotent`() {
        val outcome = execute(registry, enabled = true, RobotAction.DeregisterCanRx(Token("nope")))

        assertTrue(outcome is ApplyOutcome.Applied)
    }

    @Test
    fun `releaseSession frees the session CAN subscriptions`() {
        register(registry, session, RobotAction.RegisterCanRx(CAN_MSG, CAN_API))

        releaseSession(registry, session)

        val reclaimed = register(registry, other, RobotAction.RegisterCanRx(CAN_MSG, CAN_API))
        assertTrue(reclaimed is Ok)
    }

    @Test
    fun `CanWrite with an out-of-range byte is InvalidCanByte`() {
        val outcome =
            execute(registry, enabled = true, RobotAction.CanWrite(CAN_MSG, CAN_API, data = listOf(0, 256)))

        assertTrue(outcome is ApplyOutcome.Failed)
        assertTrue((outcome as ApplyOutcome.Failed).error is InvalidCanByte)
    }

    @Test
    fun `CanWrite is rejected as RobotDisabled when disabled`() {
        val outcome =
            execute(registry, enabled = false, RobotAction.CanWrite(CAN_MSG, CAN_API, data = listOf(1)))

        assertTrue(outcome is ApplyOutcome.Failed)
        assertTrue((outcome as ApplyOutcome.Failed).error is RobotDisabled)
    }

    @Test
    fun `a RegisterCanRx sent via the operate path is a failure`() {
        val outcome = execute(registry, enabled = true, RobotAction.RegisterCanRx(CAN_MSG, CAN_API))

        assertTrue(outcome is ApplyOutcome.Failed)
        assertTrue((outcome as ApplyOutcome.Failed).error is AllocationFailed)
    }

    // ------------------------------------------------------------- SPARK MAX

    @Test
    fun `a RegisterBrushlessSparkMax sent via the operate path is a failure`() {
        val outcome = execute(registry, enabled = true, RobotAction.RegisterBrushlessSparkMax(DEVICE_ID))

        assertTrue(outcome is ApplyOutcome.Failed)
        assertTrue((outcome as ApplyOutcome.Failed).error is AllocationFailed)
    }

    @Test
    fun `SetSparkMaxOutput with an unknown token is NotRegistered`() {
        val outcome =
            execute(registry, enabled = true, RobotAction.SetSparkMaxOutput(Token("nope"), output = 0.5))

        assertTrue(outcome is ApplyOutcome.Failed)
        assertTrue((outcome as ApplyOutcome.Failed).error is ApplyError.NotRegistered)
    }

    @Test
    fun `SetSparkMaxVoltage with an unknown token is NotRegistered`() {
        val outcome =
            execute(registry, enabled = true, RobotAction.SetSparkMaxVoltage(Token("nope"), voltage = 6.0))

        assertTrue(outcome is ApplyOutcome.Failed)
        assertTrue((outcome as ApplyOutcome.Failed).error is ApplyError.NotRegistered)
    }

    @Test
    fun `DeregisterSparkMax with an unknown token is idempotent`() {
        val outcome = execute(registry, enabled = true, RobotAction.DeregisterSparkMax(Token("nope")))

        assertTrue(outcome is ApplyOutcome.Applied)
    }

    private companion object {
        const val PORT_1 = 1
        const val PORT_2 = 2
        const val CAN_MSG = 0x205
        const val CAN_API = 0
        const val DEVICE_ID = 1
    }
}

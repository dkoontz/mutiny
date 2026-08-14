package mutiny.relay

import edu.wpi.first.hal.HAL
import mutiny.relay.ApplyError.AllocationFailed
import mutiny.relay.ApplyError.DeviceAlreadyRegistered
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
 * Unit coverage for the Phase-1 token model: exclusive registration, token
 * minting, session-owned release, mode validation, and the disabled-operate
 * gate. The sim HAL is initialized so PWM/DIO allocation works on the JVM.
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

    private companion object {
        const val PORT_1 = 1
        const val PORT_2 = 2
    }
}

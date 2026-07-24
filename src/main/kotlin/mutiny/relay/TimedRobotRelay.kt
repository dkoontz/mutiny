package mutiny.relay

import edu.wpi.first.wpilibj.TimedRobot
import edu.wpi.first.wpilibj.Timer
import java.util.concurrent.ConcurrentLinkedQueue
import java.util.concurrent.CopyOnWriteArrayList
import edu.wpi.first.wpilibj.RobotState as WpiRobotState

/**
 * A transparent relay between an external robot application and the controller's
 * WPILib hardware. It owns no control logic of its own: a separate network client
 * enqueues [RobotAction]s via [submit] (from any thread), and every periodic
 * cycle this class drains the queue, applies the actions to the hardware,
 * publishes a fresh [RobotState] snapshot, and repeats.
 *
 * The network layer (websocket / protobuf / zmq / ...) is intentionally out of
 * scope here; it only needs a reference to this object as a [NetworkProvider].
 */
class TimedRobotRelay :
    TimedRobot(),
    NetworkProvider {
    private val pending = ConcurrentLinkedQueue<RobotAction>()
    private val registry = HardwareRegistry()
    private val shutdownHooks = CopyOnWriteArrayList<() -> Unit>()

    @Volatile
    private var state: RobotState = RobotState.EMPTY
    private var sequence = 0L

    /**
     * Register a hook run once when the robot program ends, before hardware is
     * released. Used by transports (e.g. the WebSocket server) to shut down
     * cleanly without the relay knowing about any specific transport.
     */
    fun addShutdownHook(hook: () -> Unit) {
        shutdownHooks.add(hook)
    }

    override fun robotInit() {
        println("[TimedRobotRelay] initialized; period=${period}s")
    }

    override fun robotPeriodic() {
        // 1. Drain the action queue (network thread wrote here; we own it now).
        val drained = ArrayList<RobotAction>()
        while (true) {
            drained.add(pending.poll() ?: break)
        }

        // 2. Apply each action; capture failures for diagnostics.
        val errors = ArrayList<ActionError>(drained.size)
        for (action in drained) {
            when (val outcome = execute(registry, action)) {
                ApplyOutcome.Applied -> Unit
                is ApplyOutcome.Failed -> errors.add(ActionError(action, outcome.error))
            }
        }

        // 3. Sample inputs + commanded outputs and publish an atomic snapshot.
        state =
            sample(
                registry,
                sequence = ++sequence,
                timestampSec = Timer.getFPGATimestamp(),
                enabled = isEnabled,
                autonomous = isAutonomous,
                teleop = isTeleop,
                test = isTest,
                emergencyStopped = WpiRobotState.isEStopped(),
                errors = errors,
            )
    }

    override fun endCompetition() {
        shutdownHooks.forEach { runCatching(it) }
        close(registry)
    }

    // ----- NetworkProvider -----

    override fun submit(action: RobotAction) {
        pending.add(action)
    }

    override val latest: RobotState
        get() = state
}

package mutiny.relay

import edu.wpi.first.wpilibj.TimedRobot
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.Tracer
import kotlinx.coroutines.CompletableDeferred
import java.util.concurrent.ConcurrentHashMap
import java.util.concurrent.ConcurrentLinkedQueue
import java.util.concurrent.CopyOnWriteArrayList
import edu.wpi.first.wpilibj.RobotState as WpiRobotState

/** Individual actions slower than this (ms) are logged to isolate apply hotspots. */
private const val SLOW_ACTION_THRESHOLD_MS = 1.0

/**
 * One queued register RPC: the owning session, the register action, and the
 * deferred the periodic loop completes with the outcome.
 */
private data class PendingRegister(
    val session: SessionId,
    val action: RobotAction,
    val deferred: CompletableDeferred<RegisterOutcome>,
)

/**
 * A transparent relay between an external robot application and the controller's
 * WPILib hardware. It owns no control logic of its own: a separate network
 * client enqueues [RobotAction]s via [submitAction] / [submitRegister] (from any
 * thread), and every periodic cycle this class drains the two queues —
 * registers are resolved (minting tokens, completing their RPC deferreds) and
 * operates are applied (gated by the DS enabled state) — then publishes a fresh
 * [RobotState] snapshot, and repeats.
 *
 * Each WebSocket connection is a [SessionId] ([openSession] / [closeSession]);
 * [closeSession] releases every device the session minted, so reconnect is a
 * clean slate. The network layer (websocket / protobuf / zmq / ...) is
 * intentionally out of scope here; it only needs a reference to this object as a
 * [NetworkProvider].
 */
class TimedRobotRelay :
    TimedRobot(),
    NetworkProvider {
    private val pendingActions = ConcurrentLinkedQueue<Pair<SessionId, RobotAction>>()
    private val pendingRegisters = ConcurrentLinkedQueue<PendingRegister>()
    private val registry = HardwareRegistry()
    private val sessions = ConcurrentHashMap.newKeySet<SessionId>()
    private val shutdownHooks = CopyOnWriteArrayList<() -> Unit>()
    private val loopTracer = Tracer()

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
        val warmupStartSec = Timer.getFPGATimestamp()
        warmUpHal()
        val warmupMs = (Timer.getFPGATimestamp() - warmupStartSec) * 1_000.0
        println(
            "[TimedRobotRelay] initialized; period=${period}s; HAL warm-up ${"%.1f".format(warmupMs)}ms",
        )
    }

    override fun robotPeriodic() {
        val loopStartSec = Timer.getFPGATimestamp()
        loopTracer.clearEpochs()

        // 1. Drain both queues (network threads wrote here; we own them now).
        val drainedRegisters = ArrayList<PendingRegister>()
        while (true) {
            drainedRegisters.add(pendingRegisters.poll() ?: break)
        }
        val drainedActions = ArrayList<Pair<SessionId, RobotAction>>()
        while (true) {
            drainedActions.add(pendingActions.poll() ?: break)
        }
        loopTracer.addEpoch("drain (r=${drainedRegisters.size}, a=${drainedActions.size})")

        // 2. Resolve register RPCs: mint tokens (or structured errors) and complete
        //    each deferred. Register failures are per-session RPC replies only —
        //    they do NOT go into the broadcast snapshot.
        for (pending in drainedRegisters) {
            val actionStartSec = Timer.getFPGATimestamp()
            val outcome = register(registry, pending.session, pending.action)
            pending.deferred.complete(outcome)
            val actionMs = (Timer.getFPGATimestamp() - actionStartSec) * 1_000.0
            if (actionMs > SLOW_ACTION_THRESHOLD_MS) {
                println(
                    "[TimedRobotRelay] slow register ${pending.action::class.simpleName}: ${"%.2f".format(actionMs)}ms",
                )
            }
        }
        loopTracer.addEpoch("register (${drainedRegisters.size})")

        // 3. Apply operate actions; capture failures for the snapshot. Operates are
        //    rejected while the DS has the robot disabled.
        val errors = ArrayList<ActionError>(drainedActions.size)
        for ((_, action) in drainedActions) {
            val actionStartSec = Timer.getFPGATimestamp()
            val outcome = execute(registry, isEnabled, action)
            val actionMs = (Timer.getFPGATimestamp() - actionStartSec) * 1_000.0
            if (outcome is ApplyOutcome.Failed) {
                errors.add(ActionError(action, outcome.error))
            }
            if (actionMs > SLOW_ACTION_THRESHOLD_MS) {
                println(
                    "[TimedRobotRelay] slow action ${action::class.simpleName}: ${"%.2f".format(actionMs)}ms",
                )
            }
        }
        loopTracer.addEpoch("apply (${drainedActions.size})")

        // 4. Sample inputs + commanded outputs and publish an atomic snapshot.
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
        loopTracer.addEpoch("sample")

        // Only surface the per-phase breakdown when this cycle overran the budget;
        // Tracer.printEpochs() is internally rate-limited to once per second.
        if (Timer.getFPGATimestamp() - loopStartSec > period) {
            loopTracer.printEpochs()
        }
    }

    override fun endCompetition() {
        shutdownHooks.forEach { runCatching(it) }
        close(registry)
    }

    // ----- NetworkProvider -----

    override fun openSession(): SessionId {
        val id = SessionId.random()
        sessions.add(id)
        return id
    }

    override fun closeSession(id: SessionId) {
        if (sessions.remove(id)) {
            releaseSession(registry, id)
        }
    }

    override fun submitAction(
        sessionId: SessionId,
        action: RobotAction,
    ) {
        pendingActions.add(sessionId to action)
    }

    override fun submitRegister(
        sessionId: SessionId,
        action: RobotAction,
    ): CompletableDeferred<RegisterOutcome> {
        val deferred = CompletableDeferred<RegisterOutcome>()
        pendingRegisters.add(PendingRegister(sessionId, action, deferred))
        return deferred
    }

    override val latest: RobotState
        get() = state
}

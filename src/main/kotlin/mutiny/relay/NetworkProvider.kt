package mutiny.relay

/**
 * Implemented by the relay and handed to a network transport (websocket /
 * protobuf / zmq / ...) so that transport can push [RobotAction]s into the relay
 * and pull [RobotState] out, with no knowledge of WPILib.
 *
 * [submit] is safe to call from any thread; actions are drained and executed on
 * the robot periodic thread. [latest] is refreshed every cycle and safe to read
 * from any thread. The relay ([TimedRobotRelay]) is the canonical
 * implementation.
 */
interface NetworkProvider {
    fun submit(action: RobotAction)

    fun submitAll(actions: Collection<RobotAction>) {
        actions.forEach { submit(it) }
    }

    val latest: RobotState
}

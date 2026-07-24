package mutiny

import edu.wpi.first.hal.FRCNetComm
import edu.wpi.first.hal.HAL
import edu.wpi.first.wpilibj.RobotBase
import mutiny.relay.TimedRobotRelay
import mutiny.transport.websocket.WebSocketRelayServer
import mutiny.transport.websocket.start
import mutiny.transport.websocket.stop

/**
 * Do NOT add any static variables to this class, or any initialization at all. Unless you know what
 * you are doing, do not modify this file except to change the parameter class to the startRobot
 * call.
 */
object Main {
    /**
     * Main initialization function. Do not perform any initialization here.
     *
     * If you change your main robot class, change the parameter type.
     */
    @JvmStatic
    fun main(args: Array<String>) {
        RobotBase.startRobot {
            HAL.report(
                FRCNetComm.tResourceType.kResourceType_Language,
                FRCNetComm.tInstances.kLanguage_Kotlin,
            )
            println("## Done starting control program ####################")
            val relay = TimedRobotRelay()
            val server = start(WebSocketRelayServer(relay))
            relay.addShutdownHook { stop(server) }
            relay
        }
    }
}

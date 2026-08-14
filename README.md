
                                                                                         #:=*%#
                                                                       :            -%%%+%:     =%%%
             =%-                         %%                   :%*     *%%                **   +%*  .%:
            :%%%%           *%      +:   %%    *%%*           :%%%    %#%   %%%%%-        %. :%=    *%
             *%%%#    .%%   %= =%%%%%   .%+  *%%%%%     #%%%%. %%=%= %= %= :%             =*  #%=  -%#
              :%%%%.  .%%  :%  .%*  :-  -%-  %%   :   =%%-  %%.-%. *%%= *% .%%%*+==        %    =+#+
                :%%*  =%%% *% .%%%%%:   *%:  %%:    *=%%     %= %*  %%= :% :%                      .
                  %%= *%:%%%*    %- :%%:#%    %%%%%%% *%.   %%. +%  :    %:%%%%%%#        :*=
                  -%%:%%  %%+    %%*   :%%%#%%+ ==-    *%%%%%:   %=      %%              #%%%%              +-
                   :%%%%  .%:     :     ::-::.:                  %%                       #%%%+           =%%%*
                    =%%=   .                                     .*                        *%%%          .%%%%:
                     =                             ..        .:.            ::              %%%*         %%%%=
                     -                           -%%%%%%%%%%%%%%%%%%%%%%%%% -%%%:            %%%*      :%%%%
                     :         .%%                %%%%%%%%%%%%%%%%%%%%%%%%* .%%%%        *%%% %%%%    .%%%%.
                              +%%%%                .        -%%%%%    -+**  .%%%%%*      -%%%  %%%%  .%%%%:
             -=               %%%%%:      +%%      =%%%     +%%%%%           =%%%%%%      %%%   %%%%#%%%#
        :%%%%%%%             :%%%%%%     %%%%:    -%%%%*     *%%%%           :%%%%%%%-    %%%    %%%%%%=
         *%%%%%%%%:          *%%%%%%:    %%%%%     %%%%%     +%%%%.    -%%%   *%%:=%%%%   %%%:    %%%%=
           %%%%%%%%%#        %%%%*%%%:   %%%%%     =%%%%=    .%%%%.    #%%%%  .%%=  #%%%+ *%%#    %%%%
           :%%%%%%%%%%%     -%%%%.+%%%-  =%%%%=    .%%%%%:    %%%%:    =%%%%-  %%%    %%%%%%%#    %%%*
             *%%%%%%%%%%%   *%%%*  %%%%:  %%%%%     %%%%%%    %%%%-     +%%%%  %%%+    -%%%%%%    %%%+
              %%%%%:=%%%%%%#%%%%.  =%%%%. :%%%%+    %%%%%%*   *%%%=      %%%%* *%%%      %%%%%    %%%=
              -%%%%=  .%%%%%%%%%    %%%%%  -%%%%=  -%%% #%%.  =%%%%       %%%%: %%%       *%%%    %%%+
               +%%%+     #%%%%%:    :#%%%#  #%%%%%%%%%=  %%%  :%%%%        %%%% #%%        *%%-   %%%-
                %%%%       =%%%=    . %%%%*  %%%%%%%%%   .%%+  %%%%         %%%%  .        -%%-   %%%:
                 %%%%       %%%%#      %%%%+ *.#%%%%%=     *:  *%%%.        :%%%%           .:    %%%:
                 *%%%=      %%%         %%%%:=  .              .%%%          *%%%#                %%%:
                  %%%%:     :%          :%%%%                                 %%%%*               %%%.
                  -%%%*                  =%%%%                                 %%%=               %%%
                   %%%%:                  #%%%%                                                   %%%
                   :%%%%                   %%%%%                                                  %%#
                    #%%%*                   %%%%-                                                 %%+
                     %%%%                                                                         %%=
                     :%%%#                                                                        %%:
                      %%%%%                                                                        :
                       %%%%*
                       -%%%%:
                        *%%%%-
                         %%%%#
                          %%%#
                            =:
                             .


Mutiny is a bridge between an external robot application and an FRC controller (RoboRIO, Core One). Mutiny runs on the controller, receives commands over a network transport, executes them against attached hardware (motors, sensors, CAN devices), and streams back state snapshots.

## Architecture

```
Client ──(JSON)──▶ Transport (WebSocket) ──(RobotAction)──▶ NetworkProvider
                                                                 │
                                                    ConcurrentLinkedQueue
                                                                 │
                                                              [drain]
                                                                 │
                                                          HardwareRegistry
                                                               │
                                                          execute / sample
                                                               │
Client ◀──(JSON)── Transport (WebSocket) ◀──(RobotState)── NetworkProvider
```

### Layers

**Transport layer** (`src/main/kotlin/mutiny/transport/`) — Wire protocol and networking. Knows nothing about WPILib. Talks to the relay layer only through the `NetworkProvider` interface.

**Relay layer** (`src/main/kotlin/mutiny/relay/`) — WPILib-aware core. Runs on the robot's periodic loop (~20ms). Drains queued actions, executes them against WPILib hardware, captures errors, and samples device state.

### Key components

| File | Role |
|------|------|
| `Main.kt` | Entry point — wires transport to relay, hands to WPILib |
| `NetworkProvider.kt` | Interface — sessions, register RPC, action queue, `latest` snapshot |
| `TimedRobotRelay.kt` | Core loop — drains registers + actions, executes, samples |
| `RobotAction.kt` | Sealed interface — every command the relay understands |
| `ApplyError.kt` | Sealed interface — structured errors (no exceptions escape) |
| `RobotState.kt` | Immutable snapshot of all device state |
| `HardwareRegistry.kt` | Token-keyed live WPILib handles + `register()`/`execute()`/`sample()` |
| `WebSocketRelayServer.kt` | Ktor WebSocket transport |
| `WireTypes.kt` | Client/server message DTOs and subscription filtering |

## Building

**Prerequisites:** JDK 17, internet access for Gradle dependencies.

```bash
# Full build (compiles, formats Kotlin, runs tests)
./gradlew build

# Compile only
./gradlew compileKotlin

# Format Kotlin code
./gradlew ktlintFormat

# Run tests
./gradlew test

# Deploy to RoboRIO (requires network connection to the robot)
# Set your team number in .wpilib/wpilib_preferences.json first
./gradlew deploy

# Desktop simulation (WPILib GUI)
./gradlew simulateJava
```

The build produces a fat JAR (`build/libs/Mutiny-all.jar`) containing all runtime dependencies, ready for deployment.

## Adding a New Network Provider

Transports live under `src/main/kotlin/mutiny/transport/`. They depend only on the `NetworkProvider` interface — no WPILib knowledge needed.

1. Create a new package, e.g. `transport/zmq/`.

2. Accept a `NetworkProvider` as a constructor parameter:
   ```kotlin
   class ZmqRelayServer(val networkProvider: NetworkProvider, val port: Int = 5801)
   ```

3. **Ingress:** Receive messages from your transport, deserialize to `RobotAction`, call `networkProvider.submitAction(sessionId, action)` for operates (or `submitRegister(sessionId, action)` for the register RPC, which replies with a minted token).

4. **Egress:** Periodically poll `networkProvider.latest` and send the `RobotState` snapshot to clients. Optionally implement subscription filtering (see `WebSocketRelayServer.kt:126-142` for the pattern).

5. Register cleanup with `relay.addShutdownHook { stop(yourServer) }` so your transport shuts down cleanly when the robot is disabled.

6. Wire it in `Main.kt` alongside (or instead of) the WebSocket server:
   ```kotlin
   val relay = TimedRobotRelay()
   val wsServer = start(WebSocketRelayServer(relay))
   val zmqServer = start(ZmqRelayServer(relay))
   relay.addShutdownHook { stop(wsServer); stop(zmqServer) }
   ```

## Adding a New RobotAction

1. **Add register / operate variants** to `RobotAction.kt`. A registration
   identifies the device by port / channel / id; the reply mints an opaque
   `Token` that later operate / deregister actions carry in its place:
   ```kotlin
   @Serializable
   @SerialName("relay.registerOutput")
   data class RegisterRelayOutput(val channel: Int) : RobotAction

   @Serializable
   @SerialName("relay.setForward")
   data class SetRelayForward(val token: Token, val on: Boolean) : RobotAction
   ```

2. **Add a `DeviceKind`** in `ApplyError.kt` (e.g. `RELAY`).

3. **Add an entry + maps** in `HardwareRegistry.kt` — one token-keyed map of
   entries, one reverse index by port/channel/id, and cleanup in
   `releaseSession()` and `close()`:
   ```kotlin
   internal val relayOutputs = HashMap<Token, RelayOutputEntry>()
   internal val relayOutputsByChannel = HashMap<Int, Token>()
   ```

4. **Add a `register()` branch** in `HardwareRegistry.kt` — reject an in-use
   port/channel with `DeviceAlreadyRegistered`, wrap the WPILib allocation in a
   try/catch (`AllocationFailed`), then `installToken(...)`.

5. **Add an `execute()` branch** for the operate / deregister variants — resolve
   the entry by token (`NotRegistered` if absent), validate ranges before
   operating, and reject operates while disabled with `RobotDisabled`.

6. **Add sampling** in `HardwareRegistry.kt:sample()` to read the new device's
   state. If new fields are needed on `RobotState`, add them there and to the
   `EMPTY` companion.

7. (Optional) Add a field to `Subscription` in `WireTypes.kt` and update the `filter()` function if you want clients to be able to filter the new data.

No codec changes are needed — `@Serializable` and `@SerialName` annotations on the variant automatically handle JSON serialization.


## Why 'Mutiny'?

Mutiny is both a reference to the excellent TV series `Halt and Catch Fire` and to the idea of rebelling against the way FRC promotes the use of WPILib, Java, and object oriented programming. Just like Cameron and Donna we are creating our own way forward.


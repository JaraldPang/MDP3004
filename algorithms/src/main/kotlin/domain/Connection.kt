package domain

import com.fasterxml.jackson.annotation.JsonIgnore
import com.fasterxml.jackson.annotation.JsonProperty
import io.ktor.network.selector.ActorSelectorManager
import io.ktor.network.sockets.Socket
import io.ktor.network.sockets.aSocket
import io.ktor.network.sockets.openReadChannel
import io.ktor.network.sockets.openWriteChannel
import io.ktor.util.KtorExperimentalAPI
import javafx.beans.property.SimpleObjectProperty
import kotlinx.coroutines.*
import kotlinx.coroutines.channels.Channel
import kotlinx.coroutines.channels.ReceiveChannel
import kotlinx.coroutines.channels.SendChannel
import kotlinx.coroutines.io.*
import kotlinx.coroutines.selects.selectUnbiased
import model.CellInfoModel
import tornadofx.*
import java.util.concurrent.TimeUnit

data class ConnectionJson(@get:JsonProperty("isConnected") val isConnected: Boolean) : JsonSerializable {
    @JsonIgnore
    override val filename = "Connection"
}

class Connection(
    private val hostname: String = HOSTNAME,
    private val port: Int = PORT,
    private val wayPointChannel: SendChannel<Pair<Int, Int>>,
    private val startCommandChannel: SendChannel<String>
) {
    companion object {
        const val HOSTNAME = "192.168.17.1"
        const val PORT = 45000
        const val ANDROID_PREFIX = "an"
        const val ARDUINO_PREFIX = "ar"
        const val RPI_PREFIX = "rpi"
        const val MOVE_FORWARD_COMMAND = "w"
        const val TURN_LEFT_COMMAND = "a"
        const val TURN_RIGHT_COMMAND = "d"
        const val GET_SENSOR_DATA_COMMAND = "g"
        const val CALIBRATE_COMMAND = "c"
        const val FASTEST_PATH_COMMAND = "x"
        const val START_EXPLORATION_COMMAND = "starte"
        const val START_FASTEST_PATH_COMMAND = "startf"
        const val STOP_COMMAND = "stop"
        const val OK_COMMAND = "ok"
        fun arrowCommand(x: Int, y: Int, direction: Direction): String {
            val directionCommand = when (direction) {
                Direction.UP -> "u"
                Direction.DOWN -> "d"
                Direction.LEFT -> "l"
                Direction.RIGHT -> "r"
            }
            return "arrow{$x,$y,$directionCommand}"
        }

        fun mdfStringCommand(part1: String, part2: String) = "mdf{$part1,$part2}"

        fun robotCenterCommand(cellInfo: CellInfoModel): String {
            val directionCommand = when (cellInfo.direction) {
                Direction.UP -> "u"
                Direction.DOWN -> "d"
                Direction.LEFT -> "l"
                Direction.RIGHT -> "r"
            }
            return "${cellInfo.col},${cellInfo.row},$directionCommand"
        }

        fun robotCenterCommandAndroid(cellInfo: CellInfoModel): String {
            return "center{${robotCenterCommand(cellInfo)}}"
        }

        /**
         * @param parameter: Distance in cm when [movement] is [Movement.MOVE_FORWARD], or angle in degrees when [movement] is [Movement.TURN_RIGHT] or [Movement.TURN_LEFT]
         */
        fun movementWithParameterCommand(movement: Movement, parameter: Int): String {
            val movementCommand = when (movement) {
                Movement.TURN_RIGHT -> TURN_RIGHT_COMMAND
                Movement.MOVE_FORWARD -> MOVE_FORWARD_COMMAND
                Movement.TURN_LEFT -> TURN_LEFT_COMMAND
            }
            return "$movementCommand;$parameter"
        }
    }

    val socketProperty = SimpleObjectProperty<Socket?>(null)
    private var socket by socketProperty
    private var input: ByteReadChannel? = null
    private var output: ByteWriteChannel? = null

    val isConnected get() = socket != null

    val sensedDataChannels = List(NUMBER_OF_SENSORS) { Channel<Int>(Channel.UNLIMITED) }
    val arrowFoundChannel = Channel<String>(Channel.UNLIMITED)

    private val okCommandChannel = Channel<String>(Channel.UNLIMITED)

    @UseExperimental(KtorExperimentalAPI::class)
    suspend fun connect() {
        check(socket == null)
        println("Connecting...")
        socket = aSocket(ActorSelectorManager(Dispatchers.IO)).tcp().connect(hostname, port)
            .also { socket ->
                println("Connected")
                saveJson(ConnectionJson(true))
                input = socket.openReadChannel()
                output = socket.openWriteChannel(autoFlush = true)
            }
    }

    suspend fun startReadingLoop() {
        while (true) {
            val input = checkNotNull(input)
            try {
                val line = withTimeout(TimeUnit.SECONDS.toMillis(60)) { input.readUTF8Line() }
                println("Line read: ${line ?: "<null>"}")
                if (line == null) {
                    reconnect()
                    continue
                }
                processLineRead(line)
            } catch (e: Throwable) {
                System.err.println(e.message)
                reconnect()
            }
        }
    }

    private suspend fun processLineRead(line: String) {
        when {
            line.startsWith("way") -> {
                val indexOfComma = line.indexOf(',')
                val x = line.substring(4 until indexOfComma).toInt()
                val y = line.substring(indexOfComma + 1 until line.length - 1).toInt()
                wayPointChannel.send(x to y)
                println("Process way done")
            }
            line.startsWith("sensor") -> {
                val sensedData = line.substring(6).split(',').map { it.toInt() }
                sensedData.zip(sensedDataChannels).forEach { (data, channel) -> channel.send(data) }
                println("Process sensor done")
            }
            line.startsWith("arrfound") -> {
                arrowFoundChannel.send(line)
                println("Process arrfound done")
            }
            line == START_EXPLORATION_COMMAND || line == START_FASTEST_PATH_COMMAND -> {
                startCommandChannel.send(line)
                println("Process start commands done")
            }
            line == OK_COMMAND -> {
                okCommandChannel.send(line)
                println("Process ok done")
            }
        }
    }

    private suspend fun reconnect() {
        while (true) {
            delay(TimeUnit.SECONDS.toMillis(1))
            disconnect()
            try {
                withTimeout(TimeUnit.SECONDS.toMillis(10)) { connect() }
                break
            } catch (e: Throwable) {
                println(e.message)
                continue
            }
        }
    }

    fun disconnect() {
        socket?.dispose()
        socket = null
        input?.cancel()
        input = null
        output?.close()
        output = null
        saveJson(ConnectionJson(false))
    }

    private suspend fun writeLine(line: String) {
        while (true) {
            try {
                val output = checkNotNull(output)
                output.writeStringUtf8("$line\n")
                println("$line sent")
                return
            } catch (e: Throwable) {
                println(e.message)
                reconnect()
                continue
            }
        }
    }

    private suspend fun sendToAndroid(line: String) {
        writeLine("$ANDROID_PREFIX$line")
    }

    private suspend fun sendToArduino(line: String) {
        writeLine("$ARDUINO_PREFIX$line")
    }

    private suspend fun sendToRpi(line: String) {
        writeLine("$RPI_PREFIX$line")
    }

    suspend fun sendMoveForwardWithDistanceAndWait(numberOfGrids: Int) {
        sendToArduino(movementWithParameterCommand(Movement.MOVE_FORWARD, numberOfGrids * 10))
        okCommandChannel.receive()
    }

    suspend fun sendTurnCommandWithCountAndWait(movement: Movement, turns: Int) {
        sendToArduino(movementWithParameterCommand(movement, turns * 90))
        okCommandChannel.receive()
    }

    suspend fun sendMdfString(part1: String, part2: String) {
        sendToAndroid(mdfStringCommand(part1, part2))
    }

    suspend fun sendCalibrationCommandAndWait() {
        sendToArduino(CALIBRATE_COMMAND)
        okCommandChannel.receive()
    }

    suspend fun sendGetSensorDataCommand() {
        sendToArduino(GET_SENSOR_DATA_COMMAND)
    }

    suspend fun sendRobotCenter(cellInfo: CellInfoModel) {
        sendToRpi(robotCenterCommand(cellInfo))
        sendToAndroid(robotCenterCommandAndroid(cellInfo))
    }

    suspend fun sendArrowCommand(x: Int, y: Int, face: Direction) {
        sendToAndroid(arrowCommand(x, y, face))
    }

    suspend fun sendFastestPathCommandAndWait() {
        sendToArduino(FASTEST_PATH_COMMAND)
        okCommandChannel.receive()
    }

    suspend fun sendStopCommand() {
        sendToAndroid(STOP_COMMAND)
    }
}

private object Reconnect

private fun CoroutineScope.connect(
    hostname: String,
    port: Int,
    lineReadChannel: SendChannel<String>,
    lineWriteChannel: ReceiveChannel<String>,
    reconnectSignalChannel: SendChannel<Reconnect>
) = launch {
    val socket = createSocket(hostname, port)
    val input = socket.openReadChannel()
    val output = socket.openWriteChannel()

    launch {
        while (isActive) {
            val line = try {
                withTimeoutOrNull(TimeUnit.SECONDS.toMillis(60)) { input.readUTF8Line() }
                    .also { println("Read: ${it ?: "<null>"}") }
            } catch (e: Throwable) {
                System.err.println(e.message)
                null
            }
            if (line == null) {
                socket.dispose()
                reconnectSignalChannel.send(Reconnect)
                break
            } else {
                lineReadChannel.send(line)
            }
        }
    }

    launch {
        for (line in lineWriteChannel) {
            try {
                output.writeStringUtf8("$line\n")
                println("Written: $line")
            } catch (e: Throwable) {
                System.err.println(e.message)
                socket.dispose()
                reconnectSignalChannel.send(Reconnect)
            }
        }
    }
}

fun CoroutineScope.processMessages(
    hostname: String,
    port: Int,
    inputMessageChannel: SendChannel<InputMessage>,
    outputMessageChannel: ReceiveChannel<OutputMessage>
) = launch {
    val lineReadChannel = Channel<String>()
    val lineWriteChannel = Channel<String>()
    val reconnectSignalChannel = Channel<Reconnect>()
    var connection = connect(hostname, port, lineReadChannel, lineWriteChannel, reconnectSignalChannel)

    while (isActive) {
        selectUnbiased<Unit> {
            reconnectSignalChannel.onReceive {
                connection.cancel()
                connection = connect(hostname, port, lineReadChannel, lineWriteChannel, reconnectSignalChannel)
            }
            lineReadChannel.onReceive { line ->
                val message = InputMessage.PARSABLES.firstOrNull { it.isParsableFrom(line) }?.fromLine(line)
                if (message == null) {
                    println("Unable to parse $line")
                } else {
                    inputMessageChannel.send(message)
                }
            }
            outputMessageChannel.onReceive {
                lineWriteChannel.send("${it.prefix}$it")
            }
        }
    }
}

@UseExperimental(KtorExperimentalAPI::class)
private suspend fun createSocket(hostname: String, port: Int): Socket {
    var delayTime = TimeUnit.SECONDS.toMillis(1L)
    while (true) {
        try {
            return aSocket(ActorSelectorManager(Dispatchers.IO)).tcp().connect(hostname, port)
        } catch (e: Throwable) {
            System.err.println(e.message)
            delay(delayTime)
            delayTime *= 2
            if (delayTime > TimeUnit.SECONDS.toMillis(5)) {
                delayTime = TimeUnit.SECONDS.toMillis(5)
            }
        }
    }
}
package domain

import io.ktor.network.selector.ActorSelectorManager
import io.ktor.network.sockets.Socket
import io.ktor.network.sockets.aSocket
import io.ktor.network.sockets.openReadChannel
import io.ktor.network.sockets.openWriteChannel
import io.ktor.network.util.ioCoroutineDispatcher
import javafx.beans.property.SimpleObjectProperty
import kotlinx.coroutines.experimental.channels.Channel
import kotlinx.coroutines.experimental.channels.SendChannel
import kotlinx.coroutines.experimental.delay
import kotlinx.coroutines.experimental.io.ByteReadChannel
import kotlinx.coroutines.experimental.io.ByteWriteChannel
import kotlinx.coroutines.experimental.io.readUTF8Line
import kotlinx.coroutines.experimental.io.writeStringUtf8
import kotlinx.coroutines.experimental.withTimeout
import model.CellInfoModel
import tornadofx.*
import java.util.concurrent.TimeUnit

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
        const val RPI_PREFIX = "rp"
        const val MOVE_FORWARD_COMMAND = "w"
        const val TURN_LEFT_COMMAND = "a"
        const val TURN_RIGHT_COMMAND = "d"
        const val GET_SENSOR_DATA_COMMAND = "g"
        const val CALIBRATE_COMMAND = "c"
        const val START_EXPLORATION_COMMAND = "starte"
        const val START_FASTEST_PATH_COMMAND = "startf"
        const val OK_COMMAND = "ok"
        fun obstacleCommand(x: Int, y: Int) = "obstacle{$x,$y}"
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
    }

    val socketProperty = SimpleObjectProperty<Socket?>(null)
    private var socket by socketProperty
    private var input: ByteReadChannel? = null
    private var output: ByteWriteChannel? = null

    val isConnected get() = socket != null

    val sensedDataChannels = List(NUMBER_OF_SENSORS) { Channel<Int>() }
    val arrowFoundChannel = Channel<String>()

    private val okCommandChannel = Channel<String>()

    suspend fun connect() {
        check(socket == null)
        println("Connecting...")
        socket = aSocket(ActorSelectorManager(ioCoroutineDispatcher)).tcp().connect(hostname, port)
            .also { socket ->
                println("Connected")
                input = socket.openReadChannel()
                output = socket.openWriteChannel(autoFlush = true)
            }
    }

    suspend fun startReadingLoop() {
        while (true) {
            val input = checkNotNull(input)
            try {
                println("Reading...")
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
            }
            line.startsWith("sensor") -> {
                val sensedData = line.substring(6).split(',').map { it.toInt() }
                sensedData.zip(sensedDataChannels).forEach { (data, channel) -> channel.send(data) }
            }
            line.startsWith("arrfound") -> arrowFoundChannel.send(line)
            line == START_EXPLORATION_COMMAND || line == START_FASTEST_PATH_COMMAND ->
                startCommandChannel.send(line)
            line == OK_COMMAND -> okCommandChannel.send(line)
        }
    }

    private suspend fun reconnect() {
        while (true) {
            delay(TimeUnit.SECONDS.toMillis(1))
            socket?.dispose()
            socket = null
            try {
                withTimeout(TimeUnit.SECONDS.toMillis(10)) { connect() }
                break
            } catch (e: Throwable) {
                println(e.message)
                continue
            }
        }
    }

    private suspend fun writeLine(line: String) {
        println("Sending $line...")
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

    suspend fun sendMovementAndWait(movement: Movement) {
        when (movement) {
            Movement.TURN_RIGHT -> sendToArduino(TURN_RIGHT_COMMAND)
            Movement.MOVE_FORWARD -> sendToArduino(MOVE_FORWARD_COMMAND)
            Movement.TURN_LEFT -> sendToArduino(TURN_LEFT_COMMAND)
        }
        okCommandChannel.receive()
    }

    suspend fun sendObstacle(row: Int, col: Int) {
        sendToAndroid(obstacleCommand(col, row))
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
    }

    suspend fun sendArrowCommand(x: Int, y: Int, face: Direction) {
        sendToAndroid(arrowCommand(x, y, face))
    }
}
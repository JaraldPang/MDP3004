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
import tornadofx.*
import java.util.concurrent.TimeUnit

class Connection(
    private val hostname: String = HOSTNAME,
    private val port: Int = PORT,
    private val wayPointChannel: SendChannel<Pair<Int, Int>>
) {
    companion object {
        const val HOSTNAME = "192.168.17.1"
        const val PORT = 45000
        const val ANDROID_PREFIX = "an"
        const val ARDUINO_PREFIX = "ar"
        const val MOVE_FORWARD_COMMAND = "w"
        const val TURN_LEFT_COMMAND = "a"
        const val TURN_RIGHT_COMMAND = "d"
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

        const val CALIBRATION = "calibration"

        fun mdfStringCommand(part1: String, part2: String) = "mdf{$part1,$part2}"
    }

    val socketProperty = SimpleObjectProperty<Socket?>(null)
    private var socket by socketProperty
    private var input: ByteReadChannel? = null
    private var output: ByteWriteChannel? = null

    val isConnected get() = socket != null

    val sensedDataChannels = List(NUMBER_OF_SENSORS) { Channel<Int>() }

    suspend fun connect() {
        check(socket == null)
        println("Connecting...")
        socket = aSocket(ActorSelectorManager(ioCoroutineDispatcher)).tcp().connect(hostname, port).also { socket ->
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
                val line = withTimeout(15, TimeUnit.SECONDS) { input.readUTF8Line() }
                println("Line read: ${line ?: "<null>"}")
                if (line == null) {
                    reconnect()
                    continue
                }
                if (line.startsWith("way")) {
                    val indexOfComma = line.indexOf(',')
                    val x = line.substring(4 until indexOfComma).toInt()
                    val y = line.substring(indexOfComma + 1 until line.length - 1).toInt()
                    wayPointChannel.send(x to y)
                } else if (line.startsWith("sensor")) {
                    val sensedData = line.substring(6).split(',').map { it.toInt() }
                    sensedData.zip(sensedDataChannels).forEach { (data, channel) -> channel.send(data) }
                }
            } catch (e: Throwable) {
                println(e.message)
                reconnect()
            }
        }
    }

    private suspend fun reconnect() {
        while (true) {
            delay(1, TimeUnit.SECONDS)
            socket?.dispose()
            socket = null
            try {
                withTimeout(5, TimeUnit.SECONDS) { connect() }
                break
            } catch (e: Throwable) {
                println(e.message)
                continue
            }
        }
    }

    private suspend fun writeLine(line: String) {
        while (true) {
            try {
                println("Sending $line...")
                val output = checkNotNull(output)
                output.writeStringUtf8("$line\n")
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

    suspend fun sendMovement(movement: Movement) {
        when (movement) {
            Movement.TURN_RIGHT -> {
                sendToAndroid(TURN_RIGHT_COMMAND)
                sendToArduino(TURN_RIGHT_COMMAND)
            }
            Movement.MOVE_FORWARD -> {
                sendToAndroid(MOVE_FORWARD_COMMAND)
                sendToArduino(MOVE_FORWARD_COMMAND)
            }
            Movement.TURN_LEFT -> {
                sendToAndroid(TURN_LEFT_COMMAND)
                sendToArduino(TURN_LEFT_COMMAND)
            }
        }
    }

    suspend fun sendObstacle(row: Int, col: Int) {
        sendToAndroid(obstacleCommand(col, row))
    }

    suspend fun sendMdfString(part1: String, part2: String) {
        sendToAndroid(mdfStringCommand(part1, part2))
    }

    suspend fun sendCalibrationCommand() {
        sendToArduino(CALIBRATION)
    }
}
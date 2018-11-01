package controller

import com.fasterxml.jackson.annotation.JsonIgnore
import com.fasterxml.jackson.annotation.JsonProperty
import domain.*
import javafx.beans.property.SimpleBooleanProperty
import javafx.beans.property.SimpleObjectProperty
import kotlinx.atomicfu.AtomicRef
import kotlinx.atomicfu.atomic
import kotlinx.atomicfu.update
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.Job
import kotlinx.coroutines.channels.Channel
import kotlinx.coroutines.launch
import model.CellInfoModel
import model.ConfigurationModel
import model.MazeModel
import tornadofx.*
import java.io.FileInputStream
import java.io.FileOutputStream
import java.io.PrintWriter

enum class AppState {
    IDLE,
    EXPLORATION,
    FASTEST_PATH
}

data class AppStateJson(@get:JsonProperty("appState") val appState: AppState) : JsonSerializable {
    @JsonIgnore
    override val filename = "AppState"
}

enum class ConnectionState {
    DISCONNECTED,
    CONNECTING,
    CONNECTED,
    DISCONNECTING
}

inline fun AtomicRef<AppState>.withState(runningState: AppState, restoredState: AppState, block: () -> Unit) {
    if (!compareAndSet(restoredState, runningState)) {
        return
    }
    saveJson(AppStateJson(runningState))
    block()
    update { restoredState }
    saveJson(AppStateJson(restoredState))
}

class AtomicProperty<T>(initial: T) {
    private val atomicRef = atomic(initial)
    val property = SimpleObjectProperty(initial)

    fun compareAndSet(expect: T, update: T): Boolean {
        val result = atomicRef.compareAndSet(expect, update)
        if (result) {
            property.value = update
        }
        return result
    }

    fun update(function: (T) -> T) {
        val cur = atomicRef.value
        val upd = function(cur)
        atomicRef.update { upd }
        property.value = upd
    }
}

class MainController : Controller(), CoroutineScope {
    private val job = Job()
    override val coroutineContext get() = Dispatchers.Main + job

    val centerCell: CellInfoModel by inject()
    val explorationMaze = MazeModel()
    val realMaze = MazeModel()
    val configurationModel: ConfigurationModel by inject()
    val displayRealMaze = SimpleBooleanProperty(true)
    private val appState = atomic(AppState.IDLE)

    val connectionStateProperty = AtomicProperty(ConnectionState.DISCONNECTED)

    private val wayPointChannel = Channel<Pair<Int, Int>>(Channel.UNLIMITED)
    private val startCommandChannel = Channel<String>(Channel.UNLIMITED)

    private val connection = Connection(wayPointChannel = wayPointChannel, startCommandChannel = startCommandChannel)

    init {
        with(centerCell) {
            row = 1
            col = 1
            direction = Direction.UP
        }
        explorationMaze.initExploration()

        launch {
            for ((x, y) in wayPointChannel) {
                configurationModel.wayPointX = x
                configurationModel.wayPointY = y
            }
        }

        launch {
            for (command in startCommandChannel) {
                when (command) {
                    Connection.START_EXPLORATION_COMMAND -> runExploration()
                    Connection.START_FASTEST_PATH_COMMAND -> runFastestPath()
                }
            }
        }

//        launch {
//            tryToRecover()
//        }
    }

    fun runExploration() = launch {
        appState.withState(AppState.EXPLORATION, AppState.IDLE) {
            val speed = configurationModel.speed
            val robot = Robot(centerCell, explorationMaze, speed, connection)
            val sensors = if (!connection.isConnected) {
                listOf(2, 3, 4, 5, 6).map { SimulatedSensor(it, 0..2, robot, realMaze) }
            } else {
                listOf(3, 4, 5, 6, 8, 2).zip(connection.sensedDataChannels).map { (position, channel) ->
                    val senseRange = when (position) {
                        8 -> SENSE_RANGE_LONG
                        5 -> SENSE_RANGE_RIGHT
                        else -> SENSE_RANGE_SHORT
                    }
                    ActualSensor(position, senseRange, channel)
                }
            }
            robot.setSensors(sensors)
            displayRealMaze.value = false
            val coverageLimit = configurationModel.coverage
            val timeLimit = configurationModel.time
            if (connection.isConnected) {
                connection.sendRobotCenter(robot.centerCell)
            }
            when {
                speed != null && coverageLimit != null -> {
                    val exploration = CoverageLimitedExploration(robot, connection, coverageLimit)
                    exploration.explore()
                }
                speed != null && timeLimit != null -> {
                    val exploration = TimeLimitedExploration(robot, connection, timeLimit)
                    exploration.explore()
                }
                else -> {
                    val exploration = Exploration(robot, connection)
                    exploration.explore()
                }
            }
            val part1 = robot.explorationMaze.outputMapDescriptorPart1()
            val part2 = robot.explorationMaze.outputMapDescriptorPart2()
            configurationModel.mapDescriptorPart1 = part1
            configurationModel.mapDescriptorPart2 = part2
            if (connection.isConnected) {
                connection.sendMdfString(part1, part2)
                for (arrow in robot.correctArrows) {
                    connection.sendArrowCommand(arrow.col, arrow.row, arrow.direction)
                }
            }
        }
    }


    fun runFastestPath() = launch {
        appState.withState(AppState.FASTEST_PATH, AppState.IDLE) {
            if (connection.isConnected) {
                connection.sendFastestPathCommandAndWait()
            }
            val part1 = configurationModel.mapDescriptorPart1
            val part2 = configurationModel.mapDescriptorPart2
            if (part1 != null && part1.isNotBlank() && part2 != null && part2.isNotEmpty() && connection.isConnected) {
                connection.sendMdfString(part1, part2)
            }
            val speed = configurationModel.speed
            val robot = Robot(centerCell, explorationMaze, speed, connection)
            displayRealMaze.value = false
            val wayPointX = configurationModel.wayPointX
            val wayPointY = configurationModel.wayPointY
            if (wayPointX != null && wayPointY != null) {
                val fastestPath = FastestPathWithWayPoint(connection, robot, wayPointY to wayPointX)
                fastestPath.runFastestPath()
            } else {
                val fastedPath = FastestPath(connection, robot)
                fastedPath.runFastestPath()
            }
        }
    }


    fun loadMapDescriptor() {
        val filename = configurationModel.filename
        if (filename != null) {
            val lines = FileInputStream("mazes/$filename").bufferedReader().readLines()
            if (lines.size == 2) {
                realMaze.parseFromMapDescriptors(lines[0], lines[1])
            }
        }
    }

    fun saveMapDescriptor() {
        val filename = configurationModel.filename ?: return
        val part1 = configurationModel.mapDescriptorPart1 ?: return
        val part2 = configurationModel.mapDescriptorPart2 ?: return
        if (!filename.isBlank() && !part1.isBlank() && !part2.isBlank()) {
            PrintWriter(FileOutputStream("mazes/$filename").bufferedWriter(), true).use {
                it.println(part1)
                it.println(part2)
            }
            println("Saved to .mazes/$filename")
        }
    }

    fun connect() = launch {
        if (connectionStateProperty.compareAndSet(ConnectionState.DISCONNECTED, ConnectionState.CONNECTING)) {
            connection.connect()
            connectionStateProperty.update { ConnectionState.CONNECTED }
            connection.startReadingLoop()
        }
    }

    fun disconnect() {
        if (connectionStateProperty.compareAndSet(ConnectionState.CONNECTED, ConnectionState.DISCONNECTING)) {
            connection.disconnect()
            connectionStateProperty.update { ConnectionState.DISCONNECTED }
        }
    }

    fun reset() {
        centerCell.reset()
        explorationMaze.reset()
        explorationMaze.initExploration()
        realMaze.reset()
        configurationModel.reset()
        displayRealMaze.value = true
    }

    private suspend fun tryToRecover() {
        val appState = loadJson<AppStateJson>("AppState") ?: return
        if (appState.appState != AppState.IDLE) {
            val centerCell = loadJson<CenterCell>("CenterCell") ?: return
            with(this.centerCell) {
                row = centerCell.row
                col = centerCell.col
                direction = centerCell.direction
            }
            val explorationMaze = loadJson<ExplorationMaze>("ExplorationMaze") ?: return
            for (row in 0 until MAZE_ROWS) {
                for (col in 0 until MAZE_COLUMNS) {
                    this.explorationMaze[row, col] = explorationMaze.explorationMaze[row][col]
                }
            }
            val connection = loadJson<ConnectionJson>("Connection")
            if (connection?.isConnected == true) {
                connect().join()
            }
            when (appState.appState) {
                AppState.EXPLORATION -> runExploration()
                AppState.FASTEST_PATH -> runFastestPath()
                else -> {
                }
            }
        }
    }
}
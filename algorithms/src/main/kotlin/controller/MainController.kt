package controller

import domain.*
import javafx.beans.property.SimpleBooleanProperty
import kotlinx.coroutines.experimental.Dispatchers
import kotlinx.coroutines.experimental.GlobalScope
import kotlinx.coroutines.experimental.channels.actor
import kotlinx.coroutines.experimental.channels.consumeEach
import kotlinx.coroutines.experimental.javafx.JavaFx
import kotlinx.coroutines.experimental.launch
import model.CellInfoModel
import model.ConfigurationModel
import model.MazeModel
import tornadofx.*
import java.io.FileInputStream
import java.io.FileOutputStream
import java.io.PrintWriter

class MainController : Controller() {
    val centerCell: CellInfoModel by inject()
    val explorationMaze = MazeModel()
    val realMaze = MazeModel()
    val configurationModel: ConfigurationModel by inject()
    val displayRealMaze = SimpleBooleanProperty(true)
    private val wayPointChannel = GlobalScope.actor<Pair<Int, Int>>(Dispatchers.JavaFx) {
        consumeEach { (x, y) ->
            configurationModel.wayPointX = x
            configurationModel.wayPointY = y
        }
    }

    private val startCommandChannel = GlobalScope.actor<String>(Dispatchers.JavaFx) {
        consumeEach {
            when (it) {
                Connection.START_EXPLORATION_COMMAND -> runExploration()
                Connection.START_FASTEST_PATH_COMMAND -> runFastestPath()
            }
        }
    }

    val connection = Connection(
        wayPointChannel = wayPointChannel,
        startCommandChannel = startCommandChannel
    )

    init {
        with(centerCell) {
            row = 1
            col = 1
            direction = Direction.UP
        }
        explorationMaze.initExploration()
    }

    fun runExploration() {
        GlobalScope.launch(Dispatchers.JavaFx) {
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
            }
        }
    }

    fun runFastestPath() {
        GlobalScope.launch(Dispatchers.JavaFx) {
            if (connection.isConnected) {
                connection.sendFastestPathCommandAndWait()
            }
            val speed = configurationModel.speed
            val robot = Robot(centerCell, explorationMaze, speed, connection)
            displayRealMaze.value = false
            val wayPointX = configurationModel.wayPointX
            val wayPointY = configurationModel.wayPointY
            if (wayPointX != null && wayPointY != null) {
                val fastestPath = FastestPathWithWayPoint(robot, wayPointY to wayPointX)
                fastestPath.runFastestPath()
            } else {
                val fastedPath = FastestPath(robot)
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

    fun connect() {
        GlobalScope.launch(Dispatchers.JavaFx) {
            connection.connect()
            connection.startReadingLoop()
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
}
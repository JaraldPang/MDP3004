package controller

import domain.*
import javafx.beans.property.SimpleBooleanProperty
import kotlinx.coroutines.experimental.Dispatchers
import kotlinx.coroutines.experimental.GlobalScope
import kotlinx.coroutines.experimental.channels.actor
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
        for ((x, y) in this) {
            configurationModel.wayPointX = x
            configurationModel.wayPointY = y
        }
    }

    val connection = Connection(wayPointChannel = wayPointChannel)

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
                listOf(3, 4, 5, 6, 7, 2).zip(connection.sensedDataChannels).map { (position, channel) ->
                    ActualSensor(position, 0..2, channel)
                }
            }
            robot.setSensors(sensors)
            displayRealMaze.value = false
            val coverageLimit = configurationModel.coverage
            val timeLimit = configurationModel.time
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
            val part1 = robot.explorationMaze.outputExploredUnexploredString()
            val part2 = robot.explorationMaze.outputEmptyObstacleString()
            configurationModel.mapDescriptorPart1 = part1
            configurationModel.mapDescriptorPart2 = part2
            if (connection.isConnected) {
                connection.sendMdfString(part1, part2)
            }
        }
    }

    fun runFastestPath() {
        GlobalScope.launch(Dispatchers.JavaFx) {
            val speed = configurationModel.speed
            val robot = Robot(centerCell, explorationMaze, speed, connection)
            displayRealMaze.value = false
            val wayPointX = configurationModel.wayPointX
            val wayPointY = configurationModel.wayPointY
            if (speed != null && wayPointX != null && wayPointY != null) {
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
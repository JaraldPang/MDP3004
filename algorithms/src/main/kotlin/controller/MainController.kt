package controller

import domain.*
import javafx.beans.property.SimpleBooleanProperty
import kotlinx.coroutines.experimental.Dispatchers
import kotlinx.coroutines.experimental.GlobalScope
import kotlinx.coroutines.experimental.javafx.JavaFx
import kotlinx.coroutines.experimental.launch
import model.CellInfoModel
import model.ConfigurationModel
import model.MazeModel
import tornadofx.Controller

class MainController : Controller() {
    val centerCell: CellInfoModel by inject()
    val explorationMaze = MazeModel()
    val realMaze = MazeModel()
    val configurationModel: ConfigurationModel by inject()
    val displayRealMaze = SimpleBooleanProperty(true)

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
            val robot = Robot(centerCell, explorationMaze, speed)
            val sensors = listOf(2, 3, 4, 5, 6).map {
                SimulatedSensor(it, 0..2, robot, realMaze)
            }
            robot.setSensors(sensors)
            displayRealMaze.value = false
            val coverageLimit = configurationModel.coverage
            val timeLimit = configurationModel.time
            when {
                speed != null && coverageLimit != null -> {
                    val exploration = CoverageLimitedExploration(robot, coverageLimit)
                    exploration.explore()
                }
                speed != null && timeLimit != null -> {
                    val exploration = TimeLimitedExploration(robot, timeLimit)
                    exploration.explore()
                }
                else -> {
                    val exploration = Exploration(robot)
                    exploration.explore()
                }
            }
            configurationModel.mapDescriptorPart1 = robot.explorationMaze.outputExploredUnexploredString()
            configurationModel.mapDescriptorPart2 = robot.explorationMaze.outputEmptyObstacleString()
        }
    }

    fun runFastestPath() {
        GlobalScope.launch(Dispatchers.JavaFx) {
            val speed = configurationModel.speed
            val robot = Robot(centerCell, explorationMaze, speed)
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
}
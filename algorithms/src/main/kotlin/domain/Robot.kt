package domain

import kotlinx.coroutines.experimental.Dispatchers
import kotlinx.coroutines.experimental.GlobalScope
import kotlinx.coroutines.experimental.delay
import kotlinx.coroutines.experimental.javafx.JavaFx
import kotlinx.coroutines.experimental.launch
import model.CellInfoModel
import model.MazeModel
import model.MovementInfo

class Robot(
    val centerCell: CellInfoModel,
    val explorationMaze: MazeModel,
    private val speed: Int?,
    private val connection: Connection
) {
    val isAtStartZone get() = centerCell.row == 1 && centerCell.col == 1

    private var sensors = listOf<Sensor>()

    private val arrowsFound = hashMapOf<Pair<Int, Int>, Direction>()

    private var movementCount = 0

    init {
        startProcessingArrowFound()
    }

    private fun startProcessingArrowFound() {
        if (connection.isConnected) {
            GlobalScope.launch(Dispatchers.JavaFx) {
                // Hopefully the arrowFound message comes after the obstacle is sensed
                for (arrowFound in connection.arrowFoundChannel) {
                    val (x, y, directionCommand) = arrowFound.substring(8).split(",")
                    val row = y.toInt()
                    val col = x.toInt()
                    val direction = when (directionCommand) {
                        "u" -> Direction.UP
                        "d" -> Direction.DOWN
                        "l" -> Direction.LEFT
                        "r" -> Direction.RIGHT
                        else -> throw IllegalStateException("Unknown arrow found direction: $directionCommand")
                    }
                    if (MazeModel.isOutsideOfMaze(row, col)) {
                        continue
                    }
                    if (explorationMaze[row][col] == CELL_OBSTACLE) {
                        println("Received arrow at ($col,$row,$directionCommand)")
                        if (connection.isConnected) {
                            connection.sendArrowCommand(col, row, direction)
                        }
                    } else {
                        // The corresponding grid is not an obstacle (yet), cache it and check later
                        println("Received arrow at ($col,$row,$directionCommand) but caching")
                        arrowsFound[row to col] = direction
                    }
                }
            }
        }
    }

    fun setSensors(sensors: List<Sensor>) {
        this.sensors = sensors
    }

    suspend fun sense(): Boolean {
        if (connection.isConnected) {
            connection.sendGetSensorDataCommand()
        }
//        delay(100)
        var hasUpdateInMaze = false
        val (centerRow, centerCol, direction) = centerCell
        for (sensor in sensors) {
            val sensedDistance = sensor.sense()
            val (rowDiff, colDiff, rowInc, colInc) = SENSOR_INFO[sensor.position][direction.ordinal]
            if (sensedDistance == 0) {  // Found an obstacle next to the robot
                val row = centerRow + rowDiff
                val col = centerCol + colDiff
                if (!MazeModel.isOutsideOfMaze(row, col)) {
                    if (explorationMaze[row][col] != CELL_UNKNOWN && explorationMaze[row][col] != CELL_OBSTACLE) {
                        println("Warning: maze[$row][$col] was ${explorationMaze[row][col]} but setting to CELL_OBSTACLE")
                    }
                    explorationMaze[row][col] = CELL_OBSTACLE
                    hasUpdateInMaze = true
                    // Find an obstacle, check if any arrows found on it
                    val arrowDirection = arrowsFound.remove(row to col)
                    if (arrowDirection != null) {
                        if (connection.isConnected) {
                            connection.sendArrowCommand(col, row, arrowDirection)
                        }
                    }
                }
            } else if (sensedDistance in sensor.senseRange) {   // Accurate reading from the sensor
                // e.g., 2 means 2 grids away from the robot is empty
                for (i in 0 until sensedDistance) {
                    val row = centerRow + rowDiff + rowInc * i
                    val col = centerCol + colDiff + colInc * i
                    if (MazeModel.isOutsideOfMaze(row, col) || explorationMaze[row][col] == CELL_OBSTACLE) {
                        break
                    }
                    if (explorationMaze[row][col] == CELL_UNKNOWN) {
                        explorationMaze[row][col] = CELL_SENSED
                        hasUpdateInMaze = true
                    }
                }
            }
        }
        if (connection.isConnected) {
            val part1 = explorationMaze.outputMapDescriptorPart1()
            val part2 = explorationMaze.outputMapDescriptorPart2()
            connection.sendMdfString(part1, part2)
        }
        return hasUpdateInMaze
    }

    suspend fun move(movement: Movement) {
        if (!connection.isConnected) {
            delay(1000L / (speed ?: 3))
        }
        movementCount++
        if (connection.isConnected) {
            when (movement) {
                Movement.TURN_LEFT, Movement.TURN_RIGHT -> connection.sendTurnCommandWithCountAndWait(movement, 1)
                Movement.MOVE_FORWARD -> connection.sendMoveForwardWithDistanceAndWait(1)
            }
        }
        when (movement) {   // Update on UI
            Movement.TURN_RIGHT -> turnRight()
            Movement.TURN_LEFT -> turnLeft()
            Movement.MOVE_FORWARD -> moveForward()
        }
        if (connection.isConnected) {
            tryCalibrate()
        }
    }

    private suspend fun turnLeft() {
        println("Command TURN_LEFT")
        centerCell.direction = centerCell.direction.turnLeft()
        if (connection.isConnected) {
            connection.sendRobotCenter(centerCell)
        }
    }

    private suspend fun turnRight() {
        println("Command TURN_RIGHT")
        centerCell.direction = centerCell.direction.turnRight()
        if (connection.isConnected) {
            connection.sendRobotCenter(centerCell)
        }
    }

    private suspend fun moveForward() {
        println("Command MOVE_FORWARD")
        val (centerRow, centerCol, currentDirection) = centerCell
        val (rowDiff, colDiff) = NEXT_CELL[MovementInfo(Movement.MOVE_FORWARD, currentDirection)]
            ?: throw IllegalStateException()
        if (rowDiff != 0) {
            val frontRow = centerRow + rowDiff + rowDiff
            for (col in centerCol - 1..centerCol + 1) {
                explorationMaze[frontRow][col]++
            }
            centerCell.row += rowDiff
        } else {
            check(colDiff != 0)
            val frontCol = centerCol + colDiff + colDiff
            for (row in centerRow - 1..centerRow + 1) {
                explorationMaze[row][frontCol]++
            }
            centerCell.col += colDiff
        }
        if (connection.isConnected) {
            connection.sendRobotCenter(centerCell)
        }
    }

    suspend fun goToStartZone() {
        val fastestPathMaze = explorationMaze.copy()
        val path = findFastestPathToDestination(fastestPathMaze, centerCell, 1 to 1)
            .asSequence()
            .filter { it.isNotEmpty() }
            .map {
                val (row, col, direction) = it.last()
                when (direction) {
                    Direction.UP -> it
                    Direction.DOWN -> it +
                            CellInfoModel(row, col, direction.turnLeft()) +
                            CellInfoModel(row, col, direction.turnLeft().turnLeft())
                    Direction.LEFT -> it + CellInfoModel(row, col, direction.turnRight())
                    Direction.RIGHT -> it + CellInfoModel(row, col, direction.turnLeft())
                }
            }
            .minBy { it.size } ?: throw IllegalStateException("Unable to find way to go home")
        val movements = path.toMovements()
        moveFollowingMovements(movements)
    }

    suspend fun turnToFaceUp() {
        when (centerCell.direction) {
            Direction.DOWN -> {
                move(Movement.TURN_LEFT)
                move(Movement.TURN_LEFT)
            }
            Direction.LEFT -> move(Movement.TURN_RIGHT)
            Direction.RIGHT -> move(Movement.TURN_LEFT)
            else -> {
            }
        }
    }

    suspend fun moveFollowingMovements(movements: List<Movement>) {
        if (movements.isEmpty()) {
            return
        }
        val compactList = movements.compact()
        println("CompactList: $compactList")
        for ((movement, count) in compactList) {
            if (movement == Movement.MOVE_FORWARD) {
                if (connection.isConnected) {
                    connection.sendMoveForwardWithDistanceAndWait(count)
                }
                for (i in 0 until count) {
                    if (!connection.isConnected) {
                        delay(1000L / (speed ?: 3))
                    }
                    moveForward()
                }
            } else {
                if (connection.isConnected) {
                    delay(250L)
                    connection.sendTurnCommandWithCountAndWait(movement, count)
                    delay(250L)
                }
                for (i in 0 until count) {
                    if (!connection.isConnected) {
                        delay(1000L / (speed ?: 3))
                    }
                    if (movement == Movement.TURN_LEFT) {
                        turnLeft()
                    } else {
                        turnRight()
                    }
                }
            }
            movementCount++
            if (connection.isConnected) {
                tryCalibrate()
            }
        }
    }

    private suspend fun tryCalibrate() {
        if (movementCount >= 5) {
            val rightSide = explorationMaze.getSide(centerCell.copy(), Movement.TURN_RIGHT)
            val frontSide = explorationMaze.getSide(centerCell.copy(), Movement.MOVE_FORWARD)
            val leftSide = explorationMaze.getSide(centerCell.copy(), Movement.TURN_LEFT)
            if (frontSide[0] == CELL_OBSTACLE && frontSide[2] == CELL_OBSTACLE) {
                connection.sendCalibrationCommandAndWait()
                movementCount = 0
            } else if (rightSide[0] == CELL_OBSTACLE && rightSide[2] == CELL_OBSTACLE) {
                move(Movement.TURN_RIGHT)
                connection.sendCalibrationCommandAndWait()
                move(Movement.TURN_LEFT)
                movementCount = 0
            } else if (leftSide[0] == CELL_OBSTACLE && leftSide[0] == CELL_OBSTACLE) {
                move(Movement.TURN_LEFT)
                connection.sendCalibrationCommandAndWait()
                move(Movement.TURN_RIGHT)
                movementCount = 0
            }
        }
    }
}

private fun List<Movement>.compact(): List<Pair<Movement, Int>> {
    val compactList = mutableListOf<Pair<Movement, Int>>()
    compactList += first() to 1
    for (i in 1 until size) {
        val movement = this[i]
        val (lastMovement, count) = compactList.last()
        val countThreshold = when (movement) {
            Movement.TURN_LEFT, Movement.TURN_RIGHT -> 2
            Movement.MOVE_FORWARD -> 3
        }
        if (movement == lastMovement && count < countThreshold) {
            compactList[compactList.lastIndex] = lastMovement to count + 1
        } else {
            compactList += movement to 1
        }
    }
    return compactList
}
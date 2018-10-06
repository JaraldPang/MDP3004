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
                    if (explorationMaze[row][col] == CELL_OBSTACLE) {
                        if (connection.isConnected) {
                            connection.sendArrowCommand(col, row, direction)
                        }
                    } else {
                        // The corresponding grid is not an obstacle (yet), cache it and check later
                        arrowsFound[row to col] = direction
                    }
                }
            }
        }
    }

    fun setSensors(sensors: List<Sensor>) {
        this.sensors = sensors
    }

    suspend fun sense() {
        if (connection.isConnected) {
            connection.sendGetSensorDataCommand()
        }
//        delay(100)
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
                    } else {
                        explorationMaze[row][col] = CELL_OBSTACLE
                        // Find an obstacle, check if any arrows found on it
                        val arrowDirection = arrowsFound.remove(row to col)
                        if (arrowDirection != null) {
                            if (connection.isConnected) {
                                connection.sendArrowCommand(col, row, arrowDirection)
                            }
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
                    }
                }
            }
        }
        if (connection.isConnected) {
            val part1 = explorationMaze.outputMapDescriptorPart1()
            val part2 = explorationMaze.outputMapDescriptorPart2()
            connection.sendMdfString(part1, part2)
        }
    }

    suspend fun move(movement: Movement) {
        println("Command $movement")
        if (!connection.isConnected) {
            delay(1000L / (speed ?: 3))
        }
        movementCount++
        if (connection.isConnected) {
            connection.sendRobotCenter(centerCell)
            connection.sendMovementAndWait(movement)
            if (movementCount % 5 == 0) {
                connection.sendCalibrationCommandAndWait()
            }
        }
        when (movement) {   // Update on UI
            Movement.TURN_RIGHT -> turnRight()
            Movement.TURN_LEFT -> turnLeft()
            Movement.MOVE_FORWARD -> moveForward()
        }
    }

    private fun turnLeft() {
        centerCell.direction = centerCell.direction.turnLeft()
    }

    private fun turnRight() {
        centerCell.direction = centerCell.direction.turnRight()
    }

    private fun moveForward() {
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
                for (i in 0 until count) {
                    if (!connection.isConnected) {
                        delay(1000L / (speed ?: 3))
                    }
                    moveForward()
                }
                if (connection.isConnected) {
                    connection.sendMoveForwardWithDistanceAndWait(count)
                }
            } else {
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
                if (connection.isConnected) {
                    connection.sendTurnCommandWithCountAndWait(movement, count)
                }
            }
            movementCount++
            if (connection.isConnected) {
                if (movementCount % 5 == 0) {
                    connection.sendCalibrationCommandAndWait()
                }
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
        if (movement == lastMovement && count < 3) {
            compactList[compactList.lastIndex] = lastMovement to count + 1
        } else {
            compactList += movement to 1
        }
    }
    return compactList
}
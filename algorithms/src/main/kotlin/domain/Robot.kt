package domain

import kotlinx.coroutines.experimental.delay
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

    var movementCount = 0
        private set

    fun setSensors(sensors: List<Sensor>) {
        this.sensors = sensors
    }

    suspend fun sense() {
        val (centerRow, centerCol, direction) = centerCell
        for (sensor in sensors) {
            val sensedDistance = sensor.sense()
            val (rowDiff, colDiff, rowInc, colInc) = SENSOR_INFO[sensor.position][direction.ordinal]
            if (sensedDistance == -1) {
                for (i in sensor.senseRange) {
                    val row = centerRow + rowDiff + rowInc * i
                    val col = centerCol + colDiff + colInc * i
                    if (!MazeModel.isOutsideOfMaze(row, col) && explorationMaze[row][col] == CELL_UNKNOWN) {
                        explorationMaze[row][col] = CELL_SENSED
                    }
                }
            } else {
                for (i in sensor.senseRange.first..sensedDistance) {
                    val row = centerRow + rowDiff + rowInc * i
                    val col = centerCol + colDiff + colInc * i
                    if (!MazeModel.isOutsideOfMaze(row, col) && explorationMaze[row][col] == CELL_UNKNOWN) {
                        if (i == sensedDistance) {
                            explorationMaze[row][col] = CELL_OBSTACLE
                            if (connection.isConnected) {
                                connection.sendObstacle(row, col)
                            }
                        } else {
                            explorationMaze[row][col] = CELL_SENSED
                        }
                    }
                }
            }
        }
    }

    suspend fun turnLeft() {
        if (!connection.isConnected) {
            delay(1000L / (speed ?: 3))
        }
        centerCell.direction = centerCell.direction.turnLeft()
        movementCount++
        if (connection.isConnected) {
            connection.sendMovement(Movement.TURN_LEFT)
        }
    }

    suspend fun turnRight() {
        if (!connection.isConnected) {
            delay(1000L / (speed ?: 3))
        }
        centerCell.direction = centerCell.direction.turnRight()
        movementCount++
        if (connection.isConnected) {
            connection.sendMovement(Movement.TURN_RIGHT)
        }
    }

    suspend fun moveForward() {
        if (!connection.isConnected) {
            delay(1000L / (speed ?: 3))
        }
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
        movementCount++
        if (connection.isConnected) {
            connection.sendMovement(Movement.MOVE_FORWARD)
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
                turnLeft()
                turnLeft()
            }
            Direction.LEFT -> turnRight()
            Direction.RIGHT -> turnLeft()
            else -> {
            }
        }
    }

    suspend fun moveFollowingMovements(movements: List<Movement>) {
        for (movement in movements) {
            when (movement) {
                Movement.TURN_RIGHT -> turnRight()
                Movement.MOVE_FORWARD -> moveForward()
                Movement.TURN_LEFT -> turnLeft()
            }
        }
    }
}
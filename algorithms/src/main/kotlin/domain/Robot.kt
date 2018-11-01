package domain

import com.fasterxml.jackson.annotation.JsonIgnore
import com.fasterxml.jackson.annotation.JsonProperty
import kotlinx.coroutines.*
import model.CellInfoModel
import model.MazeModel
import model.MovementInfo
import kotlin.coroutines.CoroutineContext
import kotlin.math.min

data class ExplorationMaze(
    @get:JsonProperty("explorationMaze") val explorationMaze: List<List<Int>>
) : JsonSerializable {
    @JsonIgnore
    override val filename = "ExplorationMaze"
}

data class CenterCell(
    @get:JsonProperty("row") val row: Int,
    @get:JsonProperty("col") val col: Int,
    @get:JsonProperty("direction") val direction: Direction
) : JsonSerializable {
    @JsonIgnore
    override val filename = "CenterCell"
}

data class Arrows(@get:JsonProperty("arrows") val arrows: List<CenterCell>) : JsonSerializable {
    @JsonIgnore
    override val filename = "Arrows"
}

class Robot(
    val centerCell: CellInfoModel,
    val explorationMaze: MazeModel,
    private val speed: Int?,
    private val connection: Connection
) : CoroutineScope {
    private val job = Job()

    override val coroutineContext: CoroutineContext
        get() = Dispatchers.Main + job
    val isAtStartZone get() = centerCell.row == 1 && centerCell.col == 1

    private var sensors = listOf<Sensor>()

    private val arrowsFound = hashMapOf<Pair<Int, Int>, Direction>()
    val correctArrows = mutableListOf<CellInfoModel>()

    private var movementCount = 0

    init {
        startProcessingArrowFound()
    }

    private fun startProcessingArrowFound() {
        if (connection.isConnected) {
            launch {
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
                    if (explorationMaze[row, col] == CELL_OBSTACLE) {
                        println("Received arrow at ($col,$row,$directionCommand)")
                        if (connection.isConnected) {
                            connection.sendArrowCommand(col, row, direction)
                        }
                        correctArrows += CellInfoModel(row, col, direction)
                        saveJson(Arrows(correctArrows.map { CenterCell(it.row, it.col, it.direction) }))
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

    suspend fun sense(forceUpdate: Boolean = false): Movement? {
        if (connection.isConnected) {
            connection.sendGetSensorDataCommand()
        }
//        delay(250L)
        val sensorData = sensors.map { it to it.sense() }
//        if (!forceUpdate) {
//            val tempMaze = explorationMaze.copy()
//            val movement = checkConflictWithSensorData(tempMaze, sensorData)
//            if (movement != null) {
//                return movement
//            }
//        }

        val (centerRow, centerCol, direction) = centerCell
        for ((sensor, sensedDistance) in sensorData) {
            val (rowDiff, colDiff, rowInc, colInc) = SENSOR_INFO[sensor.position][direction.ordinal]
            if (sensedDistance >= sensor.senseRange.first) {
                if (sensedDistance == 0) {  // Found an obstacle next to the robot
                    val row = centerRow + rowDiff
                    val col = centerCol + colDiff
                    if (!MazeModel.isOutsideOfMaze(row, col)) {
                        if (explorationMaze[row, col] != CELL_UNKNOWN && explorationMaze[row, col] != CELL_OBSTACLE) {
                            println("Warning: maze[$row][$col] was ${explorationMaze[row, col]} but setting to CELL_OBSTACLE")
                        }
                        explorationMaze[row, col] = CELL_OBSTACLE
                        // Find an obstacle, check if any arrows found on it
                        val arrowDirection = arrowsFound.remove(row to col)
                        if (arrowDirection != null) {
                            if (connection.isConnected) {
                                connection.sendArrowCommand(col, row, arrowDirection)
                            }
                        }
                    }
                } else {
                    // e.g., 2 means 2 grids away from the robot is empty
                    for (i in 0 until min(sensor.senseRange.last, sensedDistance)) {
                        val row = centerRow + rowDiff + rowInc * i
                        val col = centerCol + colDiff + colInc * i
                        if (!MazeModel.isOutsideOfMaze(row, col)) {
                            if (explorationMaze[row, col] < CELL_UNKNOWN) {
                                println("Warning: maze[$row][$col] was ${explorationMaze[row, col]} but setting to CELL_SENSED")
                            }
                            // If original value is already greater than 0, leave it as it is
                            if (explorationMaze[row, col] < CELL_SENSED) {
                                explorationMaze[row, col] = CELL_SENSED
                            }

                            // Clearing an arrow, save back to arrowsFound
                            val arrow = correctArrows.find { it.row == row && it.col == col }
                            if (arrow != null) {
                                correctArrows.remove(arrow)
                                saveJson(Arrows(correctArrows.map { CenterCell(it.row, it.col, it.direction) }))
                                arrowsFound[row to col] = arrow.direction
                            }
                        }
                    }
                }
            }
        }
        if (connection.isConnected) {
            val part1 = explorationMaze.outputMapDescriptorPart1()
            val part2 = explorationMaze.outputMapDescriptorPart2()
            connection.sendMdfString(part1, part2)
        }
        return null
    }

    /**
     * @return If has conflict with existing map, return the movement needed to sense again, else return null
     */
    private fun checkConflictWithSensorData(mazeModel: MazeModel, sensorData: List<Pair<Sensor, Int>>): Movement? {
        val (centerRow, centerCol, direction) = centerCell
        for ((sensor, sensedDistance) in sensorData) {
            val (rowDiff, colDiff, rowInc, colInc) = SENSOR_INFO[sensor.position][direction.ordinal]
            if (sensedDistance >= sensor.senseRange.first) {
                if (sensedDistance == 0) {  // Found an obstacle next to the robot
                    val row = centerRow + rowDiff
                    val col = centerCol + colDiff
                    if (!MazeModel.isOutsideOfMaze(row, col)) {
                        if (mazeModel[row, col] != CELL_UNKNOWN && mazeModel[row, col] != CELL_OBSTACLE) {
                            println("Warning: maze[$row][$col] was ${mazeModel[row, col]} but setting to CELL_OBSTACLE")
                            when (sensor.position) {
                                in (0..2) -> return Movement.TURN_LEFT
                                in (6..8) -> return Movement.TURN_RIGHT
                            }
                        }
                    }
                } else {
                    // e.g., 2 means 2 grids away from the robot is empty
                    for (i in 0 until min(sensor.senseRange.last, sensedDistance)) {
                        val row = centerRow + rowDiff + rowInc * i
                        val col = centerCol + colDiff + colInc * i
                        if (!MazeModel.isOutsideOfMaze(row, col)) {
                            if (mazeModel[row, col] < CELL_UNKNOWN) {
                                println("Warning: maze[$row][$col] was ${mazeModel[row, col]} but setting to CELL_SENSED")
                                when (sensor.position) {
                                    in (0..2) -> return Movement.TURN_LEFT
                                    in (6..8) -> return Movement.TURN_RIGHT
                                }
                            }
                        }
                    }
                }
            }
        }
        return null
    }

    suspend fun move(movement: Movement) {
        if (!connection.isConnected) {
            delay(250L / (speed ?: 3))
        }
        if (connection.isConnected) {
            tryCalibrate(false)
        }
        movementCount++
        if (connection.isConnected) {
            when (movement) {
                Movement.TURN_LEFT, Movement.TURN_RIGHT -> {
                    connection.sendTurnCommandWithCountAndWait(movement, 1)
                    delay(250L)
                }
                Movement.MOVE_FORWARD -> connection.sendMoveForwardWithDistanceAndWait(1)
            }
        }
        when (movement) {   // Update on UI
            Movement.TURN_RIGHT -> turnRight()
            Movement.TURN_LEFT -> turnLeft()
            Movement.MOVE_FORWARD -> moveForward()
        }
    }

    private suspend fun turnLeft() {
        println("Command TURN_LEFT")
        centerCell.direction = centerCell.direction.turnLeft()
        if (connection.isConnected) {
            connection.sendRobotCenter(centerCell)
        }
        saveStateToFile()
    }

    private suspend fun turnRight() {
        println("Command TURN_RIGHT")
        centerCell.direction = centerCell.direction.turnRight()
        if (connection.isConnected) {
            connection.sendRobotCenter(centerCell)
        }
        saveStateToFile()
    }

    private suspend fun moveForward() {
        println("Command MOVE_FORWARD")
        val (centerRow, centerCol, currentDirection) = centerCell
        val (rowDiff, colDiff) = NEXT_CELL[MovementInfo(Movement.MOVE_FORWARD, currentDirection)]
            ?: throw IllegalStateException()
        if (rowDiff != 0) {
            val frontRow = centerRow + rowDiff + rowDiff
            for (col in centerCol - 1..centerCol + 1) {
                if (!MazeModel.isOutsideOfMaze(frontRow, col)) {
                    if (explorationMaze[frontRow, col] < 0) {
                        println("Warning: Covering maze[$frontRow][$col] but it was ${explorationMaze[frontRow, col]}")
                    }
                    explorationMaze[frontRow, col] = explorationMaze[frontRow, col] + 1
                }
            }
            centerCell.row += rowDiff
        } else {
            check(colDiff != 0)
            val frontCol = centerCol + colDiff + colDiff
            for (row in centerRow - 1..centerRow + 1) {
                if (!MazeModel.isOutsideOfMaze(row, frontCol)) {
                    if (explorationMaze[row, frontCol] < 0) {
                        println("Warning: Covering maze[$row][$frontCol] but it was ${explorationMaze[row, frontCol]}")
                    }
                    explorationMaze[row, frontCol] = explorationMaze[row, frontCol] + 1
                }
            }
            centerCell.col += colDiff
        }
        if (connection.isConnected) {
            connection.sendRobotCenter(centerCell)
        }
        explorationMaze.print()
        saveStateToFile()
    }

    private fun saveStateToFile() {
        saveJson(ExplorationMaze(List(MAZE_ROWS) { row ->
            List(MAZE_COLUMNS) { col ->
                explorationMaze[row, col]
            }
        }))
        saveJson(CenterCell(centerCell.row, centerCell.col, centerCell.direction))
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

    suspend fun moveFollowingMovements(movements: List<Movement>, isAtFastestPath: Boolean = false) {
        if (movements.isEmpty()) {
            return
        }
        if (isAtFastestPath) {
            val compactList = movements.compact(maxConsecutiveForward = 3)
            println("CompactList: $compactList")
            for ((movement, count) in compactList) {
                if (movement == Movement.MOVE_FORWARD) {
                    if (connection.isConnected) {
                        connection.sendMoveForwardWithDistanceAndWait(count)
                    }
                    for (i in 0 until count) {
                        if (!connection.isConnected) {
                            delay(250L / (speed ?: 3))
                        }
                        moveForward()
                    }
                } else {
                    if (connection.isConnected) {
                        connection.sendTurnCommandWithCountAndWait(movement, count)
                    }
                    for (i in 0 until count) {
                        if (!connection.isConnected) {
                            delay(250L / (speed ?: 3))
                        }
                        if (movement == Movement.TURN_LEFT) {
                            turnLeft()
                        } else {
                            turnRight()
                        }
                    }
                }
//                if (connection.isConnected) {
//                    delay(250L)
//                }
            }
        } else {
            for (movement in movements) {
                move(movement)
                if (connection.isConnected) {
                    delay(600L)
                }
            }
        }
    }

    private suspend fun tryCalibrate(force: Boolean) {
        if (movementCount >= 4 || force) {
            val rightSide = explorationMaze.getSide(centerCell.copy(), Movement.TURN_RIGHT)
            val frontSide = explorationMaze.getSide(centerCell.copy(), Movement.MOVE_FORWARD)
            val leftSide = explorationMaze.getSide(centerCell.copy(), Movement.TURN_LEFT)
            println("tryCalibrate, force=$force, movementCount=$movementCount")
            println("Center=$centerCell")
            println("Right=${rightSide.joinToString()}")
            println("Front=${frontSide.joinToString()}")
            println("Left=${leftSide.joinToString()}")
            when {
                frontSide.all { it == CELL_OBSTACLE } && leftSide.all { it == CELL_OBSTACLE } -> {
                    movementCount = 0
                    connection.sendCalibrationCommandAndWait()
                    move(Movement.TURN_LEFT)
                    delay(250L)
                    connection.sendCalibrationCommandAndWait()
                    move(Movement.TURN_RIGHT)
                    delay(250L)
                    movementCount = 0
                }
                frontSide.all { it == CELL_OBSTACLE } && rightSide.all { it == CELL_OBSTACLE } -> {
                    movementCount = 0
                    connection.sendCalibrationCommandAndWait()
                    move(Movement.TURN_RIGHT)
                    delay(250L)
                    connection.sendCalibrationCommandAndWait()
                    move(Movement.TURN_LEFT)
                    delay(250L)
                    movementCount = 0
                }
                frontSide.all { it == CELL_OBSTACLE } -> {
                    movementCount = 0
                    connection.sendCalibrationCommandAndWait()
                    movementCount = 0
                }
                leftSide.all { it == CELL_OBSTACLE } -> {
                    movementCount = 0
                    move(Movement.TURN_LEFT)
                    delay(250L)
                    connection.sendCalibrationCommandAndWait()
                    move(Movement.TURN_RIGHT)
                    delay(250L)
                    movementCount = 0
                }
                rightSide.all { it == CELL_OBSTACLE } -> {
                    movementCount = 0
                    move(Movement.TURN_RIGHT)
                    delay(250L)
                    connection.sendCalibrationCommandAndWait()
                    move(Movement.TURN_LEFT)
                    delay(250L)
                    movementCount = 0
                }
            }
        }
    }

    suspend fun calibrateForFastestPath() {
        if (!isAtStartZone || centerCell.direction != Direction.UP) {
            return
        }
        if (connection.isConnected) {
            move(Movement.TURN_LEFT)
            tryCalibrate(true)
            move(Movement.TURN_RIGHT)
        }
    }

    suspend fun calibrateAtCorner(row: Int, col: Int, direction: Direction) {
        if (connection.isConnected) {
            if (row == MAZE_ROWS - 1 - 1) {
                if (col == 1) {
                    when (direction) {
                        Direction.UP -> tryCalibrate(true)
                        Direction.DOWN -> {
                            move(Movement.TURN_RIGHT)
                            tryCalibrate(true)
                            move(Movement.TURN_LEFT)
                        }
                        Direction.LEFT -> tryCalibrate(true)
                        Direction.RIGHT -> {
                            move(Movement.TURN_LEFT)
                            tryCalibrate(true)
                            move(Movement.TURN_RIGHT)
                        }
                    }
                } else if (col == MAZE_COLUMNS - 1 - 1) {
                    when (direction) {
                        Direction.UP -> tryCalibrate(true)
                        Direction.DOWN -> {
                            move(Movement.TURN_LEFT)
                            tryCalibrate(true)
                            move(Movement.TURN_RIGHT)
                        }
                        Direction.LEFT -> {
                            move(Movement.TURN_RIGHT)
                            tryCalibrate(true)
                            move(Movement.TURN_LEFT)
                        }
                        Direction.RIGHT -> tryCalibrate(true)
                    }
                }
            } else if (row == 1) {
                if (col == MAZE_COLUMNS - 1 - 1) {
                    when (direction) {
                        Direction.UP -> {
                            move(Movement.TURN_RIGHT)
                            tryCalibrate(true)
                            move(Movement.TURN_LEFT)
                        }
                        Direction.DOWN -> tryCalibrate(true)
                        Direction.LEFT -> {
                            move(Movement.TURN_RIGHT)
                            tryCalibrate(true)
                            move(Movement.TURN_LEFT)
                        }
                        Direction.RIGHT -> tryCalibrate(true)
                    }
                }
            }
        }
    }
}

private fun List<Movement>.compact(
    maxConsecutiveForward: Int = 3,
    maxConsecutiveTurns: Int = 1
): List<Pair<Movement, Int>> {
    val compactList = mutableListOf<Pair<Movement, Int>>()
    compactList += first() to 1
    for (i in 1 until size) {
        val movement = this[i]
        val (lastMovement, count) = compactList.last()
        val countThreshold = when (movement) {
            Movement.TURN_LEFT, Movement.TURN_RIGHT -> maxConsecutiveTurns
            Movement.MOVE_FORWARD -> maxConsecutiveForward
        }
        if (movement == lastMovement && count < countThreshold) {
            compactList[compactList.lastIndex] = lastMovement to count + 1
        } else {
            compactList += movement to 1
        }
    }
    return compactList
}
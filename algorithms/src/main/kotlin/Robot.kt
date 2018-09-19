class Robot {
    var centerCell = CellInfo(1, 1, Direction.UP)
        private set

    val isAtStartZone get() = centerCell.row == 1 && centerCell.col == 1

    val explorationMaze = Maze().apply { initExploration() }

    var speed = 0

    private var sensors = listOf<Sensor>()

    var movementCount = 0
        private set

    fun setSensors(sensors: List<Sensor>) {
        this.sensors = sensors
    }

    fun sense() {
        val (centerRow, centerCol, direction) = centerCell
        for (sensor in sensors) {
            val sensedDistance = sensor.sense()
            val (rowDiff, colDiff, rowInc, colInc) = SENSOR_INFO[sensor.position][direction.ordinal]
            if (sensedDistance == -1) {
                for (i in sensor.senseRange) {
                    val row = centerRow + rowDiff + rowInc * i
                    val col = centerCol + colDiff + colInc * i
                    if (!Maze.isOutsideOfMaze(row, col) && explorationMaze[row][col] == CELL_UNKNOWN) {
                        explorationMaze[row][col] = CELL_SENSED
                    }
                }
            } else {
                for (i in sensor.senseRange.first..sensedDistance) {
                    val row = centerRow + rowDiff + rowInc * i
                    val col = centerCol + colDiff + colInc * i
                    if (!Maze.isOutsideOfMaze(row, col) && explorationMaze[row][col] == CELL_UNKNOWN) {
                        if (i == sensedDistance) {
                            explorationMaze[row][col] = CELL_OBSTACLE
                        } else {
                            explorationMaze[row][col] = CELL_SENSED
                        }
                    }
                }
            }
        }
    }

    fun turnLeft() {
        centerCell = CellInfo(centerCell.row, centerCell.col, centerCell.direction.turnLeft())
//        println("Turn left")
//        explorationMaze.prettyPrintWithRobot(this)
        movementCount++
        if (speed != 0) {
            Thread.sleep(1000L / speed)
        }
    }

    fun turnRight() {
        centerCell = CellInfo(centerCell.row, centerCell.col, centerCell.direction.turnRight())
//        println("Turn right")
//        explorationMaze.prettyPrintWithRobot(this)
        movementCount++
        if (speed != 0) {
            Thread.sleep(1000L / speed)
        }
    }

    fun moveForward() {
        val (centerRow, centerCol, currentDirection) = centerCell
        val (rowDiff, colDiff) = NEXT_CELL[MovementInfo(Movement.MOVE_FORWARD, currentDirection)]
            ?: throw IllegalStateException()
        centerCell = if (rowDiff != 0) {
            val frontRow = centerRow + rowDiff + rowDiff
            for (col in centerCol - 1..centerCol + 1) {
                explorationMaze[frontRow][col]++
            }
            CellInfo(centerRow + rowDiff, centerCol, currentDirection)
        } else {
            check(colDiff != 0)
            val frontCol = centerCol + colDiff + colDiff
            for (row in centerRow - 1..centerRow + 1) {
                explorationMaze[row][frontCol]++
            }
            CellInfo(centerRow, centerCol + colDiff, currentDirection)
        }
//        println("Move forward")
//        explorationMaze.prettyPrintWithRobot(this)
        movementCount++
        if (speed != 0) {
            Thread.sleep(1000L / speed)
        }
    }

    fun goToStartZone() {
        val fastestPathMaze = explorationMaze.copy()
        val path = findFastestPathToDestination(fastestPathMaze, centerCell, 1 to 1)
            .asSequence()
            .filter { it.isNotEmpty() }
            .map {
                val (row, col, direction) = it.last()
                when (direction) {
                    Direction.UP -> it
                    Direction.DOWN -> it +
                            CellInfo(row, col, direction.turnLeft()) +
                            CellInfo(row, col, direction.turnLeft().turnLeft())
                    Direction.LEFT -> it + CellInfo(row, col, direction.turnRight())
                    Direction.RIGHT -> it + CellInfo(row, col, direction.turnLeft())
                }
            }
            .minBy { it.size } ?: throw IllegalStateException("Unable to find way to go home")
        val movements = path.toMovements()
        moveFollowingMovements(movements)
    }

    inline fun coversRow(row: Int): Boolean {
        val rowDiff = row - centerCell.row
        return rowDiff >= -1 && rowDiff <= 1
    }

    inline fun coversColumn(col: Int): Boolean {
        val colDiff = col - centerCell.col
        return colDiff >= -1 && colDiff <= 1
    }

    fun turnToFaceUp() {
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

    fun moveFollowingMovements(movements: List<Movement>) {
        for (movement in movements) {
            when (movement) {
                Movement.TURN_RIGHT -> turnRight()
                Movement.MOVE_FORWARD -> moveForward()
                Movement.TURN_LEFT -> turnLeft()
            }
        }
    }
}
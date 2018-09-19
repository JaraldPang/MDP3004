import java.io.FileInputStream
import kotlin.math.max
import kotlin.math.min

class Maze() {
    companion object {
        fun isOutsideOfMaze(row: Int, col: Int) = row < 0 || row >= MAZE_ROWS || col < 0 || col >= MAZE_COLUMNS
    }

    private val map = Array(MAZE_ROWS) { IntArray(MAZE_COLUMNS) }

    private constructor(maze: Maze) : this() {
        for (row in 0 until MAZE_ROWS) {
            for (col in 0 until MAZE_COLUMNS) {
                map[row][col] = maze.map[row][col]
            }
        }
    }


    val isFullyExplored: Boolean
        get() {
            for (row in 0 until MAZE_ROWS) {
                for (col in 0 until MAZE_COLUMNS) {
                    if (map[row][col] == CELL_UNKNOWN) {
                        return false
                    }
                }
            }
            println("Maze is fully explored!")
            return true
        }

    fun copy() = Maze(this)

    fun prettyPrint() {
        for (row in MAZE_ROWS - 1 downTo 0) {
            for (col in 0 until MAZE_COLUMNS) {
                val toPrint = when (map[row][col]) {
                    CELL_UNKNOWN -> '?'
                    CELL_OBSTACLE -> 'X'
                    CELL_SENSED -> '.'
                    else -> '0' + map[row][col]
                }
                print(toPrint)
            }
            println()
        }
        println()
    }

    fun prettyPrintDistance() {
        for (row in MAZE_ROWS - 1 downTo 0) {
            for (col in 0 until MAZE_COLUMNS) {
                if (col != 0) {
                    print(' ')
                }
                when (map[row][col]) {
                    CELL_UNKNOWN -> print('?')
                    CELL_OBSTACLE -> print('X')
                    else -> print(map[row][col])
                }
            }
        }
    }

    fun prettyPrintWithRobot(robot: Robot) {
        for (row in MAZE_ROWS - 1 downTo 0) {
            for (col in 0 until MAZE_COLUMNS) {
                val toPrint = when (map[row][col]) {
                    CELL_UNKNOWN -> '?'
                    CELL_OBSTACLE -> 'X'
                    CELL_SENSED -> '.'
                    else -> {
                        if (robot.coversRow(row) && robot.coversColumn(col)) {
                            val (centerRow, centerCol, direction) = robot.centerCell
                            when {
                                direction == Direction.UP && row == centerRow + 1 && col == centerCol -> '^'
                                direction == Direction.DOWN && row == centerRow - 1 && col == centerCol -> 'v'
                                direction == Direction.LEFT && row == centerRow && col == centerCol - 1 -> '<'
                                direction == Direction.RIGHT && row == centerRow && col == centerCol + 1 -> '>'
                                else -> 'a' - 1 + map[row][col]
                            }
                        } else {
                            '0' + map[row][col]
                        }
                    }
                }
                print(toPrint)
            }
            println()
        }
        println()
    }

    fun initExploration() {
        for (row in 0 until MAZE_ROWS) {
            for (col in 0 until MAZE_COLUMNS) {
                if (row < ROBOT_SIZE && col < ROBOT_SIZE) {
                    map[row][col] = 1
                } else {
                    map[row][col] = CELL_UNKNOWN
                }
            }
        }
    }

    fun resetForFastestPath() {
        for (row in 0 until MAZE_ROWS) {
            for (col in 0 until MAZE_COLUMNS) {
                if (map[row][col] < 0) {
                    map[row][col] = CELL_OBSTACLE
                } else {
                    map[row][col] = CELL_SENSED
                }
            }
        }
    }

    fun loadFromDisk(filename: String) {
        FileInputStream(filename)
            .bufferedReader()
            .lineSequence()
            .forEachIndexed { row, line ->
                for (col in 0 until line.length) {
                    if (line[col] == '0') {
                        map[MAZE_ROWS - row - 1][col] = CELL_SENSED
                    } else {
                        map[MAZE_ROWS - row - 1][col] = CELL_OBSTACLE
                    }
                }
            }
    }

    operator fun get(row: Int) = map[row]

    fun getEnvironmentOnSides(cellInfo: CellInfo): IntArray {
        val (centerRow, centerCol, direction) = cellInfo
        val minSides = IntArray(3)
        val sides = SIDES[direction.ordinal]
        for (movement in Movement.values()) {
            val (rowDiff, colDiff) = sides[movement.ordinal]
            if (rowDiff != 0) {
                val rowOfSide = centerRow + rowDiff
                if (rowOfSide < 0 || rowOfSide >= MAZE_ROWS) {
                    minSides[movement.ordinal] = CELL_OBSTACLE
                } else {
                    val state1 = map[rowOfSide][centerCol - 1]
                    val state2 = map[rowOfSide][centerCol]
                    val state3 = map[rowOfSide][centerCol + 1]
                    minSides[movement.ordinal] = min3(state1, state2, state3)
                }
            } else {
                check(colDiff != 0)
                val columnOfSide = centerCol + colDiff
                if (columnOfSide < 0 || columnOfSide >= MAZE_COLUMNS) {
                    minSides[movement.ordinal] = CELL_OBSTACLE
                } else {
                    val state1 = map[centerRow - 1][columnOfSide]
                    val state2 = map[centerRow][columnOfSide]
                    val state3 = map[centerRow + 1][columnOfSide]
                    minSides[movement.ordinal] = min3(state1, state2, state3)
                }
            }
        }
        return minSides
    }

//    fun getEnvironmentOnSides(cellInfo: CellInfo): Pair<IntArray, IntArray> {
//        val (centerRow, centerCol, direction) = cellInfo
//        val minSides = IntArray(3)
//        val maxSides = IntArray(3)
//        val sides = SIDES[direction.ordinal]
//        for (movement in Movement.values()) {
//            val (rowDiff, colDiff) = sides[movement.ordinal]
//            if (rowDiff != 0) {
//                val rowOfSide = centerRow + rowDiff
//                if (rowOfSide < 0 || rowOfSide >= MAZE_ROWS) {
//                    minSides[movement.ordinal] = CELL_OBSTACLE
//                    maxSides[movement.ordinal] = CELL_OBSTACLE
//                } else {
//                    val state1 = map[rowOfSide][centerCol - 1]
//                    val state2 = map[rowOfSide][centerCol]
//                    val state3 = map[rowOfSide][centerCol + 1]
//                    minSides[movement.ordinal] = min3(state1, state2, state3)
//                    maxSides[movement.ordinal] = max3(state1, state2, state3)
//                }
//            } else {
//                check(colDiff != 0)
//                val columnOfSide = centerCol + colDiff
//                if (columnOfSide < 0 || columnOfSide >= MAZE_COLUMNS) {
//                    minSides[movement.ordinal] = CELL_OBSTACLE
//                    maxSides[movement.ordinal] = CELL_OBSTACLE
//                } else {
//                    val state1 = map[centerRow - 1][columnOfSide]
//                    val state2 = map[centerRow][columnOfSide]
//                    val state3 = map[centerRow + 1][columnOfSide]
//                    minSides[movement.ordinal] = min3(state1, state2, state3)
//                    maxSides[movement.ordinal] = max3(state1, state2, state3)
//                }
//            }
//        }
//        return minSides to maxSides
//    }

    fun calculateCoverage(): Double {
        var sum = 0
        for (row in 0 until MAZE_ROWS) {
            for (col in 0 until MAZE_COLUMNS) {
                if (map[row][col] != CELL_UNKNOWN) {
                    sum++
                }
            }
        }
        return sum * 100.0 / (MAZE_ROWS * MAZE_COLUMNS)
    }

    fun outputExploredUnexploredString(): String {
        val stringBuilder = StringBuilder()
        stringBuilder.append("11")
        for (row in 0 until MAZE_ROWS) {
            for (col in 0 until MAZE_COLUMNS) {
                if (map[row][col] == CELL_UNKNOWN) {
                    stringBuilder.append('0')
                } else {
                    stringBuilder.append('1')
                }
            }
        }
        stringBuilder.append("11")
        val bigInteger = stringBuilder.toString().toBigInteger(2)
        return bigInteger.toString(16)
    }

    fun outputEmptyObstacleString(): String {
        val stringBuilder = StringBuilder()
        for (row in 0 until MAZE_ROWS) {
            for (col in 0 until MAZE_COLUMNS) {
                if (map[row][col] != CELL_UNKNOWN) {
                    if (map[row][col] == CELL_OBSTACLE) {
                        stringBuilder.append('1')
                    } else {
                        stringBuilder.append('0')
                    }
                }
            }
        }
        val bigInteger = stringBuilder.toString().toBigInteger(2)
        return bigInteger.toString(16)
    }
}

data class CellInfo(val row: Int, val col: Int, val direction: Direction) {
    operator fun plus(movement: Movement): CellInfo {
        if (movement == Movement.MOVE_FORWARD) {
            val (rowDiff, colDiff) = when (direction) {
                Direction.UP -> 1 to 0
                Direction.DOWN -> -1 to 0
                Direction.LEFT -> 0 to -1
                Direction.RIGHT -> 0 to 1
            }
            return CellInfo(row + rowDiff, col + colDiff, direction)
        } else {
            val newDirection = when (movement) {
                Movement.TURN_RIGHT -> direction.turnRight()
                Movement.MOVE_FORWARD -> direction
                Movement.TURN_LEFT -> direction.turnLeft()
            }
            return CellInfo(row, col, newDirection)
        }
    }

    operator fun minus(other: CellInfo): Movement {
        val rowDiff = row - other.row
        val colDiff = col - other.col
        if (other.direction == direction) {
            if ((rowDiff == 1 && colDiff == 0 && direction == Direction.UP)
                || (rowDiff == -1 && colDiff == 0 && direction == Direction.DOWN)
                || (rowDiff == 0 && colDiff == -1 && direction == Direction.LEFT)
                || (rowDiff == 0 && colDiff == 1 && direction == Direction.RIGHT)
            ) {
                return Movement.MOVE_FORWARD
            } else {
                throw IllegalArgumentException("$this - $other is not defined")
            }
        } else {
            check(rowDiff == 0 && colDiff == 0)
            return when (direction) {
                other.direction.turnRight() -> Movement.TURN_RIGHT
                other.direction.turnLeft() -> Movement.TURN_LEFT
                else -> throw IllegalArgumentException("$this - $other is not defined")
            }
        }
    }
}

data class MovementInfo(val movement: Movement, val direction: Direction)

inline fun min3(a: Int, b: Int, c: Int) = min(min(a, b), c)

inline fun max3(a: Int, b: Int, c: Int) = max(max(a, b), c)
package model

import domain.*
import javafx.beans.property.SimpleIntegerProperty
import javafx.beans.property.SimpleObjectProperty
import tornadofx.*
import kotlin.math.min

class MazeModel() : ViewModel() {
    companion object {
        fun isOutsideOfMaze(row: Int, col: Int) = row < 0 || row >= MAZE_ROWS || col < 0 || col >= MAZE_COLUMNS
    }

    val mazeProperties = Array(MAZE_ROWS) { _ -> Array(MAZE_COLUMNS) { SimpleIntegerProperty(0) } }

    class PropertyArrayWrapper(private val row: Array<SimpleIntegerProperty>) {
        operator fun get(col: Int): Int = row[col].value
        operator fun set(col: Int, value: Int) {
            row[col].value = value
        }
    }

    operator fun get(row: Int) = PropertyArrayWrapper(mazeProperties[row])

    private constructor(maze: MazeModel) : this() {
        for (row in 0 until MAZE_ROWS) {
            for (col in 0 until MAZE_COLUMNS) {
                this[row][col] = maze[row][col]
            }
        }
    }


    val isFullyExplored: Boolean
        get() {
            for (row in 0 until MAZE_ROWS) {
                for (col in 0 until MAZE_COLUMNS) {
                    if (this[row][col] == CELL_UNKNOWN) {
                        return false
                    }
                }
            }
            println("MazeModel is fully explored!")
            return true
        }

    fun copy() = MazeModel(this)

    fun initExploration() {
        for (row in 0 until MAZE_ROWS) {
            for (col in 0 until MAZE_COLUMNS) {
                if (row < ROBOT_SIZE && col < ROBOT_SIZE) {
                    this[row][col] = 1
                } else {
                    this[row][col] = CELL_UNKNOWN
                }
            }
        }
    }

    fun resetForFastestPath() {
        for (row in 0 until MAZE_ROWS) {
            for (col in 0 until MAZE_COLUMNS) {
                if (this[row][col] < 0) {
                    this[row][col] = CELL_OBSTACLE
                } else {
                    this[row][col] = CELL_SENSED
                }
            }
        }
    }

    fun getEnvironmentOnSides(cellInfoModel: CellInfoModel): IntArray {
        val (centerRow, centerCol, direction) = cellInfoModel
        val minSides = IntArray(3)
        val sides = SIDES[direction.ordinal]
        for (movement in Movement.values()) {
            val (rowDiff, colDiff) = sides[movement.ordinal]
            if (rowDiff != 0) {
                val rowOfSide = centerRow + rowDiff
                if (rowOfSide < 0 || rowOfSide >= MAZE_ROWS) {
                    minSides[movement.ordinal] = CELL_OBSTACLE
                } else {
                    val state1 = this[rowOfSide][centerCol - 1]
                    val state2 = this[rowOfSide][centerCol]
                    val state3 = this[rowOfSide][centerCol + 1]
                    minSides[movement.ordinal] = min3(state1, state2, state3)
                }
            } else {
                check(colDiff != 0)
                val columnOfSide = centerCol + colDiff
                if (columnOfSide < 0 || columnOfSide >= MAZE_COLUMNS) {
                    minSides[movement.ordinal] = CELL_OBSTACLE
                } else {
                    val state1 = this[centerRow - 1][columnOfSide]
                    val state2 = this[centerRow][columnOfSide]
                    val state3 = this[centerRow + 1][columnOfSide]
                    minSides[movement.ordinal] = min3(state1, state2, state3)
                }
            }
        }
        return minSides
    }

    fun calculateCoverage(): Double {
        var sum = 0
        for (row in 0 until MAZE_ROWS) {
            for (col in 0 until MAZE_COLUMNS) {
                if (this[row][col] != CELL_UNKNOWN) {
                    sum++
                }
            }
        }
        return sum * 100.0 / (MAZE_ROWS * MAZE_COLUMNS)
    }

    fun outputExploredUnexploredString(): String {
        val result = StringBuilder()
        val binary = StringBuilder()

        binary.append("11")
        for (row in 0 until MAZE_ROWS) {
            for (col in 0 until MAZE_COLUMNS) {
                if (this[row][col] == CELL_UNKNOWN) {
                    binary.append('0')
                } else {
                    binary.append('1')
                }
                if (binary.length == 4) {
                    result.append(binary.toString().binToHex())
                    binary.setLength(0)
                }
            }
        }
        binary.append("11")
        result.append(binary.toString().binToHex())
        return result.toString()
    }

    fun outputEmptyObstacleString(): String {
        val result = StringBuilder()
        val binary = StringBuilder()
        for (row in 0 until MAZE_ROWS) {
            for (col in 0 until MAZE_COLUMNS) {
                if (this[row][col] != CELL_UNKNOWN) {
                    if (this[row][col] == CELL_OBSTACLE) {
                        binary.append('1')
                    } else {
                        binary.append('0')
                    }
                    if (binary.length == 4) {
                        result.append(binary.toString().binToHex())
                        binary.setLength(0)
                    }
                }
            }
        }
        if (binary.isNotEmpty()) {
            result.append(binary.toString().binToHex())
        }
        return result.toString()
    }

    fun parseFromMapDescriptors(part1: String, part2: String) {
        val newMaze = Array(MAZE_ROWS) { IntArray(MAZE_COLUMNS) }
        val trimmedPart1 = part1.trim()
        val trimmedPart2 = part2.trim()
        val binaryPart1 = StringBuilder()
        val firstHexDigitInBinary = trimmedPart1[0].toString().toInt(16).toString(2)
        binaryPart1.append(firstHexDigitInBinary.replaceFirst("11", ""))
        for (i in 1 until trimmedPart1.length) {
            val hexDigitInBinary = trimmedPart1[i].toString().toInt(16).toString(2).padStart(4, '0')
            binaryPart1.append(hexDigitInBinary)
        }
        for (row in 0 until MAZE_ROWS) {
            for (col in 0 until MAZE_COLUMNS) {
                val index = row * MAZE_COLUMNS + col
                if (binaryPart1[index] == '0') {
                    newMaze[row][col] = CELL_UNKNOWN
                }
            }
        }

        val binaryPart2 = StringBuilder()
        for (hexDigit in trimmedPart2) {
            val hexDigitInBinary = hexDigit.toString().toInt(16).toString(2).padStart(4, '0')
            binaryPart2.append(hexDigitInBinary)
        }
        var index = 0
        for (row in 0 until MAZE_ROWS) {
            for (col in 0 until MAZE_COLUMNS) {
                if (newMaze[row][col] != CELL_UNKNOWN) {
                    if (binaryPart2[index] == '0') {
                        newMaze[row][col] = CELL_SENSED
                    } else {
                        newMaze[row][col] = CELL_OBSTACLE
                    }
                    index++
                }
            }
        }

        for (row in 0 until MAZE_ROWS) {
            for (col in 0 until MAZE_COLUMNS) {
                this[row][col] = newMaze[row][col]
            }
        }
    }
}

private fun String.binToHex() = toInt(2).toString(16)

class CellInfoModel(row: Int = 1, col: Int = 1, direction: Direction = Direction.UP) : ViewModel() {
    val rowProperty = SimpleIntegerProperty(row)
    var row by rowProperty

    val colProperty = SimpleIntegerProperty(col)
    var col by colProperty

    val directionProperty = SimpleObjectProperty(direction)
    var direction: Direction by directionProperty

    operator fun component1() = row
    operator fun component2() = col
    operator fun component3() = direction

    override fun equals(other: Any?): Boolean {
        if (this === other) return true
        if (javaClass != other?.javaClass) return false

        other as CellInfoModel

        if (row != other.row) return false
        if (col != other.col) return false
        if (direction != other.direction) return false

        return true
    }

    override fun hashCode(): Int {
        var result = row.hashCode()
        result = 31 * result + col.hashCode()
        result = 31 * result + direction.hashCode()
        return result
    }

    override fun toString(): String {
        return "CellInfoModel(row=$row, col=$col, direction=$direction)"
    }

    operator fun plus(movement: Movement): CellInfoModel {
        if (movement == Movement.MOVE_FORWARD) {
            val (rowDiff, colDiff) = when (direction) {
                Direction.UP -> 1 to 0
                Direction.DOWN -> -1 to 0
                Direction.LEFT -> 0 to -1
                Direction.RIGHT -> 0 to 1
            }
            return CellInfoModel(row + rowDiff, col + colDiff, direction)
        } else {
            val newDirection = when (movement) {
                Movement.TURN_RIGHT -> direction.turnRight()
                Movement.MOVE_FORWARD -> direction
                Movement.TURN_LEFT -> direction.turnLeft()
            }
            return CellInfoModel(row, col, newDirection)
        }
    }

    operator fun plusAssign(movement: Movement) {
        if (movement == Movement.MOVE_FORWARD) {
            val (rowDiff, colDiff) = when (direction) {
                Direction.UP -> 1 to 0
                Direction.DOWN -> -1 to 0
                Direction.LEFT -> 0 to -1
                Direction.RIGHT -> 0 to 1
            }
            row += rowDiff
            col += colDiff
        } else {
            val newDirection = when (movement) {
                Movement.TURN_RIGHT -> direction.turnRight()
                Movement.MOVE_FORWARD -> direction
                Movement.TURN_LEFT -> direction.turnLeft()
            }
            direction = newDirection
        }
    }

    operator fun minus(other: CellInfoModel): Movement {
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

    fun copy(row: Int = this.row, col: Int = this.col, direction: Direction = this.direction) =
        CellInfoModel(row, col, direction)
}

data class MovementInfo(val movement: Movement, val direction: Direction)

fun min3(a: Int, b: Int, c: Int) = min(min(a, b), c)
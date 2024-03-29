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

    @Synchronized
    operator fun get(row: Int, col: Int): Int = mazeProperties[row][col].value

    @Synchronized
    operator fun set(row: Int, col: Int, value: Int) {
        mazeProperties[row][col].value = value
    }


    private constructor(maze: MazeModel) : this() {
        for (row in 0 until MAZE_ROWS) {
            for (col in 0 until MAZE_COLUMNS) {
                this[row, col] = maze[row, col]
            }
        }
    }

    val isFullyExplored: Boolean
        get() {
            for (row in 0 until MAZE_ROWS) {
                for (col in 0 until MAZE_COLUMNS) {
                    if (this[row, col] == CELL_UNKNOWN) {
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
                    this[row, col] = 1
                } else {
                    this[row, col] = CELL_UNKNOWN
                }
            }
        }
    }

    fun resetForFastestPath() {
        for (row in 0 until MAZE_ROWS) {
            for (col in 0 until MAZE_COLUMNS) {
                if (this[row, col] < 0) {
                    this[row, col] = CELL_OBSTACLE
                } else {
                    this[row, col] = CELL_SENSED
                }
            }
        }
    }

    fun reset() {
        for (row in 0 until MAZE_ROWS) {
            for (col in 0 until MAZE_COLUMNS) {
                this[row, col] = 0
            }
        }
    }

    fun getSide(centerCell: CellInfoModel, movement: Movement): IntArray {
        val (centerRow, centerCol, direction) = centerCell
        val (rowDiff, colDiff) = SIDES[direction.ordinal][movement.ordinal]
        val side = IntArray(3)
        if (rowDiff != 0) {
            val rowOfSide = centerRow + rowDiff
            if (rowOfSide < 0 || rowOfSide >= MAZE_ROWS) {
                side.fill(CELL_OBSTACLE)
            } else {
                side[0] = this[rowOfSide, centerCol - 1]
                side[1] = this[rowOfSide, centerCol]
                side[2] = this[rowOfSide, centerCol + 1]
            }
        } else {
            check(colDiff != 0)
            val columnOfSide = centerCol + colDiff
            if (columnOfSide < 0 || columnOfSide >= MAZE_COLUMNS) { // Outside of maze, fill with CELL_OBSTACLES
                side.fill(CELL_OBSTACLE)
            } else {
                side[0] = this[centerRow - 1, columnOfSide]
                side[1] = this[centerRow, columnOfSide]
                side[2] = this[centerRow + 1, columnOfSide]
            }
        }
        return side
    }

    fun getEnvironmentOnSides(cellInfoModel: CellInfoModel): IntArray {
        val minSides = IntArray(3)
        for (movement in Movement.values()) {
            val side = getSide(cellInfoModel, movement)
            val min = min3(side[0], side[1], side[2])
            minSides[movement.ordinal] = min
        }
        return minSides
    }

    fun calculateCoverage(): Double {
        var sum = 0
        for (row in 0 until MAZE_ROWS) {
            for (col in 0 until MAZE_COLUMNS) {
                if (this[row, col] != CELL_UNKNOWN) {
                    sum++
                }
            }
        }
        return sum * 100.0 / (MAZE_ROWS * MAZE_COLUMNS)
    }

    fun outputMapDescriptorPart1(): String {
        val result = StringBuilder()
        val binary = StringBuilder()

        binary.append("11")
        for (row in 0 until MAZE_ROWS) {
            for (col in 0 until MAZE_COLUMNS) {
                if (this[row, col] == CELL_UNKNOWN) {
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

    fun outputMapDescriptorPart2(): String {
        val result = StringBuilder()
        val binary = StringBuilder()
        for (row in 0 until MAZE_ROWS) {
            for (col in 0 until MAZE_COLUMNS) {
                if (this[row, col] != CELL_UNKNOWN) {
                    if (this[row, col] == CELL_OBSTACLE) {
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
                this[row, col] = newMaze[row][col]
            }
        }
    }

    fun print() {
        for (row in MAZE_ROWS - 1 downTo 0) {
            for (col in 0 until MAZE_COLUMNS) {
                if (col != 0) {
                    print(" ")
                }
                System.out.printf("%3d", this[row, col])
            }
            println()
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

    fun reset() {
        row = 1
        col = 1
        direction = Direction.UP
    }
}

data class MovementInfo(val movement: Movement, val direction: Direction)

fun min3(a: Int, b: Int, c: Int) = min(min(a, b), c)
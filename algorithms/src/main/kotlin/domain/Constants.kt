package domain

import model.CellInfoModel
import model.MovementInfo

enum class Direction {
    UP, DOWN, LEFT, RIGHT;

    fun turnRight() = when (this) {
        UP -> RIGHT
        DOWN -> LEFT
        LEFT -> UP
        RIGHT -> DOWN
    }

    fun turnLeft() = when (this) {
        UP -> LEFT
        DOWN -> RIGHT
        LEFT -> DOWN
        RIGHT -> UP
    }

    operator fun minus(other: Direction) = DIRECTION_TURNS[other.ordinal][this.ordinal]
}

enum class Movement { TURN_RIGHT, MOVE_FORWARD, TURN_LEFT }

const val MAZE_COLUMNS = 15
const val MAZE_ROWS = 20

val SENSE_RANGE_SHORT = 0..6
val SENSE_RANGE_LONG = 3..9
const val NUMBER_OF_SENSORS = 6

const val CELL_OBSTACLE = -2
const val CELL_UNKNOWN = -1
const val CELL_SENSED = 0

const val ROBOT_SIZE = 3

val NEXT_CELL = mapOf(
    MovementInfo(Movement.TURN_RIGHT, Direction.UP) to CellInfoModel(0, 1, Direction.RIGHT),
    MovementInfo(Movement.TURN_RIGHT, Direction.DOWN) to CellInfoModel(0, -1, Direction.LEFT),
    MovementInfo(Movement.TURN_RIGHT, Direction.LEFT) to CellInfoModel(1, 0, Direction.UP),
    MovementInfo(Movement.TURN_RIGHT, Direction.RIGHT) to CellInfoModel(-1, 0, Direction.DOWN),
    MovementInfo(Movement.MOVE_FORWARD, Direction.UP) to CellInfoModel(1, 0, Direction.UP),
    MovementInfo(Movement.MOVE_FORWARD, Direction.DOWN) to CellInfoModel(-1, 0, Direction.DOWN),
    MovementInfo(Movement.MOVE_FORWARD, Direction.LEFT) to CellInfoModel(0, -1, Direction.LEFT),
    MovementInfo(Movement.MOVE_FORWARD, Direction.RIGHT) to CellInfoModel(0, 1, Direction.RIGHT),
    MovementInfo(Movement.TURN_LEFT, Direction.UP) to CellInfoModel(0, -1, Direction.LEFT),
    MovementInfo(Movement.TURN_LEFT, Direction.DOWN) to CellInfoModel(0, 1, Direction.RIGHT),
    MovementInfo(Movement.TURN_LEFT, Direction.LEFT) to CellInfoModel(-1, 0, Direction.DOWN),
    MovementInfo(Movement.TURN_LEFT, Direction.RIGHT) to CellInfoModel(1, 0, Direction.UP)
)

val SIDES = arrayOf(
    arrayOf(
        CellInfoModel(0, 1 + 1, Direction.RIGHT),
        CellInfoModel(1 + 1, 0, Direction.UP),
        CellInfoModel(0, -1 - 1, Direction.LEFT)
    ),
    arrayOf(
        CellInfoModel(0, -1 - 1, Direction.LEFT),
        CellInfoModel(-1 - 1, 0, Direction.DOWN),
        CellInfoModel(0, 1 + 1, Direction.RIGHT)
    ),
    arrayOf(
        CellInfoModel(1 + 1, 0, Direction.UP),
        CellInfoModel(0, -1 - 1, Direction.LEFT),
        CellInfoModel(-1 - 1, 0, Direction.DOWN)
    ),
    arrayOf(
        CellInfoModel(-1 - 1, 0, Direction.DOWN),
        CellInfoModel(0, 1 + 1, Direction.RIGHT),
        CellInfoModel(1 + 1, 0, Direction.UP)
    )
)

private val DIRECTION_TURNS = arrayOf(
    arrayOf(
        arrayOf(),
        arrayOf(Movement.TURN_LEFT, Movement.TURN_LEFT),
        arrayOf(Movement.TURN_LEFT),
        arrayOf(Movement.TURN_RIGHT)
    ),
    arrayOf(
        arrayOf(Movement.TURN_LEFT, Movement.TURN_LEFT),
        arrayOf(),
        arrayOf(Movement.TURN_RIGHT),
        arrayOf(Movement.TURN_LEFT)
    ),
    arrayOf(
        arrayOf(Movement.TURN_RIGHT),
        arrayOf(Movement.TURN_LEFT),
        arrayOf(),
        arrayOf(Movement.TURN_LEFT, Movement.TURN_LEFT)
    ),
    arrayOf(
        arrayOf(Movement.TURN_LEFT),
        arrayOf(Movement.TURN_RIGHT),
        arrayOf(Movement.TURN_LEFT, Movement.TURN_RIGHT),
        arrayOf()
    )
)

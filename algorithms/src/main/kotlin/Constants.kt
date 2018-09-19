enum class Direction {
    UP, DOWN, LEFT, RIGHT;

    fun turnRight() = when (this) {
        Direction.UP -> Direction.RIGHT
        Direction.DOWN -> Direction.LEFT
        Direction.LEFT -> Direction.UP
        Direction.RIGHT -> Direction.DOWN
    }

    fun turnLeft() = when (this) {
        Direction.UP -> Direction.LEFT
        Direction.DOWN -> Direction.RIGHT
        Direction.LEFT -> Direction.DOWN
        Direction.RIGHT -> Direction.UP
    }

    operator fun minus(other: Direction) = DIRECTION_TURNS[other.ordinal][this.ordinal]
}

enum class Movement { TURN_RIGHT, MOVE_FORWARD, TURN_LEFT }

const val MAZE_COLUMNS = 15
const val MAZE_ROWS = 20

const val SENSE_RANGE_SHORT = 3
const val SENSE_RANGE_LONG = 2

const val CELL_OBSTACLE = -2
const val CELL_UNKNOWN = -1
const val CELL_SENSED = 0

const val ROBOT_SIZE = 3

val CELL_DIFF = mapOf(
    CellInfo(-1, 0, Direction.DOWN) to MovementInfo(Movement.MOVE_FORWARD, Direction.DOWN),
    CellInfo(-1, 0, Direction.LEFT) to MovementInfo(Movement.TURN_LEFT, Direction.DOWN),
    CellInfo(-1, 0, Direction.RIGHT) to MovementInfo(Movement.TURN_RIGHT, Direction.DOWN),
    CellInfo(0, 1, Direction.RIGHT) to MovementInfo(Movement.MOVE_FORWARD, Direction.RIGHT),
    CellInfo(0, 1, Direction.UP) to MovementInfo(Movement.TURN_RIGHT, Direction.RIGHT),
    CellInfo(0, 1, Direction.DOWN) to MovementInfo(Movement.TURN_LEFT, Direction.RIGHT),
    CellInfo(0, -1, Direction.LEFT) to MovementInfo(Movement.MOVE_FORWARD, Direction.LEFT),
    CellInfo(0, -1, Direction.UP) to MovementInfo(Movement.TURN_LEFT, Direction.LEFT),
    CellInfo(0, -1, Direction.DOWN) to MovementInfo(Movement.TURN_RIGHT, Direction.LEFT),
    CellInfo(1, 0, Direction.UP) to MovementInfo(Movement.MOVE_FORWARD, Direction.UP),
    CellInfo(1, 0, Direction.LEFT) to MovementInfo(Movement.TURN_RIGHT, Direction.UP),
    CellInfo(1, 0, Direction.RIGHT) to MovementInfo(Movement.TURN_LEFT, Direction.UP)
)

val NEXT_CELL = mapOf(
    MovementInfo(Movement.TURN_RIGHT, Direction.UP) to CellInfo(0, 1, Direction.RIGHT),
    MovementInfo(Movement.TURN_RIGHT, Direction.DOWN) to CellInfo(0, -1, Direction.LEFT),
    MovementInfo(Movement.TURN_RIGHT, Direction.LEFT) to CellInfo(1, 0, Direction.UP),
    MovementInfo(Movement.TURN_RIGHT, Direction.RIGHT) to CellInfo(-1, 0, Direction.DOWN),
    MovementInfo(Movement.MOVE_FORWARD, Direction.UP) to CellInfo(1, 0, Direction.UP),
    MovementInfo(Movement.MOVE_FORWARD, Direction.DOWN) to CellInfo(-1, 0, Direction.DOWN),
    MovementInfo(Movement.MOVE_FORWARD, Direction.LEFT) to CellInfo(0, -1, Direction.LEFT),
    MovementInfo(Movement.MOVE_FORWARD, Direction.RIGHT) to CellInfo(0, 1, Direction.RIGHT),
    MovementInfo(Movement.TURN_LEFT, Direction.UP) to CellInfo(0, -1, Direction.LEFT),
    MovementInfo(Movement.TURN_LEFT, Direction.DOWN) to CellInfo(0, 1, Direction.RIGHT),
    MovementInfo(Movement.TURN_LEFT, Direction.LEFT) to CellInfo(-1, 0, Direction.DOWN),
    MovementInfo(Movement.TURN_LEFT, Direction.RIGHT) to CellInfo(1, 0, Direction.UP)
)

val SIDES = arrayOf(
    arrayOf(
        CellInfo(0, 1 + 1, Direction.RIGHT),
        CellInfo(1 + 1, 0, Direction.UP),
        CellInfo(0, -1 - 1, Direction.LEFT)
    ),
    arrayOf(
        CellInfo(0, -1 - 1, Direction.LEFT),
        CellInfo(-1 - 1, 0, Direction.DOWN),
        CellInfo(0, 1 + 1, Direction.RIGHT)
    ),
    arrayOf(
        CellInfo(1 + 1, 0, Direction.UP),
        CellInfo(0, -1 - 1, Direction.LEFT),
        CellInfo(-1 - 1, 0, Direction.DOWN)
    ),
    arrayOf(
        CellInfo(-1 - 1, 0, Direction.DOWN),
        CellInfo(0, 1 + 1, Direction.RIGHT),
        CellInfo(1 + 1, 0, Direction.UP)
    )
)

private val DIRECTION_TURNS = arrayOf(
    arrayOf(
        listOf(),
        listOf(Movement.TURN_LEFT, Movement.TURN_LEFT),
        listOf(Movement.TURN_LEFT),
        listOf(Movement.TURN_RIGHT)
    ),
    arrayOf(
        listOf(Movement.TURN_LEFT, Movement.TURN_LEFT),
        listOf(),
        listOf(Movement.TURN_RIGHT),
        listOf(Movement.TURN_LEFT)
    ),
    arrayOf(
        listOf(Movement.TURN_RIGHT),
        listOf(Movement.TURN_LEFT),
        listOf(),
        listOf(Movement.TURN_LEFT, Movement.TURN_LEFT)
    ),
    arrayOf(
        listOf(Movement.TURN_LEFT),
        listOf(Movement.TURN_RIGHT),
        listOf(Movement.TURN_LEFT, Movement.TURN_RIGHT),
        listOf()
    )
)

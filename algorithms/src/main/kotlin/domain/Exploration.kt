package domain

import com.fasterxml.jackson.annotation.JsonIgnore
import com.fasterxml.jackson.annotation.JsonProperty
import kotlinx.coroutines.withTimeoutOrNull
import model.CellInfoModel
import java.util.*
import java.util.concurrent.TimeUnit

data class ExplorationJson(@get:JsonProperty("stack") val stack: List<CenterCell>) : JsonSerializable {
    @JsonIgnore
    override val filename = "Exploration"
}

open class Exploration(private val robot: Robot, private val connection: Connection) {
    private var calibratedAtTopRightCorner = false
    private var calibratedAtTopLeftCorner = false
    private var calibratedAtBottomRightCorner = false

    private val stack = Stack<CellInfoModel>().apply {
        push(robot.centerCell.copy())
    }

    open suspend fun explore() {
        while (!exploreInternal(100.0)) {
        }
        if (!robot.isAtStartZone) {
            robot.goToStartZone()
        }
        robot.turnToFaceUp()
        if (connection.isConnected) {
            connection.sendStopCommand()
        }
        robot.calibrateForFastestPath()
    }

    protected suspend fun exploreInternal(coverageLimit: Double): Boolean {
        var coverage: Double
        var shouldBacktrack: Boolean

        do {
            tryToCalibrateAtCorner()

            val senseAgainMovement = robot.sense()
            if (senseAgainMovement != null) {
                // Find conflicts in the sensorData, turn and sense again
                when (senseAgainMovement) {
                    Movement.TURN_LEFT -> {
                        robot.move(Movement.TURN_LEFT)
                        robot.sense(forceUpdate = true)
                        robot.move(Movement.TURN_RIGHT)
                        robot.sense(forceUpdate = true)
                    }
                    Movement.TURN_RIGHT -> {
                        robot.move(Movement.TURN_RIGHT)
                        robot.sense(forceUpdate = true)
                        robot.move(Movement.TURN_LEFT)
                        robot.sense(forceUpdate = true)
                    }
                    else -> {
                    }
                }
            }

            val explorationMaze = robot.explorationMaze
            val sides = explorationMaze.getEnvironmentOnSides(robot.centerCell.copy())
            println("Center: ${robot.centerCell}, Sides: ${sides.joinToString()}")
            shouldBacktrack = sides.all { it > 0 || it == CELL_OBSTACLE }
            if (!shouldBacktrack) {
                makeMovingDecision(sides)
            }
            coverage = explorationMaze.calculateCoverage()
        } while (coverage < coverageLimit && !shouldBacktrack)
        return if (coverage >= coverageLimit) {
            if (!robot.isAtStartZone) {
                robot.goToStartZone()
            }
            true
        } else {
            backtrack()
        }
    }

    private suspend fun tryToCalibrateAtCorner() {
        val (row, col, direction) = robot.centerCell
        if (row == MAZE_ROWS - 1 - 1) {
            if (col == MAZE_COLUMNS - 1 - 1 && !calibratedAtTopRightCorner) {
                robot.calibrateAtCorner(row, col, direction)
                calibratedAtTopRightCorner = true
            } else if (col == 1 && !calibratedAtTopLeftCorner) {
                robot.calibrateAtCorner(row, col, direction)
                calibratedAtTopLeftCorner = true
            }
        } else if (row == 1) {
            if (col == MAZE_COLUMNS - 1 - 1 && !calibratedAtBottomRightCorner) {
                robot.calibrateAtCorner(row, col, direction)
                calibratedAtBottomRightCorner = true
            }
        }
    }

    private suspend fun makeMovingDecision(sides: IntArray) {
        val choices = Movement.values().sortedWith(Comparator { o1, o2 ->
            when {
                sides[o1.ordinal] == sides[o2.ordinal] -> 0
                sides[o1.ordinal] == CELL_OBSTACLE -> 1
                sides[o2.ordinal] == CELL_OBSTACLE -> -1
                sides[o1.ordinal] < sides[o2.ordinal] -> -1
                else -> 1
            }
        })
        val finalChoice = choices.first()
        when {
            sides[finalChoice.ordinal] == CELL_UNKNOWN -> when (finalChoice) {
                Movement.TURN_RIGHT, Movement.MOVE_FORWARD -> turnRight()
                Movement.TURN_LEFT -> turnLeft()
            }
            finalChoice == Movement.TURN_RIGHT -> {
                turnRight()
                moveForward()
            }
            finalChoice == Movement.TURN_LEFT -> {
                turnLeft()
                moveForward()
            }
            finalChoice == Movement.MOVE_FORWARD -> moveForward()
        }
    }

    private suspend fun backtrack(): Boolean {
        println("Backtracking")
        while (stack.isNotEmpty()) {
            val cell = stack.pop()
            saveJson(ExplorationJson(stack.map { CenterCell(it.row, it.col, it.direction) }))
            val sides = robot.explorationMaze.getEnvironmentOnSides(cell)
            println("Checking $cell, sides=${sides.joinToString()}")
            if (sides.any { it == CELL_UNKNOWN || it == CELL_SENSED }) {
                val pathsToCell = findFastestPathToDestination(
                    robot.explorationMaze.copy(),
                    robot.centerCell.copy(),
                    cell.row to cell.col
                )
                if (pathsToCell.size <= cell.direction.ordinal || pathsToCell[cell.direction.ordinal].isEmpty()) {
                    throw IllegalStateException("Unable to go back to cell $cell")
                }
                val movement = Movement.values()[sides.indexOfFirst { it == CELL_UNKNOWN || it == CELL_SENSED }]
                val direction = when (movement) {
                    Movement.TURN_RIGHT -> cell.direction.turnRight()
                    Movement.TURN_LEFT -> cell.direction.turnLeft()
                    Movement.MOVE_FORWARD -> cell.direction
                }
                val pathToCell = pathsToCell[direction.ordinal]
                val movements = pathToCell.toMovements()
                robot.moveFollowingMovements(movements)
                return false
            }
        }
        return true
    }

    private suspend fun moveForward() {
        robot.move(Movement.MOVE_FORWARD)
        stack += robot.centerCell.copy()
        saveJson(ExplorationJson(stack.map { CenterCell(it.row, it.col, it.direction) }))
    }

    private suspend fun turnLeft() {
        robot.move(Movement.TURN_LEFT)
        stack += robot.centerCell.copy()
        saveJson(ExplorationJson(stack.map { CenterCell(it.row, it.col, it.direction) }))
    }

    private suspend fun turnRight() {
        robot.move(Movement.TURN_RIGHT)
        stack += robot.centerCell.copy()
        saveJson(ExplorationJson(stack.map { CenterCell(it.row, it.col, it.direction) }))
    }
}

class TimeLimitedExploration(robot: Robot, connection: Connection, private val timeLimit: Long) :
    Exploration(robot, connection) {
    override suspend fun explore() {
        withTimeoutOrNull(TimeUnit.SECONDS.toMillis(timeLimit)) { super.explore() }
    }
}

class CoverageLimitedExploration(robot: Robot, connection: Connection, private val coverageLimit: Double) :
    Exploration(robot, connection) {
    override suspend fun explore() {
        while (!exploreInternal(coverageLimit)) {
        }
    }
}
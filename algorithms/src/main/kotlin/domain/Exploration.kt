package domain

import kotlinx.coroutines.experimental.withTimeoutOrNull
import model.CellInfoModel
import java.util.*
import java.util.concurrent.TimeUnit

open class Exploration(private val robot: Robot) {
    private val stack = Stack<CellInfoModel>().apply {
        push(robot.centerCell.copy())
    }

    open suspend fun explore() {
        while (!exploreInternal(100.0)) {
        }
        if (!robot.isAtStartZone) {
            robot.goToStartZone()
        }
    }

    protected suspend fun exploreInternal(coverageLimit: Double): Boolean {
        var coverage: Double
        var shouldBacktrack: Boolean
        do {
            robot.sense()
            val explorationMaze = robot.explorationMaze
            val sides = explorationMaze.getEnvironmentOnSides(robot.centerCell.copy())
            println("Center: ${robot.centerCell}, Sides: ${sides.joinToString()}")
//            shouldBacktrack = sides.all { it > 0 || it == CELL_OBSTACLE } && !sides.all { it == CELL_OBSTACLE }
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
//            sides[finalChoice.ordinal] == CELL_OBSTACLE -> {
//                turnLeft()
//                turnLeft()
//                moveForward()
//            }
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
        robot.turnToFaceUp()
        return true
    }

    private suspend fun moveForward() {
        robot.move(Movement.MOVE_FORWARD)
        stack += robot.centerCell.copy()
    }

    private suspend fun turnLeft() {
        robot.move(Movement.TURN_LEFT)
        stack += robot.centerCell.copy()
    }

    private suspend fun turnRight() {
        robot.move(Movement.TURN_RIGHT)
        stack += robot.centerCell.copy()
    }
}

class TimeLimitedExploration(robot: Robot, private val timeLimit: Long) :
    Exploration(robot) {
    override suspend fun explore() {
        withTimeoutOrNull(TimeUnit.SECONDS.toMillis(timeLimit)) { super.explore() }
    }
}

class CoverageLimitedExploration(robot: Robot, private val coverageLimit: Double) :
    Exploration(robot) {
    override suspend fun explore() {
        while (!exploreInternal(coverageLimit)) {
        }
    }
}
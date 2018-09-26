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
    }

    protected suspend fun exploreInternal(coverageLimit: Double): Boolean {
        do {
            robot.sense()
            makeMovingDecision()
        } while (robot.explorationMaze.calculateCoverage() < coverageLimit && !robot.isAtStartZone)
        when {
            robot.explorationMaze.isFullyExplored -> {
                if (!robot.isAtStartZone) {
                    robot.goToStartZone()
                }
                return true
            }
            robot.explorationMaze.calculateCoverage() >= coverageLimit -> return true
            else -> {
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
                        val pathToCell = pathsToCell[cell.direction.ordinal]
                        val movements = pathToCell.toMovements()
                        robot.moveFollowingMovements(movements)
                        return false
                    }
                }
                robot.turnToFaceUp()
                return true
            }
        }
    }

    private suspend fun makeMovingDecision() {
        val explorationMaze = robot.explorationMaze
        val sides = explorationMaze.getEnvironmentOnSides(robot.centerCell.copy())
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
            sides[finalChoice.ordinal] == CELL_OBSTACLE -> {
                turnLeft()
                turnLeft()
                moveForward()
            }
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

    private suspend fun moveForward() {
        robot.moveForward()
        stack += robot.centerCell.copy()
    }

    private suspend fun turnLeft() {
        robot.turnLeft()
        stack += robot.centerCell.copy()
    }

    private suspend fun turnRight() {
        robot.turnRight()
        stack += robot.centerCell.copy()
    }
}

class TimeLimitedExploration(robot: Robot, private val timeLimit: Long) : Exploration(robot) {
    override suspend fun explore() {
        withTimeoutOrNull(timeLimit, TimeUnit.SECONDS) {
            super.explore()
        }
    }
}

class CoverageLimitedExploration(robot: Robot, private val coverageLimit: Double) : Exploration(robot) {
    override suspend fun explore() {
        while (!exploreInternal(coverageLimit)) {
        }
    }
}
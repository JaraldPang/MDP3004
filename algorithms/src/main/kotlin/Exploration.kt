import java.util.*
import java.util.concurrent.Executors
import java.util.concurrent.TimeUnit
import java.util.concurrent.TimeoutException

fun main(args: Array<String>) {
    val realMaze = Maze().apply { loadFromDisk("Week11.txt") }
    val robot = Robot()

    val sensors = List(9) {
        SimulatedSensor(it, 0..0, robot, realMaze)
    }
    robot.setSensors(sensors)
    explore(robot, 0)
    println("Robot: Exploration finished: Coverage=${robot.explorationMaze.calculateCoverage()}%")
    println("Robot: Number of movements: ${robot.movementCount}")
    robot.explorationMaze.prettyPrintWithRobot(robot)

    val robot2 = Robot()
    val sensors2 = listOf(2, 3, 4, 5, 6).map {
        SimulatedSensor(it, 0..0, robot2, realMaze)
    }
    robot2.setSensors(sensors2)
    explore(robot2, 0)
    println("Robot2: Exploration finished: Coverage=${robot2.explorationMaze.calculateCoverage()}%")
    println("Robot2: Number of movements: ${robot2.movementCount}")
    robot2.explorationMaze.prettyPrintWithRobot(robot2)

//    Thread.sleep(2000L)
//    val movements = findFastestPath(robot.explorationMaze.copy())
//    robot.moveFollowingMovements(movements)

//    exploreWithTimeLimit(robot, realMaze, 2, 1000)

//    exploreWithCoverageLimit(robot, realMaze, 2, 30.0)
}

private fun explore(robot: Robot, speed: Int = 2) {
    robot.speed = speed
    val stack = Stack<CellInfo>()
    stack += robot.centerCell
    while (!exploreInternal(robot, stack)) {
    }
}

private fun exploreWithCoverageLimit(robot: Robot, speed: Int, coverageLimit: Double) {
    robot.speed = speed
    val stack = Stack<CellInfo>()
    stack += robot.centerCell
    exploreInternal(robot, stack, coverageLimit)
}

private fun exploreWithTimeLimit(robot: Robot, speed: Int, timeLimit: Long) {
    robot.speed = speed
    val service = Executors.newSingleThreadExecutor()
    val future = service.submit { explore(robot) }
    try {
        future.get(timeLimit, TimeUnit.SECONDS)
    } catch (e: TimeoutException) {
        println("Timed out after $timeLimit seconds")
        future.cancel(true)
    }
    service.shutdownNow()
}

private fun exploreInternal(robot: Robot, stack: Stack<CellInfo>, coverageLimit: Double = 100.0): Boolean {
    do {
        robot.sense()
        makeMovingDecision(robot, stack)
    } while (robot.explorationMaze.calculateCoverage() < coverageLimit && !robot.isAtStartZone)
    if (robot.explorationMaze.isFullyExplored) {
        if (!robot.isAtStartZone) {
            robot.goToStartZone()
        }
        return true
    } else {
        while (stack.isNotEmpty()) {
            val cell = stack.pop()
            val sides = robot.explorationMaze.getEnvironmentOnSides(cell)
            if (sides.any { it == CELL_UNKNOWN || it == CELL_SENSED }) {
                val pathsToCell = findFastestPathToDestination(
                    robot.explorationMaze.copy(),
                    robot.centerCell,
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

private fun makeMovingDecision(robot: Robot, stack: Stack<CellInfo>) {
    val explorationMaze = robot.explorationMaze
    val sides = explorationMaze.getEnvironmentOnSides(robot.centerCell)
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
            robot.turnLeft()
            stack += robot.centerCell
            robot.turnLeft()
            stack += robot.centerCell
        }
        sides[finalChoice.ordinal] == CELL_UNKNOWN -> {
            stack += when (finalChoice) {
                Movement.TURN_RIGHT, Movement.MOVE_FORWARD -> {
                    robot.turnRight()
                    robot.centerCell
                }
                Movement.TURN_LEFT -> {
                    robot.turnLeft()
                    robot.centerCell
                }
            }
            return
        }
        finalChoice == Movement.TURN_RIGHT -> {
            robot.turnRight()
            stack += robot.centerCell
        }
        finalChoice == Movement.TURN_LEFT -> {
            robot.turnLeft()
            stack += robot.centerCell
        }
    }
    robot.moveForward()
    stack += robot.centerCell
}
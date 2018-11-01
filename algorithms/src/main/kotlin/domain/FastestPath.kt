package domain

import model.CellInfoModel
import model.MazeModel
import java.util.*

open class FastestPath(
    private val connection: Connection,
    protected val robot: Robot,
    protected val start: CellInfoModel = robot.centerCell.copy(),
    protected val dest: Pair<Int, Int> = MAZE_ROWS - 1 - 1 to MAZE_COLUMNS - 1 - 1
) {
    suspend fun runFastestPath() {
        val movements = findFastestPathMovements()
        robot.moveFollowingMovements(movements, true)
//        if (connection.isConnected) {
//            connection.sendStopCommand()
//        }
    }

    open fun findFastestPathMovements(): List<Movement> {
        val paths = findFastestPathToDestination(robot.explorationMaze.copy(), start, dest)
        val minPath = paths.asSequence()
            .filter { it.isNotEmpty() }
            .minBy { it.size }
        return minPath?.toMovements() ?: listOf()
    }
}

class FastestPathWithWayPoint(
    connection: Connection,
    robot: Robot,
    private val wayPoint: Pair<Int, Int>,
    start: CellInfoModel = robot.centerCell.copy(),
    dest: Pair<Int, Int> = MAZE_ROWS - 1 - 1 to MAZE_COLUMNS - 1 - 1
) : FastestPath(connection, robot, start, dest) {
    override fun findFastestPathMovements(): List<Movement> {
        val realMaze = robot.explorationMaze.copy()
        val pathsToWayPoint = findFastestPathToDestination(realMaze, start, wayPoint).filter { it.isNotEmpty() }
        if (pathsToWayPoint.isEmpty()) {
            println("Cannot go to way point (${wayPoint.first}, ${wayPoint.second})")
            return super.findFastestPathMovements()
        }
        val pathsToDest = Direction.values().flatMap { direction ->
            val wayPointCell = CellInfoModel(wayPoint.first, wayPoint.second, direction)
            findFastestPathToDestination(realMaze, wayPointCell, dest).filter { it.isNotEmpty() }
        }
        val combinedPaths = mutableListOf<List<CellInfoModel>>()
        for (pathToWayPoint in pathsToWayPoint) {
            for (pathToDest in pathsToDest) {
                val lastInFirstSegment = pathToWayPoint.last()
                val firstInSecondSegment = pathToDest.first()
                val combinedPath = when (firstInSecondSegment.direction) {
                    lastInFirstSegment.direction -> pathToWayPoint.dropLast(1) + pathToDest
                    lastInFirstSegment.direction.turnLeft() -> pathToWayPoint + pathToDest
                    lastInFirstSegment.direction.turnRight() -> pathToWayPoint + pathToDest
                    else -> pathToWayPoint + CellInfoModel(
                        lastInFirstSegment.row,
                        lastInFirstSegment.col,
                        lastInFirstSegment.direction.turnLeft()
                    ) + pathToDest
                }
                combinedPaths += combinedPath
            }
        }
        val minPath = combinedPaths.minBy { it.size }
        return minPath?.toMovements() ?: listOf()
    }
}

fun dijkstra(
    realMaze: MazeModel,
    start: CellInfoModel,
    backtrace: MutableMap<CellInfoModel, CellInfoModel>,
    distanceMap: MutableMap<CellInfoModel, Int>
) {
    realMaze.resetForFastestPath()

    backtrace[start] = start
    distanceMap[start] = 0
    val queue = PriorityQueue<Pair<CellInfoModel, Int>> { o1, o2 -> o1.second.compareTo(o2.second) }
    queue.add(start to 0)

    while (queue.isNotEmpty()) {
        val (cell, distance) = queue.remove()
        val existingDistance = distanceMap[cell]
        if (existingDistance != null && existingDistance < distance) {
            continue
        }
        val sides = realMaze.getEnvironmentOnSides(CellInfoModel(cell.row, cell.col, cell.direction))

        for (movement in Movement.values()) {
            if (movement == Movement.MOVE_FORWARD) {
                val side = sides[Movement.MOVE_FORWARD.ordinal]
                // Should not move into an obstacle or unknown space
                if (side == CELL_OBSTACLE || side == CELL_OBSTACLE) {
                    continue
                }
            }
            val nextCell = cell + movement
            val distanceOfNextCell = distanceMap[nextCell]
            val updatedDistance = distance + 1
            if (distanceOfNextCell == null || updatedDistance < distanceOfNextCell) {
                backtrace[nextCell] = cell
                distanceMap[nextCell] = updatedDistance
                queue += nextCell to updatedDistance
            }
        }
    }
}

fun buildPath(dest: CellInfoModel, backtrace: Map<CellInfoModel, CellInfoModel>): List<CellInfoModel> {
    val path = LinkedList<CellInfoModel>()
    var cell = dest
    while (true) {
        val prevCell = backtrace[cell]
        if (prevCell != null) {
            path.addFirst(cell)
            if (prevCell == cell) {
                break
            }
            cell = prevCell
        } else {
            break
        }
    }
    return path
}

fun findFastestPathToDestination(
    realMaze: MazeModel,
    start: CellInfoModel,
    dest: Pair<Int, Int>
): List<List<CellInfoModel>> {
    val backtrace = mutableMapOf<CellInfoModel, CellInfoModel>()
    val distanceMap = mutableMapOf<CellInfoModel, Int>()
    dijkstra(realMaze, start, backtrace, distanceMap)

    return Direction.values().map { buildPath(CellInfoModel(dest.first, dest.second, it), backtrace) }
}

fun List<CellInfoModel>.toMovements(): List<Movement> {
    val movements = mutableListOf<Movement>()
    for (i in 1 until size) {
        movements += this[i] - this[i - 1]
    }
    return movements
}
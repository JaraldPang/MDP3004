import java.util.*

fun main(args: Array<String>) {
    val realMaze = Maze()
    realMaze.loadFromDisk("sample_arena.txt")
    findFastestPath(realMaze)
    findFastestPathWithWayPoint(realMaze, 17 to 1)
}

fun findFastestPath(realMaze: Maze): List<Movement> {
    val start = CellInfo(1, 1, Direction.UP)
    val dest = MAZE_ROWS - 1 - 1 to MAZE_COLUMNS - 1 - 1
    val paths = findFastestPathToDestination(realMaze, start, dest)
    val minPath = paths.asSequence()
        .filter { it.isNotEmpty() }
        .minBy { it.size }
    return if (minPath != null) {
        println("$minPath, size=${minPath.size}")
        val movements = minPath.toMovements()
        println("$movements, size=${movements.size}")
        movements
    } else {
        println("Not accessible")
        listOf()
    }
}

fun findFastestPathWithWayPoint(realMaze: Maze, wayPoint: Pair<Int, Int>): List<Movement> {
    val start = CellInfo(1, 1, Direction.UP)
    val pathsToWayPoint = findFastestPathToDestination(realMaze, start, wayPoint).filter { it.isNotEmpty() }
    if (pathsToWayPoint.isEmpty()) {
        println("Way point not accessible")
        return findFastestPath(realMaze)
    }
    val dest = MAZE_ROWS - 1 - 1 to MAZE_COLUMNS - 1 - 1
    val pathsToDest = Direction.values().flatMap { direction ->
        val wayPointCell = CellInfo(wayPoint.first, wayPoint.second, direction)
        findFastestPathToDestination(realMaze, wayPointCell, dest).filter { it.isNotEmpty() }
    }
    val combinedPaths = mutableListOf<List<CellInfo>>()
    for (pathToWayPoint in pathsToWayPoint) {
        for (pathToDest in pathsToDest) {
            val lastInFirstSegment = pathToWayPoint.last()
            val firstInSecondSegment = pathToDest.first()
            val combinedPath = when (firstInSecondSegment.direction) {
                lastInFirstSegment.direction -> pathToWayPoint.dropLast(1) + pathToDest
                lastInFirstSegment.direction.turnLeft() -> pathToWayPoint + pathToDest
                lastInFirstSegment.direction.turnRight() -> pathToWayPoint + pathToDest
                else -> pathToWayPoint + CellInfo(
                    lastInFirstSegment.row,
                    lastInFirstSegment.col,
                    lastInFirstSegment.direction.turnLeft()
                ) + pathToDest
            }
            combinedPaths += combinedPath
        }
    }
    val minPath = combinedPaths.minBy { it.size }
    return if (minPath != null) {
        println("$minPath, size=${minPath.size}")
        val movements = minPath.toMovements()
        println("$movements, size=${movements.size}")
        movements
    } else {
        println("Not accessible")
        listOf()
    }
}

fun findFastestPathToDestination(realMaze: Maze, start: CellInfo, dest: Pair<Int, Int>): List<List<CellInfo>> {
    realMaze.resetForFastestPath()

    val backtrace = mutableMapOf(start to start)
    val distanceMap = mutableMapOf(start to 0)
    val queue = PriorityQueue<Pair<CellInfo, Int>> { o1, o2 -> o1.second.compareTo(o2.second) }
    queue.add(start to 0)

    while (queue.isNotEmpty()) {
        val (cell, distance) = queue.remove()
        val existingDistance = distanceMap[cell]
        if (existingDistance != null && existingDistance < distance) {
            continue
        }
        val sides = realMaze.getEnvironmentOnSides(CellInfo(cell.row, cell.col, cell.direction))

        for (movement in Movement.values()) {
            if (movement == Movement.MOVE_FORWARD && sides[Movement.MOVE_FORWARD.ordinal] == CELL_OBSTACLE) {
                continue
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

    return Direction.values().map {
        val path = ArrayDeque<CellInfo>()
        var cell = CellInfo(dest.first, dest.second, it)
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
        path.toList()
    }
}

fun List<CellInfo>.toMovements(): List<Movement> {
    val movements = mutableListOf<Movement>()
    for (i in 1 until size) {
        movements += this[i] - this[i - 1]
    }
    return movements
}
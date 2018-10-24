package domain

import kotlinx.coroutines.channels.ReceiveChannel
import model.MazeModel

/**
 *  345
 * 2xxx6
 * 1xxx7
 * 0xxx8 -- position of the sensor, including the direction the sensor is facing
 * E.g., position 4 means the sensor is facing the front of the robot, sensing the left row/column related to the
 * center of the robot
 */
abstract class Sensor(val position: Int, val senseRange: IntRange) {
    /**
     * Returns how many cells away from the sensor is there an obstacle or a wall
     * If sensor finds no obstacles in the [senseRange], return -1.
     */
    abstract suspend fun sense(): Int
}

class SimulatedSensor(position: Int, senseRange: IntRange, private val robot: Robot, private val realMaze: MazeModel) :
    Sensor(position, senseRange) {

    override suspend fun sense(): Int {
        val (centerRow, centerCol, direction) = robot.centerCell
        val (rowDiff, colDiff, rowInc, colInc) = SENSOR_INFO[position][direction.ordinal]
        for (i in 0 until senseRange.first) {
            val row = centerRow + rowDiff + rowInc * i
            val col = centerCol + colDiff + colInc * i
            if (MazeModel.isOutsideOfMaze(row, col) || realMaze[row, col] == CELL_OBSTACLE) {
                return -1
            }
        }
        for (i in senseRange) {
            val row = centerRow + rowDiff + rowInc * i
            val col = centerCol + colDiff + colInc * i
            if (MazeModel.isOutsideOfMaze(row, col) || realMaze[row, col] == CELL_OBSTACLE) {
                return i
            }
        }
        return senseRange.last
    }
}

class ActualSensor(position: Int, sensedRange: IntRange, private val sensedDataChannel: ReceiveChannel<Int>) :
    Sensor(position, sensedRange) {

    override suspend fun sense(): Int {
        return sensedDataChannel.receive()
    }
}

val SENSOR_INFO = arrayOf(
    arrayOf(    // position = 0
        intArrayOf(-1, -2, 0, -1),  // UP: rowDiff, colDiff, rowInc, colInc
        intArrayOf(1, 2, 0, 1),     // DOWN
        intArrayOf(-2, 1, -1, 0),   // LEFT
        intArrayOf(2, -1, 1, 0)     // RIGHT
    ),
    arrayOf(    // position = 1
        intArrayOf(0, -2, 0, -1),   // UP
        intArrayOf(0, 2, 0, 1),     // DOWN
        intArrayOf(-2, 0, -1, 0),   // LEFT
        intArrayOf(2, 0, 1, 0)      // RIGHT
    ),
    arrayOf(    // position = 2
        intArrayOf(1, -2, 0, -1),   // UP
        intArrayOf(-1, 2, 0, 1),    // DOWN
        intArrayOf(-2, -1, -1, 0),  // LEFT
        intArrayOf(2, 1, 1, 0)      // RIGHT
    ),
    arrayOf(    // position = 3
        intArrayOf(2, -1, 1, 0),    // UP
        intArrayOf(-2, 1, -1, 0),   // DOWN
        intArrayOf(-1, -2, 0, -1),  // LEFT
        intArrayOf(1, 2, 0, 1)      // RIGHT
    ),
    arrayOf(    // position = 4
        intArrayOf(2, 0, 1, 0),     // UP
        intArrayOf(-2, 0, -1, 0),   // DOWN
        intArrayOf(0, -2, 0, -1),   // LEFT
        intArrayOf(0, 2, 0, 1)      // RIGHT
    ),
    arrayOf(    // position = 5
        intArrayOf(2, 1, 1, 0),     // UP
        intArrayOf(-2, -1, -1, 0),  // DOWN
        intArrayOf(1, -2, 0, -1),   // LEFT
        intArrayOf(-1, 2, 0, 1)     // RIGHT
    ),
    arrayOf(    // position = 6
        intArrayOf(1, 2, 0, 1),     // UP
        intArrayOf(-1, -2, 0, -1),  // DOWN
        intArrayOf(2, -1, 1, 0),    // LEFT
        intArrayOf(-2, 1, -1, 0)    // RIGHT
    ),
    arrayOf(    // position = 7
        intArrayOf(0, 2, 0, 1),     // UP
        intArrayOf(0, -2, 0, -1),   // DOWN
        intArrayOf(2, 0, 1, 0),     // LEFT
        intArrayOf(-2, 0, -1, 0)    // RIGHT
    ),
    arrayOf(    // position = 8
        intArrayOf(-1, 2, 0, 1),    // UP
        intArrayOf(1, -2, 0, -1),   // DOWN
        intArrayOf(2, 1, 1, 0),     // LEFT
        intArrayOf(-2, -1, -1, 0)   // RIGHT
    )
)
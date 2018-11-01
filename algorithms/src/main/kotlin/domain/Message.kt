package domain

import model.CellInfoModel

sealed class InputMessage {
    interface Parsable<T> {
        fun isParsableFrom(line: String): Boolean
        fun fromLine(line: String): T
    }

    companion object {
        val PARSABLES = listOf(
            Android.WayPoint,
            Android.StartExploration,
            Android.StartFastestPath,
            Arduino.Ok,
            Arduino.SensorData,
            Rpi.Arrow
        )
    }

    sealed class Android : InputMessage() {
        data class WayPoint(val x: Int, val y: Int) : Android() {
            companion object : Parsable<WayPoint> {
                override fun isParsableFrom(line: String) = line.startsWith("way")
                override fun fromLine(line: String): WayPoint {
                    val indexOfComma = line.indexOf(',')
                    val x = line.substring(4 until indexOfComma).toInt()
                    val y = line.substring(indexOfComma + 1 until line.lastIndex).toInt()
                    return WayPoint(x, y)
                }
            }
        }

        object StartExploration : Android(), Parsable<StartExploration> {
            override fun isParsableFrom(line: String) = line == "starte"
            override fun fromLine(line: String) = StartExploration
        }

        object StartFastestPath : Android(), Parsable<StartFastestPath> {
            override fun isParsableFrom(line: String) = line == "startf"
            override fun fromLine(line: String) = StartFastestPath
        }
    }

    sealed class Arduino : InputMessage() {
        object Ok : Arduino(), Parsable<Ok> {
            override fun isParsableFrom(line: String) = line == "ok"
            override fun fromLine(line: String) = Ok
        }

        data class SensorData(val sensorData: List<Int>) : Arduino() {
            companion object : Parsable<SensorData> {
                override fun isParsableFrom(line: String) = line.startsWith("sensor")
                override fun fromLine(line: String): SensorData {
                    val sensorData = line.substring(6).split(',').map { it.toInt() }
                    return SensorData(sensorData)
                }
            }
        }
    }

    sealed class Rpi : InputMessage() {
        class Arrow(val x: Int, val y: Int, val face: Direction) : Rpi() {
            companion object : Parsable<Arrow> {
                override fun isParsableFrom(line: String) = line.startsWith("arrfound")
                override fun fromLine(line: String): Arrow {
                    val (x, y, directionCommand) = line.substring(8).split(",")
                    val direction = when (directionCommand) {
                        "u" -> Direction.UP
                        "d" -> Direction.DOWN
                        "l" -> Direction.LEFT
                        "r" -> Direction.RIGHT
                        else -> throw IllegalStateException("Unknown arrow found direction: $directionCommand")
                    }
                    return Arrow(x.toInt(), y.toInt(), direction)
                }
            }
        }
    }
}

sealed class OutputMessage {
    abstract val prefix: String
    abstract override fun toString(): String

    sealed class Android : OutputMessage() {
        override val prefix get() = "an"

        data class Arrow(val x: Int, val y: Int, val direction: Direction) : Android() {
            override fun toString(): String {
                val directionCommand = when (direction) {
                    Direction.UP -> "u"
                    Direction.DOWN -> "d"
                    Direction.LEFT -> "l"
                    Direction.RIGHT -> "r"
                }
                return "arrow{$x,$y,$directionCommand}"
            }
        }

        data class MdfString(val part1: String, val part2: String) : Android() {
            override fun toString() = "mdf{$part1,$part2}"
        }

        data class RobotCenter(val cellInfo: CellInfoModel) : Android() {
            override fun toString(): String {
                val directionCommand = when (cellInfo.direction) {
                    Direction.UP -> "u"
                    Direction.DOWN -> "d"
                    Direction.LEFT -> "l"
                    Direction.RIGHT -> "r"
                }
                return "center{${cellInfo.col},${cellInfo.row},$directionCommand}"
            }
        }

        object Stop : Android() {
            override fun toString() = "stop"
        }
    }

    sealed class Arduino : OutputMessage() {
        override val prefix get() = "ar"

        data class MoveForward(val grids: Int) : Arduino() {
            override fun toString() = "w;${grids * 10}"
        }

        data class TurnLeft(val turns: Int) : Arduino() {
            override fun toString() = "a;${turns * 90}"
        }

        data class TurnRight(val turns: Int) : Arduino() {
            override fun toString() = "d;${turns * 90}"
        }

        object GetSensorData : Arduino() {
            override fun toString() = "g"
        }

        object Calibrate : Arduino() {
            override fun toString() = "c"
        }

        object StartFastestPath : Arduino() {
            override fun toString() = "x"
        }
    }

    sealed class Rpi : OutputMessage() {
        override val prefix get() = "rpi"

        data class RobotCenter(val cellInfo: CellInfoModel) : Rpi() {
            override fun toString(): String {
                val directionCommand = when (cellInfo.direction) {
                    Direction.UP -> "u"
                    Direction.DOWN -> "d"
                    Direction.LEFT -> "l"
                    Direction.RIGHT -> "r"
                }
                return "${cellInfo.col},${cellInfo.row},$directionCommand"
            }
        }
    }
}

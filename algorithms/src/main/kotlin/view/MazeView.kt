package view

import controller.MainController
import domain.*
import javafx.beans.binding.Bindings
import javafx.beans.binding.ObjectBinding
import javafx.scene.paint.Color
import tornadofx.*
import java.util.concurrent.Callable

class MazeView : View() {
    val controller: MainController by inject()
    override val root = gridpane {
        for (row in MAZE_ROWS - 1 downTo 0) {
            row {
                for (col in 0 until MAZE_COLUMNS) {
                    rectangle(width = 20.0, height = 20.0) {
                        setOnMouseClicked {
                            if (controller.displayRealMaze.value != true) {
                                return@setOnMouseClicked
                            }
                            val value = controller.realMaze[row, col]
                            if (value == CELL_SENSED) {
                                controller.realMaze[row, col] = CELL_OBSTACLE
                            } else {
                                controller.realMaze[row, col] = CELL_SENSED
                            }
                        }
                        fillProperty().bind(createRealMapBinding(row, col))
                        controller.displayRealMaze.onChange { value ->
                            fillProperty().unbind()
                            if (value) {
                                fillProperty().bind(createRealMapBinding(row, col))
                            } else {
                                fillProperty().bind(createExplorationMapBinding(row, col))
                            }
                        }
                    }
                }
            }
        }
        isGridLinesVisible = true
    }

    private fun createRealMapBinding(row: Int, col: Int): ObjectBinding<Color> = Bindings.createObjectBinding(
        Callable {
            val cell = controller.realMaze[row, col]
            val wayPointX = controller.configurationModel.wayPointX
            val wayPointY = controller.configurationModel.wayPointY
            when {
                cell == CELL_OBSTACLE -> Color.BLACK
                cell == CELL_UNKNOWN -> Color.GRAY
                (row in 0..2 && col in 0..2) || (row in MAZE_ROWS - 3 until MAZE_ROWS && col in MAZE_COLUMNS - 3 until MAZE_COLUMNS) -> Color.YELLOW
                row == wayPointY && col == wayPointX -> Color.BLUE
                else -> Color.WHITE
            }
        },
        controller.realMaze.mazeProperties[row][col],
        controller.configurationModel.wayPointXProperty,
        controller.configurationModel.wayPointYProperty
    )

    private fun createExplorationMapBinding(row: Int, col: Int): ObjectBinding<Color> = Bindings.createObjectBinding(
        Callable {
            val (centerRow, centerCol, direction) = controller.centerCell
            val cell = controller.explorationMaze[row, col]
            val wayPointX = controller.configurationModel.wayPointX
            val wayPointY = controller.configurationModel.wayPointY
            when {
                cell == CELL_UNKNOWN -> Color.GRAY
                cell == CELL_OBSTACLE -> Color.BLACK
                row in centerRow - 1..centerRow + 1 && col in centerCol - 1..centerCol + 1 -> when {
                    direction == Direction.UP && row == centerRow + 1 && col == centerCol -> Color.RED
                    direction == Direction.DOWN && row == centerRow - 1 && col == centerCol -> Color.RED
                    direction == Direction.LEFT && row == centerRow && col == centerCol - 1 -> Color.RED
                    direction == Direction.RIGHT && row == centerRow && col == centerCol + 1 -> Color.RED
                    else -> Color.GREEN
                }
                (row in 0..2 && col in 0..2) || (row in MAZE_ROWS - 3 until MAZE_ROWS && col in MAZE_COLUMNS - 3 until MAZE_COLUMNS) -> Color.YELLOW
                row == wayPointY && col == wayPointX -> Color.BLUE
                else -> Color.WHITE
            }
        },
        controller.explorationMaze.mazeProperties[row][col],
        controller.centerCell.rowProperty,
        controller.centerCell.colProperty,
        controller.centerCell.directionProperty,
        controller.configurationModel.wayPointXProperty,
        controller.configurationModel.wayPointYProperty
    )
}
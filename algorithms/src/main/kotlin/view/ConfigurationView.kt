package view

import controller.MainController
import javafx.geometry.Pos
import javafx.geometry.VPos
import javafx.util.converter.DoubleStringConverter
import javafx.util.converter.IntegerStringConverter
import javafx.util.converter.LongStringConverter
import tornadofx.*

class ConfigurationView : View() {
    val controller: MainController by inject()
    override val root = vbox {
        spacing = 8.0
        label("Exploration")
        hbox {
            alignment = Pos.CENTER_LEFT
            spacing = 8.0
            val speedCheckBox = checkbox("Speed")
            textfield {
                textProperty().bindBidirectional(controller.configurationModel.speedProperty, IntegerStringConverter())
                enableWhen(speedCheckBox.selectedProperty())
                prefColumnCount = 3
            }
            label("steps/second")
            val timeCheckBox = checkbox("Time") {
                enableWhen(speedCheckBox.selectedProperty())
            }
            textfield {
                textProperty().bindBidirectional(controller.configurationModel.timeProperty, LongStringConverter())
                enableWhen(timeCheckBox.selectedProperty())
                prefColumnCount = 3
            }
            label("seconds")
            val coverageCheckBox = checkbox("Coverage") {
                enableWhen(speedCheckBox.selectedProperty())
            }
            textfield {
                textProperty().bindBidirectional(
                    controller.configurationModel.coverageProperty,
                    DoubleStringConverter()
                )
                enableWhen(coverageCheckBox.selectedProperty())
                prefColumnCount = 5
            }
            label("%")
        }
        label("Fastest Path")
        hbox {
            alignment = Pos.CENTER_LEFT
            spacing = 8.0
            val wayPointCheckBox = checkbox("Way Point")
            label("X =")
            textfield {
                textProperty().bindBidirectional(
                    controller.configurationModel.wayPointXProperty,
                    IntegerStringConverter()
                )
                enableWhen(wayPointCheckBox.selectedProperty())
                prefColumnCount = 2
            }
            label("Y =")
            textfield {
                textProperty().bindBidirectional(
                    controller.configurationModel.wayPointYProperty,
                    IntegerStringConverter()
                )
                enableWhen(wayPointCheckBox.selectedProperty())
                prefColumnCount = 2
            }
        }
        label("Map Descriptor")
        gridpane {
            useMaxWidth = true
            hgap = 8.0
            vgap = 8.0
            row {
                label("Part 1") {
                    gridpaneConstraints { vAlignment = VPos.TOP }
                }
                textarea {
                    bind(controller.configurationModel.mapDescriptorPart1Property)
                    prefRowCount = 2
                    prefColumnCount = 30
                }
            }
            row {
                label("Part 2") {
                    gridpaneConstraints { vAlignment = VPos.TOP }
                }
                val part2TextArea = textarea {
                    bind(controller.configurationModel.mapDescriptorPart2Property)
                    prefRowCount = 2
                    prefColumnCount = 30
                }
                button("Load") {
                    enableWhen(part2TextArea.textProperty().isNotEmpty)
                    gridpaneConstraints { vAlignment = VPos.BOTTOM }
                    action {
                        println("Part 1: ${controller.configurationModel.mapDescriptorPart1}")
                        println("Part 2: ${controller.configurationModel.mapDescriptorPart2}")
                    }
                }
            }
        }
        hbox {
            alignment = Pos.CENTER_LEFT
            spacing = 8.0
            button("Exploration") {
                action { controller.runExploration() }
            }
            button("Fastest Path") {
                action { controller.runFastestPath() }
            }
        }
    }
}
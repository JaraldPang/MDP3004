package view

import controller.MainController
import javafx.event.EventHandler
import javafx.geometry.Pos
import javafx.scene.Node
import javafx.scene.input.MouseEvent
import javafx.util.converter.DoubleStringConverter
import javafx.util.converter.IntegerStringConverter
import javafx.util.converter.LongStringConverter
import kotlinx.coroutines.experimental.Dispatchers
import kotlinx.coroutines.experimental.GlobalScope
import kotlinx.coroutines.experimental.channels.Channel
import kotlinx.coroutines.experimental.channels.actor
import kotlinx.coroutines.experimental.javafx.JavaFx
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
        hbox {
            spacing = 8.0
            label("Part 1")
            textarea(controller.configurationModel.mapDescriptorPart1Property) {
                prefRowCount = 2
                prefColumnCount = 30
                isWrapText = true
            }
        }
        hbox {
            spacing = 8.0
            label("Part 2")
            textarea(controller.configurationModel.mapDescriptorPart2Property) {
                prefRowCount = 2
                prefColumnCount = 30
                isWrapText = true
            }
        }
        hbox {
            alignment = Pos.CENTER_LEFT
            spacing = 8.0
            button("Load") {
                action { controller.loadMapDescriptor() }
            }
            button("Save") {
                action { controller.saveMapDescriptor() }
            }
            label("File: ./mazes/")
            textfield(controller.configurationModel.filenameProperty)
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
            button("Connect") {
                enableWhen(controller.connection.socketProperty.booleanBinding { it == null })
                onClick { controller.connect() }
            }
            button("Reset") {
                action { controller.reset() }
            }
        }
    }
}

fun Node.onClick(action: suspend (MouseEvent) -> Unit) {
    val eventActor = GlobalScope.actor<MouseEvent>(Dispatchers.JavaFx, capacity = Channel.CONFLATED) {
        for (event in channel) {
            action(event)
        }
    }

    onMouseClicked = EventHandler {
        eventActor.offer(it)
    }
}
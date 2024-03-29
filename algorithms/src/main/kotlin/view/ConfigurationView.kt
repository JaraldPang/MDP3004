package view

import controller.ConnectionState
import controller.MainController
import javafx.geometry.Pos
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
            button {
                textProperty().bind(controller.connectionStateProperty.property.stringBinding {
                    when (it) {
                        null -> ""
                        ConnectionState.DISCONNECTED -> "Connect"
                        ConnectionState.CONNECTING -> "Connecting"
                        ConnectionState.CONNECTED -> "Disconnect"
                        ConnectionState.DISCONNECTING -> "Disconnecting"
                    }
                })
                enableWhen(controller.connectionStateProperty.property.booleanBinding {
                    !(it == ConnectionState.CONNECTING || it == ConnectionState.DISCONNECTING)
                })
                action {
                    when (controller.connectionStateProperty.property.value) {
                        ConnectionState.DISCONNECTED -> controller.connect()
                        ConnectionState.CONNECTED -> controller.disconnect()
                        else -> {
                        }
                    }
                }
            }
            button("Reset") {
                action { controller.reset() }
            }
        }
    }
}

//fun Node.onClick(action: suspend (MouseEvent) -> Unit) {
//    val eventActor = Channel<MouseEvent>(Channel.CONFLATED)
//    GlobalScope.launch(Dispatchers.Main) {
//        for (event in eventActor.debounce(300)) {
//            action(event)
//        }
//    }
//
//    onMouseClicked = EventHandler { eventActor.offer(it) }
//}
//
//@UseExperimental(ExperimentalCoroutinesApi::class)
//fun <T> ReceiveChannel<T>.debounce(timeMillis: Long): ReceiveChannel<T> {
//    val producer = Channel<T>(Channel.UNLIMITED)
//    GlobalScope.launch(Dispatchers.Main) {
//        var value = receive()
//        while (true) {
//            selectUnbiased<Unit> {
//                onTimeout(timeMillis) {
//                    producer.send(value)
//                    value = receive()
//                }
//                onReceive {
//                    value = it
//                }
//            }
//        }
//    }
//    return producer
//}
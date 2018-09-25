package model

import javafx.beans.property.SimpleObjectProperty
import javafx.beans.property.SimpleStringProperty
import tornadofx.ViewModel
import tornadofx.getValue
import tornadofx.setValue

class ConfigurationModel : ViewModel() {
    val speedProperty = SimpleObjectProperty<Int?>(null)
    var speed by speedProperty

    val timeProperty = SimpleObjectProperty<Long?>(null)
    var time by timeProperty

    val coverageProperty = SimpleObjectProperty<Double?>(null)
    var coverage by coverageProperty

    val wayPointXProperty = SimpleObjectProperty<Int?>(null)
    var wayPointX by wayPointXProperty

    val wayPointYProperty = SimpleObjectProperty<Int?>(null)
    var wayPointY by wayPointYProperty

    val mapDescriptorPart1Property = SimpleStringProperty(null)
    var mapDescriptorPart1: String? by mapDescriptorPart1Property

    val mapDescriptorPart2Property = SimpleStringProperty(null)
    var mapDescriptorPart2: String? by mapDescriptorPart2Property

    override fun toString(): String {
        return "ConfigurationModel(speed=$speed, time=$time, coverage=$coverage, wayPointX=$wayPointX, wayPointY=$wayPointY, mapDescriptorPart1=$mapDescriptorPart1, mapDescriptorPart2=$mapDescriptorPart2)"
    }
}
package view

import tornadofx.View
import tornadofx.borderpane
import tornadofx.paddingAll

class MainView : View("MDP Group 17") {
    override val root = borderpane {
        paddingAll = 8.0
        left(MazeView::class)
        right(ConfigurationView::class)
    }
}
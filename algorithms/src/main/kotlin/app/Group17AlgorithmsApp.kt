package app

import javafx.stage.Stage
import tornadofx.*
import view.MainView

class Group17AlgorithmsApp : App(MainView::class) {
    override fun start(stage: Stage) {
        stage.isResizable = false
        super.start(stage)
    }
}

fun main() {
    launch<Group17AlgorithmsApp>()
}
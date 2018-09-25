package app

import javafx.stage.Stage
import tornadofx.App
import tornadofx.launch
import view.MainView

class Group17AlgorithmsApp : App(MainView::class) {
    override fun start(stage: Stage) {
        stage.isResizable = false
        super.start(stage)
    }
}

fun main(args: Array<String>) {
    launch<Group17AlgorithmsApp>(args)
}
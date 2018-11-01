package domain

import com.fasterxml.jackson.module.kotlin.jacksonObjectMapper
import com.fasterxml.jackson.module.kotlin.readValue
import java.io.File
import java.io.FileOutputStream

val mapper = jacksonObjectMapper()

interface JsonSerializable {
    val filename: String
}

fun saveJson(jsonObject: JsonSerializable) {
    mapper.writeValue(FileOutputStream(".${jsonObject.filename}.json"), jsonObject)
}

inline fun <reified T : Any> loadJson(typename: String): T? {
    val file = File(".$typename.json")
    return if (file.exists()) {
        mapper.readValue(file)
    } else {
        null
    }
}
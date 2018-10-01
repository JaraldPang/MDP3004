void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
}

String readed;
void loop() {
  while (Serial.available()){
    //messages(commands) can come in a chunk if accumulated.
    //newest message is on right side.
    //eg of a string with | as delimiters. ALGOmoveleft|ALGOmoveforward|ANDROIDupdate|
    readed = Serial.readString();
    
    Serial.println("abc"); //Write what you want to send here
  }
}

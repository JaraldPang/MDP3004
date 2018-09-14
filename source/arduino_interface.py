import serial
import os.path

class ArduinoInterface:

    #minimize method calls to reduce stack overhead for function initialization

    #constructor method which initializes instance variables
    def __init__(self):
        arduino_path = "";

        #check which device node is the arduino on
        if(os.path.exist("/dev/ttyACM0")):
            arduino_path = "/dev/ttyACM0"
        else
            arduino_path = "/dev/ttyACM1"

        self.serial = serial.Serial(arduino_path, 115200)
        pass

    #writes to the serial interface
    def write(msg):
        pass

    #reads a newline terminated string from the serial interface
    #serial.readline() is blocking, meaning that the execution will not continue until it receives a newline
    def read(self):
        try:
            return self.serial.readline()
        except(Exception):
            return False

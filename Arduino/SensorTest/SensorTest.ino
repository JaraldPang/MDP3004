

/**
 * Sensor Variables Declaration
 */
#define pinSensor0 0
#define pinSensor1 1
#define pinSensor2 2
#define pinSensor3 3
#define pinSensor4 4
#define pinSensor5 5

/**
 * Sharp IR Model :
 * GP2Y0A02YK0F --> "20150"
 * GP2Y0A21YK --> "1080"
 * GP2Y0A710K0F --> "100500"
 * GP2YA41SK0F --> "430"
 */

#define MODEL_SHORT 1080
#define MODEL_LONG 20150

//Sampling Elements of 100
//RunningMedian sample0 = RunningMedian(100);
//RunningMedian sample1 = RunningMedian(100);
//RunningMedian sample2 = RunningMedian(100);
//RunningMedian sample3 = RunningMedian(100);
//RunningMedian sample4 = RunningMedian(100);
//RunningMedian sample5 = RunningMedian(100);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

}

void loop() {
  // read the input on analog pin 0:
  float sensorValue = analogRead(A0);
  // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 5V):
  // https://www.arduino.cc/en/Tutorial/ReadAnalogVoltage
  float voltage = sensorValue * (5.0 / 1023.0);
  
  int voltFromRaw = map(voltage, 0, 1023, 0, 5000);
  
  float distances = 27.728 * pow(voltFromRaw, -1.2045);
  
  Serial.print(sensorValue);
  Serial.print("\t");
  Serial.print(voltage);
  Serial.print("\t");
  Serial.print(distances);
  Serial.println();

  delay(300);
}

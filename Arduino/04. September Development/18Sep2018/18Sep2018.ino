#include <DualVNH5019MotorShield.h>
#include <EnableInterrupt.h>
#include <RunningMedian.h>

/*
     ********************************************************************************************************************************
*/
/**
 * Everything About Sensor
 *
 * MDP Board Pin <> Arduino Pin <> Sensor Range <> Model <> Location
 * Top Sensor
 * PS1 <> A0  <> SE5  Distance <= 85          (1080)    TLeft
 * PS3 <> A2  <> SE7  30 <= Distance <= 130   (20150)   TMiddle
 * PS5 <> A4  <> SE2  Distance <= 85          (1080)    TRight
 *
 * Bottom Sensor
 * PS2 <> A1  <> SE3 Distance <=80            (1080)    BRight(Front)
 * PS4 <> A3  <> SE1 Distance <=70            (1080)    BRight(Back)
 * PS6 <> A5  <> SE6 Distance <=85            (1080)    BLeft(Front)
 *
 *
 * Sensor Variables Declaration
 */

#define pinSensor0 0
#define pinSensor2 2
#define pinSensor4 4

#define pinSensor1 1
#define pinSensor3 3
#define pinSensor5 5


#define MODEL_SHORT 1080 // 10-80 cm (Short)
#define MODEL_LONG 20150 // 20-150 cm (Long)

#define SampleSize 50

#define Eq0x 0.041755
#define Eq0off 0.00375

#define Eq2x 0.420821
#define Eq2off 0.012706

#define Eq4x 0.043097
#define Eq4off 0.00477

#define Eq1x 0.045623
#define Eq1off 0.00493

#define Eq3x 0.046427
#define Eq3off 0.00891

#define Eq5x 0.041436
#define Eq5off 0.00574

long ps1, ps2, ps4;
double sensorVal1, sensorVal2, sensorVal3;
float voltage1, voltage2, voltage3, dis1, dis2, dis4;

RunningMedian sample0 = RunningMedian(SampleSize);
RunningMedian sample1 = RunningMedian(SampleSize);
RunningMedian sample2 = RunningMedian(SampleSize);
RunningMedian sample3 = RunningMedian(SampleSize);
RunningMedian sample4 = RunningMedian(SampleSize);
RunningMedian sample5 = RunningMedian(SampleSize);

/*
     ********************************************************************************************************************************
*/

/**
 * Everything About Motor
 *
 * Motor Variables for Calibration
 *
 *
 * M1 = E1 = Right Side (Weak)
 * M2 = E2 = Left Side (Strong)
 * md.setSpeeds(R,L) / (E1,E2)
 */

#define kpValue -20 //adjustment later > 50 wheel no spin
#define kiValue 0 //error rate > 0.5 turn circle, cannot go higher than 0.1
#define kdValue -10 //post adjustment

// Moving speed.
#define SPEED_MOVE 380 //305//355//305

// Turning speed
#define SPEED_SPIN 385 //295//345//295

#define SPEED_CALIBRATE 100

//E2 Left Side
#define M2A 11
#define M2B 13

//E1 Right Side
#define M1A 3
#define M1B 5

DualVNH5019MotorShield md;

int motorStatus;
double integral;
long prevTick, prevMillis = 0;
volatile long encoderLeftCounter, encoderRightCounter;

String robotRead;
bool newData = false, isStarted = false;
bool robotReady = false;

/**
 * Interrupt pins functions
 */
//E1
void showEncode1() {
  encoderLeftCounter++;
  enableInterrupt(M1A, showEncode1, FALLING);
}
//E2
void showEncode2() {
  encoderRightCounter++;
  enableInterrupt(M2A, showEncode2, FALLING);
}

/**
 * This function is to get encoder values
 */
double computePID() {
  //Serial.println(String(encoderLeftCounter) + ", " + String(encoderRightCounter) + ", " + String(encoderLeftCounter - encoderRightCounter));
  double kp, ki, kd, p, i, d, error, pid;

  kp = kpValue;
  ki = kiValue;
  kd = kdValue;

  error = encoderLeftCounter - encoderRightCounter;
  integral += error;

  p = kp * error;
  i = ki * integral;
  d = kd * (prevTick - encoderLeftCounter);
  pid = p + i + d;

  prevTick = encoderLeftCounter;

  return pid;
}

void testMotors() {
  if (true) {
    //Serial.print(millis()-prevMillis);
    //Serial.print('\t');
    //Serial.println(motorStatus);
    if (millis() - prevMillis > 3000) {
      switch (motorStatus) {

      case 0:
      {
        //md.setSpeeds(R,L)
        md.setSpeeds(50, 50);
        break;
      }
      case 1:
      {
        md.setSpeeds(100, 100);
        break;
      }
      case 2:
      {
        md.setSpeeds(150, 150);
        break;
      }
      case 3:
      {
        md.setSpeeds(200, 200);
        break;
      }
      case 4:
      {
        md.setSpeeds(250, 250);
        break;
      }
      case 5:
      {
        md.setSpeeds(300, 300);
        break;
      }
      case 6:
      {
        md.setSpeeds(350, 350);
        break;
      }
      case 7:
      {
        md.setSpeeds(400, 400);
        break;
      }
      }
      motorStatus = (motorStatus + 1) % 8;
      prevMillis = millis();
    }
  }
}

void moveForward(double cm) {
  //Serial.print("Moving Forward by : ");
  //Serial.println(value);
  //delay(3000);
  double pid;
  int targetTick;
  integral = 0;
  encoderLeftCounter = encoderRightCounter = prevTick = 0;

  targetTick = cm * 29.65; //29.2;//29.3;
  //29.35;
  //29.38;//29.4;//29;//29.5;//29.85;//30.05;//30.15;//30.20;//30.35; // Caliberated to 30.25 ticks per cm

  // Move Forward 1 grid
  if (cm <= 10) {
    targetTick = cm * 26.85;
    //27;
    //27.5M;//28.15M;//28.65;
    while (encoderLeftCounter < min(50, targetTick)) {
      pid = computePID();
      md.setSpeeds(
        ((0.6 * SPEED_MOVE) + pid),
        ((0.6 * SPEED_MOVE) - pid)
      );
    }
    while (encoderLeftCounter < targetTick - 50) {
      pid = computePID();
      md.setSpeeds(
        ((1.0 * SPEED_MOVE) + pid),
        ((1.0 * SPEED_MOVE) - pid)
      );
    }
    while (encoderLeftCounter < targetTick - 25) {
      pid = computePID();
      md.setSpeeds(
        ((0.8 * SPEED_MOVE) + pid),
        ((0.8 * SPEED_MOVE) - pid)
      );
    }
    while (encoderLeftCounter < targetTick - 15) {
      pid = computePID();
      md.setSpeeds(
        ((0.6 * SPEED_MOVE) + pid),
        ((0.6 * SPEED_MOVE) - pid)
      );
    }
    while (encoderLeftCounter < targetTick) {
      pid = computePID();
      md.setSpeeds(
        ((0.5 * SPEED_MOVE) + pid),
        ((0.5 * SPEED_MOVE) - pid)
      );
    }
  }
  // Move Forward 2 grids
  else if (cm <= 30) {
    targetTick = cm * 29;
    //28.25;//28.5;
    //29.2
    while (encoderLeftCounter < targetTick) {
      pid = computePID();
      md.setSpeeds(
        ((0.86 * SPEED_MOVE) + pid),
        ((0.86 * SPEED_MOVE) - pid)
      );
    }
    //turnRight_sil(1);
  }
  // Move Forward 5 grids
  else if (cm <= 50) {
    targetTick = cm * 29.5; //28.75;//29M;//28.5; //29.2
    while (encoderLeftCounter < targetTick - 50) {
      pid = computePID();
      //0.885
      md.setSpeeds(
        ((1.0 * SPEED_MOVE) + pid),
        ((1.0 * SPEED_MOVE) - pid)
      );
    }

    while (encoderLeftCounter < targetTick - 25) {
      pid = computePID();
      md.setSpeeds(
        ((0.8 * SPEED_MOVE) + pid),
        ((0.8 * SPEED_MOVE) - pid)
      );
    }
    while (encoderLeftCounter < targetTick - 15) {
      pid = computePID();
      md.setSpeeds(
        ((0.6 * SPEED_MOVE) + pid),
        ((0.6 * SPEED_MOVE) - pid)
      );
    }
    while (encoderLeftCounter < targetTick) {
      pid = computePID();
      md.setSpeeds(
        ((0.5 * SPEED_MOVE) + pid),
        ((0.5 * SPEED_MOVE) - pid)
      );
    }
    //to bypass the curve motion movement
    //turnRight_sil(2);
  }
  // Move Forward 6 grids
  else if (cm <= 60) {
    //targetTick = cm * 29;//28.5; //29.2
    while (encoderLeftCounter < targetTick - 50) {
      pid = computePID();
      md.setSpeeds(
        ((1.0 * SPEED_MOVE) + pid),
        ((1.0 * SPEED_MOVE) - pid)
      );
    }

    while (encoderLeftCounter < targetTick - 25) {
      pid = computePID();
      md.setSpeeds(
        ((0.6 * SPEED_MOVE) + pid),
        ((0.6 * SPEED_MOVE) - pid)
      );
    }
    while (encoderLeftCounter < targetTick - 15) {
      pid = computePID();
      md.setSpeeds(
        ((0.5 * SPEED_MOVE) + pid),
        ((0.5 * SPEED_MOVE) - pid)
      );
    }
    while (encoderLeftCounter < targetTick) {
      pid = computePID();
      md.setSpeeds(
        ((0.4 * SPEED_MOVE) + pid),
        ((0.4 * SPEED_MOVE) - pid)
      );
    }
    //to bypass the curve motion movement
    //turnRight_sil(2);
  }
  // Just Move Forward
  else {
    while (encoderLeftCounter < targetTick - 50) {
      pid = computePID();
      //md.setSpeeds(-((SPEED_MOVE ) - pid), SPEED_MOVE + pid);
      //md.setSpeeds(-((SPEED_MOVE * 0.86) - pid), SPEED_MOVE + pid);
      //md.setSpeeds(-((SPEED_MOVE * 0.845) - pid), SPEED_MOVE + pid);
      md.setSpeeds(
        ((0.7 * SPEED_MOVE) + pid),
        ((0.7 * SPEED_MOVE) - pid)
      );

    }
    while (encoderLeftCounter < targetTick - 20) {
      pid = computePID();
      md.setSpeeds(
        ((0.8 * SPEED_MOVE) + pid),
        ((0.8 * SPEED_MOVE) - pid)
      );

    }
    while (encoderLeftCounter < targetTick - 15) {
      pid = computePID();
      md.setSpeeds(
        ((0.6 * SPEED_MOVE) + pid),
        ((0.6 * SPEED_MOVE) - pid)
      );

    }
    while (encoderLeftCounter < targetTick) {
      pid = computePID();
      md.setSpeeds(
        ((0.4 * SPEED_MOVE) + pid),
        ((0.4 * SPEED_MOVE) - pid)
      );

    }
    //to bypass the curve motion movement
    //rotate back by 5% of the distance
    //100 * 5% = 5
    if (cm <= 80) {
      //turnRight_sil(2.7);
    } else {
      //turnRight_sil(3.5);
    }
    //turnRight_sil(3.5);
  }
  //Serial.println(encoderLeftCounter);
  md.setM2Brake(400);
  delayMicroseconds(500);
  md.setM1Brake(400);
  
  //Serial.print("OK\r\n");
}

void moveBack(int cm) {
  double pid;
  int targetTick;
  integral = 0;
  encoderLeftCounter = encoderRightCounter = prevTick = 0;

  targetTick = cm * 30.20; //30.35; // Caliberated to 30.25 ticks per cm

  while (encoderLeftCounter < min(50, targetTick)) {
    pid = computePID();
    md.setSpeeds(
      -((0.5 * SPEED_MOVE) + pid),
      -((0.5 * SPEED_MOVE) - pid)
    );
  }

  while (encoderLeftCounter < targetTick - 50) {
    pid = computePID();
    md.setSpeeds(
      -((1.0 * SPEED_MOVE) + pid),
      -((1.0 * SPEED_MOVE) - pid)
    );
  }

  while (encoderLeftCounter < targetTick) {
    pid = computePID();
    md.setSpeeds(
      -((0.5 * SPEED_MOVE) + pid),
      -((0.5 * SPEED_MOVE) - pid)
    );
  }

  md.setM2Brake(400);
  delayMicroseconds(500);
  md.setM1Brake(400);
  
  delay(100);
  Serial.print("OK\r\n");
}

void turnLeft(double deg){
  double pid;
  float targetTick;
  integral = 0;
  encoderLeftCounter = encoderRightCounter = prevTick = 0;

/*
  if (deg <= 90) targetTick = deg * 4.39; //4.523
  else if (deg <= 180 ) targetTick = deg * 4.62;
  else if (deg <= 360 ) targetTick = deg * 4.675;
  else targetTick = deg * 4.65;
  */
  if (deg <= 90) targetTick = deg * 4.17;//4.17(test)//4.095(on maze)//4.0935;//4.0925;//4.09L;//4.085L;//4.08L;//4.0775L;
  //4.076L;//4.078M;//4.075L;//4.08M;//4.07L;//4.09;
  //4.102;//4.11;//4.121;M//4.122M;//4.1224M;
  //4.1225M;//4.1145L;//4.11L;//4.1L;//4.115M;
  //4.12;//4.125M;//4.15M;//4.195M;//4.2;//4.205;//4.21;//4.258;
  else if (deg <= 180 ) targetTick = deg * 4.322;//4.322(test)//4.62;
  else if (deg <= 360 ) targetTick = deg * 4.41;
  else targetTick = deg * 4.45;

  while ( encoderLeftCounter < min(50, targetTick)) {
    pid = computePID();
    md.setSpeeds(
      ((0.5 * SPEED_SPIN) + pid), 
      -((0.5 * SPEED_SPIN) - pid)
      );
  }
  while ( encoderLeftCounter < targetTick - 50) {
    pid = computePID();
    md.setSpeeds(
      ((1.0 * SPEED_SPIN) + pid), 
      -((1.0 * SPEED_SPIN) - pid)
      );
  }
  while ( encoderLeftCounter < targetTick) {
    pid = computePID();
    md.setSpeeds(
      ((0.5 * SPEED_SPIN) + pid), 
      -((0.5 * SPEED_SPIN) - pid));
  }
  
  md.setM2Brake(400);//400(test)//380(on maze)
  delayMicroseconds(500);
  md.setM1Brake(400);//400(test)//380(on maze)
  
  delay(100);

  Serial.print("OK\r\n");
}

void turnRight(double deg){
  double pid;
  float targetTick;
  integral = 0;
  encoderLeftCounter = encoderRightCounter = prevTick = 0;

  if (deg <= 90) targetTick = deg * 4.155;//4.155(on maze)//4.175M;//4.186M;//4.19M;//4.185;//4.175L;
  //4.148L;//4.15M;//4.170M;//4.175M;//4.21M;//4.205;//4.185;//4.175; 
  //4.2;//4.185;//4.175L;//4.17L;
  //4.165;//4.1545L;//4.154L;//4.153L;
  //4.155M;//4.165M;//4.1655M;//4.166M;//4.167M;
  //4.168;//4.1695;//4.171M;//4.168L;//4.165L;//4.15L;//4.18M;//4.19M;//4.192M;
  //4.187L;//4.185;//4.1825;//4.175L;//4.17225L;//4.1715L;//4.17L;//4.165L;//4.1725M;
  //4.17L;//4.185M;//4.19M;//4.2;//4.22M;z//4.24M;//4.25;//4.335; //24/10/17
  
  else if (deg <= 180 ) targetTick = deg * 4.33;//4.33(test)//4.333M;//4.335M;//4.336M;//4.338M;//4.342M;//4.335;
  //4.32L;//4.35M;
  //4.34;//4.33;
  //4.34;//4.35M;//4.36;//4.415;//4.63; 
  else if (deg <= 360 ) targetTick = deg * 4.4;//4.4(test)
  else targetTick = deg * 4.43;//4.3(test)
  
  while ( encoderLeftCounter < min(50, targetTick)) {
    pid = computePID();
    md.setSpeeds(
      -((0.5 * SPEED_SPIN) + pid), 
      ((0.5 * SPEED_SPIN) - pid)
      );
  }
  
  while ( encoderLeftCounter < targetTick - 50) {
    pid = computePID();
    md.setSpeeds(
      -((1.0 * SPEED_SPIN) + pid), 
      ((1.0 * SPEED_SPIN) - pid)
      );
  }
  while ( encoderLeftCounter < targetTick) {
    pid = computePID();
    md.setSpeeds(
      -((0.5 * SPEED_SPIN) + pid), 
      ((0.5 * SPEED_SPIN) - pid)
      );
  }
  
  md.setM2Brake(400);//400(test, on maze)
  delayMicroseconds(500);
  md.setM1Brake(400);//400(test, on maze)
  
  delay(100);

  Serial.print("OK\r\n");
}

void obstacleAvoid(){
  int count = 0;
  bool avoided = false;
  //delay(3000);
  while(count!=10){
    for(int i=0;i<100;i++){
      long ps1 = analogRead(A0);
      //long ps2 = analogRead(A1);
      long ps3 = analogRead(A2);
      //long ps4 = analogRead(A3);
      //long ps5 = analogRead(A4);
      //long ps6 = analogRead(A5);
      sample0.add(ps1);
      //sample1.add(ps2);
      sample2.add(ps3);
      //sample3.add(ps4); 
      //sample4.add(ps5);
      //sample5.add(ps6);
    }
    float sensorValue0 = sample0.getMedian();
    //float sensorValue1 = sample1.getMedian();
    float sensorValue2 = sample2.getMedian();
    //float sensorValue3 = sample3.getMedian();
    //float sensorValue4 = sample4.getMedian();
    //float sensorValue5 = sample5.getMedian();
        
    float voltage0 = sensorValue0 * (5.0 / 1023.0);
    //float voltage1 = sensorValue1 * (5.0 / 1023.0);
    float voltage2 = sensorValue2 * (5.0 / 1023.0);
    //float voltage3 = sensorValue3 * (5.0 / 1023.0);
    //float voltage4 = sensorValue4 * (5.0 / 1023.0);
    //float voltage5 = sensorValue5 * (5.0 / 1023.0);
        
    float dis0=(1/(0.0444*voltage0 - 0.0061)) - 0.42;
    //float dis1=(1/(0.0444*voltage1 - 0.0061)) - 0.42;
    float dis2=(1/(0.0417*voltage2 - 0.004)) - 0.42;
    //float dis3=(1/(0.0421*voltage3 - 0.0057)) - 0.42;
    //float dis4=(1/(0.0428*voltage4 - 0.0048)) - 0.42;
    //float dis5=(1/(0.044*voltage5 - 0.009)) - 0.42;

    //front Left sensor has obstacle
    if(avoided==false){
      if(dis1<=15 && dis1 > 0){
        turnLeft(180);
        //turnRight(45);
        /*
        moveForward(20);
        turnLeft(45);
        moveForward(20);
        turnLeft(45);
        moveForward(20);
        turnRight(45);
        */
        //moveForward(30);
        //turnLeft(90);
        //moveForward(30);
        //turnRight(45);
        avoided=true;
      } else if(dis2<=15 && dis2 > 0){
        //front right sensor has obstacle
        //turnLeft(45);
        turnRight(180);
        /*
        moveForward(20);
        turnRight(45);
        moveForward(20);
        turnRight(45);
        moveForward(20);
        turnLeft(45);
        */
        //moveForward(30);
        //turnRight(90);
        //moveForward(30);
        //turnLeft(45);
        avoided=true;
      } else{
        moveForward(10);
        count++;
      }
    }else{
      moveForward(10);
      count++;
    }
    delay(500);
  }
}

/*
     ********************************************************************************************************************************
*/

void sensordata() {

  uint32_t lastTime = 0;

  //avoidBurstRead causes a delay of 20ms
  while (millis() <= lastTime + 20) {
    //wait for sensor's sampling time
  }

  lastTime = millis();

  for (int i = 0; i < SampleSize; i++) {
    sample0.add((long) analogRead(A0));
    sample2.add((long) analogRead(A2));
    sample4.add((long) analogRead(A4));

    sample1.add((long) analogRead(A1));
    sample3.add((long) analogRead(A3));
    sample5.add((long) analogRead(A5));
  }

  //Get Median Value
  float sensorValue0 = sample0.getMedian();
  float sensorValue2 = sample2.getMedian();
  float sensorValue4 = sample4.getMedian();

  float sensorValue1 = sample1.getMedian();
  float sensorValue3 = sample3.getMedian();
  float sensorValue5 = sample5.getMedian();

  //Analog value to voltage
  //0.0048875855327468 = 5/1023
  //Reciprocal Value = 1/(DistanceReflective + 0.42)
  //Distance Reflective = Analog * 5/1023
  float dis0 = 1 / (Eq0x * (sensorValue0 * (0.0048875855327468)) - Eq0off) - 0.42;
  float dis2 = 1 / (Eq2x * (sensorValue2 * (0.0048875855327468)) - Eq2off) - 0.42;
  float dis4 = 1 / (Eq4x * (sensorValue4 * (0.0048875855327468)) - Eq4off) - 0.42;

  float dis1 = 1 / (Eq1x * (sensorValue1 * (0.0048875855327468)) - Eq1off) - 0.42;
  float dis3 = 1 / (Eq3x * (sensorValue3 * (0.0048875855327468)) - Eq3off) - 0.42;
  float dis5 = 1 / (Eq5x * (sensorValue5 * (0.0048875855327468)) - Eq5off) - 0.42;

  Serial.print(String(dis0) + " , " + String(dis2) + " , " + String(dis4 - 2.0) + " , " +
               String(dis1) + " , " + String(dis3 - 1.0) + " , " + String(dis5) + "\r\n");

  sample0.clear();
  sample1.clear();
  sample2.clear();
  sample3.clear();
  sample4.clear();
  sample5.clear();
}

/**
 * This function is the processing of any serial data coming in to arduino into a string
 * https://goo.gl/gfVUR4
 */

void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char) Serial.read();

    if (inChar == '\n') {
      newData = true;
      break;
    }
    // add it to the inputString:
    robotRead += inChar;
  }
}

String getValue(String data, char separator, int index)
{
  int found = 0;
  int strIndex[] = {0, -1};
  int maxIndex = data.length() - 1;

  for (int i = 0; i <= maxIndex && found <= index ; i++) {
    if (data.charAt(i) == separator || i == maxIndex) {
      found ++;
      strIndex[0] = strIndex[1] + 1;
      strIndex[1] = (i == maxIndex) ? i + 1 : i;
    }
  }
  return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}



/*
     ********************************************************************************************************************************
*/

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  pinMode(M1A, INPUT);
  pinMode(M2A, INPUT);
  digitalWrite(M1A, LOW);
  digitalWrite(M2A, LOW);
  enableInterrupt(M1A, showEncode1, FALLING);
  enableInterrupt(M2A, showEncode2, FALLING);
  md.init();
}

void loop() {
  /*
  if (robotRead == "start") {
    if (robotReady == false) {
      //delay(5000);
      //moveForward(150);
      //delay(5000);
      //turnRight(180);
      //moveForward(150);
      //turnRight(180);
      robotReady = true;
    }
  }

  if (newData) {
    //char buffer[20];
    //userRead.toCharArray(buffer,20);
    //char* token = strtok(buffer,",");
    //Serial.print(token);=
    double movementValue = getValue(robotRead, ',', 1).toInt();
    char condition = robotRead.charAt(0);
    if (robotRead == "motor") {
      isStarted = !isStarted;
    }
    switch (condition) {
    case 'W':
    case 'w':
      {
        if (movementValue == 0)
          //moveForward(10);
        else
          //moveForward(movementValue);
          break;
      }

    case 'A':
    case 'a':
      {
        if (movementValue == 0)
          //turnLeft(90);
        else
          //turnLeft(movementValue);
          break;
      }

    case 'D':
    case 'd':
      {
        if (movementValue == 0)
          //turnRight(90);
        else
          //turnRight(movementValue);
          break;
      }

    case 'S':
    case 's':
      {
        if (movementValue == 0)
          //moveBack(10);
        else
          //moveBack(movementValue);
          break;
      }
    case 'Z':
    case 'z':
      {
        //sensorData(userRead.charAt(2));
        break;
      }
    case 'O': case'o':{
        obstacleAvoid();
        break;
      }
    case 'C':
    case 'c':
      {
        //caliberate();
        break;
      }
    case 'A': case'a':{
        addOne();
        break;
      }

    case 'X':
    case 'x':
      {
        //updateSensorData();
        break;
      }
    default:
      {
        //defaultResponse();
        break;
      }
    }

    userRead = "";
    newData = false;
  }
  */
  //obstacleAvoid();
  moveForward(100);
  //moveBack(10);
  //delay(500);
  turnLeft(180);
  //delay(500);
  //moveForward(20);
  //delay(500);
  //turnRight(90);
  delay(1000);

}

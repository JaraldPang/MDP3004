#include <DualVNH5019MotorShield.h>
#include <EnableInterrupt.h>
#include <RunningMedian.h>


/**
 * Everything About Sensor
 *
 * MDP Board Pin <> Arduino Pin <> Sensor Range <> Model
 * Top Sensor
 * PS1 <> A0  <> SE5  Distance <= 85          (1080)    TLeft
 * PS3 <> A2  <> SE7  30 <= Distance <= 130   (20150)   TMiddle
 * PS5 <> A4  <> SE2  Distance <= 85          (1080)    TRight
 *
 * Bottom Sensor
 * PS2 <> A1  <> SE3 Distance <=80            (1080)    BRight(Front)
 * PS4 <> A3  <> SE1 Distance <=70            (1080)    BRight(Back)
 * PS6 <> A5  <> SE6 Distance < ????          (20150)   BLeft(Front)
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

// 10-80 cm (Short)
#define MODEL_SHORT 1080
// 20-150 cm (Long)
#define MODEL_LONG 20150

#define SampleSize 40

#define Eq0x 0.041755
#define Eq0off 0.00375
#define Eq2x 0
#define Eq2off 0
#define Eq4x 0.043097
#define Eq4off 0.00477

#define Eq1x 0.045623
#define Eq1off 0.00493
#define Eq3x 0.046427
#define Eq3off 0.00891
#define Eq5x 0
#define Eq5off 0

long ps1, ps2, ps4;
double sensorVal1, sensorVal2, sensorVal3;
float voltage1, voltage2, voltage3, dis1, dis2, dis4;


RunningMedian sample0 = RunningMedian(SampleSize);
RunningMedian sample1 = RunningMedian(SampleSize);
RunningMedian sample2 = RunningMedian(SampleSize);
RunningMedian sample3 = RunningMedian(SampleSize);
RunningMedian sample4 = RunningMedian(SampleSize);
RunningMedian sample5 = RunningMedian(SampleSize);


/**
 * Everything About Motor
 *
 * Motor Variables for Calibration
 */

// Moving speed.
#define SPEED_MOVE 390//305//355//305

// Turning speed
#define SPEED_SPIN 385//295//345//295

#define SPEED_CALIBRATE 100

DualVNH5019MotorShield md;
int motorStatus;
double integral;
long prevTick, prevMillis = 0;
volatile long encoderLeftCounter, encoderRightCounter;

//E1 Right Side
#define M1A 3
#define M1B 5
// Ping 5 is faulty

//E2 Left Side
#define M2A 11
#define M2B 13




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

  kp = 20;
  ki = 0;
  kd = 0;

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

      case 0 : {
        //md.setSpeeds(R,L)
        md.setSpeeds(50, 50);
        break;
      } case 1 : {
        md.setSpeeds(100, 100);
        break;
      } case 2 : {
        md.setSpeeds(150, 150);
        break;
      } case 3 : {
        md.setSpeeds(200, 200);
        break;
      } case 4 : {
        md.setSpeeds(250, 250);
        break;
      } case 5 : {
        md.setSpeeds(300, 300);
        break;
      } case 6 : {
        md.setSpeeds(350, 350);
        break;
      } case 7 : {
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

  targetTick = cm * 29.5;//29.2;//29.3;
  //29.35;
  //29.38;//29.4;//29;//29.5;//29.85;//30.05;//30.15;//30.20;//30.35; // Caliberated to 30.25 ticks per cm

  if (cm <= 10) {
    targetTick = cm * 26.85;
    //27;
    //27.5M;//28.15M;//28.65;
    while (encoderLeftCounter < min(50, targetTick)) {
      pid = computePID();
      md.setSpeeds(((0.6 * SPEED_MOVE ) - pid), (0.6 * SPEED_MOVE) + pid);
    }
    while (encoderLeftCounter < targetTick  - 50) {
      pid = computePID();
      md.setSpeeds(((SPEED_MOVE ) - pid), SPEED_MOVE + pid);
    }
    while (encoderLeftCounter < targetTick - 25) {
      pid = computePID();
      md.setSpeeds(((0.8 * SPEED_MOVE) - pid), (0.8 * SPEED_MOVE) + pid);
    }
    while (encoderLeftCounter < targetTick - 15) {
      pid = computePID();
      md.setSpeeds(((0.6 * SPEED_MOVE) - pid), (0.6 * SPEED_MOVE) + pid);
    }
    while (encoderLeftCounter < targetTick) {
      pid = computePID();
      md.setSpeeds(((0.5 * SPEED_MOVE) - pid), (0.5 * SPEED_MOVE) + pid);
    }
  }
  else if (cm <= 30) {
    targetTick = cm * 28.5;
    //28.25;//28.5;
    //29.2
    while (encoderLeftCounter < targetTick) {
      pid = computePID();
      md.setSpeeds(((SPEED_MOVE * 0.86) - pid), SPEED_MOVE + pid);
    }
    //turnRight_sil(1);
  }
  else if (cm <= 50) {
    targetTick = cm * 29.052;//28.75;//29M;//28.5; //29.2
    while (encoderLeftCounter < targetTick  - 50) {
      pid = computePID();
      //0.885
      md.setSpeeds(((SPEED_MOVE * 0.875) - pid), SPEED_MOVE + pid);
    }

    while (encoderLeftCounter < targetTick - 25) {
      pid = computePID();
      md.setSpeeds(((0.8 * SPEED_MOVE) - pid), (0.85 * SPEED_MOVE) + pid);
    }
    while (encoderLeftCounter < targetTick - 15) {
      pid = computePID();
      md.setSpeeds(((0.6 * SPEED_MOVE) - pid), (0.65 * SPEED_MOVE) + pid);
    }
    while (encoderLeftCounter < targetTick) {
      pid = computePID();
      md.setSpeeds(((0.5 * SPEED_MOVE) - pid), (0.55 * SPEED_MOVE) + pid);
    }
    //to bypass the curve motion movement
    //turnRight_sil(2);
  } else if (cm <= 60) {
    //targetTick = cm * 29;//28.5; //29.2
    while (encoderLeftCounter < targetTick  - 50) {
      pid = computePID();
      md.setSpeeds(((SPEED_MOVE * 0.82) - pid), SPEED_MOVE + pid);
    }

    while (encoderLeftCounter < targetTick - 25) {
      pid = computePID();
      md.setSpeeds(((0.8 * SPEED_MOVE) - pid), (0.85 * SPEED_MOVE) + pid);
    }
    while (encoderLeftCounter < targetTick - 15) {
      pid = computePID();
      md.setSpeeds(((0.6 * SPEED_MOVE) - pid), (0.65 * SPEED_MOVE) + pid);
    }
    while (encoderLeftCounter < targetTick) {
      pid = computePID();
      md.setSpeeds(((0.5 * SPEED_MOVE) - pid), (0.55 * SPEED_MOVE) + pid);
    }
    //to bypass the curve motion movement
    //turnRight_sil(2);
  }
  else {
    while (encoderLeftCounter < targetTick  - 50) {
      pid = computePID();
      //md.setSpeeds(-((SPEED_MOVE ) - pid), SPEED_MOVE + pid);
      //md.setSpeeds(-((SPEED_MOVE * 0.86) - pid), SPEED_MOVE + pid);
      //md.setSpeeds(-((SPEED_MOVE * 0.845) - pid), SPEED_MOVE + pid);
      md.setSpeeds(((SPEED_MOVE * 0.7) - pid), SPEED_MOVE + pid);
    }

    while (encoderLeftCounter < targetTick - 25) {
      pid = computePID();
      md.setSpeeds(((0.8 * SPEED_MOVE) - pid), (0.85 * SPEED_MOVE) + pid);
    }
    while (encoderLeftCounter < targetTick - 15) {
      pid = computePID();
      md.setSpeeds(((0.6 * SPEED_MOVE) - pid), (0.65 * SPEED_MOVE) + pid);
    }
    while (encoderLeftCounter < targetTick) {
      pid = computePID();
      md.setSpeeds(((0.5 * SPEED_MOVE) - pid), (0.55 * SPEED_MOVE) + pid);
    }
    //to bypass the curve motion movement
    //rotate back by 5% of the distance
    //100 * 5% = 5
    if (cm <= 80) {
      //turnRight_sil(2.7);
    }
    else {
      //turnRight_sil(3.5);
    }
    //turnRight_sil(3.5);
  }
  md.setBrakes(400, 400);
  //Serial.print("OK\r\n");
}

void moveBack(int cm) {
  double pid;
  int targetTick;
  integral = 0;
  encoderLeftCounter = encoderRightCounter = prevTick = 0;

  targetTick = cm * 30.20;//30.35; // Caliberated to 30.25 ticks per cm

  while (encoderLeftCounter < min(50, targetTick)) {
    pid = computePID();
    md.setSpeeds(((0.5 * SPEED_MOVE) - pid), ((0.5 * SPEED_MOVE) + pid));
  }
  while (encoderLeftCounter < targetTick  - 50) {
    pid = computePID();
    md.setSpeeds(((SPEED_MOVE) - pid), (SPEED_MOVE + pid));
  }
  while (encoderLeftCounter < targetTick) {
    pid = computePID();
    md.setSpeeds(((0.5 * SPEED_MOVE) - pid, ((0.5 * SPEED_MOVE) + pid));
  }

  md.setBrakes(400, 400);
  delay(100);
  Serial.print("OK\r\n");
}


void sensordata() {

  uint32_t lastTime = 0 ;

//avoidBurstRead causes a delay of 20ms
  while (millis() <= lastTime + 20 )
  {
    //wait for sensor's sampling time
  }

  lastTime = millis();

  for (int i = 0; i < SampleSize; i++) {
    sample0.add((long)analogRead(A0));
    sample2.add((long)analogRead(A2));
    sample4.add((long)analogRead(A4));

    sample1.add((long)analogRead(A1));
    sample3.add((long)analogRead(A3));
    sample5.add((long)analogRead(A5));
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
  float dis1 = 1 / (Eq1x * (sensorValue1 * (0.0048875855327468)) - Eq1off) - 0.42;
  float dis2 = 1 / (Eq2x * (sensorValue2 * (0.0048875855327468)) - Eq2off) - 0.42;
  float dis3 = 1 / (Eq3x * (sensorValue3 * (0.0048875855327468)) - Eq3off) - 0.42;
  float dis4 = 1 / (Eq4x * (sensorValue4 * (0.0048875855327468)) - Eq4off) - 0.42;
  float dis5 = 1 / (Eq5x * (sensorValue5 * (0.0048875855327468)) - Eq5off) - 0.42;

  Serial.print(String(dis1) + "," + String(dis3) + "," + String(dis5) + ","
               + String(dis2) + "," + String(dis4) + "," + String(dis6) + "\r\n");

  sample0.clear();
  sample1.clear();
  sample2.clear();
  sample3.clear();
  sample4.clear();
  sample5.clear();
}













void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(M1A, INPUT);
  pinMode(M2A, INPUT);
  digitalWrite(3, LOW);
  digitalWrite(11, LOW);

  enableInterrupt(M1A, showEncode1, FALLING);
  enableInterrupt(M2A, showEncode2, FALLING);
  md.init();
}

void loop() {
  moveForward(50);
  delay(3000);
  moveBack(10);
}

#include <DualVNH5019MotorShield.h>
#include <EnableInterrupt.h>
#include <RunningMedian.h>
#include <SharpIR.h>

/*
     ******************************************************************************************************************************
*/
/**
 * Everything About Sensor
 *
 * MDP Board Pin <> Arduino Pin <> Sensor Range <> Model <> Location
 * Top Sensor
 * PS1 <> A0  <> SE5  Distance <= 85          (1080)    TLeft
 * PS3 <> A2  <> SE7  Distance <= 85          (1080)    TMiddle
 * PS5 <> A4  <> SE2  Distance <= 85          (1080)    TRight
 *
 * Bottom Sensor
 * PS2 <> A1  <> SE3 Distance <=80            (1080)    BRight(Front)
 * PS4 <> A3  <> SE1 Distance 30 <= x <= 130  (20150)   BRight(Back)
 * PS6 <> A5  <> SE6 Distance <=85            (1080)    BLeft(Front)
 *
 *
 * Sensor Variables Declaration
 */

#define TL 0
#define TM 2
#define TR 4
#define BRT 1
#define BRB 3
#define BLT 5

#define irTL A0
#define irTM A2
#define irTR A4
#define irBRT A1
#define irBRB A3
#define irBLT A5

#define MODEL_SHORT 1080 // 1080 (Short), 20150 (Long)
#define MODEL_LONG 20150 // 1080 (Short), 20150 (Long)

#define WALL_GAP 10

SharpIR sensorTL(TL, MODEL_SHORT);
SharpIR sensorTM(TM, MODEL_SHORT);
SharpIR sensorTR(TR, MODEL_SHORT);

SharpIR sensorBRT(BRT, MODEL_SHORT);
SharpIR sensorBRB(BRB, MODEL_LONG);
SharpIR sensorBLT(BLT, MODEL_SHORT);

#define MIN_RANGE_OF_SHORT_SENSOR 1
#define MAX_RANGE_OF_SHORT_SENSOR 4

#define MIN_RANGE_OF_LONG_SENSOR 3
#define MAX_RANGE_OF_LONG_SENSOR 7

#define SHORT_OFFSET 10
#define LONG_OFFSET 20

/*
     ******************************************************************************************************************************
     2Oct2018
     ******************************************************************************************************************************
*/
//1 - TL, 2 - TM, 3 - TR, 4 - BRT, 5 - BLT

//Front
double arrMapping1[] = {10.00, 21.27, 34.61, 43.78, 50.29};
double arrMapping2[] = {10.59, 21.17, 33.25, 44.60, 47.41};
double arrMapping3[] = {10.51, 21.80, 33.96, 42.03, 46.24};

//Right
double arrMapping4[] = {10.97, 23.12, 36.22, 48.37, 54.46};
//Right - LR
double arrMapping0[] = {20.20, 22.85, 30.37, 39.52, 49.73, 62.40, 74.94, 85.88, 96.24};

//Left
double arrMapping5[] = {8.98, 20.59, 32.39, 44.60, 53.33};
/*
     ******************************************************************************************************************************
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

#define kp 40
#define ki 0
#define kd 0

// Moving speed.
#define Speed_Move 325

// Turning speed
#define Speed_Spin 325

#define Speed_Brake 400


//E1 Right Side
const int M1A = 3;
const int M1B = 5;

//E2 Left Side
const int M2A = 11;
const int M2B = 13;

volatile long encoderLeftCounter, encoderRightCounter;

DualVNH5019MotorShield md;

int motorStatus;
double integral;
long prevTick, prevMillis = 0;

//String robotRead;
bool newData = false, isStarted = false;
bool robotReady = false;

String robotRead;

//E1
void showEncode1() {
  encoderLeftCounter++;
}
//E2
void showEncode2() {
  encoderRightCounter++;
}

/**
 * This function is to get encoder values
 */
double computePID() {
  //Serial.println("[PID] M1Ticks(" + String(M1Ticks) + ") - M2Ticks(" + String(M2Ticks) + ") = " + String(M1Ticks-M2Ticks));
  double p, i, d, pid, error, integral;

  error = encoderLeftCounter - encoderRightCounter;
  integral += error;
  p = kp * error;
  i = ki * integral;
  d = kd * (prevTick - encoderLeftCounter);
  pid = p + i + d;
  //Serial.println("PID: " + String(pid));

  return pid;
}

void moveForward(double cm) {
  //Serial.print("Moving Forward by : ");
  //Serial.println(value);
  //delay(3000);
  double pid;
  int targetTick;
  integral = 0;
  encoderLeftCounter = encoderRightCounter = prevTick = 0;

  // Caliberated to 30.25 ticks per cm
  //29.2; //29.3; //29.35; //29.38; //29.4; //29; //29.5; //29.85; //30.05; //30.15; //30.20; //30.35;
  targetTick = cm * 29.65;


  // Move Forward 1 grid
  if (cm <= 10) {
    targetTick = cm * 28.85;
    //27; //27.5; //28.15; //28.65;
    while (encoderLeftCounter < min(50, targetTick)) {
      pid = computePID();
      //Serial.println("R/E1 : " + String((0.6 * Speed_Move) + pid) + " L/E2 : " + String((0.6 * Speed_Move) - pid));
      md.setSpeeds(
        ((Speed_Move) - pid),
        ((Speed_Move) + pid)
      );
    }
    while (encoderLeftCounter < targetTick - 50) {
      pid = computePID();
      //Serial.println("R/E1 : " + String((1.0 * Speed_Move) + pid) + " L/E2 : " + String((1.0 * Speed_Move) - pid));
      md.setSpeeds(
        ((Speed_Move) - pid),
        ((Speed_Move) + pid)
      );
    }
    while (encoderLeftCounter < targetTick - 25) {
      pid = computePID();
      //Serial.println("R/E1 : " + String((0.8 * Speed_Move) + pid) + " L/E2 : " + String((0.8 * Speed_Move) - pid));
      md.setSpeeds(
        ((Speed_Move) - pid),
        ((Speed_Move) + pid)
      );
    }
    while (encoderLeftCounter < targetTick - 15) {
      pid = computePID();
      //Serial.println("R/E1 : " + String((0.6 * Speed_Move) + pid) + " L/E2 : " + String((0.6 * Speed_Move) - pid));
      md.setSpeeds(
        ((Speed_Move) - pid),
        ((Speed_Move) + pid)
      );
    }
    while (encoderLeftCounter < targetTick) {
      pid = computePID();
      //Serial.println("R/E1 : " + String((0.5 * Speed_Move) + pid) + " L/E2 : " + String((0.5 * Speed_Move) - pid));
      md.setSpeeds(
        ((Speed_Move) - pid),
        ((Speed_Move) + pid)
      );
    }
  }
  // Move Forward 2 grids
  else if (cm <= 30) {
    targetTick = cm * 29.5;
    //28.25;//28.5; //29.2
    while (encoderLeftCounter < targetTick) {
      pid = computePID();
      md.setSpeeds(
        ((Speed_Move) - pid),
        ((Speed_Move) + pid)
      );
    }
    //turnRight_sil(1);
  }
  // Move Forward 5 grids
  else if (cm <= 50) {
    //28.75; //29M; //28.5; //29.2
    while (encoderLeftCounter < targetTick - 50) {
      targetTick = cm * 29.75;
      pid = computePID();
      //0.885
      md.setSpeeds(
        ((Speed_Move) - pid),
        ((Speed_Move) + pid)
      );
    }

    while (encoderLeftCounter < targetTick - 25) {
      pid = computePID();
      md.setSpeeds(
        ((0.8 * Speed_Move) - pid),
        ((0.8 * Speed_Move) + pid)
      );
    }
    while (encoderLeftCounter < targetTick - 15) {
      pid = computePID();
      md.setSpeeds(
        ((Speed_Move) - pid),
        ((Speed_Move) + pid)
      );
    }
    while (encoderLeftCounter < targetTick) {
      pid = computePID();
      md.setSpeeds(
        ((Speed_Move) - pid),
        ((Speed_Move) + pid)
      );
    }
    //to bypass the curve motion movement
    //turnRight_sil(2);
  }
  // Move Forward 6 grids
  else if (cm <= 60) {
    //28.5; //29.2
    targetTick = cm * 29.75;
    while (encoderLeftCounter < targetTick - 50) {
      pid = computePID();
      md.setSpeeds(
        ((Speed_Move) - pid),
        ((Speed_Move) + pid)
      );
    }

    while (encoderLeftCounter < targetTick - 25) {
      pid = computePID();
      md.setSpeeds(
        ((Speed_Move) - pid),
        ((Speed_Move) + pid)
      );
    }
    while (encoderLeftCounter < targetTick - 15) {
      pid = computePID();
      md.setSpeeds(
        ((Speed_Move) - pid),
        ((Speed_Move) + pid)
      );
    }
    while (encoderLeftCounter < targetTick) {
      pid = computePID();
      md.setSpeeds(
        ((Speed_Move) - pid),
        ((Speed_Move) + pid)
      );
    }
    //to bypass the curve motion movement
    //turnRight_sil(2);
  }
  // Just Move Forward
  else {
    while (encoderLeftCounter < targetTick) {
      pid = computePID();
      md.setSpeeds (
        (Speed_Move - pid),
        (Speed_Move + pid)
      );
      //Serial.println("M1setSpeed: " + String(int((Speed_Move - pid))) + ", M2setSpeed: " + String(int((Speed_Move + pid))));
      //Serial.println("M1Ticks: " + String(int((encoderRightCounter))) + ", M2Ticks: " + String(int((encoderLeftCounter))));
      //Serial.println();
    }
  }
  brake();
  Serial.println("forward OK");
}

void moveBack(int cm) {
  double pid;
  int targetTick;
  integral = 0;
  encoderLeftCounter = encoderRightCounter = prevTick = 0;

  // Calibrated to 30.25 ticks per cm
  //30.35;
  targetTick = cm * 29.5;

  while (encoderLeftCounter < min(50, targetTick)) {
    pid = computePID();
    md.setSpeeds(
      -((Speed_Move) - pid),
      -((Speed_Move) + pid)
    );
  }

  while (encoderLeftCounter < targetTick - 50) {
    pid = computePID();
    md.setSpeeds(
      -((Speed_Move) - pid),
      -((Speed_Move) + pid)
    );
  }

  while (encoderLeftCounter < targetTick) {
    pid = computePID();
    md.setSpeeds(
      -((Speed_Move) - pid),
      -((Speed_Move) + pid)
    );
  }

  md.setBrakes(400, 400);

  Serial.println("backward OK");
}

void turnLeft(double deg) {
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
      ((Speed_Spin) - pid),
      -((Speed_Spin) + pid)
    );
  }
  while ( encoderLeftCounter < targetTick - 50) {
    pid = computePID();
    md.setSpeeds(
      ((Speed_Spin) - pid),
      -((Speed_Spin) + pid)
    );
  }
  while ( encoderLeftCounter < targetTick) {
    pid = computePID();
    md.setSpeeds(
      ((Speed_Spin) - pid),
      -((Speed_Spin) + pid));
  }

  md.setBrakes(400, 400);

  Serial.println("left OK");
}

void turnRight(double deg) {
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

  else if (deg <= 180) targetTick = deg * 4.33;//4.33(test)//4.333M;//4.335M;//4.336M;//4.338M;//4.342M;//4.335;
  //4.32L;//4.35M;
  //4.34;//4.33;
  //4.34;//4.35M;//4.36;//4.415;//4.63;
  else if (deg <= 360) targetTick = deg * 4.42;//4.4(test)
  else targetTick = deg * 4.48;

  while ( encoderLeftCounter < min(50, targetTick)) {
    pid = computePID();
    md.setSpeeds(
      -((Speed_Spin) - pid),
      ((Speed_Spin) + pid)
    );
  }

  while (encoderLeftCounter < targetTick - 50) {
    pid = computePID();
    md.setSpeeds(
      -((Speed_Spin) - pid),
      ((Speed_Spin) + pid)
    );
  }
  while (encoderLeftCounter < targetTick) {
    pid = computePID();
    md.setSpeeds(
      -((Speed_Spin) - pid),
      ((Speed_Spin) + pid)
    );
  }

  md.setBrakes(400, 400);

  Serial.println("right OK");
}

void obstacleAvoid() {
  /*
    int count = 0;
    bool avoided = false;
      for(int i=0;i<100;i++){
        long ps1 = analogRead(A0);
        long ps2 = analogRead(A1);
        long ps3 = analogRead(A2);
        long ps4 = analogRead(A3);
        long ps5 = analogRead(A4);
        long ps6 = analogRead(A5);
        sample0.add(ps1);
        sample1.add(ps2);
        sample2.add(ps3);
        sample3.add(ps4);
        sample4.add(ps5);
        sample5.add(ps6);
      }
      float sensorValue0 = sample0.getMedian();
      float sensorValue1 = sample1.getMedian();
      float sensorValue2 = sample2.getMedian();
      float sensorValue3 = sample3.getMedian();
      float sensorValue4 = sample4.getMedian();
      float sensorValue5 = sample5.getMedian();

      float voltage0 = sensorValue0 * (5.0 / 1023.0);
      float voltage1 = sensorValue1 * (5.0 / 1023.0);
      float voltage2 = sensorValue2 * (5.0 / 1023.0);
      float voltage3 = sensorValue3 * (5.0 / 1023.0);
      float voltage4 = sensorValue4 * (5.0 / 1023.0);
      float voltage5 = sensorValue5 * (5.0 / 1023.0);

      float dis0=(1/(0.0444*voltage0 - 0.0061)) - 0.42;
      float dis1=(1/(0.0444*voltage1 - 0.0061)) - 0.42;
      float dis2=(1/(0.0417*voltage2 - 0.004)) - 0.42;
      float dis3=(1/(0.0421*voltage3 - 0.0057)) - 0.42;
      float dis4=(1/(0.0428*voltage4 - 0.0048)) - 0.42;
      float dis5=(1/(0.044*voltage5 - 0.009)) - 0.42;

      //front Left sensor has obstacle
      if(avoided==false){
        if(dis0<=15 && dis0 > 0){
          avoidRight();
          avoided=true;
        }
        else if(dis2<=15 && dis2 > 0){
          //front right sensor has obstacle
          avoidLeft();
          avoided=true;
        } //***
        if(dis0<=15 && dis0>0 || dis4<=15 && dis4>0){

          avoided=true;
        }
        else{
          moveForward(10);
          count++;
        }
      }
      else{
          moveForward(10);
          count++;
      }
      delay(500);
      */
  bool avoidComplete = false;
  while (avoidComplete == false) {
    //if right has wall, obstacle at any part
    /*if(final_MedianRead(irBRT)<=25 && final_MedianRead(irBRT)>0 && final_MedianRead(irTR)<=25 && final_MedianRead(irTR)>0 || final_MedianRead(irTM)<=25 && final_MedianRead(irTM)>0 ||
    final_MedianRead(irTL)<=25 && final_MedianRead(irTL)>0) {
      Serial.println("Obstacle near wall");
      turnLeft(45);
      //delay(500);
      moveForward(30);
      //delay(500);
      turnRight(45);
      //delay(500);
      moveForward(20);
      //delay(500);
      turnRight(45);
      //delay(500);
      moveForward(30);
      //delay(500);
      turnLeft(42);
      avoidComplete = true;
    }*/
    //if right no wall, obstacle at front right
    if (final_MedianRead(irTR) <= 25 && final_MedianRead(irTR) > 0) {
      Serial.println("Obstacle at front right");
      turnLeft(45);
      //delay(500);
      moveForward(30);
      //delay(500);
      turnRight(45);
      //delay(500);
      moveForward(20);
      //delay(500);
      turnRight(45);
      //delay(500);
      moveForward(30);
      //delay(500);
      turnLeft(42);
      moveForward(40);
      avoidComplete = true;
    }
    //if right no wall, obstacle at front middle
    else if (final_MedianRead(irTM) <= 25 && final_MedianRead(irTM) > 0) {
      Serial.println("Obstacle at front middle");
      turnRight(45);
      //delay(500);
      moveForward(30);
      //delay(500);
      turnLeft(45);
      //delay(500);
      moveForward(20);
      //delay(500);
      turnLeft(45);
      //delay(500);
      moveForward(30);
      //delay(500);
      turnRight(42);
      moveForward(40);
      avoidComplete = true;
    }
    //if right no wall, obstacle at front left
    if (final_MedianRead(irTL) <= 25 && final_MedianRead(irTL) > 0) {
      Serial.println("Obstacle at front left");
      turnRight(45);
      //delay(500);
      moveForward(30);
      //delay(500);
      turnLeft(47);
      //delay(500);
      moveForward(20);
      //delay(500);
      turnLeft(47);
      //delay(500);
      moveForward(33);
      //delay(500);
      turnRight(42);
      moveForward(40);
      avoidComplete = true;
    }
    else {
      moveForward(10);
    }
    delay(500);
  }
}

void brake() {
  md.setBrakes(Speed_Brake, Speed_Brake - 10);
}

/*
     ********************************************************************************************************************************
*/


void sensordata() {

  String resultTL = String(final_MedianRead(irTL)) + String(" , ");
  String resultTM = String(final_MedianRead(irTM)) + String(" , ");
  String resultTR = String(final_MedianRead(irTR)) + String(" , ");
  String resultBRT = String(final_MedianRead(irBRT)) + String(" , ");
  String resultBRB = String(final_MedianRead(irBRB)) + String(" , ");
  String resultBLT = String(final_MedianRead(irBLT));
  Serial.println("an" + resultTL + resultTM + resultTR + resultBRT + resultBRB + resultBLT);
}

double final_MedianRead(int tpin) {
  double x[9];

  for (int i = 0; i < 9; i ++) {
    x[i] = distanceEvaluate(tpin);
  }

  insertionsort(x, 9);

  return x[4];
}

double distanceEvaluate(int pin)
{
  double distanceReturn = 0.0;
  switch (pin)
  {
  case irTL:
    distanceReturn = sensorTL.distance();
    break;
  case irTM:
    distanceReturn = sensorTM.distance();
    break;
  case irTR:
    distanceReturn = sensorTR.distance();
    break;
  case irBRT:
    distanceReturn = sensorBRT.distance();
    break;
  case irBRB:
    distanceReturn = sensorBRB.distance();
    break;
  case irBLT:
    distanceReturn = sensorBLT.distance();
    break;
  default:
    distanceReturn = 0.0;
    break;
  }
  return distanceReturn;
}

void insertionsort(double array[], int length) {
  double temp;
  for (int i = 1; i < length; i++) {
    for (int j = i; j > 0; j--) {
      if (array[j] < array[j - 1])
      {
        temp = array[j];
        array[j] = array[j - 1];
        array[j - 1] = temp;
      }
      else
        break;
    }
  }
}


void readSensors() {
  int i;
  String output = "";
/*
  double distTL = calibrateSensorValue(sensorTL.distance(), 1);
  double distTM = calibrateSensorValue(sensorTM.distance(), 2);
  double distTR = calibrateSensorValue(sensorTR.distance(), 3);
  double distBLT = calibrateSensorValue(sensorBLT.distance(), 5);
  double distBRT = calibrateSensorValue(sensorBRT.distance(), 4);
  double distBRB = calibrateSensorValue(sensorBRB.distance(), 0);
  */
  double distTL = sensorTL.distance();
  double distTM = sensorTM.distance();
  double distTR = sensorTR.distance();
  double distBLT = sensorBLT.distance();
  double distBRT = sensorBRT.distance();
  double distBRB = sensorBRB.distance();

  int posTL = obstaclePosition(
                distTL,
                1);

  int posTM = obstaclePosition(
                distTM,
                1);

  int posTR = obstaclePosition(
                distTR,
                1);


  int posBLT = obstaclePosition(
                 distBLT,
                 2);


  int posBRT = obstaclePosition(
                 distBRT,
                 2);

  int posBRB = obstaclePosition(
                 distBRB,
                 0);


  // check for any values that are not satisfied
  for (i = 0; i < 5 ; i++) {
    if (posTL != -2) {
      break;
    }
    else {
      posTL = obstaclePosition(
                calibrateSensorValue(
                  sensorTL.distance(), 1),
                1);
    }
  }
  for (i = 0; i < 5 ; i++) {
    if (posTM != -2) {
      break;
    }
    else {
      posTM = obstaclePosition(
                calibrateSensorValue(
                  sensorTM.distance(), 2),
                1);
    }
  }
  for (i = 0; i < 5 ; i++) {
    if (posTR != -2) {
      break;
    }
    else {
      posTR = obstaclePosition(
                calibrateSensorValue(
                  sensorTR.distance(), 3),
                1);
    }
  }
  for (i = 0; i < 5 ; i++) {
    if (posBRT != -2) {
      break;
    }
    else {
      posBRT = obstaclePosition(
                 calibrateSensorValue(
                   sensorBRT.distance(), 4),
                 2);
    }
  }
  for (i = 0; i < 5 ; i++) {
    if (posBLT != -2) {
      break;
    }
    else {
      posBLT = obstaclePosition(
                 calibrateSensorValue(
                   sensorBLT.distance(), 5),
                 2);
    }
  }
  for (i = 0; i < 10 ; i++) {
    if (posBRB != -2) {
      break;
    }
    else {
      posBRB = obstaclePosition(
                 calibrateSensorValue(
                   sensorBRB.distance(), 0),
                 0);
    }
  }

  // ensure that there are no "-1" sensor values
  /*
  posTL = (posTL == -1) ? 0 : posTL;
  posTM = (posTM == -1) ? 0 : posTM;
  posTR = (posTR == -1) ? 0 : posTR;
  posBRT = (posBRT == -1) ? 0 : posBRT;
  posBRB = (posBRB == -1) ? 0 : posBRB;
  posBLT = (posBLT == -1) ? 0 : posBLT;
  */
  
  // for checking how many obstacles are there on left
  /*
  if (!sensor_command) {
    if (forward_command) {
      obstacle_left_rear = obstacle_left_center;
      obstacle_left_center = (posBLT == 1) ? true : false;
      obstacle_right_rear = (posBRB == 1) ? true : false;
    }
    else {
      obstacle_left_center = false;
      obstacle_left_rear = false;
      obstacle_right_rear = false;
    }
  }
  */

  // concatenate all position into a string and send

  output += String(posTL);  output += ",";
  output += String(posTM);  output += ",";
  output += String(posTR);  output += ",";
  output += String(posBRT); output += ",";
  output += String(posBRB); output += ",";
  output += String(posBLT); output += "; \t";
  output += String(distTL);  output += ",";
  output += String(distTM);  output += ",";
  output += String(distTR);  output += ",";
  output += String(distBRT); output += ",";
  output += String(distBRB); output += ",";
  output += String(distBLT)
            ;
  Serial.println(output);
}

double calibrateSensorValue(double dist, int category) {
  double *arr;
  int i, len;

  /*0 - BRB, 1 - TL, 2 - TM, 3 - TR, 4 - BRT, 5 - BLT*/

  switch (category) {
  case 0: arr = arrMapping0; len = sizeof(arrMapping0) / sizeof(*arr); break;
  case 1: arr = arrMapping1; len = sizeof(arrMapping1) / sizeof(*arr); break;
  case 2: arr = arrMapping2; len = sizeof(arrMapping2) / sizeof(*arr); break;
  case 3: arr = arrMapping3; len = sizeof(arrMapping3) / sizeof(*arr); break;
  case 4: arr = arrMapping4; len = sizeof(arrMapping4) / sizeof(*arr); break;
  case 5: arr = arrMapping5; len = sizeof(arrMapping5) / sizeof(*arr); break;
  default: return -1;
  }

  for (i = 0; i < len; i++) {
    if (dist < arr[i]) {
      int a = (i == 0) ? 0 : arr[i - 1];
      int offset = (category == 0) ? 1 : 0;

      if ((i == 0) && (category == 0)) {
        return map(dist, a, arr[i], 0, ((i + offset + 1) * 10));
      }

      return map(dist, a, arr[i], ((i + offset) * 10), ((i + offset + 1) * 10));
    }
  }

  return i * 10;
}

int obstaclePosition(double val, int shortrange) {
  /*
    values for shortrange
    0 = long range
    1 = front
    2 = right & left side
    */

  int tmp = 0;

  //Front
  if (shortrange == 1) {
    tmp = (val - SHORT_OFFSET) / 10;
    Serial.println(tmp);
    if ((tmp < MIN_RANGE_OF_SHORT_SENSOR)){
      return tmp;
    }
    else if ((tmp >= MIN_RANGE_OF_SHORT_SENSOR) && (tmp <= MAX_RANGE_OF_SHORT_SENSOR)) {
      return tmp;
    }
    else{
      return -1;
    }
  }
  //Side
  else if (shortrange == 2) {
    tmp = (val - SHORT_OFFSET) / 10;
    if ((tmp < MIN_RANGE_OF_SHORT_SENSOR)){
      return tmp;
    }
    else if ((tmp >= MIN_RANGE_OF_SHORT_SENSOR) && (tmp <= MAX_RANGE_OF_SHORT_SENSOR)) {
      return tmp;
    }
    else {
      return -1;
    }
  }
  //Long
  else {
    tmp = (val - LONG_OFFSET) / 10;
    if ((tmp < MIN_RANGE_OF_LONG_SENSOR)){
      return 0;
    }
    else if ((tmp >= MIN_RANGE_OF_LONG_SENSOR) && (tmp <= MAX_RANGE_OF_LONG_SENSOR)) {
      return tmp;
    }
    else {
      return -1;
    }
  }
}

/*
     ********************************************************************************************************************************
*/

String getValue(String data, char separator, int index) {
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

void serialEvent() {
  if (Serial.available() > 0) {
    // get the new byte:
    robotRead = Serial.readString();
    if (robotRead.length() != 0) {
      newData = true;
    }
    // add it to the inputString:
    Serial.println("AN" + robotRead);
  }
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  pinMode(irTL, INPUT);
  pinMode(irTM, INPUT);
  pinMode(irTR, INPUT);
  pinMode(irBRT, INPUT);
  pinMode(irBRB, INPUT);
  pinMode(irBLT, INPUT);

  digitalWrite(irTL, LOW);
  digitalWrite(irTM, LOW);
  digitalWrite(irTR, LOW);
  digitalWrite(irBRT, LOW);
  digitalWrite(irBRB, LOW);
  digitalWrite(irBLT, LOW);


  enableInterrupt(M1A, showEncode1, RISING);
  enableInterrupt(M2B, showEncode2, RISING);

  md.init();
}

void loop() {
  if (robotRead == "ok") {
    if (robotReady == false) {
      robotReady = true;
      Serial.println("Ok");
    }
  }
  if (newData) {
    double movementValue = getValue(robotRead, ';', 1).toInt();
    char condition = robotRead.charAt(0);
    if (robotRead == "motor") {
      isStarted = !isStarted;
    }
    switch (condition) {
    case 'W':
    case 'w':
    {
      (movementValue == 0) ? moveForward(10) : moveForward(movementValue);
      break;
    }

    case 'A':
    case 'a':
    {
      (movementValue == 0) ? turnLeft(90) : turnLeft(movementValue);
      break;
    }

    case 'D':
    case 'd':
    {
      (movementValue == 0) ? turnRight(90) : turnRight(movementValue);
      break;
    }

    case 'S':
    case 's':
    {
      (movementValue == 0) ? moveBack(10) : moveBack(movementValue);
      break;
    }
    case 'Z':
    case 'z':
    {
      sensordata();
      break;
    }
    case 'X':
    case 'x': {
      obstacleAvoid();
      break;
    }
    case 'C':
    case 'c':
    {
      //caliberate();
      readSensors();

      break;
    }
    case 'V':
    case 'v':
    {
      //updateSensorData();
      break;
    }
    case 'T':
    case 't':
    {
      //(movementValue == 0) ? testMotors(0) : testMotors(movementValue);
      break;
    }
    default:
    {
      //defaultResponse();
      break;
    }
    }

    robotRead = "";
    newData = false;
  }
  //obstacleAvoid();
  //md.setSpeeds(325,325);
  //sensordata();
  //moveForward(10);
  //moveBack(10);
  //delay(1000);
  //turnRight(180);
  //delay(1000);
  //moveForward(10);
  //delay(1000);
  //turnRight(90);

  //delay(500);
}

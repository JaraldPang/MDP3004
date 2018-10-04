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

#define irTL A0
#define irTM A2
#define irTR A4
#define irBRT A1
#define irBRB A3
#define irBLT A5

#define MODEL_SHORT 1080 // 1080 (Short), 20150 (Long)
#define MODEL_LONG 20150 // 1080 (Short), 20150 (Long)

#define WALL_GAP 10

SharpIR sensorTL(irTL, MODEL_SHORT);
SharpIR sensorTM(irTM, MODEL_SHORT);
SharpIR sensorTR(irTR, MODEL_SHORT);

SharpIR sensorBRT(irBRT, MODEL_SHORT);
SharpIR sensorBRB(irBRB, MODEL_LONG);
SharpIR sensorBLT(irBLT, MODEL_SHORT);

#define MIN_RANGE_OF_SHORT_SENSOR 1
#define MAX_RANGE_OF_SHORT_SENSOR 4

#define MIN_RANGE_OF_LONG_SENSOR 3
#define MAX_RANGE_OF_LONG_SENSOR 7

#define SHORT_OFFSET 10
#define LONG_OFFSET 20

#define WALL_GAP 10

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

// Brake speed
#define Speed_Brake 400

// Calibration speed
#define Speed_Calibration 200

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

//position calibration variables
#define STEPS_TO_CALIBRATE 5
#define STEPS_TO_BEST_CALIBRATE 3

int step_counter = 0, step_best_calibrate = 0;
bool obstacle_left_center, obstacle_left_rear, obstacle_right_rear;
bool opportunity_calibrate_left, opportunity_calibrate_front, opportunity_calibrate_right;
bool mode_fastest_path, mode_fastest_diagonal, mode_calibration, forward_command, sensor_command;


bool calibration_state = false;

/*
     ******************************************************************************************************************************
*/

String robotRead;

String messageHeader = "an";
String messageTail = "";

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

  double pid;
  int targetTick;
  int Set_Speed = (calibration_state == true) ? Speed_Calibration : Speed_Move;

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
      //Serial.println("R/E1 : " + String((0.6 * Set_Speed) + pid) + " L/E2 : " + String((0.6 * Set_Speed) - pid));
      md.setSpeeds(
        ((Set_Speed) - pid),
        ((Set_Speed) + pid)
      );
    }
    while (encoderLeftCounter < targetTick - 50) {
      pid = computePID();
      //Serial.println("R/E1 : " + String((1.0 * Set_Speed) + pid) + " L/E2 : " + String((1.0 * Set_Speed) - pid));
      md.setSpeeds(
        ((Set_Speed) - pid),
        ((Set_Speed) + pid)
      );
    }
    while (encoderLeftCounter < targetTick - 25) {
      pid = computePID();
      //Serial.println("R/E1 : " + String((0.8 * Set_Speed) + pid) + " L/E2 : " + String((0.8 * Set_Speed) - pid));
      md.setSpeeds(
        ((Set_Speed) - pid),
        ((Set_Speed) + pid)
      );
    }
    while (encoderLeftCounter < targetTick - 15) {
      pid = computePID();
      //Serial.println("R/E1 : " + String((0.6 * Set_Speed) + pid) + " L/E2 : " + String((0.6 * Set_Speed) - pid));
      md.setSpeeds(
        ((Set_Speed) - pid),
        ((Set_Speed) + pid)
      );
    }
    while (encoderLeftCounter < targetTick) {
      pid = computePID();
      //Serial.println("R/E1 : " + String((0.5 * Set_Speed) + pid) + " L/E2 : " + String((0.5 * Set_Speed) - pid));
      md.setSpeeds(
        ((Set_Speed) - pid),
        ((Set_Speed) + pid)
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
        ((Set_Speed) - pid),
        ((Set_Speed) + pid)
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
        ((Set_Speed) - pid),
        ((Set_Speed) + pid)
      );
    }

    while (encoderLeftCounter < targetTick - 25) {
      pid = computePID();
      md.setSpeeds(
        ((0.8 * Set_Speed) - pid),
        ((0.8 * Set_Speed) + pid)
      );
    }
    while (encoderLeftCounter < targetTick - 15) {
      pid = computePID();
      md.setSpeeds(
        ((Set_Speed) - pid),
        ((Set_Speed) + pid)
      );
    }
    while (encoderLeftCounter < targetTick) {
      pid = computePID();
      md.setSpeeds(
        ((Set_Speed) - pid),
        ((Set_Speed) + pid)
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
        ((Set_Speed) - pid),
        ((Set_Speed) + pid)
      );
    }

    while (encoderLeftCounter < targetTick - 25) {
      pid = computePID();
      md.setSpeeds(
        ((Set_Speed) - pid),
        ((Set_Speed) + pid)
      );
    }
    while (encoderLeftCounter < targetTick - 15) {
      pid = computePID();
      md.setSpeeds(
        ((Set_Speed) - pid),
        ((Set_Speed) + pid)
      );
    }
    while (encoderLeftCounter < targetTick) {
      pid = computePID();
      md.setSpeeds(
        ((Set_Speed) - pid),
        ((Set_Speed) + pid)
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
        (Set_Speed - pid),
        (Set_Speed + pid)
      );
      //Serial.println("M1setSpeed: " + String(int((Set_Speed - pid))) + ", M2setSpeed: " + String(int((Set_Speed + pid))));
      //Serial.println("M1Ticks: " + String(int((encoderRightCounter))) + ", M2Ticks: " + String(int((encoderLeftCounter))));
      //Serial.println();
    }
  }

  md.setBrakes(Speed_Brake, Speed_Brake - 10);

  Serial.println(messageHeader + "ok" + messageTail);
}

void moveBack(int cm) {
  double pid;
  int targetTick;
  int Set_Speed = (calibration_state == true) ? Speed_Calibration : Speed_Move;

  integral = 0;
  encoderLeftCounter = encoderRightCounter = prevTick = 0;

  // Calibrated to 30.25 ticks per cm
  //30.35;
  targetTick = cm * 29.5;

  while (encoderLeftCounter < min(50, targetTick)) {
    pid = computePID();
    md.setSpeeds(
      -((Set_Speed) - pid),
      -((Set_Speed) + pid)
    );
  }

  while (encoderLeftCounter < targetTick - 50) {
    pid = computePID();
    md.setSpeeds(
      -((Set_Speed) - pid),
      -((Set_Speed) + pid)
    );
  }

  while (encoderLeftCounter < targetTick) {
    pid = computePID();
    md.setSpeeds(
      -((Set_Speed) - pid),
      -((Set_Speed) + pid)
    );
  }

  md.setBrakes(400, 400);

  Serial.println(messageHeader + "ok" + messageTail);
}

void turnLeft(double deg) {
  double pid;
  float targetTick;
  int Set_Speed = (calibration_state == true) ? Speed_Calibration : Speed_Spin;

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
      ((Set_Speed) - pid),
      -((Set_Speed) + pid)
    );
  }
  while ( encoderLeftCounter < targetTick - 50) {
    pid = computePID();
    md.setSpeeds(
      ((Set_Speed) - pid),
      -((Set_Speed) + pid)
    );
  }
  while ( encoderLeftCounter < targetTick) {
    pid = computePID();
    md.setSpeeds(
      ((Set_Speed) - pid),
      -((Set_Speed) + pid));
  }

  md.setBrakes(Speed_Brake, Speed_Brake);

  Serial.println(messageHeader + "ok" + messageTail);
}

void turnRight(double deg) {
  double pid;
  float targetTick;
  int Set_Speed = (calibration_state == true) ? Speed_Calibration : Speed_Spin;

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
      -((Set_Speed) - pid),
      ((Set_Speed) + pid)
    );
  }

  while (encoderLeftCounter < targetTick - 50) {
    pid = computePID();
    md.setSpeeds(
      -((Set_Speed) - pid),
      ((Set_Speed) + pid)
    );
  }
  while (encoderLeftCounter < targetTick) {
    pid = computePID();
    md.setSpeeds(
      -((Set_Speed) - pid),
      ((Set_Speed) + pid)
    );
  }

  md.setBrakes(Speed_Brake, Speed_Brake);

  Serial.println(messageHeader + "ok" + messageTail);
}

void obstacleAvoid() {
  bool avoidComplete = false;
  while (avoidComplete == false) {
    //if left has wall, obstacle at any part
    if (final_MedianRead(irBLT) <= 25 && final_MedianRead(irBLT) > 0 && final_MedianRead(irTR) <= 15 && final_MedianRead(irTR) > 0) {
      Serial.println("Obstacle near wall");
      turnRight(90);
      delay(500);
      moveForward(10);
      delay(500);
      turnLeft(90);
      delay(500);
      turnRight(45);
      delay(500);
      moveForward(20);
      delay(500);
      turnLeft(45);
      delay(500);
      moveForward(30);
      delay(500);
      turnLeft(45);
      delay(500);
      moveForward(20);
      delay(500);
      turnRight(42);
      avoidComplete = true;
    }
    else if (final_MedianRead(irBRB) <= 25 && final_MedianRead(irBRB) > 0 && final_MedianRead(irTL) <= 15 && final_MedianRead(irTL) > 0) {
      Serial.println("Obstacle near wall");
      turnLeft(90);
      delay(500);
      moveForward(10);
      delay(500);
      turnLeft(90);
      delay(500);
      turnLeft(45);
      delay(500);
      moveForward(20);
      delay(500);
      turnRight(45);
      delay(500);
      moveForward(30);
      delay(500);
      turnRight(45);
      delay(500);
      moveForward(20);
      delay(500);
      turnLeft(42);
      avoidComplete = true;
    }

    //if right no wall, obstacle at front right
    if (final_MedianRead(irTR) <= 15 && final_MedianRead(irTR) > 0) {
      Serial.println("Obstacle at front right");
      turnLeft(45);
      delay(500);
      moveForward(20);
      delay(500);
      turnRight(45);
      delay(500);
      moveForward(30);
      delay(500);
      turnRight(45);
      delay(500);
      moveForward(20);
      delay(500);
      turnLeft(42);
      moveForward(40);
      avoidComplete = true;
    }
    //if right no wall, obstacle at front middle
    else if (final_MedianRead(irTM) <= 15 && final_MedianRead(irTM) > 0) {
      Serial.println("Obstacle at front middle");
      turnRight(45);
      delay(500);
      moveForward(20);
      delay(500);
      turnLeft(45);
      delay(500);
      moveForward(30);
      delay(500);
      turnLeft(45);
      delay(500);
      moveForward(20);
      delay(500);
      turnRight(42);
      moveForward(40);
      avoidComplete = true;
    }
    //if right no wall, obstacle at front left
    if (final_MedianRead(irTL) <= 15 && final_MedianRead(irTL) > 0) {
      Serial.println("Obstacle at front left");
      turnRight(45);
      delay(500);
      moveForward(20);
      delay(500);
      turnLeft(47);
      delay(500);
      moveForward(30);
      delay(500);
      turnLeft(47);
      delay(500);
      moveForward(20);
      delay(500);
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

void startCalibrate() {
  double distTL = calibrateSensorValue(sensorTL.distance(), 1);
  double distTM = calibrateSensorValue(sensorTM.distance(), 2);
  double distTR = calibrateSensorValue(sensorTR.distance(), 3);
  double distBRT = calibrateSensorValue(sensorBRT.distance(), 4);
  double distBLT = calibrateSensorValue(sensorBLT.distance(), 5);
  double distBRB = calibrateSensorValue(sensorBRB.distance(), 0);

  turnLeft(180);
  delay(500);
  calibrateWithFront();
  if ((distBLT <= WALL_GAP + 4) && ((distBLT <= (WALL_GAP - 2)) || (distBLT >= (WALL_GAP + 2)))) {
    turnLeft(90);
    delay(500);
    calibrateWithFront();
    delay(500);
    turnLeft(90);
  }
  else if ((distBRT <= WALL_GAP + 4) && ((distBRT <= (WALL_GAP - 2)) || (distBRT >= (WALL_GAP + 2)))) {
    turnRight(90);
    delay(500);
    calibrateWithFront();
    delay(500);
    turnRight(90);
  }
}

void autoCalibrate(int force_calibrate) {
  calibration_state = true;

  bool to_calibrate = false;

  double distTL = calibrateSensorValue(sensorTL.distance(), 1);
  double distTM = calibrateSensorValue(sensorTM.distance(), 2);
  double distTR = calibrateSensorValue(sensorTR.distance(), 3);
  double distBRT = calibrateSensorValue(sensorBRT.distance(), 4);
  double distBLT = calibrateSensorValue(sensorBLT.distance(), 5);
  double distBRB = calibrateSensorValue(sensorBRB.distance(), 0);

  int calibrate_front = 0;
  int calibrate_right = 0;

  if (force_calibrate == 0) {
    bool to_calibrate1, to_calibrate2, to_calibrate3, to_calibrate4, to_calibrate5, to_calibrate6;
    to_calibrate1 = ((distTL <= WALL_GAP + 4) && ((distTL <= (WALL_GAP - 2)) || (distTL >= (WALL_GAP + 2)))) ? true : false;
    to_calibrate2 = ((distTM <= WALL_GAP + 4) && ((distTM <= (WALL_GAP - 2)) || (distTM >= (WALL_GAP + 2)))) ? true : false;
    to_calibrate3 = ((distTR <= WALL_GAP + 4) && ((distTR <= (WALL_GAP - 2)) || (distTR >= (WALL_GAP + 2)))) ? true : false;
    to_calibrate4 = ((distBRT <= WALL_GAP + 4) && ((distBRT <= (WALL_GAP - 2)) || (distBRT >= (WALL_GAP + 2)))) ? true : false;
    to_calibrate5 = ((distBRB <= (2 * WALL_GAP) + 4) && ((distBRB <= (2 * WALL_GAP - 1)) || (distBRB >= ((2 * WALL_GAP) + 2)))) ? true : false;
    to_calibrate6 = ((distBLT <= WALL_GAP + 4) && ((distBLT <= (WALL_GAP - 2)) || (distBLT >= (WALL_GAP + 2)))) ? true : false;
    to_calibrate = (to_calibrate1 || to_calibrate2 || to_calibrate3 || to_calibrate4 || to_calibrate5 || to_calibrate6);
    force_calibrate++;
  }
  else if ((force_calibrate == 1) || (force_calibrate == 2)) {
    to_calibrate = true;
  }
  else {
    return;
  }

  force_calibrate++;

  // check for best opportunity to calibrate
  if (step_best_calibrate >= STEPS_TO_BEST_CALIBRATE) {
    if (((abs(distTL - distTR) < 5) && ((distTL + distTR) <= (3 * WALL_GAP))) || ((abs(distTL - distTM) < 5) && ((distTL + distTM) <= (3 * WALL_GAP))) || ((abs(distTM - distTR) < 5) && ((distTM + distTR) <= (3 * WALL_GAP)))) {
      if ((distBRT <= (WALL_GAP + 4)) || (distBLT <= (WALL_GAP + 4)) || (distBRB <= ((2 * WALL_GAP) + 4))) {
        step_best_calibrate = 0;
        to_calibrate = true;
      }
    }
    else if ((abs(distBRT - distBRB) < 5) && ((distBRT + distBRB) <= (3 * WALL_GAP))) {
      if ((distTL <= (WALL_GAP + 4)) || (distTM <= (WALL_GAP + 4)) || (distTR <= (WALL_GAP + 4))) {
        step_best_calibrate = 0;
        to_calibrate = true;
      }
    }
  }

  if (to_calibrate) {
    if (opportunity_calibrate_left) {
      step_counter = 0;
      turnLeft(90);
      delay(500);
      calibrateWithFront();
      delay(500);
      turnRight(90);

      distTL = calibrateSensorValue(sensorTL.distance(), 1);

      if ((distTL <= WALL_GAP + 4) && ((distTL <= (WALL_GAP - 2)) || (distTL >= (WALL_GAP + 2)))) {
        calibrateDistance(sensorTL, 1);
      }
      else {
        distTM = calibrateSensorValue(sensorTM.distance(), 2);

        if ((distTM <= WALL_GAP + 4) && ((distTM <= (WALL_GAP - 2)) || (distTM >= (WALL_GAP + 2)))) {
          calibrateDistance(sensorTM, 2);
        }
        else {
          distTR = calibrateSensorValue(sensorTR.distance(), 3);

          if ((distTR <= WALL_GAP + 4) && ((distTR <= (WALL_GAP - 2)) || (distTR >= (WALL_GAP + 2)))) {
            calibrateDistance(sensorTR, 3);
          }
        }
      }
    }
    else {
      if ((abs(distTL - distTR) < 5) && ((distTL + distTR) <= (3 * WALL_GAP))) {
        // check if target side can calibrate angle
        calibrateAngle(sensorTL, 1, sensorTR, 3, 17);
        calibrateDistance(sensorTL, 1);
        calibrate_front = 1;
      }
      else if ((abs(distTL - distTM) < 5) && ((distTL + distTM) <= (3 * WALL_GAP))) {
        calibrateAngle(sensorTL, 1, sensorTM, 2, 9);
        calibrateDistance(sensorTM, 2);
        calibrate_front = 2;
      }
      else if ((abs(distTM - distTR) < 5) && ((distTM + distTR) <= (3 * WALL_GAP))) {
        calibrateAngle(sensorTM, 2, sensorTR, 3, 9);
        calibrateDistance(sensorTM, 2);
        calibrate_front = 3;
      }
      // check for 1 obstacle on other side
      // if yes, calibrate dist on other side
      if (calibrate_front > 0) {
        step_counter = 0;

        distBRT = calibrateSensorValue(sensorTR.distance(), 4);
        if ((distBRT <= (WALL_GAP + 4)) && ((distBRT >= (WALL_GAP + 2)) || (distBRT <= (WALL_GAP - 2)))) {
          turnRight(90);
          delay(500);
          calibrateDistance(sensorTL, 1);
          delay(500);
          turnLeft(90);
          calibrate_right = 1;
        }
        else {
          distBRB = calibrateSensorValue(sensorBRB.distance(), 0);
          if ((distBRB <= ((WALL_GAP * 2) + 4)) && ((distBRB >= (2 * WALL_GAP + 2)) || (distBRB <= (2 * WALL_GAP - 1)))) {
            turnRight(90);
            delay(500);
            calibrateDistance(sensorTM, 2);
            delay(500);
            turnLeft(90);
            calibrate_right = 1;
          }
          else {
            distBLT = calibrateSensorValue(sensorTL.distance(), 5);

            if ((distBLT <= (WALL_GAP + 4)) && ((distBLT >= (WALL_GAP + 2)) || (distBLT <= (WALL_GAP - 2)))) {
              turnLeft(90);
              delay(500);
              calibrateDistance(sensorTR, 3);
              delay(500);
              turnRight(90);
              calibrate_right = 1;
            }
          }
        }

        // calibrate the angle after turning back
        if (calibrate_right == 1) {
          switch (calibrate_front) {
          case 1: calibrateAngle(sensorTL, 1, sensorTR, 3, 17); break;
          case 2: calibrateAngle(sensorTL, 1, sensorTM, 2, 9); break;
          case 3: calibrateAngle(sensorTM, 2, sensorTR, 3, 9); break;
          default: break;
          }
        }
      }
      else if (((abs(distBRT - distBRB) < 5) && ((distBRT + distBRB) <= (3 * WALL_GAP))) || opportunity_calibrate_right || (forward_command && obstacle_right_rear && ((distBRT <= (WALL_GAP + 4)) || (distBRB <= (WALL_GAP + 4))))) {
        // check for right wall and calibrate
        step_counter = 0;

        // check to see if there is enough clearance from wall
        if (distTL <= (WALL_GAP + 4)) {
          calibrateDistance(sensorTL, 1);
        }
        else if (distTM <= (WALL_GAP + 4)) {
          calibrateDistance(sensorTM, 2);
        }
        else if (distTR <= (WALL_GAP + 4)) {
          calibrateDistance(sensorTR, 3);
        }

        if ((abs(distBRT - distBRB) < 5) && ((distBRT + distBRB) <= (3 * WALL_GAP))) {
          if (((distBRT + distBRB) < ((2 * WALL_GAP) - 4)) || ((distBRT + distBRB) > ((2 * WALL_GAP) + 4))) {
            // calibrate to right
            turnRight(90);
            delay(500);
            calibrateWithFront();
            delay(500);
            turnLeft(90);
          }
          else {
            calibrateAngle(sensorTR, 4, sensorBRB, 5, 9);
          }
        }
        else {
          turnRight(90);
          delay(500);
          calibrateWithFront();
          delay(500);
          turnLeft(90);
        }


        // check for front obstacles to calibrate with
        distTL = calibrateSensorValue(sensorTL.distance(), 1);
        if (distTL <= (WALL_GAP + 4)) {
          calibrateDistance(sensorTL, 1);
          calibrate_front = 1;
        }
        else {
          distTM = calibrateSensorValue(sensorTM.distance(), 2);
          if (distTM <= (WALL_GAP + 4)) {
            calibrateDistance(sensorTM, 2);
            calibrate_front = 2;
          }
          else {
            distTR = calibrateSensorValue(sensorTR.distance(), 3);
            if (distTR <= (WALL_GAP + 4)) {
              calibrateDistance(sensorTR, 3);
              calibrate_front = 3;
            }
          }
        }
      }
      else {
        // turnLeft(90);
        // calibrateWithFront();
        // turnRight(90);

        // distTL = calibrateSensorValue(sensorTL.distance(), 1);

        // if ((distTL <= WALL_GAP + 4) && ((distTL <= (WALL_GAP - 2)) || (distTL >= (WALL_GAP + 2)))) {
        //   calibrateDistance(sensorTL, 1);
        // }
        // else {
        //   distTM = calibrateSensorValue(sensorTM.distance(), 2);

        //   if ((distTM <= WALL_GAP + 4) && ((distTM <= (WALL_GAP - 2)) || (distTM >= (WALL_GAP + 2)))) {
        //     calibrateDistance(sensorTM, 2);
        //   }
        //   else {
        //     distTR = calibrateSensorValue(sensorTR.distance(), 3);

        //     if ((distTR <= WALL_GAP + 4) && ((distTR <= (WALL_GAP - 2)) || (distTR >= (WALL_GAP + 2)))) {
        //       calibrateDistance(sensorTR, 3);
        //     }
        //   }
        // }
      }
    }
  }
  else if ((abs(distBRT - distBRB) < 5) && ((distBRT + distBRB) <= (3 * WALL_GAP))) {
    step_counter = 0;

    calibrateAngle(sensorTR, 4, sensorBRB, 5, 9);

    if (((distBRT + distBRB) < ((2 * WALL_GAP) - 4)) || ((distBRT + distBRB) > ((2 * WALL_GAP) + 4))) {
      turnRight(90);
      delay(500);
      calibrateDistance(sensorTL, 1);
      delay(500);
      turnLeft(90);
      calibrateAngle(sensorTR, 4, sensorBRB, 5, 9);
    }
  }
  else if (distBLT <= ((WALL_GAP * 2) + 4)) {
    if ((distBLT <= ((WALL_GAP * 2) - 1)) || (distBLT >= ((WALL_GAP * 2) + 2))) {
      step_counter = STEPS_TO_CALIBRATE;

      turnLeft(90);
      delay(500);
      if (calibrateWithFront()) {
        step_counter = 0;
      }
      delay(500);
      turnRight(90);

      distTL = calibrateSensorValue(sensorTL.distance(), 1);

      if ((distTL <= WALL_GAP + 4) && ((distTL <= (WALL_GAP - 2)) || (distTL >= (WALL_GAP + 2)))) {
        calibrateDistance(sensorTL, 1);
      }
      else {
        distTM = calibrateSensorValue(sensorTM.distance(), 2);

        if ((distTM <= WALL_GAP + 4) && ((distTM <= (WALL_GAP - 2)) || (distTM >= (WALL_GAP + 2)))) {
          calibrateDistance(sensorTM, 2);
        }
        else {
          distTR = calibrateSensorValue(sensorTR.distance(), 3);

          if ((distTR <= WALL_GAP + 4) && ((distTR <= (WALL_GAP - 2)) || (distTR >= (WALL_GAP + 2)))) {
            calibrateDistance(sensorTR, 3);
          }
        }
      }
    }
  }


  distTL = calibrateSensorValue(sensorTL.distance(), 1);
  distTM = calibrateSensorValue(sensorTM.distance(), 2);
  distTR = calibrateSensorValue(sensorTR.distance(), 3);
  distBRT = calibrateSensorValue(sensorBRT.distance(), 4);
  distBLT = calibrateSensorValue(sensorBLT.distance(), 5);
  distBRB = calibrateSensorValue(sensorBRB.distance(), 0);

  // if too close to one obstacle (calibrate to that one obstacle)
  if (distTL <= WALL_GAP - 2) {
    if (!calibrateWithFront()) {
      calibrateDistance(sensorTL, 1);
    }
  }

  if (distTM <= WALL_GAP - 2) {
    if (!calibrateWithFront()) {
      calibrateDistance(sensorTM, 2);
    }
  }

  if (distTR <= WALL_GAP - 2) {
    if (!calibrateWithFront()) {
      calibrateDistance(sensorTR, 3);
    }
  }

  if (distBRT <= WALL_GAP - 2) {
    turnRight(90);
    if (!calibrateWithFront()) {
      calibrateDistance(sensorTL, 1);
    }
    turnLeft(90);
  }

  if (distBRB <= WALL_GAP - 2) {
    turnRight(90);
    if (!calibrateWithFront()) {
      calibrateDistance(sensorTM, 2);
    }
    turnLeft(90);
  }

  if (distBLT <= ((WALL_GAP * 2) - 1)) {
    turnLeft(90);
    if (!calibrateWithFront()) {
      calibrateDistance(sensorTR, 3);
    }
    turnRight(90);
  }

  calibration_state = false;
}

bool calibrateWithLeft() {
  bool output = false;

  turnLeft(90);
  delay(500);
  output = calibrateWithFront();
  delay(500);
  turnRight(90);

  return output; // return whether calibrated angle
}

bool calibrateWithFront() {
  bool output = false;

  double distBLT = calibrateSensorValue(sensorTL.distance(), 1);
  double distTM = calibrateSensorValue(sensorTM.distance(), 2);
  double distTR = calibrateSensorValue(sensorTR.distance(), 3);

  if ((abs(distBLT - distTR) < 5) && ((distBLT + distTR) <= (3 * WALL_GAP))) {
    calibrateAngle(sensorTL, 1, sensorTR, 3, 17);
    calibrateDistance(sensorTL, 1);
    output = true;
  }
  else if ((abs(distBLT - distTM) < 5) && ((distBLT + distTM) <= (3 * WALL_GAP))) {
    calibrateAngle(sensorTL, 1, sensorTM, 2, 9);
    calibrateDistance(sensorTM, 2);
    output = true;
  }
  else if ((abs(distTM - distTR) < 5) && ((distTM + distTR) <= (3 * WALL_GAP))) {
    calibrateAngle(sensorTM, 2, sensorTR, 3, 9);
    calibrateDistance(sensorTM, 2);
    output = true;
  }

  // if (distTL <= (WALL_GAP + 4)) {
  //   calibrateDistance(sensorTL, 1);
  // }
  // else if (distTM <= (WALL_GAP + 4)) {
  //   calibrateDistance(sensorTM, 2);
  // }
  // else if (distTR <= (WALL_GAP + 4)) {
  //   calibrateDistance(sensorTR, 3);
  // }

  return output; // return whether calibrated angle
}

bool calibrateWithRight() {
  bool output = false;
  /*
  double distBLT = calibrateSensorValue(sensorBLT.distance(), 5);
  double distTR = calibrateSensorValue(sensorBRT.distance(), 4);

  if ((abs(distBLT - distTR) < 5) && ((distBLT + distTR) <= (3 * WALL_GAP))) {
    if (((distBLT + distTR) < ((2 * WALL_GAP) - 1)) || ((distBLT + distTR) > ((2 * WALL_GAP) + 1))) {
      turnRight(90);
      calibrateWithFront();
      turnLeft(90);
    }
    else {
      calibrateAngle(sensorTL, 1, sensorTR, 3, 17);
    }
    output = true;
  }
  else {
    turnRight(90);
    output = calibrateWithFront();
    turnLeft(90);
  }

  // TODO: implement check and calibrate with front
  */
  turnRight(90);
  delay(500);
  output = calibrateWithFront();
  delay(500);
  turnLeft(90);

  return output; // return whether calibrated angle
}

void calibrateAngle(SharpIR sensorTL, int arrL, SharpIR sensorR, int arrR, int dist) {
  mode_calibration = true;
  double distL = calibrateSensorValue(sensorTL.distance(), arrL);
  double distR = calibrateSensorValue(sensorR.distance(), arrR);
  double diff = abs(distL - distR);
  int i = 0;
  double angle = 0;

  for (i = 0; i < 9; i++) {
    if (diff < 0.25) {
      break;
    }
    // Serial.println("dist: " + String(distBLT) + ", " + String(distTR) + ", " + String(diff) + ", " + String(diff));
    angle = (atan(diff / dist) * (180 / 3.14159265));
    //angle = (diff > 0.5) ? angle : angle/2;
    angle = angle / 2;
    // Serial.println("angle: " + String(angle));
    if (distL > distR) {
      turnRight(angle);
    }
    else if (distR > distL) {
      turnLeft(angle);
    }
    delay(20);
    distL = calibrateSensorValue(sensorTL.distance(), arrL);
    distR = calibrateSensorValue(sensorR.distance(), arrR);
    diff = abs(distL - distR);
  }
  // Serial.println(diff);
  mode_calibration = false;
}

void calibrateDistance(SharpIR sensor, int arr) {
  mode_calibration = true;
  double dist = calibrateSensorValue(sensor.distance(), arr);
  //double dist = sensor.distance();

  if (dist < WALL_GAP) {
    moveBack(WALL_GAP - dist);
  }
  else if (dist > WALL_GAP) {
    moveForward(dist - WALL_GAP);
  }
  mode_calibration = false;
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
  Serial.println(messageHeader + resultTL + resultTM + resultTR + resultBRT + resultBRB + resultBLT + messageTail);
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
  double distTL = final_MedianRead(irTL);
  double distTM = final_MedianRead(irTM);
  double distTR = final_MedianRead(irTR);
  double distBLT = final_MedianRead(irBLT);
  double distBRT = final_MedianRead(irBRT);
  double distBRB = final_MedianRead(irBRB);

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
  /*
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
  */

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
  output += String(posBLT);
  Serial.println("alsensor" + output + messageTail);
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
    //Serial.println(tmp);
    if ((tmp < MIN_RANGE_OF_SHORT_SENSOR)) {
      return tmp;
    }
    else if ((tmp >= MIN_RANGE_OF_SHORT_SENSOR) && (tmp <= MAX_RANGE_OF_SHORT_SENSOR)) {
      return tmp;
    }
    else {
      return -1;
    }
  }
  //Side
  else if (shortrange == 2) {
    tmp = (val - SHORT_OFFSET) / 10;
    if ((tmp < MIN_RANGE_OF_SHORT_SENSOR)) {
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
    if ((tmp < MIN_RANGE_OF_LONG_SENSOR)) {
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
    Serial.println(messageHeader + robotRead + messageTail);
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
      readSensors();
      //startCalibrate();
      Serial.println(messageHeader + "ok" + messageTail);
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
      /*
      if (step_counter == STEPS_TO_CALIBRATE) {
        autoCalibrate(1);
      }
      */
      step_counter++;
      step_best_calibrate++;
      readSensors();
      break;
    }

    case 'A':
    case 'a':
    {
      (movementValue == 0) ? turnLeft(90) : turnLeft(movementValue);
      /*
      if (step_counter == STEPS_TO_CALIBRATE) {
        autoCalibrate(1);
      }
      */
      step_counter++;
      step_best_calibrate++;
      readSensors();
      break;
    }

    case 'D':
    case 'd':
    {
      (movementValue == 0) ? turnRight(90) : turnRight(movementValue);
      /*
      if (step_counter == STEPS_TO_CALIBRATE) {
        autoCalibrate(1);
      }
      */
      step_counter++;
      step_best_calibrate++;
      readSensors();
      break;
    }

    case 'S':
    case 's':
    {
      (movementValue == 0) ? moveBack(10) : moveBack(movementValue);
      /*
      if (step_counter == STEPS_TO_CALIBRATE) {
        autoCalibrate(1);
      }
      */
      step_counter++;
      step_best_calibrate++;
      readSensors();
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
      calibrateAngle(sensorTL, 1, sensorTR, 3, 17);
      //updateSensorData();
      break;
    }
    case 'T':
    case 't':
    {
      //(movementValue == 0) ? testMotors(0) : testMotors(movementValue);
      autoCalibrate(1);
      break;
    }
    case 'B':
    case 'b':
    {
      calibrateWithFront();
      break;
    }
    case 'N':
    case 'n':
    {
      calibrateWithLeft();
      break;
    }
    case 'M':
    case 'm':
    {
      calibrateWithRight();
      break;
    }
    case 'Q':
    case 'q':
    {
      startCalibrate();
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
  //delay(500);
  //turnRight(180);
  //delay(500);
  //moveForward(10);
  //delay(500);
  //turnRight(90);

  //delay(500);
}

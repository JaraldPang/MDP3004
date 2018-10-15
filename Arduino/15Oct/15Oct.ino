#include <DualVNH5019MotorShield.h>
#include <EnableInterrupt.h>
#include <RunningMedian.h>
#include <SharpIR.h>


/**
 * Everything About Sensor
 *
 * MDP Board Pin <> Arduino Pin <> Sensor Range <> Model <> Location
 * Top Sensor
 * PS1 <> A0  <> SE5  Distance <= 85          (10801)    TLeft
 * PS3 <> A2  <> SE7  Distance <= 85          (10802)    TMiddle
 * PS5 <> A4  <> SE2  Distance <= 85          (10803)    TRight Kel 2
 *
 * Bottom Sensor
 * PS2 <> A1  <> SE3 Distance <=80            (10804)    BRight(Front)
 * PS4 <> A3  <> SE1 Distance 30 <= x <= 130  (201505)   BRight(Back)
 * PS6 <> A5  <> SE6 Distance <=85            (10806)    BLeft(Front) Kel 1
 *
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

SharpIR sensorTL(irTL, 10801);
SharpIR sensorTM(irTM, 10802);
SharpIR sensorTR(irTR, 10803);

SharpIR sensorBRT(irBRT, 10804);
SharpIR sensorBRB(irBRB, 201505);
SharpIR sensorBLT(irBLT, 10806);

double distTL = 0.0, distTM = 0.0, distTR = 0.0, distBLT = 0.0, distBRT = 0.0, distBRB = 0.0;

#define MIN_RANGE_OF_SHORT_SENSOR 1
#define MAX_RANGE_OF_SHORT_SENSOR 4

#define MIN_RANGE_OF_LONG_SENSOR 3
#define MAX_RANGE_OF_LONG_SENSOR 9

#define SHORT_OFFSET 10
#define LONG_OFFSET 20

#define WALL_GAP 10
#define WALL_MIN_TOL 0.25
#define WALL_MAX_TOL 3
#define ANGLE_TOL 0.1

//position calibration variables
#define STEPS_TO_CALIBRATE 5

int step_counter = 0;
bool calibration_state = false;
bool calibration_angle = false;
bool calibrated = false;
bool recalibrate = false;
bool fastest_path = false;

#define REPLY_AlAn_OK 1
#define REPLY_An_Echo 2

/*
 *  Pololu Dual VNH5019 Motor Driver Shield Variables
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
#define Speed_Calibration_Angle 60

//Fastest path speed
#define Speed_Move_Fastest 375

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

String robotRead;
bool newData = false, isStarted = false;
bool robotReady = false;

/*
 *  Pololu Dual VNH5019 Motor Driver Shield Encoder Methods
 */

//E1
void showEncode1() {
  encoderLeftCounter++;
}
//E2
void showEncode2() {
  encoderRightCounter++;
}

/*
 *  Pololu Dual VNH5019 Motor Driver Shield Functions & Robot Calibrations
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
  Set_Speed = (fastest_path == true) ? Speed_Move_Fastest : Set_Speed;

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
    //moveRight_sil(1);
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
    //moveRight_sil(2);
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
    //moveRight_sil(2);
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

  if (calibration_state != true) {
    replyFx(REPLY_AlAn_OK);
  }
  if (fastest_path == true) {
    delay(250);
    replyFx(REPLY_AlAn_OK);
  }
}

void moveReverse(double cm) {

  double pid;
  int targetTick;
  int Set_Speed = (calibration_state == true) ? Speed_Calibration : Speed_Move;
  Set_Speed = (fastest_path == true) ? Speed_Move_Fastest : Set_Speed;

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

  if (calibration_state != true) {
    replyFx(REPLY_AlAn_OK);
  }
  if (fastest_path == true) {
    delay(250);
    replyFx(REPLY_AlAn_OK);
  }
}

void moveLeft(double deg) {

  double pid;
  float targetTick;
  int Set_Speed = (calibration_angle == true) ? Speed_Calibration_Angle : Speed_Spin;

  integral = 0;
  encoderLeftCounter = encoderRightCounter = prevTick = 0;

  /*
    if (deg <= 90) targetTick = deg * 4.39; //4.523
    else if (deg <= 180 ) targetTick = deg * 4.62;
    else if (deg <= 360 ) targetTick = deg * 4.675;
    else targetTick = deg * 4.65;
    */
  if (deg <= 90) targetTick = deg * 4.27;//4.17(test)//4.095(on maze)//4.0935;//4.0925;//4.09L;//4.085L;//4.08L;//4.0775L;
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

  if (calibration_state != true) {
    replyFx(REPLY_AlAn_OK);
  }
  if (fastest_path == true) {
    delay(250);
    replyFx(REPLY_AlAn_OK);
  }
}

void moveRight(double deg) {

  double pid;
  float targetTick;
  int Set_Speed = (calibration_angle == true) ? Speed_Calibration_Angle : Speed_Spin;

  integral = 0;
  encoderLeftCounter = encoderRightCounter = prevTick = 0;

  if (deg <= 90) targetTick = deg * 4.255;//4.155(on maze)//4.175M;//4.186M;//4.19M;//4.185;//4.175L;
  //4.148L;//4.15M;//4.170M;//4.175M;//4.21M;//4.205;//4.185;//4.175;
  //4.2;//4.185;//4.175L;//4.17L;
  //4.165;//4.1545L;//4.154L;//4.153L;
  //4.155M;//4.165M;//4.1655M;//4.166M;//4.167M;
  //4.168;//4.1695;//4.171M;//4.168L;//4.165L;//4.15L;//4.18M;//4.19M;//4.192M;
  //4.187L;//4.185;//4.1825;//4.175L;//4.17225L;//4.1715L;//4.17L;//4.165L;//4.1725M;
  //4.17L;//4.185M;//4.19M;//4.2;//4.22M;z//4.24M;//4.25;//4.335; //24/10/17

  else if (deg <= 180) targetTick = deg * 4.36;//4.33(test)//4.333M;//4.335M;//4.336M;//4.338M;//4.342M;//4.335;
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

  if (calibration_state != true) {
    replyFx(REPLY_AlAn_OK);
  }
  if (fastest_path == true) {
    delay(250);
    replyFx(REPLY_AlAn_OK);
  }
}

void replyFx(int category) {
  switch (category)
  {
  /*
  * Case 1 : Algo Ok, Android Ok
  * Case 2 : Android Echo
  */
  case 1 :
    Serial.println("anok");
    Serial.flush();
    Serial.println("alok");
    Serial.flush();
    break;
  case 2 :
    Serial.println("an" + robotRead);
    Serial.flush();
    break;
  case 3 :
    break;
  default :
    break;
  }
}

void calibrate_Robot_Position() {
  calibration_state = true;
  int turn = 0;
  calibrated = false;

  while (calibrated != true) {
    print_Median_SensorData();

    bool leftTooClose = distTL > 0 && distTL < (WALL_GAP - WALL_MIN_TOL);
    bool leftTooFar = distTL > (WALL_GAP + WALL_MIN_TOL) && distTL < (WALL_GAP + WALL_MAX_TOL);
    bool midTooClose = distTM > 0 && distTM < (WALL_GAP - WALL_MIN_TOL);
    bool midTooFar = distTM > (WALL_GAP + WALL_MIN_TOL) && distTM < (WALL_GAP + WALL_MAX_TOL);
    bool rightTooClose = distTR > 0 && distTR < (WALL_GAP - WALL_MIN_TOL);
    bool rightTooFar = distTR > (WALL_GAP + WALL_MIN_TOL) && distTR < (WALL_GAP + WALL_MAX_TOL);

    //detects left and right, not in position
    if ((leftTooClose && rightTooClose) || (leftTooFar && rightTooFar) || (leftTooClose && rightTooFar) || (leftTooFar && rightTooClose)) {
      if (abs(distTL - distTR) > ANGLE_TOL) {
        calibrate_Robot_Angle(irTL, irTR);
        calibrateDistance(irTL);
        calibrate_Robot_Angle(irTL, irTR);
      } else {
        calibrateDistance(irTL);
        calibrate_Robot_Angle(irTL, irTR);
      }
      calibrated = true;
      break;
    }

    //detects left and mid, not in position
    else if ((leftTooClose && midTooClose) || (leftTooFar && midTooFar) || (leftTooClose && midTooFar) || (leftTooFar && midTooClose)) {
      if (abs(distTL - distTM) > ANGLE_TOL) {
        calibrate_Robot_Angle(irTL, irTM);
        calibrateDistance(irTL);
        calibrate_Robot_Angle(irTL, irTM);
      } else {
        calibrateDistance(irTL);
        calibrate_Robot_Angle(irTL, irTM);
      }
      calibrated = true;
      break;
    }

    //detects mid and right, not in position
    else if ((leftTooClose && midTooClose) || (leftTooFar && midTooFar) || (leftTooClose && midTooFar) || (leftTooFar && midTooClose)) {
      if (abs(distTM - distTR) > ANGLE_TOL) {
        calibrate_Robot_Angle(irTM, irTR);
        calibrateDistance(irTM);
        calibrate_Robot_Angle(irTM, irTR);
      } else {
        calibrateDistance(irTM);
        calibrate_Robot_Angle(irTM, irTR);
      }
      calibrated = true;
      break;
    }

    //detects only left, not in position
    else if (leftTooClose || leftTooFar) {
      calibrateDistance(irTL);
      calibrated = true;
      break;
    }

    //detects only mid, not in position
    else if (midTooClose || midTooFar) {
      calibrateDistance(irTM);
      calibrated = true;
      break;
    }

    //detects only right, not in position
    else if (rightTooClose || rightTooFar) {
      calibrateDistance(irTR);
      calibrated = true;
      break;
    }

    //doesn't detect, assume nothing to calibrate to
    else {
      if (turn == 1) {
        moveLeft(90);
        delay(500);
        break;
      } else {
        turn++;
        moveRight(90);
        delay(500);
      }
    }
  }

  Serial.println("alok");
  Serial.flush();
  calibration_state = false;
}

void calibrate_Robot_Angle(int tpinL, int tpinR) {
  calibration_angle = true;
  double distL;
  double distR;
  double diff;

  for (int i = 0; i < 5; i++) {
    distL = final_MedianRead(tpinL);
    distR = final_MedianRead(tpinR);
    diff = abs(distL - distR);
    if (diff < ANGLE_TOL) {
      break;
    }
    if (distL > distR) {
      moveRight(diff);
    }
    else if (distR > distL) {
      moveLeft(diff);
    }
  }
  delay(200);
  calibration_angle = false;
}

void calibrateDistance(int tpin) {
  //use only one of the 3 front sensors
  double dist;
  for (int i = 0; i < 2; i++) {
    dist = final_MedianRead(tpin);
    if (dist < WALL_GAP) {
      moveReverse(WALL_GAP - dist);
    }
    else if (dist > WALL_GAP) {
      moveForward(dist - WALL_GAP);
    }
  }
  delay(200);
}

/*
 *  Sharp IR Functions & Grids Processing
 */

void flush_SensorData() {
  distTL = 0.0; distTM = 0.0; distTR = 0.0; distBLT = 0.0; distBRT = 0.0; distBRB = 0.0;
}

double final_MedianRead(int tpin) {
  double x[9];

  for (int i = 0; i < 9; i ++) {
    x[i] = evaluate_Distance(tpin);
  }

  insertion_Sort(x, 9);

  return x[4];
}

void insertion_Sort(double array[], int length) {
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

double evaluate_Distance(int pin) {
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

void print_Median_SensorData_Grids() {
  int i;
  String output = "";

  // Flush variable
  flush_SensorData();

  // Read Median Distance
  distTL = final_MedianRead(irTL);
  distTM = final_MedianRead(irTM);
  distTR = final_MedianRead(irTR);
  distBLT = final_MedianRead(irBLT);
  distBRT = final_MedianRead(irBRT);
  distBRB = final_MedianRead(irBRB);

  // obstacle_GridConversation usage instruction
  // obstacle_GridConversation(distance_from_sensor, category)
  int posTL = obstacle_GridConversation(distTL, 1);
  int posTM = obstacle_GridConversation(distTM, 1);
  int posTR = obstacle_GridConversation(distTR, 1);
  int posBLT = obstacle_GridConversation(distBLT, 2);
  int posBRT = obstacle_GridConversation(distBRT, 2);
  int posBRB = obstacle_GridConversation(distBRB, 0);

  // Concatenate all position into a string and send
  output += String(posTL);  output += ",";
  output += String(posTM);  output += ",";
  output += String(posTR);  output += ",";
  output += String(posBRT); output += ",";
  output += String(posBRB); output += ",";
  output += String(posBLT);

  // Output to Serial
  if (calibration_state == false) {
    Serial.println("alsensor" + output);
    Serial.flush();
  }
}

void print_Median_SensorData() {

  String output = "";

  // Flush variable
  flush_SensorData();

  // Read Median Distance
  distTL = evaluate_Distance(irTL);
  distTM = evaluate_Distance(irTM);
  distTR = evaluate_Distance(irTR);
  distBLT = evaluate_Distance(irBLT);
  distBRT = evaluate_Distance(irBRT);
  distBRB = evaluate_Distance(irBRB);

  // Concatenate all position into a string and send
  output += String(distTL);  output += ",";
  output += String(distTM);  output += ",";
  output += String(distTR);  output += ",";
  output += String(distBRT); output += ",";
  output += String(distBRB); output += ",";
  output += String(distBLT);
  Serial.println(output);

  // Output to Serial
  if (calibration_state == false) {
    Serial.println("alsensor" + output);
    Serial.flush();
  }
}

int obstacle_GridConversation(double sensor_data, int sensor_category) {
  int temp_value = 0;

  // Round Up value by first dividing then rounding up. Lastly return to value in terms of 10.
  sensor_data /= 10; sensor_data = round(sensor_data); sensor_data *= 10;

  // Front Sensor
  if (sensor_category == 1) {
    // Remove Wall. Convert to Grids.
    temp_value = (sensor_data - SHORT_OFFSET) / 10;
    // Next To Imaginary, return 0
    if (temp_value < 0){
      return MAX_RANGE_OF_SHORT_SENSOR;
    }
    else if ((temp_value < MIN_RANGE_OF_SHORT_SENSOR)) {
      return temp_value;
    }
    // Within Range, return Grids.
    else if ((temp_value >= MIN_RANGE_OF_SHORT_SENSOR) &&
             (temp_value <= MAX_RANGE_OF_SHORT_SENSOR)) {
      return temp_value;
    }
    else {
      // Over Range, return Max Value
      return MAX_RANGE_OF_SHORT_SENSOR;
    }
  }
  // Side Sensor
  else if (sensor_category == 2) {
    temp_value = (sensor_data - SHORT_OFFSET) / 10;
    if (temp_value < 0){
      return MAX_RANGE_OF_SHORT_SENSOR;
    }
    else if ((temp_value < MIN_RANGE_OF_SHORT_SENSOR)) {
      return temp_value;
    }
    else if ((temp_value >= MIN_RANGE_OF_SHORT_SENSOR) &&
             (temp_value <= MAX_RANGE_OF_SHORT_SENSOR)) {
      return temp_value;
    }
    else {
      return MAX_RANGE_OF_SHORT_SENSOR;
    }
  }
  // Long Sensor
  else {
    // Convert to Grids.
    temp_value = (sensor_data) / 10;
    // Less than Minimum Range, return -1. Minimum Range : 30cm (2Grids)
    if ((temp_value < MIN_RANGE_OF_LONG_SENSOR)) {
      return -1;
    }
    // Within Range, return Grids with Wall Gaps
    else if ((temp_value >= MIN_RANGE_OF_LONG_SENSOR) &&
             (temp_value <= MAX_RANGE_OF_LONG_SENSOR)) {
      return (temp_value - (WALL_GAP / 10));
    }
    else {
      // Over Range, return Max Value
      return MAX_RANGE_OF_LONG_SENSOR;
    }

  }
}

/*
 *  Arduino Serial & Data Processing
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

/*
 *  Arduino Default Function
 */

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

  if (newData) {
    double movementValue = getValue(robotRead, ';', 1).toFloat();
    char condition = robotRead.charAt(0);

    switch (condition) {
    case 'W':
    case 'w':
    {
      step_counter++;
      (movementValue == 0) ? moveForward(10) : moveForward(movementValue);
      break;
    }
    case 'A':
    case 'a':
    {
      step_counter++;
      (movementValue == 0) ? moveLeft(90) : moveLeft(movementValue);
      break;
    }
    case 'S':
    case 's':
    {
      step_counter++;
      (movementValue == 0) ? moveReverse(10) : moveReverse(movementValue);
      break;
    }
    case 'D':
    case 'd':
    {
      step_counter++;
      (movementValue == 0) ? moveRight(90) : moveRight(movementValue);
      break;
    }
    case 'G':
    case 'g':
    {
      replyFx(REPLY_An_Echo);
      print_Median_SensorData_Grids();
      break;
    }
    case 'Z':
    case 'z':
    {
      replyFx(REPLY_An_Echo);
      print_Median_SensorData();
      print_Median_SensorData_Grids();
      break;
    }
    case 'X':
    case 'x': {
      fastest_path = true;
      // Serial.println("alok");
      Serial.flush();
      break;
    }
    case 'C':
    case 'c':
    {
      replyFx(REPLY_An_Echo);
      calibrate_Robot_Position();
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
}

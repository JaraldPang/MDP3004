#include <DualVNH5019MotorShield.h>
#include <EnableInterrupt.h>
#include <RunningMedian.h>


/**
 * Everything Sensor
 */

/**
 * Sensor Variables Declaration
 */

#define pinSensor0 0
#define pinSensor1 1
#define pinSensor2 2
#define pinSensor3 3
#define pinSensor4 4
#define pinSensor5 5

// 10-80cm (Short)
#define MODEL_SHORT 1080
// 20-150cm (Long)
#define MODEL_LONG 20150

long ps1, ps2, ps4;
double sensorVal1, sensorVal2, sensorVal3;
float voltage1, voltage2, voltage3, dis1, dis2, dis4;

/**
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
long prevTick;
long prevMillis = 0;
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
  kd = 1;

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
      md.setSpeeds(-((SPEED_MOVE * 0.7) - pid), SPEED_MOVE + pid);
    }

    while (encoderLeftCounter < targetTick - 25) {
      pid = computePID();
      md.setSpeeds(-((0.8 * SPEED_MOVE) - pid), (0.85 * SPEED_MOVE) + pid);
    }
    while (encoderLeftCounter < targetTick - 15) {
      pid = computePID();
      md.setSpeeds(-((0.6 * SPEED_MOVE) - pid), (0.65 * SPEED_MOVE) + pid);
    }
    while (encoderLeftCounter < targetTick) {
      pid = computePID();
      md.setSpeeds(-((0.5 * SPEED_MOVE) - pid), (0.55 * SPEED_MOVE) + pid);
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
    md.setSpeeds(-((0.5 * SPEED_MOVE) - pid), -((0.4 * SPEED_MOVE) + pid));
  }
  while (encoderLeftCounter < targetTick  - 50) {
    pid = computePID();
    md.setSpeeds(-((SPEED_MOVE) - pid), -(SPEED_MOVE + pid));
  }
  while (encoderLeftCounter < targetTick) {
    pid = computePID();
    md.setSpeeds(-((0.5 * SPEED_MOVE) - pid), -((0.4 * SPEED_MOVE) + pid));
  }

  md.setBrakes(400, 400);
  delay(100);
  Serial.print("OK\r\n");
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
  moveBack(20);
  delay(2000);
}

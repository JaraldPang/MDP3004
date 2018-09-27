#include <EnableInterrupt.h>
#include <DualVNH5019MotorShield.h>

//motors pin
const int M1A = 3; //Right
const int M1B = 5;
const int M2A = 11; //Left
const int M2B = 13;

volatile long M1Ticks, M2Ticks;
long PrevTicks;
const int MSpeed = 350;
bool activate = true;
char commands;

const int Distance_10CM = 278;
const int Distance_20CM = 558;
const int d180 = 5004;
const int Rotate_90deg = 377;

DualVNH5019MotorShield md;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("Motor Test");
  enableInterrupt(M1A, encoder1, RISING);
  enableInterrupt(M2B, encoder2, RISING);
  md.init();
}

void loop() {
  // put your main code here, to run repeatedly:
  moveFront(d180);
  while(activate){
    while (Serial.available() > 0){
      commands = Serial.read();
      delay(100);
      switch(commands){
      case 'w': moveFront(Distance_10CM); break;
      case 's': moveReverse(Distance_10CM); break;
      case 'a': moveFront(d180);
      case 'f': activate = false; break;
      default: break;
      }
    }
  }
  while(!activate){
    md.setBrakes(0, 0);
  }
}

void encoder1(){
  M1Ticks++;
}

void encoder2(){
  M2Ticks++;
}

void brake(){
  md.setBrakes(400, 400);
}

void moveFront(double distance){
  double pid;
  M1Ticks = 0;
  M2Ticks = 0;

  while (M1Ticks < distance) {
    pid = computePID();
    int M1setSpeed = (MSpeed - pid);
    int M2setSpeed = (MSpeed + pid);
    //md.setSpeeds (M1setSpeed,M2setSpeed);
    md.setSpeeds (0,400);
    Serial.println("M1setSpeed: " + String(int(M1setSpeed)) + ", M2setSpeed: " + String(int(M2setSpeed)));
  }
  brake();
  delay(100);
}

void moveReverse(double distance){
  double pid;
  M1Ticks = 0;
  M2Ticks = 0;

  while (M1Ticks < distance) {
    pid = computePID();
    int M1setSpeed = -(MSpeed - pid);
    int M2setSpeed = -(-MSpeed + pid);
    md.setSpeeds (M1setSpeed,M2setSpeed);
    Serial.println("M1setSpeed: " + String(int(M1setSpeed)) + ", M2setSpeed: " + String(int(M2setSpeed)));
  }
  brake();
  delay(100);
}

double computePID() {
  Serial.println("[PID] M1Ticks(" + String(M1Ticks) + ") - M2Ticks(" + String(M2Ticks) + ") = " + String(M1Ticks-M2Ticks));
  double kp, ki, kd, p, i, d, pid, error, integral;
 //M1 Left, M2 Right
  kp = 40;
  ki = 0;
  kd = 0;

  error = M1Ticks - M2Ticks;
  integral += error;
  p = kp * error;
  i = ki*integral;
  d = kd * (PrevTicks - M1Ticks);
  pid = p + i + d;
  Serial.println("PID: " + String(pid));
  
  return pid;
}

// Motor Test Modules
// Carried Out By Kelvin 
// 9th Sep 2018 

#include "DualVNH5019MotorShield.h"
#include "EnableInterrupt.h"

DualVNH5019MotorShield md;

#define M1A 3
#define M1B 5   //Faulty
#define M2A 11
#define M2B 13

unsigned long pulse_duration;
volatile long encoderLeftCounter,encoderRightCounter;
bool change1=false,change2=false;
long prevMillis = 0;
unsigned long rpm;

void showEncode1(){
    change1=true;
    encoderLeftCounter++;
    enableInterrupt(M1A, showEncode1, FALLING);
}
void showEncode2(){
    change2=true;
    encoderRightCounter++;
    enableInterrupt(M2A, showEncode2, FALLING);
}

void setup()
{
  Serial.begin(115200);
  md.init();
  
  // pinMode(M1A,INPUT);
  pinMode(M2A,INPUT);
  // enableInterrupt(M1A, showEncode1, FALLING);
  enableInterrupt(M2A, showEncode2, FALLING);
}

double calc_rpm(double pulse_width_t) {
  double encode_rpm = (2*pulse_width_t)/562.25;
  return encode_rpm;
}

void loop()
{
  // loops endlessly; main loop goes here
  // the following code is a simple example:
  //if(millis()-prevMillis==1000) {
  //   disableInterrupt(M1A);
  //   rpm = encoderLeftCounter*30;
  //   Serial.println(rpm);
  //   encoderLeftCounter = 0;
  //   prevMillis = millis();
  //   enableInterrupt(M1A,showEncode1, FALLING);
  //}
  
  
  //md.setM1Speed(400);
  md.setM2Speed(50); // single-channel motor full-speed "forward"
  pulse_duration = pulseIn(M2A, HIGH);
  delay(2000); // wait for 2 seconds
  Serial.println(calc_rpm(pulse_duration));
  //md.setM1Speed(0);
  md.setM2Speed(0); // single-channel motor stop (coast)
  delay(500); // wait for 0.5 s
  //md.setM1Speed(400);
  md.setM2Speed(-400); // single-channel motor full-speed "reverse"
  delay(2000); // wait for 2 seconds
  //md.setM1Speed(0);
  md.setM2Speed(0); // single-channel motor stop (coast)
  delay(500); // wait for 0.5 s
}

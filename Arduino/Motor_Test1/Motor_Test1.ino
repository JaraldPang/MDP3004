#include "DualVNH5019MotorShield.h"
#include "RunningMedian.h"

DualVNH5019MotorShield md;
RunningMedian samples = RunningMedian(100);

double calc_rpm(double pulse) {
  double encoder_rpm = (562.25 * 2 * pulse)/1000000; //1 revolution in seconds
  encoder_rpm = 60/encoder_rpm; //
  return encoder_rpm;
}

unsigned long pulseWidth;

void setup()
{
  Serial.begin(115200);
  Serial.println("Dual VNH5019 Motor Shield");
  md.init();
  
  //Set PIN 3 (connected to motor encoder output A) to input
  //pinMode (3,INPUT); //Left wheel M2

  //Set PIN 11 (connected to motor encoder output A) to input
   //pinMode (13,INPUT);  //Right wheel M1

  //Pin5 RIGHT
  
  //Pin13 LEFT
  //md.setM2Speed(50);
}

void loop()
{
  double y1 , y2;
  /*if(millis()%10000>5000) {
    md.setM1Speed(250);
    md.setM2Speed(250);
  }
  else {
    md.setM1Speed(300);
    md.setM2Speed(300);
  }*/
  if(millis()>0) {
    md.setM1Speed(400); //Right 11
    md.setM2Speed(0); //Left 3
    Serial.print("400");
    Serial.print("\t");
  }
 
  //Counts the duration(microseconds) when input pulse in PIN 5 goes from HIGH to HIGH
  pulseWidth = pulseIn(11,HIGH); //3 left, 11 right
  //Serial.print(pulseWidth);
  y1=calc_rpm(pulseWidth);
  //pulseWidthL = pulseIn(13,HIGH);
  //y2=calc_rpm(pulseWidthL);
  //Display the duration(microseconds)
  //Serial.println("-------RIGHT--------");
  //Serial.print("Pin5 ");
  //Serial.println(pulseWidthR);
  //Serial.print("RPM ");
  //Serial.println(y1);
  test1(y1);
  //test1(pulseWidth);
  
  //Counts the duration(microseconds) when input pulse in PIN 13 goes from HIGH to HIGH
  //pulseWidthL = pulseIn(13,HIGH); 
  
  //Serial.print("Pulse ms:");
  //Serial.println(pulseWidthL);
  //test1(pulseWidthL);
  /*
  y2=calc_rpm(pulseWidthL);
  //Display the duration(microseconds)
  Serial.println("-------LEFT---------");
  Serial.print("Pin13 ");
  Serial.println(pulseWidthL);
  Serial.print("RPM ");
  Serial.println(y2);
  test1(y2);
  */
}

void test1(double x)
{ 
  samples.add(x);
  //double l = samples.getLowest();
  double m = samples.getMedian();
  //double a = samples.getAverage();
  //double h = samples.getHighest();
  
  //Serial.print(millis());
  //Serial.print("\t");  
  //Serial.print(x);
  //Serial.print("\t");
  //Serial.print(l);
  //Serial.print("\t");
  //Serial.print(a);
  //Serial.print("\t");
  //Serial.println(h);
  Serial.println(m);
  //delay(100);
}

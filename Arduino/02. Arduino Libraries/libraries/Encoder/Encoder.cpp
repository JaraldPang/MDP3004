#include "Arduino.h"
#include "Encoder.h"

static int countAnt1 = 0;
static int countAnt2 = 0;
static unsigned long lastTime1 = 0;
static unsigned long lastTime2 = 0;

Encoder::Encoder(unsigned char pinA, unsigned char pinB, unsigned char pinC, unsigned char pinD){
	pinA1 = pinA;
	pinB1 = pinB;
	pinA2 = pinC;
	pinB2 = pinD;
}

void Encoder::init(){
	pinMode(pinA1, INPUT);
	pinMode(pinB1, INPUT);
	pinMode(pinA2, INPUT);
	pinMode(pinB2, INPUT);
	count1 = 0;
	count2 = 0;
}

double Encoder::getMotor1RPM(unsigned long time1){
//	int speed = (count1 - countAnt1) / (time1 - lastTime1);4
	double speed = getMotor1Revs() / ((time1 - lastTime1) / 60000.0);
	lastTime1 = time1;
	return speed;
}

double Encoder::getMotor2RPM(unsigned long time2){
	double speed = getMotor2Revs() / ((time2 - lastTime2) / 60000.0);
	lastTime2 = time2;
	return speed;
}

double Encoder::getMotor1Revs(){
	int res = countAnt1 - count1;
	countAnt1 = count1;
	return res/562.215;
}

double Encoder::getMotor2Revs(){
	int res = count2 - countAnt2
	;
	countAnt2 = count2;
	return res/562.215;
}

void Encoder::rencoder1()  {
  if (digitalRead(pinB1) == HIGH)
    count1++; 
  else
    count1--;
}

void Encoder::rencoder2()  {
  if (digitalRead(pinB2) == HIGH)
    count2++; 
  else
    count2--;
}

int Encoder::getCount1(){
	return count1;
}

int Encoder::getCount2(){
	return count2;
}
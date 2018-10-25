#ifndef Encoder_h
#define Encoder_h

#include "Arduino.h"

class Encoder
{
	private:
		unsigned char pinA1;
		unsigned char pinB1;
		unsigned char pinA2;
		unsigned char pinB2;
		volatile unsigned int count1;
		volatile unsigned int count2;
		
	public:
		//constructor
		Encoder(unsigned char, unsigned char, unsigned char, unsigned char);
	
		void init();
		void rencoder1();
		void rencoder2();
		double getMotor1RPM(unsigned long);
		double getMotor2RPM(unsigned long);
		double getMotor1Revs();
		double getMotor2Revs();
		int getCount1();
		int getCount2();
};

#endif
/**
 * Name: IRRanger
 * Author: Nick Lamprianidis (lampnick67@yahoo.com)
 * Version: 2.0
 * Description: An Arduino library for interfacing the Sharp IR range finders
 * License: Copyright (c) 2013 Nick Lamprianidis 
 *          This library is licensed under the MIT license
 *          http://www.opensource.org/licenses/mit-license.php
 *
 * Filename: IRRanger.cpp
 * File description: Implementation of methods for the IRRanger library
 *
 * Version:
 * 1.0 - Support for the following Sharp IR rangers { GP2D120XJ00F, GP2Y0A21YK, GP2Y0A02YK0F }
 * 2.0 - Parameters of the model of the sensor were made data members of the class
 *       Added setModel method to allow for update of the parameters of the model
 */

#include "IRRanger.h"

// Initializes an IRRanger instance with input the sensor's part code
// and the analog pin that the sensor is connected to
// Optional input, a value (*) to be added on every measurement
// * Valid only upon requesting a measurement in cm
IRRanger::IRRanger(ir_ranger code, uint8_t pin, double p_ref)
{
	_code = code;
	_pin = pin;
	_a_ref = 5.0;
	_p_ref = p_ref;
	_repeat = 1;
	_threshold = 0.0;
	initModel(code);
}

// Initializes the parameters of the model of the sensor
inline void IRRanger::initModel(ir_ranger code)
{
	switch(_code)
	{
		case GP2D120XJ00F:
			_model_t = EXPS;
			_a = 75.59;
			_b = -3.877;
			_c = 21.31;
			_d = -0.6346;
			_e = 0;
			_f = 0;
			break;
		case GP2Y0A21YK:
			_model_t = EXPS;
			_a = 223.9;
			_b = -4.307;
			_c = 49.65;
			_d = -0.7061;
			_e = 0;
			_f = 0;
			break;
		case GP2Y0A02YK0F:
			_model_t = POLY5;
			_a = -15.01;
			_b = 136.9;
			_c = -495;
			_d = 896.9;
			_e = -846.8;
			_f = 385.7;
			break;
	}
}

// Updates the model of the sensor
void IRRanger::setModel(model_type model_t, double a, double b, double c, double d, double e, double f)
{
	_model_t = model_t;
	_a = a;
	_b = b;
	_c = c;
	_d = d;
	_e = e;
	_f = f;
}

// Sets and utilizes the voltage level on the AREF pin
// Read this http://arduino.cc/en/Reference/AnalogReference before using the AREF pin
void IRRanger::setARef(double a_ref)
{
	analogReference(EXTERNAL);
	_a_ref = a_ref;
}

// Resets the reference to the default internally generated voltage level
// Use with Caution: Possible damage of the microcontroller
// Read this http://arduino.cc/en/Reference/AnalogReference
void IRRanger::resetARef()
{
	analogReference(DEFAULT);
	_a_ref = 5.0;
}

// Sets a value to be added on every measurement in cm
// Use: If you want to find the distance of an object, not from the sensor,
//      but from another point of reference (perhaps the center of a robot),
//      then you need state the (signed) distance of the sensor from that point of interest
void IRRanger::setPRef(double p_ref)
{
	_p_ref = p_ref;
}

// Sets the number of measurements to be averaged per request for the sensor's output
// Use: Don't overdo it. Every additional measurement will take NEXT_MEAS_DELAY ms (default is 50)
void IRRanger::setRepeat(uint8_t repeat)
{
	_repeat = repeat;
}

// Sets a threshold (in cm) for identifying whether an object is close or far
void IRRanger::setThreshold(double threshold)
{
	_threshold = threshold;
}

// Returns the raw value from the A/D converter
// If _repeat was set to > 1, an average of #(_repeat) measurements will be returned
unsigned int IRRanger::getRaw()
{
	if (_repeat == 1)
		return analogRead(_pin);

	uint16_t sum = 0;
	for (uint8_t i = 0; i < _repeat; ++i)
	{
		sum += analogRead(_pin);
		delay(NEXT_MEAS_DELAY);
	}
	return sum / _repeat;
}

// Returns the output voltage of the sensor
// Of course, this value is the one after discretization
double IRRanger::getV()
{
	int value = getRaw();
	return _a_ref * ( value / AD_MAX_VALUE );
}

// Returns the distance (in cm) of an object from the sensor, or if _p_ref was set
// this distance will be from the point of reference (_p_ref adds upon the immediate result)
double IRRanger::getCM()
{
	double v = getV();
	return v2CM(v);
}

// Same as getCM, but now the unit of length is inches
double IRRanger::getIN()
{
	return getCM() * 0.393701;
}

// Translates a raw value to cm
// Use: In conjunction with getRaw, to avoid adding _p_ref to the measurement
double IRRanger::raw2CM(int value)
{
	double v = _a_ref * ( value / AD_MAX_VALUE );
	double tmp = _p_ref;
	_p_ref = 0;
	double v_cm = v2CM(v);
	_p_ref = tmp;
	return v_cm;
}

// Translates a raw value to inches
// Use: See raw2CM
double IRRanger::raw2IN(int value)
{
	return raw2CM(value) * 0.393701;
}

// Translates volts to cm
double IRRanger::v2CM(double v)
{
	switch(_model_t)
	{
		case EXPS:
			return model_exps(v);
		case POLY5:
			return model_poly5(v);
	}
}

// It represents the model of the sensor
// Returns the distance in cm given volts as input
inline double IRRanger::model_exps(double v)
{  // a * exp(b * x) + c * exp(d * x)
	return ( _a * exp( _b * v ) + _c * exp( _d * v ) ) + _p_ref;
}

// It represents the model of the sensor
// Returns the distance in cm given volts as input
inline double IRRanger::model_poly5(double v)
{  // a * x^5 + b * x^4 + c * x^3 + d * x^2 + e * x + f
	return ( v * ( v * ( v * ( v * ( _a * v + _b ) + _c ) + _d ) + _e ) + _f ) + _p_ref;
}

// Checks whether an object is close (distance <= threshold)
boolean IRRanger::isClose()
{
	return getCM() > _threshold ? false : true;
}

// Checks whether an object is far (distance > threshold)
boolean IRRanger::isFar()
{
	return getCM() > _threshold ? true : false;
}

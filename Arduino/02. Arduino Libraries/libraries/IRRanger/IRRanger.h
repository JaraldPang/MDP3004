/**
 * Name: IRRanger
 * Author: Nick Lamprianidis (lampnick67@yahoo.com)
 * Version: 2.0
 * Description: An Arduino library for interfacing the Sharp IR range finders
 * License: Copyright (c) 2013 Nick Lamprianidis 
 *          This library is licensed under the MIT license
 *          http://www.opensource.org/licenses/mit-license.php
 *
 * Filename: IRRanger.h
 * File description: Definitions and methods for the IRRanger library
 *
 * Version:
 * 1.0 - Support for the following Sharp IR rangers { GP2D120XJ00F, GP2Y0A21YK, GP2Y0A02YK0F }
 * 2.0 - Parameters of the model of the sensor were made data members of the class
 *       Added setModel method to allow for update of the parameters of the model
 */

#ifndef IRRanger_h
#define IRRanger_h

#include <Arduino.h>

// In case of use of repeated measurements, NEXT_MEAS_DELAY is the number of
// milliseconds between two successive measurements
#define NEXT_MEAS_DELAY 50  // This is a constraint of the sensor

// Consider the resolution of your A/D Converter, and set its max value
#define AD_MAX_VALUE 1023.  // 10bit A/D Converter

// Part code
enum ir_ranger {
                 GP2D120XJ00F,  // Valid for a range from 3cm to 30cm
                 GP2Y0A21YK,    // Valid for a range from 6cm to 80cm
                 GP2Y0A02YK0F   // Valid for a range from 15cm to 150cm
               };

// Family of models
enum model_type {
                  EXPS,  // a * exp(b * x) + c * exp(d * x)
                  POLY5  // a * x^5 + b * x^4 + c * x^3 + d * x^2 + e * x + f
                };

// Class for interfacing the Sharp IR range finders
class IRRanger
{
public:
    IRRanger(ir_ranger code, uint8_t pin, double p_ref = 0);
    void setModel(model_type model_t, double a, double b, double c = 0, double d = 0, double e = 0, double f = 0);
    void setARef(double a_ref);
    void resetARef();
    void setPRef(double p_ref);
    void setRepeat(uint8_t repeat);
    void setThreshold(double threshold);
    unsigned int getRaw();
    double getV();
    double getCM();
    double getIN();
    double raw2CM(int value);
    double raw2IN(int value);
    boolean isClose();
    boolean isFar();

private:
    ir_ranger _code;    // Sensor's part code
    int _pin;           // Input pin
    double _a_ref;      // Voltage reference level
    double _p_ref;      // Distance (in cm) of the sensor from the point of reference
                        //   for the sensor's measurements
    uint8_t _repeat;    // Number of measurements per request for the sensor's output
    double _threshold;  // Threshold (in cm) for identifying whether an object is close or far
    
    // Parameters of the model
    model_type _model_t;
    double _a; double _b;
    double _c; double _d;
    double _e; double _f;

    inline void initModel(ir_ranger code);
    inline double model_exps(double v);
    inline double model_poly5(double v);
    double v2CM(double v);
};

#endif // IRRanger
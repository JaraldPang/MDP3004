/*
    SharpIR
    Arduino library for retrieving distance (in cm) from the analog GP2Y0A21Y and GP2Y0A02YK
    From an original version of Dr. Marcal Casas-Cartagena (marcal.casas@gmail.com)

    Version : 1.0 : Guillaume Rico
    + Remove average and use median
    + Definition of number of sample in .h
    + Define IR pin as input
    Version : 1.1 : Thibaut Mauon
    + Add SHARP GP2Y0A710K0F for 100cm to 500cm by Thibaut Mauron
    https://github.com/guillaume-rico/SharpIR

    Original comment from Dr. Marcal Casas-Cartagena :
   The Sahrp IR sensors are cheap but somehow unreliable. I've found that when doing continous readings to a
   fix object, the distance given oscilates quite a bit from time to time. For example I had an object at
   31 cm. The readings from the sensor were mainly steady at the correct distance but eventually the distance
   given dropped down to 25 cm or even 16 cm. That's quite a bit and for some applications it is quite
   unacceptable. I checked the library http://code.google.com/p/gp2y0a21yk-library/ by Jeroen Doggen
   (jeroendoggen@gmail.com) and what the author was doing is to take a bunch of readings and give an average of them
   The present library works similary. It reads a bunch of readings (avg), it checks if the current reading
   differs a lot from the previous one (tolerance) and if it doesn't differ a lot, it takes it into account
   for the mean distance.
   The distance is calculated from a formula extracted from the graphs on the sensors datasheets
   After some tests, I think that a set of 20 to 25 readings is more than enough to get an accurate distance
   Reading 25 times and return a mean distance takes 53 ms. For my application of the sensor is fast enough.
   This library has the formulas to work with the GP2Y0A21Y and the GP2Y0A02YK sensors but exanding it for
   other sensors is easy enough.
*/

#ifdef Arduino
#include "Arduino.h"
#elif defined(SPARK)
#include "Particle.h"
#include "math.h"
#endif
#include "SharpIR.h"

// Initialisation function
//  + irPin : is obviously the pin where the IR sensor is attached
//  + sensorModel is a int to differentiate the two sensor models this library currently supports:
//    > 1080 is the int for the GP2Y0A21Y and
//    > 20150 is the int for GP2Y0A02YK and
//    > 100500 is the long for GP2Y0A710K0F
//    The numbers reflect the distance range they are designed for (in cm)
SharpIR::SharpIR(int irPin, long sensorModel) {

    _irPin = irPin;
    _model = sensorModel;

    // Define pin as Input
    pinMode (_irPin, INPUT);

#ifdef ARDUINO
    analogReference(DEFAULT);
#endif
}

// Sort an array
void SharpIR::sort(double arr[], int left, int right) {
    int i = left, j = right;
    double tmp;
    double pivot = arr[(left + right) / 2];

    /* partition */
    while (i <= j) {
        while (arr[i] < pivot)
            i++;
        while (arr[j] > pivot)
            j--;
        if (i <= j) {
            tmp = arr[i];
            arr[i] = arr[j];
            arr[j] = tmp;
            i++;
            j--;
        }
    };

    /* recursion */
    if (left < j)
        sort(arr, left, j);
    if (i < right)
        sort(arr, i, right);
}

// Read distance and compute it
double SharpIR::distance() {
    double voltage;
    double ir_val[NB_SAMPLE];
    double distanceCM;

    for (int i = 0; i < NB_SAMPLE; i++) {
        // Read analog value
        ir_val[i] = analogRead(_irPin);
    }

    // Sort it
    sort(ir_val, 0, (NB_SAMPLE - 1));


    if (_model == 10801) {
    #ifdef ARDUINO
        distanceCM = (1 / (0.035584 * (map(ir_val[NB_SAMPLE / 2], 0, 1023, 0, 5000) / 1000.0) + 0.000645)) - 0.42;
    #elif defined(SPARK)
        distanceCM = (1 / (0.035584 * (map(ir_val[NB_SAMPLE / 2], 0, 4095, 0, 5000) / 1000.0) + 0.000645)) - 0.42;
    #endif
    }
    else if (_model == 10802) {
    #ifdef ARDUINO
        distanceCM = (1 / (0.037573 * (map(ir_val[NB_SAMPLE / 2], 0, 1023, 0, 5000) / 1000.0) - 0.001921)) - 0.42;
    #elif defined(SPARK)
        distanceCM = (1 / (0.037573 * (map(ir_val[NB_SAMPLE / 2], 0, 4095, 0, 5000) / 1000.0) - 0.001921)) - 0.42;
    #endif
    }
    else if (_model == 10803) {
    #ifdef ARDUINO
        distanceCM = (1 / (0.039143 * (map(ir_val[NB_SAMPLE / 2], 0, 1023, 0, 5000) / 1000.0) - 0.002895)) - 0.42;
    #elif defined(SPARK)
        distanceCM = (1 / (0.039143  * (map(ir_val[NB_SAMPLE / 2], 0, 4095, 0, 5000) / 1000.0) - 0.002895)) - 0.42;
    #endif
    }
    else if (_model == 10804) {
    #ifdef ARDUINO
        distanceCM = (1 / (0.033861 * (map(ir_val[NB_SAMPLE / 2], 0, 1023, 0, 5000) / 1000.0) + 0.001142)) - 0.42;
    #elif defined(SPARK)
        distanceCM = (1 / (0.033861 * (map(ir_val[NB_SAMPLE / 2], 0, 4095, 0, 5000) / 1000.0) + 0.001142)) - 0.42;
    #endif
    }
    else if (_model == 201505) {
    #ifdef ARDUINO
        voltage = (map(ir_val[NB_SAMPLE / 2], 0, 1023, 0, 5000) / 1000.0);
        distanceCM = (1 / ((-0.001502 * pow(voltage,6)) + (0.017811 * pow(voltage,5)) - 
            (0.075565 * pow(voltage,4)) + (0.151922 * pow(voltage,3)) - 
            (0.155959 * pow(voltage,2)) + (0.093823 * pow(voltage,1)) - 0.007963)) - 0.42;
    #elif defined(SPARK)
        voltage = (map(ir_val[NB_SAMPLE / 2], 0, 4095, 0, 5000) / 1000.0);
        distanceCM = (1 / ((-0.001502 * pow(voltage,6)) + (0.017811 * pow(voltage,5)) - 
            (0.075565 * pow(voltage,4)) + (0.151922 * pow(voltage,3)) - 
            (0.155959 * pow(voltage,2)) + (0.093823 * pow(voltage,1)) - 0.007963)) - 0.42;
    #endif
    }
    else if (_model == 10806) {
    #ifdef ARDUINO
        distanceCM = (1 / (0.034789 * (map(ir_val[NB_SAMPLE / 2], 0, 1023, 0, 5000) / 1000.0) - 0.002085)) - 0.42;
    #elif defined(SPARK)
        distanceCM = (1 / (0.034789 * (map(ir_val[NB_SAMPLE / 2], 0, 4095, 0, 5000) / 1000.0) - 0.002085)) - 0.42;
    #endif
    }

    return distanceCM;
}
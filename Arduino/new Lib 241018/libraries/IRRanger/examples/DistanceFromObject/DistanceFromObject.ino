/**
 * A simple example showcasing the functionality of the library
 * The distance reported by the sensor, gets written on the serial port
 * and then sets the duty cycle and blinks the onboard LED
 *
 * Notes:
 * AREF: If your project uses an external voltage reference, then you should
 *       specify that reference voltage to the IRRanger instance with the setARef method
 * resetARef: Resets the reference to the default internal voltage level
 *            Please read this first -> http://arduino.cc/en/Reference/AnalogReference
 * Repeated measurements: You can set a number of successive measurements to be made
 *                        and then take their average as the final result. This might be
 *                        desirable since the sensor's output varies a bit.
 *                        Don't overdo it! For every additional measurement, a default
 *                        50ms delay is applied in order for the sensor to refresh.
 * Point of reference: If you want to find the distance of an object, not from
 *                     the sensor, but from another point of reference (perhaps the
 *                     center of your robot), then you can state the distance (in cm)
 *                     of the sensor from that point of interest either during the
 *                     initialization of an instance by supplying a third argument to
 *                     the constructor, or later with the setPRef method. From thereafter
 *                     this value will be getting added to every measurement you take.
 *                     This has a sensible meaning only when we are talking about 
 *                     units of length, and hence it will apply only when calling
 *                     methods getCM() and getIN().
 */

#include "IRRanger.h"  // Includes the IRRanger library

int LED = 13;  // Onboard LED

int PIN = A0;  // Pin for sensor's output

// Initializes a library instance with the part code of the sensor
// { GP2D120XJ00F, GP2Y0A21YK, GP2Y0A02YK0F }, the input pin, and
// a value (in cm) to be added on every measurement, as a distance
// of the sensor from a desired point of reference
IRRanger ir(GP2D120XJ00F, PIN, 5);

void setup()
{
    Serial.begin(9600);  // Initializes serial port
    // Waits for serial port to connect. Needed for Leonardo only
    while ( !Serial ) ;
  
    pinMode(LED, OUTPUT);
    
    // If you have fitted a curve for your sensor, you can set the model here
    // The available model families are:
    //   * Sum of exponentials: a * exp(b * x) + c * exp(d * x)
    //   * 5th degree polynomial: a * x^5 + b * x^4 + c * x^3 + d * x^2 + e * x + f
    // So, you can set the model to an exponential, a sum of expontials,
    // or a polynomial up to 5th degree
    // First argument to setModel is the model type { EXPS, POLY5 }
    // e.g. double a = .., b = .., c = .., d = .., e = .., f = ..;
    //      ir.setModel(EXPS, a, b, c, d);  // sum of exponentials
    // or   ir.setModel(EXPS, a, b);  // exponential
    // or   ir.setModel(POLY5, a, b, c, d, e, f);  // 5th degree polynomial
    // or   ir.setModel(POLY5, 0, 0, c, d, e, f);  // 3rd degree polynomial
}

void loop()
{
    double distance = ir.getCM();  // Gets a distance in cm
    double duty_cycle = ir.getRaw() / 1023.;  // Defines a ratio
  
    // Writes distance on the serial port
    Serial.print(distance);
    Serial.println("cm");
    
    // Blinks the onboard LED
    digitalWrite(LED, HIGH);
    delay((int)(duty_cycle * 300));
    digitalWrite(LED, LOW);
    delay((int)((1-duty_cycle) * 300));
}


/**
 * A simple example showcasing the functionality of the library
 * If an object comes closer than a known threshold,
 * the onboard LED will lit and an indication will appear on the serial port
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

// Initializes a library instance with the part code of your sensor
// { GP2D120XJ00F, GP2Y0A21YK, GP2Y0A02YK0F } and the input pin
IRRanger ir(GP2D120XJ00F, PIN);

void setup()
{
    Serial.begin(9600);  // Initializes serial port
    // Waits for serial port to connect. Needed for Leonardo only
    while ( !Serial ) ;
  
    pinMode(LED, OUTPUT);
    
    ir.setThreshold(17);  // Sets a threshold (17cm) for the distance from the sensor
}

void loop()
{
    // If an object comes closer than the threshold
    if ( ir.isClose() )
    { // Sets an alarm
        Serial.println("Alarm! Danger!");
        digitalWrite(LED,HIGH);
    }
    else
        digitalWrite(LED,LOW);
    
    delay(50);  // Waits for sensor to refresh
}


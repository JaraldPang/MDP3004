/**
 * This example is about the Sharp IR range finders
 * This example is supposed to be used in conjuction with the
 * data_acquisition.py script found here https://github.com/pAIgn10/SharpIRLib
 * As the first step, you're going to take some measurements with your sensor
 * Next, you will be utilize these data for fitting a curve for the
 * particular sensor in your possession
 *
 * Apparently not every sensor has the exact response indicated on the datasheet.
 * So, if you want a bit more accurate measurements, you can follow the instructions
 * on the data_acquisition and curve_fitting scripts to do a curve fitting on your sensor.
 *
 * Load this code on your arduino, and then go to
 * https://github.com/pAIgn10/SharpIRLib for more details
 */

#include "IRRanger.h"  // Include the IRRanger library

int PIN = A0;  // Connect the output of your sensor to A0 pin

// Initializes a library instance with the part code of your sensor
// { GP2D120XJ00F, GP2Y0A21YK, GP2Y0A02YK0F } and the input pin
IRRanger ir(GP2D120XJ00F, PIN);

void setup()
{
    Serial.begin(115200);  // Initializes serial port
    // Waits for serial port to connect. Needed for Leonardo only
    while ( !Serial ) ;
}


void loop()
{
    // If the host computer has sent a byte...
    if ( Serial.available() )
    {
        // Takes a measurement and sends it to the host computer
        Serial.read();
        Serial.println(ir.getRaw());
        delay(50);  // Waits for sensor to refresh
    }
}


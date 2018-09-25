"""
Author: Nick Lamprianidis (lampnick67@yahoo.com)
Version: 1.0
License: Copyright (c) 2013 Nick Lamprianidis 
         This library is licensed under the MIT license
         http://www.opensource.org/licenses/mit-license.php

Description:
This script takes measurements from a Sharp IR range finder, and saves them on a file
The purpose of the script is to collect data to be utilized on a curve fitting
for the particular ranger in use

There will be a serial communication between the host computer and an arduino...
  the CurveFitting example will have to be uploaded on the arduino
You can print ruler_board.pdf to help you with the positioning of the object
You will need to find an object with a flat surface, dress it with a white paper,
and use it as the obstacle for the measurements
Upon completion, a file named 'datapoints.csv' will have been created
You will use this file on the curve_fitting.{m, py} script to do the fitting of the curve
It's only a matter of taste and availability whether you will use matlab or python

Input arguments for the script:
* port      - name of the serial port
* part_code - the name of the sensor { GP2D120XJ00F, GP2Y0A21YK, GP2Y0A02YK0F }
* step      - distance (in cm) between two successive measurements (1 or 2cm is recommended)
* nmpd      - number of measurements per distance (40 is recommended)

Example: python data_acquisition.py /dev/ttyACM0 GP2D120XJ00F 1 40

The script will make #(nmpd) measurements/distance for the #((r_max - r_min) / step + 1)
different distances of the object from the sensor
The script, prior to starting the measurements, will ask you to position the obstacle
to the appropriate distance, and then press Enter
"""

from sys import argv
# For Microsoft Windows, download pySerial from http://pyserial.sourceforge.net/
from serial import *

AREF = 5.0  # Voltage reference of the A/D Converter on the arduino
AD_MAX_VALUE = 1023.  # 10bit A/D Converter

# Specifies the bounds of the working range of the sensors (in cm)
part_specs = {
                'GP2D120XJ00F' : (3, 30),
                'GP2Y0A21YK'   : (6, 80),
                'GP2Y0A02YK0F' : (15, 150)
             }

def drange(start, stop, step):
    r = start
    while r <= stop:
        yield r
        r += step


if __name__ == "__main__":
    if (len(argv) < 5): sys.exit('Error: Not enough arguments.\n')
    # To identify the name of your serial port:
    # * On Windows: Go to Computer -> System properties -> Device Manager -> Ports (COM & LPT)
    #               The name of the serial port will be 'COMx'"'
    # * On Linux  : Connect your arduino, open a terminal and type "dmesg | tail"
    #               The last entries will be about your arduino
    #               The name of the serial port will be '/dev/ttyx'
    port = argv[1]
    part_code = argv[2]
    step = float(argv[3])
    nmpd = int(argv[4])

    arduino = Serial(port, 115200)
    r_min, r_max = part_specs[part_code]
    distances = drange(r_min, r_max, step)

    with open('datapoints.csv', 'w') as f:
        f.write('cm, V\n')
        print 'You can terminate the process prematurely at any step by pressing T and Enter'
        print 'Process initiated...'
        for cm in distances:
            key = raw_input('Place the obstacle at %5.2fcm and press P and Enter to proceed ' %cm)
            if (key == 'T' or key == 't'): break

            volts = []
            for i in range(nmpd):
                arduino.write('f')
                volts.append(AREF * int(arduino.readline()) / AD_MAX_VALUE)

            volts = sorted(volts)
            v = volts[nmpd/4:3*nmpd/4+1]  # keeps the middle 50th percentile
            v = sum(v) / len(v)
            f.write('%.2f, %.2f\n' %(cm, v))
    print 'Process terminated!'
    print 'A file \'datapoints.csv\' has been created in the current directory.'
    print 'Exiting...'

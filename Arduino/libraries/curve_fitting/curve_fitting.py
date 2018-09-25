"""
Author: Nick Lamprianidis (lampnick67@yahoo.com)
Version: 1.0
License: Copyright (c) 2013 Nick Lamprianidis 
         This library is licensed under the MIT license
         http://www.opensource.org/licenses/mit-license.php

Description:
This script performs a curve fitting for a Sharp IR range finder

Arguments:
part_code - the name of the sensor { GP2D120XJ00F, GP2Y0A21YK, GP2Y0A02YK0F }
filename  - the file containing the data

Example:
python curve_fitting.py GP2D120XJ00F datapoints.csv

Note:
On Windows, you will have to download pythonxy (http://code.google.com/p/pythonxy/)
and make sure to install numpy, scipy, matplotlib
"""

from sys import argv
from numpy import array, exp, linspace
from scipy.optimize import curve_fit
from matplotlib.pyplot import *

# Specifies the models of the sensors that are derived from the datasheets
part_ds_model = {
                'GP2D120XJ00F' : lambda(v): 75.59 * exp( -3.877 * v ) + 21.31 * exp( -0.6346 * v ),
                'GP2Y0A21YK'   : lambda(v): 223.9 * exp( -4.307 * v ) + 49.65 * exp( -0.7061 * v ),
                'GP2Y0A02YK0F' : lambda(v): v * ( v * ( v * ( v * ( -15.01 * v + 136.9 ) - 495 ) + 896.9 ) - 846.8 ) + 385.7
             }

def model(x,a,b,c,d):
    """
    A model for the data. You could change this to whatever fits you
    """
    return a * exp( b * x ) + c * exp( d * x )


if __name__ == "__main__":
    if (len(argv) < 3): sys.exit('Error: Please specify a filename.\n')
    part_code, filename = argv[1], argv[2]

    # Picks the right model that comes from the datasheet
    # to compare it with the estimated one later
    fd = part_ds_model[part_code]

    # Parses the data
    cms, vs = [], []
    with open(filename, 'r') as f:
        f.readline()
        for line in f:
            cm, v = [float(i) for i in line.split(',')]
            vs.append(v)
            cms.append(cm)

    # Estimates the model
    x = array(vs)
    y = array(cms)
    params, var = curve_fit(model, x, y)
    print 'parameters = ', list(params)

    # Plots the results
    xs = linspace(0.3, 3.2, 1000)
    yd = fd(xs)  # model from the datasheet
    ys = model(xs, *params)  # estimated model
    plot(x, y, 'o', label = 'datapoints')
    plot(xs, yd, label = 'model from datasheet')
    plot(xs, ys, label = 'estimated model')
    xlabel('V [V]')
    ylabel('x [cm]')
    title('%s Transfer Function' %part_code)
    legend()
    show()
    print 'Now go back to IRRanger.h and update the model'
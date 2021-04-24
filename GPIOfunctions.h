// Miscellaneous data processing functions for MS GPIO project

#ifndef _GPIOfunctions_H    // Put these two lines at the top of your file.
#define _GPIOfunctions_H    // (Use a suitable name, usually based on the file name.)

#include "Arduino.h"

// Steinhart-Hart thermistor calibration
// Required inputs: pin to read voltage from, value of pullup resistor (assuming sensor connected to ground),
// three Steinhart-Hart model constants found from best-fit calculation.
float steinhartHart(int pin_, float R_, float c1_, float c2_, float c3_)
{
    // Read voltage of specified pin
    int Vo_ = analogRead(pin_);

    // Back-calculate resistance of thermistor from voltage and pullup resistor, R_
    float RT_ = R_ / (1023 / (float)Vo_ - 1);

    float logRT_ = log(RT_);

    // Calculate oil temperature (in Kelvin) using provided constants
    float temp_ = (1.0 / (c1_ + c2_*logRT_ + c3_*logRT_*logRT_*logRT_));

    // Convert back to Farenheit
    temp_ = temp_ - 273.15;
    temp_ = (temp_ * 9.0)/ 5.0 + 32.0; 

    return temp_;
}

// Linear interpolation function - for 0-5V or 0-12V powered sensors
// Required inputs: pin to read voltage from, two data points corresponding to top and bottom of range
// Voltage on x-axis, sensor reading on y-axis
float linearInterpolate(int pin_, float x1_, float y1_, float x2_, float y2_)
{
    // Calculate slope of line between two points
    float slope_ = (y2_ - y1_)/(x2_ - x1_);

    // Read voltage of specified pin
    int Vo_ = analogRead(pin_);

    // Interpolate (or extrapolate) the voltage reading
    float y3_ = y1_ + slope_*(Vo_ - x1_);

    return y3_;
}

// Keep this line at end of code
#endif

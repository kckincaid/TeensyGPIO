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
    temp_ = ((temp_ -273.15)*1.8) + 32.0; 

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

// Linear interpolation function type 2 - for resistive sensors with voltage divider on input pin
// Required inputs: pin to read voltage from, voltage to reading slope, voltage to reading intercept, voltage divider factor
// Voltage on x-axis, sensor reading on y-axis
float linearInterpolate(int pin_, float m_, float b_, float vd_)
{
    // Read voltage of specified pin
    int Vo_ = analogRead(pin_);

    // Calculate reading from pin voltage, slope, intercept, and voltage divider value
    float y3_ = m_*(Vo_/vd_) + b_;

    return y3_;
}

int gaugeCurrentControl(int pin_, float value_, float m_, float b_, float rshunt_, float vmin_, float vmax_)
{
    // Calculate necessary output voltage for desired constant current value
    float vout_ = ((value_*m_) + b_)*rshunt_;

    Serial.print("Calculated output voltage is: ");
    Serial.println(vout_);

    int voutint_ = vout_/3.3*256;

    Serial.print("Converted to integer (out of 256): ");
    Serial.println(voutint_);

    // Write pin output based on value and provided thresholds
    if ( vout_ < vmin_ )
    {
        analogWrite(pin_, (int)(vmin_/3.3*256));
    }
    else if ( vout_ > vmax_ )
    {
        analogWrite(pin_, (int)(vmax_/3.3*256));
    }
    else
    {
        analogWrite(pin_, (int)(voutint_));
    }

    return voutint_;
}

void warningLight(int pin_, float value_, float lowSet_, float highSet_, float freq_, bool reverse_)
{
    // Declare on and off times
    int onTime_ = 0, offTime_ = 0, status_ = 0;
    
    // Fast blink if the extreme setpoint is exceeded
    if ((value_ > highSet_ && !reverse_) || (value_ < lowSet_ && reverse_))
    {
        // If "off" interval is done, turn on for specified interval
        if (millis() > onTime_ && status_ == 0)
        {
            offTime_ = millis() + freq_/3;

            digitalWrite(pin_, HIGH);

            status_ = 1;
        }
        // If "on" interval is done, turn off for specified interval
        if (millis() > offTime_  && status_ == 1)
        {
            onTime_ = millis() + freq_/3;
          
            digitalWrite(pin_, LOW);

            status_ = 0;
        }
    }
    // Slow blink if warning value is exceeded
    if ((value_ > lowSet_ && !reverse_) || (value_ < highSet_ && reverse_))
    {
        // If "off" interval is done, turn on for specified interval
        if (millis() > onTime_ && status_ == 0)
        {
            offTime_ = millis() + freq_;

            digitalWrite(pin_, HIGH);

            status_ = 1;
        }
        // If "on" interval is done, turn off for specified interval
        if (millis() > offTime_  && status_ == 1)
        {
            onTime_ = millis() + freq_;
          
            digitalWrite(pin_, LOW);

            status_ = 0;
        }
    }
    else
    {
        digitalWrite(pin_, LOW);

        status_ = 0;
    }
}

// Keep this line at end of code
#endif

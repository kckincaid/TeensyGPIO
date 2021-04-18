# TeensyGPIO

This code is for use with a Teensy 4.1 or compatible board. It has been designed as a general purpose input/output board for automotive purposes, allowing additional sensors and outputs on top of existing ECU capabilities. CAN BUS communication with Megasquirt ECUs is the first priority, allowing for the Megasquirt to recieve additional inputs, and the Teensy to recieve real-time data from sensors interfacing with the Megasquirt.

A number of general purpose analog inputs are used; in this case, oil temperature and pressure are both read in from external sensors. Additional inputs are configurable by the user.

Provisions are made for 2x external thermocouple amplifiers (on Adafruit MAX 31856 boards) in order to read in two EGT signals.

Additionally, outputs are created to control aftermarket gauges, in order to consolidate the number of sensors on the engine. Note that these outputs are low current (<20mA) and will require an external buffer to safely interface with gauges.

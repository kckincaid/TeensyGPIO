# TeensyGPIO

This code is for use with a Teensy 4.1 or compatible board. It has been designed as a general purpose input/output board for automotive purposes, allowing additional sensors and outputs on top of existing ECU capabilities. CAN BUS communication with Megasquirt ECUs is the first priority, allowing for the Megasquirt to recieve additional inputs, and the Teensy to recieve real-time data from sensors interfacing with the Megasquirt.

A number of general purpose analog inputs are used; in this case, oil temperature, oil pressure, and fuel pressure are read in from sensors not controlled by the Megasquirt. Additional inputs are configurable by the user.

Provisions are made for 2x external thermocouple amplifiers (on Adafruit MAX 31856 boards) in order to read in two EGT signals.

Additionally, outputs are created to control aftermarket gauges, in order to consolidate the number of sensors on the engine. Most aftermarket gauges, e.g. the Autometer ones I am working with, are controlled by the amount of current flowing from the sensor terminal to ground, rather than the actual voltage of the sensor. A voltage controlled current sink circuit is used with the Teensy code and values calibrated from testing gauges to control these gauges from sensors other than those they came with. 

Some gauges may be voltage controlled, perhaps the OEM dash gauges - the capability to control these will be added as time allows.

This code relies on work by a number of others, including:
- The MegaCAN library by Mikey Antonakakis on the MSExtra forums
- The Teensy / MS dash interface developed by ToplessFC3Sman on RX7Club
- The FlexCAN_T4 library fork created by PJRC forum member tonton81
- The 
- Adafruit MAX31586 thermocouple amplifier library

Many thanks for their hard work, and the work of all others who played a part but weren't mentioned above.

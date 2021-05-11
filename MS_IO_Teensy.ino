// ------------------------------------------------------------------- //
// ----------------------- Megasquirt CAN GPIO ----------------------- //
// ----------------------- Coded for Teensy 4.1 ---------------------- //
// ----------------------- Last update: 4/24/21 ---------------------- //
// -------------------- Written by Kellis Kincaid -------------------- //
// ------------------------------------------------------------------- //

/* Much of this code has been taken from other sources: Teensy and 
 * Adafruit tutorials, RX7Club user toplessFC3Sman's Teensy project,
 * and many others. Many thanks to those who write and maintain the
 * libraries used herein.
 */

// ------------------------------------------------------------------- //
// ------------------------ Teensy 4.1 Pinout ------------------------ //
// ------------------------------------------------------------------- //

// -------------------------- Micro USB Port ------------------------- //
// GND      CAN GND   ||    Vin       Main power input
// 00 (PWM) CAN RX    ||    GND       Main ground
// 01 (PWM) CAN TX    ||    3.3V
// 02 (PWM)           ||    23 (A9)
// 03 (PWM)           ||    22 (A8)
// 04 (PWM) EGTF FLT  ||    21 (A7)
// 05 (PWM) EGTF CS   ||    20 (A6)
// 06 (PWM) EGTF SDI  ||    19 (A5)
// 07 (PWM) EGTF SDO  ||    18 (A4)
// 08 (PWM) EGTF SCK  ||    17 (A3)
// 09 (PWM)           ||    16 (A2)   Fuel pressure in (10kR pulldown)
// 10 (PWM)           ||    15 (A1)   Oil pressure in (10kR/10kR divider)
// 11 (PWM)           ||    14 (A0)   Oil temperature in (470R pullup)
// 12 (PWM)           ||    13 (LED)
// 3.3V     CAN V+    ||    GND
// 24 (A10) EGTL FLT  ||    41 (A17)
// 25 (A11) EGTL CS   ||    40 (A16)
// 26 (A12) EGTL SDI  ||    39 (A15)
// 27 (A13) EGTL SDO  ||    38 (A14)
// 28 (PWM) EGTL SCK  ||    37 (PWM)  Autometer coolant temp gauge output
// 29 (PWM)           ||    36 (PWM)  Autometer oil temp gauge output
// 30                 ||    35
// 31                 ||    34
// 32                 ||    33 (PWM)
// -------------------------- Micro SD Slot -------------------------- //

// ------------------------------------------------------------------- //
// -------------------------- Include Files -------------------------- //
// ------------------------------------------------------------------- //

//#include <circular_buffer.h>
#include <FlexCAN_T4.h>
//#include <imxrt_flexcan.h>
//#include <kinetis_flexcan.h>
#include <MegaCAN.h>
#include <SD.h>
#include <Adafruit_MAX31856.h>
#include "GPIOfunctions.h"

// Create instance of FlexCAN class using 2nd bus on Teensy
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> CANbus;

// Create instance of MegaCAN class
MegaCAN MCAN;

// Select on-board SD slot for data logging
const int chipSelect = BUILTIN_SDCARD;

// ------------------------------------------------------------------- //
// ----------------------- Variable Definitions ---------------------- //
// ------------------------------------------------------------------- //

// -------------------------- User Controls -------------------------- //
bool  debug = true;       // Output debug help?
bool  datalog = false;    // Turn on datalogging?
int   logPeriod  = 100;   // How frequently to datalog (ms)
int   navg = 10;          // Number of sensor data points to average
int   canRefresh = 1000;  // Time elapsed before CAN timeout reached (ms)

// -------------------------- CAN Messaging -------------------------- //
MegaCAN_message_t           msreq;    // Stores request for info from MS
MegaCAN_message_t           msresp;   // Stores info to be sent to MS
MegaCAN_broadcast_message_t msbroad;  // Stores info broadcast from MS
CAN_message_t               rxmsg;
CAN_message_t               rspmsg;
bool                        sendsuccess;

// ------------------------------- EGTs ------------------------------ //
uint16_t  EGTF, EGTR;
float EGTFTOT = 0, EGTRTOT = 0;
// Software SPI pin order: CS, DI, DO, CLK
Adafruit_MAX31856 EGTFBoard = Adafruit_MAX31856(5, 6, 7, 8);
Adafruit_MAX31856 EGTRBoard = Adafruit_MAX31856(25, 26, 27, 28);

// ------------------------- Oil Temperature ------------------------- //
// Read from center of 470R/sensor voltage divider, with sensor grounded
// and top of divider powered by 5V. Steinhart-Hart model used to calculate
// temperature based on readings taken from freezing and boiling water, 
// and generic middle value from DIYAutoTune website.
uint16_t  OILT;
int       oiltemppin = 14; // A0
float     oiltempres = 470; // resistor value for oil temp circuit
float     OTC1 = 1.37e-3, OTC2 = 2.52e-4, OTC3 = 6.47e-9; // Steinhart-Hart model constants
float     OILTTOT = 0;

// -------------------------- Oil Pressure --------------------------- //
// Read from center of 10k/10k voltage divider, with top connected to 
// sensor wire. Voltage varies from approx. 6V at 0 psi, to 3.5V at 90 psi,
// so a divider is required to bring voltage down to Teensy levels. Gauge
// operates based on sensor current draw, so large resistors chosen to limit
// effects on reading. See Gauge Driver google sheet for correlation.
uint16_t  OILP;
int       oilpresspin = 15; // A1
float     OPM = -0.0314, OPB = 6.46, OPVD = 0.5, OPVmin = 1.0, OPVmax = 4.0;
float     OILPTOT = 0;

// -------------------------- Fuel Pressure -------------------------- //
// 5V powered sensor with linear interpolation between min and max values.
uint16_t  FULP;
int       fuelpresspin = 16; // A2
float     FPV1 = 0.5;
float     FPP1 = 0;
float     FPV2 = 4.5;
float     FPP2 = 15;
float     FULPTOT = 0;

// ------------------- MS CAN Broadcast Variables -------------------- //
// These values are recieved from MS via CAN broadcasting. Initialize to
// bad values so it's obvious if they haven't been updated.
const uint32_t baseID = 1520;
int       CLTT = -777;

// -------------------------- Gauge Drivers -------------------------- //
// Use calculated values for coolant temp, oil temp, and oil pressure to
// drive Autometer coolant and oil temp gauges, and OEM dash coolant and
// oil pressure gauges. This reduces redundant sensors on engine.
// Autometer coolant gauge values
int       AMCLTTPIN = 37, AMCLTTout = 0;
float     AMCLTVmin = 0.66, AMCLTVmax = 4.0, AMCLTVout = 0;
float     AMCLTRshunt = 66, AMCLTm = 0.000349, AMCLTb = -0.0297;

// Autometer oil temperature gauge values
int       AMOILTPIN = 36;


// -------------------------- Miscellaneous -------------------------- //
int       Vo;
int       j = 0;
unsigned long canTime = millis();
unsigned long lastLogTime = millis();

// ------------------------------------------------------------------- //
// ---------------------------- Setup Area --------------------------- //
// ------------------------------------------------------------------- //
void setup()
{
    Serial.begin(9600);
    CANbus.begin();
    CANbus.setBaudRate(500000);

    // Need to tell FlexCAN to use extended frames
    rspmsg.flags.extended = 1;
    rspmsg.flags.remote = 0;

    Serial.print("Initializing SD card...");
  
    // see if the card is present and can be initialized:
    if (!SD.begin(chipSelect)) 
    {
        Serial.println("Card failed, or not present");
        // don't do anything more:
        return;
    }
    
    Serial.println("card initialized.");

    // Initialize EGT boards
    EGTFBoard.begin();
    EGTRBoard.begin();

    // Set thermocouple types
    EGTFBoard.setThermocoupleType(MAX31856_TCTYPE_K);
    EGTRBoard.setThermocoupleType(MAX31856_TCTYPE_K);
}

// ------------------------------------------------------------------- //
// ---------------------------- Main Loop ---------------------------- //
// ------------------------------------------------------------------- //
void loop()
{
    // Read in analog sensor inputs to running totals
    OILTTOT += steinhartHart(oiltemppin, oiltempres, OTC1, OTC2, OTC3);
    OILPTOT += linearInterpolate(oilpresspin, OPM, OPB, OPVD);
    FULPTOT += linearInterpolate(fuelpresspin, FPV1, FPP1, FPV2, FPP2);
    EGTFTOT += EGTFBoard.readThermocoupleTemperature();
    EGTRTOT += EGTRBoard.readThermocoupleTemperature();

    // Increment the sample counter
    j++;

    // Calculate average after reading the specified number of times
    if (j == navg)
    {
        // Compute average value over cycle
        OILT = OILTTOT/j;
        OILP = OILPTOT/j;
        FULP = FULPTOT/j;
        EGTF = ((EGTFTOT/j)*1.8)+32;
        EGTR = ((EGTRTOT/j)*1.8)+32;

        // Reset running totals
        OILTTOT = 0;
        OILPTOT = 0;
        FULPTOT = 0;
        EGTFTOT = 0;
        EGTRTOT = 0;

        // Reset dummy variable to restart averaging process
        j = 0;

        // Write output for gauge drivers
        AMCLTTout = gaugeCurrentControl(AMCLTTPIN, CLTT, AMCLTm, AMCLTb, AMCLTRshunt, AMCLTVmin, AMCLTVmax);

        if(debug)
        {
            Serial.print("Oil temperature: ");
            Serial.println(OILT);
//            Serial.print("Oil pressure: ");
//            Serial.println(OILP);
//            Serial.print("Fuel pressure: ");
//            Serial.println(FULP);
//            Serial.print("Front exhaust gas temperature: ");
//            Serial.println(EGTF);
//            Serial.print("Rear exhaust gas temperature: ");
//            Serial.println(EGTR);
              Serial.print("Autometer CLT gauge integer output (out of 256): ");
              Serial.println(AMCLTTout);
        }
    }
  
    // Check to see if a message was received
    if(CANbus.read(rxmsg))
    {
        if(debug)
        {
            Serial.println("Message received!");
        }
        
        // Check data length - 3 means the message is a request (per Megamanual)
        if (rxmsg.len == 3)
        {
          if (debug)
          {
              Serial.println("Data request recieved from MS. Processing...");
          }
          
          // Process MS data request
          MCAN.processMSreq(rxmsg.id, rxmsg.buf, msreq);

          // Create response to data request using integer-converted values
          // Note: Fuel pressure will read 10x actual value, to preserve 1st decimal place
          MCAN.setMSresp(msreq, msresp, (uint16_t)EGTF, (uint16_t)EGTR, (uint16_t)OILT, (uint16_t)(FULP*10));

          // Transfer data from MegaCAN structure to FlexCAN structure
          rspmsg.id = msresp.responseCore;
          rspmsg.len = sizeof(msresp.data.response);
          
          for (int i = 0; i < rspmsg.len; i++)
          {
              rspmsg.buf[i] = msresp.data.response[i];
          }

          // Send data back to MS
          sendsuccess = CANbus.write(rspmsg);

          if (debug)
          {
              Serial.print("Data processed. Send status = ");
              Serial.println(sendsuccess);
          }
        }

        // If it's not a data req, it's probably broadcast data (read in if desired)
        else
        {
          if (debug)
          {
              Serial.println("CAN broadcast recieved from MS. Processing...");
          }
          
          // Record time that data broadcast was recieved
          canTime = millis();
          
          // read in data with getBCastData function, this is only of secondary importance.
          MCAN.getBCastData(rxmsg.id, rxmsg.buf, msbroad);

          CLTT = msbroad.clt;

          if (debug)
          {
              Serial.print("Coolant temperature from broadcast: ");
              Serial.println(CLTT);
          }
        } 
    }

    // Check to see if timeout period has elapsed since last message recieved
    if (millis() - canTime >= canRefresh)
    {
        if(debug)
        {
            Serial.println("No CAN information recieved in specified refresh period.");
        }
        // Set variables to -777 if timeout occurs
        CLTT = -777;
    }

    // Data log variables at the specified rate
    if (datalog && millis() > lastLogTime + logPeriod)
    {         
        // Update log time
        lastLogTime = millis();

        // Update data log array with most recent data
        float dataToLog[5] = {OILT, OILP, FULP, EGTF, EGTR};
  
        String dataString = "";
        int datasize = sizeof(dataToLog);
  
        // Convert data array to a string before writing to card
        for (int i = 0; i < datasize; i++) 
        {
            // Append array value to string
            dataString += String(dataToLog[i]);

            // Add comma after value except for last data entry
            if (i < datasize-1) 
            {
                dataString += ","; 
            }
        }

        // open the file.
        File dataFile = SD.open("datalog.txt", FILE_WRITE);

        // if the file is available, write to it:
        if (dataFile)
        {
            dataFile.println(dataString);
            dataFile.close();
            // print to the serial port too:
            Serial.println(dataString);
        }  
        // if the file isn't open, pop up an error:
        else
        {
            Serial.println("error opening datalog.txt");
        }
    }
}

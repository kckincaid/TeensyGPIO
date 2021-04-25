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
// 00 (PWM) CAN Low?  ||    GND       Main ground
// 01 (PWM) CAN High? ||    3.3V
// 02 (PWM)           ||    23 (A9)
// 03 (PWM)           ||    22 (A8)
// 04 (PWM) EGTF FLT  ||    21 (A7)
// 05 (PWM) EGTF CS   ||    20 (A6)
// 06 (PWM) EGTF SDI  ||    19 (A5)
// 07 (PWM) EGTF SDO  ||    18 (A4)
// 08 (PWM) EGTF SCK  ||    17 (A3)
// 09 (PWM)           ||    16 (A2)   Fuel pressure in (10kR pulldown)
// 10 (PWM)           ||    15 (A1)   Oil pressure in
// 11 (PWM)           ||    14 (A0)   Oil temperature in (470R pullup)
// 12 (PWM)           ||    13 (LED)
// 3.3V     CAN V+    ||    GND
// 24 (A10) EGTL FLT  ||    41 (A17)
// 25 (A11) EGTL CS   ||    40 (A16)
// 26 (A12) EGTL SDI  ||    39 (A15)
// 27 (A13) EGTL SDO  ||    38 (A14)
// 28 (PWM) EGTL SCK  ||    37 (PWM)
// 29 (PWM)           ||    36 (PWM)
// 30                 ||    35
// 31                 ||    34
// 32                 ||    33 (PWM)
// -------------------------- Micro SD Slot -------------------------- //

// ------------------------------------------------------------------- //
// -------------------------- Include Files -------------------------- //
// ------------------------------------------------------------------- //

#include <circular_buffer.h>
#include <FlexCAN_T4.h>
#include <imxrt_flexcan.h>
#include <kinetis_flexcan.h>
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
static CAN_message_t        rxmsg;
CAN_message_t               rspmsg;
bool                        sendsuccess;

// ------------------------------- EGTs ------------------------------ //
uint16_t  EGTF, EGTR;
float EGTFF, EGTRF, EGTFAvg, EGTRAvg, EGTFTOT, EGTRTOT;
// Software SPI pin order: CS, DI, DO, CLK
Adafruit_MAX31856 EGTFBoard = Adafruit_MAX31856(5, 6, 7, 8);
Adafruit_MAX31856 EGTRBoard = Adafruit_MAX31856(25, 26, 27, 28);

// ------------------------- Oil Temperature ------------------------- //
uint16_t  OILT;
int       oiltemppin = 14; // A0
float     oiltempres = 470; // resistor value for oil temp circuit
float     OTC1 = 1.37e-3, OTC2 = 2.52e-4, OTC3 = 6.47e-9; // Steinhart-Hart model constants
float     OILTF, OILTTOT, OILTAvg;

// -------------------------- Oil Pressure --------------------------- //
uint16_t  OILP;
int       oilpresspin = 15; // A1

// -------------------------- Fuel Pressure -------------------------- //
uint16_t  FULP;
int       fuelpresspin = 16; // A2
float     FPV1 = 0.5;
float     FPP1 = 0;
float     FPV2 = 4.5;
float     FPP2 = 15;
float     FULPF, FULPTOT, FULPAvg;

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
        FULP = FULPTOT/j;
        EGTF = ((EGTFTOT/j)*1.8)+32;
        EGTR = ((EGTRTOT/j)*1.8)+32;

        // Reset dummy variable to restart averaging process
        j = 0;

        if(debug)
        {
            Serial.print("Oil temperature: ");
            Serial.println(OILT);
            Serial.print("Fuel pressure: ");
            Serial.println(FULP);
            Serial.print("Front exhaust gas temperature: ");
            Serial.println(EGTF);
            Serial.print("Rear exhaust gas temperature: ");
            Serial.println(EGTR);
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

          uint8_t varBlk    = msreq.data.request.varBlk;
          uint8_t fromID    = msreq.core.fromID;
          uint8_t toID      = msreq.core.toID;
          uint32_t varOffset = msreq.data.request.varOffset;
          uint8_t msgType   = 2;

          msresp.responseCore = uint32_t(0);
          msresp.responseCore |= (varBlk >> 4) << 2;            //isolate table bit 4, write to core bit 2
          msresp.responseCore |= (varBlk & ~(0b00010000)) << 3; //isolate table bits 3-0, write to core bits 6-3
          msresp.responseCore |= fromID << 7;                   //write ToID bits 3-0 to core bits 10-7
          msresp.responseCore |= toID << 11;                    //write FromID bits 3-0 to core bits 14-11
          msresp.responseCore |= msgType << 15;                 //write MsgType bits 2-0 to core bits 17-15
          msresp.responseCore |= varOffset << 18;               //write Offset bits 10-0 to core bits 28-18

          // Transfer data from MegaCAN structure to FlexCAN structure
          rspmsg.id = msresp.responseCore;
          rspmsg.len = msresp.data.request.varByt;
          memcpy(msresp.data.response, rspmsg.buf, 8);

          // Send data back to MS
          //CANbus.sendMsgBuf(msresp.responseCore, 1, msreq.data.request.varByt, msresp.data.response);
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
          // Record time that data broadcast was recieved
          canTime = millis();
          
          // read in data with getBCastData function, this is only of secondary importance.
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
    }

    // Data log variables at the specified rate
    if (datalog && millis() > lastLogTime + logPeriod)
    {         
        // Update log time
        lastLogTime = millis();

        // Update data log array with most recent data
        float dataToLog[2] = {OILTF, FULPF};
  
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

// ------------------------------------------------------------------- //
// ---------- Teensy / Megasquirt CAN Communication Sample ----------- //
// ----------------------- Coded for Teensy 4.1 ---------------------- //
// ----------------------- Last update: 5/11/21 ---------------------- //
// ------------------------------------------------------------------- //

#include <FlexCAN_T4.h>
#include <MegaCAN.h>

// Create instance of FlexCAN class using 2nd bus on Teensy
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> CANbus;

// Create instance of MegaCAN class
MegaCAN MCAN;

// ------------------------------------------------------------------- //
// ----------------------- Variable Definitions ---------------------- //
// ------------------------------------------------------------------- //

// -------------------------- User Controls -------------------------- //
bool  debug = true;       // Output debug help?
int   canRefresh = 1000;  // Time elapsed before CAN timeout reached (ms)

// -------------------------- CAN Messaging -------------------------- //
MegaCAN_message_t           msreq;    // For processing MS data requiest
MegaCAN_message_t           msresp;   // For constructing MS response
MegaCAN_broadcast_message_t msbroad;  // Stores info broadcast from MS
CAN_message_t               rxmsg;    // For recieving using FlexCAN
CAN_message_t               rspmsg;   // For sending using FlexCAN
bool                        sendsuccess;

// --------------------------- Sensor Stuff -------------------------- //
// Define variables for your sensors here, to keep it simple I just
// initialized them with a value that doesn't change.

// Stuff to send to MS:
float oilt = 101.1, oilp = 60.8, fulp = 3.45, egt1 = 1000, egt2 = 1100;

// Stuff to recieve from MS (only using 2 here, can add all if you really want):
// Initialize to random value so you know if they get changed.
float clt = -777, rpm = -777;

// Change this value to whatever yours is in the CAN parameters in TS
const uint32_t baseID = 1520;

// -------------------------- Miscellaneous -------------------------- //
unsigned long canTime = millis();
unsigned long lastLogTime = millis();

// ------------------------------------------------------------------- //
// ---------------------------- Setup Area --------------------------- //
// ------------------------------------------------------------------- //
void setup()
{
    // Set up serial bus so you can see debug output. Teensy is finicky
    // on serial output but usually will connect for a few seconds at least
    Serial.begin(9600);

    // Start up FlexCAN
    CANbus.begin();
    CANbus.setBaudRate(500000);

    // Need to tell FlexCAN to use extended frames
    rspmsg.flags.extended = 1;
    rspmsg.flags.remote = 0;
}

// ------------------------------------------------------------------- //
// ---------------------------- Main Loop ---------------------------- //
// ------------------------------------------------------------------- //
void loop()
{  
    // Check to see if a message was received using FlexCAN
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
          
          // Process MS data request using MegaCAN
          MCAN.processMSreq(rxmsg.id, rxmsg.buf, msreq);

          // Create response to data request using integer-converted values
          // Note: CAN only supports sending integers to my knowledge. For sensors with low values, e.g. fuel
          // pressure for a carbureted vehicle, you can multiply by 10 (or 100, if you wanted) to preserve the
          // 1st (or 2nd) decimal place before converting to an integer. The value will be 10x high when displayed
          // to the MS, but this should be obvious. This is done below for the fuel pressure (fulp).

          // Check data request offset. This determines whether to send ADC0-3 or 4-7. The specific values used in
          // the "if" and "else if" statements should be set to match those in the CAN Parameters / CAN ADC Selection
          // settings in TunerStudio.
          if (msreq.core.toOffset == 2)
          {
              if (debug)
              {
                  Serial.println("Request recieved for ADC bits 0-3");
              }
              // ADC0-3.
              MCAN.setMSresp(msreq, msresp, (uint16_t)oilt, (uint16_t)oilp, (uint16_t)(fulp*10), (uint16_t)egt1);
          }
          // Make sure this offset value matches that in Tuner Studio for ADC 4-7
          else if (msreq.core.toOffset == 10)
          {
              if (debug)
              {
                  Serial.println("Request recieved for ADC bits 4-7");
              }
              // ADC4-7. Can add any additional variables you want to the remaining spots where 0s are sent
              MCAN.setMSresp(msreq, msresp, (uint16_t)egt2, (uint16_t)0, (uint16_t)0, (uint16_t)0);
          }
          

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
          
          // read in data with getBCastData function
          MCAN.getBCastData(rxmsg.id, rxmsg.buf, msbroad);

          // Data can then be accessed by using the MegaCAN message structure
          // as long as you have enabled the correct data to be sent in TS.
          // See MegaCAN.h file for a list of the variables you can use.
          clt = msbroad.clt;
          rpm = msbroad.rpm;

          if (debug)
          {
              Serial.print("Coolant temperature from broadcast: ");
              Serial.println(clt);
              Serial.print("Engine rpm from broadcast: ");
              Serial.println(rpm);
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
        clt = -777;
        rpm = -777;
    }
}

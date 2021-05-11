// Arduino + MCP2515 to Megasquirt CANBUS Interface

#include <SPI.h>          // SPI is used to talk to the CAN Controller
#include <mcp_can.h>      // MCPCAN used to send/recieve messages
#include <MegaCAN.h>      // MegaCAN used to process and construct messages per Megasquirt standards

MCP_CAN CAN(10);          // Set SPI Chip Select to pin 10
MegaCAN MCAN;             // Create instance of MegaCAN class

// Create variables for CAN messages
unsigned char len = 0;
uint32_t      canID;
uint8_t       buf[3];
int           sendstatus;

// CAN base ID: change to match value in TunerStudio
const uint32_t baseID = 1520;

// Variables we want to send to MS (up to 8):
float         oilt = 100, oilp = 60.1, fulp = 3.43, egt1 = 1000, egt2 = 1100;

// Variables to get from MS data broadcast
float         clt, rpm;

MegaCAN_message_t msreq;    // Stores request for info from MS
MegaCAN_message_t msresp;   // Stores info to be sent to MS
MegaCAN_broadcast_message_t msbroad;  // Stores info broadcast from MS

// ----------------------------- Setup Loop ----------------------------- //
void setup()
{
    Serial.begin(115200);   //to communicate with Serial monitor
  
    // MCP CAN tries to initialize, if failed --> it will loop here for ever
    START_INIT:

    if(CAN_OK == CAN.begin(CAN_500KBPS))      //setting CAN baud rate to 500Kbps
    {
        Serial.println("CAN BUS Shield init ok!");
    }
    else
    {
        Serial.println("CAN BUS Shield init fail");
        Serial.println("Init CAN BUS Shield again");
        delay(100);
        goto START_INIT;
    }
}

// ----------------------------- Main Loop ------------------------------ //
void loop()
{
    // Check to see if a message was received
    if(CAN_MSGAVAIL == CAN.checkReceive())
    {
        Serial.println("Message received!");
      
        CAN.readMsgBuf(&len, buf);    // read data,  len: data length, buf: data buffer
        canID = CAN.getCanId();       // getting the ID of the incoming message

        // Check to see if recieved message is a data req from MS
        //if (CAN.isRemoteRequest() == 0)
        if (len == 3)
        {
          Serial.println("Data request recieved from MS. Processing...");
          
          // Process MS data request
          MCAN.processMSreq(canID, buf, msreq);

          Serial.print("CAN ID is: ");
          Serial.println(canID);
          Serial.print("Offset of request is: ");
          Serial.println(msreq.core.toOffset);

          // Make oil temperature time-dependent so you can track it in Tuner Studio to see if it updates correctly.
          oilt = 100*(sin(millis()/1000)+1);

          // Create response to data request. Data must be sent as integers, to type cast to uint16_t prior to sending.
          // For small valued data like fuel pressure on a carbureted engine, can multiply by 10 (or 100, etc.) to preserve
          // decimal places. This will make the value read 10x (or 100x, etc.) high in TunerStudio, but will retain
          // greater accuracy. This is done for fuel pressure: the actual value is 3.43, if we sent this as-is, TunerStudio
          // would display a value of 3, which is a loss of about 13% loss in accuracy. Multiplying by 10, TS will display
          // a value of 34: we know this means the actual reading is 3.4, which is more than 99% accurate.

          // Check request offset to determine if ADC0-3 or 4-7 are requested. Make sure the values in these conditional
          // statements match those in your TunerStudio CAN settings.
          if (msreq.core.toOffset == 2)
          {
              // ADC 0-3
              Serial.println("Recieved request for ADC 0-3");
              MCAN.setMSresp(msreq, msresp, (uint16_t)oilt, (uint16_t)oilp, (uint16_t)(fulp*10), (uint16_t)egt1);
          }
          // Can fill the last 3 spots with data as well, only sending 5 variables here just to demonstrate how to use offset
          // to send to ADC0-3 or 4-7.
          if (msreq.core.toOffset == 10)
          {
              // ADC 4-7
              Serial.println("Recieved request for ADC 4-7");
              MCAN.setMSresp(msreq, msresp, (uint16_t)egt2, (uint16_t)0, (uint16_t)0, (uint16_t)0);
          }
          
          // I am unsure if this block is necessary, but adding it fixed issues I was having with Arduino and MCP CAN. These
          // variables are defined in the MegaCAN code, but I found that explicitly re-defining them with appropriate types
          // solved issues with the header not being constructed appropriately.
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

          // Send data back to MS
          sendstatus = CAN.sendMsgBuf(msresp.responseCore, 1, msreq.data.request.varByt, msresp.data.response);

          Serial.print("Data processed and sent to MS. Send status: ");
          Serial.println(sendstatus);
        }

        // If it's not a data req, it's probably broadcast data (read in if desired)
        else
        {
          // read in data with getBCastData function
          Serial.println("Broadcast data recieved from MS. Processing...");

          // This line creates a LOT of variables which can cause issues with stability on basic boards like the Uno.
          // Can comment out unnecessary stuff in the MegaCAN.cpp file to save memory space.
          MCAN.getBCastData(canID, buf, msbroad);

          // Broadcast data can be accessed directly from the broadcast message where it was stored, as long
          // as that specific data broadcast is enabled in TunerStudio.
          clt = msbroad.clt;
          rpm = msbroad.rpm;

          Serial.print("Coolant temperature from broadcast: ");
          Serial.println(clt);
          Serial.print("Engine rpm from broadcast: ");
          Serial.println(rpm);
        } 
    }
    
}

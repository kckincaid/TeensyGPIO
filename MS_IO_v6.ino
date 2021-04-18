// Megasquirt CAN IO board v6

// Trying functionality from MegaCAN library

#include <SPI.h>          //SPI is used to talk to the CAN Controller
#include <mcp_can.h>
#include <MegaCAN.h>

MCP_CAN CAN(10);          //set SPI Chip Select to pin 10
MegaCAN MCAN;

unsigned char len = 0;
uint32_t      canID;
uint8_t       buf[3];
uint16_t       EGT1 = 1780; // these are the two variables we want to send
uint16_t       EGT2 = 1900; // over to the megasquirt (will get from MAX modules later)
uint16_t       OILT;
uint16_t       FUEL = 35;

int oiltemppin = 1;
int Vo;
float R1 = 440; // resistor value for oil temp circuit
float logR2, R2, OILTF;
float c1 = 1.37e-3, c2 = 2.52e-4, c3 = 6.47e-9;

MegaCAN_message_t msreq;    // Stores request for info from MS
MegaCAN_message_t msresp;   // Stores info to be sent to MS
MegaCAN_broadcast_message_t msbroad;  // Stores info broadcast from MS

void setup()
{
    Serial.begin(115200);   //to communicate with Serial monitor
  
    //tries to initialize, if failed --> it will loop here for ever
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


void loop()
{
    // Read in oil temp pin and calculate oil temperature
    Vo = analogRead(oiltemppin);
    R2 = R1/(1023 / (float)Vo - 1);
    logR2 = log(R2);
    OILTF = (1.0 / (c1 + c2*logR2 + c3*logR2*logR2*logR2));
    OILTF = OILTF - 273.15;
    OILTF = (OILTF * 9.0)/ 5.0 + 32.0; 
    OILT = (uint16_t)OILTF;
  
    // Check to see if a message was received
    if(CAN_MSGAVAIL == CAN.checkReceive())
    {
        Serial.println("Message received!");
      
        CAN.readMsgBuf(&len, buf);    // read data,  len: data length, buf: data buffer
        canID = CAN.getCanId();       // getting the ID of the incoming message

        // Check to see if recieved message is a data req from MS
        if (CAN.isRemoteRequest() == 0)
        {
          Serial.println("Data request recieved from MS. Processing...");
          
          // Process MS data request
          MCAN.processMSreq(canID, buf, msreq);

          Serial.println("CAN ID is: ");
          Serial.println(canID);

          // Create response to data request
          MCAN.setMSresp(msreq, msresp, EGT1, EGT2, OILT, FUEL);

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
          CAN.sendMsgBuf(msresp.responseCore, 1, msreq.data.request.varByt, msresp.data.response);

          Serial.println("Data processed and sent to MS.");
        }

        // If it's not a data req, it's probably broadcast data (read in if desired)
        else
        {
          // read in data with getBCastData function, this is only of secondary importance.
        } 
    }
    
}

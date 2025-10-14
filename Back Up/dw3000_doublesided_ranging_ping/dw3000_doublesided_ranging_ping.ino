// Tag - Initiator/Ping

#include "DW3000.h"

#define DEBUG_PRINT 1 

/*
   BE AWARE: Baud Rate got changed to 2.000.000!

   Approach based on the application note APS011 ("SOURCES OF ERROR IN DW1000 BASED
   TWO-WAY RANGING (TWR) SCHEMES")

   see chapter 2.4 figure 6 and the corresponding description for more information

   This approach tackles the problem of a big clock offset between the ping and pong side
   by reducing the clock offset to a minimum.

   This approach is a more advanced version of the classical ping and pong with timestamp examples.
*/

#define ROUND_DELAY 500 // Delay in milliseconds that the chip waits between PING requests

//Setting the ID of anchor & tag
#define TAG_ID 0xA0
#define ANCHOR1_ID 0x01
#define BROADCAST_ID 0xFF




static int frame_buffer = 0; // Variable to store the transmitted message
static int rx_status; // Variable to store the current status of the receiver operation
static int tx_status; // Variable to store the current status of the receiver operation

/*
   valid stages:
   0 - default stage; starts ranging
   1 - ranging sent; awaiting response
   2 - response received; sending second range
   3 - second ranging sent; awaiting final answer
   4 - final answer received
*/
static int curr_stage = 0;

static int t_roundA = 0;
static int t_replyA = 0;

static long long rx = 0;
static long long tx = 0;

static int clock_offset = 0;

static int ranging_time = 0;

static float distance = 0;

void setup()
{
  Serial.begin(2000000); // Init Serial
  DW3000.begin(); // Init SPI
  DW3000.hardReset(); // hard reset in case that the chip wasn't disconnected from power
  delay(200); // Wait for DW3000 chip to wake up

  if(!DW3000.checkSPI())
  {
    Serial.println("[ERROR] Could not establish SPI Connection to DW3000! Please make sure that all pins are set correctly.");
    while(100);
  }
  
  while (!DW3000.checkForIDLE()) // Make sure that chip is in IDLE before continuing
  {
    Serial.println("[ERROR] IDLE1 FAILED\r");
    delay(1000);
  }

  DW3000.softReset();
  delay(200); // Wait for DW3000 chip to wake up


  if (!DW3000.checkForIDLE())
  {
    Serial.println("[ERROR] IDLE2 FAILED\r");
    while (100);
  }


  DW3000.init(); // Initialize chip (write default values, calibration, etc.)
  DW3000.setupGPIO(); //Setup the DW3000s GPIO pins for use of LEDs
  Serial.println("> double-sided PING with timestamp example <\n");
  Serial.println("[INFO] Setup is finished.");

  // Set this deviceID
  DW3000.setSenderID(TAG_ID);
  
  DW3000.configureAsTX(); // Configure basic settings for frame transmitting

  DW3000.clearSystemStatus();
}

void loop()
{
  switch (curr_stage) {

    //Transmit the Poll message
    case 0:  // Start ranging.
      
      DW3000.setDestinationID(BROADCAST_ID);
      
      t_roundA = 0;
      t_replyA = 0;
      DW3000.ds_sendFrame(1);
      if (DEBUG_PRINT){
        Serial.println("-----------------------------------------");
        Serial.println("----------------!!START!!----------------");
        Serial.println("Start Transmit the Poll msg");
      }
      tx = DW3000.readTXTimestamp();

      curr_stage = 1;
      break;

    //Receive the Response message
    case 1:  // Await first response.
      if (rx_status = DW3000.receivedFrameSucc()) {
        DW3000.clearSystemStatus();
        if (rx_status == 1) { // If frame reception was successful
          if (DW3000.ds_isErrorFrame()) {
            Serial.println("[WARNING] Error frame detected! Reverting back to stage 0.");
            curr_stage = 0;
          } else if (DW3000.ds_getStage() != 2) {
            Serial.println("Error Get Stage");
            Serial.println(DW3000.ds_getStage());
            DW3000.ds_sendErrorFrame();
            curr_stage = 0;
          } else {

            if (DEBUG_PRINT){
              //Debug msg
              Serial.println("Success recieved the Response msg");
              int response_sender = DW3000.getSenderID();
              int response_destination = DW3000.getDestinationID();
              int response_stage = DW3000.ds_getStage();
              
              //Debug msg print
              Serial.print("[RESPONSE msg] sender=0x");
              Serial.print(response_sender, HEX);
              Serial.print(" dest=0x");
              Serial.print(response_destination, HEX);
              Serial.print(" stage=");
              Serial.println(response_stage);
            }
            
            curr_stage = 2;
          }
        } else // if rx_status returns error (2)
        {
          Serial.println("[ERROR] Receiver Error occured! Aborting event.");
          DW3000.clearSystemStatus();
        }
      }
      break;

    //Transmit the Final message
    case 2:  // Response received. Send second ranging.
      rx = DW3000.readRXTimestamp();
      DW3000.setDestinationID(ANCHOR1_ID);
      DW3000.ds_sendFrame(3);
      Serial.println("Start Transmit the Final msg");
      t_roundA = rx - tx;

      tx = DW3000.readTXTimestamp();
      t_replyA = tx - rx;

      curr_stage = 3;
      break;

    //Receive the Report message
    case 3:  // Await second response.
      if (rx_status = DW3000.receivedFrameSucc()) {
        DW3000.clearSystemStatus();
        if (rx_status == 1) { // If frame reception was successful
          if (DW3000.ds_isErrorFrame()) {
            Serial.println("[WARNING] Error frame detected! Reverting back to stage 0.");
            curr_stage = 0;
          } else {

            if (DEBUG_PRINT){
              //Debug msg
              Serial.println("Success recieved the Report msg");
              int report_sender = DW3000.getSenderID();
              int report_destination = DW3000.getDestinationID();
              int report_stage = DW3000.ds_getStage();
              
              //Debug msg print
              Serial.print("[REPORT msg] sender=0x");
              Serial.print(report_sender, HEX);
              Serial.print(" dest=0x");
              Serial.print(report_destination, HEX);
              Serial.print(" stage=");
              Serial.println(report_stage);
            }
            clock_offset = DW3000.getRawClockOffset();
            curr_stage = 4;
          }
        } else // if rx_status returns error (2)
        {
          Serial.println("[ERROR] Receiver Error occured! Aborting event.");
          DW3000.clearSystemStatus();
        }
      }
      break;
    case 4:  // Response received. Calculating results.
      
      ranging_time = DW3000.ds_processRTInfo(t_roundA, t_replyA, DW3000.read(0x12, 0x04), DW3000.read(0x12, 0x08), clock_offset);
      distance = DW3000.convertToCM(ranging_time);

      DW3000.printDouble(distance, 100, false); // 100 equals to 2 decimal places, 1000 to 3, 10000 to 4...
      Serial.println("cm");

      curr_stage = 0;
      
      if (DEBUG_PRINT){
      Serial.println("-----------------!!END!!-----------------");
      Serial.println("-----------------------------------------");
      }
      
      delay(ROUND_DELAY);
      break;
      
    default:
      Serial.print("[ERROR] Entered unknown stage (");
      Serial.print(curr_stage);
      Serial.println("). Reverting back to stage 0");

      curr_stage = 0;
      break;
  }
}

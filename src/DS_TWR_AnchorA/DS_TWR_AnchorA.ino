// AnchorB - Responder/Pong

#include "DW3000.h"

#define DEBUG_PRINT 1
#define DEBUG_PRINT1 1 

/*
   BE AWARE: Baud Rate got changed to 2.000.000!

   Approach based on the application note APS011 ("SOURCES OF ERROR IN DW1000 BASED
   TWO-WAY RANGING (TWR) SCHEMES")

   see chapter 2.4 figure 6 and the corresponding description for more information

   This approach tackles the problem of a big clock offset between the ping and pong side
   by reducing the clock offset to a minimum.

   This approach is a more advanced version of the classical ping and pong with timestamp examples.
*/

//Setting the ID of anchor & tag
#define TAG_ID 0xA0
#define ANCHOR_A
#define TEST_DELAY 0

#define AnchorA_ID 0x01
#define AnchorB_ID 0x02
#define AnchorC_ID 0x03
#define AnchorD_ID 0x04
// Setting the Anchor Config
#ifdef ANCHOR_A
  #define ANCHOR_ID     0x01
  #define ANCHOR_SLOT   1
#elif defined(ANCHOR_B)
  #define ANCHOR_ID     0x02
  #define ANCHOR_SLOT   2
#elif defined(ANCHOR_C)
  #define ANCHOR_ID     0x03
  #define ANCHOR_SLOT   3
#elif defined(ANCHOR_D)
  #define ANCHOR_ID     0x04
  #define ANCHOR_SLOT   4
#else
  #error "No Define Anchor(ANCHOR_A OR ANCHOR_B OR ANCHOR_C OR ANCHOR_D)"
#endif

#define SLOT_TIME 10000

const int MAX_RESP = 20;
static int frame_buffer = 0; // Variable to store the transmitted message
static int rx_status; // Variable to store the current status of the receiver operation
static int tx_status; // Variable to store the current status of the receiver operation

/*
   valid stages:
   0 - default stage; await ranging
   1 - ranging received; sending response
   2 - response sent; await second response
   3 - second response received; sending information frame
   4 - information frame sent
*/
static int curr_stage = 0;
static int count = 0;
struct AnchorIn
{
  unsigned long long poll_Rx = 0;
  unsigned long long resp_Tx = 0;
  unsigned long long final_Rx = 0;

  int t_round = 0;
  int t_reply = 0;

} anchor;


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
  DW3000.softReset(); // Reset in case that the chip wasn't disconnected from power
  delay(200); // Wait for DW3000 chip to wake up

  if (!DW3000.checkForIDLE())
  {
    Serial.println("[ERROR] IDLE2 FAILED\r");
    while (100);
  }

  DW3000.init(); // Initialize chip (write default values, calibration, etc.)
  DW3000.setupGPIO(); //Setup the DW3000s GPIO pins for use of LEDs

  Serial.println("> double-sided PONG with timestamp example <\n");

  Serial.println("[INFO] Setup finished.");

  // Set this deviceID
  DW3000.setSenderID(ANCHOR_ID);
  
  DW3000.configureAsTX(); // Configure basic settings for frame transmitting

  DW3000.clearSystemStatus();

  DW3000.standardRX();
}

void loop()
{
  switch (curr_stage) {
    //Receive the Poll message
    case 0:  // Await ranging.

      // Initialize Anchor Struct
      resetAnchorStruct(anchor);
      
      DW3000.standardRX();

      if (0){
        //Debug msg
        Serial.println("-----------------------------------------");
        Serial.println("----------------!!START!!----------------");
        Serial.println("Wait Poll");
      }
      
      if(rx_status = DW3000.receivedFrameSucc()){
      if (rx_status == 1) {
        DW3000.clearSystemStatus();

        if (DW3000.ds_getStage() == 1 && DW3000.getSenderID() == TAG_ID) {
          if (DEBUG_PRINT) {
            //Debug msg
            Serial.println("Success recieved Poll");
          }
          anchor.poll_Rx = DW3000.readRXTimestamp();
          curr_stage = 1;

        }} else {
          if (DEBUG_PRINT) {
                Serial.println("[ERROR] Poll error Receiver Error occured! Aborting event.");
          }
          DW3000.clearSystemStatus();
          DW3000.standardRX();
        }
      }
      break;
      
    //Transmit the Response message
    case 1:  // Ranging received. Sending response.
      //Send the Response Message
      if (DEBUG_PRINT){
      Serial.println("Start Response");
      }

      DW3000.setDestinationID(TAG_ID);
      DW3000.ds_sendResp(2, ANCHOR_SLOT, (SLOT_TIME + TEST_DELAY));

      anchor.resp_Tx = DW3000.readTXTimestamp();
      curr_stage = 2;
      break;


    //Receive the Final message
    case 2:
      rx_status = DW3000.receivedFrameSucc();
      if (rx_status == 1) {
        DW3000.clearSystemStatus();
        if (DW3000.ds_getStage() == 3 && DW3000.getSenderID() == TAG_ID) {
          if (DEBUG_PRINT) {
            //Debug msg
            Serial.println("Success recieved Poll");
          }
          anchor.final_Rx = DW3000.readRXTimestamp();
          curr_stage = 3;
        } else if (DW3000.ds_getStage() == 1 && DW3000.getSenderID() == TAG_ID) {
          if (DEBUG_PRINT){
            Serial.println("[WARNING] AGAIN POLL - stage 0");
          }
          curr_stage = 0;
          DW3000.clearSystemStatus();
          DW3000.standardRX();
        }
        
        else {
          if (DEBUG_PRINT) {
            Serial.println("[ERROR] Poll error Receiver Error occured! Aborting event.");
          }
          DW3000.clearSystemStatus();
          DW3000.standardRX();
        }
      }
      break;
    
    //Transmit the Report message
    case 3:  // Second response received. Sending information frame.
      //Calculate Round & Reply time
      anchor.t_reply = int(anchor.resp_Tx - anchor.poll_Rx);
      anchor.t_round = int(anchor.final_Rx - anchor.resp_Tx);

      // Send the Report message
      DW3000.setDestinationID(TAG_ID);
      DW3000.ds_sendRTInfo(anchor.t_round, anchor.t_reply, ANCHOR_SLOT, (SLOT_TIME + TEST_DELAY));

      if (DEBUG_PRINT){
        Serial.println("Start Report");
        Serial.println("-----------------!!END!!-----------------");
        Serial.println("-----------------------------------------");
      }
      
      curr_stage = 0;
      break;

      
    default:
      Serial.print("[ERROR] Entered unknown stage (");
      Serial.print(curr_stage);
      Serial.println(").");

      curr_stage = 0;
      DW3000.clearSystemStatus();
      DW3000.standardRX();
      break;
  }
}

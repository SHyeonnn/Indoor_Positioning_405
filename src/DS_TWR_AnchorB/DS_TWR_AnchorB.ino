// AnchorB - Responder/Pong

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

//Setting the ID of anchor & tag
#define TAG_ID 0xA0
#define AnchorA_ID 0x01
#define AnchorB_ID 0x02
#define AnchorC_ID 0x03
#define AnchorD_ID 0x04

#define A_Count 0x04
#define B_Count 0x02
#define C_Count 0x03
#define D_Count 0x04

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

struct AnchorIn {
  unsigned long long poll_Rx = 0; // t2
  unsigned long long resp_Tx = 0; // t3
  unsigned long long final_Rx = 0; // t5

  int t_round = 0; // resp_Tx - final_Rx = t3 - t5
  int t_reply = 0; // poll_Rx - resp_Tx = t2 - t3

  int resp_count = 0;
  unsigned long long resp_Tx_buffer[MAX_RESP];
} anchor;

void setup(){
  Serial.begin(2000000); // Init Serial
  DW3000.begin(); // Init SPI
  DW3000.hardReset(); // hard reset in case that the chip wasn't disconnected from power
  delay(200); // Wait for DW3000 chip to wake up

  if(!DW3000.checkSPI()) {
    Serial.println("[ERROR] Could not establish SPI Connection to DW3000! Please make sure that all pins are set correctly.");
    while(100);
  }
  
  while (!DW3000.checkForIDLE()) { // Make sure that chip is in IDLE before continuing
    Serial.println("[ERROR] IDLE1 FAILED\r");
    delay(1000);
  }
  DW3000.softReset(); // Reset in case that the chip wasn't disconnected from power
  delay(200); // Wait for DW3000 chip to wake up

  if (!DW3000.checkForIDLE()) {
    Serial.println("[ERROR] IDLE2 FAILED\r");
    while (100);
  }

  DW3000.init(); // Initialize chip (write default values, calibration, etc.)
  DW3000.setupGPIO(); //Setup the DW3000s GPIO pins for use of LEDs

  Serial.println("> double-sided PONG with timestamp example <\n");
  Serial.println("[INFO] Setup finished.");

  // Set this deviceID
  DW3000.setSenderID(AnchorB_ID);
  DW3000.configureAsTX(); // Configure basic settings for frame transmitting
  DW3000.clearSystemStatus();
  DW3000.standardRX();
}

void loop(){
  switch (curr_stage) {
    // stage 1: Receive the Poll message
    case 0:  // Await ranging.
      // Initialize Anchor Struct
      resetAnchorStruct(anchor);
      DW3000.standardRX();

      if (rx_status = DW3000.receivedFrameSucc()) {
        DW3000.clearSystemStatus();
        if (rx_status == 1) { // If frame reception was successful
          if (DW3000.ds_isErrorFrame()) {
            Serial.println("[WARNING] Error frame detected! Reverting back to stage 0.");
            curr_stage = 0;
            DW3000.standardRX();
          } 
          else if (DW3000.ds_getStage() != 1) {
            if (DEBUG_PRINT){
              //Debug msg print
              Serial.print("Error Stage : ");
              Serial.println(rx_status);
            }
            DW3000.ds_sendErrorFrame();
            DW3000.standardRX();
            curr_stage = 0;
          }
          else {
            if (DW3000.getSenderID() == TAG_ID){
              // Successfully Recieved the Poll Message
              if (DEBUG_PRINT){
                //Debug msg
                Serial.println("-----------------------------------------");
                Serial.println("----------------!!START!!----------------");
                Serial.println("Success recieved the Poll msg");
                int poll_sender = DW3000.getSenderID();
                int poll_destination = DW3000.getDestinationID();
                int poll_stage = DW3000.ds_getStage();
                
                // Debug msg print
                Serial.print("[POLL msg] sender=0x");
                Serial.print(poll_sender, HEX);
                Serial.print(" dest=0x");
                Serial.print(poll_destination, HEX);
                Serial.print(" stage=");
                Serial.println(poll_stage);
              }
              anchor.poll_Rx = DW3000.readRXTimestamp();
              curr_stage = 1;
            }
            else{
              Serial.println("[WARNING] Not Tag ID (Not Poll).");
            }
          }
        }
        else // if rx_status returns error (2){
          Serial.println("[ERROR] Receiver Error occured! Aborting event.");
          DW3000.clearSystemStatus();
        }
      }
      break;
      
    // stage 2: Transmit the Response message
    case 1:  // Ranging received. Sending response.
      delay((AnchorB_ID - 1) * 10); // AnchorB (ID:2) waits 10ms
      anchor.resp_count++;
      //Send the Response Message
      if (DEBUG_PRINT){
      Serial.println("Start Transmit the Response msg");
      }

      DW3000.setDestinationID(TAG_ID);
      DW3000.ds_sendResp(2, anchor.resp_count);
      // sender = 0x1, dest = 0xA0, stage = 2

      anchor.resp_Tx = DW3000.readTXTimestamp();
      anchor.resp_Tx_buffer[anchor.resp_count % MAX_RESP] = anchor.resp_Tx;
      
      DW3000.standardRX();

      // Check whether the Final signal has arrived
      if (rx_status = DW3000.receivedFrameSucc()){
        DW3000.clearSystemStatus();
        if (rx_status == 1) { // If frame reception was successful
          if (DW3000.ds_isErrorFrame()) {
            Serial.println("[WARNING] Error frame detected!");
            DW3000.standardRX();
          }
          else if (DW3000.ds_getStage() != 3) {
            DW3000.ds_sendErrorFrame();
            DW3000.standardRX();
          }
          else if (DW3000.getSenderID() != TAG_ID){
            Serial.print("[WARNING] Final msg ignored from non-Tag sender=0x");
            Serial.println(DW3000.getSenderID(), HEX);
            DW3000.standardRX();
          }
          else{
            curr_stage = 2;
          }
        }
        else { // if rx_status returns error (2)
          Serial.println("[ERROR] Receiver Error occured! Aborting event.");
          DW3000.clearSystemStatus();
          DW3000.standardRX();
        }
      }
      break;
      
    // stage 3: Receive the Final message
    case 2:
      if (DEBUG_PRINT){
        //Debug msg
        Serial.println("Success Received the Final msg");
        int Final_sender = DW3000.getSenderID();
        int Final_destination = DW3000.getDestinationID();
        int Final_stage = DW3000.ds_getStage();
        
        //Debug msg print
        Serial.print("[Final msg] sender=0x");
        Serial.print(Final_sender, HEX);
        Serial.print(" dest=0x");
        Serial.print(Final_destination, HEX);
        Serial.print(" stage=");
        Serial.println(Final_stage);
      }
      count = DW3000.read(0x12, B_Count) & 0xFF;
      anchor.final_Rx = DW3000.readRXTimestamp();

      if (count >= 0 && count < MAX_RESP){
        unsigned long long matched_respTx = anchor.resp_Tx_buffer[count];
        anchor.resp_Tx = matched_respTx;
        curr_stage = 3; 
        count = 0;
      } else{
        Serial.print("[WARNING] Invalid count index=");
        Serial.println(count);
      }
      
      break;

    // stage 4: Transmit the Report message
    case 3:  // Second response received. Sending information frame.
      //Calculate Round & Reply time
      anchor.t_reply = int(anchor.resp_Tx - anchor.poll_Rx);
      anchor.t_round = int(anchor.final_Rx - anchor.resp_Tx);

      // Send the Report message
      DW3000.setDestinationID(TAG_ID);
      DW3000.ds_sendRTInfo(anchor.t_round, anchor.t_reply);

      if (DEBUG_PRINT){
      Serial.println("Start Transmit the Report msg");
      Serial.println("-----------------!!END!!-----------------");
      Serial.println("-----------------------------------------");
      }

      DW3000.standardRX();
      // Check the New poll msg
      if (rx_status = DW3000.receivedFrameSucc()) {
        DW3000.clearSystemStatus();
        if (rx_status == 1 && DW3000.ds_getStage() == 1 && DW3000.getSenderID() == TAG_ID) {
          Serial.println("[INFO] New Poll received -> restart ranging");
          anchor.poll_Rx = DW3000.readRXTimestamp();
          curr_stage = 4;
        }
      }
      
      break;

    case 4:  // Recieved New poll msg
      // Initialize Anchor Struct
      resetAnchorStruct(anchor);
      // Successfully Recieved the Poll Message
      if (DEBUG_PRINT){
        //Debug msg
        Serial.println("-----------------------------------------");
        Serial.println("----------------!!START!!----------------");
        Serial.println("Success recieved the Poll msg");
        int poll_sender = DW3000.getSenderID();
        int poll_destination = DW3000.getDestinationID();
        int poll_stage = DW3000.ds_getStage();
        
        //Debug msg print
        Serial.print("[POLL msg] sender=0x");
        Serial.print(poll_sender, HEX);
        Serial.print(" dest=0x");
        Serial.print(poll_destination, HEX);
        Serial.print(" stage=");
        Serial.println(poll_stage);
      }
      anchor.poll_Rx = DW3000.readRXTimestamp();
      curr_stage = 1;
      
      break;


    default:
      Serial.print("[ERROR] Entered unknown stage (");
      Serial.print(curr_stage);
      Serial.println(").");

      // curr_stage = 0;
      DW3000.standardRX();
      break;
  }
}


void resetAnchorStruct(AnchorIn &anchor){
  anchor.poll_Rx = 0;
  anchor.resp_Tx = 0;
  anchor.final_Rx = 0;

  anchor.t_round = 0;
  anchor.t_reply = 0;

  anchor.resp_count = 0;
  anchor.resp_Tx_buffer[MAX_RESP];

  for (int i = 0; i < MAX_RESP; i++) {
    anchor.resp_Tx_buffer[i] = 0;
  }

}

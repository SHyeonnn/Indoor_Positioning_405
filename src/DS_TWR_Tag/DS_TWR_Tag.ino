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
#define AnchorA_ID 0x01
#define AnchorB_ID 0x02
#define AnchorC_ID 0x03
#define AnchorD_ID 0x04

#define BROADCAST_ID 0xFF



static int frame_buffer = 0; // Variable to store the transmitted message
static int rx_status; // Variable to store the current status of the receiver operation
static int tx_status; // Variable to store the current status of the receiver operation
static int curr_stage = 0;

/*
   valid stages:
   0 - default stage; starts ranging
   1 - ranging sent; awaiting response
   2 - response received; sending second range
   3 - second ranging sent; awaiting final answer
   4 - final answer received
*/


// Anchor Struct Setting
AnchorAll anchor_all;
Tag tag;


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

    // Transmit the Poll message
    case 0:  // Start ranging.
      
      // Initialize Anchor & Tag Struct
      resetAnchorAll(anchor_all);
      resetTag(tag);

      // Send the Poll message
      DW3000.setDestinationID(BROADCAST_ID);
      DW3000.ds_sendPoll(1);

      if (DEBUG_PRINT){
        Serial.println("-----------------------------------------");
        Serial.println("----------------!!START!!----------------");
        Serial.println("Start Transmit the Poll msg");
      }
      
      // Store Value; Poll Tx Time
      tag.poll_Tx = DW3000.readTXTimestamp();

      curr_stage = 1;
      break;


    //Receive the Response message
    case 1:  // Await first response.

      if (rx_status = DW3000.receivedFrameSucc()) {
        DW3000.clearSystemStatus();
        if (rx_status == 1) { // If frame reception was successful
          if (DW3000.ds_isErrorFrame()) {
            Serial.println("[WARNING] Error frame detected! Reverting back to stage 0.");
          } else if (DW3000.ds_getStage() != 2) {
            Serial.println("Error Get Stage");
            Serial.println(DW3000.ds_getStage());
            DW3000.ds_sendErrorFrame();
          } else {

            if (DEBUG_PRINT){
              //Debug msg
              Serial.println("Success received the Response msg");
              int response_sender = DW3000.getSenderID();
              int response_destination = DW3000.getDestinationID();
              int response_stage = DW3000.ds_getStage();
              int response_count = DW3000.ds_getRespCount();
              //Debug msg print
              Serial.print("[RESPONSE msg] sender=0x");
              Serial.print(response_sender, HEX);
              Serial.print(" dest=0x");
              Serial.print(response_destination, HEX);
              Serial.print(" stage=");
              Serial.println(response_stage);
              Serial.print(" Resp Count=");
              Serial.println(response_count);
            }

            if (saveResp(anchor_all, tag, DW3000.getSenderID(), DW3000.ds_getRespCount(), DW3000.readRXTimestamp())){

              if (allRespReceived(anchor_all)){
                if (DEBUG_PRINT){
                  Serial.println("[INFO] All anchors' Response responded. Moving to stage 2.");
                }
                curr_stage = 2;
              } else {
                if (DEBUG_PRINT){
                  Serial.println("[INFO] This Response msg already saved ");
                }
                DW3000.standardRX();
              }

            } else {
              DW3000.standardRX();
            }
            
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

      DW3000.setDestinationID(BROADCAST_ID);
      DW3000.ds_sendFinal(3, anchor_all);
      Serial.println("Start Transmit the Final msg");

      tag.final_Tx = DW3000.readTXTimestamp();

      curr_stage = 3;
      break;

    //Receive the Report message
    case 3:  // Await second response.
      if (rx_status = DW3000.receivedFrameSucc()) {
        
        DW3000.clearSystemStatus();
        
        if (rx_status == 1) { // If frame reception was successful
          
          if (DW3000.ds_isErrorFrame()) {
            Serial.println("[WARNING] Error frame detected!");
          } else {
            
            if (DEBUG_PRINT){
              //Debug msg
              Serial.println("Success received the Report msg");
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

            if (saveReport(anchor_all, DW3000.getSenderID(), DW3000.read(0x12, 0x04),  DW3000.read(0x12, 0x08), DW3000.getRawClockOffset())){
              // round, reply, clock offset
              if (allReportsReceived(anchor_all)){
                if (DEBUG_PRINT){
                  Serial.println("[INFO] All anchors' Report responded. Moving to stage 2.");
                }
                curr_stage = 2;
              } else {
                if (DEBUG_PRINT){
                  Serial.println("[INFO] This Report msg already saved ");
                }
                DW3000.standardRX();
              }

            curr_stage = 4;
          } else {
            // if rx_status returns error (2)
            Serial.println("[ERROR] Receiver Error occured! Aborting event.");
            DW3000.clearSystemStatus();
            }
          
          }
        }
      }
      break;

    case 4:  // Response received. Calculating results.
      
      // Anchor A
      anchor_all.AncA.t_round_tag = (int)(anchor_all.AncA.resp_Rx - tag.poll_Tx);
      anchor_all.AncA.t_reply_tag = (int)(tag.final_Tx - anchor_all.AncA.resp_Rx);
      anchor_all.AncA.ranging_time = DW3000.ds_processRTInfo(anchor_all.AncA.t_round_tag, anchor_all.AncA.t_reply_tag, 
                                                             anchor_all.AncA.t_round_Anc,  anchor_all.AncA.t_reply_Anc,  anchor_all.AncA.clock_offset);
      anchor_all.AncA.distance = DW3000.convertToCM(anchor_all.AncA.ranging_time);

      // Anchor B
      anchor_all.AncB.t_round_tag = (int)(anchor_all.AncB.resp_Rx - tag.poll_Tx);
      anchor_all.AncB.t_reply_tag = (int)(tag.final_Tx - anchor_all.AncB.resp_Rx);
      anchor_all.AncB.ranging_time = DW3000.ds_processRTInfo(anchor_all.AncB.t_round_tag, anchor_all.AncB.t_reply_tag, 
                                                             anchor_all.AncB.t_round_Anc,  anchor_all.AncB.t_reply_Anc,  anchor_all.AncB.clock_offset);
      anchor_all.AncB.distance = DW3000.convertToCM(anchor_all.AncB.ranging_time);

      // Anchor C
      anchor_all.AncC.t_round_tag = (int)(anchor_all.AncC.resp_Rx - tag.poll_Tx);
      anchor_all.AncC.t_reply_tag = (int)(tag.final_Tx - anchor_all.AncC.resp_Rx);
      anchor_all.AncC.ranging_time = DW3000.ds_processRTInfo(anchor_all.AncC.t_round_tag, anchor_all.AncC.t_reply_tag, 
                                                             anchor_all.AncC.t_round_Anc,  anchor_all.AncC.t_reply_Anc,  anchor_all.AncC.clock_offset);
      anchor_all.AncC.distance = DW3000.convertToCM(anchor_all.AncC.ranging_time);

      // Anchor D
      anchor_all.AncD.t_round_tag = (int)(anchor_all.AncD.resp_Rx - tag.poll_Tx);
      anchor_all.AncD.t_reply_tag = (int)(tag.final_Tx - anchor_all.AncD.resp_Rx);
      anchor_all.AncD.ranging_time = DW3000.ds_processRTInfo(anchor_all.AncD.t_round_tag, anchor_all.AncD.t_reply_tag, 
                                                             anchor_all.AncD.t_round_Anc,  anchor_all.AncD.t_reply_Anc,  anchor_all.AncD.clock_offset);
      anchor_all.AncD.distance = DW3000.convertToCM(anchor_all.AncD.ranging_time);

      
      if (DEBUG_PRINT) {
        Serial.print("[RESULT] Anchor A distance = "); Serial.println(anchor_all.AncA.distance);
        Serial.print("[RESULT] Anchor B distance = "); Serial.println(anchor_all.AncB.distance);
        Serial.print("[RESULT] Anchor C distance = "); Serial.println(anchor_all.AncC.distance);
        Serial.print("[RESULT] Anchor D distance = "); Serial.println(anchor_all.AncD.distance);
      }

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


void resetAnchorAll(AnchorAll &a) {
  a.AncA = Anchor();
  a.AncB = Anchor();
  a.AncC = Anchor();
  a.AncD = Anchor();
}

void resetTag(Tag &t) {
  t.poll_Tx = 0;
  t.final_Tx = 0;
}

bool saveResp(AnchorAll &anchors, Tag &tag, int sender_id, int resp_count, unsigned long long resp_Rx) {
  Anchor* target = nullptr;

  switch (sender_id) {
    case AnchorA_ID:
      target = &anchors.AncA;
      break;
    case AnchorB_ID:
      target = &anchors.AncB;
      break;
    case AnchorC_ID:
      target = &anchors.AncC;
      break;
    case AnchorD_ID:
      target = &anchors.AncD;
      break;
    default:
      Serial.print("[WARNING] Unknown sender=0x");
      Serial.println(sender_id, HEX);
      return false;
  }

  if (target->received_resp){
    Serial.print("[INFO] Duplicate response ignored from sender=0x");
    Serial.println(sender_id, HEX);
    return false;
  }

  target->resp_count = resp_count;
  target->resp_Rx = resp_Rx;
  target->received_resp = true;

  return true;

}

bool allRespReceived(const AnchorAll &anchors) {
  return anchors.AncA.received_resp &&
         anchors.AncB.received_resp &&
         anchors.AncC.received_resp &&
         anchors.AncD.received_resp;
}

bool saveReport(AnchorAll &anchors, int sender_id, int t_round, int t_reply, int clk_offset) {
  Anchor* target = nullptr;

  switch (sender_id) {
    case AnchorA_ID: target = &anchors.AncA; break;
    case AnchorB_ID: target = &anchors.AncB; break;
    case AnchorC_ID: target = &anchors.AncC; break;
    case AnchorD_ID: target = &anchors.AncD; break;
    default:
      Serial.print("[WARNING] Unknown anchor in Report sender=0x");
      Serial.println(sender_id, HEX);
      return false;
  }

  if (target->received_report) {
    Serial.print("[INFO] Duplicate Report ignored from sender=0x");
    Serial.println(sender_id, HEX);
    return false;
  }

  target->t_round_Anc = t_round;
  target->t_reply_Anc = t_reply;
  target->clock_offset = clk_offset;
  target->received_report = true;
  return true;
}

bool allReportsReceived(const AnchorAll &anchors) {
  return anchors.AncA.received_report &&
         anchors.AncB.received_report &&
         anchors.AncC.received_report &&
         anchors.AncD.received_report;
}

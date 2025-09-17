#include <Arduino.h>
#include "DW3000.h"

#define TAG_ID       0x01
#define ANCHOR_ID    0x0A      // 빌드 때 A/B/C/D로만 바꿔 업로드
#define BROADCAST_ID 0xFF

#define STAGE_POLL   1
#define STAGE_RESP   2
#define STAGE_FINAL  3
#define STAGE_REPORT 4

// 사용자 구조체(앵커 자신 상태 보관)
struct Anchor {
  int curr_stage = 0;
  unsigned long long poll_Rx = 0;   // t2
  unsigned long long resp_Tx = 0;   // t3
  unsigned long long final_Rx = 0;  // t6
  int t_round = 0;                  // t6 - t3
  int t_reply = 0;                  // t3 - t2
  int clock_offset = 0;             // (앵커에선 사용하지 않음)
  int ranging_time = 0;             // (앵커에선 사용하지 않음)
  float distance = 0;               // (앵커에선 사용하지 않음)
};
static Anchor selfA;

// 40b write helper
static void writeTS40(uint8_t off, unsigned long long ts){
  DW3000.write(TX_BUFFER_REG, off, (uint32_t)(ts & 0xFFFFFFFFULL));
  DW3000.write(TX_BUFFER_REG, off+4, (uint8_t)((ts>>32)&0xFF), 1);
}

// RESP 지연 송신 (라이브러리 상수/루틴 사용 가정)
static void sendDelayedRESP(unsigned long long rx_ts){
  int antenna_delay = DW3000.getTXAntennaDelay();
  uint32_t exact_tx_timestamp =
    (uint32_t)((rx_ts + TRANSMIT_DELAY) >> 8);
  unsigned long long calc_tx_timestamp =
    ((rx_ts + TRANSMIT_DELAY) & ~((unsigned long long)TRANSMIT_DIFF)) + (unsigned long long)antenna_delay;

  uint32_t reply_delay = (uint32_t)(calc_tx_timestamp - rx_ts);

  DW3000.setMode(1);
  DW3000.write(TX_BUFFER_REG, 0x01, ANCHOR_ID);
  DW3000.write(TX_BUFFER_REG, 0x02, TAG_ID);
  DW3000.write(TX_BUFFER_REG, 0x03, STAGE_RESP);
  DW3000.write(TX_BUFFER_REG, 0x04, reply_delay);  // 참고용
  DW3000.setFrameLength(8);
  DW3000.writeTXDelay(exact_tx_timestamp);
  DW3000.delayedTXThenRX();

  for(int i=0;i<50;i++){ if(DW3000.sentFrameSucc()) break; delay(1); }
  selfA.resp_Tx = DW3000.readTXTimestamp();   // t3
  selfA.curr_stage = STAGE_RESP;
}

// REPORT (앵커 로컬 3타임스탬프) 송신
static void sendREPORT(){
  DW3000.clearSystemStatus();
  DW3000.setMode(1);
  DW3000.write(TX_BUFFER_REG, 0x01, ANCHOR_ID);
  DW3000.write(TX_BUFFER_REG, 0x02, TAG_ID);
  DW3000.write(TX_BUFFER_REG, 0x03, STAGE_REPORT);
  writeTS40(0x04, selfA.poll_Rx);
  writeTS40(0x09, selfA.resp_Tx);
  writeTS40(0x0E, selfA.final_Rx);
  DW3000.setFrameLength(0x13);
  DW3000.standardTX();
}

void setup(){
  Serial.begin(115200); while(!Serial){}
  DW3000.begin(); DW3000.init(); DW3000.setupGPIO();
  DW3000.setSenderID(ANCHOR_ID);
  DW3000.setDestinationID(TAG_ID);
  DW3000.standardRX();
  Serial.print("[ANCHOR 0x"); Serial.print(ANCHOR_ID,HEX); Serial.println("] ready");
}

void loop(){
  int r = DW3000.receivedFrameSucc();
  if(r==1){
    uint8_t sid = DW3000.getSenderID() & 0xFF;
    if(sid==TAG_ID){
      uint8_t stage = DW3000.read8bit(RX_BUFFER_0_REG, 0x03);
      if(stage==STAGE_POLL){
        selfA.poll_Rx = DW3000.readRXTimestamp(); // t2
        selfA.curr_stage = STAGE_POLL;
        DW3000.clearSystemStatus();
        sendDelayedRESP(selfA.poll_Rx);           // t3 기록됨
      }else if(stage==STAGE_FINAL){
        selfA.final_Rx = DW3000.readRXTimestamp();// t6
        selfA.t_round  = (int)(selfA.final_Rx - selfA.resp_Tx); // t6 - t3
        selfA.t_reply  = (int)(selfA.resp_Tx  - selfA.poll_Rx); // t3 - t2
        selfA.curr_stage = STAGE_FINAL;
        DW3000.clearSystemStatus();
        sendREPORT();                              // t2,t3,t6 전달
        DW3000.standardRX();
      }else{
        DW3000.clearSystemStatus();
        DW3000.standardRX();
      }
    }else{
      DW3000.clearSystemStatus();
      DW3000.standardRX();
    }
  }else if(r==2){
    DW3000.clearSystemStatus();
    DW3000.standardRX();
  }else{
    delay(1);
  }
}

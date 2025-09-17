// Anchor - Responder / Multi-anchor PONG
#include "DW3000.h"

#define DEBUG_PRINT 1
#define BAUDRATE 2000000

/*
  단계 정의(네 코드와 동일 의미)
  1 - Poll 수신 (TAG→ALL)
  2 - Response 송신 (ANCHOR→TAG)
  3 - Final 수신 (TAG→ALL)
  4 - Report 송신 (ANCHOR→TAG)
*/

// ───────── 앵커 역할 선택 ─────────
// 빌드 시 -DANCHOR_ROLE='A' 처럼 지정해서 각 보드 A/B/C/D로 올리면 됨
#ifndef ANCHOR_ROLE
#define ANCHOR_ROLE 'A'                 // 'A' / 'B' / 'C' / 'D'
#endif

// ───────── 앵커 ID 매핑 ─────────
#define ANCHOR_A_ID 0x0A
#define ANCHOR_B_ID 0x0B
#define ANCHOR_C_ID 0x0C
#define ANCHOR_D_ID 0x0D

#if   (ANCHOR_ROLE=='A')
  #define THIS_ANCHOR_ID ANCHOR_A_ID
#elif (ANCHOR_ROLE=='B')
  #define THIS_ANCHOR_ID ANCHOR_B_ID
#elif (ANCHOR_ROLE=='C')
  #define THIS_ANCHOR_ID ANCHOR_C_ID
#elif (ANCHOR_ROLE=='D')
  #define THIS_ANCHOR_ID ANCHOR_D_ID
#else
  #error "ANCHOR_ROLE must be 'A','B','C','D'"
#endif

// ───────── 내부 상태 ─────────
static int  curr_stage = 0;                 // 간단한 상태기계(네 코드와 동일)
static int  rx_status  = 0;

static unsigned long long rx_ts = 0;        // 마지막 RX timestamp
static unsigned long long tx_ts = 0;        // 마지막 TX timestamp

static int t_roundB = 0;                    // (t6 - t3)
static int t_replyB = 0;                    // (t3 - t2)

static int last_tag_id = -1;                // 이번 라운드를 시작한 TAG ID

void setup()
{
  Serial.begin(BAUDRATE);
  DW3000.begin();
  DW3000.hardReset(); delay(200);

  if(!DW3000.checkSPI()){
    Serial.println("[ERROR] SPI 연결 실패");
    while(1);
  }
  while (!DW3000.checkForIDLE()){
    Serial.println("[ERROR] IDLE1 실패"); delay(1000);
  }

  DW3000.softReset(); delay(200);
  if (!DW3000.checkForIDLE()){
    Serial.println("[ERROR] IDLE2 실패"); while(1);
  }

  DW3000.init();
  DW3000.setupGPIO();

  // 이 보드의 송신자 ID만 설정 (TAG ID는 고정하지 않음!)
  DW3000.setSenderID(THIS_ANCHOR_ID);
  DW3000.configureAsTX();

  DW3000.clearSystemStatus();
  DW3000.standardRX();

  Serial.print("> DS-TWR PONG (Role ");
  Serial.print((char)ANCHOR_ROLE);
  Serial.print(", ID=0x"); Serial.print(THIS_ANCHOR_ID, HEX);
  Serial.println(") <");
  Serial.println("[INFO] setup 완료");
}

void loop()
{
  switch (curr_stage) {
    // 0) 대기: Poll(stage=1) 수신 대기
    case 0:
      t_roundB = 0; t_replyB = 0;
      if ( (rx_status = DW3000.receivedFrameSucc()) ) {
        DW3000.clearSystemStatus();
        if (rx_status == 1) {
          if (DW3000.ds_isErrorFrame()) {
            if (DEBUG_PRINT) Serial.println("[WARN] Error frame → stay 0");
            DW3000.standardRX();
            curr_stage = 0;
          } else if (DW3000.ds_getStage() != 1) {
            // Poll이 아니면 무시(다른 라운드의 잔여 프레임일 수 있음)
            if (DEBUG_PRINT){
              Serial.print("[INFO] stage "); Serial.print(DW3000.ds_getStage());
              Serial.println(" ignored in state 0");
            }
            DW3000.ds_sendErrorFrame();     // 필요 없다면 제거 가능
            DW3000.standardRX();
            curr_stage = 0;
          } else {
            // 정상 Poll 수신
            last_tag_id = DW3000.getSenderID();   // 이번 라운드의 TAG
            rx_ts = DW3000.readRXTimestamp();     // = t2
            if (DEBUG_PRINT){
              Serial.println("-----------------------------------------");
              Serial.println("----------------!!START!!----------------");
              Serial.print("[POLL] from TAG=0x"); Serial.println(last_tag_id, HEX);
            }
            curr_stage = 1;
          }
        } else {
          if (DEBUG_PRINT) Serial.println("[ERROR] RX Error → stay 0");
        }
      } else {
        // nothing; 계속 대기
      }
      break;

    // 1) Response(stage=2) 송신
    case 1:
      // 목적지: 방금 Poll을 보낸 TAG
      if (last_tag_id < 0) { curr_stage = 0; break; }

      // (충돌 회피가 필요하면 여기에서 역할별로 지연슬롯 추가 가능)
      // 예: if(ANCHOR_ROLE=='B') delayMicroseconds(300); ...

      DW3000.setDestinationID(last_tag_id);
      if (DEBUG_PRINT) Serial.println("[RESP] send");
      DW3000.ds_sendFrame(2);               // stage 2: RESP
      // t3는 방금 TX timestamp
      tx_ts = DW3000.readTXTimestamp();     // = t3
      // t2는 직전 Poll 수신 시각(rx_ts)
      t_replyB = (int)(tx_ts - rx_ts);      // = t3 - t2

      curr_stage = 2;
      break;

    // 2) Final(stage=3) 수신 대기
    case 2:
      if ( (rx_status = DW3000.receivedFrameSucc()) ) {
        DW3000.clearSystemStatus();
        if (rx_status == 1) {
          if (DW3000.ds_isErrorFrame()) {
            if (DEBUG_PRINT) Serial.println("[WARN] Error frame → reset");
            DW3000.standardRX();
            curr_stage = 0;
          } else if (DW3000.ds_getStage() != 3) {
            // Final이 아닌 다른 프레임(다른 라운드/기기) → 무시
            DW3000.ds_sendErrorFrame();     // 필요 없다면 제거 가능
            DW3000.standardRX();
            curr_stage = 0;
          } else {
            // Final 수신
            int this_sender = DW3000.getSenderID();
            if (last_tag_id >= 0 && this_sender != last_tag_id) {
              // 다른 TAG에서 온 Final이면 이번 라운드 무효화
              if (DEBUG_PRINT) {
                Serial.print("[INFO] Final from other TAG 0x");
                Serial.print(this_sender, HEX);
                Serial.print(" (expected 0x");
                Serial.print(last_tag_id, HEX);
                Serial.println(") → reset");
              }
              DW3000.standardRX();
              curr_stage = 0;
              break;
            }
            rx_ts = DW3000.readRXTimestamp();   // = t6
            if (DEBUG_PRINT){
              Serial.print("[FINAL] from TAG=0x"); Serial.println(this_sender, HEX);
            }
            curr_stage = 3;
          }
        } else {
          if (DEBUG_PRINT) Serial.println("[ERROR] RX Error in state 2 → reset");
          DW3000.clearSystemStatus();
          curr_stage = 0;
        }
      }
      break;

    // 3) Report(stage=4) 송신 (t_roundB, t_replyB)
    case 3:
      // t6 - t3
      t_roundB = (int)(rx_ts - tx_ts);

      // 목적지: 이번 라운드의 TAG
      if (last_tag_id < 0) { curr_stage = 0; break; }
      DW3000.setDestinationID(last_tag_id);

      if (DEBUG_PRINT){
        Serial.print("[REPORT] send  t_roundB="); Serial.print(t_roundB);
        Serial.print("  t_replyB="); Serial.println(t_replyB);
      }
      DW3000.ds_sendRTInfo(t_roundB, t_replyB);  // 0x04=t_roundB, 0x08=t_replyB

      // 라운드 종료 → 초기 상태 복귀
      last_tag_id = -1;
      curr_stage  = 0;

      if (DEBUG_PRINT){
        Serial.println("-----------------!!END!!-----------------");
        Serial.println("-----------------------------------------");
      }
      break;

    default:
      if (DEBUG_PRINT){
        Serial.print("[ERROR] unknown state "); Serial.println(curr_stage);
      }
      curr_stage = 0;
      DW3000.standardRX();
      break;
  }
}

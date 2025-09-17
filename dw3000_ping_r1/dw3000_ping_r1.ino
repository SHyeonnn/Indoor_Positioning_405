// Tag - Initiator / Multi-Anchor Broadcast Ping
#include "DW3000.h"

#define DEBUG_PRINT 1
#define BAUDRATE 2000000

#define TAG_ID 0xA0   // 브로드캐스트(0xFF)나 앵커(0x0A~0x0D)와 겹치지 않는 값

#define RESP_COLLECT_MS    120   // POLL 이후 RESP 모으는 시간
#define REPORT_COLLECT_MS  200   // FINAL 이후 REPORT 모으는 시간
#define RX_POLL_MS           2   // RX 폴링 간격
#define ROUND_DELAY        500   // 라운드 간 간격

// 앵커 ID (A,B,C,D)
static const uint8_t ANCHOR_ID[4] = { 0x0A, 0x0B, 0x0C, 0x0D };

// ── 최소 필드 구조체 ──
struct PerAnchor {
  unsigned long long t4_resp_Rx = 0; // t4
  int t_roundB = 0;                  // 앵커가 보낸 t6 - t3
  int t_replyB = 0;                  // 앵커가 보낸 t3 - t2
  int clk_raw  = 0;                  // 위 RESP 수신 순간 raw clock offset
  int range_ps = 0;                  // 계산된 one-way 시간(칩 단위)
};
struct TagState {
  unsigned long long t1_poll_Tx  = 0;  // POLL TX (t1)
  unsigned long long t5_final_Tx = 0;  // FINAL TX (t5)
  PerAnchor anc[4];                    // A,B,C,D 슬롯
} tag;

// 
static inline int idToIdx(uint8_t id){
  for(int i=0;i<4;i++) if(id==ANCHOR_ID[i]) return i;
  return -1;
}
static inline void resetRound(){
  tag.t1_poll_Tx = 0;
  tag.t5_final_Tx = 0;
  for(int i=0;i<4;i++) tag.anc[i] = PerAnchor{}; // 모두 0으로 초기화
}

void setup() {
  Serial.begin(BAUDRATE);
  DW3000.begin();
  DW3000.hardReset();
  delay(200);
  if(!DW3000.checkSPI()){
    Serial.println("[ERROR] SPI 연결 실패");
    while(1); }
  while (!DW3000.checkForIDLE()){ Serial.println("[ERROR] IDLE 실패");
  delay(200); }
  DW3000.softReset(); delay(200);
  if (!DW3000.checkForIDLE()){ Serial.println("[ERROR] IDLE 재시도 실패");
  while(1); }

  DW3000.init();
  DW3000.setupGPIO();
  DW3000.setSenderID(TAG_ID);
  DW3000.clearSystemStatus();

  Serial.println("[INFO] Multi-Anchor DS-TWR (TAG)");
  Serial.println("[INFO] setup 완료");
}

void loop() {
  resetRound();
  // 1) POLL 브로드캐스트 (stage=1)
  DW3000.setDestinationID(0xFF);          // 브로드캐스트
  DW3000.ds_sendFrame(1);                 // stage 1: POLL
  if (DEBUG_PRINT) Serial.println("\n--- POLL → Broadcast");
  tag.t1_poll_Tx = DW3000.readTXTimestamp(); // t1

  // 2) RESP 수집 (stage=2)
  unsigned long long deadline = millis() + RESP_COLLECT_MS;
  while ((long)(millis() - deadline) < 0){
    int r = DW3000.receivedFrameSucc();   // 1=성공,2=에러,0=없음
    
    if (r == 1 && DW3000.ds_getStage() == 2){ // stage 2: RESPONSE
      uint8_t sid = DW3000.getSenderID() & 0xFF;
      int idx = idToIdx(sid);
      if (idx >= 0 && tag.anc[idx].t4_resp_Rx == 0){
        tag.anc[idx].t4_resp_Rx = DW3000.readRXTimestamp(); // t4 저장
        tag.anc[idx].clk_raw = DW3000.getRawClockOffset();
        if (DEBUG_PRINT){
          Serial.print("[RESP] from 0x"); Serial.println(sid, HEX);
        }
      }
      DW3000.clearSystemStatus(); DW3000.standardRX();
    } else if (r == 2){
      DW3000.clearSystemStatus(); DW3000.standardRX();
    } else {
      delay(RX_POLL_MS);
    }
  }

  // 3) FINAL 브로드캐스트 (stage=3)
  DW3000.setDestinationID(0xFF);
  DW3000.ds_sendFrame(3);                 // stage 3: FINAL
  if (DEBUG_PRINT) Serial.println("--- FINAL → Broadcast");
  tag.t5_final_Tx = DW3000.readTXTimestamp(); // t5

  // 4) REPORT 수집 (stage=4)
  deadline = millis() + REPORT_COLLECT_MS;
  while ((long)(millis() - deadline) < 0){
    int r = DW3000.receivedFrameSucc();
    if (r == 1 && DW3000.ds_getStage() == 4){  //
      uint8_t sid = DW3000.getSenderID() & 0xFF;
      int idx = idToIdx(sid);
      if (idx >= 0){
        // ds_sendRTInfo() 포맷: 0x04=t_roundB, 0x08=t_replyB
        tag.anc[idx].t_roundB = (int)DW3000.read(RX_BUFFER_0_REG, 0x04);
        tag.anc[idx].t_replyB = (int)DW3000.read(RX_BUFFER_0_REG, 0x08);

        if (DEBUG_PRINT){
          Serial.print("[REPORT] from 0x"); Serial.print(sid, HEX);
          Serial.print("  t_roundB="); Serial.print(tag.anc[idx].t_roundB);
          Serial.print("  t_replyB="); Serial.println(tag.anc[idx].t_replyB);
        }
      }
      DW3000.clearSystemStatus(); DW3000.standardRX();
    } else if (r == 2){
      DW3000.clearSystemStatus(); DW3000.standardRX();
    } else {
      delay(RX_POLL_MS);
    }
  }

  // 5) 앵커별 계산 & 출력
  for (int i=0;i<4;i++){
    if (!tag.anc[i].t4_resp_Rx){          // RESPONSE 못 받은 경우
      if (DEBUG_PRINT){
        Serial.print("[RESULT] 0x"); Serial.print(ANCHOR_ID[i], HEX);
        Serial.println("  RESP missing → skip");
      }
      continue;
    }
    if (tag.anc[i].t_roundB==0 && tag.anc[i].t_replyB==0){ // REPORT 못 받은 경우
      if (DEBUG_PRINT){
        Serial.print("[RESULT] 0x"); Serial.print(ANCHOR_ID[i], HEX);
        Serial.println("  REPORT missing → skip");
      }
      continue;
    }

    int t_roundA = (int)(tag.anc[i].t4_resp_Rx - tag.t1_poll_Tx);  // t4 - t1
    int t_replyA = (int)(tag.t5_final_Tx - tag.anc[i].t4_resp_Rx); // t5 - t4

    tag.anc[i].range_ps = DW3000.ds_processRTInfo(
      t_roundA, t_replyA,
      tag.anc[i].t_roundB, tag.anc[i].t_replyB,
      tag.anc[i].clk_raw);

    double dist_cm = DW3000.convertToCM(tag.anc[i].range_ps);

    Serial.print("[RESULT] anc 0x"); Serial.print(ANCHOR_ID[i], HEX);
    Serial.print("  roundA="); Serial.print(t_roundA);
    Serial.print("  replyA="); Serial.print(t_replyA);
    Serial.print("  roundB="); Serial.print(tag.anc[i].t_roundB);
    Serial.print("  replyB="); Serial.print(tag.anc[i].t_replyB);
    Serial.print("  dist(cm)="); Serial.println(dist_cm, 2);
  }

  if (DEBUG_PRINT) Serial.println("--- round done ---");
  delay(ROUND_DELAY);
}

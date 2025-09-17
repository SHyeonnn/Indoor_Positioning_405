#include <Arduino.h>
#include "DW3000.h"

// ===== ID & Stage =====
#define TAG_ID        0x01
#define ANCHOR_A_ID   0x0A
#define ANCHOR_B_ID   0x0B
#define ANCHOR_C_ID   0x0C
#define ANCHOR_D_ID   0x0D
#define BROADCAST_ID  0xFF

#define STAGE_POLL    1
#define STAGE_RESP    2
#define STAGE_FINAL   3
#define STAGE_REPORT  4

// ===== Timeouts =====
#define RESP_COLLECT_MS    120
#define REPORT_COLLECT_MS  200
#define RX_POLL_MS         2

// ===== User structs (캡처 반영) =====
struct Anchor {
  int curr_stage = 0;
  unsigned long long poll_Rx = 0;
  unsigned long long resp_Tx = 0;
  unsigned long long final_Rx = 0;
  int t_round = 0;                 // t_roundB = t6 - t3
  int t_reply = 0;                 // t_replyB = t3 - t2
  int clock_offset = 0;            // TAG가 RESP 수신 당시 읽은 raw offset 저장처
  int ranging_time = 0;            // 계산 결과(원하면 사용)
  float distance = 0;              // 계산 결과
};
struct AnchorAll { Anchor AncA, AncB, AncC, AncD; };

struct AnchorResp {
  unsigned long long AncA_resp_Rx = 0;
  unsigned long long AncB_resp_Rx = 0;
  unsigned long long AncC_resp_Rx = 0;
  unsigned long long AncD_resp_Rx = 0;
};
struct Tag {
  unsigned long long poll_Tx = 0;
  AnchorResp anchor_resp;          // 앵커별 RESP 수신시각(t4들)
  unsigned long long final_Tx = 0;
  int t_round = 0;                 // 최근 계산(anchor 기준)의 t_roundA = t4 - t1
  int t_reply = 0;                 // 최근 계산(anchor 기준)의 t_replyA = t5 - t4
};

// ===== Global =====
static AnchorAll g_ancAll;
static Tag       g_tag;

// ===== Small helpers =====
static inline int idToIdx(uint8_t id){
  if(id==ANCHOR_A_ID) return 0;
  if(id==ANCHOR_B_ID) return 1;
  if(id==ANCHOR_C_ID) return 2;
  if(id==ANCHOR_D_ID) return 3;
  return -1;
}
static inline Anchor* ancByIdx(int i){
  switch(i){ case 0: return &g_ancAll.AncA; case 1: return &g_ancAll.AncB;
             case 2: return &g_ancAll.AncC; case 3: return &g_ancAll.AncD; }
  return nullptr;
}
static inline unsigned long long readTS40_fromRX(uint8_t off){
  uint32_t low  = DW3000.read(RX_BUFFER_0_REG, off);
  uint8_t  high = DW3000.read8bit(RX_BUFFER_0_REG, off+4);
  return ((unsigned long long)high<<32) | low;
}

// REPORT(payload: 0x04..0x08=poll_Rx, 0x09..0x0D=resp_Tx, 0x0E..0x12=final_Rx) → Anchor 채우기
static void parseReportToAnchor(Anchor& A){
  A.poll_Rx  = readTS40_fromRX(0x04);
  A.resp_Tx  = readTS40_fromRX(0x09);
  A.final_Rx = readTS40_fromRX(0x0E);
  A.t_round  = (int)(A.final_Rx - A.resp_Tx);  // t6 - t3
  A.t_reply  = (int)(A.resp_Tx  - A.poll_Rx);  // t3 - t2
  A.curr_stage = STAGE_REPORT;
}

// per-anchor 계산 → Anchor.distance에 기록
static bool computeFor(int idx){
  Anchor* A = ancByIdx(idx); if(!A) return false;

  // 태그 공통 송신시각
  unsigned long long t1 = g_tag.poll_Tx;
  unsigned long long t5 = g_tag.final_Tx;
  if(!t1 || !t5) return false;

  // 앵커별 RESP 수신시각(t4) 선택
  unsigned long long t4 = 0;
  if(idx==0) t4 = g_tag.anchor_resp.AncA_resp_Rx;
  if(idx==1) t4 = g_tag.anchor_resp.AncB_resp_Rx;
  if(idx==2) t4 = g_tag.anchor_resp.AncC_resp_Rx;
  if(idx==3) t4 = g_tag.anchor_resp.AncD_resp_Rx;
  if(!t4) return false;

  // 태그 관측 기반
  int t_roundA = (int)(t4 - t1);   // Tag: t4 - t1
  int t_replyA = (int)(t5 - t4);   // Tag: t5 - t4
  g_tag.t_round = t_roundA;        // (최근 계산 앵커 기준) 로그용 저장
  g_tag.t_reply = t_replyA;

  // 앵커 관측 기반(이미 REPORT 파싱 시 채워둠)
  int t_roundB = A->t_round;       // Anchor: t6 - t3
  int t_replyB = A->t_reply;       // Anchor: t3 - t2

  // clock offset(태그가 RESP 수신 당시 읽어 해당 앵커 슬롯에 저장해둠)
  int clk_raw   = A->clock_offset;

  // 편도 ToF(칩 시간단위) → cm
  int tof_units = DW3000.ds_processRTInfo(t_roundA, t_replyA, t_roundB, t_replyB, clk_raw);
  A->ranging_time = tof_units;
  A->distance = DW3000.convertToCM(tof_units);
  return true;
}

void setup(){
  Serial.begin(115200);
  while(!Serial){}
  DW3000.begin(); DW3000.init(); DW3000.setupGPIO();
  DW3000.setSenderID(TAG_ID);
  DW3000.setDestinationID(BROADCAST_ID);
  Serial.println("[TAG] ready");
}

void loop(){
  // 1) POLL broadcast
  DW3000.clearSystemStatus();
  DW3000.setMode(1);
  DW3000.write(TX_BUFFER_REG, 0x01, TAG_ID);
  DW3000.write(TX_BUFFER_REG, 0x02, BROADCAST_ID);
  DW3000.write(TX_BUFFER_REG, 0x03, STAGE_POLL);
  DW3000.setFrameLength(4);
  DW3000.TXInstantRX();
  for(int i=0;i<50;i++){ if(DW3000.sentFrameSucc()) break; delay(1); }
  g_tag.poll_Tx = DW3000.readTXTimestamp();

  // 2) RESP 수집 (각 앵커의 t4 + clock_offset을 해당 Anchor 슬롯에 저장)
  bool gotResp[4]={0,0,0,0};
  uint32_t t0 = millis();
  while(millis()-t0 < RESP_COLLECT_MS){
    int r = DW3000.receivedFrameSucc();
    if(r==1){
      uint8_t sid = DW3000.getSenderID() & 0xFF;
      int idx = idToIdx(sid);
      if(idx>=0){
        unsigned long long rxTS = DW3000.readRXTimestamp();
        int clk = DW3000.getRawClockOffset();  // RESP 수신 순간
        if(idx==0) g_tag.anchor_resp.AncA_resp_Rx = rxTS;
        if(idx==1) g_tag.anchor_resp.AncB_resp_Rx = rxTS;
        if(idx==2) g_tag.anchor_resp.AncC_resp_Rx = rxTS;
        if(idx==3) g_tag.anchor_resp.AncD_resp_Rx = rxTS;
        Anchor* A = ancByIdx(idx);
        if(A) A->clock_offset = clk;          // per-anchor로 보관
        gotResp[idx]=true;
      }
      DW3000.clearSystemStatus(); DW3000.standardRX();
    }else if(r==2){
      DW3000.clearSystemStatus(); DW3000.standardRX();
    }else{
      delay(RX_POLL_MS);
    }
    if(gotResp[0] && gotResp[1] && gotResp[2] && gotResp[3]) break;
  }

  // 3) FINAL broadcast
  DW3000.clearSystemStatus();
  DW3000.setMode(1);
  DW3000.write(TX_BUFFER_REG, 0x01, TAG_ID);
  DW3000.write(TX_BUFFER_REG, 0x02, BROADCAST_ID);
  DW3000.write(TX_BUFFER_REG, 0x03, STAGE_FINAL);
  DW3000.setFrameLength(4);
  DW3000.TXInstantRX();
  for(int i=0;i<50;i++){ if(DW3000.sentFrameSucc()) break; delay(1); }
  g_tag.final_Tx = DW3000.readTXTimestamp();

  // 4) REPORT 수집 → AnchorAll 채우기
  bool gotReport[4]={0,0,0,0};
  t0 = millis();
  while(millis()-t0 < REPORT_COLLECT_MS){
    int r = DW3000.receivedFrameSucc();
    if(r==1){
      uint8_t sid = DW3000.getSenderID() & 0xFF;
      int idx = idToIdx(sid);
      if(idx>=0){
        uint8_t stage = DW3000.read8bit(RX_BUFFER_0_REG, 0x03);
        if(stage==STAGE_REPORT){
          Anchor* A = ancByIdx(idx);
          if(A){ parseReportToAnchor(*A); gotReport[idx]=true; }
        }
      }
      DW3000.clearSystemStatus(); DW3000.standardRX();
    }else if(r==2){
      DW3000.clearSystemStatus(); DW3000.standardRX();
    }else{
      delay(RX_POLL_MS);
    }
    if(gotReport[0] && gotReport[1] && gotReport[2] && gotReport[3]) break;
  }

  // 5) per-anchor 거리 계산 & 출력
  const uint8_t ids[4]={ANCHOR_A_ID,ANCHOR_B_ID,ANCHOR_C_ID,ANCHOR_D_ID};
  for(int i=0;i<4;i++){
    bool ok = computeFor(i);
    Serial.print("[TAG] anc 0x"); Serial.print(ids[i],HEX);
    if(ok){
      Serial.print(" dist(cm)="); Serial.print(ancByIdx(i)->distance,2);
      Serial.print(" tA(round,reply)="); Serial.print(g_tag.t_round); Serial.print(","); Serial.print(g_tag.t_reply);
      Serial.print(" tB(round,reply)="); Serial.print(ancByIdx(i)->t_round); Serial.print(","); Serial.println(ancByIdx(i)->t_reply);
    }else{
      Serial.println(" not-ready");
    }
  }
  delay(300);
}

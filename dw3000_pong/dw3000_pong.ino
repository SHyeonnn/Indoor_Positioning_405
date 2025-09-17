// dw3000_doublesided_ranging_pong.ino
#include "DW3000.h"

#define DEBUG_PRINT 1

// ---- 보드마다 여기만 바꾸면 됨 ----
#define ANCHOR_ID   'A'   // 'B' or 'C' or 'D'

// ---- IDs ----
#define TAG_ID        0xA0
#define BROADCAST_ID  0xFF

// 앵커별 Response 슬롯 지연(us) - 충돌 완화
static inline uint16_t resp_slot_us() {
  switch (ANCHOR_ID) {
    case 'A': return 0;
    case 'B': return 300;
    case 'C': return 600;
    case 'D': return 900;
    default:  return 0;
  }
}

static unsigned long long poll_rx_ts  = 0;
static unsigned long long resp_tx_ts  = 0;
static unsigned long long final_rx_ts = 0;

void setup() {
  Serial.begin(2000000);
  DW3000.begin();
  DW3000.hardReset();
  delay(200);

  if (!DW3000.checkSPI()) { Serial.println("[ERROR] SPI fail"); while(1){} }
  while (!DW3000.checkForIDLE()) delay(10);

  DW3000.softReset(); delay(200);
  if (!DW3000.checkForIDLE()) { Serial.println("[ERROR] IDLE2"); while(1){} }

  DW3000.init();
  DW3000.setupGPIO();
  DW3000.setSenderID(ANCHOR_ID);
  DW3000.configureAsTX();
  DW3000.clearSystemStatus();
  DW3000.standardRX();

  Serial.print("> Anchor "); Serial.write(ANCHOR_ID); Serial.println(" ready <");
}

void loop() {
  int rx_status = DW3000.receivedFrameSucc();
  if (!rx_status) return;

  DW3000.clearSystemStatus();
  if (DW3000.ds_isErrorFrame()) { DW3000.standardRX(); return; }

  int stage  = DW3000.ds_getStage();
  int dest   = DW3000.getDestinationID();

  // 1) Poll(Broadcast) 수신 → Response(TX)
  if (stage == 1 && (dest == BROADCAST_ID || dest == ANCHOR_ID)) {
    poll_rx_ts = DW3000.readRXTimestamp();

    uint16_t slot = resp_slot_us();
    if (slot) delayMicroseconds(slot);   // 간단 슬롯팅

    DW3000.setDestinationID(TAG_ID);
    if (DEBUG_PRINT) { Serial.print("RESP->TAG from "); Serial.write(ANCHOR_ID); Serial.println(); }
    DW3000.ds_sendFrame(2);
    resp_tx_ts = DW3000.readTXTimestamp();

    DW3000.standardRX();
    return;
  }

  // 2) Final(Broadcast) 수신 → Report(TX)
  if (stage == 3 && (dest == BROADCAST_ID || dest == ANCHOR_ID)) {
    final_rx_ts = DW3000.readRXTimestamp();

    // Report payload: poll_rx, resp_tx, final_rx (각 4B LSB)
    DW3000.setMode(1);                                   // DS 모드 헤더 수동 구성
    DW3000.write(TX_BUFFER_REG, 0x01, ANCHOR_ID & 0xFF); // sender
    DW3000.write(TX_BUFFER_REG, 0x02, TAG_ID & 0xFF);    // dest
    DW3000.write(TX_BUFFER_REG, 0x03, 4);                // stage=4

    DW3000.write(TX_BUFFER_REG, 0x04, (uint32_t)(poll_rx_ts  & 0xFFFFFFFF));
    DW3000.write(TX_BUFFER_REG, 0x08, (uint32_t)(resp_tx_ts  & 0xFFFFFFFF));
    DW3000.write(TX_BUFFER_REG, 0x0C, (uint32_t)(final_rx_ts & 0xFFFFFFFF));

    DW3000.setFrameLength(16); // 1+1+1+1 + 12
    if (DEBUG_PRINT) { Serial.print("REPORT->TAG from "); Serial.write(ANCHOR_ID); Serial.println(); }
    DW3000.standardTX();
    DW3000.standardRX();
    return;
  }

  DW3000.standardRX();
}

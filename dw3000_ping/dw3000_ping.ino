// dw3000_doublesided_ranging_ping.ino
#include "DW3000.h"

#define DEBUG_PRINT 1

#define TAG_ID        0xA0
#define BROADCAST_ID  0xFF

#define ANCHOR_A 'A'
#define ANCHOR_B 'B'
#define ANCHOR_C 'C'
#define ANCHOR_D 'D'
static const uint8_t kAnchors[4] = { ANCHOR_A, ANCHOR_B, ANCHOR_C, ANCHOR_D };

// 타임아웃/주기 (환경에 맞게 조정)
#define RESP_COLLECT_TIMEOUT_MS    50
#define REPORT_COLLECT_TIMEOUT_MS  80
#define CYCLE_DELAY_MS             120

// 구조체 저장
typedef struct {
  char     anchor_id;     // 'A'/'B'/'C'/'D'
  uint32_t poll_rx_ts;    // (anchor측) Poll Rx (LSB32)
  uint32_t resp_tx_ts;    // (anchor측) Response Tx (LSB32)
  uint32_t final_rx_ts;   // (anchor측) Final Rx (LSB32)
  bool     valid;
} AnchorReport;

static AnchorReport reports[4];

static void resetReports() {
  for (int i=0;i<4;i++){ reports[i] = { (char)kAnchors[i], 0,0,0, false }; }
}
static int idxOf(char id) {
  for (int i=0;i<4;i++) if (reports[i].anchor_id == id) return i;
  return -1;
}

void setup() {
  Serial.begin(2000000);
  DW3000.begin();
  DW3000.hardReset(); delay(200);

  if (!DW3000.checkSPI()) { Serial.println("[ERROR] SPI fail"); while(1){} }
  while (!DW3000.checkForIDLE()) delay(10);

  DW3000.softReset(); delay(200);
  if (!DW3000.checkForIDLE()) { Serial.println("[ERROR] IDLE2"); while(1){} }

  DW3000.init();
  DW3000.setupGPIO();
  DW3000.setSenderID(TAG_ID);
  DW3000.configureAsTX();
  DW3000.clearSystemStatus();

  Serial.println("> Tag: Broadcast Poll/Final, collect 4x Response & Report <");
}

void loop() {
  resetReports();

  // 1) POLL Broadcast
  DW3000.setDestinationID(BROADCAST_ID);
  DW3000.ds_sendFrame(1);
  if (DEBUG_PRINT) Serial.println("TX: POLL (broadcast)");

  // Response 4개 수집
  bool gotResp[4] = {false,false,false,false};
  int  respCount = 0;
  uint32_t tStart = millis();

  while (respCount < 4 && (millis() - tStart) < RESP_COLLECT_TIMEOUT_MS) {
    int rx_status = DW3000.receivedFrameSucc();
    if (!rx_status) continue;

    DW3000.clearSystemStatus();
    if (DW3000.ds_isErrorFrame()) continue;

    int stage  = DW3000.ds_getStage();
    int sender = DW3000.getSenderID();
    int dest   = DW3000.getDestinationID();
    if (stage != 2 || dest != TAG_ID) continue;

    int idx = idxOf((char)sender);
    if (idx < 0 || gotResp[idx]) continue;

    gotResp[idx] = true;
    respCount++;
    if (DEBUG_PRINT) { Serial.print("RX: RESPONSE from "); Serial.write((char)sender); Serial.println(); }
  }
  if (respCount < 4) { Serial.print("[WARN] responses: "); Serial.println(respCount); }

  // 2) FINAL Broadcast
  DW3000.setDestinationID(BROADCAST_ID);
  DW3000.ds_sendFrame(3);
  if (DEBUG_PRINT) Serial.println("TX: FINAL (broadcast)");

  // Report 4개 수집
  int  reportCount = 0;
  tStart = millis();

  while (reportCount < 4 && (millis() - tStart) < REPORT_COLLECT_TIMEOUT_MS) {
    int rx_status = DW3000.receivedFrameSucc();
    if (!rx_status) continue;

    DW3000.clearSystemStatus();
    if (DW3000.ds_isErrorFrame()) continue;

    int stage  = DW3000.ds_getStage();
    int sender = DW3000.getSenderID();
    int dest   = DW3000.getDestinationID();
    if (stage != 4 || dest != TAG_ID) continue;

    int idx = idxOf((char)sender);
    if (idx < 0 || reports[idx].valid) continue;

    uint32_t poll_rx  = DW3000.read(RX_BUFFER_0_REG, 0x04);
    uint32_t resp_tx  = DW3000.read(RX_BUFFER_0_REG, 0x08);
    uint32_t final_rx = DW3000.read(RX_BUFFER_0_REG, 0x0C);

    reports[idx].poll_rx_ts  = poll_rx;
    reports[idx].resp_tx_ts  = resp_tx;
    reports[idx].final_rx_ts = final_rx;
    reports[idx].valid = true;
    reportCount++;

    if (DEBUG_PRINT) {
      Serial.print("RX: REPORT from "); Serial.write((char)sender);
      Serial.print("  poll_rx=");  Serial.print(poll_rx);
      Serial.print("  resp_tx=");  Serial.print(resp_tx);
      Serial.print("  final_rx="); Serial.println(final_rx);
    }
  }

  // 출력
  Serial.println("---- REPORTS ----");
  for (int i=0;i<4;i++) {
    Serial.print(reports[i].anchor_id); Serial.print(": ");
    if (reports[i].valid) {
      Serial.print("poll_rx=");  Serial.print(reports[i].poll_rx_ts);
      Serial.print("  resp_tx=");Serial.print(reports[i].resp_tx_ts);
      Serial.print("  final_rx=");Serial.print(reports[i].final_rx_ts);
      Serial.println();
    } else {
      Serial.println("missing");
    }
  }
  Serial.println("-----------------");

  delay(CYCLE_DELAY_MS);
}

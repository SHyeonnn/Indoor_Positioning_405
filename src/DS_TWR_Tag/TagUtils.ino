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

bool saveResp(AnchorAll &anchors, Tag &tag, int sender_id, unsigned long long resp_Rx) {
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
  target->resp_Rx = resp_Rx;
  target->received_resp = true;

  return true;

}

bool allRespReceived(const AnchorAll &anchors) {
  return anchors.AncA.received_resp &&
         anchors.AncB.received_resp; //&&
         //anchors.AncC.received_resp &&
         //anchors.AncD.received_resp;
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
         anchors.AncB.received_report; // &&
         //anchors.AncC.received_report &&
         //anchors.AncD.received_report;
}

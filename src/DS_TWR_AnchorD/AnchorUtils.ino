
void resetAnchorStruct(AnchorIn &anchor){

  anchor.poll_Rx = 0;
  anchor.resp_Tx = 0;
  anchor.final_Rx = 0;

  anchor.t_round = 0;
  anchor.t_reply = 0;

}

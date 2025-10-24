# main.py
# -*- coding: utf-8 -*-
import sys
sys.stdout.reconfigure(encoding='utf-8')

import socket
import struct
import time
import json
from datetime import datetime

import numpy as np

from uwb_receiver import UWBReceiver
from ToA_estimator import ToAEstimator

UDP_port = 8888

if __name__ == "__main__":
    # 기본 수신기 실행
    receiver = UWBReceiver(port=UDP_port, log_file="uwb_mean_data.jsonl")
    toa_estimator = ToAEstimator()
    print("UWB 거리 측정 데이터 수신을 시작합니다...")
    print("종료하려면 Ctrl+C를 누르세요.")
    
    try:
        while True:
            parsed_data = receiver.start_receiving()
            if parsed_data:
                # distances = parsed_data['distances']
                distances = parsed_data['mean_distances_m']
                print("mean_distances_m", distances)
                estimate_pos = toa_estimator.toa_LLS(distances)
                print("estimate_pos : ", estimate_pos)

    except KeyboardInterrupt:
        print("Program Exit")
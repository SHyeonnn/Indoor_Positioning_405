# uwb_receiver.py
# -*- coding: utf-8 -*-
import sys
sys.stdout.reconfigure(encoding='utf-8')

import socket
import struct
import time
import json
import numpy as np
from datetime import datetime
from collections import defaultdict

class UWBReceiver:
    def __init__(self, port=8888, log_file="uwb_mean_data.jsonl"):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('', port))
        self.log_file = log_file
        self.data_log = []
        
        self.distance_buffer = defaultdict(list) # key: anchor_id -> [distance, ...]
        self.packet_count = 0
        self.averaging_window = 50  # 평균을 계산할 데이터 개수
        
        print(f"=== UWB 데이터 수신 서버 시작 ===")
        print(f"포트: {port}")
        print(f"평균 로그 파일(JSONL): {self.log_file}")
        # print(f"수신 가능한 노드 수: 2~5개")
        print("=" * 40)
    
    def parse_uwb_data(self, data):
        """바이너리 데이터를 파싱하여 UWB 거리 정보 추출"""
        if len(data) < 6:
            print(f"데이터 크기 부족: {len(data)} bytes")
            return None
        
        try:
            # 헤더 파싱
            timestamp = struct.unpack('<I', data[0:4])[0]     # little-endian uint32
            node_id = struct.unpack('<B', data[4:5])[0]       # uint8
            target_count = struct.unpack('<B', data[5:6])[0]  # uint8
            
            # 데이터 유효성 검사
            if target_count < 1 or target_count > 4:  # 최대 4개 타겟 (5-1)
                print(f"잘못된 타겟 수: {target_count}")
                return None
            expected_size = 6 + (target_count * 5)  # 헤더 6바이트 + 각 타겟 5바이트
            if len(data) < expected_size:
                print(f"데이터 크기 불일치: 예상 {expected_size}, 실제 {len(data)}")
                return None
            
            # 거리 데이터 파싱
            distances = []
            offset = 6
            
            for _ in range(target_count):
                target_id = struct.unpack('<B', data[offset:offset+1])[0]
                distance = struct.unpack('<f', data[offset+1:offset+5])[0]  # little-endian float
                distances.append({
                    'target_id': target_id,
                    'distance': round(distance, 3)
                })
                offset += 5
            return {
                'timestamp': timestamp,
                'node_id': node_id,
                'target_count': target_count,
                'distances': distances,
                'received_at': datetime.now().isoformat()
            }

        except struct.error as e:
            print(f"데이터 파싱 오류: {e}")
            return None
        except Exception as e:
            print(f"예상치 못한 오류: {e}")
            return None

    def display_data(self, parsed_data, hex_data):
        """파싱된 데이터를 콘솔에 출력"""
        if not parsed_data:
            return
        
        print(f"\n=== 새 데이터 수신 ===")
        print(f"\n원본 데이터 ({len(hex_data.split())} bytes): {hex_data}")
        print(f"수신 시간: {parsed_data['received_at']}")
        print(f"TAG 노드: U{parsed_data['node_id']}")
        print(f"측정 시간: {parsed_data['timestamp']}ms")
        print(f"측정된 노드 수: {parsed_data['target_count']}")
        
        print("거리 측정 결과:")
        for dist_info in parsed_data['distances']:
            target_id = dist_info['target_id']
            distance = dist_info['distance']
            print(f"  U{parsed_data['node_id']} → U{target_id}: {distance:.3f}m")
        
        print("-" * 30)
        
    # 평균 누적
    def _accumulate_for_mean(self, parsed_data):
        """앵커가 4개인 패킷만 누적"""
        if not parsed_data:
            return
        dlist = parsed_data['distances']
        if len(dlist) != 4:
            return
        outlier = False
        # anchor_id 오름차순으로 정렬하여 일관된 순서 유지
        for i, d in enumerate(dlist):
            distance = d['distance']
            if distance > 100:
                print("Outlier")
                outlier = True
 
        if outlier == False:
            for item in sorted(dlist, key=lambda x: x['target_id']):
                self.distance_buffer[item['target_id']].append(item['distance'])
                self.packet_count += 1
        
    # 평균 요약 저장
    def _save_mean_summary(self, anchor_ids, means, ns):
        """평균 결과를 JSONL로 1줄 저장"""
        anchor_ids = np.asarray(anchor_ids, dtype=int).tolist()
        means      = np.asarray(means, dtype=float).tolist()
        ns         = np.asarray(ns, dtype=int).tolist()

        if not (len(anchor_ids) == len(means) == len(ns)):
            print(f"[평균 로그 저장 경고] 길이 불일치: ids={len(anchor_ids)}, means={len(means)}, ns={len(ns)}")
            return
        
        summary = {
            "timestamp": datetime.now().isoformat(),
            "window_size": int(self.averaging_window),
            "anchor_ids": anchor_ids,
            "mean_distances_m": means,
            "samples_per_anchor": ns
        }
        try:
            with open(self.log_file, "a", encoding="utf-8") as f:
                f.write(json.dumps(summary, ensure_ascii=False) + "\n")
        except Exception as e:
            print(f"[평균 로그 저장 오류] {e}")
        return summary


    def _finalize_window(self):
        """
        현재 버퍼에 쌓인 값으로 평균 계산하고
        - 파일에 저장
        - 내부 상태 초기화
        - (anchor_ids, means, ns) 반환
        """
        sorted_anchor_ids = np.array(sorted(self.distance_buffer.keys()), dtype=int)

        means, ns = [], []
        for aid in sorted_anchor_ids:
            arr = np.asarray(self.distance_buffer[aid], dtype=float)
            if arr.size == 0:
                continue
            means.append(float(arr.mean()))
            ns.append(int(arr.size))

        # 콘솔 출력(선택)
        print("\n" + "=" * 50)
        print(f"{self.averaging_window}개 수집 완료! 앵커별 평균 거리:")
        for aid, m, n in zip(sorted_anchor_ids, means, ns):
            print(f"  - 앵커 U{aid}: mean={m:.3f} m (n={n})")
        print("=" * 50 + "\n")

        # 파일 저장(평균만 저장)
        final_value = self._save_mean_summary(sorted_anchor_ids, means, ns)

        # 다음 윈도우를 위해 초기화
        self.packet_count = 0
        self.distance_buffer.clear()
        return final_value
    
    def start_receiving(self, position_callback=None):
        print(f"{self.averaging_window}개 모아 평균 후 main으로 반환합니다...")
        while True:
            data, _ = self.sock.recvfrom(1024)
            parsed = self.parse_uwb_data(data)
            if not parsed:
                print("데이터 파싱 실패")
                continue

                # 보기(원하면 주석처리)
                # hex_data = ' '.join(f'{b:02X}' for b in data)
                # self.display_data(parsed, hex_data)
            
            # 누적
            self._accumulate_for_mean(parsed)

            # 윈도우 완료되면 평균을 계산하고 즉시 반환
            if self.packet_count >= self.averaging_window:
                print(f"{self.averaging_window}개 패킷 수집 완료, 평균 계산 중...")
                return self._finalize_window()
            
# 실시간 모니터링용 클래스
class UWBMonitor:
    def __init__(self):
        self.last_positions = {}
        self.measurement_count = 0
    
    def update_positions(self, parsed_data):
        """위치 정보 업데이트 및 통계"""
        if not parsed_data:
            return
        
        node_id = parsed_data['node_id']
        self.measurement_count += 1
        
        # 거리 정보 저장
        distances = {}
        for dist_info in parsed_data['distances']:
            distances[dist_info['target_id']] = dist_info['distance']
        
        self.last_positions[node_id] = {
            'timestamp': parsed_data['timestamp'],
            'distances': distances,
            'measurement_count': self.measurement_count
        }
    
    def print_status(self):
        """현재 상태 출력"""
        print(f"\n=== 모니터링 상태 ===")
        print(f"총 측정 횟수: {self.measurement_count}")
        
        for node_id, info in self.last_positions.items():
            print(f"U{node_id} 최신 측정:")
            for target_id, distance in info['distances'].items():
                print(f"  → U{target_id}: {distance:.3f}m")


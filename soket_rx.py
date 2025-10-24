# -*- coding: utf-8 -*-
import sys
sys.stdout.reconfigure(encoding='utf-8')

import socket
import struct
import time
import json
from datetime import datetime

class UWBReceiver:
    def __init__(self, port=8888, log_file="uwb_data.json"):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('', port))
        self.log_file = log_file
        self.data_log = []
        
        print(f"=== UWB 데이터 수신 서버 시작 ===")
        print(f"포트: {port}")
        print(f"로그 파일: {log_file}")
        print(f"수신 가능한 노드 수: 2~5개")
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
            if target_count > 4 or target_count < 1:  # 최대 4개 타겟 (5-1)
                print(f"잘못된 타겟 수: {target_count}")
                return None
            
            expected_size = 6 + (target_count * 5)  # 헤더 6바이트 + 각 타겟 5바이트
            if len(data) < expected_size:
                print(f"데이터 크기 불일치: 예상 {expected_size}, 실제 {len(data)}")
                return None
            
            # 거리 데이터 파싱
            distances = []
            offset = 6
            
            for i in range(target_count):
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
    
    def display_data(self, parsed_data):
        """파싱된 데이터를 콘솔에 출력"""
        if not parsed_data:
            return
        
        print(f"\n=== 새 데이터 수신 ===")
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
    
    def save_to_log(self, parsed_data):
        """데이터를 JSON 파일에 저장"""
        if not parsed_data:
            return
        
        self.data_log.append(parsed_data)
        
        # 100개 데이터마다 파일에 저장
        if len(self.data_log) >= 100:
            try:
                with open(self.log_file, 'a') as f:
                    for data in self.data_log:
                        f.write(json.dumps(data) + '\n')
                self.data_log = []
                print(f"데이터 100개를 {self.log_file}에 저장했습니다.")
            except Exception as e:
                print(f"파일 저장 오류: {e}")
    
    def get_network_config_message(self, target_count):
        """노드 수에 따른 네트워크 구성 메시지"""
        configs = {
            1: "TAG(U1) ↔ ANCHOR(U2)",
            2: "TAG(U1) ↔ ANCHOR(U2, U3)",
            3: "TAG(U1) ↔ ANCHOR(U2, U3, U4)",
            4: "TAG(U1) ↔ ANCHOR(U2, U3, U4, U5)"
        }
        return configs.get(target_count, f"알 수 없는 구성 (타겟 수: {target_count})")
    
    def start_receiving(self):
        """메인 수신 루프"""
        try:
            while True:
                # UDP 데이터 수신
                data, addr = self.sock.recvfrom(1024)
                
                # 16진수로 원본 데이터 출력 (디버깅용)
                hex_data = ' '.join(f'{b:02X}' for b in data)
                print(f"\n원본 데이터 ({len(data)} bytes): {hex_data}")
                
                # 데이터 파싱
                parsed_data = self.parse_uwb_data(data)
                
                if parsed_data:
                    # 네트워크 구성 정보 출력
                    config_msg = self.get_network_config_message(parsed_data['target_count'])
                    print(f"네트워크 구성: {config_msg}")
                    
                    # 콘솔 출력
                    self.display_data(parsed_data)
                    
                    # 파일 저장
                    self.save_to_log(parsed_data)
                else:
                    print("데이터 파싱 실패")
                
        except KeyboardInterrupt:
            print("\n\n=== 수신 종료 ===")
            # 남은 데이터 저장
            if self.data_log:
                try:
                    with open(self.log_file, 'a') as f:
                        for data in self.data_log:
                            f.write(json.dumps(data) + '\n')
                    print(f"남은 데이터 {len(self.data_log)}개를 저장했습니다.")
                except Exception as e:
                    print(f"마지막 저장 중 오류: {e}")
            
        except Exception as e:
            print(f"수신 중 오류 발생: {e}")
        
        finally:
            self.sock.close()
            print("소켓 연결 종료")

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

if __name__ == "__main__":
    # 기본 수신기 실행
    receiver = UWBReceiver(port=8888, log_file="uwb_measurements.json")
    
    print("UWB 거리 측정 데이터 수신을 시작합니다...")
    print("종료하려면 Ctrl+C를 누르세요.")
    
    receiver.start_receiving()
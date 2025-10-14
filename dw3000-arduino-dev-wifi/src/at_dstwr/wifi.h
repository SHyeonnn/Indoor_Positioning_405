#ifndef WIFI_H
#define WIFI_H

#include <stdint.h>

#ifdef WIFI_ENABLED
#define MAX_TARGETS 5

#pragma pack(push, 1)
// 바이너리 데이터 구조체
struct DistanceData {
    uint8_t target_id;
    float distance;
} __attribute__((packed));

struct UWBData {
    uint32_t timestamp;
    uint8_t node_id;
    uint8_t target_count;
    DistanceData distances[MAX_TARGETS];
} __attribute__((packed));

#pragma pack(pop)

// WiFi 함수 선언
void setup_wifi();
bool is_wifi_connected();
void send_distance_data(uint32_t timestamp, uint8_t node_id, uint8_t target_count, 
                       uint8_t* target_ids, float* distances);
void send_simple_distance_data(uint32_t timestamp, uint8_t node_id, 
                              uint8_t target_id, float distance);
void wifi_reconnect();
void check_wifi_status();

#else
// WiFi가 비활성화된 경우 빈 함수들 (인라인으로 처리)
inline void setup_wifi() {}
inline bool is_wifi_connected() { return false; }
inline void send_distance_data(uint32_t timestamp, uint8_t node_id, uint8_t target_count, 
                              uint8_t* target_ids, float* distances) {}
inline void send_simple_distance_data(uint32_t timestamp, uint8_t node_id, 
                                     uint8_t target_id, float distance) {}
inline void wifi_reconnect() {}
inline void check_wifi_status() {}
#endif

#endif // WIFI_H
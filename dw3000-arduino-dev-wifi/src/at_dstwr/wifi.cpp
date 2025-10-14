#include "at_dstwr/wifi.h"

#ifdef WIFI_ENABLED
#include <WiFi.h>
#include <WiFiUdp.h>

// WiFi 설정
const char* ssid = "I2Coinz_Lab_2.4G";
const char* password = "11223344";
const char* raspberry_pi_ip = "192.168.0.118";  // 라즈베리파이 IP
const int udp_port = 8888;

WiFiUDP udp;

void setup_wifi() {
    WiFi.begin(ssid, password);
    Serial.print("WiFi 연결 중");
    
    while (WiFi.status() != WL_CONNECTED) {
        delay(100);
        Serial.print(".");
    }
    
    Serial.println();
    Serial.println("WiFi 연결 완료");
    Serial.print("IP 주소: ");
    Serial.println(WiFi.localIP());
}

bool is_wifi_connected() {
    return WiFi.status() == WL_CONNECTED;
}

void send_distance_data(uint32_t timestamp, uint8_t node_id, uint8_t target_count, 
                       uint8_t* target_ids, float* distances) {
    static unsigned long last_send_time = 0;
    const unsigned long MIN_SEND_INTERVAL = 50; // 50ms 최소 간격
    
    unsigned long now = millis();
    if (now - last_send_time < MIN_SEND_INTERVAL) {
        return; // 너무 빨리 전송하려고 하면 스킵
    }
    last_send_time = now;
    
    if (!is_wifi_connected()) {
        Serial.println("WiFi 연결 끊어짐");
        return;
    }
    
    UWBData data;
    data.timestamp = timestamp;
    data.node_id = node_id;
    data.target_count = target_count;
    
    for (int i = 0; i < target_count && i < MAX_TARGETS; i++) {
        data.distances[i].target_id = target_ids[i];
        data.distances[i].distance = distances[i];
    }
    
    // UDP로 바이너리 데이터 전송
    if (udp.beginPacket(raspberry_pi_ip, udp_port)) {
        size_t data_size = sizeof(uint32_t) + sizeof(uint8_t) * 2 + 
                          target_count * sizeof(DistanceData);
        udp.write((uint8_t*)&data, data_size);
        
        if (udp.endPacket()) {
            Serial.print("데이터 전송 완료 (");
            Serial.print(data_size);
            Serial.println(" bytes)");
        } else {
            Serial.println("UDP 전송 실패");
        }
    } else {
        Serial.println("UDP 패킷 시작 실패");
    }
}

void send_simple_distance_data(uint32_t timestamp, uint8_t node_id, 
                              uint8_t target_id, float distance) {
    uint8_t target_ids[1] = {target_id};
    float distances[1] = {distance};
    send_distance_data(timestamp, node_id, 1, target_ids, distances);
}

void wifi_reconnect() {
    if (!is_wifi_connected()) {
        Serial.println("WiFi 재연결 시도...");
        WiFi.disconnect();
        delay(100);
        setup_wifi();
    }
}

void check_wifi_status() {
    static unsigned long last_check = 0;
    unsigned long now = millis();
    
    // 5초마다 WiFi 상태 확인
    if (now - last_check > 5000) {
        if (!is_wifi_connected()) {
            Serial.println("WiFi 연결 상태 확인 중...");
            wifi_reconnect();
        }
        last_check = now;
    }
}

#endif // WIFI_ENABLED
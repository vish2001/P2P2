#include "wifi_udp_sender.h"

#if WIFI_ENABLED

WiFiUDPSender wifiSender;

WiFiUDPSender::WiFiUDPSender() {
    connected = false;
    last_send_time = 0;
    last_reconnect_attempt = 0;
}

bool WiFiUDPSender::begin() {
    Serial.println("[WIFI] Connecting to WiFi...");
    Serial.print("[WIFI] SSID: ");
    Serial.println(WIFI_SSID);
    
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    
    int timeout = 0;
    while (WiFi.status() != WL_CONNECTED && timeout < 20) {
        delay(500);
        Serial.print(".");
        timeout++;
    }
    Serial.println();
    
    if (WiFi.status() == WL_CONNECTED) {
        connected = true;
        Serial.println("[WIFI] Connected!");
        Serial.print("[WIFI] IP: ");
        Serial.println(WiFi.localIP());
        
        serverIP.fromString(UDP_SERVER_IP);
        Serial.print("[WIFI] Server: ");
        Serial.print(UDP_SERVER_IP);
        Serial.print(":");
        Serial.println(UDP_SERVER_PORT);
        
        udp.begin(UDP_SERVER_PORT + TAG_ID);
        return true;
    } else {
        connected = false;
        Serial.println("[WIFI] Connection failed!");
        return false;
    }
}

bool WiFiUDPSender::isConnected() {
    return connected && (WiFi.status() == WL_CONNECTED);
}

void WiFiUDPSender::checkConnection() {
    if (WiFi.status() != WL_CONNECTED) {
        connected = false;
        if (millis() - last_reconnect_attempt > 5000) {
            last_reconnect_attempt = millis();
            Serial.println("[WIFI] Reconnecting...");
            WiFi.reconnect();
        }
    } else if (!connected) {
        connected = true;
        Serial.println("[WIFI] Reconnected!");
    }
}

void WiFiUDPSender::sendPacket(const char* data) {
    if (!isConnected()) return;
    if (millis() - last_send_time < UDP_SEND_INTERVAL_MS) return;
    last_send_time = millis();
    
    udp.beginPacket(serverIP, UDP_SERVER_PORT);
    udp.print(data);
    udp.endPacket();
}

void WiFiUDPSender::sendRangingResult(uint8_t from_id, uint8_t to_id, 
                                       float distance_cm, float rssi, 
                                       uint32_t timestamp) {
    if (!isConnected()) return;
    
    char buf[128];
    snprintf(buf, sizeof(buf), "R,%d,%d,%.2f,%.1f,%lu",
             from_id, to_id, distance_cm, rssi, timestamp);
    sendPacket(buf);
}

void WiFiUDPSender::sendNeighborInfo(uint8_t node_id, uint8_t neighbor_id,
                                      uint8_t hello_count, uint8_t range_pct,
                                      float distance_cm, float rssi,
                                      bool is_stale, uint32_t stale_ms) {
    if (!isConnected()) return;
    
    char buf[128];
    snprintf(buf, sizeof(buf), "N,%d,%d,%d,%d,%.2f,%.1f,%d,%lu",
             node_id, neighbor_id, hello_count, range_pct,
             distance_cm, rssi, is_stale ? 1 : 0, stale_ms);
    sendPacket(buf);
}

void WiFiUDPSender::sendHeartbeat(uint8_t node_id, uint32_t frame_num,
                                   uint8_t neighbor_count, uint32_t uptime_ms) {
    if (!isConnected()) return;
    
    char buf[64];
    snprintf(buf, sizeof(buf), "H,%d,%lu,%d,%lu",
             node_id, frame_num, neighbor_count, uptime_ms);
    sendPacket(buf);
}

void WiFiUDPSender::sendIMUClassification(uint8_t node_id, int behavior,
                                           int confidence,
                                           float ax, float ay, float az,
                                           float gx, float gy, float gz,
                                           uint32_t timestamp,
                                           uint32_t range_age_ms,
                                           uint8_t nbr_count,
                                           uint8_t range_pct) {
    if (!isConnected()) return;
    
    char buf[250];
    snprintf(buf, sizeof(buf),
             "C,%d,%d,%d,%.2f,%.2f,%.2f,%.1f,%.1f,%.1f,%lu,%lu,%d,%d",
             node_id, behavior, confidence,
             ax, ay, az, gx, gy, gz,
             timestamp, range_age_ms, nbr_count, range_pct);
    sendPacket(buf);
}

#endif // WIFI_ENABLED

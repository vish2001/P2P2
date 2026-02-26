#pragma once

#include <Arduino.h>
#include "peer_config.h"

#if WIFI_ENABLED

#include <WiFi.h>
#include <WiFiUdp.h>

// =============================================================================
// WiFi UDP Sender for Ranging + Classification Data
// =============================================================================

class WiFiUDPSender {
public:
    WiFiUDPSender();
    
    bool begin();
    bool isConnected();
    void checkConnection();
    
    // Format: "R,<from_id>,<to_id>,<distance_cm>,<rssi>,<timestamp>"
    void sendRangingResult(uint8_t from_id, uint8_t to_id, 
                           float distance_cm, float rssi, uint32_t timestamp);
    
    // Format: "N,<node>,<nbr>,<hellos>,<range_pct>,<dist>,<rssi>,<stale>,<stale_ms>"
    void sendNeighborInfo(uint8_t node_id, uint8_t neighbor_id,
                          uint8_t hello_count, uint8_t range_pct,
                          float distance_cm, float rssi,
                          bool is_stale, uint32_t stale_ms);
    
    // Format: "H,<node_id>,<frame_num>,<neighbor_count>,<uptime_ms>"
    void sendHeartbeat(uint8_t node_id, uint32_t frame_num, 
                       uint8_t neighbor_count, uint32_t uptime_ms);

    // Format: "C,<node>,<behav>,<conf>,<ax>,<ay>,<az>,<gx>,<gy>,<gz>,<ts>,<range_age>,<nbrs>,<range_pct>"
    void sendIMUClassification(uint8_t node_id, int behavior, int confidence,
                               float ax, float ay, float az,
                               float gx, float gy, float gz,
                               uint32_t timestamp,
                               uint32_t range_age_ms, uint8_t nbr_count,
                               uint8_t range_pct);

private:
    WiFiUDP udp;
    IPAddress serverIP;
    bool connected;
    uint32_t last_send_time;
    uint32_t last_reconnect_attempt;
    
    void sendPacket(const char* data);
};

extern WiFiUDPSender wifiSender;

#endif // WIFI_ENABLED

#pragma once

#include <Arduino.h>
#include "peer_config.h"

#if ESP_NOW_ENABLED

#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>

class ESPNowSender {
public:
    ESPNowSender();

    bool begin();
    bool isConnected();
    void checkConnection();

    // Format: "R,<from>,<to>,<dist_cm>,<rssi>,<ts>"
    void sendRangingResult(uint8_t from_id, uint8_t to_id,
                           float distance_cm, float rssi, uint32_t timestamp);

    // Format: "N,<node>,<nbr>,<hellos>,<range_pct>,<dist>,<rssi>,<stale>,<stale_ms>"
    void sendNeighborInfo(uint8_t node_id, uint8_t neighbor_id,
                          uint8_t hello_count, uint8_t range_pct,
                          float distance_cm, float rssi,
                          bool is_stale, uint32_t stale_ms);

    // Format: "H,<node>,<frame>,<n_nbrs>,<uptime_ms>"
    void sendHeartbeat(uint8_t node_id, uint32_t frame_num,
                       uint8_t neighbor_count, uint32_t uptime_ms);

    // Format: "C,<node>,<behav>,<conf>,<ax>,<ay>,<az>,<gx>,<gy>,<gz>,<ts>,<range_age>,<nbrs>,<range_pct>"
    // behav: 0=Walking 1=Grazing 2=Resting 3=Misc
    // range_age: ms since last successful ranging (staleness indicator)
    // nbrs: current neighbor count
    // range_pct: overall ranging success rate
    void sendIMUClassification(uint8_t node_id, int behavior, int confidence,
                               float ax, float ay, float az,
                               float gx, float gy, float gz,
                               uint32_t timestamp,
                               uint32_t range_age_ms, uint8_t nbr_count,
                               uint8_t range_pct);

private:
    bool     initialized;
    uint8_t  base_mac[6];
    uint32_t last_send_time;
    uint32_t send_count;
    uint32_t fail_count;

    void sendPacket(const char* data);
    static void onSent(const uint8_t* mac, esp_now_send_status_t status);
};

extern ESPNowSender espnowSender;

#endif // ESP_NOW_ENABLED

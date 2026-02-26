#include "espnow_sender.h"

#if ESP_NOW_ENABLED

ESPNowSender espnowSender;

static const uint8_t BROADCAST_MAC[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

static volatile uint32_t _send_success = 0;
static volatile uint32_t _send_fail    = 0;

ESPNowSender::ESPNowSender() {
    initialized    = false;
    last_send_time = 0;
    send_count     = 0;
    fail_count     = 0;
    memcpy(base_mac, BROADCAST_MAC, 6);
}

bool ESPNowSender::begin() {
    Serial.println("[ESP-NOW] Initializing (broadcast mode)...");

    WiFi.mode(WIFI_STA);
    WiFi.disconnect();

    Serial.print("[ESP-NOW] Node MAC: ");
    Serial.println(WiFi.macAddress());

    esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);

    if (esp_now_init() != ESP_OK) {
        Serial.println("[ESP-NOW] Init failed!");
        return false;
    }

    esp_now_register_send_cb(onSent);

    esp_now_peer_info_t peer;
    memset(&peer, 0, sizeof(peer));
    memcpy(peer.peer_addr, BROADCAST_MAC, 6);
    peer.channel = 0;
    peer.encrypt = false;

    if (esp_now_add_peer(&peer) != ESP_OK) {
        Serial.println("[ESP-NOW] Failed to add broadcast peer");
        return false;
    }

    esp_wifi_set_max_tx_power(50);  // 12.5 dBm

    initialized = true;
    Serial.println("[ESP-NOW] Ready (broadcasting to all base stations)");
    return true;
}

bool ESPNowSender::isConnected() {
    return initialized;
}

void ESPNowSender::checkConnection() {
    if (!initialized) begin();
}

// =============================================================================
// SEND METHODS
// All respect ESP_NOW_MIN_INTERVAL rate limiter to avoid flooding.
// =============================================================================

void ESPNowSender::sendRangingResult(uint8_t from_id, uint8_t to_id,
                                      float distance_cm, float rssi,
                                      uint32_t timestamp) {
    if (!initialized) return;
    if (millis() - last_send_time < ESP_NOW_MIN_INTERVAL) return;

    char buf[250];
    snprintf(buf, sizeof(buf), "R,%d,%d,%.1f,%.1f,%lu",
             from_id, to_id, distance_cm, rssi, timestamp);
    sendPacket(buf);
}

void ESPNowSender::sendNeighborInfo(uint8_t node_id, uint8_t neighbor_id,
                                     uint8_t hello_count, uint8_t range_pct,
                                     float distance_cm, float rssi,
                                     bool is_stale, uint32_t stale_ms) {
    if (!initialized) return;
    if (millis() - last_send_time < ESP_NOW_MIN_INTERVAL) return;

    char buf[250];
    snprintf(buf, sizeof(buf), "N,%d,%d,%d,%d,%.1f,%.1f,%d,%lu",
             node_id, neighbor_id, hello_count, range_pct,
             distance_cm, rssi, is_stale ? 1 : 0, stale_ms);
    sendPacket(buf);
}

void ESPNowSender::sendHeartbeat(uint8_t node_id, uint32_t frame_num,
                                  uint8_t neighbor_count, uint32_t uptime_ms) {
    if (!initialized) return;
    if (millis() - last_send_time < ESP_NOW_MIN_INTERVAL) return;

    char buf[250];
    snprintf(buf, sizeof(buf), "H,%d,%lu,%d,%lu",
             node_id, frame_num, neighbor_count, uptime_ms);
    sendPacket(buf);
}

void ESPNowSender::sendIMUClassification(uint8_t node_id, int behavior,
                                          int confidence,
                                          float ax, float ay, float az,
                                          float gx, float gy, float gz,
                                          uint32_t timestamp,
                                          uint32_t range_age_ms,
                                          uint8_t nbr_count,
                                          uint8_t range_pct) {
    if (!initialized) return;
    if (millis() - last_send_time < ESP_NOW_MIN_INTERVAL) return;

    char buf[250];
    snprintf(buf, sizeof(buf),
             "C,%d,%d,%d,%.2f,%.2f,%.2f,%.1f,%.1f,%.1f,%lu,%lu,%d,%d",
             node_id, behavior, confidence,
             ax, ay, az, gx, gy, gz,
             timestamp, range_age_ms, nbr_count, range_pct);
    sendPacket(buf);
}

// =============================================================================
// INTERNAL SEND
// =============================================================================
void ESPNowSender::sendPacket(const char* data) {
    int len = strlen(data);
    if (len > 249) len = 249;

    esp_err_t result = esp_now_send(base_mac, (uint8_t*)data, len + 1);

    if (result == ESP_OK) {
        send_count++;
        last_send_time = millis();
    } else {
        fail_count++;
        if (DEBUG_OUTPUT) {
            Serial.print("[ESP-NOW] Send failed: ");
            Serial.println(result);
        }
    }
}

void ESPNowSender::onSent(const uint8_t* mac, esp_now_send_status_t status) {
    if (status == ESP_NOW_SEND_SUCCESS) {
        _send_success++;
    } else {
        _send_fail++;
    }
}

#endif // ESP_NOW_ENABLED

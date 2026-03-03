// =============================================================================
// UWB MESH BRIDGE — ESP32 Base Station Firmware
// =============================================================================
// Receives ESP-NOW unicast packets from mesh nodes and outputs them via:
//   • USB Serial  (always — Pi reads /dev/ttyUSBx)
//   • WiFi UDP    (optional — set OUTPUT_UDP=1, router must be on channel 1)
//
// DEPLOY MULTIPLE BRIDGES
//   Flash with different STATION_ID values via build_flags in platformio.ini.
//   All bridges can forward to the same Pi. The Pi deduplicates.
//
// OUTPUT LINE FORMAT
//   rx_ms,station_id,src_mac,type,fields…
//   Types:
//     R  rx_ms,sid,mac,R,from,to,dist_cm,rssi,node_ts
//     N  rx_ms,sid,mac,N,node,nbr,hellos,range_pct,dist_cm,rssi
//     H  rx_ms,sid,mac,H,node,frame,n_nbrs,uptime_ms
//     C  rx_ms,sid,mac,C,node,behav,conf,ax,ay,az,gx,gy,gz,node_ts
//        behav: 0=Walking 1=Grazing 2=Resting 3=Misc
// =============================================================================

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <esp_now.h>
#include <esp_wifi.h>

// =============================================================================
// CONFIGURATION — override via build_flags in platformio.ini
// =============================================================================

#ifndef STATION_ID
#define STATION_ID      1
#endif

#define OUTPUT_SERIAL   1       // Always forward via USB Serial

#ifndef OUTPUT_UDP
#define OUTPUT_UDP      0       // Set 1 for wireless UDP to Pi
#endif

// WiFi credentials (only used when OUTPUT_UDP = 1)
#ifndef WIFI_SSID
#define WIFI_SSID       "YourNetwork"
#endif
#ifndef WIFI_PASSWORD
#define WIFI_PASSWORD   "YourPassword"
#endif

// Raspberry Pi UDP target
#ifndef RPI_IP
#define RPI_IP          "192.168.1.100"
#endif
#ifndef RPI_UDP_PORT
#define RPI_UDP_PORT    5005
#endif

#define WIFI_RECONNECT_MS   15000
#define STATUS_INTERVAL_MS  30000

// =============================================================================
// RING BUFFER
// ESP-NOW callback runs in a WiFi task — must not call Serial/UDP directly.
// Push to ring buffer here, drain in loop().
// =============================================================================

#define RING_SIZE   64
#define LINE_MAX    300

static char  ring_buf[RING_SIZE][LINE_MAX];
static volatile int ring_head = 0;
static volatile int ring_tail = 0;

static inline int  ring_next(int i) { return (i + 1) % RING_SIZE; }
static inline bool ring_full()      { return ring_next(ring_head) == ring_tail; }
static inline bool ring_empty()     { return ring_head == ring_tail; }

// =============================================================================
// GLOBALS
// =============================================================================

WiFiUDP udp;

volatile uint32_t g_rx_count   = 0;
volatile uint32_t g_drop_count = 0;
uint32_t g_fwd_serial = 0;
uint32_t g_fwd_udp    = 0;

uint32_t last_status_ms   = 0;
uint32_t last_wifi_try_ms = 0;
bool     wifi_ok          = false;

// =============================================================================
// HELPERS
// =============================================================================

static void mac_to_str(const uint8_t* mac, char* out) {
    snprintf(out, 18, "%02X:%02X:%02X:%02X:%02X:%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

// =============================================================================
// ESP-NOW RECEIVE CALLBACK
// =============================================================================

void IRAM_ATTR onDataRecv(const uint8_t* mac, const uint8_t* data, int len) {
    g_rx_count++;
    if (len < 2 || len > 249) return;
    if (ring_full()) { g_drop_count++; return; }

    char mac_str[18];
    mac_to_str(mac, mac_str);

    char payload[250];
    memcpy(payload, data, len);
    payload[len] = '\0';

    snprintf(ring_buf[ring_head], LINE_MAX,
             "%lu,%d,%s,%s",
             (unsigned long)millis(), STATION_ID, mac_str, payload);

    ring_head = ring_next(ring_head);
}

// =============================================================================
// WiFi MANAGEMENT
// =============================================================================

void wifi_connect() {
    if (!OUTPUT_UDP) return;

    Serial.printf("# [WiFi] Connecting to '%s'...\n", WIFI_SSID);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    int tries = 20;
    while (WiFi.status() != WL_CONNECTED && tries-- > 0) {
        delay(500);
        Serial.print('#');
    }
    Serial.println();

    if (WiFi.status() == WL_CONNECTED) {
        wifi_ok = true;
        udp.begin(RPI_UDP_PORT);
        Serial.printf("# [WiFi] Connected — IP %s\n",
                      WiFi.localIP().toString().c_str());
        Serial.printf("# [WiFi] Forwarding UDP -> %s:%d\n", RPI_IP, RPI_UDP_PORT);
    } else {
        wifi_ok = false;
        Serial.println("# [WiFi] Failed — Serial only.");
    }

    // Re-lock channel to 1 after WiFi connects
    esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);

    last_wifi_try_ms = millis();
}

void wifi_check_reconnect() {
    if (!OUTPUT_UDP) return;
    if (WiFi.status() == WL_CONNECTED) { wifi_ok = true; return; }
    if (millis() - last_wifi_try_ms < WIFI_RECONNECT_MS) return;
    Serial.println("# [WiFi] Disconnected — reconnecting...");
    wifi_ok = false;
    WiFi.disconnect(true);
    delay(100);
    wifi_connect();
}

// =============================================================================
// SETUP
// =============================================================================

void setup() {
    Serial.begin(921600);
    delay(800);

    Serial.println();
    Serial.println("# ============================================");
    Serial.printf ("# UWB MESH BRIDGE  —  Station ID: %d\n", STATION_ID);
    Serial.println("# ============================================");
    Serial.printf ("# Output: Serial=%d  UDP=%d\n", OUTPUT_SERIAL, OUTPUT_UDP);

    // WiFi must be STA mode for ESP-NOW
    WiFi.mode(WIFI_STA);
    Serial.printf("# Bridge MAC: %s\n", WiFi.macAddress().c_str());

    // Connect to router first if UDP enabled (negotiates channel),
    // then init ESP-NOW, then re-lock channel to 1.
    if (OUTPUT_UDP) {
        wifi_connect();
    }

    // =========================================================================
    // FIX: esp_now_init() BEFORE esp_wifi_set_channel()
    // Calling set_channel before init is silently ignored on ESP32.
    // =========================================================================
    if (esp_now_init() != ESP_OK) {
        Serial.println("# [ESP-NOW] *** Init FAILED — halting ***");
        while (true) delay(1000);
    }

    // Lock channel to 1 AFTER init — nodes transmit on channel 1
    esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);

    // Confirm actual channel
    uint8_t ch; wifi_second_chan_t sec;
    esp_wifi_get_channel(&ch, &sec);
    Serial.printf("# [ESP-NOW] Channel locked to: %d\n", ch);

    if (ch != 1) {
        Serial.println("# [WARN] Channel is NOT 1 — nodes transmit on ch 1.");
        Serial.println("# [WARN] If using UDP, set your router 2.4GHz to channel 1.");
    }

    esp_now_register_recv_cb(onDataRecv);

    Serial.println("# [ESP-NOW] Listening for node packets...");
    Serial.println("# ============================================");
    Serial.println("#");
    Serial.println("# Line format: rx_ms,station_id,src_mac,type,fields...");
    Serial.println("# Types: R=Ranging  N=Neighbour  H=Heartbeat  C=Classification");
    Serial.println("#   R: rx_ms,sid,mac,R,from,to,dist_cm,rssi,node_ts");
    Serial.println("#   N: rx_ms,sid,mac,N,node,nbr,hellos,range_pct,dist_cm,rssi");
    Serial.println("#   H: rx_ms,sid,mac,H,node,frame,n_nbrs,uptime_ms");
    Serial.println("#   C: rx_ms,sid,mac,C,node,behav,conf,ax,ay,az,gx,gy,gz,node_ts");
    Serial.println("#      behav: 0=Walking 1=Grazing 2=Resting 3=Misc");
    Serial.println("#");

    last_status_ms = millis();
}

// =============================================================================
// LOOP — drain ring buffer, forward lines
// =============================================================================

void loop() {
    wifi_check_reconnect();

    while (!ring_empty()) {
        const char* line = ring_buf[ring_tail];

        if (OUTPUT_SERIAL) {
            Serial.println(line);
            g_fwd_serial++;
        }

        if (OUTPUT_UDP && wifi_ok) {
            udp.beginPacket(RPI_IP, RPI_UDP_PORT);
            udp.print(line);
            udp.print('\n');
            if (udp.endPacket()) g_fwd_udp++;
        }

        ring_tail = ring_next(ring_tail);
    }

    // Periodic status line (prefixed '#' so Pi logger ignores it)
    if (millis() - last_status_ms > STATUS_INTERVAL_MS) {
        Serial.printf(
            "# [STATUS] rx=%lu  serial=%lu  udp=%lu  drop=%lu  uptime=%lus  wifi=%s\n",
            (unsigned long)g_rx_count,
            (unsigned long)g_fwd_serial,
            (unsigned long)g_fwd_udp,
            (unsigned long)g_drop_count,
            (unsigned long)(millis() / 1000),
            wifi_ok ? WiFi.localIP().toString().c_str() : "offline"
        );
        last_status_ms = millis();
    }

    delay(2);
}
// =============================================================================
// UWB MESH BASE STATION - ESP-NOW Receiver (v2)
// =============================================================================
// Receives ranging + classification data from mesh nodes via ESP-NOW.
// Forwards to PC via Serial (USB) or WiFi UDP.
//
// DATA FORMAT (CSV to PC):
//   timestamp_ms,station_id,TYPE,fields...
//
// R (Ranging):    ts,st,R,from,to,distance_cm,rssi,node_uptime
// N (Neighbor):   ts,st,N,node,neighbor,hellos,range_pct,dist,rssi,stale,stale_ms
// H (Heartbeat):  ts,st,H,node,frame,neighbors,uptime
// C (Classify):   ts,st,C,node,behavior,confidence,ax,ay,az,gx,gy,gz,node_ts,range_age_ms,nbrs,range_pct
//
// behavior: 0=Walking, 1=Grazing, 2=Resting, 3=Misc
// stale:    0=fresh, 1=stale (link not updated within threshold)
// range_age_ms: ms since this node's last successful ranging (0xFFFFFFFF = never)
// =============================================================================

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

// =============================================================================
// CONFIGURATION
// =============================================================================

#define OUTPUT_SERIAL    1
#define OUTPUT_UDP       0

#define UDP_TARGET_IP    "192.168.1.100"
#define UDP_TARGET_PORT  5000

#define STATION_ID       1

#define WIFI_SSID        "YourNetwork"
#define WIFI_PASSWORD    "YourPassword"

#define LOG_CSV_FORMAT   1

// =============================================================================
// PER-NODE CLASSIFICATION TRACKER
// =============================================================================
// Tracks latest behavior per node for local display / alerting

#define MAX_TRACKED_NODES 32

static const char* BEH_NAMES[4] = {"Walking", "Grazing", "Resting", "Misc"};

struct NodeClassification {
    uint8_t  node_id;
    uint8_t  behavior;       // 0-3
    uint8_t  confidence;     // 0-3
    uint32_t last_update_ms; // base station clock
    uint32_t range_age_ms;   // how stale the node's ranging data is
    uint8_t  nbr_count;
    uint8_t  range_pct;
    bool     active;
};

static NodeClassification node_class[MAX_TRACKED_NODES];

static void updateClassification(uint8_t node_id, uint8_t behavior, uint8_t conf,
                                  uint32_t range_age, uint8_t nbrs, uint8_t rpct) {
    // Find existing or empty slot
    int slot = -1;
    for (int i = 0; i < MAX_TRACKED_NODES; i++) {
        if (node_class[i].active && node_class[i].node_id == node_id) {
            slot = i;
            break;
        }
        if (!node_class[i].active && slot < 0) slot = i;
    }
    if (slot < 0) return; // table full

    node_class[slot].node_id       = node_id;
    node_class[slot].behavior      = behavior;
    node_class[slot].confidence    = conf;
    node_class[slot].last_update_ms = millis();
    node_class[slot].range_age_ms  = range_age;
    node_class[slot].nbr_count     = nbrs;
    node_class[slot].range_pct     = rpct;
    node_class[slot].active        = true;
}

// =============================================================================
// GLOBALS
// =============================================================================

#if OUTPUT_UDP
#include <WiFiUdp.h>
WiFiUDP udp;
#endif

uint32_t packets_received  = 0;
uint32_t class_received    = 0;
uint32_t range_received    = 0;
uint32_t last_status_time  = 0;
uint32_t boot_time_ms      = 0;

// =============================================================================
// ESP-NOW CALLBACK
// =============================================================================

void onDataReceived(const uint8_t* mac, const uint8_t* data, int len) {
    packets_received++;
    
    if (len < 2) return;
    
    uint32_t timestamp_ms = millis();
    
    char incoming[256];
    memcpy(incoming, data, min(len, 255));
    incoming[min(len, 255)] = '\0';
    
    // Track packet types
    if (incoming[0] == 'C') {
        class_received++;
        
        // Parse and track classification locally
        // C,node,behav,conf,...,range_age,nbrs,range_pct
        int node_id = 0, behavior = 0, conf = 0;
        uint32_t range_age = 0;
        int nbrs = 0, rpct = 0;
        
        // Quick parse - just need fields 1,2,3 and 11,12,13
        char tmp[256];
        strncpy(tmp, incoming, sizeof(tmp));
        char* tok = strtok(tmp, ",");  // "C"
        int field = 0;
        while (tok) {
            tok = strtok(NULL, ",");
            field++;
            if (!tok) break;
            switch (field) {
                case 1: node_id  = atoi(tok); break;
                case 2: behavior = atoi(tok); break;
                case 3: conf     = atoi(tok); break;
                case 11: range_age = strtoul(tok, NULL, 10); break;
                case 12: nbrs    = atoi(tok); break;
                case 13: rpct    = atoi(tok); break;
            }
        }
        
        if (node_id > 0 && behavior >= 0 && behavior <= 3) {
            updateClassification(node_id, behavior, conf, range_age, nbrs, rpct);
        }
    } else if (incoming[0] == 'R') {
        range_received++;
    }
    
    // Forward to output
    char output[512];
    
#if LOG_CSV_FORMAT
    snprintf(output, sizeof(output), "%lu,%d,%s",
             timestamp_ms, STATION_ID, incoming);
#else
    snprintf(output, sizeof(output), "T,%lu,S,%d,%s",
             timestamp_ms, STATION_ID, incoming);
#endif
    
#if OUTPUT_SERIAL
    Serial.println(output);
#endif

#if OUTPUT_UDP
    udp.beginPacket(UDP_TARGET_IP, UDP_TARGET_PORT);
    udp.print(output);
    udp.endPacket();
#endif
}

// =============================================================================
// SETUP
// =============================================================================

void setup() {
    Serial.begin(921600);
    delay(1000);
    
    boot_time_ms = millis();
    memset(node_class, 0, sizeof(node_class));
    
    Serial.println();
    Serial.println("# ==========================================");
    Serial.println("#   UWB MESH BASE STATION v2");
    Serial.print("#   Station ID: ");
    Serial.println(STATION_ID);
    Serial.println("# ==========================================");
    
    WiFi.mode(WIFI_STA);
    
    Serial.print("# MAC: ");
    Serial.println(WiFi.macAddress());
    Serial.println("#");
    
#if LOG_CSV_FORMAT
    Serial.println("# CSV Format: timestamp_ms,station_id,type,data...");
    Serial.println("# Types: R=Ranging, N=Neighbor, H=Heartbeat, C=Classification");
    Serial.println("#");
    Serial.println("# R: ts,st,R,from,to,dist_cm,rssi,node_ts");
    Serial.println("# N: ts,st,N,node,nbr,hellos,range_pct,dist,rssi,stale,stale_ms");
    Serial.println("# H: ts,st,H,node,frame,nbr_count,uptime");
    Serial.println("# C: ts,st,C,node,behavior,conf,ax,ay,az,gx,gy,gz,node_ts,range_age_ms,nbrs,range_pct");
    Serial.println("#   behavior: 0=Walking 1=Grazing 2=Resting 3=Misc");
    Serial.println("#   stale: 0=fresh 1=stale");
    Serial.println("#   range_age_ms: ms since node's last successful range");
    Serial.println("#");
#endif
    
#if OUTPUT_UDP
    Serial.print("# [WIFI] Connecting to ");
    Serial.println(WIFI_SSID);
    
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    
    int timeout = 20;
    while (WiFi.status() != WL_CONNECTED && timeout > 0) {
        delay(500);
        Serial.print(".");
        timeout--;
    }
    
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println();
        Serial.println("# [WIFI] Connected!");
        Serial.print("# [WIFI] IP: ");
        Serial.println(WiFi.localIP());
        udp.begin(UDP_TARGET_PORT);
    } else {
        Serial.println();
        Serial.println("# [WIFI] Connection failed - UDP disabled");
    }
#endif
    
    if (esp_now_init() != ESP_OK) {
        Serial.println("# [ESP-NOW] Init failed!");
        return;
    }
    
    esp_now_register_recv_cb(onDataReceived);
    
    Serial.println("# [ESP-NOW] Ready - listening for mesh data...");
    Serial.println("# ==========================================");
    Serial.println();
    
    last_status_time = millis();
}

// =============================================================================
// LOOP
// =============================================================================

void loop() {
    // Status report every 30 seconds (prefixed with # so CSV parsers skip it)
    if (millis() - last_status_time > 30000) {
        Serial.print("# [STATUS] pkts=");
        Serial.print(packets_received);
        Serial.print(" range=");
        Serial.print(range_received);
        Serial.print(" class=");
        Serial.print(class_received);
        Serial.print(" uptime=");
        Serial.print((millis() - boot_time_ms) / 1000);
        Serial.println("s");
        
        // Print per-node classification summary
        bool any_active = false;
        for (int i = 0; i < MAX_TRACKED_NODES; i++) {
            if (!node_class[i].active) continue;
            
            uint32_t age_ms = millis() - node_class[i].last_update_ms;
            if (age_ms > 60000) {
                node_class[i].active = false; // prune stale entries
                continue;
            }
            
            if (!any_active) {
                Serial.println("# [CLASSIFY] Node behaviors:");
                any_active = true;
            }
            
            const char* bname = (node_class[i].behavior < 4) 
                                ? BEH_NAMES[node_class[i].behavior] : "?";
            
            bool range_stale = (node_class[i].range_age_ms > 10000 
                                || node_class[i].range_age_ms == 0xFFFFFFFF);
            
            Serial.printf("#   Node %3d: %-8s (conf=%d/3, nbrs=%d, range=%d%%, %s, class_age=%lus)\n",
                          node_class[i].node_id,
                          bname,
                          node_class[i].confidence,
                          node_class[i].nbr_count,
                          node_class[i].range_pct,
                          range_stale ? "RANGE_STALE" : "range_ok",
                          age_ms / 1000);
        }
        
        if (!any_active) {
            Serial.println("# [CLASSIFY] No active classifications");
        }
        
        last_status_time = millis();
    }
    
    delay(10);
}

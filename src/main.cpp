// =============================================================================
// PEER-TO-PEER UWB MESH SYSTEM + CATTLE BEHAVIOR CLASSIFIER (INTEGRATED)
// =============================================================================
//
// Combines:
//   - P2P UWB mesh ranging (DS-TWR, symmetric formula)
//   - BMI270 IMU cattle behavior classification (Decision Tree, 60 features)
//   - ESP-NOW / WiFi UDP telemetry to base station
//
// IMU runs non-blocking alongside UWB loop:
//   sampleIMU()         — every 100ms in LISTENING state only (~0.3ms)
//   runClassification() — every 2.5s when window fills (~2ms)
//
// Classification packets include freshness metadata so the base station
// knows how stale the data is:
//   - last_range_age_ms:  time since last successful ranging
//   - neighbor_count:     current mesh connectivity
//   - range_success_pct:  overall ranging health
//
// =============================================================================

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include "SparkFun_BMI270_Arduino_Library.h"
#include "peer_config.h"
#include "DW3000.h"
#include "neighbor_table.h"
#include "tdma_scheduler.h"
#include "range_logger.h"
#include "cattle_classifier.h"    // Decision Tree primary (60 features)

#if WIFI_ENABLED
#include "wifi_udp_sender.h"
#endif

#if ESP_NOW_ENABLED
#include "espnow_sender.h"
#endif

// =============================================================================
// IMU CONFIGURATION
// =============================================================================
#define IMU_SDA_PIN      21
#define IMU_SCL_PIN      22
#define G_TO_MS2         9.807f
#define SAMPLE_INTERVAL  100      // ms between IMU reads (10 Hz)
#define SAMPLES_PER_WIN  50       // 5 second window
#define OVERLAP_SAMPLES  25       // 50% overlap → classify every 2.5s

// =============================================================================
// GLOBAL STATE — UWB
// =============================================================================

uint8_t TAG_ID = 1;
DW3000Class uwb;

int ANTENNA_DELAY = ANTENNA_DELAY_DEFAULT;
int led_status    = 0;
int destination   = 0;
int sender        = 0;

enum class NodeState {
    LISTENING,
    INIT_SEND_POLL,
    INIT_WAIT_RESP,
    INIT_SEND_FINAL,
    INIT_WAIT_REPORT,
    INIT_CALCULATE,
    RESP_SEND_RESP,
    RESP_WAIT_FINAL,
    RESP_SEND_REPORT,
    HELLO_SEND,
};

NodeState state = NodeState::LISTENING;
uint32_t  state_entry_time = 0;
uint8_t   retry_count = 0;

struct RangingSession {
    uint8_t  target_id;
    long long tx_poll;
    long long rx_resp;
    long long tx_final;
    uint32_t  t_round_a;
    uint32_t  t_reply_a;
    int       clock_offset;
    bool      is_initiator;
    long long rx_poll;
    long long tx_resp;
    long long rx_final;
    uint32_t  t_round_b;
    uint32_t  t_reply_b;
    uint32_t  report_t_round_b;
    uint32_t  report_t_reply_b;
} session;

uint8_t hello_seq   = 0;
uint8_t lbt_retries = 0;

#define BROADCAST_ID 0xFF

// Track last successful ranging time (for staleness reporting)
static uint32_t last_range_success_ms = 0;

// =============================================================================
// GLOBAL STATE — IMU
// =============================================================================

BMI270 imu;
bool imu_ok = false;

static float buf_ax[SAMPLES_PER_WIN], buf_ay[SAMPLES_PER_WIN], buf_az[SAMPLES_PER_WIN];
static float buf_gx[SAMPLES_PER_WIN], buf_gy[SAMPLES_PER_WIN], buf_gz[SAMPLES_PER_WIN];
static int      buf_pos   = 0;
static int      buf_count = 0;
static int      new_samples = 0;
static uint32_t last_sample_ms = 0;

static CattleVoter voter;
static int   current_behavior   = -1;
static int   current_confidence = 0;
static float latest_ax = 0, latest_ay = 0, latest_az = 0;
static float latest_gx = 0, latest_gy = 0, latest_gz = 0;

// =============================================================================
// GLOBAL STATE — BATTERY MONITOR
// =============================================================================
#if BATTERY_MONITOR_ENABLED
static float    battery_voltage     = 4.2f;  // Assume full until first read
static uint32_t last_batt_check_ms  = 0;
static uint8_t  critical_count      = 0;     // Consecutive critical readings
static bool     battery_warned      = false;
static bool     battery_shutdown    = false;
#endif

// =============================================================================
// FORWARD DECLARATIONS
// =============================================================================
void initializeUWB();
void initializeIMU();
void resetRadio();
void hardResetDW();
void generateNodeID();
bool performLBT();
bool checkStateTimeout(uint32_t timeout_ms);
void transitionTo(NodeState new_state);
void enterListeningMode();

void sampleIMU();
void runClassification();

#if BATTERY_MONITOR_ENABLED
float readBatteryVoltage();
void  checkBattery();
void  batteryShutdown(float voltage);
#endif

void handleListeningState();
void handleInitSendPoll();
void handleInitWaitResp();
void handleInitSendFinal();
void handleInitWaitReport();
void handleInitCalculate();
void handleRespSendResp();
void handleRespWaitFinal();
void handleRespSendReport();
void handleHelloSend();

void broadcastNeighborTable();
void printStatus();

// =============================================================================
// AUTO-ADDRESSING FROM MAC
// =============================================================================
void generateNodeID() {
#if TAG_ID_OVERRIDE > 0
    TAG_ID = TAG_ID_OVERRIDE;
    Serial.print("[ID] Override: ");
    Serial.println(TAG_ID);
#else
    uint64_t mac = ESP.getEfuseMac();
    Serial.print("[MAC] ");
    for (int i = 0; i < 6; i++) {
        uint8_t byte = (mac >> (i * 8)) & 0xFF;
        if (byte < 0x10) Serial.print("0");
        Serial.print(byte, HEX);
        if (i < 5) Serial.print(":");
    }
    Serial.println();
    uint32_t hash = 2166136261;
    for (int i = 0; i < 6; i++) {
        uint8_t byte = (mac >> (i * 8)) & 0xFF;
        hash ^= byte;
        hash *= 16777619;
    }
    TAG_ID = (hash % 253) + 1;
    Serial.print("[ID] Auto-generated: ");
    Serial.println(TAG_ID);
#endif
  scheduler.recalculateSlot();
  Serial.print("[SLOT] ");
  Serial.print(scheduler.getMySlot());
  Serial.print("/");
  Serial.println(NUM_SLOTS - 1);
}

// =============================================================================
// IMU INITIALIZATION
// =============================================================================
void initializeIMU() {
    Serial.println("[IMU] Initializing BMI270...");
    Wire.begin(IMU_SDA_PIN, IMU_SCL_PIN);
    Wire.setClock(400000);

    if (imu.beginI2C(BMI2_I2C_PRIM_ADDR) == BMI2_OK) {
        imu_ok = true;
        Serial.println("[IMU] BMI270 OK");

        int pass = cc_selftest();
        Serial.printf("[IMU] Classifier self-test: %d/4 (MLP primary)\n", pass);

        imu.getSensorData();
        Serial.printf("[IMU] az = %.2f m/s2\n", imu.data.accelZ * G_TO_MS2);
    } else {
        imu_ok = false;
        Serial.println("[IMU] BMI270 not found — classification disabled, UWB continues");
    }
    cc_voter_init(&voter);
}

// =============================================================================
// IMU SAMPLING — non-blocking, LISTENING state only
// I2C read takes ~300-500us. Gated to LISTENING to avoid corrupting
// DW3000 SPI timestamps during active DS-TWR exchanges.
// =============================================================================
void sampleIMU() {
    if (!imu_ok) return;
    uint32_t now = millis();
    if (now - last_sample_ms < SAMPLE_INTERVAL) return;
    last_sample_ms = now;

    imu.getSensorData();

    float ax = imu.data.accelX * G_TO_MS2;
    float ay = imu.data.accelY * G_TO_MS2;
    float az = imu.data.accelZ * G_TO_MS2;
    float gx = imu.data.gyroX;
    float gy = imu.data.gyroY;
    float gz = imu.data.gyroZ;

    latest_ax = ax; latest_ay = ay; latest_az = az;
    latest_gx = gx; latest_gy = gy; latest_gz = gz;

    buf_ax[buf_pos] = ax; buf_ay[buf_pos] = ay; buf_az[buf_pos] = az;
    buf_gx[buf_pos] = gx; buf_gy[buf_pos] = gy; buf_gz[buf_pos] = gz;
    buf_pos = (buf_pos + 1) % SAMPLES_PER_WIN;
    if (buf_count < SAMPLES_PER_WIN) buf_count++;
    new_samples++;

    if (buf_count >= SAMPLES_PER_WIN && new_samples >= OVERLAP_SAMPLES) {
        runClassification();
        new_samples = 0;
    }
}

// =============================================================================
// CLASSIFICATION — fires every ~2.5s, takes ~2ms
// Sends C packet with behavior + staleness metadata.
// =============================================================================
void runClassification() {
    float win_ax[SAMPLES_PER_WIN], win_ay[SAMPLES_PER_WIN], win_az[SAMPLES_PER_WIN];
    float win_gx[SAMPLES_PER_WIN], win_gy[SAMPLES_PER_WIN], win_gz[SAMPLES_PER_WIN];

    for (int i = 0; i < SAMPLES_PER_WIN; i++) {
        int idx = (buf_pos + i) % SAMPLES_PER_WIN;
        win_ax[i] = buf_ax[idx]; win_ay[i] = buf_ay[idx]; win_az[i] = buf_az[idx];
        win_gx[i] = buf_gx[idx]; win_gy[i] = buf_gy[idx]; win_gz[i] = buf_gz[idx];
    }

    // cc_classify handles: extract → select → scale → MLP → voter
    int raw_pred = -1;
    int smoothed = cc_classify(win_ax, win_ay, win_az,
                               win_gx, win_gy, win_gz,
                               &voter, &raw_pred);

    // Compute confidence from voter history
    int conf = 0;
    int vcount = voter.count < CC_VOTE_WINDOW ? voter.count : CC_VOTE_WINDOW;
    for (int i = 0; i < vcount; i++) {
        if (voter.history[i] == smoothed) conf++;
    }
    current_behavior   = smoothed;
    current_confidence = conf;

    Serial.printf("[CLASS] %s (conf %d/%d, raw=%s)\n",
                  BEH_NAMES[smoothed], conf, vcount, BEH_NAMES[raw_pred]);

    // Compute staleness metadata
    uint32_t now = millis();
    uint32_t range_age_ms = (last_range_success_ms > 0) ? (now - last_range_success_ms) : 0xFFFFFFFF;
    uint8_t  nbr_count    = neighborTable.getActiveCount();
    MeshStats stats       = neighborTable.getStats();
    uint8_t  range_pct    = (stats.total_attempts > 0)
                            ? (uint8_t)((stats.total_successes * 100) / stats.total_attempts)
                            : 0;

    // Send classification with metadata
#if ESP_NOW_ENABLED
    espnowSender.sendIMUClassification(
        TAG_ID, smoothed, conf,
        latest_ax, latest_ay, latest_az,
        latest_gx, latest_gy, latest_gz,
        now, range_age_ms, nbr_count, range_pct
    );
#elif WIFI_ENABLED
    wifiSender.sendIMUClassification(
        TAG_ID, smoothed, conf,
        latest_ax, latest_ay, latest_az,
        latest_gx, latest_gy, latest_gz,
        now, range_age_ms, nbr_count, range_pct
    );
#endif
}

// =============================================================================
// SETUP
// =============================================================================
void setup() {
    Serial.begin(SERIAL_BAUD);
    delay(200);

    Serial.println("\n==========================================");
    Serial.println("  P2P UWB MESH + CATTLE CLASSIFIER v2");
    Serial.println("  DS-TWR: symmetric formula");
    Serial.println("  Model:  MLP (F1=0.93, 39 features)");
    Serial.println("==========================================");

    generateNodeID();

    Serial.print("Slot: ");
    Serial.print(scheduler.getMySlot());
    Serial.print("/");
    Serial.print(NUM_SLOTS - 1);
    Serial.print(" | Frame: ");
    Serial.print(FRAME_LENGTH_MS);
    Serial.println("ms");
    Serial.println("==========================================\n");

#if PRODUCTION_MODE
    Serial.flush();   // let those bytes leave the UART
    delay(50);
    Serial.end();     // silent after this point
#endif

    randomSeed(TAG_ID * 1000 + millis());

#if ESP_NOW_ENABLED
    espnowSender.begin();
    if (REDUCE_CPU_FREQ) {
        setCpuFrequencyMhz(CPU_FREQ_NORMAL);
    }
#elif WIFI_ENABLED
    wifiSender.begin();
#endif

    initializeIMU();
    initializeUWB();

    rangeLogger.setVerbose(true);
    rangeLogger.setJsonOutput(false);

    enterListeningMode();
    last_sample_ms = millis();
}

// =============================================================================
// BROADCAST NEIGHBOR TABLE
// =============================================================================
void broadcastNeighborTable() {
    uint8_t count = neighborTable.getActiveCount();

#if ESP_NOW_ENABLED
    espnowSender.sendHeartbeat(TAG_ID, scheduler.getFrameNumber(), count, millis());
#elif WIFI_ENABLED
    wifiSender.sendHeartbeat(TAG_ID, scheduler.getFrameNumber(), count, millis());
#endif

    const Neighbor* neighbors = neighborTable.getNeighborArray();

    for (uint8_t i = 0; i < MAX_NEIGHBORS; i++) {
        const Neighbor& n = neighbors[i];
        if (!n.active) continue;

        uint8_t range_pct = 0;
        if (n.ranging_attempts > 0) {
            range_pct = (n.ranging_successes * 100) / n.ranging_attempts;
        }

        // Compute per-neighbor staleness
        uint32_t stale_ms = (n.last_range_time > 0) ? (millis() - n.last_range_time) : 0xFFFFFFFF;
        bool is_stale = (stale_ms > LINK_STALE_THRESHOLD_MS);

#if ESP_NOW_ENABLED
        espnowSender.sendNeighborInfo(
            TAG_ID, n.id, n.hello_count, range_pct,
            n.filtered_distance_cm, n.avg_rssi,
            is_stale, stale_ms
        );
#elif WIFI_ENABLED
        wifiSender.sendNeighborInfo(
            TAG_ID, n.id, n.hello_count, range_pct,
            n.filtered_distance_cm, n.avg_rssi,
            is_stale, stale_ms
        );
#endif
        delay(5);
    }
}

// =============================================================================
// MAIN LOOP
// =============================================================================
void loop() {
    // Periodic comms health check
#if ESP_NOW_ENABLED
    static uint32_t last_check = 0;
    if (millis() - last_check > 5000) {
        last_check = millis();
        espnowSender.checkConnection();
    }
#elif WIFI_ENABLED
    static uint32_t last_check = 0;
    if (millis() - last_check > 1000) {
        last_check = millis();
        wifiSender.checkConnection();
    }
#endif

    // IMU sample — ONLY when UWB is idle (LISTENING state).
    // I2C interrupts during active DS-TWR would corrupt DW3000 timestamps.
    if (state == NodeState::LISTENING) {
        sampleIMU();
    }

    // UWB state machine
    scheduler.update();

    switch (state) {
        case NodeState::LISTENING:        handleListeningState();  break;
        case NodeState::INIT_SEND_POLL:   handleInitSendPoll();    break;
        case NodeState::INIT_WAIT_RESP:   handleInitWaitResp();    break;
        case NodeState::INIT_SEND_FINAL:  handleInitSendFinal();   break;
        case NodeState::INIT_WAIT_REPORT: handleInitWaitReport();  break;
        case NodeState::INIT_CALCULATE:   handleInitCalculate();   break;
        case NodeState::RESP_SEND_RESP:   handleRespSendResp();    break;
        case NodeState::RESP_WAIT_FINAL:  handleRespWaitFinal();   break;
        case NodeState::RESP_SEND_REPORT: handleRespSendReport();  break;
        case NodeState::HELLO_SEND:       handleHelloSend();       break;
        default: enterListeningMode(); break;
    }

    // Periodic tasks (staggered to avoid clustering)
    static uint32_t last_status = 0;
    if (millis() - last_status > 10000) {
        printStatus();
        last_status = millis();
    }

    static uint32_t last_mesh = 0;
    if (millis() - last_mesh > 30000) {
        neighborTable.printMeshStatus();
        last_mesh = millis();
    }

    static uint32_t last_neighbor_broadcast = 0;
    if (millis() - last_neighbor_broadcast > 5000) {
        broadcastNeighborTable();
        last_neighbor_broadcast = millis();
    }

    static uint32_t last_flush = 0;
    if (millis() - last_flush > 1000) {
        rangeLogger.flushToSerial();
        last_flush = millis();
    }
}

// =============================================================================
// LISTEN-BEFORE-TALK
// =============================================================================
#if LBT_ENABLED
bool performLBT() {
    uwb.clearSystemStatus();
    uwb.standardRX();
    uint32_t start = millis();
    while (millis() - start < LBT_LISTEN_MS) {
        if (uwb.receivedFrameSucc() == 1) {
            uwb.clearSystemStatus();
            return false;
        }
        delayMicroseconds(100);
    }
    return true;
}
#else
bool performLBT() { return true; }
#endif

// =============================================================================
// LISTENING STATE
// =============================================================================
void handleListeningState() {
    if (scheduler.shouldSendHello()) {
        transitionTo(NodeState::HELLO_SEND);
        return;
    }

    if (scheduler.canInitiateRanging()) {
        // Power saving: only initiate ranging every Nth frame
        static uint32_t range_frame_counter = 0;
        range_frame_counter++;
        if ((range_frame_counter % RANGE_EVERY_N_FRAMES) == 0) {
            uint8_t target = neighborTable.getNextRangingTarget();
            if (target != 0) {
                session.target_id    = target;
                session.is_initiator = true;
                retry_count          = 0;
                lbt_retries          = 0;
                Serial.print("[RANGE] -> ");
                Serial.println(target);
                transitionTo(NodeState::INIT_SEND_POLL);
                return;
            }
        }
    }

    int rx = uwb.receivedFrameSucc();

    if (rx == 1) {
        uint8_t mode      = uwb.read(RX_BUFFER_0_REG, 0x00) & 0x07;
        uint8_t sender_id = uwb.read(RX_BUFFER_0_REG, 0x01) & 0xFF;
        uint8_t dest_id   = uwb.read(RX_BUFFER_0_REG, 0x02) & 0xFF;
        uint8_t stage     = uwb.read(RX_BUFFER_0_REG, 0x03) & 0x07;

        if (mode == 1) {
            // HELLO beacon
            if (stage == 0 && dest_id == BROADCAST_ID) {
                neighborTable.processHello(sender_id, 0, neighborTable.getActiveCount());

                uint8_t sender_slot = sender_id % NUM_SLOTS;
                uint8_t my_slot     = TAG_ID % NUM_SLOTS;
                if (sender_slot == my_slot) {
                    Serial.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
                    Serial.print("!! SLOT COLLISION: Node ");
                    Serial.print(sender_id);
                    Serial.print(" has same slot ");
                    Serial.println(my_slot);
                    Serial.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
                }

#if SYNC_TO_LOWER_ID
                if (sender_id < TAG_ID) {
                    uint8_t s_slot = sender_id % NUM_SLOTS;
                    uint32_t now = millis();
                    uint32_t frame_start = now - (s_slot * SLOT_LENGTH_MS) - (SLOT_LENGTH_MS / 2);
                    scheduler.syncFrameStart(frame_start);
                }
#endif
                uwb.clearSystemStatus();
                uwb.standardRX();
                return;
            }

            // Incoming POLL — become responder
            if (stage == STAGE_POLL && (dest_id == TAG_ID || dest_id == BROADCAST_ID)) {
                session.target_id    = sender_id;
                session.is_initiator = false;
                session.rx_poll      = uwb.readRXTimestamp();
                uwb.clearSystemStatus();
                Serial.print("[POLL] <- ");
                Serial.println(sender_id);
                scheduler.enterRespondingMode();
                transitionTo(NodeState::RESP_SEND_RESP);
                return;
            }
        }

        uwb.clearSystemStatus();
        uwb.standardRX();
    } else if (rx == 2) {
        uwb.clearSystemStatus();
        uwb.standardRX();
    }
}

// =============================================================================
// INITIATOR STATES
// =============================================================================
void handleInitSendPoll() {
    if (scheduler.shouldAddJitter()) {
        delay(scheduler.getJitterDelay());
    }

#if LBT_ENABLED
    if (!performLBT()) {
        lbt_retries++;
        if (lbt_retries >= LBT_MAX_RETRIES) {
            Serial.println("[LBT] Busy, abort");
            scheduler.reportCollision();
            neighborTable.recordCollision(session.target_id);
            enterListeningMode();
            return;
        }
        delay(random(LBT_BACKOFF_BASE_MS, LBT_BACKOFF_MAX_MS));
        return;
    }
#endif

    uwb.clearSystemStatus();
    uwb.setSenderID(TAG_ID);
    uwb.setDestinationID(session.target_id);
    if (REDUCE_CPU_FREQ) setCpuFrequencyMhz(CPU_FREQ_HIGH);
    uwb.ds_sendFrame(STAGE_POLL);
    session.tx_poll = uwb.readTXTimestamp();
    transitionTo(NodeState::INIT_WAIT_RESP);
}

void handleInitWaitResp() {
    if (checkStateTimeout(RESPONSE_TIMEOUT_MS)) {
        retry_count++;
        if (retry_count < MAX_RANGING_RETRIES) {
            transitionTo(NodeState::INIT_SEND_POLL);
        } else {
            LOG_RANGE_FAILURE(TAG_ID, session.target_id, scheduler.getFrameNumber(), "RESP_TIMEOUT");
            neighborTable.recordRangingFailure(session.target_id);
            enterListeningMode();
        }
        return;
    }

    int rx = uwb.receivedFrameSucc();
    if (rx == 1) {
        uint8_t mode      = uwb.read(RX_BUFFER_0_REG, 0x00) & 0x07;
        uint8_t sender_id = uwb.read(RX_BUFFER_0_REG, 0x01) & 0xFF;
        uint8_t stage     = uwb.read(RX_BUFFER_0_REG, 0x03) & 0x07;
        unsigned long long rx_ts = uwb.readRXTimestamp();
        uwb.clearSystemStatus();
        if (mode == 1 && sender_id == session.target_id && stage == STAGE_RESP) {
            session.rx_resp   = rx_ts;
            session.t_round_a = session.rx_resp - session.tx_poll;
            transitionTo(NodeState::INIT_SEND_FINAL);
        } else {
            uwb.standardRX();
        }
    } else if (rx == 2) {
        uwb.clearSystemStatus();
        uwb.standardRX();
    }
}

void handleInitSendFinal() {
    uwb.setSenderID(TAG_ID);
    uwb.setDestinationID(session.target_id);
    uwb.ds_sendFrame(STAGE_FINAL);
    session.tx_final  = uwb.readTXTimestamp();
    session.t_reply_a = session.tx_final - session.rx_resp;
    transitionTo(NodeState::INIT_WAIT_REPORT);
}

void handleInitWaitReport() {
    if (checkStateTimeout(RESPONSE_TIMEOUT_MS)) {
        LOG_RANGE_FAILURE(TAG_ID, session.target_id, scheduler.getFrameNumber(), "REPORT_TIMEOUT");
        neighborTable.recordRangingFailure(session.target_id);
        enterListeningMode();
        return;
    }

    int rx = uwb.receivedFrameSucc();
    if (rx == 1) {
        uint8_t mode      = uwb.read(RX_BUFFER_0_REG, 0x00) & 0x07;
        uint8_t sender_id = uwb.read(RX_BUFFER_0_REG, 0x01) & 0xFF;
        uint8_t stage     = uwb.read(RX_BUFFER_0_REG, 0x03) & 0x07;
        if (mode == 1 && sender_id == session.target_id && stage == STAGE_REPORT) {
            session.report_t_round_b = uwb.read(RX_BUFFER_0_REG, 0x04);
            session.report_t_reply_b = uwb.read(RX_BUFFER_0_REG, 0x08);
            session.clock_offset     = uwb.getRawClockOffset();
            uwb.clearSystemStatus();
            transitionTo(NodeState::INIT_CALCULATE);
        } else {
            uwb.clearSystemStatus();
            uwb.standardRX();
        }
    } else if (rx == 2) {
        uwb.clearSystemStatus();
        uwb.standardRX();
    }
}

void handleInitCalculate() {
    int ranging_time = uwb.ds_processRTInfo(
        session.t_round_a, session.t_reply_a,
        session.report_t_round_b, session.report_t_reply_b,
        session.clock_offset
    );

    float distance_cm = uwb.convertToCM(ranging_time);
    float rssi        = uwb.getSignalStrength();
    float fp_rssi     = uwb.getFirstPathSignalStrength();

    LOG_RANGE_SUCCESS(TAG_ID, session.target_id, distance_cm, rssi, fp_rssi,
                      scheduler.getFrameNumber());
    neighborTable.recordRangingSuccess(session.target_id, distance_cm, rssi);

    // Track last successful range for staleness reporting
    last_range_success_ms = millis();

    Neighbor* n = neighborTable.getNeighbor(session.target_id);
    float filtered_dist = (n && n->filtered_distance_cm > 0)
                          ? n->filtered_distance_cm : distance_cm;

#if ESP_NOW_ENABLED
    espnowSender.sendRangingResult(TAG_ID, session.target_id, filtered_dist, rssi, millis());
#elif WIFI_ENABLED
    wifiSender.sendRangingResult(TAG_ID, session.target_id, filtered_dist, rssi, millis());
#endif

    Serial.print("[DIST] ");
    Serial.print(TAG_ID);
    Serial.print("->");
    Serial.print(session.target_id);
    Serial.print(": ");
    Serial.print(distance_cm, 1);
    Serial.print(" cm (filtered: ");
    Serial.print(filtered_dist, 1);
    Serial.println(" cm)");

    enterListeningMode();
}

// =============================================================================
// RESPONDER STATES
// =============================================================================
void handleRespSendResp() {
    uwb.setSenderID(TAG_ID);
    uwb.setDestinationID(session.target_id);
    uwb.ds_sendFrame(STAGE_RESP);
    session.tx_resp   = uwb.readTXTimestamp();
    session.t_reply_b = session.tx_resp - session.rx_poll;
    transitionTo(NodeState::RESP_WAIT_FINAL);
}

void handleRespWaitFinal() {
    if (checkStateTimeout(RESPONSE_TIMEOUT_MS)) {
        scheduler.exitRespondingMode();
        enterListeningMode();
        return;
    }

    int rx = uwb.receivedFrameSucc();
    if (rx == 1) {
        uint8_t mode      = uwb.read(RX_BUFFER_0_REG, 0x00) & 0x07;
        uint8_t sender_id = uwb.read(RX_BUFFER_0_REG, 0x01) & 0xFF;
        uint8_t stage     = uwb.read(RX_BUFFER_0_REG, 0x03) & 0x07;
        unsigned long long rx_ts = uwb.readRXTimestamp();
        uwb.clearSystemStatus();
        if (mode == 1 && sender_id == session.target_id && stage == STAGE_FINAL) {
            session.rx_final  = rx_ts;
            session.t_round_b = session.rx_final - session.tx_resp;
            transitionTo(NodeState::RESP_SEND_REPORT);
        } else {
            uwb.standardRX();
        }
    } else if (rx == 2) {
        uwb.clearSystemStatus();
        uwb.standardRX();
    }
}

void handleRespSendReport() {
    uwb.setSenderID(TAG_ID);
    uwb.setDestinationID(session.target_id);
    uwb.ds_sendRTInfo(session.t_round_b, session.t_reply_b);
    scheduler.exitRespondingMode();
    enterListeningMode();
}

// =============================================================================
// HELLO BEACON
// =============================================================================
void handleHelloSend() {
    delay(random(0, 50));
    uwb.clearSystemStatus();
    uwb.setSenderID(TAG_ID);
    uwb.setDestinationID(BROADCAST_ID);
    uwb.ds_sendFrame(0);
    scheduler.markHelloSent();
    if (DEBUG_OUTPUT) {
        Serial.print("[HELLO] seq=");
        Serial.println(hello_seq);
    }
    hello_seq++;
    enterListeningMode();
}

// =============================================================================
// HELPERS
// =============================================================================
void transitionTo(NodeState new_state) {
    state            = new_state;
    state_entry_time = millis();
}

bool checkStateTimeout(uint32_t timeout_ms) {
    return (millis() - state_entry_time) > timeout_ms;
}

void enterListeningMode() {
    state            = NodeState::LISTENING;
    state_entry_time = millis();
    uwb.clearSystemStatus();
    uwb.standardRX();
    if (REDUCE_CPU_FREQ) {
        setCpuFrequencyMhz(CPU_FREQ_NORMAL);
    }
}

// =============================================================================
// UWB INIT
// =============================================================================
void initializeUWB() {
    Serial.println("[UWB] Init...");
    uwb.begin();
    hardResetDW();
    delay(200);

    if (!uwb.checkSPI()) {
        Serial.println("[FATAL] SPI fail");
        while (1) delay(1000);
    }

    int timeout = 100;
    while (!uwb.checkForIDLE() && timeout-- > 0) delay(100);
    if (timeout <= 0) {
        Serial.println("[FATAL] IDLE timeout");
        while (1) delay(1000);
    }

    uwb.softReset();
    delay(200);

    if (!uwb.checkForIDLE()) {
        Serial.println("[FATAL] IDLE fail");
        while (1) delay(1000);
    }

    uwb.init();
    uwb.setupGPIO();
    uwb.setTXAntennaDelay(ANTENNA_DELAY_DEFAULT);
    uwb.setSenderID(TAG_ID);
    uwb.configureAsTX();
    uwb.clearSystemStatus();
    Serial.println("[UWB] Ready");
}

void resetRadio() {
    uwb.softReset();
    delay(100);
    uwb.init();
    uwb.setTXAntennaDelay(ANTENNA_DELAY_DEFAULT);
    uwb.setSenderID(TAG_ID);
    uwb.clearSystemStatus();
    uwb.configureAsTX();
    uwb.standardRX();
}

void hardResetDW() {
    pinMode(PIN_RST, OUTPUT);
    digitalWrite(PIN_RST, LOW);
    delay(3);
    digitalWrite(PIN_RST, HIGH);
    delay(5);
}

// =============================================================================
// STATUS
// =============================================================================
void printStatus() {
    Serial.println("\n======== MESH NODE + IMU v2 ========");
    Serial.print("ID: ");        Serial.print(TAG_ID);
    Serial.print(" | Slot: ");   Serial.print(scheduler.getMySlot());
    Serial.print("/");           Serial.print(NUM_SLOTS - 1);
    Serial.print(" | Frame: ");  Serial.println(scheduler.getFrameNumber());
    Serial.print("Neighbors: "); Serial.print(neighborTable.getActiveCount());
    Serial.print(" | Eligible: "); Serial.println(neighborTable.getEligibleCount());

    MeshStats stats = neighborTable.getStats();
    if (stats.total_attempts > 0) {
        Serial.printf("Ranging: %lu/%lu (%d%%)\n",
                      stats.total_successes, stats.total_attempts,
                      (int)((stats.total_successes * 100) / stats.total_attempts));
    }

    uint32_t range_age = (last_range_success_ms > 0) ? (millis() - last_range_success_ms) : 0;
    Serial.printf("Last range: %lu ms ago\n", range_age);

    if (imu_ok && current_behavior >= 0) {
        Serial.printf("Behavior: %s (conf %d/3)\n",
                      BEH_NAMES[current_behavior], current_confidence);
        Serial.printf("IMU: ax=%.1f ay=%.1f az=%.1f | gx=%.1f gy=%.1f gz=%.1f\n",
                      latest_ax, latest_ay, latest_az,
                      latest_gx, latest_gy, latest_gz);
    } else if (!imu_ok) {
        Serial.println("IMU: OFFLINE");
    } else {
        Serial.println("Behavior: initializing...");
    }

    neighborTable.printTable();
    Serial.println("====================================\n");
}

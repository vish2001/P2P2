// =============================================================================
// Decentralised Social Network Analysis of Dairy Cattle Using Peer-to-Peer
// UWB Ranging and IMU-Based Behaviour Classification
// =============================================================================
//
// Features:
//   - P2P UWB mesh ranging (DS-TWR, symmetric formula)
//   - BMI270 IMU cattle behavior classification (Decision Tree, 60 features)
//   - ESP-NOW / WiFi UDP telemetry to base station
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

int ANTENNA_DELAY = ANTENNA_DELAY_DEFAULT; // Calibrated antenna delay for accurate ranging (in DWM3000 time units)
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
}; // UWB ranging state machine states

NodeState state = NodeState::LISTENING;
uint32_t  state_entry_time = 0; // Timestamp when we entered the current state, for timeout handling
uint8_t   retry_count = 0; // Counts retries for ranging attempts, resets on success

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
} ranging_session; // State for the current ranging session, used across initiator and responder states

uint8_t hello_sequence_num   = 0; // Counts HELLO messages sent, for neighbor table management
uint8_t lbt_retry_count = 0; 

#define BROADCAST_ID 0xFF

// Track last successful ranging time (for staleness reporting)
static uint32_t last_range_success_ms = 0;

// =============================================================================
// GLOBAL STATE — IMU
// =============================================================================

BMI270 imu;
bool imu_available = false; // Flag to indicate if IMU was successfully initialized

static float buf_ax[SAMPLES_PER_WIN], buf_ay[SAMPLES_PER_WIN], buf_az[SAMPLES_PER_WIN]; // Circular buffer for accelerometer data
static float buf_gx[SAMPLES_PER_WIN], buf_gy[SAMPLES_PER_WIN], buf_gz[SAMPLES_PER_WIN]; // Circular buffer for gyroscope data
static int      buf_pos   = 0; // Current position in the circular buffer
static int      buf_count = 0; // Number of valid samples in the buffer (max SAMPLES_PER_WIN)
static int      new_samples = 0; // Counts new samples since last classification
static uint32_t last_sample_ms = 0; // Timestamp of last IMU sample, for sampling rate control

static CattleVoter behaviour_voter;
static int   current_behavior   = -1; // Current classified behavior (0-3 for primary classes, -1 for unclassified)
static int   current_confidence = 0; // Confidence in current behavior (number of votes in behavior voter history)
static float latest_ax = 0, latest_ay = 0, latest_az = 0;
static float latest_gx = 0, latest_gy = 0, latest_gz = 0;


// =============================================================================
// FORWARD DECLARATIONS
// =============================================================================
void initializeUWB(); // Initializes DW3000 UWB module with settings and antenna delay calibration
void initializeIMU(); // Initializes BMI270 IMU, checks availability, and runs classifier self-test

void hardResetUWB(); // Performs a hard reset of the DW3000 module by toggling the RST pin, used for recovery from error states
void generateNodeID(); 
bool performLBT();
bool checkStateTimeout(uint32_t timeout_ms);
void transitionTo(NodeState new_state);
void enterListeningMode();

void sampleIMU();
void runClassification();


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
// Generates a unique node ID from ESP32 MAC address or override, and updates TDMA slot
void generateNodeID() {
#if TAG_ID_OVERRIDE > 0
    TAG_ID = TAG_ID_OVERRIDE;
    Serial.print("[ID] Override: ");
    Serial.println(TAG_ID);
#else
    uint64_t mac = ESP.getEfuseMac();
    Serial.print("[MAC] ");
    // Print MAC in standard format
    for (int i = 0; i < 6; i++) {
        uint8_t byte = (mac >> (i * 8)) & 0xFF;
        if (byte < 0x10) Serial.print("0");
        Serial.print(byte, HEX);
        if (i < 5) Serial.print(":");
    }
    Serial.println();
    // FNV-1a hash of MAC to 1-253 range for TAG_ID
    uint32_t hash = 2166136261;
    for (int i = 0; i < 6; i++) {
        uint8_t byte = (mac >> (i * 8)) & 0xFF;
        hash ^= byte;
        hash *= 16777619;
    }
    TAG_ID = (hash % 253) + 1; // Map to 1-253
    Serial.print("[ID] Auto-generated: ");
    Serial.println(TAG_ID);
#endif
  scheduler.recalculateSlot(); // Update TDMA slot based on new TAG_ID
  Serial.print("[SLOT] ");
  Serial.print(scheduler.getMySlot()); 
  Serial.print("/");
  Serial.println(NUM_SLOTS - 1);
}

// =============================================================================
// IMU INITIALIZATION
// =============================================================================
// Initializes BMI270 IMU, checks availability, and runs classifier self-test
void initializeIMU() {
    Serial.println("[IMU] Initializing BMI270...");
    Wire.begin(IMU_SDA_PIN, IMU_SCL_PIN);
    Wire.setClock(400000);

    if (imu.beginI2C(BMI2_I2C_PRIM_ADDR) == BMI2_OK) {
        imu_available = true;
        Serial.println("[IMU] BMI270 OK");

        int pass = cc_selftest(); // Self-test runs the feature extraction and Decision Tree classification on 4 fixed test vectors, checks against expected classes
        Serial.printf("[IMU] Classifier self-test: %d/4 (Decision Tree primary)\n", pass);

        imu.getSensorData(); // Read initial IMU sensor data to display current Z-axis acceleration for verification
        Serial.printf("[IMU] az = %.2f m/s2\n", imu.data.accelZ * G_TO_MS2);
    } else {
        imu_available = false;
        Serial.println("[IMU] BMI270 not found — classification disabled, UWB continues");
    }
    cc_voter_init(&behaviour_voter); // Initialize the CattleVoter structure to smooth behavior classifications over multiple predictions for stability
}

// =============================================================================
// IMU SAMPLING — non-blocking, LISTENING state only
// =============================================================================
// Samples IMU data non-blockingly during LISTENING state, buffers for classification
void sampleIMU() {
    if (!imu_available) return;
    uint32_t now = millis();
    if (now - last_sample_ms < SAMPLE_INTERVAL) return;
    last_sample_ms = now;

    imu.getSensorData(); // Read latest sensor data from BMI270
    
    // Convert raw IMU data to physical units (m/s^2 for accel, deg/s for gyro)
    float ax = imu.data.accelX * G_TO_MS2;
    float ay = imu.data.accelY * G_TO_MS2;
    float az = imu.data.accelZ * G_TO_MS2;
    float gx = imu.data.gyroX;
    float gy = imu.data.gyroY;
    float gz = imu.data.gyroZ;
    
    // Update latest IMU readings for telemetry, and add to circular buffer for classification
    latest_ax = ax; latest_ay = ay; latest_az = az;
    latest_gx = gx; latest_gy = gy; latest_gz = gz;

    buf_ax[buf_pos] = ax; buf_ay[buf_pos] = ay; buf_az[buf_pos] = az;
    buf_gx[buf_pos] = gx; buf_gy[buf_pos] = gy; buf_gz[buf_pos] = gz;
    buf_pos = (buf_pos + 1) % SAMPLES_PER_WIN; // Move buffer position forward, wrap around if needed
    if (buf_count < SAMPLES_PER_WIN) buf_count++; // Increment count of valid samples in buffer, up to the max window size
    new_samples++;

    if (buf_count >= SAMPLES_PER_WIN && new_samples >= OVERLAP_SAMPLES) {
        runClassification(); // Run classification on the buffered IMU data once we have enough samples for a full window and have reached the overlap threshold
        new_samples = 0; // Reset new samples count after classification 
    }
}

// =============================================================================
// CLASSIFICATION — fires every ~2.5s
// Sends C packet with behavior + staleness metadata.
// =============================================================================
// Runs cattle behavior classification on buffered IMU data, sends results via telemetry
void runClassification() {
    float win_ax[SAMPLES_PER_WIN], win_ay[SAMPLES_PER_WIN], win_az[SAMPLES_PER_WIN];
    float win_gx[SAMPLES_PER_WIN], win_gy[SAMPLES_PER_WIN], win_gz[SAMPLES_PER_WIN];

    for (int i = 0; i < SAMPLES_PER_WIN; i++) {
        int idx = (buf_pos + i) % SAMPLES_PER_WIN;
        win_ax[i] = buf_ax[idx]; win_ay[i] = buf_ay[idx]; win_az[i] = buf_az[idx];
        win_gx[i] = buf_gx[idx]; win_gy[i] = buf_gy[idx]; win_gz[i] = buf_gz[idx];
    }

    // cc_classify handles: extract → select → scale → Decision Tree → behaviour_voter
    int raw_pred = -1;
    int smoothed = cc_classify(win_ax, win_ay, win_az,
                               win_gx, win_gy, win_gz,
                               &behaviour_voter, &raw_pred);

    // Compute confidence from behaviour_voter history
    int conf = 0;
    int vcount = behaviour_voter.count < CC_VOTE_WINDOW ? behaviour_voter.count : CC_VOTE_WINDOW;
    for (int i = 0; i < vcount; i++) {
        if (behaviour_voter.history[i] == smoothed) conf++;
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
// Initializes serial, generates ID, sets up IMU/UWB, enters listening mode
// =============================================================================
void setup() {
    Serial.begin(SERIAL_BAUD);
    delay(200);

    Serial.println("\n==========================================");
    Serial.println("  P2P UWB RANGING + CATTLE CLASSIFIER v2.0");
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

    randomSeed(TAG_ID * 1000 + millis()); // Seed random number generator with a combination of TAG_ID and current time for any randomized operations (e.g. LBT backoff)

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

    enterListeningMode(); // Start in listening mode, waiting for HELLO/POLL messages to initiate ranging
    last_sample_ms = millis();
}

// =============================================================================
// BROADCAST NEIGHBOR TABLE
// Broadcasts neighbor table and heartbeat via ESP-NOW or WiFi
// =============================================================================
void broadcastNeighborTable() {
    uint8_t count = neighborTable.getActiveCount(); // Count active neighbors for telemetry

#if ESP_NOW_ENABLED
    espnowSender.sendHeartbeat(TAG_ID, scheduler.getFrameNumber(), count, millis()); // Send heartbeat with current frame number and neighbor count for monitoring connectivity and network health
#elif WIFI_ENABLED
    wifiSender.sendHeartbeat(TAG_ID, scheduler.getFrameNumber(), count, millis());
#endif

    const Neighbor* neighbors = neighborTable.getNeighborArray(); // Get pointer to neighbor array for iteration
    

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
        ); // Send neighbor info including ranging success rate, filtered distance, average RSSI, and staleness indicators for monitoring link quality and neighbor status in the mesh network
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
// Main loop: handles comms checks, IMU sampling, UWB state machine, periodic tasks
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
    while (millis() - start < LBT_LISTEN_MS) { // Listen for a short period to check if the channel is busy before transmitting
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
// Handles LISTENING state: checks for HELLO/POLL messages, initiates ranging
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
                ranging_session.target_id    = target; // Set target ID for ranging session
                ranging_session.is_initiator = true; // This node will be the initiator in the ranging exchange
                retry_count          = 0;
                lbt_retry_count          = 0;
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

#if SYNC_TO_LOWER_ID  // If the sender has a lower ID, we can assume they are earlier in the TDMA schedule and sync to their frame start for better timing alignment in the mesh. 
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
                ranging_session.target_id    = sender_id;
                ranging_session.is_initiator = false;
                ranging_session.rx_poll      = uwb.readRXTimestamp();
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
// Initiator: Sends POLL message after LBT, transitions to wait for response
// =============================================================================
void handleInitSendPoll() {
    if (scheduler.shouldAddJitter()) {
        delay(scheduler.getJitterDelay());
    }

#if LBT_ENABLED
    if (!performLBT()) {
        lbt_retry_count++;
        if (lbt_retry_count >= LBT_MAX_RETRIES) {
            Serial.println("[LBT] Busy, abort");
            scheduler.reportCollision();
            neighborTable.recordCollision(ranging_session.target_id);
            enterListeningMode();
            return;
        }
        delay(random(LBT_BACKOFF_BASE_MS, LBT_BACKOFF_MAX_MS));
        return;
    }
#endif

    uwb.clearSystemStatus();
    uwb.setSenderID(TAG_ID);
    uwb.setDestinationID(ranging_session.target_id);
    if (REDUCE_CPU_FREQ) setCpuFrequencyMhz(CPU_FREQ_HIGH);
    uwb.ds_sendFrame(STAGE_POLL);
    ranging_session.tx_poll = uwb.readTXTimestamp();
    transitionTo(NodeState::INIT_WAIT_RESP);
}

// Initiator: Waits for RESP message, handles timeout/retry
void handleInitWaitResp() {
    if (checkStateTimeout(RESPONSE_TIMEOUT_MS)) {
        retry_count++;
        if (retry_count < MAX_RANGING_RETRIES) {
            transitionTo(NodeState::INIT_SEND_POLL);
        } else {
            LOG_RANGE_FAILURE(TAG_ID, ranging_session.target_id, scheduler.getFrameNumber(), "RESP_TIMEOUT");
            neighborTable.recordRangingFailure(ranging_session.target_id);
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
        if (mode == 1 && sender_id == ranging_session.target_id && stage == STAGE_RESP) {
            ranging_session.rx_resp   = rx_ts;
            ranging_session.t_round_a = ranging_session.rx_resp - ranging_session.tx_poll;
            transitionTo(NodeState::INIT_SEND_FINAL);
        } else {
            uwb.standardRX();
        }
    } else if (rx == 2) {
        uwb.clearSystemStatus();
        uwb.standardRX();
    }
// Initiator: Sends FINAL message, calculates reply time
}

// Initiator: Sends FINAL message, calculates reply time
void handleInitSendFinal() {
    uwb.setSenderID(TAG_ID);
    uwb.setDestinationID(ranging_session.target_id);
    uwb.ds_sendFrame(STAGE_FINAL);
    ranging_session.tx_final  = uwb.readTXTimestamp();
    ranging_session.t_reply_a = ranging_session.tx_final - ranging_session.rx_resp;
    transitionTo(NodeState::INIT_WAIT_REPORT);

}

// Initiator: Waits for REPORT message, handles timeout, extracts timing info for distance calculation
void handleInitWaitReport() {
    if (checkStateTimeout(RESPONSE_TIMEOUT_MS)) {
        LOG_RANGE_FAILURE(TAG_ID, ranging_session.target_id, scheduler.getFrameNumber(), "REPORT_TIMEOUT");
        neighborTable.recordRangingFailure(ranging_session.target_id);
        enterListeningMode();
        return;
    }

    int rx = uwb.receivedFrameSucc();
    if (rx == 1) {
        uint8_t mode      = uwb.read(RX_BUFFER_0_REG, 0x00) & 0x07;
        uint8_t sender_id = uwb.read(RX_BUFFER_0_REG, 0x01) & 0xFF;
        uint8_t stage     = uwb.read(RX_BUFFER_0_REG, 0x03) & 0x07;
        if (mode == 1 && sender_id == ranging_session.target_id && stage == STAGE_REPORT) {
            ranging_session.report_t_round_b = uwb.read(RX_BUFFER_0_REG, 0x04);
            ranging_session.report_t_reply_b = uwb.read(RX_BUFFER_0_REG, 0x08);
            ranging_session.clock_offset     = uwb.getRawClockOffset();
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

// Initiator: Calculates distance from timing info, logs result, updates neighbor table, sends telemetry, returns to listening mode
void handleInitCalculate() {
    int ranging_time = uwb.ds_processRTInfo(
        ranging_session.t_round_a, ranging_session.t_reply_a,
        ranging_session.report_t_round_b, ranging_session.report_t_reply_b,
        ranging_session.clock_offset
    );

    float distance_cm = uwb.convertToCM(ranging_time);
    float rssi        = uwb.getSignalStrength();
    float fp_rssi     = uwb.getFirstPathSignalStrength();

    LOG_RANGE_SUCCESS(TAG_ID, ranging_session.target_id, distance_cm, rssi, fp_rssi,
                      scheduler.getFrameNumber());
    neighborTable.recordRangingSuccess(ranging_session.target_id, distance_cm, rssi);

    // Track last successful range for staleness reporting
    last_range_success_ms = millis();

    Neighbor* n = neighborTable.getNeighbor(ranging_session.target_id);
    float filtered_dist = (n && n->filtered_distance_cm > 0)
                          ? n->filtered_distance_cm : distance_cm;

#if ESP_NOW_ENABLED
    espnowSender.sendRangingResult(TAG_ID, ranging_session.target_id, filtered_dist, rssi, millis());
#elif WIFI_ENABLED
    wifiSender.sendRangingResult(TAG_ID, ranging_session.target_id, filtered_dist, rssi, millis());
#endif

    Serial.print("[DIST] ");
    Serial.print(TAG_ID);
    Serial.print("->");
    Serial.print(ranging_session.target_id);
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

// Responder: Sends RESP message, calculates reply time
void handleRespSendResp() {
    uwb.setSenderID(TAG_ID);
    uwb.setDestinationID(ranging_session.target_id);
    uwb.ds_sendFrame(STAGE_RESP);
    ranging_session.tx_resp   = uwb.readTXTimestamp();
// Responder: Waits for FINAL message, handles timeout
    ranging_session.t_reply_b = ranging_session.tx_resp - ranging_session.rx_poll;
    transitionTo(NodeState::RESP_WAIT_FINAL);
}

// Responder: Waits for FINAL message, handles timeout, extracts timing info for distance calculation
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
        if (mode == 1 && sender_id == ranging_session.target_id && stage == STAGE_FINAL) {
            ranging_session.rx_final  = rx_ts;
            ranging_session.t_round_b = ranging_session.rx_final - ranging_session.tx_resp;
            transitionTo(NodeState::RESP_SEND_REPORT);
        } else {
            uwb.standardRX();
        }
    } else if (rx == 2) {
        uwb.clearSystemStatus();
// Responder: Sends REPORT with timing info, exits responding mode
        uwb.standardRX();
    }
}

// Responder: Sends REPORT with timing info, exits responding mode
void handleRespSendReport() {
    uwb.setSenderID(TAG_ID);
    uwb.setDestinationID(ranging_session.target_id);
    uwb.ds_sendRTInfo(ranging_session.t_round_b, ranging_session.t_reply_b);
    scheduler.exitRespondingMode();
    enterListeningMode();
}

// =============================================================================
// HELLO BEACON
// =============================================================================

// Handles HELLO beacon transmission: sends broadcast HELLO, updates neighbor table, returns to listening mode
void handleHelloSend() {
    delay(random(0, 50));
    uwb.clearSystemStatus();
    uwb.setSenderID(TAG_ID);
    uwb.setDestinationID(BROADCAST_ID);
    uwb.ds_sendFrame(0);
    scheduler.markHelloSent();
    if (DEBUG_OUTPUT) {
        Serial.print("[HELLO] seq=");
        Serial.println(hello_sequence_num);
    }
    hello_sequence_num++;
    enterListeningMode();

}

// =============================================================================
// HELPER FUNCTIONS
// =============================================================================

// Transitions to a new state, records entry time for timeout handling
void transitionTo(NodeState new_state) {
    state            = new_state;
    state_entry_time = millis();
}

// Checks if the current state has exceeded the specified timeout duration
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
// Initializes DW3000 UWB module, performs reset and configuration
    }
}

// =============================================================================
// UWB INIT
// =============================================================================
void initializeUWB() {
    Serial.println("[UWB] Init...");
    uwb.begin();
    hardResetUWB();
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


void hardResetUWB() { // Performs a hard reset of the DW3000 module by toggling the RST pin, used for recovery from error states
    pinMode(PIN_RST, OUTPUT);
    digitalWrite(PIN_RST, LOW);
    delay(3);
    digitalWrite(PIN_RST, HIGH);
    delay(5);
// Prints current node status, neighbors, ranging stats, IMU data
// Prints current node status, neighbors, ranging stats, IMU data
}

// =============================================================================
// STATUS
// =============================================================================
void printStatus() {
    Serial.println("\n======== STARTING ========");
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

    if (imu_available && current_behavior >= 0) {
        Serial.printf("Behavior: %s (conf %d/3)\n",
                      BEH_NAMES[current_behavior], current_confidence);
        Serial.printf("IMU: ax=%.1f ay=%.1f az=%.1f | gx=%.1f gy=%.1f gz=%.1f\n",
                      latest_ax, latest_ay, latest_az,
                      latest_gx, latest_gy, latest_gz);
    } else if (!imu_available) {
        Serial.println("IMU: OFFLINE");
    } else {
        Serial.println("Behavior: initializing...");
    }

    neighborTable.printTable();
    Serial.println("====================================\n");
}

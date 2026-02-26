#pragma once
#include <Arduino.h>

// =============================================================================
// PEER-TO-PEER UWB MESH SYSTEM - SCALABLE CONFIGURATION
// =============================================================================
// ALL NODES ARE IDENTICAL PEERS. No anchors, no tags, no coordinator.
// Positions are computed as RELATIVE coordinates within the mesh.
// =============================================================================

// --- Hardware Pin Mapping (ESP32 + DW3000) ---
#define PIN_RST   27
#define PIN_CS     4
#define PIN_IRQ   34
#define PIN_SCK   18
#define PIN_MISO  19
#define PIN_MOSI  23
#define LED_PIN    2

// =============================================================================
// NODE IDENTITY - AUTO-ADDRESSING
// =============================================================================
// TAG_ID is generated automatically from MAC address at startup.
// Can be overridden via build flags for testing: -DTAG_ID_OVERRIDE=X
#ifndef TAG_ID_OVERRIDE
#define TAG_ID_OVERRIDE 0   // 0 = auto-assign from MAC
#endif

// Global variable declaration (defined in main.cpp)
extern uint8_t TAG_ID;

// =============================================================================
// BATTERY MONITORING  (3.7V single-cell LiPo)
// =============================================================================
// Connect battery through a voltage divider to an ADC-capable GPIO:
//   VBAT ---[R1=100K]---+---[R2=100K]--- GND
//                        |
//                      ADC_PIN
//
// With 100K/100K divider: ratio = 2.0, so 4.2V bat → 2.1V ADC, 3.0V → 1.5V
// If using different resistors, adjust VBAT_DIVIDER_RATIO accordingly.
// Set BATTERY_MONITOR_ENABLED to 0 if no divider is wired up.

#define BATTERY_MONITOR_ENABLED  1
#define VBAT_ADC_PIN            35       // GPIO35 (ADC1_CH7) — input-only, safe choice
#define VBAT_DIVIDER_RATIO      2.0f     // R1=R2=100K → ratio = (R1+R2)/R2 = 2.0
#define VBAT_CHECK_INTERVAL_MS  30000    // Check every 30 seconds
#define VBAT_WARN_VOLTAGE       3.30f    // Start warning (send alert packet)
#define VBAT_CRITICAL_VOLTAGE   3.00f    // Safe shutdown — protect the cell
#define VBAT_SHUTDOWN_COUNT     3        // Require N consecutive critical reads before shutdown
#define VBAT_ADC_SAMPLES        16       // Oversample for noise reduction

// =============================================================================
// COMMUNICATION SELECTION
// =============================================================================
#define ESP_NOW_ENABLED      1
#define WIFI_ENABLED         0

// =============================================================================
// ESP-NOW CONFIGURATION
// =============================================================================
#define BASE_STATION_MAC     {0xA8, 0x03, 0x2A, 0xF7, 0x0C, 0x88}
#define ESP_NOW_MIN_INTERVAL 100

// =============================================================================
// WIFI UDP CONFIGURATION (used when WIFI_ENABLED=1)
// =============================================================================
#ifndef WIFI_SSID
#define WIFI_SSID            "YourNetwork"
#endif
#ifndef WIFI_PASSWORD
#define WIFI_PASSWORD        "YourPassword"
#endif
#ifndef UDP_SERVER_IP
#define UDP_SERVER_IP        "192.168.1.100"
#endif
#ifndef UDP_SERVER_PORT
#define UDP_SERVER_PORT      5000
#endif
#ifndef UDP_SEND_INTERVAL_MS
#define UDP_SEND_INTERVAL_MS 100
#endif

// =============================================================================
// ENERGY EFFICIENCY
// =============================================================================
#define REDUCE_CPU_FREQ      1
#define CPU_FREQ_HIGH        240    // Only during UWB TX/RX critical sections
#define CPU_FREQ_NORMAL      80     // Main loop (was 160 — 80 is fine for state machine)
#define CPU_FREQ_LOW         80

// WiFi modem sleep: nodes only SEND via ESP-NOW, never receive.
// Radio can sleep between burst sends.
#define ESPNOW_MODEM_SLEEP   1      // Enable WiFi modem sleep
#define ESPNOW_BURST_INTERVAL_MS  2500  // Queue packets, burst-send every 2.5s

// Production mode: suppress serial debug output for power saving
#ifndef PRODUCTION_MODE
#define PRODUCTION_MODE      0      // 1 = suppress verbose serial, saves ~2mA
#endif

// =============================================================================
// SCALABLE TDMA TIMING
// =============================================================================
// Prime number of slots minimizes hash collisions.
// With 53 slots, IDs must differ by exactly 53 to collide.
#define NUM_SLOTS            23       // Prime, suits 10 nodes with headroom

// --- Power saving: range every Nth frame ---
// Node still LISTENS and RESPONDS every frame (stays in mesh).
// Only INITIATES ranging every Nth frame to reduce TX power.
//   1 = range every frame  (max accuracy, ~575ms updates)
//   2 = range every 2nd    (~1.15s updates, ~10% battery saving)
//   3 = range every 3rd    (~1.73s updates, ~15% battery saving)
#define RANGE_EVERY_N_FRAMES 2

// Minimum slot for DS-TWR: Poll + Resp + Final + Report + processing
#define MIN_SLOT_MS          25

// Frame timing derived from physics
#define SLOT_LENGTH_MS       MIN_SLOT_MS
#define FRAME_LENGTH_MS      (NUM_SLOTS * SLOT_LENGTH_MS)  // 575ms with 23 slots

// Slot ownership computed at runtime (TAG_ID is variable)
#define COMPUTE_MY_SLOT()    (TAG_ID % NUM_SLOTS)

// =============================================================================
// COLLISION MITIGATION
// =============================================================================
#define SLOT_JITTER_MAX_MS   8
#define COLLISION_BACKOFF_MS 50

// =============================================================================
// HELLO BEACON PARAMETERS
// =============================================================================
#define HELLO_INTERVAL_MS    3000
#define HELLO_FRAME_TYPE     0xAA
#define NEIGHBOR_TIMEOUT_MS  15000
#define HELLO_JITTER_MS      500

// =============================================================================
// FRAME SYNCHRONIZATION
// =============================================================================
#define SYNC_TO_LOWER_ID     1
#define SYNC_HOLDOFF_MS      5000

// =============================================================================
// NEIGHBOR MANAGEMENT
// =============================================================================
#define MAX_NEIGHBORS        16       // Enough for 10 nodes + headroom
#define K_NEIGHBORS          6
#define MIN_HELLO_COUNT      2

// Bridge detection (for prioritizing long-distance mesh links)
#define CLUSTER_DISTANCE_THRESHOLD_CM  1500
#define BRIDGE_LINK_PRIORITY_BOOST     50.0

// =============================================================================
// CONNECTIVITY PRESERVATION
// =============================================================================
// Range with ALL neighbors to maintain mesh connectivity
#define RANGE_ALL_NEIGHBORS  1
#define STALE_LINK_PRIORITY  100.0
#define LINK_STALE_THRESHOLD_MS  10000

// =============================================================================
// DS-TWR PROTOCOL
// =============================================================================
#define DS_TWR_FRAME_TYPE    0x01
#define RESPONSE_TIMEOUT_MS  35
#define MAX_RANGING_RETRIES  2

#define STAGE_POLL           1
#define STAGE_RESP           2
#define STAGE_FINAL          3
#define STAGE_REPORT         4
#define STAGE_ERROR          7

// =============================================================================
// LISTEN-BEFORE-TALK
// =============================================================================
#define LBT_ENABLED          1
#define LBT_LISTEN_MS        5
#define LBT_BACKOFF_BASE_MS  10
#define LBT_BACKOFF_MAX_MS   50
#define LBT_MAX_RETRIES      3

// =============================================================================
// DISTANCE FILTERING
// =============================================================================
#define FILTER_SIZE          7
#define MIN_DISTANCE_CM      10.0
#define MAX_DISTANCE_CM      10000.0
#define MAX_ACCEPT_DISTANCE_CM  3000.0   // Ignore measurements beyond 30m (less reliable)
#define OUTLIER_THRESHOLD    200.0
#define EMA_ALPHA            0.3

// =============================================================================
// LOGGING & DEBUG
// =============================================================================
#define LOG_BUFFER_SIZE      32
#define SERIAL_BAUD          921600

#ifndef DEBUG_OUTPUT
#define DEBUG_OUTPUT         0
#endif

// =============================================================================
// ANTENNA CALIBRATION
// =============================================================================
#define ANTENNA_DELAY_DEFAULT 16350

// =============================================================================
// DW3000 RADIO CONSTANTS
// =============================================================================
#define LEN_RX_CAL_CONF 4
#define LEN_TX_FCTRL_CONF 6
#define LEN_AON_DIG_CFG_CONF 3

#define PMSC_STATE_IDLE 0x3
#define FCS_LEN 2

#define STDRD_SYS_CONFIG 0x188
#define DTUNE0_CONFIG 0x0F

#define SYS_STATUS_FRAME_RX_SUCC 0x2000
#define SYS_STATUS_RX_ERR 0x4279000
#define SYS_STATUS_FRAME_TX_SUCC 0x80

#define PREAMBLE_32 4
#define PREAMBLE_64 8
#define PREAMBLE_128 5
#define PREAMBLE_256 9
#define PREAMBLE_512 11
#define PREAMBLE_1024 2
#define PREAMBLE_2048 10
#define PREAMBLE_4096 3
#define PREAMBLE_1536 6

#define CHANNEL_5 0x0
#define CHANNEL_9 0x1

#define PAC4 0x03
#define PAC8 0x00
#define PAC16 0x01
#define PAC32 0x02

#define DATARATE_6_8MB 0x1
#define DATARATE_850KB 0x0

#define PHR_MODE_STANDARD 0x0
#define PHR_MODE_LONG 0x1

#define PHR_RATE_6_8MB 0x1
#define PHR_RATE_850KB 0x0

#define SPIRDY_MASK 0x80
#define RCINIT_MASK 0x100
#define BIAS_CTRL_BIAS_MASK 0x1F

#define GEN_CFG_AES_LOW_REG 0x00
#define GEN_CFG_AES_HIGH_REG 0x01
#define STS_CFG_REG 0x2
#define RX_TUNE_REG 0x3
#define EXT_SYNC_REG 0x4
#define GPIO_CTRL_REG 0x5
#define DRX_REG 0x6
#define RF_CONF_REG 0x7
#define RF_CAL_REG 0x8
#define FS_CTRL_REG 0x9
#define AON_REG 0xA
#define OTP_IF_REG 0xB
#define CIA_REG1 0xC
#define CIA_REG2 0xD
#define CIA_REG3 0xE
#define DIG_DIAG_REG 0xF
#define PMSC_REG 0x11
#define RX_BUFFER_0_REG 0x12
#define RX_BUFFER_1_REG 0x13
#define TX_BUFFER_REG 0x14
#define ACC_MEM_REG 0x15
#define SCRATCH_RAM_REG 0x16
#define AES_RAM_REG 0x17
#define SET_1_2_REG 0x18
#define INDIRECT_PTR_A_REG 0x1D
#define INDIRECT_PTR_B_REG 0x1E
#define IN_PTR_CFG_REG 0x1F

#define TRANSMIT_DELAY 0x3B9ACA00
#define TRANSMIT_DIFF 0x1FF

#define NS_UNIT 4.0064102564102564
#define PS_UNIT 15.6500400641025641
#define SPEED_OF_LIGHT 0.029979245800

#define CLOCK_OFFSET_CHAN_5_CONSTANT -0.5731e-3f
#define CLOCK_OFFSET_CHAN_9_CONSTANT -0.1252e-3f

#define NO_OFFSET 0x0

// =============================================================================
// GLOBAL STATE VARIABLES
// =============================================================================
extern int ANTENNA_DELAY;
extern int led_status;
extern int destination;
extern int sender;

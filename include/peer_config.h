#pragma once
#include <Arduino.h>

// =============================================================================
// PEER-TO-PEER UWB RANGING CONFIGURATION
// =============================================================================
// ALL NODES ARE IDENTICAL PEERS. Each node acts as both initiator and responder, ranging with neighbors in a TDMA schedule.
// =============================================================================

// --- Hardware Pin Mapping (ESP32 + DW3000) --- According to the custom PCB design.
#define PIN_RST   27
#define PIN_CS     4
#define PIN_IRQ   34
#define PIN_SCK   18
#define PIN_MISO  19
#define PIN_MOSI  23
#define LED_PIN    2

// TAG_ID is generated automatically from MAC address at startup.
// Can be overridden via build flags for testing: -DTAG_ID_OVERRIDE=X
#ifndef TAG_ID_OVERRIDE
#define TAG_ID_OVERRIDE 0   // 0 = auto-assign from MAC
#endif

// Global variable declaration (defined in main.cpp)
extern uint8_t TAG_ID;


// =============================================================================
// COMMUNICATION SELECTION
// =============================================================================
#define ESP_NOW_ENABLED      1
#define WIFI_ENABLED         0

// =============================================================================
// ESP-NOW CONFIGURATION
// =============================================================================
#define BASE_STATION_MAC     {0xA8, 0x03, 0x2A, 0xF7, 0x0C, 0x88}   // MAC address of the base station (for ESP-NOW communication)
#define ESP_NOW_MIN_INTERVAL 100 // Minimum interval between ESP-NOW sends (ms) to prevent congestion

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
// Node still LISTENS and RESPONDS every frame .
// Only INITIATES ranging every Nth frame to reduce TX power.
//   1 = range every frame  (max accuracy, ~575ms updates)
//   2 = range every 2nd    (~1.15s updates, ~10% battery saving)
//   3 = range every 3rd    (~1.73s updates, ~15% battery saving)
#define RANGE_EVERY_N_FRAMES 2

// Minimum slot for DS-TWR: Poll + Resp + Final + Report + processing
#define MIN_SLOT_MS          25

// Frame timing derived from physics
#define SLOT_LENGTH_MS       MIN_SLOT_MS // ms per slot (must accommodate the entire DS-TWR exchange)
#define FRAME_LENGTH_MS      (NUM_SLOTS * SLOT_LENGTH_MS)  // 575ms with 23 slots

// Slot ownership computed at runtime (TAG_ID is variable)
#define COMPUTE_MY_SLOT()    (TAG_ID % NUM_SLOTS)

// =============================================================================
// COLLISION MITIGATION
// =============================================================================

// Random jitter added before each TX as a secondary collision mitigation
// mechanism alongside LBT. Even with TDMA, two nodes
// with the same hash slot would collide every frame without this jitter.
// 8ms fits within the 25ms slot without affecting the DS-TWR exchange.
#define SLOT_JITTER_MAX_MS   8 

// Backoff duration if LBT detects a busy channel. Combined with exponential retry (up to 3 attempts at 10-50ms)
#define COLLISION_BACKOFF_MS 50

// =============================================================================
// HELLO BEACON PARAMETERS
// =============================================================================
#define HELLO_INTERVAL_MS    3000 // Minimum interval between HELLO beacons from the same node (ms)
#define NEIGHBOR_TIMEOUT_MS  15000  // If no HELLO is received for 15 seconds (= 5 missed beacons) the neighbour is removed.
#define HELLO_JITTER_MS      500 // spread beacons to reduce simultaneous TX

// =============================================================================
// FRAME SYNCHRONIZATION
// =============================================================================
// When two nodes hear each other's HELLOs, the one with the higher TAG_ID
// adjusts its frame start to align with the lower ID. This loose sync reduces
// inter-slot interference without needing a dedicated sync master.
// The holdoff prevents rapid re-syncing during startup.
#define SYNC_TO_LOWER_ID     1  
#define SYNC_HOLDOFF_MS      5000

// =============================================================================
// NEIGHBOR MANAGEMENT
// =============================================================================
#define MAX_NEIGHBORS        16       // 16 slots is enough for 10 nodes with headroom for future expansion.
#define MIN_HELLO_COUNT      2 // Only consider nodes we've heard at least 2 HELLOs from (to filter out transient noise),is added to the neighbour table. This prevents false entries from stray packets during startup .
#define CLUSTER_DISTANCE_THRESHOLD_CM  1500 // Neighbours measured beyond this distance are treated as potential bridge links, changed according to the space requirements. 
#define BRIDGE_LINK_PRIORITY_BOOST     50.0 //adds 50 points to the priority score of a long-range neighbour, making it more likely to be selected as a ranging target to preserve connectivity across larger spaces.

// =============================================================================
// CONNECTIVITY PRESERVATION
// =============================================================================
// Range with ALL neighbors to maintain mesh connectivity
#define RANGE_ALL_NEIGHBORS  1
// Stale long-range links are ranged first because they are likely bridge
// edges — removing them could split the mesh into disconnected sub-groups.
#define STALE_LINK_PRIORITY  100.0
// LINK_STALE_THRESHOLD_MS=10s means a link is considered stale if no
// successful range has happened in the last 10 seconds.
#define LINK_STALE_THRESHOLD_MS  10000

// =============================================================================
// DS-TWR PROTOCOL
// =============================================================================
// The four-message DS-TWR exchange uses these stage numbers in the frame
// payload so each node knows which part of the handshake it is in.
// STAGE_ERROR=7 is reserved for the error frame that aborts a failed exchange.
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
// Before transmitting a POLL the node listens for 5ms. If the channel is
// already active it backs off and retries up to 3 times with exponential
// backoff (10–50ms). This is a second layer of collision protection on top
// of TDMA, needed because the barn metal environment causes multipath that
// can make a slot appear busy even when it isn't.
#define LBT_ENABLED          1
#define LBT_LISTEN_MS        5
#define LBT_BACKOFF_BASE_MS  10
#define LBT_BACKOFF_MAX_MS   50
#define LBT_MAX_RETRIES      3

// =============================================================================
// DISTANCE FILTERING
// =============================================================================
#define FILTER_SIZE          7 //rolling median over 7 samples smooths impulse noise
#define MIN_DISTANCE_CM      10.0 // Filter out measurements below 10cm (likely reflections or errors)
#define MAX_DISTANCE_CM      10000.0 // 100m max range for DW3000, filter outliers beyond this range
#define MAX_ACCEPT_DISTANCE_CM  3000.0   // Ignore measurements beyond 30m (less reliable)
#define OUTLIER_THRESHOLD    200.0 // Reject measurements that deviate from the median by more than 2m
#define EMA_ALPHA            0.3 // exponential moving average with alpha=0.3 forgentle tracking of slow node movement

// =============================================================================
// LOGGING & DEBUG
// =============================================================================
#define RANGE_LOG_BUFFER_SIZE      32 // Size of the circular ring buffer in range_logger.
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
// NS_UNIT and PS_UNIT convert raw DW3000 timestamp ticks to nanoseconds and
// picoseconds respectively. The DW3000 runs a 64GHz clock internally — each
// tick is ~15.65ps. SPEED_OF_LIGHT is in m/ps, used in the ToF→distance
// conversion: distance_m = tof_ticks × PS_UNIT × SPEED_OF_LIGHT

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

// Radio config options — passed to DW3000Class::config[] during init
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

// DW3000 status/config bitmasks
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
#define SPEED_OF_LIGHT 0.029979245800 // m/ps

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

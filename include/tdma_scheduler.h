#pragma once
#include <Arduino.h>
#include "peer_config.h"

// =============================================================================
// SCALABLE TDMA SCHEDULER
// =============================================================================
// Enhanced for dynamic ID assignment and collision mitigation
// =============================================================================

struct SlotInfo {
    uint32_t frame_start_ms;
    uint16_t frame_number;
    uint8_t current_slot;
    uint32_t slot_start_ms;
    uint32_t slot_elapsed_ms;
    uint32_t slot_remaining_ms;
    bool is_my_slot;
};

class TDMAScheduler {
public:
    TDMAScheduler();
    
    // --- Core Update ---
    void update();
    void recalculateSlot();  // Call after TAG_ID changes
    
    // --- Slot Information ---
    uint8_t calculateCurrentSlot();
    SlotInfo getSlotInfo();
    uint8_t getMySlot();
    
    // --- Initiation Control ---
    bool canInitiateRanging();
    bool shouldAddJitter();
    uint8_t getJitterDelay();
    
    // --- HELLO Timing ---
    bool shouldSendHello();
    void markHelloSent();
    
    // --- Responding State ---
    void enterRespondingMode();
    void exitRespondingMode();
    bool isResponding();
    
    // --- Synchronization ---
    void syncFrameStart(uint32_t new_frame_start);
    bool isSynced();
    uint32_t getLastSyncTime();
    
    // --- Timing Helpers ---
    uint32_t timeUntilMySlot();
    uint16_t getFrameNumber();
    uint32_t getFrameStart();
    
    // --- Collision Handling ---
    void reportCollision();
    uint8_t getCollisionBackoff();
    bool shouldSkipSlot();
    
    // --- Debug ---
    void printStatus();

private:
    uint32_t frame_start_ms;
    uint16_t frame_number;
    uint32_t last_hello_ms;
    uint32_t last_sync_ms;
    bool responding;
    bool hello_pending;
    bool synced;
    uint8_t my_slot;  // Cached slot value
    
    // Collision mitigation
    uint8_t collision_count;
    uint32_t last_collision_ms;
    bool skip_next_slot;
    
    void checkFrameRollover();
};

extern TDMAScheduler scheduler;

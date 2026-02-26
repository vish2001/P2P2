#include "tdma_scheduler.h"

// Global instance
TDMAScheduler scheduler;

// =============================================================================
// CONSTRUCTOR
// =============================================================================
TDMAScheduler::TDMAScheduler() {
    frame_start_ms = millis();
    frame_number = 0;
    last_hello_ms = 0;
    last_sync_ms = 0;
    responding = false;
    hello_pending = true;  // Send HELLO at startup
    synced = false;
    my_slot = 0;  // Will be set properly in recalculateSlot()
    
    // Collision mitigation
    collision_count = 0;
    last_collision_ms = 0;
    skip_next_slot = false;
}

// =============================================================================
// SLOT RECALCULATION
// =============================================================================
void TDMAScheduler::recalculateSlot() {
    // Called after TAG_ID is assigned
    my_slot = TAG_ID % NUM_SLOTS;
    
    Serial.print("[TDMA] Slot assigned: ");
    Serial.print(my_slot);
    Serial.print(" (TAG_ID=");
    Serial.print(TAG_ID);
    Serial.print(", NUM_SLOTS=");
    Serial.print(NUM_SLOTS);
    Serial.println(")");
}

// =============================================================================
// CORE UPDATE
// =============================================================================
void TDMAScheduler::update() {
    checkFrameRollover();
    
    // Check if HELLO is due (with jitter to prevent storms)
    uint32_t now = millis();
    uint32_t hello_interval = HELLO_INTERVAL_MS + random(0, HELLO_JITTER_MS);
    if (now - last_hello_ms >= hello_interval) {
        hello_pending = true;
    }
    
    // Reset skip flag at start of new frame
    if (frame_number > 0 && skip_next_slot) {
        skip_next_slot = false;
    }
}

// =============================================================================
// SLOT CALCULATION
// =============================================================================
uint8_t TDMAScheduler::calculateCurrentSlot() {
    uint32_t now = millis();
    uint32_t frame_elapsed = now - frame_start_ms;
    
    if (frame_elapsed >= FRAME_LENGTH_MS) {
        frame_elapsed = frame_elapsed % FRAME_LENGTH_MS;
    }
    
    return (uint8_t)(frame_elapsed / SLOT_LENGTH_MS);
}

uint8_t TDMAScheduler::getMySlot() {
    return my_slot;
}

void TDMAScheduler::checkFrameRollover() {
    uint32_t now = millis();
    uint32_t frame_elapsed = now - frame_start_ms;
    
    if (frame_elapsed >= FRAME_LENGTH_MS) {
        frame_start_ms = now - (frame_elapsed % FRAME_LENGTH_MS);
        frame_number++;
        
        // Reset collision skip at frame boundary
        skip_next_slot = false;
        
        if (DEBUG_OUTPUT && (frame_number % 10 == 0)) {
            Serial.print("[TDMA] Frame ");
            Serial.print(frame_number);
            Serial.print(" (");
            Serial.print(FRAME_LENGTH_MS);
            Serial.println("ms)");
        }
    }
}

// =============================================================================
// SLOT INFO
// =============================================================================
SlotInfo TDMAScheduler::getSlotInfo() {
    SlotInfo info;
    uint32_t now = millis();
    uint32_t frame_elapsed = now - frame_start_ms;
    
    info.frame_start_ms = frame_start_ms;
    info.frame_number = frame_number;
    info.current_slot = calculateCurrentSlot();
    info.slot_start_ms = frame_start_ms + (info.current_slot * SLOT_LENGTH_MS);
    info.slot_elapsed_ms = now - info.slot_start_ms;
    info.slot_remaining_ms = SLOT_LENGTH_MS - info.slot_elapsed_ms;
    info.is_my_slot = (info.current_slot == my_slot);
    
    return info;
}

// =============================================================================
// INITIATION CHECK
// =============================================================================
bool TDMAScheduler::canInitiateRanging() {
    if (responding) {
        return false;
    }
    
    // Check if we should skip due to collision backoff
    if (skip_next_slot) {
        return false;
    }
    
    SlotInfo info = getSlotInfo();
    
    if (!info.is_my_slot) {
        return false;
    }
    
    // Need at least MIN_SLOT_MS for DS-TWR exchange
    const uint32_t MIN_TIME_FOR_RANGING = MIN_SLOT_MS - 5;
    
    if (info.slot_remaining_ms < MIN_TIME_FOR_RANGING) {
        return false;
    }
    
    return true;
}

bool TDMAScheduler::shouldAddJitter() {
    // Always add jitter - it helps with collision mitigation
    return true;
}

uint8_t TDMAScheduler::getJitterDelay() {
    // Base jitter
    uint8_t jitter = random(0, SLOT_JITTER_MAX_MS);
    
    // Add extra backoff if we've had recent collisions
    if (collision_count > 0) {
        uint32_t extra = collision_count * LBT_BACKOFF_BASE_MS;
        extra = min(extra, (uint32_t)LBT_BACKOFF_MAX_MS);
        jitter += random(0, extra);
    }
    
    return jitter;
}

// =============================================================================
// HELLO TIMING
// =============================================================================
bool TDMAScheduler::shouldSendHello() {
    return hello_pending && !responding;
}

void TDMAScheduler::markHelloSent() {
    last_hello_ms = millis();
    hello_pending = false;
}

// =============================================================================
// RESPONDING STATE
// =============================================================================
void TDMAScheduler::enterRespondingMode() {
    responding = true;
}

void TDMAScheduler::exitRespondingMode() {
    responding = false;
}

bool TDMAScheduler::isResponding() {
    return responding;
}

// =============================================================================
// SYNCHRONIZATION
// =============================================================================
void TDMAScheduler::syncFrameStart(uint32_t new_frame_start) {
    uint32_t now = millis();
    
    // Don't re-sync too frequently
    if (synced && (now - last_sync_ms < SYNC_HOLDOFF_MS)) {
        return;
    }
    
    uint32_t old_frame_start = frame_start_ms;
    frame_start_ms = new_frame_start;
    last_sync_ms = now;
    synced = true;
    
    Serial.print("[SYNC] Frame adjusted by ");
    Serial.print((int32_t)(new_frame_start - old_frame_start));
    Serial.println(" ms");
}

bool TDMAScheduler::isSynced() {
    return synced;
}

uint32_t TDMAScheduler::getLastSyncTime() {
    return last_sync_ms;
}

// =============================================================================
// COLLISION HANDLING
// =============================================================================
void TDMAScheduler::reportCollision() {
    collision_count++;
    last_collision_ms = millis();
    
    // After multiple collisions, skip next slot opportunity
    if (collision_count >= 3) {
        skip_next_slot = true;
        Serial.println("[TDMA] Too many collisions, skipping next slot");
    }
    
    // Decay collision count over time
    if (collision_count > 0 && millis() - last_collision_ms > 10000) {
        collision_count = collision_count / 2;
    }
}

uint8_t TDMAScheduler::getCollisionBackoff() {
    if (collision_count == 0) return 0;
    
    uint8_t backoff = COLLISION_BACKOFF_MS * collision_count;
    return min(backoff, (uint8_t)100);
}

bool TDMAScheduler::shouldSkipSlot() {
    return skip_next_slot;
}

// =============================================================================
// TIMING HELPERS
// =============================================================================
uint32_t TDMAScheduler::timeUntilMySlot() {
    SlotInfo info = getSlotInfo();
    
    if (info.is_my_slot) {
        return 0;
    }
    
    uint8_t slots_until_mine;
    if (my_slot > info.current_slot) {
        slots_until_mine = my_slot - info.current_slot;
    } else {
        slots_until_mine = NUM_SLOTS - info.current_slot + my_slot;
    }
    
    return info.slot_remaining_ms + ((slots_until_mine - 1) * SLOT_LENGTH_MS);
}

uint16_t TDMAScheduler::getFrameNumber() {
    return frame_number;
}

uint32_t TDMAScheduler::getFrameStart() {
    return frame_start_ms;
}

// =============================================================================
// DEBUG
// =============================================================================
void TDMAScheduler::printStatus() {
    SlotInfo info = getSlotInfo();
    
    Serial.println("\n=== TDMA STATUS ===");
    Serial.print("Frame: ");
    Serial.print(info.frame_number);
    Serial.print(" | Length: ");
    Serial.print(FRAME_LENGTH_MS);
    Serial.print("ms | Synced: ");
    Serial.println(synced ? "YES" : "NO");
    
    Serial.print("Current Slot: ");
    Serial.print(info.current_slot);
    Serial.print("/");
    Serial.print(NUM_SLOTS - 1);
    Serial.print(" | My Slot: ");
    Serial.print(my_slot);
    Serial.print(" | Is Mine: ");
    Serial.println(info.is_my_slot ? "YES" : "NO");
    
    Serial.print("Slot Remaining: ");
    Serial.print(info.slot_remaining_ms);
    Serial.print("ms | Slot Length: ");
    Serial.print(SLOT_LENGTH_MS);
    Serial.println("ms");
    
    Serial.print("Collisions: ");
    Serial.print(collision_count);
    Serial.print(" | Skip Next: ");
    Serial.println(skip_next_slot ? "YES" : "NO");
    
    Serial.println("===================\n");
}

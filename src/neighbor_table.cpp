#include "neighbor_table.h"

NeighborTable neighborTable;

// =============================================================================
// CONSTRUCTOR
// =============================================================================
NeighborTable::NeighborTable() {
    for (int i = 0; i < MAX_NEIGHBORS; i++) {
        neighbors[i].active = false;
        neighbors[i].id = 0;
        neighbors[i].last_hello_time = 0;
        neighbors[i].last_range_time = 0;
        neighbors[i].hello_count = 0;
        neighbors[i].ranging_attempts = 0;
        neighbors[i].ranging_successes = 0;
        neighbors[i].consecutive_failures = 0;
        neighbors[i].last_distance_cm = 0;
        neighbors[i].filtered_distance_cm = 0;
        neighbors[i].ema_distance_cm = 0;
        neighbors[i].history_index = 0;
        neighbors[i].history_count = 0;
        neighbors[i].last_rssi = -100;
        neighbors[i].avg_rssi = -100;
        neighbors[i].is_long_range = false;
        neighbors[i].slot = 0;
        
        for (int j = 0; j < FILTER_SIZE; j++) {
            neighbors[i].distance_history[j] = 0;
        }
    }
    
    current_index = 0;
    stats.total_attempts = 0;
    stats.total_successes = 0;
    stats.collision_detections = 0;
    stats.timeout_failures = 0;
}

// =============================================================================
// HELLO PROCESSING
// =============================================================================

// Processes a received HELLO message: updates neighbor table with new/existing neighbor, tracks HELLO count and recency for eligibility and target selection
void NeighborTable::processHello(uint8_t neighbor_id, uint8_t seq_num, uint8_t neighbor_count) {
    if (neighbor_id == TAG_ID) return;
    
    Neighbor* n = findOrCreate(neighbor_id); // Find existing neighbor or create new entry for this neighbor ID
    if (n == nullptr) {
        Serial.println("Neighbor table full");
        return;
    }
    
    n->last_hello_time = millis(); // Update last HELLO time for staleness tracking
    n->hello_count++; // Increment HELLO count for eligibility and neighbor status monitoring
    
    if (DEBUG_OUTPUT) {
        Serial.print("[HELLO] Node ");
        Serial.print(neighbor_id);
        Serial.print(" (neighbors: ");
        Serial.print(neighbor_count);
        Serial.println(")");
    }
}


// =============================================================================
// RANGING RESULTS
// =============================================================================

// Records a successful ranging result: updates neighbor entry with new distance and RSSI, applies filtering, updates long-range status, and tracks stats for performance monitoring
void NeighborTable::recordRangingSuccess(uint8_t neighbor_id, float distance_cm, float rssi) {
    Neighbor* n = getNeighbor(neighbor_id);
    if (n == nullptr) return;
    
    n->ranging_attempts++;
    stats.total_attempts++;
    
    // Reject measurements beyond max accept distance (less reliable)
    if (distance_cm > MAX_ACCEPT_DISTANCE_CM) {
        if (DEBUG_OUTPUT) {
            Serial.print("[FILTER] Rejected (too far): ");
            Serial.print(distance_cm);
            Serial.println(" cm");
        }
        return;
    }
    
    n->ranging_successes++;
    n->consecutive_failures = 0;
    n->last_range_time = millis();
    n->last_rssi = rssi;
    stats.total_successes++;
    
    // Update average RSSI
    if (n->avg_rssi < -99) {
        n->avg_rssi = rssi;
    } else {
        n->avg_rssi = EMA_ALPHA * rssi + (1 - EMA_ALPHA) * n->avg_rssi;
    }
    
    // Outlier rejection (sudden jumps)
    if (n->filtered_distance_cm > 0 && 
        abs(distance_cm - n->filtered_distance_cm) > OUTLIER_THRESHOLD) {
        if (DEBUG_OUTPUT) {
            Serial.print("[FILTER] Outlier: ");
            Serial.print(distance_cm);
            Serial.println(" cm");
        }
        return;
    }
    
    n->last_distance_cm = distance_cm;
    updateDistance(*n, distance_cm);
    
    // Update long-range status
    n->is_long_range = isLongRange(*n);
}

// Records a ranging failure: updates neighbor entry with failure count and tracks stats for performance monitoring
void NeighborTable::recordRangingFailure(uint8_t neighbor_id) {
    Neighbor* n = getNeighbor(neighbor_id);
    if (n == nullptr) return;
    
    n->ranging_attempts++;
    n->consecutive_failures++;
    stats.total_attempts++;
    stats.timeout_failures++;
}

// Records a detected collision: updates neighbor entry with failure count and tracks stats for performance monitoring
void NeighborTable::recordCollision(uint8_t neighbor_id) {
    stats.collision_detections++;
    Neighbor* n = getNeighbor(neighbor_id);
    if (n != nullptr) {
        n->consecutive_failures++;
    }
}



// Maintains mesh connectivity by selecting the next neighbor to range with based on a priority system:
// 1. Stale long-range links (critical for mesh bridges), 2. Any stale links, 3. Round-robin among active neighbors
uint8_t NeighborTable::getNextRangingTarget() {
    pruneStaleNeighbors(); // Clean up stale neighbors before selection
    
    // Collect eligible neighbors
    struct Candidate {
        uint8_t id;
        uint8_t table_index;
        bool stale;
        bool long_range;
        float priority;
    };
    
    Candidate candidates[MAX_NEIGHBORS];
    uint8_t count = 0;
    
    for (int i = 0; i < MAX_NEIGHBORS; i++) {
        if (!neighbors[i].active) continue;
        if (neighbors[i].hello_count < MIN_HELLO_COUNT) continue;
        
        candidates[count].id = neighbors[i].id;
        candidates[count].table_index = i;
        candidates[count].stale = isLinkStale(neighbors[i]);
        candidates[count].long_range = neighbors[i].is_long_range;
        candidates[count].priority = calculatePriority(neighbors[i]);
        count++;
    }
    
    if (count == 0) {
        return 0;  // No neighbors
    }
    
#if RANGE_ALL_NEIGHBORS
    // PRIORITY 1: Stale long-range links (critical mesh links)
    for (int i = 0; i < count; i++) {
        int idx = (current_index + i) % count;
        if (candidates[idx].stale && candidates[idx].long_range) {
            current_index = (idx + 1) % count;
            if (DEBUG_OUTPUT) {
                Serial.print("[TARGET] Stale long-range: ");
                Serial.println(candidates[idx].id);
            }
            return candidates[idx].id;
        }
    }
    
    // PRIORITY 2: Any stale link
    for (int i = 0; i < count; i++) {
        int idx = (current_index + i) % count;
        if (candidates[idx].stale) {
            current_index = (idx + 1) % count;
            if (DEBUG_OUTPUT) {
                Serial.print("[TARGET] Stale: ");
                Serial.println(candidates[idx].id);
            }
            return candidates[idx].id;
        }
    }
    
    // PRIORITY 3: Normal round-robin
    uint8_t target_idx = current_index % count;
    uint8_t target = candidates[target_idx].id;
    current_index = (current_index + 1) % count;
    
    if (DEBUG_OUTPUT) {
        Serial.print("[TARGET] Round-robin ");
        Serial.print(target_idx + 1);
        Serial.print("/");
        Serial.print(count);
        Serial.print(": ");
        Serial.println(target);
    }
    
    return target;
    
#else
    // K-best selection (alternative, less mesh-friendly)
    // Sort by priority
    for (int i = 0; i < count - 1; i++) {
        for (int j = i + 1; j < count; j++) {
            if (candidates[j].priority > candidates[i].priority) {
                Candidate temp = candidates[i];
                candidates[i] = candidates[j];
                candidates[j] = temp;
            }
        }
    }
    
    uint8_t k = min(count, (uint8_t)K_NEIGHBORS);
    uint8_t idx = current_index % k;
    current_index = (current_index + 1) % k;
    
    return candidates[idx].id;
#endif
}

// =============================================================================
// PRIORITY CALCULATION
// =============================================================================
float NeighborTable::calculatePriority(const Neighbor& n) {
    float score = 0.0;
    
    // Recency of HELLO: exponential decay over 5 seconds, gives 1.0 for very recent, ~0.135 for 5s old, ~0.007 for 10s old
    // Multiplied by 25.0: recent neighbors get up to 25 priority points
    uint32_t hello_age = millis() - n.last_hello_time;
    float recency = exp(-((float)hello_age / 5000.0));
    score += recency * 25.0;
    
    // Success rate: 0.5 (default for new neighbors) to 1.0 (perfect success)
    // Multiplied by 20.0: reliable neighbors get up to 20 priority points
    float success_rate = 0.5;
    if (n.ranging_attempts > 0) {
        success_rate = (float)n.ranging_successes / (float)n.ranging_attempts;
    }
    score += success_rate * 20.0;
    
    // STALE LINK BOOST - critical for mesh maintenance: adds fixed priority to revive inactive links
    if (isLinkStale(n)) {
        score += STALE_LINK_PRIORITY;
    }
    
    // LONG RANGE BOOST - these are mesh bridge links: adds priority to maintain connectivity across clusters
    if (n.is_long_range) {
        score += BRIDGE_LINK_PRIORITY_BOOST;
    }
    
    // Signal quality: RSSI from -100 (worst) to 0 (best), normalized to 0-2 scale
    // Multiplied by 10.0: good signal neighbors get up to 20 priority points
    if (n.avg_rssi > -100) {
        float rssi_score = (n.avg_rssi + 100) / 50.0;  // -100 -> 0, 0 -> 2
        rssi_score = constrain(rssi_score, 0.0f, 2.0f);
        score += rssi_score * 10.0;
    }
    
    // Penalize failures: subtract 5 points per consecutive failure to avoid repeatedly trying bad links
    if (n.consecutive_failures > 0) {
        score -= n.consecutive_failures * 5.0;
    }
    
    return score;
}

// =============================================================================
// HELPERS
// =============================================================================
bool NeighborTable::isLinkStale(const Neighbor& n) {
    if (n.last_range_time == 0) return true;
    return (millis() - n.last_range_time) > LINK_STALE_THRESHOLD_MS;
}

bool NeighborTable::isLongRange(const Neighbor& n) {
    return (n.filtered_distance_cm > CLUSTER_DISTANCE_THRESHOLD_CM);
}

// =============================================================================
// MAINTENANCE
// =============================================================================
void NeighborTable::pruneStaleNeighbors() {
    uint32_t now = millis();
    
    for (int i = 0; i < MAX_NEIGHBORS; i++) {
        if (!neighbors[i].active) continue;
        
        if (now - neighbors[i].last_hello_time > NEIGHBOR_TIMEOUT_MS) {
            Serial.print("[MESH] Pruning node ");
            Serial.println(neighbors[i].id);
            
            neighbors[i].active = false;
            neighbors[i].id = 0;
            neighbors[i].hello_count = 0;
            neighbors[i].ranging_attempts = 0;
            neighbors[i].ranging_successes = 0;
            neighbors[i].history_count = 0;
        }
    }
}

// =============================================================================
// QUERIES
// =============================================================================
Neighbor* NeighborTable::getNeighbor(uint8_t neighbor_id) {
    for (int i = 0; i < MAX_NEIGHBORS; i++) {
        if (neighbors[i].active && neighbors[i].id == neighbor_id) {
            return &neighbors[i];
        }
    }
    return nullptr;
}

uint8_t NeighborTable::getActiveCount() {
    uint8_t count = 0;
    for (int i = 0; i < MAX_NEIGHBORS; i++) {
        if (neighbors[i].active) count++;
    }
    return count;
}

uint8_t NeighborTable::getEligibleCount() {
    uint8_t count = 0;
    for (int i = 0; i < MAX_NEIGHBORS; i++) {
        if (neighbors[i].active && neighbors[i].hello_count >= MIN_HELLO_COUNT) {
            count++;
        }
    }
    return count;
}

float NeighborTable::getSuccessRate(uint8_t neighbor_id) {
    Neighbor* n = getNeighbor(neighbor_id);
    if (n == nullptr) return 0;
    if (n->ranging_attempts == 0) return 0.5;
    return (float)n->ranging_successes / (float)n->ranging_attempts;
}

bool NeighborTable::hasSlotCollision(uint8_t neighbor_id) {
    Neighbor* n = getNeighbor(neighbor_id);
    if (n == nullptr) return false;
    return (n->slot == COMPUTE_MY_SLOT());
}

MeshStats NeighborTable::getStats() {
    return stats;
}


// =============================================================================
// PRIVATE HELPERS
// =============================================================================

// Finds existing neighbor entry or creates a new one if space available, initializing all fields
Neighbor* NeighborTable::findOrCreate(uint8_t neighbor_id) {
    // Look for existing
    for (int i = 0; i < MAX_NEIGHBORS; i++) {
        if (neighbors[i].active && neighbors[i].id == neighbor_id) {
            return &neighbors[i];
        }
    }
    
    // Create new
    for (int i = 0; i < MAX_NEIGHBORS; i++) {
        if (!neighbors[i].active) {
            neighbors[i].active = true;
            neighbors[i].id = neighbor_id;
            neighbors[i].last_hello_time = millis();
            neighbors[i].last_range_time = 0;
            neighbors[i].hello_count = 0;
            neighbors[i].ranging_attempts = 0;
            neighbors[i].ranging_successes = 0;
            neighbors[i].consecutive_failures = 0;
            neighbors[i].last_distance_cm = 0;
            neighbors[i].filtered_distance_cm = 0;
            neighbors[i].ema_distance_cm = 0;
            neighbors[i].history_index = 0;
            neighbors[i].history_count = 0;
            neighbors[i].last_rssi = -100;
            neighbors[i].avg_rssi = -100;
            neighbors[i].is_long_range = false;
            neighbors[i].slot = neighbor_id % NUM_SLOTS;
            
            for (int j = 0; j < FILTER_SIZE; j++) {
                neighbors[i].distance_history[j] = 0;
            }
            
            Serial.print("[MESH] Discovered node ");
            Serial.println(neighbor_id);
            
            return &neighbors[i];
        }
    }
    
    return nullptr;
}

// Updates neighbor's distance history with median filtering and exponential moving average for stable distance estimates
void NeighborTable::updateDistance(Neighbor& n, float new_distance) {
    if (!isValidDistance(new_distance)) return;
    
    n.distance_history[n.history_index] = new_distance;
    n.history_index = (n.history_index + 1) % FILTER_SIZE;
    if (n.history_count < FILTER_SIZE) {
        n.history_count++;
    }
    
    // Median filter
    float valid[FILTER_SIZE];
    int valid_count = 0;
    
    for (int i = 0; i < n.history_count; i++) {
        if (isValidDistance(n.distance_history[i])) {
            valid[valid_count++] = n.distance_history[i];
        }
    }
    
    if (valid_count > 0) {
        n.filtered_distance_cm = calculateMedian(valid, valid_count);
        
        // EMA update
        if (n.ema_distance_cm <= 0) {
            n.ema_distance_cm = n.filtered_distance_cm;
        } else {
            n.ema_distance_cm = EMA_ALPHA * n.filtered_distance_cm + 
                               (1 - EMA_ALPHA) * n.ema_distance_cm;
        }
    }
}

// Checks if distance measurement falls within acceptable bounds for filtering
bool NeighborTable::isValidDistance(float distance) {
    return (distance >= MIN_DISTANCE_CM && distance <= MAX_DISTANCE_CM);
}

// Computes median value from array using bubble sort for robust distance estimation
float NeighborTable::calculateMedian(float* arr, int size) {
    if (size == 0) return 0;
    if (size == 1) return arr[0];
    
    float temp[FILTER_SIZE];
    for (int i = 0; i < size; i++) {
        temp[i] = arr[i];
    }
    
    // Bubble sort
    for (int i = 0; i < size - 1; i++) {
        for (int j = i + 1; j < size; j++) {
            if (temp[j] < temp[i]) {
                float t = temp[i];
                temp[i] = temp[j];
                temp[j] = t;
            }
        }
    }
    
    if (size % 2 == 0) {
        return (temp[size / 2 - 1] + temp[size / 2]) / 2.0;
    }
    return temp[size / 2];
}

// =============================================================================
// DEBUG OUTPUT
// =============================================================================
void NeighborTable::printTable() {
    Serial.println("\n=== MESH NEIGHBORS ===");
    Serial.println("ID\tHello\tRange%\tDist(cm)\tRSSI\tStale\tLongR");
    
    uint32_t now = millis();
    
    for (int i = 0; i < MAX_NEIGHBORS; i++) {
        if (!neighbors[i].active) continue;
        
        Serial.print(neighbors[i].id);
        Serial.print("\t");
        Serial.print(neighbors[i].hello_count);
        Serial.print("\t");
        Serial.print((int)(getSuccessRate(neighbors[i].id) * 100));
        Serial.print("%\t");
        Serial.print(neighbors[i].filtered_distance_cm, 0);
        Serial.print("\t\t");
        Serial.print(neighbors[i].avg_rssi, 0);
        Serial.print("\t");
        Serial.print(isLinkStale(neighbors[i]) ? "YES" : "-");
        Serial.print("\t");
        Serial.println(neighbors[i].is_long_range ? "YES" : "-");
    }
    Serial.println("======================\n");
}

void NeighborTable::printMeshStatus() {
    Serial.println("\n=== MESH STATUS ===");
    Serial.print("Active: ");
    Serial.print(getActiveCount());
    Serial.print(" | Eligible: ");
    Serial.println(getEligibleCount());
    
    // Count long-range and stale
    uint8_t long_range = 0, stale = 0;
    for (int i = 0; i < MAX_NEIGHBORS; i++) {
        if (neighbors[i].active) {
            if (neighbors[i].is_long_range) long_range++;
            if (isLinkStale(neighbors[i])) stale++;
        }
    }
    
    Serial.print("Long-range links: ");
    Serial.print(long_range);
    Serial.print(" | Stale links: ");
    Serial.println(stale);
    
    Serial.print("Ranging: ");
    Serial.print(stats.total_successes);
    Serial.print("/");
    Serial.print(stats.total_attempts);
    if (stats.total_attempts > 0) {
        Serial.print(" (");
        Serial.print((int)(100.0 * stats.total_successes / stats.total_attempts));
        Serial.print("%)");
    }
    Serial.println();
    
    Serial.print("Collisions: ");
    Serial.println(stats.collision_detections);
    Serial.println("===================\n");
}

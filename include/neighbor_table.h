#pragma once
#include <Arduino.h>
#include "peer_config.h"

// =============================================================================
// MESH NEIGHBOR TABLE
// =============================================================================
// All nodes are equal peers. No anchors, no hierarchy.
// Tracks neighbors for mesh connectivity and ranging.
// =============================================================================

struct Neighbor {
    uint8_t id;
    uint32_t last_hello_time;
    uint32_t last_range_time;
    uint16_t hello_count;
    uint16_t ranging_attempts;
    uint16_t ranging_successes;
    uint16_t consecutive_failures;
    float last_distance_cm;
    float distance_history[FILTER_SIZE];
    uint8_t history_index;
    uint8_t history_count;
    float filtered_distance_cm;
    float ema_distance_cm;
    float last_rssi;
    float avg_rssi;
    bool active;
    bool is_long_range;  // Far neighbor (potential bridge link)
    uint8_t slot;
};

struct SelectedNeighbor {
    uint8_t id;
    float priority_score;
    bool is_stale;
    bool is_long_range;
};

struct MeshStats {
    uint32_t total_attempts;
    uint32_t total_successes;
    uint32_t collision_detections;
    uint32_t timeout_failures;
};

class NeighborTable {
public:
    NeighborTable();
    
    // HELLO Processing
    void processHello(uint8_t neighbor_id, uint8_t seq_num, uint8_t neighbor_count);
    void processHelloWithSlot(uint8_t neighbor_id, uint8_t seq_num, 
                              uint8_t neighbor_count, uint8_t slot_num);
    
    // Ranging Results
    void recordRangingSuccess(uint8_t neighbor_id, float distance_cm, float rssi);
    void recordRangingFailure(uint8_t neighbor_id);
    void recordCollision(uint8_t neighbor_id);
    
    // Target Selection - maintains mesh connectivity
    uint8_t getNextRangingTarget();
    
    // Maintenance
    void pruneStaleNeighbors();
    
    // Queries
    Neighbor* getNeighbor(uint8_t neighbor_id);
    const Neighbor* getNeighborArray() const { return neighbors; }  // For iteration
    uint8_t getActiveCount();
    uint8_t getEligibleCount();
    float getFilteredDistance(uint8_t neighbor_id);
    float getSuccessRate(uint8_t neighbor_id);
    bool hasSlotCollision(uint8_t neighbor_id);
    
    // Stats
    MeshStats getStats();
    void resetStats();
    
    // Debug
    void printTable();
    void printMeshStatus();

private:
    Neighbor neighbors[MAX_NEIGHBORS];
    uint8_t current_index;
    MeshStats stats;
    
    Neighbor* findOrCreate(uint8_t neighbor_id);
    float calculatePriority(const Neighbor& n);
    void updateDistance(Neighbor& n, float new_distance);
    bool isValidDistance(float distance);
    float calculateMedian(float* arr, int size);
    bool isLinkStale(const Neighbor& n);
    bool isLongRange(const Neighbor& n);
};

extern NeighborTable neighborTable;

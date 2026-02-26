#pragma once
#include <Arduino.h>
#include "peer_config.h"

// =============================================================================
// RANGING LOG
// =============================================================================
// Structured logging for off-device processing
// Each ranging attempt (success or failure) generates a log entry
// =============================================================================

// Log entry structure
struct RangeLogEntry {
    uint32_t timestamp_ms;      // millis() when ranging completed
    uint16_t frame_number;      // TDMA frame number
    uint8_t initiator_id;       // Who initiated the ranging
    uint8_t responder_id;       // Who responded
    float distance_cm;          // Measured distance (0 if failed)
    bool success;               // True if ranging succeeded
    float rssi_dbm;             // Signal strength
    float fp_rssi_dbm;          // First path signal strength (for NLOS detection)
};

class RangeLogger {
public:
    RangeLogger();
    
    // --- Logging ---
    // Log a successful ranging
    void logSuccess(uint8_t initiator, uint8_t responder, 
                    float distance_cm, float rssi, float fp_rssi,
                    uint16_t frame);
    
    // Log a failed ranging attempt
    void logFailure(uint8_t initiator, uint8_t responder, 
                    uint16_t frame, const char* reason);
    
    // Log a HELLO beacon event
    void logHello(uint8_t sender_id, uint8_t seq_num);
    
    // --- Output ---
    // Flush buffered logs to Serial (CSV format)
    void flushToSerial();
    
    // Get number of buffered entries
    uint8_t getBufferedCount();
    
    // --- Format Control ---
    // Enable/disable verbose mode
    void setVerbose(bool verbose);
    
    // Output as JSON instead of CSV
    void setJsonOutput(bool json);

private:
    RangeLogEntry buffer[LOG_BUFFER_SIZE];
    uint8_t write_index;
    uint8_t read_index;
    uint8_t count;
    bool verbose_mode;
    bool json_output;
    
    // Add entry to buffer
    void addEntry(const RangeLogEntry& entry);
    
    // Print single entry
    void printEntry(const RangeLogEntry& entry);
    void printEntryCSV(const RangeLogEntry& entry);
    void printEntryJSON(const RangeLogEntry& entry);
};

// Global logger instance
extern RangeLogger rangeLogger;

// =============================================================================
// CONVENIENCE MACROS
// =============================================================================
#define LOG_RANGE_SUCCESS(init, resp, dist, rssi, fp, frame) rangeLogger.logSuccess(init, resp, dist, rssi, fp, frame)

#define LOG_RANGE_FAILURE(init, resp, frame, reason) rangeLogger.logFailure(init, resp, frame, reason)

#define LOG_HELLO(sender, seq) rangeLogger.logHello(sender, seq)
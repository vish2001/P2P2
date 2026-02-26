#include "range_logger.h"

// Global instance
RangeLogger rangeLogger;

// =============================================================================
// CONSTRUCTOR
// =============================================================================
RangeLogger::RangeLogger() {
    write_index = 0;
    read_index = 0;
    count = 0;
    verbose_mode = false;
    json_output = false;
}

// =============================================================================
// LOGGING FUNCTIONS
// =============================================================================
void RangeLogger::logSuccess(uint8_t initiator, uint8_t responder,
                              float distance_cm, float rssi, float fp_rssi,
                              uint16_t frame) {
    RangeLogEntry entry;
    entry.timestamp_ms = millis();
    entry.frame_number = frame;
    entry.initiator_id = initiator;
    entry.responder_id = responder;
    entry.distance_cm = distance_cm;
    entry.success = true;
    entry.rssi_dbm = rssi;
    entry.fp_rssi_dbm = fp_rssi;
    
    addEntry(entry);
    
    if (verbose_mode) {
        Serial.print("[RANGE] ");
        Serial.print(initiator);
        Serial.print(" -> ");
        Serial.print(responder);
        Serial.print(": ");
        Serial.print(distance_cm, 1);
        Serial.print(" cm (RSSI: ");
        Serial.print(rssi, 1);
        Serial.println(" dBm)");
    }
}

void RangeLogger::logFailure(uint8_t initiator, uint8_t responder,
                              uint16_t frame, const char* reason) {
    RangeLogEntry entry;
    entry.timestamp_ms = millis();
    entry.frame_number = frame;
    entry.initiator_id = initiator;
    entry.responder_id = responder;
    entry.distance_cm = 0;
    entry.success = false;
    entry.rssi_dbm = -100;
    entry.fp_rssi_dbm = -100;
    
    addEntry(entry);
    
    if (verbose_mode) {
        Serial.print("[RANGE FAIL] ");
        Serial.print(initiator);
        Serial.print(" -> ");
        Serial.print(responder);
        Serial.print(": ");
        Serial.println(reason);
    }
}

void RangeLogger::logHello(uint8_t sender_id, uint8_t seq_num) {
    if (verbose_mode) {
        Serial.print("[HELLO] From ");
        Serial.print(sender_id);
        Serial.print(" seq=");
        Serial.println(seq_num);
    }
}

// =============================================================================
// BUFFER MANAGEMENT
// =============================================================================
void RangeLogger::addEntry(const RangeLogEntry& entry) {
    buffer[write_index] = entry;
    write_index = (write_index + 1) % LOG_BUFFER_SIZE;
    
    if (count < LOG_BUFFER_SIZE) {
        count++;
    } else {
        // Buffer full, overwrite oldest
        read_index = (read_index + 1) % LOG_BUFFER_SIZE;
    }
}

uint8_t RangeLogger::getBufferedCount() {
    return count;
}

// =============================================================================
// OUTPUT
// =============================================================================
void RangeLogger::flushToSerial() {
    while (count > 0) {
        printEntry(buffer[read_index]);
        read_index = (read_index + 1) % LOG_BUFFER_SIZE;
        count--;
    }
}

void RangeLogger::printEntry(const RangeLogEntry& entry) {
    if (json_output) {
        printEntryJSON(entry);
    } else {
        printEntryCSV(entry);
    }
}

void RangeLogger::printEntryCSV(const RangeLogEntry& entry) {
    // Format: timestamp,frame,initiator,responder,distance,success,rssi,fp_rssi
    Serial.print(entry.timestamp_ms);
    Serial.print(",");
    Serial.print(entry.frame_number);
    Serial.print(",");
    Serial.print(entry.initiator_id);
    Serial.print(",");
    Serial.print(entry.responder_id);
    Serial.print(",");
    Serial.print(entry.distance_cm, 2);
    Serial.print(",");
    Serial.print(entry.success ? "1" : "0");
    Serial.print(",");
    Serial.print(entry.rssi_dbm, 1);
    Serial.print(",");
    Serial.println(entry.fp_rssi_dbm, 1);
}

void RangeLogger::printEntryJSON(const RangeLogEntry& entry) {
    Serial.print("{\"ts\":");
    Serial.print(entry.timestamp_ms);
    Serial.print(",\"frame\":");
    Serial.print(entry.frame_number);
    Serial.print(",\"init\":");
    Serial.print(entry.initiator_id);
    Serial.print(",\"resp\":");
    Serial.print(entry.responder_id);
    Serial.print(",\"dist\":");
    Serial.print(entry.distance_cm, 2);
    Serial.print(",\"ok\":");
    Serial.print(entry.success ? "true" : "false");
    Serial.print(",\"rssi\":");
    Serial.print(entry.rssi_dbm, 1);
    Serial.print(",\"fp_rssi\":");
    Serial.print(entry.fp_rssi_dbm, 1);
    Serial.println("}");
}

// =============================================================================
// CONFIGURATION
// =============================================================================
void RangeLogger::setVerbose(bool verbose) {
    verbose_mode = verbose;
}

void RangeLogger::setJsonOutput(bool json) {
    json_output = json;
}
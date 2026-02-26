# P2P UWB Mesh + Cattle Behavior Classifier (Integrated v2)

## What Changed

### 1. DS-TWR Symmetric Formula Fix (`lib/DW3000/DW3000.cpp`)
The original `ds_processRTInfo()` had a bug where the clock offset sign was 
conditionally flipped based on which reply time was larger — an arbitrary condition 
unrelated to the actual clock frequency difference. This caused asymmetric distance 
measurements when nodes swapped initiator/responder roles.

**Fix:** Replaced with the standard Decawave APS013 formula:
```
ToF = (T_roundA × T_roundB − T_replyA × T_replyB) / (T_roundA + T_roundB + T_replyA + T_replyB)
```
This is mathematically symmetric — swapping A↔B gives identical results. Uses `int64_t` 
intermediates to prevent overflow.

### 2. MLP Classifier (`cattle_classifier2.h`)
Switched from the original Decision Tree (F1=0.9254) to the newer MLP model:
- **MLP primary** (F1=0.9327, 39→32→16→4 architecture)
- **39 selected features** (down from 60) — faster extraction
- DT and LR still available as fallbacks in `cattle_classifier2.h`

### 3. Classification Staleness Metadata
Every classification packet (`C`) now includes:
- `range_age_ms` — ms since this node's last successful ranging
- `nbr_count` — current number of mesh neighbors  
- `range_pct` — overall ranging success rate (0-100)

This lets the base station / PC client know whether the node's ranging data is fresh 
or stale (e.g. node moved out of range of neighbors but IMU still classifying).

### 4. Neighbor Staleness in `N` Packets
Neighbor info packets now include per-link staleness:
- `stale` — 0=fresh, 1=stale (link not updated within threshold)
- `stale_ms` — ms since last successful range to this neighbor

### 5. WiFi UDP Sender Updated
WiFi UDP path now supports the same classification + staleness packets as ESP-NOW.

---

## Packet Formats (CSV)

All packets from base station to PC are CSV:

```
timestamp_ms, station_id, TYPE, fields...
```

| Type | Format |
|------|--------|
| **R** (Range) | `ts,st,R,from,to,dist_cm,rssi,node_ts` |
| **N** (Neighbor) | `ts,st,N,node,nbr,hellos,range_pct,dist,rssi,stale,stale_ms` |
| **H** (Heartbeat) | `ts,st,H,node,frame,nbr_count,uptime` |
| **C** (Classify) | `ts,st,C,node,behavior,conf,ax,ay,az,gx,gy,gz,node_ts,range_age_ms,nbrs,range_pct` |

**Behavior codes:** 0=Walking, 1=Grazing, 2=Resting, 3=Misc

---

## Timing Budget

Nothing blocks the UWB state machine:

| Task | When | Duration | Impact on UWB |
|------|------|----------|---------------|
| IMU read | Every 100ms, LISTENING only | ~0.3ms I2C | None (gated to idle) |
| Classification | Every 2.5s (50% overlap) | ~2ms compute | None (runs in LISTENING) |
| ESP-NOW send | After classify + after range | ~0.5ms | Rate-limited, non-blocking |
| DS-TWR exchange | TDMA-scheduled | ~20ms | Normal operation |

The key guard: `sampleIMU()` only runs when `state == NodeState::LISTENING`. This 
prevents I2C bus activity from interrupting SPI transactions with the DW3000 during 
active ranging (POLL/RESP/FINAL/REPORT), which would corrupt timestamps.

---

## Build

```bash
# Node firmware
cd P2P-mesh-IMU
pio run -e node

# Base station firmware  
cd base_station
pio run
```

Override node ID via build flags:
```ini
build_flags = -DTAG_ID_OVERRIDE=54
```

---

## Files Modified vs Original

| File | Change |
|------|--------|
| `lib/DW3000/DW3000.cpp` | Symmetric DS-TWR formula |
| `src/main.cpp` | Uses classifier2, sends staleness metadata |
| `include/espnow_sender.h` | Extended `sendIMUClassification()` + `sendNeighborInfo()` |
| `src/espnow_sender.cpp` | Implements extended formats |
| `include/wifi_udp_sender.h` | Added classification + staleness |
| `src/wifi_udp_sender.cpp` | Implements extended formats |
| `include/peer_config.h` | Added WiFi config defaults |
| `base_station/src/main.cpp` | Parses extended C/N formats, tracks per-node behavior |

**Unchanged:** `neighbor_table.*`, `tdma_scheduler.*`, `range_logger.*`, `DW3000.h`, 
`cattle_classifier.h`, `cattle_classifier2.h`, `platformio.ini`, Python tools.

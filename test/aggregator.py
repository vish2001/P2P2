#!/usr/bin/env python3
"""
UWB MESH DATA AGGREGATOR

Collects ranging data from multiple base stations and aggregates
into a single distance matrix for localization.

Supports:
- Multiple serial ports (USB-connected base stations)
- UDP input (WiFi-connected base stations)
- Deduplication of measurements received by multiple stations
- Wall-clock timestamps for all data
- CSV logging with timestamps

Usage:
    python aggregator.py --serial COM3 COM4 COM5
    python aggregator.py --udp 5000
    python aggregator.py --serial COM3 --udp 5000
    python aggregator.py --auto --log mesh_data.csv
"""

import argparse
import threading
import time
import serial
import serial.tools.list_ports
import socket
from collections import defaultdict
from dataclasses import dataclass
from typing import Dict, List, Optional, Set
from datetime import datetime
import json
import os

# =============================================================================
# DATA STRUCTURES
# =============================================================================

@dataclass
class RangingMeasurement:
    from_id: int
    to_id: int
    distance_cm: float
    rssi: float
    node_timestamp: int      # Timestamp from node (millis since boot)
    station_id: int
    receive_time: float      # Unix timestamp when received
    wall_clock: str          # Human-readable timestamp

@dataclass
class NeighborInfo:
    node_id: int
    neighbor_id: int
    hello_count: int
    range_pct: int
    distance_cm: float
    rssi: float
    receive_time: float
    wall_clock: str

@dataclass 
class NodeInfo:
    node_id: int
    last_seen: float
    neighbor_count: int
    frame_num: int
    uptime_ms: int

# =============================================================================
# AGGREGATOR
# =============================================================================

class MeshAggregator:
    def __init__(self, max_accept_distance_cm=3000.0, log_file=None):
        self.measurements: Dict[tuple, RangingMeasurement] = {}  # (from, to) -> latest
        self.measurement_history: List[RangingMeasurement] = []
        self.neighbor_table: Dict[tuple, NeighborInfo] = {}  # (node, neighbor) -> info
        self.nodes: Dict[int, NodeInfo] = {}
        self.stations_seen: Set[int] = set()
        self.lock = threading.Lock()
        
        # Configuration
        self.max_accept_distance = max_accept_distance_cm
        
        # Deduplication: measurements within this window are considered same
        self.dedup_window_ms = 500
        
        # Stats
        self.total_received = 0
        self.duplicates_filtered = 0
        self.distance_filtered = 0
        
        # CSV logging
        self.log_file = None
        if log_file:
            self.log_file = open(log_file, 'w')
            self._write_csv_header()
            print(f"[LOG] Logging to {log_file}")
        
    def _write_csv_header(self):
        """Write CSV header"""
        if self.log_file:
            self.log_file.write("timestamp,wall_clock,type,station_id,from_id,to_id,distance_cm,rssi,node_timestamp,extra\n")
            self.log_file.flush()
            
    def _log_csv(self, msg_type: str, station_id: int, data: dict):
        """Log a line to CSV"""
        if not self.log_file:
            return
            
        now = time.time()
        wall_clock = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
        
        line = f"{now:.3f},{wall_clock},{msg_type},{station_id},"
        line += f"{data.get('from_id', '')},{data.get('to_id', '')},"
        line += f"{data.get('distance_cm', '')},{data.get('rssi', '')},"
        line += f"{data.get('node_timestamp', '')},{data.get('extra', '')}\n"
        
        self.log_file.write(line)
        self.log_file.flush()
        
    def process_line(self, line: str, source: str = "unknown"):
        """Process a line of data from any source"""
        line = line.strip()
        if not line or line.startswith('#'):
            return
            
        parts = line.split(',')
        
        try:
            # New CSV format from base station: timestamp,station,type,data...
            # e.g., "12345,1,R,1,2,150.5,-65,1000"
            if len(parts) >= 4 and parts[2] in ['R', 'N', 'H']:
                base_ts = int(parts[0])  # Base station timestamp
                station_id = int(parts[1])
                msg_type = parts[2]
                
                if msg_type == 'R' and len(parts) >= 8:
                    # Ranging: timestamp,station,R,from,to,distance,rssi,node_ts
                    from_id = int(parts[3])
                    to_id = int(parts[4])
                    distance = float(parts[5])
                    rssi = float(parts[6])
                    node_ts = int(parts[7]) if len(parts) > 7 else 0
                    
                    self._add_ranging(station_id, from_id, to_id, distance, rssi, node_ts)
                    
                elif msg_type == 'N' and len(parts) >= 9:
                    # Neighbor: timestamp,station,N,node,neighbor,hello,range_pct,dist,rssi
                    node_id = int(parts[3])
                    neighbor_id = int(parts[4])
                    hello_count = int(parts[5])
                    range_pct = int(parts[6])
                    distance = float(parts[7])
                    rssi = float(parts[8])
                    
                    self._add_neighbor(station_id, node_id, neighbor_id, hello_count, 
                                      range_pct, distance, rssi)
                    
                elif msg_type == 'H' and len(parts) >= 7:
                    # Heartbeat: timestamp,station,H,node,frame,neighbors,uptime
                    node_id = int(parts[3])
                    frame_num = int(parts[4])
                    neighbor_count = int(parts[5])
                    uptime = int(parts[6]) if len(parts) > 6 else 0
                    
                    self._update_node(station_id, node_id, frame_num, neighbor_count, uptime)
                return
            
            # Old format: S,<station>,R,<from>,<to>,<distance>,<rssi>,<timestamp>
            if len(parts) >= 7 and parts[0] == 'S' and parts[2] == 'R':
                station_id = int(parts[1])
                from_id = int(parts[3])
                to_id = int(parts[4])
                distance = float(parts[5])
                rssi = float(parts[6])
                timestamp = int(parts[7]) if len(parts) > 7 else 0
                
                self._add_ranging(station_id, from_id, to_id, distance, rssi, timestamp)
                
            # Direct from node: R,<from>,<to>,<distance>,<rssi>,<timestamp>
            elif len(parts) >= 5 and parts[0] == 'R':
                from_id = int(parts[1])
                to_id = int(parts[2])
                distance = float(parts[3])
                rssi = float(parts[4]) if len(parts) > 4 else -50
                timestamp = int(parts[5]) if len(parts) > 5 else 0
                
                self._add_ranging(0, from_id, to_id, distance, rssi, timestamp)
                
            # Old format heartbeat: S,<station>,H,<node>,<frame>,<neighbors>,<uptime>
            elif len(parts) >= 6 and parts[0] == 'S' and parts[2] == 'H':
                station_id = int(parts[1])
                node_id = int(parts[3])
                frame_num = int(parts[4])
                neighbor_count = int(parts[5])
                uptime = int(parts[6]) if len(parts) > 6 else 0
                
                self._update_node(station_id, node_id, frame_num, neighbor_count, uptime)
                
            # Direct heartbeat: H,<node>,<frame>,<neighbors>,<uptime>
            elif len(parts) >= 4 and parts[0] == 'H':
                node_id = int(parts[1])
                frame_num = int(parts[2])
                neighbor_count = int(parts[3])
                uptime = int(parts[4]) if len(parts) > 4 else 0
                
                self._update_node(0, node_id, frame_num, neighbor_count, uptime)
                
            # Neighbor info: N,<node>,<neighbor>,<hello>,<range_pct>,<dist>,<rssi>
            elif len(parts) >= 7 and parts[0] == 'N':
                node_id = int(parts[1])
                neighbor_id = int(parts[2])
                hello_count = int(parts[3])
                range_pct = int(parts[4])
                distance = float(parts[5])
                rssi = float(parts[6])
                
                self._add_neighbor(0, node_id, neighbor_id, hello_count, range_pct, distance, rssi)
                
        except (ValueError, IndexError) as e:
            pass  # Ignore malformed lines
            
    def _add_ranging(self, station_id: int, from_id: int, to_id: int, 
                     distance: float, rssi: float, node_timestamp: int):
        """Add a ranging measurement with deduplication"""
        
        self.total_received += 1
        self.stations_seen.add(station_id)
        
        # Filter by max accept distance
        if distance > self.max_accept_distance:
            self.distance_filtered += 1
            return
        
        # Normalize key (always smaller ID first)
        key = (min(from_id, to_id), max(from_id, to_id))
        now = time.time()
        wall_clock = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
        
        with self.lock:
            # Check for duplicate (same measurement from different station)
            if key in self.measurements:
                existing = self.measurements[key]
                time_diff = abs(now - existing.receive_time) * 1000
                
                # If within dedup window and similar distance, it's a duplicate
                if time_diff < self.dedup_window_ms:
                    dist_diff = abs(distance - existing.distance_cm)
                    if dist_diff < 50:  # 50cm tolerance
                        self.duplicates_filtered += 1
                        return
            
            # Store measurement
            meas = RangingMeasurement(
                from_id=from_id,
                to_id=to_id,
                distance_cm=distance,
                rssi=rssi,
                node_timestamp=node_timestamp,
                station_id=station_id,
                receive_time=now,
                wall_clock=wall_clock
            )
            
            self.measurements[key] = meas
            self.measurement_history.append(meas)
            
            # Keep history bounded
            if len(self.measurement_history) > 10000:
                self.measurement_history = self.measurement_history[-5000:]
                
        # Log to CSV
        self._log_csv('R', station_id, {
            'from_id': from_id,
            'to_id': to_id,
            'distance_cm': distance,
            'rssi': rssi,
            'node_timestamp': node_timestamp
        })
                
        # Print
        print(f"[{wall_clock}] [{station_id}] R: {from_id}<->{to_id}: {distance:.1f}cm ({rssi:.1f}dBm)")
        
    def _add_neighbor(self, station_id: int, node_id: int, neighbor_id: int,
                     hello_count: int, range_pct: int, distance: float, rssi: float):
        """Add neighbor table entry"""
        now = time.time()
        wall_clock = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
        
        key = (node_id, neighbor_id)
        
        with self.lock:
            self.neighbor_table[key] = NeighborInfo(
                node_id=node_id,
                neighbor_id=neighbor_id,
                hello_count=hello_count,
                range_pct=range_pct,
                distance_cm=distance,
                rssi=rssi,
                receive_time=now,
                wall_clock=wall_clock
            )
            
        # Log to CSV
        self._log_csv('N', station_id, {
            'from_id': node_id,
            'to_id': neighbor_id,
            'distance_cm': distance,
            'rssi': rssi,
            'extra': f"hello={hello_count};range_pct={range_pct}"
        })
            
        print(f"[{wall_clock}] [{station_id}] N: {node_id}->{neighbor_id}: {distance:.1f}cm (hello={hello_count}, {range_pct}%)")
        
    def _update_node(self, station_id: int, node_id: int, frame_num: int, 
                    neighbor_count: int, uptime_ms: int):
        """Update node info"""
        now = time.time()
        wall_clock = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
        
        with self.lock:
            self.nodes[node_id] = NodeInfo(
                node_id=node_id,
                last_seen=now,
                neighbor_count=neighbor_count,
                frame_num=frame_num,
                uptime_ms=uptime_ms
            )
            self.stations_seen.add(station_id)
            
        # Log to CSV
        self._log_csv('H', station_id, {
            'from_id': node_id,
            'extra': f"frame={frame_num};neighbors={neighbor_count};uptime={uptime_ms}"
        })
            
        print(f"[{wall_clock}] [{station_id}] H: node {node_id}, {neighbor_count} neighbors, frame {frame_num}")
            
    def get_distance_matrix(self, max_age_sec: float = 15.0):
        """Get current distance matrix"""
        with self.lock:
            now = time.time()
            
            # Find active nodes
            active_nodes = set()
            for (a, b), meas in self.measurements.items():
                if now - meas.receive_time < max_age_sec:
                    active_nodes.add(a)
                    active_nodes.add(b)
            
            node_list = sorted(active_nodes)
            n = len(node_list)
            
            if n < 2:
                return None, node_list
            
            # Build matrix
            import numpy as np
            D = np.full((n, n), np.nan)
            np.fill_diagonal(D, 0)
            
            for (a, b), meas in self.measurements.items():
                if now - meas.receive_time > max_age_sec:
                    continue
                if a in active_nodes and b in active_nodes:
                    i = node_list.index(a)
                    j = node_list.index(b)
                    D[i, j] = meas.distance_cm
                    D[j, i] = meas.distance_cm
                    
            return D, node_list
            
    def get_neighbor_table_distances(self, max_age_sec: float = 30.0):
        """Get distances from neighbor table reports (more complete than ranging)"""
        with self.lock:
            now = time.time()
            distances = {}
            
            for (node, neighbor), info in self.neighbor_table.items():
                if now - info.receive_time < max_age_sec:
                    key = (min(node, neighbor), max(node, neighbor))
                    # Use the most recent report
                    if key not in distances or info.receive_time > distances[key][1]:
                        distances[key] = (info.distance_cm, info.receive_time)
                        
            return {k: v[0] for k, v in distances.items()}
            
    def get_stats(self):
        """Get aggregator statistics"""
        with self.lock:
            now = time.time()
            active_links = sum(1 for m in self.measurements.values() 
                              if now - m.receive_time < 15)
            active_nodes = len([n for n in self.nodes.values() 
                               if now - n.last_seen < 30])
            
            return {
                'total_received': self.total_received,
                'duplicates_filtered': self.duplicates_filtered,
                'distance_filtered': self.distance_filtered,
                'active_links': active_links,
                'active_nodes': active_nodes,
                'neighbor_entries': len(self.neighbor_table),
                'stations_seen': len(self.stations_seen),
                'station_ids': list(self.stations_seen)
            }
            
    def print_status(self):
        """Print current status"""
        stats = self.get_stats()
        wall_clock = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        
        print("\n" + "="*60)
        print(f"MESH AGGREGATOR STATUS - {wall_clock}")
        print("="*60)
        print(f"Base stations: {stats['station_ids']}")
        print(f"Active nodes: {stats['active_nodes']}")
        print(f"Active ranging links: {stats['active_links']}")
        print(f"Neighbor table entries: {stats['neighbor_entries']}")
        print(f"Total received: {stats['total_received']}")
        print(f"Duplicates filtered: {stats['duplicates_filtered']}")
        print(f"Distance filtered (>{self.max_accept_distance}cm): {stats['distance_filtered']}")
        
        # Show node details
        if self.nodes:
            print("\nNode Details:")
            now = time.time()
            for nid, info in sorted(self.nodes.items()):
                age = now - info.last_seen
                status = "✓" if age < 30 else "?"
                print(f"  {status} Node {nid}: {info.neighbor_count} neighbors, "
                      f"frame {info.frame_num}, uptime {info.uptime_ms//1000}s, "
                      f"seen {age:.0f}s ago")
        
        print("="*60 + "\n")
        
    def close(self):
        """Close log file"""
        if self.log_file:
            self.log_file.close()


# =============================================================================
# SERIAL READER
# =============================================================================

class SerialReader:
    def __init__(self, port: str, aggregator: MeshAggregator, baudrate: int = 921600):
        self.port = port
        self.aggregator = aggregator
        self.baudrate = baudrate
        self.running = False
        self.thread = None
        
    def start(self):
        self.running = True
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()
        print(f"[SERIAL] Started reading from {self.port}")
        
    def stop(self):
        self.running = False
        
    def _loop(self):
        try:
            ser = serial.Serial(self.port, self.baudrate, timeout=1)
            while self.running:
                try:
                    line = ser.readline().decode('utf-8', errors='ignore')
                    if line:
                        self.aggregator.process_line(line, f"serial:{self.port}")
                except Exception as e:
                    print(f"[SERIAL] {self.port} read error: {e}")
                    time.sleep(1)
            ser.close()
        except Exception as e:
            print(f"[SERIAL] {self.port} failed to open: {e}")


# =============================================================================
# UDP READER
# =============================================================================

class UDPReader:
    def __init__(self, port: int, aggregator: MeshAggregator):
        self.port = port
        self.aggregator = aggregator
        self.running = False
        self.thread = None
        
    def start(self):
        self.running = True
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()
        print(f"[UDP] Listening on port {self.port}")
        
    def stop(self):
        self.running = False
        
    def _loop(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind(('0.0.0.0', self.port))
        sock.settimeout(1.0)
        
        while self.running:
            try:
                data, addr = sock.recvfrom(1024)
                line = data.decode('utf-8', errors='ignore')
                self.aggregator.process_line(line, f"udp:{addr[0]}")
            except socket.timeout:
                continue
            except Exception as e:
                print(f"[UDP] Error: {e}")
                
        sock.close()


# =============================================================================
# AUTO-DETECT SERIAL PORTS
# =============================================================================

def find_serial_ports():
    """Find all connected serial ports that look like ESP32"""
    ports = []
    for port in serial.tools.list_ports.comports():
        # Common ESP32 USB-Serial chips
        if any(x in port.description.lower() for x in ['cp210', 'ch340', 'ftdi', 'usb']):
            ports.append(port.device)
    return ports


# =============================================================================
# MAIN
# =============================================================================

def main():
    parser = argparse.ArgumentParser(description='UWB Mesh Data Aggregator')
    parser.add_argument('--serial', nargs='*', help='Serial ports (e.g., COM3 COM4 or /dev/ttyUSB0)')
    parser.add_argument('--udp', type=int, help='UDP port to listen on')
    parser.add_argument('--auto', action='store_true', help='Auto-detect serial ports')
    parser.add_argument('--baudrate', type=int, default=921600, help='Serial baudrate')
    parser.add_argument('--max-distance', type=float, default=3000, help='Max accept distance in cm (default: 3000)')
    parser.add_argument('--log', type=str, help='CSV log file path (e.g., mesh_data.csv)')
    
    args = parser.parse_args()
    
    print("="*60)
    print("  UWB MESH DATA AGGREGATOR")
    print("  Collects data from multiple base stations")
    print(f"  Max accept distance: {args.max_distance} cm")
    if args.log:
        print(f"  Logging to: {args.log}")
    print("="*60)
    
    aggregator = MeshAggregator(
        max_accept_distance_cm=args.max_distance,
        log_file=args.log
    )
    readers = []
    
    # Auto-detect serial ports
    if args.auto:
        ports = find_serial_ports()
        if ports:
            print(f"[AUTO] Found serial ports: {ports}")
            args.serial = ports
        else:
            print("[AUTO] No serial ports found")
    
    # Start serial readers
    if args.serial:
        for port in args.serial:
            reader = SerialReader(port, aggregator, args.baudrate)
            reader.start()
            readers.append(reader)
    
    # Start UDP reader
    if args.udp:
        reader = UDPReader(args.udp, aggregator)
        reader.start()
        readers.append(reader)
    
    if not readers:
        print("\nNo input sources specified!")
        print("Usage:")
        print("  python aggregator.py --serial COM3 COM4")
        print("  python aggregator.py --udp 5000")
        print("  python aggregator.py --auto")
        print("  python aggregator.py --auto --log mesh_data.csv")
        return
    
    print("\nListening for mesh data...\n")
    print("Data types received:")
    print("  R = Ranging measurement (from, to, distance)")
    print("  N = Neighbor table entry (node's view of neighbor)")
    print("  H = Heartbeat (node status)")
    print()
    
    # Main loop - print status periodically
    try:
        while True:
            time.sleep(10)
            aggregator.print_status()
    except KeyboardInterrupt:
        print("\nShutting down...")
        for reader in readers:
            reader.stop()
        aggregator.close()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""
UWB MESH DATA LOGGER & SERVER - Raspberry Pi

Runs on Raspberry Pi connected to ESP32 base station receiver.
- Reads ESP-NOW data from ESP32 via Serial
- Logs all data to CSV with wall-clock timestamps
- Streams data to PC clients over TCP for real-time visualization
- Optionally broadcasts via UDP for discovery

Architecture:
    [Mesh Nodes] --ESP-NOW--> [ESP32] --Serial--> [This Script] --TCP--> [PC Visualizer]

Usage:
    python mesh_logger.py --serial /dev/ttyUSB0 --log mesh_data.csv --port 5000
    
    On PC, connect with:
    python mesh_client.py --host raspberrypi.local --port 5000

Requirements:
    pip install pyserial
"""

import argparse
import serial
import serial.tools.list_ports
import socket
import threading
import time
import os
import json
from datetime import datetime
from collections import deque
from typing import List, Dict, Set

# =============================================================================
# CONFIGURATION
# =============================================================================

DEFAULT_SERIAL = '/dev/ttyUSB0'
DEFAULT_BAUDRATE = 921600
DEFAULT_TCP_PORT = 5000
DEFAULT_LOG_DIR = '/home/pi/uwb_logs'

# =============================================================================
# DATA STORAGE
# =============================================================================

class MeshDataStore:
    """Thread-safe storage for mesh data"""
    
    def __init__(self, log_file=None):
        self.lock = threading.Lock()
        
        # Current state
        self.distances: Dict[tuple, dict] = {}  # (from, to) -> {distance, rssi, time}
        self.neighbors: Dict[tuple, dict] = {}  # (node, neighbor) -> {info}
        self.nodes: Dict[int, dict] = {}        # node_id -> {info}
        self.classifications: Dict[int, dict] = {}  # node_id -> {behavior, imu, time}
        
        # Recent data for streaming (last 1000 lines)
        self.recent_lines: deque = deque(maxlen=1000)
        
        # Stats
        self.total_packets = 0
        self.start_time = time.time()
        
        # Logging
        self.log_file = None
        if log_file:
            os.makedirs(os.path.dirname(log_file) or '.', exist_ok=True)
            self.log_file = open(log_file, 'a')
            self._write_header()
            
    def _write_header(self):
        """Write CSV header if file is empty"""
        if self.log_file and self.log_file.tell() == 0:
            header = "unix_ts,wall_clock,type,station_id,from_id,to_id,distance_cm,rssi,node_ts,extra\n"
            self.log_file.write(header)
            self.log_file.flush()
            
    def process_line(self, line: str):
        """Process a line from ESP32 and store/log it"""
        line = line.strip()
        if not line or line.startswith('#'):
            return None
            
        now = time.time()
        wall_clock = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
        
        parts = line.split(',')
        
        try:
            # Parse based on format
            # New CSV: timestamp,station,type,data...
            if len(parts) >= 4 and parts[2] in ['R', 'N', 'H', 'C']:
                base_ts = parts[0]
                station_id = int(parts[1])
                msg_type = parts[2]
                
                record = {
                    'unix_ts': now,
                    'wall_clock': wall_clock,
                    'type': msg_type,
                    'station_id': station_id,
                    'raw': line
                }
                
                if msg_type == 'R' and len(parts) >= 8:
                    record.update({
                        'from_id': int(parts[3]),
                        'to_id': int(parts[4]),
                        'distance_cm': float(parts[5]),
                        'rssi': float(parts[6]),
                        'node_ts': parts[7] if len(parts) > 7 else ''
                    })
                    self._store_ranging(record)
                    
                elif msg_type == 'N' and len(parts) >= 9:
                    record.update({
                        'from_id': int(parts[3]),  # node
                        'to_id': int(parts[4]),    # neighbor
                        'hello_count': int(parts[5]),
                        'range_pct': int(parts[6]),
                        'distance_cm': float(parts[7]),
                        'rssi': float(parts[8])
                    })
                    self._store_neighbor(record)
                    
                elif msg_type == 'H' and len(parts) >= 7:
                    record.update({
                        'from_id': int(parts[3]),  # node
                        'frame_num': int(parts[4]),
                        'neighbor_count': int(parts[5]),
                        'uptime_ms': int(parts[6]) if len(parts) > 6 else 0
                    })
                    self._store_heartbeat(record)
                    
                elif msg_type == 'C' and len(parts) >= 13:
                    record.update({
                        'from_id': int(parts[3]),   # node
                        'behavior': int(parts[4]),   # 0-3
                        'confidence': int(parts[5]), # 1-3
                        'ax': float(parts[6]),
                        'ay': float(parts[7]),
                        'az': float(parts[8]),
                        'gx': float(parts[9]),
                        'gy': float(parts[10]),
                        'gz': float(parts[11]),
                        'node_ts': parts[12]
                    })
                    self._store_classification(record)
                    
                self._log_record(record)
                return record
                
            # Direct format: R,from,to,dist,rssi,ts or N,... or H,...
            elif len(parts) >= 4 and parts[0] in ['R', 'N', 'H', 'C']:
                msg_type = parts[0]
                
                record = {
                    'unix_ts': now,
                    'wall_clock': wall_clock,
                    'type': msg_type,
                    'station_id': 1,
                    'raw': line
                }
                
                if msg_type == 'R' and len(parts) >= 5:
                    record.update({
                        'from_id': int(parts[1]),
                        'to_id': int(parts[2]),
                        'distance_cm': float(parts[3]),
                        'rssi': float(parts[4]) if len(parts) > 4 else -50,
                        'node_ts': parts[5] if len(parts) > 5 else ''
                    })
                    self._store_ranging(record)
                    
                elif msg_type == 'N' and len(parts) >= 7:
                    record.update({
                        'from_id': int(parts[1]),
                        'to_id': int(parts[2]),
                        'hello_count': int(parts[3]),
                        'range_pct': int(parts[4]),
                        'distance_cm': float(parts[5]),
                        'rssi': float(parts[6])
                    })
                    self._store_neighbor(record)
                    
                elif msg_type == 'H' and len(parts) >= 4:
                    record.update({
                        'from_id': int(parts[1]),
                        'frame_num': int(parts[2]),
                        'neighbor_count': int(parts[3]),
                        'uptime_ms': int(parts[4]) if len(parts) > 4 else 0
                    })
                    self._store_heartbeat(record)
                    
                elif msg_type == 'C' and len(parts) >= 10:
                    record.update({
                        'from_id': int(parts[1]),
                        'behavior': int(parts[2]),
                        'confidence': int(parts[3]),
                        'ax': float(parts[4]),
                        'ay': float(parts[5]),
                        'az': float(parts[6]),
                        'gx': float(parts[7]),
                        'gy': float(parts[8]),
                        'gz': float(parts[9]),
                        'node_ts': parts[10] if len(parts) > 10 else ''
                    })
                    self._store_classification(record)
                    
                self._log_record(record)
                return record
                
        except (ValueError, IndexError) as e:
            pass
            
        return None
        
    def _store_ranging(self, record):
        """Store ranging measurement"""
        with self.lock:
            key = (min(record['from_id'], record['to_id']), 
                   max(record['from_id'], record['to_id']))
            self.distances[key] = {
                'distance_cm': record['distance_cm'],
                'rssi': record['rssi'],
                'time': record['unix_ts']
            }
            self.total_packets += 1
            
    def _store_neighbor(self, record):
        """Store neighbor table entry"""
        with self.lock:
            key = (record['from_id'], record['to_id'])
            self.neighbors[key] = {
                'hello_count': record['hello_count'],
                'range_pct': record['range_pct'],
                'distance_cm': record['distance_cm'],
                'rssi': record['rssi'],
                'time': record['unix_ts']
            }
            self.total_packets += 1
            
    def _store_heartbeat(self, record):
        """Store node heartbeat"""
        with self.lock:
            self.nodes[record['from_id']] = {
                'frame_num': record['frame_num'],
                'neighbor_count': record['neighbor_count'],
                'uptime_ms': record.get('uptime_ms', 0),
                'time': record['unix_ts']
            }
            self.total_packets += 1

    BEHAVIOR_NAMES = ['Walking', 'Grazing', 'Resting', 'Misc']
    
    def _store_classification(self, record):
        """Store IMU classification and raw sensor data"""
        with self.lock:
            beh_idx = record.get('behavior', 3)
            beh_name = self.BEHAVIOR_NAMES[beh_idx] if 0 <= beh_idx <= 3 else 'Unknown'
            self.classifications[record['from_id']] = {
                'behavior': beh_idx,
                'behavior_name': beh_name,
                'confidence': record.get('confidence', 0),
                'ax': record.get('ax', 0), 'ay': record.get('ay', 0), 'az': record.get('az', 0),
                'gx': record.get('gx', 0), 'gy': record.get('gy', 0), 'gz': record.get('gz', 0),
                'time': record['unix_ts']
            }
            self.total_packets += 1
            
    def _log_record(self, record):
        """Log record to CSV file"""
        if not self.log_file:
            return
            
        # Format CSV line
        csv_line = f"{record['unix_ts']:.3f},{record['wall_clock']},{record['type']},"
        csv_line += f"{record.get('station_id', '')},"
        csv_line += f"{record.get('from_id', '')},{record.get('to_id', '')},"
        csv_line += f"{record.get('distance_cm', '')},{record.get('rssi', '')},"
        csv_line += f"{record.get('node_ts', '')},"
        
        # Extra info
        extra_parts = []
        if 'hello_count' in record:
            extra_parts.append(f"hello={record['hello_count']}")
        if 'range_pct' in record:
            extra_parts.append(f"range_pct={record['range_pct']}")
        if 'frame_num' in record:
            extra_parts.append(f"frame={record['frame_num']}")
        if 'neighbor_count' in record:
            extra_parts.append(f"neighbors={record['neighbor_count']}")
        if 'uptime_ms' in record:
            extra_parts.append(f"uptime={record['uptime_ms']}")
        if 'behavior' in record:
            beh_name = self.BEHAVIOR_NAMES[record['behavior']] if 0 <= record['behavior'] <= 3 else '?'
            extra_parts.append(f"beh={beh_name}")
            extra_parts.append(f"conf={record.get('confidence', 0)}")
            extra_parts.append(f"ax={record.get('ax', 0):.2f}")
            extra_parts.append(f"ay={record.get('ay', 0):.2f}")
            extra_parts.append(f"az={record.get('az', 0):.2f}")
            extra_parts.append(f"gx={record.get('gx', 0):.1f}")
            extra_parts.append(f"gy={record.get('gy', 0):.1f}")
            extra_parts.append(f"gz={record.get('gz', 0):.1f}")
        csv_line += ';'.join(extra_parts) + "\n"
        
        self.log_file.write(csv_line)
        self.log_file.flush()
        
        # Store for streaming
        with self.lock:
            self.recent_lines.append(record)
            
    def get_current_state(self):
        """Get current mesh state as JSON"""
        with self.lock:
            now = time.time()
            
            # Filter to recent data (last 30 seconds)
            distances = {
                f"{k[0]}-{k[1]}": v for k, v in self.distances.items()
                if now - v['time'] < 30
            }
            neighbors = {
                f"{k[0]}->{k[1]}": v for k, v in self.neighbors.items()
                if now - v['time'] < 60
            }
            nodes = {
                str(k): v for k, v in self.nodes.items()
                if now - v['time'] < 60
            }
            classifications = {
                str(k): v for k, v in self.classifications.items()
                if now - v['time'] < 30
            }
            
            return {
                'timestamp': now,
                'wall_clock': datetime.now().isoformat(),
                'distances': distances,
                'neighbors': neighbors,
                'nodes': nodes,
                'classifications': classifications,
                'stats': {
                    'total_packets': self.total_packets,
                    'uptime': now - self.start_time,
                    'active_links': len(distances),
                    'active_nodes': len(nodes)
                }
            }
            
    def get_recent_records(self, count=100):
        """Get recent records for streaming"""
        with self.lock:
            return list(self.recent_lines)[-count:]
            
    def close(self):
        if self.log_file:
            self.log_file.close()


# =============================================================================
# SERIAL READER
# =============================================================================

class SerialReader:
    """Reads data from ESP32 via Serial"""
    
    def __init__(self, port: str, baudrate: int, data_store: MeshDataStore):
        self.port = port
        self.baudrate = baudrate
        self.data_store = data_store
        self.running = False
        self.thread = None
        self.serial = None
        self.lines_read = 0
        
    def start(self):
        self.running = True
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()
        
    def stop(self):
        self.running = False
        if self.serial:
            self.serial.close()
            
    def _loop(self):
        while self.running:
            try:
                self.serial = serial.Serial(self.port, self.baudrate, timeout=1)
                print(f"[SERIAL] Connected to {self.port}")
                
                while self.running:
                    try:
                        line = self.serial.readline().decode('utf-8', errors='ignore')
                        if line:
                            self.lines_read += 1
                            record = self.data_store.process_line(line)
                            if record:
                                self._print_record(record)
                    except Exception as e:
                        print(f"[SERIAL] Read error: {e}")
                        break
                        
            except Exception as e:
                print(f"[SERIAL] Connection error: {e}")
                time.sleep(2)
                
    def _print_record(self, record):
        """Print record to console"""
        t = record['type']
        ts = record['wall_clock']
        
        if t == 'R':
            print(f"[{ts}] R: {record['from_id']}<->{record['to_id']}: "
                  f"{record['distance_cm']:.1f}cm ({record['rssi']:.1f}dBm)")
        elif t == 'N':
            print(f"[{ts}] N: {record['from_id']}->{record['to_id']}: "
                  f"{record['distance_cm']:.1f}cm (hello={record['hello_count']}, "
                  f"{record['range_pct']}%)")
        elif t == 'H':
            print(f"[{ts}] H: node {record['from_id']}, "
                  f"{record['neighbor_count']} neighbors, "
                  f"frame {record['frame_num']}")


# =============================================================================
# TCP SERVER - Streams data to PC clients
# =============================================================================

class TCPServer:
    """TCP server for streaming data to PC clients"""
    
    def __init__(self, port: int, data_store: MeshDataStore):
        self.port = port
        self.data_store = data_store
        self.running = False
        self.clients: List[socket.socket] = []
        self.clients_lock = threading.Lock()
        
    def start(self):
        self.running = True
        
        # Start accept thread
        self.accept_thread = threading.Thread(target=self._accept_loop, daemon=True)
        self.accept_thread.start()
        
        # Start broadcast thread
        self.broadcast_thread = threading.Thread(target=self._broadcast_loop, daemon=True)
        self.broadcast_thread.start()
        
    def stop(self):
        self.running = False
        with self.clients_lock:
            for client in self.clients:
                try:
                    client.close()
                except:
                    pass
                    
    def _accept_loop(self):
        """Accept new client connections"""
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind(('0.0.0.0', self.port))
        server.listen(5)
        server.settimeout(1.0)
        
        print(f"[TCP] Listening on port {self.port}")
        
        while self.running:
            try:
                client, addr = server.accept()
                print(f"[TCP] Client connected: {addr}")
                
                with self.clients_lock:
                    self.clients.append(client)
                    
                # Send current state to new client
                try:
                    state = self.data_store.get_current_state()
                    msg = json.dumps({'type': 'state', 'data': state}) + '\n'
                    client.send(msg.encode())
                except:
                    pass
                    
            except socket.timeout:
                continue
            except Exception as e:
                print(f"[TCP] Accept error: {e}")
                
        server.close()
        
    def _broadcast_loop(self):
        """Broadcast new data to all clients"""
        last_count = 0
        
        while self.running:
            time.sleep(0.1)  # 10Hz update rate
            
            # Get new records
            records = self.data_store.get_recent_records(100)
            if len(records) <= last_count:
                continue
                
            new_records = records[last_count:]
            last_count = len(records)
            
            if not new_records:
                continue
                
            # Prepare message
            for record in new_records:
                try:
                    # Convert to JSON-safe format
                    safe_record = {k: v for k, v in record.items() 
                                  if k != 'raw' and not callable(v)}
                    msg = json.dumps({'type': 'record', 'data': safe_record}) + '\n'
                    msg_bytes = msg.encode()
                    
                    # Send to all clients
                    with self.clients_lock:
                        dead_clients = []
                        for client in self.clients:
                            try:
                                client.send(msg_bytes)
                            except:
                                dead_clients.append(client)
                                
                        # Remove dead clients
                        for client in dead_clients:
                            self.clients.remove(client)
                            print("[TCP] Client disconnected")
                            
                except Exception as e:
                    pass


# =============================================================================
# UDP BROADCAST - For discovery
# =============================================================================

class UDPBroadcaster:
    """Broadcasts presence on UDP for auto-discovery"""
    
    def __init__(self, port: int, tcp_port: int):
        self.port = port
        self.tcp_port = tcp_port
        self.running = False
        
    def start(self):
        self.running = True
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()
        
    def stop(self):
        self.running = False
        
    def _loop(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        
        hostname = socket.gethostname()
        
        while self.running:
            try:
                msg = json.dumps({
                    'service': 'uwb-mesh-logger',
                    'hostname': hostname,
                    'tcp_port': self.tcp_port
                })
                sock.sendto(msg.encode(), ('<broadcast>', self.port))
            except:
                pass
            time.sleep(5)
            
        sock.close()


# =============================================================================
# MAIN
# =============================================================================

def find_serial_port():
    """Auto-detect ESP32 serial port"""
    for port in serial.tools.list_ports.comports():
        if any(x in port.description.lower() for x in ['cp210', 'ch340', 'ftdi', 'usb', 'acm']):
            return port.device
        if 'ttyUSB' in port.device or 'ttyACM' in port.device:
            return port.device
    return None


def main():
    parser = argparse.ArgumentParser(description='UWB Mesh Data Logger (Raspberry Pi)')
    parser.add_argument('--serial', type=str, help='Serial port (e.g., /dev/ttyUSB0)')
    parser.add_argument('--baudrate', type=int, default=DEFAULT_BAUDRATE, help='Serial baudrate')
    parser.add_argument('--port', type=int, default=DEFAULT_TCP_PORT, help='TCP port for streaming')
    parser.add_argument('--log', type=str, help='Log file path (e.g., mesh_data.csv)')
    parser.add_argument('--log-dir', type=str, default=DEFAULT_LOG_DIR, help='Log directory')
    parser.add_argument('--auto', action='store_true', help='Auto-detect serial port')
    parser.add_argument('--broadcast', action='store_true', help='Enable UDP broadcast for discovery')
    
    args = parser.parse_args()
    
    print("="*60)
    print("  UWB MESH DATA LOGGER - Raspberry Pi")
    print("="*60)
    
    # Find serial port
    serial_port = args.serial
    if args.auto or not serial_port:
        serial_port = find_serial_port()
        if serial_port:
            print(f"[AUTO] Found serial port: {serial_port}")
        else:
            print("[ERROR] No serial port found!")
            print("Specify with --serial /dev/ttyUSB0")
            return
            
    # Setup log file
    log_file = args.log
    if not log_file:
        os.makedirs(args.log_dir, exist_ok=True)
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        log_file = os.path.join(args.log_dir, f'mesh_{timestamp}.csv')
        
    print(f"[LOG] Writing to: {log_file}")
    print(f"[SERIAL] Port: {serial_port} @ {args.baudrate} baud")
    print(f"[TCP] Streaming on port: {args.port}")
    print()
    
    # Create data store
    data_store = MeshDataStore(log_file=log_file)
    
    # Start serial reader
    serial_reader = SerialReader(serial_port, args.baudrate, data_store)
    serial_reader.start()
    
    # Start TCP server
    tcp_server = TCPServer(args.port, data_store)
    tcp_server.start()
    
    # Optional UDP broadcast
    udp_broadcaster = None
    if args.broadcast:
        udp_broadcaster = UDPBroadcaster(args.port + 1, args.port)
        udp_broadcaster.start()
        print(f"[UDP] Broadcasting on port {args.port + 1}")
    
    print()
    print("Connect from PC with:")
    print(f"  python mesh_client.py --host <raspberry-pi-ip> --port {args.port}")
    print()
    print("Waiting for data...")
    print()
    
    # Status loop
    try:
        while True:
            time.sleep(30)
            state = data_store.get_current_state()
            stats = state['stats']
            
            print(f"\n[STATUS] {datetime.now().strftime('%H:%M:%S')}")
            print(f"  Packets: {stats['total_packets']}")
            print(f"  Active links: {stats['active_links']}")
            print(f"  Active nodes: {stats['active_nodes']}")
            print(f"  TCP clients: {len(tcp_server.clients)}")
            print(f"  Uptime: {stats['uptime']:.0f}s")
            
    except KeyboardInterrupt:
        print("\nShutting down...")
        
    serial_reader.stop()
    tcp_server.stop()
    if udp_broadcaster:
        udp_broadcaster.stop()
    data_store.close()
    
    print("Done.")


if __name__ == '__main__':
    main()

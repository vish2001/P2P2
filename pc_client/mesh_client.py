#!/usr/bin/env python3
"""
UWB MESH CLIENT - PC Visualizer

Connects to Raspberry Pi mesh logger and displays real-time visualization.

Usage:
    python mesh_client.py --host 192.168.1.100 --port 5000
    python mesh_client.py --host raspberrypi.local
    
    # Auto-discover (if logger has --broadcast enabled)
    python mesh_client.py --discover

Requirements:
    pip install numpy matplotlib
"""

import argparse
import socket
import threading
import time
import json
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from collections import defaultdict
from datetime import datetime

# =============================================================================
# CONFIGURATION
# =============================================================================

MAX_ACCEPT_DISTANCE_CM = 3000
DATA_TIMEOUT_SEC = 15
UPDATE_INTERVAL_MS = 500
MIN_NODES_FOR_MDS = 3
MIN_CROSS_LINKS_FOR_MERGE = 3

# =============================================================================
# DISTANCE MATRIX
# =============================================================================

class DistanceMatrix:
    """Stores and manages distance measurements"""
    
    def __init__(self, max_distance=MAX_ACCEPT_DISTANCE_CM):
        self.distances = {}  # (from, to) -> {distance, rssi, time}
        self.max_distance = max_distance
        self.lock = threading.Lock()
        
    def update(self, from_id: int, to_id: int, distance: float, rssi: float = None):
        """Update a distance measurement"""
        if distance > self.max_distance:
            return
            
        key = (min(from_id, to_id), max(from_id, to_id))
        
        with self.lock:
            self.distances[key] = {
                'distance': distance,
                'rssi': rssi,
                'time': time.time()
            }
            
    def get_matrix(self, max_age=DATA_TIMEOUT_SEC):
        """Get distance matrix for MDS"""
        with self.lock:
            now = time.time()
            
            # Find active nodes
            nodes = set()
            for (a, b), data in self.distances.items():
                if now - data['time'] < max_age:
                    nodes.add(a)
                    nodes.add(b)
                    
            node_list = sorted(nodes)
            n = len(node_list)
            
            if n < 2:
                return None, node_list
                
            # Build matrix
            D = np.full((n, n), np.nan)
            np.fill_diagonal(D, 0)
            
            for (a, b), data in self.distances.items():
                if now - data['time'] > max_age:
                    continue
                if a in nodes and b in nodes:
                    i = node_list.index(a)
                    j = node_list.index(b)
                    D[i, j] = data['distance']
                    D[j, i] = data['distance']
                    
            return D, node_list
            
    def get_stats(self):
        """Get statistics"""
        with self.lock:
            now = time.time()
            active = sum(1 for d in self.distances.values() 
                        if now - d['time'] < DATA_TIMEOUT_SEC)
            return {
                'total': len(self.distances),
                'active': active
            }


# =============================================================================
# MDS LOCALIZER
# =============================================================================

def classical_mds(D):
    """Classical MDS algorithm"""
    n = D.shape[0]
    if n < 2:
        return None
        
    # Fill NaN with mean distance
    D_filled = D.copy()
    valid_mask = ~np.isnan(D) & (D > 0)
    if np.sum(valid_mask) > 0:
        mean_dist = np.mean(D[valid_mask])
    else:
        mean_dist = 100
        
    D_filled = np.where(np.isnan(D_filled), mean_dist, D_filled)
    np.fill_diagonal(D_filled, 0)
    
    # Square distances
    D2 = D_filled ** 2
    
    # Double centering
    H = np.eye(n) - np.ones((n, n)) / n
    B = -0.5 * H @ D2 @ H
    
    # Eigendecomposition
    eigenvalues, eigenvectors = np.linalg.eigh(B)
    
    # Sort by eigenvalue (descending)
    idx = np.argsort(eigenvalues)[::-1]
    eigenvalues = eigenvalues[idx]
    eigenvectors = eigenvectors[:, idx]
    
    # Take top 2 dimensions
    pos = np.zeros((n, 2))
    for i in range(2):
        if eigenvalues[i] > 0:
            pos[:, i] = eigenvectors[:, i] * np.sqrt(eigenvalues[i])
            
    return pos


def find_articulation_points(D):
    """Find articulation points (hinge nodes)"""
    n = D.shape[0]
    if n < 3:
        return []
        
    # Build adjacency
    adj = [[] for _ in range(n)]
    for i in range(n):
        for j in range(i+1, n):
            if not np.isnan(D[i, j]) and D[i, j] > 0:
                adj[i].append(j)
                adj[j].append(i)
                
    # Tarjan's algorithm
    visited = [False] * n
    disc = [0] * n
    low = [0] * n
    parent = [-1] * n
    ap = []
    time = [0]
    
    def dfs(u):
        children = 0
        visited[u] = True
        disc[u] = low[u] = time[0]
        time[0] += 1
        
        for v in adj[u]:
            if not visited[v]:
                children += 1
                parent[v] = u
                dfs(v)
                low[u] = min(low[u], low[v])
                
                if parent[u] == -1 and children > 1:
                    if u not in ap:
                        ap.append(u)
                elif parent[u] != -1 and low[v] >= disc[u]:
                    if u not in ap:
                        ap.append(u)
            elif v != parent[u]:
                low[u] = min(low[u], disc[v])
                
    for i in range(n):
        if not visited[i]:
            dfs(i)
            
    return ap


def find_biconnected_components(D):
    """Find biconnected components"""
    n = D.shape[0]
    if n < 2:
        return [list(range(n))]
        
    # Build adjacency
    adj = [[] for _ in range(n)]
    for i in range(n):
        for j in range(i+1, n):
            if not np.isnan(D[i, j]) and D[i, j] > 0:
                adj[i].append(j)
                adj[j].append(i)
                
    # DFS to find components
    visited = [False] * n
    components = []
    
    def dfs(u, component):
        visited[u] = True
        component.append(u)
        for v in adj[u]:
            if not visited[v]:
                dfs(v, component)
                
    for i in range(n):
        if not visited[i]:
            component = []
            dfs(i, component)
            components.append(component)
            
    return components


def count_cross_links(D, cluster1, cluster2):
    """Count links between two clusters"""
    count = 0
    for i in cluster1:
        for j in cluster2:
            if not np.isnan(D[i, j]) and D[i, j] > 0:
                count += 1
    return count


def should_separate_clusters(D, node_list):
    """Determine if clusters should be separated"""
    ap = find_articulation_points(D)
    if not ap:
        return False, None
        
    components = find_biconnected_components(D)
    if len(components) <= 1:
        return False, None
        
    # Check cross-links between all pairs
    for i, c1 in enumerate(components):
        for j, c2 in enumerate(components):
            if i < j:
                links = count_cross_links(D, c1, c2)
                if links <= MIN_CROSS_LINKS_FOR_MERGE:
                    return True, components
                    
    return False, None


# =============================================================================
# TCP CLIENT
# =============================================================================

class TCPClient:
    """Connects to Raspberry Pi logger via TCP"""
    
    def __init__(self, host: str, port: int, distance_matrix: DistanceMatrix):
        self.host = host
        self.port = port
        self.dm = distance_matrix
        self.running = False
        self.connected = False
        self.packet_count = 0
        
    def start(self):
        self.running = True
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()
        
    def stop(self):
        self.running = False
        
    def _loop(self):
        buffer = ""
        
        while self.running:
            try:
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                sock.settimeout(5.0)
                print(f"[TCP] Connecting to {self.host}:{self.port}...")
                sock.connect((self.host, self.port))
                sock.settimeout(1.0)
                self.connected = True
                print(f"[TCP] Connected!")
                
                while self.running:
                    try:
                        data = sock.recv(4096)
                        if not data:
                            raise ConnectionError("Server closed")
                            
                        buffer += data.decode('utf-8', errors='ignore')
                        
                        while '\n' in buffer:
                            line, buffer = buffer.split('\n', 1)
                            if line.strip():
                                self._process(line)
                                
                    except socket.timeout:
                        continue
                        
            except Exception as e:
                print(f"[TCP] Connection error: {e}")
                self.connected = False
                time.sleep(2)
                
    def _process(self, line: str):
        """Process a JSON message from server"""
        self.packet_count += 1
        
        try:
            msg = json.loads(line)
            
            if msg['type'] == 'record':
                record = msg['data']
                
                if record.get('type') == 'R':
                    self.dm.update(
                        record['from_id'],
                        record['to_id'],
                        record['distance_cm'],
                        record.get('rssi')
                    )
                    
                elif record.get('type') == 'N':
                    self.dm.update(
                        record['from_id'],
                        record['to_id'],
                        record['distance_cm'],
                        record.get('rssi')
                    )
                    
            elif msg['type'] == 'state':
                # Initial state from server
                state = msg['data']
                for key, data in state.get('distances', {}).items():
                    a, b = map(int, key.split('-'))
                    self.dm.update(a, b, data['distance_cm'], data.get('rssi'))
                    
        except Exception as e:
            pass


# =============================================================================
# VISUALIZER
# =============================================================================

class MeshVisualizer:
    """Real-time mesh visualization"""
    
    def __init__(self, distance_matrix: DistanceMatrix, client: TCPClient):
        self.dm = distance_matrix
        self.client = client
        
        # Setup figure
        self.fig, (self.ax_mesh, self.ax_info) = plt.subplots(1, 2, figsize=(14, 7))
        self.fig.suptitle('UWB Mesh Visualizer (Connected to Raspberry Pi)', fontsize=12)
        
        # Previous positions for stability
        self.prev_positions = None
        self.prev_node_list = None
        
    def update(self, frame):
        """Update visualization"""
        self.ax_mesh.clear()
        self.ax_info.clear()
        
        # Get distance matrix
        D, node_list = self.dm.get_matrix()
        
        if D is None or len(node_list) < MIN_NODES_FOR_MDS:
            self._draw_waiting(len(node_list) if node_list else 0)
            self._draw_info(None, node_list, D)
            return
            
        # Check for cluster separation
        separate, clusters = should_separate_clusters(D, node_list)
        
        if separate and clusters:
            self._draw_separated(D, node_list, clusters)
        else:
            self._draw_unified(D, node_list)
            
        self._draw_info(D, node_list, D)
        
    def _draw_waiting(self, node_count):
        """Draw waiting message"""
        self.ax_mesh.set_xlim(-1, 1)
        self.ax_mesh.set_ylim(-1, 1)
        self.ax_mesh.set_aspect('equal')
        
        status = "Connected" if self.client.connected else "Connecting..."
        self.ax_mesh.text(0, 0.1, status, ha='center', va='center', fontsize=16)
        self.ax_mesh.text(0, -0.1, f"Nodes: {node_count} (need {MIN_NODES_FOR_MDS})", 
                         ha='center', va='center', fontsize=12, color='gray')
        self.ax_mesh.set_title('Waiting for data...')
        
    def _draw_unified(self, D, node_list):
        """Draw unified mesh view"""
        n = len(node_list)
        
        # Run MDS
        positions = classical_mds(D)
        if positions is None:
            return
            
        # Procrustes alignment to previous
        if self.prev_positions is not None and self.prev_node_list == node_list:
            positions = self._procrustes_align(self.prev_positions, positions)
            
        self.prev_positions = positions.copy()
        self.prev_node_list = node_list.copy()
        
        # Draw edges
        for i in range(n):
            for j in range(i+1, n):
                if not np.isnan(D[i, j]) and D[i, j] > 0:
                    self.ax_mesh.plot(
                        [positions[i, 0], positions[j, 0]],
                        [positions[i, 1], positions[j, 1]],
                        'b-', alpha=0.3, linewidth=1
                    )
                    # Distance label
                    mx = (positions[i, 0] + positions[j, 0]) / 2
                    my = (positions[i, 1] + positions[j, 1]) / 2
                    self.ax_mesh.text(mx, my, f'{D[i,j]:.0f}', fontsize=7, 
                                     ha='center', va='center', color='gray')
                    
        # Draw nodes
        self.ax_mesh.scatter(positions[:, 0], positions[:, 1], 
                            s=400, c='orange', edgecolors='black', linewidth=2, zorder=5)
        
        for i, nid in enumerate(node_list):
            self.ax_mesh.annotate(str(nid), (positions[i, 0], positions[i, 1]),
                                 ha='center', va='center', fontweight='bold', fontsize=10)
                                 
        self.ax_mesh.set_aspect('equal')
        self.ax_mesh.set_title('Mesh Topology (MDS)')
        self.ax_mesh.grid(True, alpha=0.3)
        
    def _draw_separated(self, D, node_list, clusters):
        """Draw separated clusters"""
        colors = ['#ff9f43', '#00d9ff', '#2ed573', '#ff6b81', '#a55eea']
        
        n_clusters = len(clusters)
        cols = int(np.ceil(np.sqrt(n_clusters)))
        rows = int(np.ceil(n_clusters / cols))
        
        self.ax_mesh.set_xlim(-0.1, cols + 0.1)
        self.ax_mesh.set_ylim(-0.1, rows + 0.1)
        
        for c_idx, cluster in enumerate(clusters):
            col = c_idx % cols
            row = rows - 1 - c_idx // cols
            
            # Extract sub-matrix
            sub_D = D[np.ix_(cluster, cluster)]
            
            # Run MDS on cluster
            if len(cluster) >= 2:
                positions = classical_mds(sub_D)
                if positions is None:
                    positions = np.random.rand(len(cluster), 2) * 0.5
            else:
                positions = np.array([[0.5, 0.5]])
                
            # Scale to box
            if len(cluster) > 1:
                positions = positions - positions.mean(axis=0)
                scale = 0.3 / (np.max(np.abs(positions)) + 1e-6)
                positions = positions * scale
                
            positions[:, 0] += col + 0.5
            positions[:, 1] += row + 0.5
            
            # Draw box
            box_color = colors[c_idx % len(colors)]
            rect = plt.Rectangle((col + 0.05, row + 0.05), 0.9, 0.9,
                                 fill=False, edgecolor=box_color, 
                                 linestyle='--', linewidth=2)
            self.ax_mesh.add_patch(rect)
            
            # Draw edges
            for i in range(len(cluster)):
                for j in range(i+1, len(cluster)):
                    if not np.isnan(sub_D[i, j]) and sub_D[i, j] > 0:
                        self.ax_mesh.plot(
                            [positions[i, 0], positions[j, 0]],
                            [positions[i, 1], positions[j, 1]],
                            color=box_color, alpha=0.4, linewidth=1
                        )
                        
            # Draw nodes
            self.ax_mesh.scatter(positions[:, 0], positions[:, 1],
                               s=300, c=box_color, edgecolors='black', 
                               linewidth=2, zorder=5)
                               
            for i, idx in enumerate(cluster):
                self.ax_mesh.annotate(str(node_list[idx]), 
                                     (positions[i, 0], positions[i, 1]),
                                     ha='center', va='center', 
                                     fontweight='bold', fontsize=9)
                                     
            # Cluster label
            self.ax_mesh.text(col + 0.5, row + 0.92, f'Cluster {c_idx+1}',
                             ha='center', va='top', fontsize=9, color=box_color)
                             
        # Draw "?" between clusters
        self.ax_mesh.text(cols/2, rows/2, '?', ha='center', va='center',
                         fontsize=30, color='red', alpha=0.5)
                         
        self.ax_mesh.set_aspect('equal')
        self.ax_mesh.set_title('Separated Clusters (Weak Connectivity)')
        self.ax_mesh.axis('off')
        
    def _draw_info(self, D, node_list, matrix):
        """Draw info panel"""
        info = []
        info.append(f"CONNECTION")
        info.append(f"───────────────")
        info.append(f"Status: {'✓ Connected' if self.client.connected else '✗ Disconnected'}")
        info.append(f"Server: {self.client.host}:{self.client.port}")
        info.append(f"Packets: {self.client.packet_count}")
        info.append(f"")
        
        stats = self.dm.get_stats()
        info.append(f"MESH")
        info.append(f"───────────────")
        info.append(f"Nodes: {len(node_list) if node_list else 0}")
        info.append(f"Active links: {stats['active']}")
        info.append(f"")
        
        if matrix is not None and len(node_list) > 0:
            valid = matrix[~np.isnan(matrix) & (matrix > 0)]
            if len(valid) > 0:
                info.append(f"DISTANCES")
                info.append(f"───────────────")
                info.append(f"Min: {np.min(valid):.0f} cm")
                info.append(f"Max: {np.max(valid):.0f} cm")
                info.append(f"Avg: {np.mean(valid):.0f} cm")
                
        self.ax_info.axis('off')
        text = '\n'.join(info)
        self.ax_info.text(0.05, 0.95, text, transform=self.ax_info.transAxes,
                         fontsize=11, va='top', ha='left', family='monospace')
                         
    def _procrustes_align(self, target, source):
        """Align source to target using Procrustes"""
        # Center both
        target_c = target - target.mean(axis=0)
        source_c = source - source.mean(axis=0)
        
        # SVD for rotation
        H = source_c.T @ target_c
        U, S, Vt = np.linalg.svd(H)
        R = Vt.T @ U.T
        
        # Apply rotation
        aligned = source_c @ R.T
        
        # Translate back
        aligned += target.mean(axis=0)
        
        return aligned
        
    def run(self):
        """Start visualization"""
        ani = FuncAnimation(self.fig, self.update, interval=UPDATE_INTERVAL_MS,
                           cache_frame_data=False)
        plt.tight_layout()
        plt.show()


# =============================================================================
# AUTO-DISCOVERY
# =============================================================================

def discover_server(timeout=5):
    """Discover mesh logger via UDP broadcast"""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(('', 5001))  # Default discovery port
    sock.settimeout(timeout)
    
    print(f"[DISCOVER] Listening for mesh logger broadcast...")
    
    try:
        data, addr = sock.recvfrom(1024)
        msg = json.loads(data.decode())
        if msg.get('service') == 'uwb-mesh-logger':
            host = addr[0]
            port = msg.get('tcp_port', 5000)
            print(f"[DISCOVER] Found: {host}:{port} ({msg.get('hostname', 'unknown')})")
            return host, port
    except socket.timeout:
        print("[DISCOVER] No server found")
    except Exception as e:
        print(f"[DISCOVER] Error: {e}")
        
    sock.close()
    return None, None


# =============================================================================
# MAIN
# =============================================================================

def main():
    parser = argparse.ArgumentParser(description='UWB Mesh Client (PC Visualizer)')
    parser.add_argument('--host', type=str, help='Raspberry Pi hostname or IP')
    parser.add_argument('--port', type=int, default=5000, help='TCP port')
    parser.add_argument('--discover', action='store_true', help='Auto-discover server')
    parser.add_argument('--max-distance', type=float, default=MAX_ACCEPT_DISTANCE_CM,
                       help='Max accept distance (cm)')
    
    args = parser.parse_args()
    
    print("="*60)
    print("  UWB MESH CLIENT - Real-time Visualizer")
    print("="*60)
    
    host = args.host
    port = args.port
    
    # Auto-discover if requested
    if args.discover or not host:
        discovered_host, discovered_port = discover_server()
        if discovered_host:
            host = discovered_host
            port = discovered_port
        elif not host:
            print("\nNo server found and no --host specified!")
            print("Usage: python mesh_client.py --host <raspberry-pi-ip>")
            return
            
    print(f"Server: {host}:{port}")
    print(f"Max distance: {args.max_distance} cm")
    print()
    
    # Create components
    dm = DistanceMatrix(max_distance=args.max_distance)
    client = TCPClient(host, port, dm)
    viz = MeshVisualizer(dm, client)
    
    # Start client
    client.start()
    
    # Run visualizer
    try:
        viz.run()
    except KeyboardInterrupt:
        print("\nShutdown")
    finally:
        client.stop()


if __name__ == '__main__':
    main()

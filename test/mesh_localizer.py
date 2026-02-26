#!/usr/bin/env python3
"""
UWB P2P MESH LOCALIZER - Pure Relative Positioning

ALL NODES ARE EQUAL PEERS. No anchors, no fixed positions.
Uses MDS (Multidimensional Scaling) with Procrustes alignment to:
- Compute relative positions from pairwise distances
- Maintain consistent orientation frame-to-frame (prevents flips/jumps)

The coordinate frame is arbitrary and may rotate/translate, but:
- Relative positions between nodes are accurate
- Shape is preserved across frames

Usage:
    python mesh_localizer.py

Requirements:
    pip install numpy matplotlib scipy
"""

import socket
import threading
import time
import numpy as np
from collections import defaultdict
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from scipy.linalg import eigh, orthogonal_procrustes
import warnings
warnings.filterwarnings('ignore')

# =============================================================================
# CONFIGURATION
# =============================================================================
UDP_PORT = 5000
DISTANCE_EXPIRY_SEC = 15.0
UPDATE_INTERVAL_MS = 500
MAX_NODES = 50

# Distance filtering
MAX_ACCEPT_DISTANCE_CM = 3000.0  # Ignore measurements beyond 30m

# MDS parameters
MDS_MIN_NODES = 3
PROCRUSTES_ENABLED = True  # Align new frame to previous to prevent flips

# Cluster separation
MIN_CROSS_LINKS_FOR_MERGE = 3  # Minimum links between clusters to merge them
SEPARATE_WEAK_CLUSTERS = True  # Show clusters separately if weakly connected

# =============================================================================
# DATA STRUCTURES
# =============================================================================
class DistanceMatrix:
    """Thread-safe storage for pairwise distances"""
    
    def __init__(self):
        self.distances = {}  # (min_id, max_id) -> (distance, timestamp, rssi)
        self.nodes = set()
        self.lock = threading.Lock()
        
    def update(self, from_id, to_id, distance_cm, rssi=None):
        """Add or update a distance measurement"""
        with self.lock:
            # Basic validation
            if distance_cm <= 0 or distance_cm > 50000:
                return
            
            # Reject measurements beyond max accept distance
            if distance_cm > MAX_ACCEPT_DISTANCE_CM:
                return
            
            key = (min(from_id, to_id), max(from_id, to_id))
            
            # EMA smoothing if existing
            if key in self.distances:
                old_dist = self.distances[key][0]
                distance_cm = 0.3 * distance_cm + 0.7 * old_dist
            
            self.distances[key] = (distance_cm, time.time(), rssi)
            self.nodes.add(from_id)
            self.nodes.add(to_id)
    
    def get_matrix(self):
        """Return distance matrix D and node list"""
        with self.lock:
            now = time.time()
            
            # Prune expired
            expired = [k for k, v in self.distances.items() 
                      if now - v[1] > DISTANCE_EXPIRY_SEC]
            for k in expired:
                del self.distances[k]
            
            # Update node set
            active_nodes = set()
            for (a, b) in self.distances.keys():
                active_nodes.add(a)
                active_nodes.add(b)
            self.nodes = active_nodes
            
            node_list = sorted(self.nodes)
            n = len(node_list)
            
            if n < 2:
                return None, node_list
            
            # Build matrix
            D = np.full((n, n), np.nan)
            np.fill_diagonal(D, 0)
            
            for i, ni in enumerate(node_list):
                for j, nj in enumerate(node_list):
                    if i >= j:
                        continue
                    key = (ni, nj)  # already sorted
                    if key in self.distances:
                        dist = self.distances[key][0]
                        D[i, j] = dist
                        D[j, i] = dist
            
            return D, node_list
    
    def get_all_distances(self):
        """Get list of all distances for display"""
        with self.lock:
            now = time.time()
            result = []
            for (a, b), (dist, ts, rssi) in self.distances.items():
                age = now - ts
                if age < DISTANCE_EXPIRY_SEC:
                    result.append({
                        'from': a, 'to': b,
                        'distance': dist,
                        'rssi': rssi,
                        'age': age
                    })
            return result
    
    def get_connectivity(self):
        """Get adjacency list"""
        with self.lock:
            graph = defaultdict(set)
            for (a, b) in self.distances.keys():
                graph[a].add(b)
                graph[b].add(a)
            return dict(graph)


# =============================================================================
# MDS LOCALIZATION
# =============================================================================
def classical_mds(D, n_dims=2):
    """
    Classical Multidimensional Scaling.
    
    Given distance matrix D, compute positions that preserve distances.
    Result is in an arbitrary coordinate frame (can rotate/translate).
    
    Args:
        D: NxN distance matrix. NaN = missing.
        n_dims: Output dimensionality (2 for 2D)
    
    Returns:
        Nx2 position array, or None if failed
    """
    n = D.shape[0]
    if n < MDS_MIN_NODES:
        if n == 2:
            # Special case: 2 nodes on a line
            d = np.nanmean([D[0,1], D[1,0]])
            if np.isnan(d): d = 100
            return np.array([[0, 0], [d, 0]])
        return None
    
    # Fill missing values with mean distance (simple imputation)
    D_filled = D.copy()
    valid_mask = ~np.isnan(D) & (D > 0)
    if np.sum(valid_mask) == 0:
        return None
    
    mean_dist = np.nanmean(D[valid_mask])
    D_filled[np.isnan(D_filled)] = mean_dist
    D_filled[D_filled <= 0] = mean_dist
    
    # Symmetrize
    D_sym = (D_filled + D_filled.T) / 2
    np.fill_diagonal(D_sym, 0)
    
    # Classical MDS algorithm
    # 1. Square distances
    D2 = D_sym ** 2
    
    # 2. Double centering: B = -0.5 * H * D^2 * H
    H = np.eye(n) - np.ones((n, n)) / n
    B = -0.5 * H @ D2 @ H
    
    # 3. Eigendecomposition
    try:
        eigenvalues, eigenvectors = eigh(B)
        
        # Sort descending
        idx = np.argsort(eigenvalues)[::-1]
        eigenvalues = eigenvalues[idx]
        eigenvectors = eigenvectors[:, idx]
        
        # Take top n_dims positive eigenvalues
        eigenvalues = np.maximum(eigenvalues[:n_dims], 1e-10)
        
        # Positions = V * sqrt(Lambda)
        positions = eigenvectors[:, :n_dims] * np.sqrt(eigenvalues)
        
        return positions
        
    except Exception as e:
        print(f"[MDS] Error: {e}")
        return None


def procrustes_align(target, source):
    """
    Align source positions to target using Procrustes analysis.
    
    Finds rotation, reflection, and translation to best match target.
    This prevents frame-to-frame flips and rotations.
    
    Args:
        target: Nx2 reference positions (previous frame)
        source: Nx2 positions to align (current frame)
    
    Returns:
        Nx2 aligned positions
    """
    if target is None or source is None:
        return source
    
    if target.shape != source.shape:
        return source
    
    n = target.shape[0]
    if n < 2:
        return source
    
    # Center both
    target_centered = target - target.mean(axis=0)
    source_centered = source - source.mean(axis=0)
    
    # Find optimal rotation/reflection
    try:
        R, scale = orthogonal_procrustes(source_centered, target_centered)
        
        # Apply transformation
        aligned = source_centered @ R
        
        # Translate to match target centroid
        aligned = aligned + target.mean(axis=0)
        
        return aligned
        
    except Exception as e:
        print(f"[PROCRUSTES] Error: {e}")
        return source


# =============================================================================
# CLUSTER ANALYSIS
# =============================================================================
def find_connected_components(D, node_list):
    """Find connected components in the distance graph"""
    n = len(node_list)
    if n == 0:
        return []
    
    visited = [False] * n
    components = []
    
    def dfs(node, component):
        visited[node] = True
        component.append(node)
        for i in range(n):
            if not visited[i] and not np.isnan(D[node, i]) and D[node, i] > 0:
                dfs(i, component)
    
    for i in range(n):
        if not visited[i]:
            component = []
            dfs(i, component)
            components.append(component)
    
    return components


def find_articulation_points(D):
    """Find articulation points (nodes whose removal disconnects the graph)"""
    n = D.shape[0]
    if n < 3:
        return []
    
    points = []
    
    for remove in range(n):
        # Check connectivity without this node
        visited = [False] * n
        visited[remove] = True
        
        # Find start node
        start = 0 if remove != 0 else 1
        if start >= n:
            continue
        
        # DFS from start
        count = 0
        stack = [start]
        while stack:
            node = stack.pop()
            if visited[node]:
                continue
            visited[node] = True
            count += 1
            for i in range(n):
                if not visited[i] and not np.isnan(D[node, i]) and D[node, i] > 0:
                    stack.append(i)
        
        # If we couldn't reach all nodes, this is an articulation point
        if count < n - 1:
            points.append(remove)
    
    return points


def find_biconnected_components(D, node_list):
    """
    Find clusters that would be separated if articulation points are removed.
    Returns list of clusters, where each cluster is a list of indices into node_list.
    """
    n = len(node_list)
    if n < 2:
        return [[i for i in range(n)]] if n > 0 else []
    
    articulation = find_articulation_points(D)
    
    # If no articulation points, entire graph is one component
    if not articulation:
        components = find_connected_components(D, node_list)
        return components if components else [[i for i in range(n)]]
    
    # Find clusters by removing articulation points
    visited = [False] * n
    clusters = []
    
    # Mark articulation points as visited (they're boundaries)
    for ap in articulation:
        visited[ap] = True
    
    def dfs(node, cluster):
        visited[node] = True
        cluster.append(node)
        for i in range(n):
            if not visited[i] and not np.isnan(D[node, i]) and D[node, i] > 0:
                dfs(i, cluster)
    
    # Find each cluster
    for i in range(n):
        if not visited[i]:
            cluster = []
            dfs(i, cluster)
            if cluster:
                # Add articulation points that connect to this cluster
                for ap in articulation:
                    for node in cluster:
                        if not np.isnan(D[ap, node]) and D[ap, node] > 0:
                            if ap not in cluster:
                                cluster.append(ap)
                            break
                clusters.append(cluster)
    
    # If only articulation points remain, they form their own cluster
    if not clusters and articulation:
        clusters.append(articulation[:])
    
    return clusters


def count_cross_links(D, cluster1, cluster2):
    """Count number of links between two clusters"""
    count = 0
    for i in cluster1:
        for j in cluster2:
            if not np.isnan(D[i, j]) and D[i, j] > 0:
                count += 1
    return count


def should_separate_clusters(D, node_list):
    """
    Determine if clusters should be shown separately due to weak connectivity.
    Returns (should_separate, clusters)
    """
    if not SEPARATE_WEAK_CLUSTERS:
        return False, None
    
    n = len(node_list)
    if n < 4:
        return False, None
    
    articulation = find_articulation_points(D)
    if not articulation:
        return False, None
    
    clusters = find_biconnected_components(D, node_list)
    
    if len(clusters) < 2:
        return False, None
    
    # Check if any pair of clusters has insufficient cross-links
    for i in range(len(clusters)):
        for j in range(i + 1, len(clusters)):
            links = count_cross_links(D, clusters[i], clusters[j])
            if links < MIN_CROSS_LINKS_FOR_MERGE:
                return True, clusters
    
    return False, clusters


# =============================================================================
# MESH LOCALIZER
# =============================================================================
class MeshLocalizer:
    """Maintains mesh positions over time"""
    
    def __init__(self):
        self.previous_positions = None
        self.previous_node_list = None
        self.position_history = {}
        self.cluster_positions = {}  # Per-cluster position history
        
    def compute_positions(self, D, node_list):
        """
        Compute node positions from distance matrix.
        
        Uses MDS for relative positioning, then Procrustes to align
        with previous frame (preventing flips/jumps).
        
        If clusters are weakly connected (hinge nodes), they are
        computed separately to avoid incorrect merging.
        
        Returns:
            dict with:
                'positions': Nx2 array (None for separated clusters)
                'separated': bool
                'clusters': list of cluster info (if separated)
                'node_list': list of node IDs
        """
        result = {
            'positions': None,
            'separated': False,
            'clusters': [],
            'node_list': node_list
        }
        
        if D is None or len(node_list) < 2:
            return result
        
        # Check if we should separate clusters
        separate, cluster_indices = should_separate_clusters(D, node_list)
        
        if separate and cluster_indices:
            # Compute MDS for each cluster separately
            result['separated'] = True
            cluster_colors = ['#ff9f43', '#00d9ff', '#2ed573', '#ff6b81', '#a55eea']
            
            for c_idx, cluster in enumerate(cluster_indices):
                if len(cluster) < 2:
                    continue
                
                # Extract sub-matrix for this cluster
                sub_D = D[np.ix_(cluster, cluster)]
                
                # Run MDS on cluster
                positions = classical_mds(sub_D)
                if positions is None:
                    continue
                
                # Map node indices back to IDs
                cluster_node_ids = [node_list[i] for i in cluster]
                
                result['clusters'].append({
                    'indices': cluster,
                    'node_ids': cluster_node_ids,
                    'positions': positions,
                    'color': cluster_colors[c_idx % len(cluster_colors)]
                })
            
            return result
        
        # Normal unified MDS
        positions = classical_mds(D)
        if positions is None:
            return result
        
        # Procrustes alignment to previous frame
        if PROCRUSTES_ENABLED and self.previous_positions is not None:
            # Find common nodes between frames
            if self.previous_node_list == node_list:
                # Same nodes - direct alignment
                positions = procrustes_align(self.previous_positions, positions)
            else:
                # Node set changed - align based on common nodes
                common = set(node_list) & set(self.previous_node_list or [])
                if len(common) >= 3:
                    # Build correspondence
                    prev_idx = [self.previous_node_list.index(n) for n in common 
                               if n in self.previous_node_list]
                    curr_idx = [node_list.index(n) for n in common 
                               if n in node_list]
                    
                    if len(prev_idx) >= 3 and len(curr_idx) >= 3:
                        prev_common = self.previous_positions[prev_idx[:len(curr_idx)]]
                        curr_common = positions[curr_idx[:len(prev_idx)]]
                        
                        # Align common subset
                        try:
                            curr_centered = curr_common - curr_common.mean(axis=0)
                            prev_centered = prev_common - prev_common.mean(axis=0)
                            R, _ = orthogonal_procrustes(curr_centered, prev_centered)
                            
                            # Apply to all positions
                            positions_centered = positions - positions.mean(axis=0)
                            positions = positions_centered @ R + prev_common.mean(axis=0)
                        except:
                            pass
        
        # Store for next frame
        self.previous_positions = positions.copy()
        self.previous_node_list = node_list.copy()
        
        # Update history
        now = time.time()
        for i, nid in enumerate(node_list):
            if nid not in self.position_history:
                self.position_history[nid] = []
            self.position_history[nid].append((now, positions[i].copy()))
            # Keep last 60 seconds
            self.position_history[nid] = [
                (t, p) for t, p in self.position_history[nid] 
                if now - t < 60
            ]
        
        result['positions'] = positions
        return result


# =============================================================================
# UDP RECEIVER
# =============================================================================
class DataReceiver:
    """Base class for data receivers"""
    
    def __init__(self, distance_matrix):
        self.dm = distance_matrix
        self.running = False
        self.packet_count = 0
        self.ranging_count = 0
        self.neighbor_count = 0
        
    def process_line(self, data):
        """Process a line of data (multiple formats supported)"""
        self.packet_count += 1
        try:
            parts = data.strip().split(',')
            if not parts:
                return
            
            # New format from RPi: unix_ts,datetime,base_ts,station,type,data...
            # e.g., "1705678234.123,2025-01-19 14:30:34.123,12345,1,R,1,2,150.5,-65,98765"
            if len(parts) >= 7 and parts[4] in ['R', 'N', 'H']:
                msg_type = parts[4]
                if msg_type == 'R' and len(parts) >= 10:
                    from_id = int(parts[5])
                    to_id = int(parts[6])
                    dist = float(parts[7])
                    rssi = float(parts[8]) if len(parts) > 8 else None
                    self.dm.update(from_id, to_id, dist, rssi)
                    self.ranging_count += 1
                    print(f"[R] {from_id}<->{to_id}: {dist:.1f}cm")
                elif msg_type == 'N' and len(parts) >= 11:
                    node_id = int(parts[5])
                    neighbor_id = int(parts[6])
                    dist = float(parts[9])
                    rssi = float(parts[10]) if len(parts) > 10 else None
                    self.dm.update(node_id, neighbor_id, dist, rssi)
                    self.neighbor_count += 1
                    print(f"[N] {node_id}->{neighbor_id}: {dist:.1f}cm")
                return
            
            # CSV format from RPi: unix_ts,datetime,type,data...
            # e.g., "1705678234.123,2025-01-19 14:30:34.123,R,1,2,150.5,-65,98765"
            if len(parts) >= 5 and parts[2] in ['R', 'N', 'H']:
                msg_type = parts[2]
                if msg_type == 'R' and len(parts) >= 7:
                    from_id = int(parts[3])
                    to_id = int(parts[4])
                    dist = float(parts[5])
                    rssi = float(parts[6]) if len(parts) > 6 else None
                    self.dm.update(from_id, to_id, dist, rssi)
                    self.ranging_count += 1
                    print(f"[R] {from_id}<->{to_id}: {dist:.1f}cm")
                elif msg_type == 'N' and len(parts) >= 8:
                    node_id = int(parts[3])
                    neighbor_id = int(parts[4])
                    dist = float(parts[7])
                    rssi = float(parts[8]) if len(parts) > 8 else None
                    self.dm.update(node_id, neighbor_id, dist, rssi)
                    self.neighbor_count += 1
                    print(f"[N] {node_id}->{neighbor_id}: {dist:.1f}cm")
                return
            
            # Direct format: R,<from>,<to>,<dist>,<rssi>
            if parts[0] == 'R' and len(parts) >= 4:
                from_id = int(parts[1])
                to_id = int(parts[2])
                dist = float(parts[3])
                rssi = float(parts[4]) if len(parts) > 4 else None
                self.dm.update(from_id, to_id, dist, rssi)
                self.ranging_count += 1
                print(f"[R] {from_id}<->{to_id}: {dist:.1f}cm")
                
            # Direct neighbor: N,<node>,<neighbor>,<hello>,<pct>,<dist>,<rssi>
            elif parts[0] == 'N' and len(parts) >= 6:
                node_id = int(parts[1])
                neighbor_id = int(parts[2])
                dist = float(parts[5])
                rssi = float(parts[6]) if len(parts) > 6 else None
                self.dm.update(node_id, neighbor_id, dist, rssi)
                self.neighbor_count += 1
                print(f"[N] {node_id}->{neighbor_id}: {dist:.1f}cm")
                
        except Exception as e:
            pass


class UDPReceiver(DataReceiver):
    """Receives data via UDP"""
    
    def __init__(self, port, distance_matrix):
        super().__init__(distance_matrix)
        self.port = port
        
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
                self.process_line(data.decode('utf-8'))
            except socket.timeout:
                continue
            except Exception as e:
                print(f"[UDP] Error: {e}")
        sock.close()


class TCPReceiver(DataReceiver):
    """Connects to Raspberry Pi base station server via TCP"""
    
    def __init__(self, host, port, distance_matrix):
        super().__init__(distance_matrix)
        self.host = host
        self.port = port
        self.socket = None
        
    def start(self):
        self.running = True
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()
        print(f"[TCP] Connecting to {self.host}:{self.port}...")
        
    def stop(self):
        self.running = False
        if self.socket:
            self.socket.close()
        
    def _loop(self):
        buffer = ""
        
        while self.running:
            try:
                if self.socket is None:
                    self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                    self.socket.settimeout(5.0)
                    self.socket.connect((self.host, self.port))
                    self.socket.settimeout(1.0)
                    print(f"[TCP] Connected to {self.host}:{self.port}")
                
                data = self.socket.recv(4096)
                if not data:
                    raise ConnectionError("Server closed connection")
                
                buffer += data.decode('utf-8', errors='ignore')
                
                # Process complete lines
                while '\n' in buffer:
                    line, buffer = buffer.split('\n', 1)
                    if line.strip():
                        self.process_line(line)
                        
            except socket.timeout:
                continue
            except Exception as e:
                print(f"[TCP] Connection error: {e}")
                if self.socket:
                    self.socket.close()
                    self.socket = None
                time.sleep(2)  # Wait before reconnect


# =============================================================================
# VISUALIZATION
# =============================================================================
class MeshVisualizer:
    def __init__(self, distance_matrix, localizer):
        self.dm = distance_matrix
        self.localizer = localizer
        
        self.fig, (self.ax_mesh, self.ax_info) = plt.subplots(1, 2, figsize=(14, 7))
        self.fig.suptitle('UWB P2P Mesh - Relative Positioning (No Anchors)', fontsize=14)
        
        self.colors = plt.cm.tab20(np.linspace(0, 1, MAX_NODES))
        
    def update(self, frame):
        self.ax_mesh.clear()
        self.ax_info.clear()
        
        D, node_list = self.dm.get_matrix()
        
        if D is None or len(node_list) < 2:
            self.ax_mesh.text(0.5, 0.5, 
                f'Waiting for nodes...\n\nCurrent: {len(node_list) if node_list else 0}\nNeed: {MDS_MIN_NODES}+',
                ha='center', va='center', fontsize=14)
            self.ax_mesh.set_xlim(-1, 1)
            self.ax_mesh.set_ylim(-1, 1)
            return
        
        # Compute positions
        result = self.localizer.compute_positions(D, node_list)
        
        if result['separated']:
            self._draw_separated_clusters(result, node_list, D)
        elif result['positions'] is not None:
            self._draw_mesh(result['positions'], node_list, D)
        else:
            self.ax_mesh.text(0.5, 0.5, 'MDS failed - need more connections',
                ha='center', va='center', fontsize=14)
            return
        
        self._draw_info(node_list, D, result)
    
    def _draw_separated_clusters(self, result, node_list, D):
        """Draw clusters in separate boxes when weakly connected"""
        clusters = result['clusters']
        n_clusters = len(clusters)
        
        if n_clusters == 0:
            return
        
        # Calculate grid layout
        cols = int(np.ceil(np.sqrt(n_clusters)))
        rows = int(np.ceil(n_clusters / cols))
        
        # Get axis bounds
        self.ax_mesh.set_xlim(0, 100)
        self.ax_mesh.set_ylim(0, 100)
        
        margin = 5
        gap = 3
        box_w = (100 - 2*margin - gap*(cols-1)) / cols
        box_h = (100 - 2*margin - gap*(rows-1)) / rows
        
        for c_idx, cluster in enumerate(clusters):
            col = c_idx % cols
            row = c_idx // cols
            
            box_x = margin + col * (box_w + gap)
            box_y = margin + row * (box_h + gap)
            
            # Draw cluster box
            rect = plt.Rectangle((box_x, box_y), box_w, box_h,
                fill=False, edgecolor=cluster['color'], linestyle='--', linewidth=2)
            self.ax_mesh.add_patch(rect)
            
            # Cluster label
            self.ax_mesh.text(box_x + 2, box_y + box_h - 3,
                f"Cluster {c_idx+1}", fontsize=9, fontweight='bold',
                color=cluster['color'])
            self.ax_mesh.text(box_x + 2, box_y + box_h - 8,
                "(position unknown)", fontsize=7, color='gray', style='italic')
            
            # Scale positions to fit in box
            positions = cluster['positions']
            if positions is None or len(positions) < 1:
                continue
            
            # Normalize positions to box
            pos_min = positions.min(axis=0)
            pos_max = positions.max(axis=0)
            pos_range = pos_max - pos_min
            pos_range[pos_range == 0] = 1
            
            padding = 15
            scaled = (positions - pos_min) / pos_range * (min(box_w, box_h) - 2*padding)
            scaled = scaled + np.array([box_x + padding, box_y + padding])
            
            # Draw edges within cluster
            cluster_indices = cluster['indices']
            for i in range(len(cluster_indices)):
                for j in range(i+1, len(cluster_indices)):
                    di, dj = cluster_indices[i], cluster_indices[j]
                    if not np.isnan(D[di, dj]) and D[di, dj] > 0:
                        x = [scaled[i, 0], scaled[j, 0]]
                        y = [scaled[i, 1], scaled[j, 1]]
                        self.ax_mesh.plot(x, y, color='gray', alpha=0.4, linewidth=1)
            
            # Draw nodes
            for i, nid in enumerate(cluster['node_ids']):
                self.ax_mesh.scatter(scaled[i, 0], scaled[i, 1],
                    s=200, c=[cluster['color']], edgecolors='black', linewidths=1.5, zorder=5)
                self.ax_mesh.text(scaled[i, 0], scaled[i, 1], str(nid),
                    fontsize=9, ha='center', va='center', fontweight='bold', zorder=6)
        
        # Draw "?" between clusters
        self.ax_mesh.text(50, 50, '?', fontsize=40, ha='center', va='center',
            color='red', alpha=0.5, fontweight='bold')
        
        self.ax_mesh.set_title(f'Weakly Connected - {n_clusters} Separate Clusters')
        self.ax_mesh.set_aspect('equal')
        self.ax_mesh.axis('off')
        
        # Warning note
        self.ax_mesh.text(0.5, 0.02, 
            '⚠ Clusters shown separately due to weak connectivity (hinge nodes)\n'
            'Add more links between clusters to merge them',
            transform=self.ax_mesh.transAxes, fontsize=9,
            va='bottom', ha='center', color='red', style='italic')
        
    def _draw_mesh(self, positions, node_list, D):
        n = len(node_list)
        
        # Draw edges
        for i in range(n):
            for j in range(i+1, n):
                if not np.isnan(D[i,j]) and D[i,j] > 0:
                    x = [positions[i,0], positions[j,0]]
                    y = [positions[i,1], positions[j,1]]
                    
                    # Color by distance (blue=close, red=far)
                    dist_norm = min(D[i,j] / 3000, 1.0)
                    color = plt.cm.coolwarm(dist_norm)
                    
                    self.ax_mesh.plot(x, y, color=color, alpha=0.6, linewidth=1.5)
                    
                    # Label short edges
                    if D[i,j] < 2000:
                        mx, my = (x[0]+x[1])/2, (y[0]+y[1])/2
                        self.ax_mesh.text(mx, my, f'{D[i,j]:.0f}', fontsize=7,
                            ha='center', va='center',
                            bbox=dict(boxstyle='round', facecolor='white', alpha=0.7))
        
        # Draw position trails
        for nid in node_list:
            if nid in self.localizer.position_history:
                trail = self.localizer.position_history[nid]
                if len(trail) > 1:
                    pts = np.array([p for t, p in trail])
                    self.ax_mesh.plot(pts[:,0], pts[:,1], '-',
                        color=self.colors[nid % MAX_NODES], alpha=0.3, linewidth=1)
        
        # Draw nodes
        for i, nid in enumerate(node_list):
            color = self.colors[nid % MAX_NODES]
            self.ax_mesh.scatter(positions[i,0], positions[i,1],
                s=400, c=[color], edgecolors='black', linewidths=2, zorder=5)
            self.ax_mesh.text(positions[i,0], positions[i,1], str(nid),
                fontsize=11, ha='center', va='center', fontweight='bold', zorder=6)
        
        self.ax_mesh.set_xlabel('X (cm) - Relative')
        self.ax_mesh.set_ylabel('Y (cm) - Relative')
        self.ax_mesh.set_title(f'Mesh Topology ({n} nodes)')
        self.ax_mesh.set_aspect('equal')
        self.ax_mesh.grid(True, alpha=0.3)
        
        # Note about relative coordinates
        self.ax_mesh.text(0.02, 0.98, 'Coordinates are RELATIVE\n(no fixed reference)',
            transform=self.ax_mesh.transAxes, fontsize=8,
            va='top', ha='left', style='italic', alpha=0.7)
        
    def _draw_info(self, node_list, D, result=None):
        """Draw info panel"""
        info = []
        info.append(f"MESH STATUS")
        info.append(f"───────────────")
        info.append(f"Nodes: {len(node_list)}")
        
        # Connection stats
        valid = np.sum(~np.isnan(D) & (D > 0)) // 2
        max_conn = len(node_list) * (len(node_list) - 1) // 2
        info.append(f"Links: {valid}/{max_conn}")
        
        # Cluster status
        if result and result.get('separated'):
            n_clusters = len(result.get('clusters', []))
            info.append(f"")
            info.append(f"⚠ CLUSTER STATUS")
            info.append(f"───────────────")
            info.append(f"Separated: YES")
            info.append(f"Clusters: {n_clusters}")
            for i, cluster in enumerate(result.get('clusters', [])):
                node_ids = cluster.get('node_ids', [])
                info.append(f"  #{i+1}: {len(node_ids)} nodes")
            info.append(f"")
            info.append(f"Add more links between")
            info.append(f"clusters to merge them")
        else:
            # Check for hinge nodes
            articulation = find_articulation_points(D)
            if articulation:
                info.append(f"")
                info.append(f"HINGE NODES")
                info.append(f"───────────────")
                hinge_ids = [node_list[i] for i in articulation]
                info.append(f"Nodes: {hinge_ids}")
        
        # Distance stats
        valid_d = D[~np.isnan(D) & (D > 0)]
        if len(valid_d) > 0:
            info.append(f"")
            info.append(f"DISTANCES")
            info.append(f"───────────────")
            info.append(f"Min: {np.min(valid_d):.0f} cm")
            info.append(f"Max: {np.max(valid_d):.0f} cm")
            info.append(f"Avg: {np.mean(valid_d):.0f} cm")
        
        # Node connections
        info.append(f"")
        info.append(f"NODE LINKS")
        info.append(f"───────────────")
        for i, nid in enumerate(node_list):
            conn = np.sum(~np.isnan(D[i,:]) & (D[i,:] > 0))
            info.append(f"Node {nid}: {conn} links")
        
        # Distance table
        distances = self.dm.get_all_distances()
        if distances:
            info.append(f"")
            info.append(f"RECENT MEASUREMENTS")
            info.append(f"───────────────")
            distances.sort(key=lambda x: -x['distance'])
            for d in distances[:8]:
                info.append(f"{d['from']}<->{d['to']}: {d['distance']:.0f}cm ({d['age']:.1f}s)")
        
        self.ax_info.axis('off')
        text = '\n'.join(info)
        self.ax_info.text(0.05, 0.95, text, transform=self.ax_info.transAxes,
            fontsize=10, va='top', ha='left', family='monospace')
        
    def run(self):
        ani = FuncAnimation(self.fig, self.update, interval=UPDATE_INTERVAL_MS,
            cache_frame_data=False)
        plt.tight_layout()
        plt.show()


# =============================================================================
# MAIN
# =============================================================================
def main():
    import argparse
    
    parser = argparse.ArgumentParser(description='UWB P2P Mesh Localizer')
    parser.add_argument('--udp', type=int, default=UDP_PORT, help=f'UDP port to listen on (default: {UDP_PORT})')
    parser.add_argument('--tcp', type=str, help='TCP server to connect to (e.g., 192.168.1.100:5000)')
    parser.add_argument('--max-distance', type=float, default=MAX_ACCEPT_DISTANCE_CM, 
                       help=f'Max accept distance in cm (default: {MAX_ACCEPT_DISTANCE_CM})')
    
    args = parser.parse_args()
    
    # Update global config
    global MAX_ACCEPT_DISTANCE_CM
    MAX_ACCEPT_DISTANCE_CM = args.max_distance
    
    print("=" * 60)
    print("  UWB P2P MESH LOCALIZER")
    print("  Pure Relative Positioning - No Anchors")
    print("=" * 60)
    
    if args.tcp:
        print(f"Mode: TCP Client")
        print(f"Server: {args.tcp}")
    else:
        print(f"Mode: UDP Listener")
        print(f"UDP Port: {args.udp}")
    
    print(f"Max Accept Distance: {MAX_ACCEPT_DISTANCE_CM} cm")
    print(f"MDS Minimum Nodes: {MDS_MIN_NODES}")
    print(f"Procrustes Alignment: {'Enabled' if PROCRUSTES_ENABLED else 'Disabled'}")
    print(f"Cluster Separation: {'Enabled' if SEPARATE_WEAK_CLUSTERS else 'Disabled'}")
    print(f"  Min cross-links to merge: {MIN_CROSS_LINKS_FOR_MERGE}")
    print("")
    print("All coordinates are RELATIVE (arbitrary frame).")
    print("Weakly connected clusters shown separately.")
    print("=" * 60)
    
    dm = DistanceMatrix()
    localizer = MeshLocalizer()
    
    # Create receiver based on mode
    if args.tcp:
        # Parse host:port
        if ':' in args.tcp:
            host, port = args.tcp.rsplit(':', 1)
            port = int(port)
        else:
            host = args.tcp
            port = 5000
        receiver = TCPReceiver(host, port, dm)
    else:
        receiver = UDPReceiver(args.udp, dm)
    
    receiver.start()
    
    viz = MeshVisualizer(dm, localizer)
    
    try:
        viz.run()
    except KeyboardInterrupt:
        print("\nShutdown")
    finally:
        receiver.stop()


if __name__ == '__main__':
    main()

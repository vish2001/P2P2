# UWB Mesh System - Raspberry Pi Base Station

## Architecture

```
[Cow 1] ──┐
[Cow 2] ──┼── ESP-NOW ──> [ESP32 Receiver] ──Serial──> [Raspberry Pi] ──TCP──> [PC Visualizer]
[Cow 3] ──┘                    (on RPi)                     │
                                                            ▼
                                                     [CSV Log File]
```

## Hardware Setup

### On Raspberry Pi
1. Connect ESP32 base station receiver via USB
2. Find the serial port: `ls /dev/ttyUSB* /dev/ttyACM*`
3. Usually `/dev/ttyUSB0` or `/dev/ttyACM0`

### ESP32 Base Station Firmware
Flash the base station firmware from `base_station/` to the ESP32 connected to the RPi.

## Software Setup

### On Raspberry Pi

```bash
# Install dependencies
pip3 install pyserial

# Create log directory
mkdir -p ~/uwb_logs

# Run the logger
python3 mesh_logger.py --serial /dev/ttyUSB0 --port 5000

# Or with auto-detect
python3 mesh_logger.py --auto --port 5000
```

### On PC

```bash
# Install dependencies
pip install numpy matplotlib

# Connect to Raspberry Pi
python mesh_client.py --host <raspberry-pi-ip> --port 5000

# Example with hostname
python mesh_client.py --host raspberrypi.local --port 5000

# Or use mDNS/Bonjour
python mesh_client.py --host raspberrypi.local
```

## Data Flow

### 1. Mesh Nodes → ESP32 Receiver (ESP-NOW)
Each mesh node broadcasts:
- **R** (Ranging): `R,<from>,<to>,<distance_cm>,<rssi>,<millis>`
- **N** (Neighbor): `N,<node>,<neighbor>,<hello_count>,<range%>,<distance>,<rssi>`
- **H** (Heartbeat): `H,<node>,<frame>,<neighbor_count>,<uptime>`

### 2. ESP32 → Raspberry Pi (Serial)
Base station adds timestamp and outputs CSV:
```
12345,1,R,1,2,152.3,-58.5,1000
12400,1,N,1,2,15,85,152.3,-58.5
12450,1,H,1,500,3,120000
```

### 3. Raspberry Pi → Log File
Logger adds wall-clock timestamps:
```csv
unix_ts,wall_clock,type,station_id,from_id,to_id,distance_cm,rssi,node_ts,extra
1705678234.123,2025-01-19 14:30:34.123,R,1,1,2,152.3,-58.5,1000,
1705678234.200,2025-01-19 14:30:34.200,N,1,1,2,152.3,-58.5,,hello=15;range_pct=85
```

### 4. Raspberry Pi → PC (TCP)
Streams JSON records:
```json
{"type": "record", "data": {"type": "R", "from_id": 1, "to_id": 2, "distance_cm": 152.3, ...}}
```

## Commands

### Raspberry Pi Logger
```bash
# Basic usage
python3 mesh_logger.py --serial /dev/ttyUSB0

# With custom log file
python3 mesh_logger.py --serial /dev/ttyUSB0 --log /home/pi/mesh_data.csv

# With auto-discovery broadcast
python3 mesh_logger.py --auto --broadcast

# Full options
python3 mesh_logger.py --serial /dev/ttyUSB0 --port 5000 --log mesh.csv --broadcast
```

### PC Client
```bash
# Connect to specific host
python mesh_client.py --host 192.168.1.100 --port 5000

# Auto-discover (if logger has --broadcast)
python mesh_client.py --discover

# With custom max distance filter
python mesh_client.py --host raspberrypi.local --max-distance 2000
```

## Systemd Service (Auto-start on RPi)

Create `/etc/systemd/system/uwb-logger.service`:

```ini
[Unit]
Description=UWB Mesh Data Logger
After=network.target

[Service]
ExecStart=/usr/bin/python3 /home/pi/mesh_logger.py --auto --port 5000
WorkingDirectory=/home/pi
User=pi
Restart=always
RestartSec=5

[Install]
WantedBy=multi-user.target
```

Enable and start:
```bash
sudo systemctl enable uwb-logger
sudo systemctl start uwb-logger
sudo systemctl status uwb-logger
```

## Log File Analysis

The CSV log files can be analyzed with Python/pandas:

```python
import pandas as pd

# Load log
df = pd.read_csv('mesh_data.csv')

# Filter to ranging data
ranging = df[df['type'] == 'R']

# Get distance between nodes 1 and 2
dist_1_2 = ranging[(ranging['from_id'] == 1) & (ranging['to_id'] == 2)]

# Plot distance over time
import matplotlib.pyplot as plt
plt.plot(dist_1_2['unix_ts'], dist_1_2['distance_cm'])
plt.xlabel('Time')
plt.ylabel('Distance (cm)')
plt.show()
```

## Troubleshooting

### Serial port permission denied
```bash
sudo usermod -a -G dialout $USER
# Logout and login again
```

### ESP32 not detected
```bash
# Check USB devices
lsusb

# Check serial ports
ls -la /dev/ttyUSB* /dev/ttyACM*

# Check kernel messages
dmesg | tail -20
```

### Can't connect from PC
```bash
# Check firewall
sudo ufw allow 5000/tcp

# Check if logger is running
netstat -tlnp | grep 5000
```

### No data received
1. Check ESP32 is flashed with base station firmware
2. Check serial connection: `screen /dev/ttyUSB0 921600`
3. Verify mesh nodes are transmitting (check LEDs, serial output)

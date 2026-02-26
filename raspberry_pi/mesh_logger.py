#!/usr/bin/env python3
# =============================================================================
# UWB MESH LOGGER (Raspberry Pi) — per-node CSVs + wall-clock timestamps
# =============================================================================
# Supports two input formats:
#  1) Bridge CSV:
#       rx_ms,station_id,src_mac,R,...
#  2) Wrapped/debug stream:
#       [RX] From AA:BB:CC:DD:EE:FF (xx bytes): R,...
#
# Outputs:
#   logs/<RUN>/raw.csv
#   logs/<RUN>/node_<id>/R.csv, N.csv, H.csv, C.csv
#   logs/<RUN>/node_<id>/latest.json   (latest packets for real-time UI)
#
# Stop: Ctrl+C or kill -TERM <pid>
# =============================================================================

import csv
import json
import time
import signal
import argparse
from datetime import datetime
from pathlib import Path
import socket

try:
    import serial  # pip install pyserial
except ImportError:
    serial = None

STOP = False


def handle_stop(signum, frame):
    global STOP
    STOP = True


signal.signal(signal.SIGINT, handle_stop)
signal.signal(signal.SIGTERM, handle_stop)


def ensure_dir(p: Path):
    p.mkdir(parents=True, exist_ok=True)


def make_run_dir(root: str, run_id: str | None):
    ts = datetime.now().strftime("%Y-%m-%d_%H%M%S")
    run_name = run_id or f"RUN_{ts}"
    p = Path(root) / run_name
    ensure_dir(p)
    return p


def iso_now_local() -> str:
    # e.g. 2026-02-24T21:13:05.123+01:00
    return datetime.now().astimezone().isoformat(timespec="milliseconds")


def open_csv_writer(path: Path, header: list[str]):
    exists = path.exists()
    f = open(path, "a", newline="")
    w = csv.writer(f)
    if (not exists) or (path.stat().st_size == 0):
        w.writerow(header)
        f.flush()
    return f, w


def node_id_from_payload(msg_type: str, fields: list[str]) -> str | None:
    if not fields:
        return None
    return fields[0]


def headers_for_type(msg_type: str) -> list[str]:
    base = ["iso_time", "rx_ms", "station_id", "src_mac", "type"]
    if msg_type == "R":
        return base + ["from", "to", "dist_cm", "rssi_dbm", "node_ts", "extra..."]
    if msg_type == "H":
        return base + ["node", "frame", "neighbor_count", "uptime_ms", "extra..."]
    if msg_type == "N":
        return base + ["node", "nbr", "hellos", "range_pct", "dist_cm", "rssi_dbm",
                       "is_stale", "stale_ms", "extra..."]
    if msg_type == "C":
        return base + ["node", "behav", "conf", "ax", "ay", "az", "gx", "gy", "gz",
                       "node_ts", "range_age_ms", "nbr_count", "range_pct", "extra..."]
    return base + ["fields..."]


def parse_line(line: str):
    """
    Returns (rx_ms, station_id, src_mac, msg_type, fields_list) or None.
    """
    line = line.strip()
    if not line or line.startswith("#"):
        return None

    # Case 1: Bridge CSV line begins with digits
    if line[0].isdigit():
        parts = line.split(",")
        if len(parts) < 4:
            return None
        rx_ms, station_id, src_mac = parts[0], parts[1], parts[2]
        payload = [p.strip() for p in parts[3:]]
        if not payload:
            return None
        msg_type = payload[0]
        if msg_type not in ("R", "N", "H", "C"):
            return None
        fields = payload[1:]
        return rx_ms, station_id, src_mac, msg_type, fields

    # Case 2: Debug wrapper format
    if line.startswith("[RX] From "):
        try:
            # "[RX] From AA:BB:... (xx bytes): R,...."
            after_from = line.split("From ", 1)[1]
            src_mac = after_from.split(" ", 1)[0].strip()
            payload = line.split(": ", 1)[1].strip()
            payload = payload.replace("\x00", "").strip()
            p = [x.strip() for x in payload.split(",")]
            msg_type = p[0]
            if msg_type not in ("R", "N", "H", "C"):
                return None
            fields = p[1:]
            rx_ms = str(int(time.time() * 1000))  # Pi wall clock ms since epoch-ish
            station_id = "0"
            return rx_ms, station_id, src_mac, msg_type, fields
        except Exception:
            return None

    return None


class Writers:
    def __init__(self, run_dir: Path, flush_every_s: float = 1.0):
        self.run_dir = run_dir
        self.flush_every_s = flush_every_s
        self.last_flush = time.time()

        self.raw_f, self.raw_w = open_csv_writer(
            run_dir / "raw.csv",
            ["iso_time", "rx_ms", "station_id", "src_mac", "type", "fields"]
        )

        self.node_writers = {}   # (node_id, msg_type) -> (file, writer)
        self.latest_cache = {}   # node_id -> dict of latest per msg type

    def _maybe_flush(self):
        now = time.time()
        if now - self.last_flush < self.flush_every_s:
            return
        self.last_flush = now

        self.raw_f.flush()
        for f, _ in self.node_writers.values():
            f.flush()

    def write(self, rx_ms: str, station_id: str, src_mac: str, msg_type: str, fields: list[str]):
        iso_time = iso_now_local()

        # raw log
        self.raw_w.writerow([iso_time, rx_ms, station_id, src_mac, msg_type, ",".join(fields)])

        nid = node_id_from_payload(msg_type, fields)
        if nid is None:
            self._maybe_flush()
            return

        node_dir = self.run_dir / f"node_{nid}"
        ensure_dir(node_dir)

        key = (nid, msg_type)
        if key not in self.node_writers:
            hdr = headers_for_type(msg_type)
            f, w = open_csv_writer(node_dir / f"{msg_type}.csv", hdr)
            self.node_writers[key] = (f, w)

        f, w = self.node_writers[key]

        row = [iso_time, rx_ms, station_id, src_mac, msg_type] + fields
        w.writerow(row)

        # latest snapshot for real-time viewers
        self.latest_cache.setdefault(nid, {})
        self.latest_cache[nid][msg_type] = {
            "iso_time": iso_time,
            "rx_ms": rx_ms,
            "station_id": station_id,
            "src_mac": src_mac,
            "fields": fields
        }
        with open(node_dir / "latest.json", "w") as jf:
            json.dump(self.latest_cache[nid], jf, indent=2)

        self._maybe_flush()

    def close(self):
        try:
            self.raw_f.flush()
            self.raw_f.close()
        except Exception:
            pass
        for f, _ in self.node_writers.values():
            try:
                f.flush()
                f.close()
            except Exception:
                pass


def serial_lines(port: str, baud: int):
    if serial is None:
        raise RuntimeError("pyserial not installed. Run: pip install pyserial")
    ser = serial.Serial(port, baud, timeout=0.2)
    try:
        while not STOP:
            b = ser.readline()
            if not b:
                continue
            yield b.decode("utf-8", errors="ignore")
    finally:
        ser.close()


def udp_lines(bind_ip: str, bind_port: int):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((bind_ip, bind_port))
    sock.settimeout(0.5)
    try:
        while not STOP:
            try:
                data, _ = sock.recvfrom(4096)
            except socket.timeout:
                continue
            yield data.decode("utf-8", errors="ignore")
    finally:
        sock.close()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mode", choices=["serial", "udp"], default="serial")
    ap.add_argument("--port", default="/dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=921600)
    ap.add_argument("--bind_ip", default="0.0.0.0")
    ap.add_argument("--bind_port", type=int, default=5005)
    ap.add_argument("--log_root", default="logs")
    ap.add_argument("--run_id", default=None)
    ap.add_argument("--flush_every_s", type=float, default=1.0)
    args = ap.parse_args()

    run_dir = make_run_dir(args.log_root, args.run_id)
    print(f"[logger] writing to: {run_dir}")

    writers = Writers(run_dir, flush_every_s=args.flush_every_s)

    try:
        src = serial_lines(args.port, args.baud) if args.mode == "serial" else udp_lines(args.bind_ip, args.bind_port)
        for line in src:
            parsed = parse_line(line)
            if not parsed:
                continue
            rx_ms, station_id, src_mac, msg_type, fields = parsed
            writers.write(rx_ms, station_id, src_mac, msg_type, fields)
    finally:
        writers.close()
        print("[logger] stopped cleanly")


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""Pretend to be the ESP32: send GPS + thermal packets to the ground station.
Useful to test the map and hotspot ranking before the hardware is wired.
  python fake_telemetry.py            -> sends to 127.0.0.1:4210
  python fake_telemetry.py 4210 25    -> hotspot with delta-T of 25 C (P1)"""
import json
import math
import socket
import sys
import time

port = int(sys.argv[1]) if len(sys.argv) > 1 else 4210
peak_dt = float(sys.argv[2]) if len(sys.argv) > 2 else 18.0
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
lat0, lon0 = 17.4401, 78.3489          # change to your test site
t0 = time.time()
while True:
    t = time.time() - t0
    d = 2.0 * t                          # 2 m/s along the feeder
    lat = lat0 + d * math.cos(math.radians(60)) / 111320
    lon = lon0 + d * math.sin(math.radians(60)) / (111320 * math.cos(math.radians(lat0)))
    hot = 4 < (t % 20) < 9               # a hot joint passes under the drone every 20 s
    tmed = 31.0
    tmax = tmed + (peak_dt if hot else 0.6)
    pkt = {"lat": lat, "lon": lon, "alt": 520.0, "sats": 8,
           "tmax": round(tmax, 1), "tmed": tmed, "hx": 9, "hy": 4, "tw": 16, "th": 12}
    s.sendto(json.dumps(pkt).encode(), ("127.0.0.1", port))
    time.sleep(0.2)

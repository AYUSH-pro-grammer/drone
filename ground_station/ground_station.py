#!/usr/bin/env python3
"""
GridDrishti ground station
--------------------------
Live drone video in  ->  defect detection  ->  severity ranking (P1..P4)
->  GPS geo-tagging  ->  defects.csv + defect_map.html + snapshots.

Detectors
  * corrosion   : rust-colour segmentation (HSV + LAB), works with no training
  * vegetation  : excess-green index inside the "clearance zone" (centre band)
  * hotspot     : thermal delta-T from the ESP32 + MLX90640 telemetry packet
  * yolo        : optional, any Ultralytics model (--model best.pt), e.g. trained
                  on InsPLAD / CPLID for insulators, corrosion, broken parts

Video sources (--source)
  0 / 1               laptop webcam or USB FPV receiver / capture card
  http://IP:81/stream ESP32-CAM MJPEG stream
  http://IP:8080/video Android "IP Webcam" app (phone strapped to the drone)
  some_video.mp4      recorded flight

Telemetry: UDP JSON on --telem-port (default 4210) from esp32_telemetry.ino:
  {"lat":17.38,"lon":78.48,"alt":512.3,"sats":7,
   "tmax":61.2,"tmed":31.0,"hx":10,"hy":6,"tw":16,"th":12}

Keys in the live window:  q = quit & save   s = save now   p = pause
"""

import argparse
import csv
import json
import math
import os
import socket
import threading
import time
from dataclasses import dataclass, field
from datetime import datetime

import cv2
import numpy as np

# --------------------------------------------------------------------------
# Severity
# --------------------------------------------------------------------------
PRIORITY_COLORS = {  # BGR
    "P1": (77, 72, 229),     # red
    "P2": (36, 165, 245),    # amber
    "P3": (232, 155, 76),    # blue
    "P4": (144, 122, 107),   # grey
}
PRIORITY_HEX = {"P1": "#E5484D", "P2": "#F5A524", "P3": "#4C9BE8", "P4": "#6B7A90"}
PRIORITY_ACTION = {
    "P1": "Dispatch crew now",
    "P2": "Repair within days",
    "P3": "Next planned maintenance",
    "P4": "Monitor on next survey",
}


def priority_of(score):
    if score >= 75:
        return "P1"
    if score >= 50:
        return "P2"
    if score >= 25:
        return "P3"
    return "P4"


def score_corrosion(area_frac, conf=1.0):
    # 30 (a speck of rust) .. 80 (rust covering ~6%+ of the frame)
    return (30 + 50 * min(1.0, area_frac / 0.06)) * conf


def score_vegetation(zone_cov, conf=1.0):
    # 20 (a few leaves in the clearance zone) .. 80 (zone 40%+ covered)
    return (20 + 60 * min(1.0, zone_cov / 0.40)) * conf


def score_hotspot(dt):
    # NETA-style delta-T bands vs. similar components:
    # 1-3 C possible deficiency, 4-15 C probable, >15 C major -> act now
    if dt < 1:
        return 0
    if dt < 4:
        return 10 + 5 * dt                    # 15..30  -> P4/P3
    if dt <= 15:
        return 50 + 2 * (dt - 4)              # 50..72  -> P2
    return min(100, 80 + (dt - 15))           # 80..100 -> P1


# YOLO class name -> (defect type, base score). Edit to match your model.
YOLO_CLASS_MAP = {
    "broken": ("insulator_broken", 90),
    "damaged": ("insulator_broken", 90),
    "flashover": ("insulator_flashover", 80),
    "corrosion": ("corrosion", 60),
    "rust": ("corrosion", 60),
    "nest": ("bird_nest", 55),
    "bird_nest": ("bird_nest", 55),
    "vegetation": ("vegetation", 50),
    "tree": ("vegetation", 50),
}


# --------------------------------------------------------------------------
# Detectors
# --------------------------------------------------------------------------
@dataclass
class Detection:
    dtype: str
    box: tuple          # x, y, w, h in frame pixels
    score: float
    info: str = ""
    dt: float = None    # thermal delta-T if any


def detect_corrosion(frame, min_area_frac=0.002):
    """Rust = orange/brown, fairly saturated, not too bright.
    Skin is also orange-ish but less saturated: S threshold keeps most of it out."""
    h, w = frame.shape[:2]
    blur = cv2.GaussianBlur(frame, (5, 5), 0)
    hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
    lab = cv2.cvtColor(blur, cv2.COLOR_BGR2LAB)
    m_hsv = cv2.inRange(hsv, (3, 110, 40), (22, 255, 215))
    # LAB: a* (red-green) and b* (yellow-blue) both clearly positive for rust
    a = lab[:, :, 1].astype(np.int16) - 128
    b = lab[:, :, 2].astype(np.int16) - 128
    m_lab = ((a > 12) & (b > 18)).astype(np.uint8) * 255
    mask = cv2.bitwise_and(m_hsv, m_lab)
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k, iterations=2)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    dets = []
    for c in contours:
        area = cv2.contourArea(c)
        frac = area / float(h * w)
        if frac < min_area_frac:
            continue
        x, y, bw, bh = cv2.boundingRect(c)
        fill = area / float(bw * bh)       # blotchy rust fills less of its box than a solid object
        conf = 0.85 if fill > 0.85 else 1.0
        s = score_corrosion(frac, conf)
        dets.append(Detection("corrosion", (x, y, bw, bh), s, f"{frac*100:.1f}% of view"))
    return dets, mask


def detect_vegetation(frame, zone, min_cov=0.06):
    """Excess-green index (2G - R - B) inside the clearance zone (x0,y0,x1,y1 fractions)."""
    h, w = frame.shape[:2]
    x0, y0, x1, y1 = int(zone[0] * w), int(zone[1] * h), int(zone[2] * w), int(zone[3] * h)
    roi = frame[y0:y1, x0:x1].astype(np.int16)
    B, G, R = roi[:, :, 0], roi[:, :, 1], roi[:, :, 2]
    exg = 2 * G - R - B
    mask = ((exg > 25) & (G > 50)).astype(np.uint8) * 255
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k)
    cov = cv2.countNonZero(mask) / float(mask.size)
    if cov < min_cov:
        return [], cov
    pts = cv2.findNonZero(mask)
    x, y, bw, bh = cv2.boundingRect(pts)
    s = score_vegetation(cov)
    return [Detection("vegetation", (x + x0, y + y0, bw, bh), s, f"{cov*100:.0f}% of clearance zone")], cov


def detect_hotspot(telem, frame_shape, min_dt=1.0):
    """Turn the MLX90640 summary in the telemetry packet into a detection.
    Box position is approximate: thermal (55 deg) and camera FOVs differ."""
    if not telem or "tmax" not in telem or "tmed" not in telem:
        return []
    if time.time() - telem.get("_rx", 0) > 2.0:   # stale packet
        return []
    dt = float(telem["tmax"]) - float(telem["tmed"])
    if dt < min_dt:
        return []
    h, w = frame_shape[:2]
    tw, th = telem.get("tw", 16), telem.get("th", 12)
    cx = (telem.get("hx", tw / 2) + 0.5) / tw * w
    cy = (telem.get("hy", th / 2) + 0.5) / th * h
    bw, bh = w / tw * 2, h / th * 2
    box = (int(cx - bw / 2), int(cy - bh / 2), int(bw), int(bh))
    return [Detection("hotspot", box, score_hotspot(dt),
                      f"{telem['tmax']:.1f}C, dT {dt:.1f}C", dt=round(dt, 1))]


class YoloDetector:
    def __init__(self, path, conf=0.35):
        from ultralytics import YOLO   # pip install ultralytics
        self.model = YOLO(path)
        self.conf = conf

    def __call__(self, frame):
        dets = []
        res = self.model(frame, conf=self.conf, verbose=False)[0]
        names = res.names
        for b in res.boxes:
            name = names[int(b.cls)].lower()
            conf = float(b.conf)
            dtype, base = None, 40
            for key, (t, s) in YOLO_CLASS_MAP.items():
                if key in name:
                    dtype, base = t, s
                    break
            if dtype is None:
                continue          # normal asset (e.g. "insulator") - not a defect
            x1, y1, x2, y2 = map(int, b.xyxy[0])
            dets.append(Detection(dtype, (x1, y1, x2 - x1, y2 - y1), base * (0.6 + 0.4 * conf),
                                  f"{name} {conf:.2f}"))
        return dets


# --------------------------------------------------------------------------
# Telemetry (GPS + thermal) over UDP
# --------------------------------------------------------------------------
class Telemetry(threading.Thread):
    def __init__(self, port):
        super().__init__(daemon=True)
        self.data = {}
        self.lock = threading.Lock()
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("0.0.0.0", port))
        self.sock.settimeout(0.5)

    def run(self):
        while True:
            try:
                pkt, _ = self.sock.recvfrom(4096)
                d = json.loads(pkt.decode("utf-8", "ignore"))
                d["_rx"] = time.time()
                with self.lock:
                    self.data = d      # replace, so a lost GPS fix doesn't leave stale coords
            except socket.timeout:
                pass
            except (ValueError, UnicodeDecodeError):
                pass

    def get(self):
        with self.lock:
            return dict(self.data)


class SimGPS:
    """Fake flight along a straight feeder (for demos without a GPS module)."""

    def __init__(self, lat, lon, heading_deg=60, speed_mps=2.0):
        self.lat0, self.lon0 = lat, lon
        self.hd = math.radians(heading_deg)
        self.v = speed_mps
        self.t0 = time.time()

    def get(self):
        d = self.v * (time.time() - self.t0)
        dn, de = d * math.cos(self.hd), d * math.sin(self.hd)
        lat = self.lat0 + dn / 111320.0
        lon = self.lon0 + de / (111320.0 * math.cos(math.radians(self.lat0)))
        return {"lat": lat, "lon": lon, "alt": 15.0, "sats": 0, "_sim": True, "_rx": time.time()}


def haversine_m(a_lat, a_lon, b_lat, b_lon):
    r = 6371000.0
    p1, p2 = math.radians(a_lat), math.radians(b_lat)
    dp, dl = p2 - p1, math.radians(b_lon - a_lon)
    x = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2 * r * math.asin(math.sqrt(x))


# --------------------------------------------------------------------------
# Defect register: merges repeated sightings, keeps the worst one
# --------------------------------------------------------------------------
@dataclass
class Defect:
    id: int
    dtype: str
    score: float
    lat: float
    lon: float
    first_seen: float
    last_seen: float
    hits: int = 1
    info: str = ""
    dt: float = None
    snapshot: str = ""
    last_lat: float = None
    last_lon: float = None

    @property
    def priority(self):
        return priority_of(self.score)


@dataclass
class Register:
    out_dir: str
    merge_m: float = 6.0
    merge_s: float = 4.0
    min_hits: int = 3
    defects: list = field(default_factory=list)
    _next: int = 1

    def add(self, det, gps, frame):
        now = time.time()
        lat, lon = (gps.get("lat"), gps.get("lon")) if gps else (None, None)
        if lat is not None and abs(lat) < 1e-6 and abs(lon) < 1e-6:
            lat = lon = None     # GPS without a fix reports 0,0
        for d in self.defects:
            if d.dtype != det.dtype:
                continue
            # same defect if we are still tracking it (seen < 1 s ago), or we are
            # back within merge_m of where it was last seen
            if (now - d.last_seen) <= 1.0:
                same = True
            elif lat is not None and d.last_lat is not None:
                same = haversine_m(lat, lon, d.last_lat, d.last_lon) <= self.merge_m
            else:
                same = (now - d.last_seen) <= self.merge_s
            if same:
                d.hits += 1
                d.last_seen = now
                if lat is not None:
                    d.last_lat, d.last_lon = lat, lon
                if det.score > d.score:
                    d.score, d.info, d.dt = det.score, det.info, det.dt
                    if lat is not None:
                        d.lat, d.lon = lat, lon      # map pin at the worst sighting
                    self._snap(d, det, frame)
                elif d.hits == self.min_hits and not d.snapshot:
                    self._snap(d, det, frame)
                return d
        d = Defect(self._next, det.dtype, det.score, lat, lon, now, now, 1, det.info, det.dt,
                   last_lat=lat, last_lon=lon)
        self._next += 1
        self.defects.append(d)
        return d

    def _snap(self, d, det, frame):
        x, y, w, h = det.box
        img = frame.copy()
        cv2.rectangle(img, (x, y), (x + w, y + h), PRIORITY_COLORS[priority_of(det.score)], 3)
        cv2.putText(img, f"#{d.id} {d.dtype} {priority_of(det.score)}", (x, max(20, y - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, PRIORITY_COLORS[priority_of(det.score)], 2)
        os.makedirs(os.path.join(self.out_dir, "snapshots"), exist_ok=True)
        rel = os.path.join("snapshots", f"defect_{d.id:04d}.jpg")
        cv2.imwrite(os.path.join(self.out_dir, rel), img)
        d.snapshot = rel

    def confirmed(self):
        c = [d for d in self.defects if d.hits >= self.min_hits]
        return sorted(c, key=lambda d: -d.score)

    # ---------------- outputs ----------------
    def save(self):
        os.makedirs(self.out_dir, exist_ok=True)
        rows = self.confirmed()
        with open(os.path.join(self.out_dir, "defects.csv"), "w", newline="") as f:
            wr = csv.writer(f)
            wr.writerow(["rank", "id", "priority", "action", "type", "score", "delta_T_C",
                         "lat", "lon", "detail", "sightings", "first_seen", "snapshot"])
            for i, d in enumerate(rows, 1):
                wr.writerow([i, d.id, d.priority, PRIORITY_ACTION[d.priority], d.dtype,
                             round(d.score, 1), d.dt if d.dt is not None else "",
                             f"{d.lat:.7f}" if d.lat is not None else "",
                             f"{d.lon:.7f}" if d.lon is not None else "",
                             d.info, d.hits,
                             datetime.fromtimestamp(d.first_seen).strftime("%Y-%m-%d %H:%M:%S"),
                             d.snapshot])
        self._save_map(rows)
        return len(rows)

    def _save_map(self, rows):
        pts = [{
            "id": d.id, "type": d.dtype, "p": d.priority, "score": round(d.score, 1),
            "lat": d.lat, "lon": d.lon, "info": d.info, "dt": d.dt,
            "action": PRIORITY_ACTION[d.priority], "color": PRIORITY_HEX[d.priority],
            "img": d.snapshot,
            "time": datetime.fromtimestamp(d.first_seen).strftime("%H:%M:%S"),
        } for d in rows]
        html = MAP_TEMPLATE.replace("__DATA__", json.dumps(pts)).replace(
            "__GENERATED__", datetime.now().strftime("%d %b %Y, %H:%M"))
        with open(os.path.join(self.out_dir, "defect_map.html"), "w", encoding="utf-8") as f:
            f.write(html)


MAP_TEMPLATE = r"""<!DOCTYPE html>
<html><head><meta charset="utf-8"><title>GridDrishti – Defect Map</title>
<meta name="viewport" content="width=device-width, initial-scale=1">
<link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css">
<script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
<style>
 body{margin:0;font-family:Segoe UI,Inter,Arial,sans-serif;background:#0D1726;color:#F3F1EA;display:flex;height:100vh}
 #side{width:420px;overflow:auto;padding:18px;box-sizing:border-box}
 #map{flex:1}
 h1{font-size:22px;margin:0 0 4px} .sub{color:#9AA8BD;font-size:13px;margin-bottom:14px}
 .stats{display:flex;gap:8px;margin-bottom:14px}
 .stat{flex:1;border-radius:10px;padding:8px;text-align:center;font-weight:700}
 .stat span{display:block;font-size:22px}
 .item{background:#16233A;border-radius:10px;padding:10px 12px;margin-bottom:8px;cursor:pointer;border-left:6px solid}
 .item b{font-size:15px} .item div{color:#9AA8BD;font-size:13px;margin-top:3px}
 .pill{font-weight:700;padding:1px 8px;border-radius:99px;color:#0D1726;font-size:12px;margin-right:6px}
 .popup img{width:260px;border-radius:6px;margin-top:6px}
 .nogps{color:#F5A524;font-size:13px;margin:8px 0}
</style></head><body>
<div id="side"><h1>GridDrishti defect report</h1><div class="sub">Generated __GENERATED__ · ranked by severity</div>
<div class="stats" id="stats"></div><div id="list"></div></div>
<div id="map"></div>
<script>
const D = __DATA__;
const ACT={P1:"Dispatch crew now",P2:"Repair within days",P3:"Next planned maintenance",P4:"Monitor on next survey"};
const COL={P1:"#E5484D",P2:"#F5A524",P3:"#4C9BE8",P4:"#6B7A90"};
const map = L.map('map');
L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png',{maxZoom:20,attribution:'© OpenStreetMap'}).addTo(map);
const st=document.getElementById('stats');
["P1","P2","P3","P4"].forEach(p=>{const n=D.filter(d=>d.p===p).length;
 st.innerHTML+=`<div class="stat" style="background:${COL[p]};color:${p==='P1'||p==='P4'?'#fff':'#0D1726'}">${p}<span>${n}</span></div>`});
const list=document.getElementById('list'); const geo=[]; const mk={};
D.forEach((d,i)=>{
 const pop=`<div class="popup"><b>#${d.id} ${d.type}</b> – <b style="color:${d.color}">${d.p}</b> (score ${d.score})<br>${d.info}<br><i>${d.action}</i><br>Seen ${d.time}${d.img?`<br><img src="${d.img}">`:''}</div>`;
 if(d.lat!==null){const m=L.circleMarker([d.lat,d.lon],{radius:d.p==='P1'?12:9,color:'#fff',weight:2,fillColor:d.color,fillOpacity:.95}).addTo(map).bindPopup(pop);mk[d.id]=m;geo.push([d.lat,d.lon]);}
 list.innerHTML+=`<div class="item" style="border-color:${d.color}" onclick="go(${d.id})"><span class="pill" style="background:${d.color}">${d.p}</span><b>${i+1}. ${d.type.replace('_',' ')}</b><div>${d.info} · ${d.action}${d.lat===null?' · no GPS fix':''}</div></div>`;
});
function go(id){const m=mk[id]; if(m){map.setView(m.getLatLng(),19);m.openPopup();}}
if(geo.length){map.fitBounds(geo,{padding:[40,40],maxZoom:19});}else{map.setView([17.385,78.4867],12);
 list.insertAdjacentHTML('afterbegin','<div class="nogps">No GPS positions yet – connect the GPS module or run with --sim-gps.</div>');}
if(!D.length) list.innerHTML='<div class="sub">No confirmed defects.</div>';
</script></body></html>"""


# --------------------------------------------------------------------------
# Video reader that always hands back the newest frame (low latency)
# --------------------------------------------------------------------------
class LatestFrame(threading.Thread):
    def __init__(self, src, realtime=True):
        super().__init__(daemon=True)
        self.cap = cv2.VideoCapture(int(src) if str(src).isdigit() else src)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        if not self.cap.isOpened():
            raise SystemExit(f"Could not open video source: {src}")
        self.realtime = realtime
        self.frame, self.ok, self.seq = None, True, 0
        self.lock = threading.Lock()
        self.cond = threading.Condition(self.lock)
        fps = self.cap.get(cv2.CAP_PROP_FPS)
        self.file_delay = 1.0 / fps if (not realtime and fps and fps < 120) else 0

    def run(self):
        while self.ok:
            ok, f = self.cap.read()
            with self.cond:
                if not ok:
                    self.ok = False
                else:
                    self.frame, self.seq = f, self.seq + 1
                self.cond.notify_all()
            if self.file_delay:
                time.sleep(self.file_delay)

    def read(self, last_seq):
        with self.cond:
            self.cond.wait_for(lambda: self.seq != last_seq or not self.ok, timeout=2.0)
            return self.frame, self.seq, self.ok


# --------------------------------------------------------------------------
# Drawing
# --------------------------------------------------------------------------
def draw(frame, dets, zone, gps, telem, reg, fps, show_zone=True):
    h, w = frame.shape[:2]
    if show_zone:
        x0, y0, x1, y1 = int(zone[0] * w), int(zone[1] * h), int(zone[2] * w), int(zone[3] * h)
        cv2.rectangle(frame, (x0, y0), (x1, y1), (200, 200, 200), 1)
        cv2.putText(frame, "clearance zone", (x0 + 4, y0 + 16), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                    (200, 200, 200), 1)
    for d in dets:
        p = priority_of(d.score)
        c = PRIORITY_COLORS[p]
        x, y, bw, bh = d.box
        cv2.rectangle(frame, (x, y), (x + bw, y + bh), c, 2)
        label = f"{p} {d.dtype} {d.info}"
        (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
        ty = max(th + 6, y)
        cv2.rectangle(frame, (x, ty - th - 6), (x + tw + 6, ty), c, -1)
        cv2.putText(frame, label, (x + 3, ty - 4), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

    # HUD bar
    cv2.rectangle(frame, (0, 0), (w, 28), (38, 23, 13), -1)
    if gps and gps.get("lat") is not None:
        src = "SIM" if gps.get("_sim") else f"{gps.get('sats', '?')} sats"
        g = f"GPS {gps['lat']:.6f},{gps['lon']:.6f} ({src})"
    else:
        g = "GPS: no fix"
    t = ""
    if telem and "tmax" in telem and time.time() - telem.get("_rx", 0) < 2:
        t = f" | IR max {telem['tmax']:.1f}C med {telem['tmed']:.1f}C"
    conf = reg.confirmed()
    counts = " ".join(f"{p}:{sum(1 for d in conf if d.priority == p)}" for p in ("P1", "P2", "P3", "P4"))
    cv2.putText(frame, f"GridDrishti | {fps:4.1f} fps | {g}{t} | {counts}", (8, 19),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (234, 241, 243), 1)
    return frame


# --------------------------------------------------------------------------
# Main loop
# --------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(description="GridDrishti ground station")
    ap.add_argument("--source", default="0", help="0 | http://ESP32-IP:81/stream | video.mp4")
    ap.add_argument("--telem-port", type=int, default=4210, help="UDP port for ESP32 GPS/thermal (0 = off)")
    ap.add_argument("--sim-gps", default="", help="lat,lon to fake a GPS track, e.g. 17.4401,78.3489")
    ap.add_argument("--model", default="", help="optional YOLO .pt for insulators/corrosion")
    ap.add_argument("--zone", default="0.15,0.25,0.85,0.75",
                    help="clearance zone as x0,y0,x1,y1 fractions of the frame")
    ap.add_argument("--min-hits", type=int, default=3, help="sightings before a defect is confirmed")
    ap.add_argument("--width", type=int, default=960, help="resize frames to this width")
    ap.add_argument("--out", default="", help="output folder (default runs/<timestamp>)")
    ap.add_argument("--record", action="store_true", help="save annotated video (for your demo)")
    ap.add_argument("--no-gui", action="store_true", help="run without a window (testing)")
    ap.add_argument("--max-frames", type=int, default=0, help="stop after N frames (testing)")
    ap.add_argument("--debug-mask", action="store_true", help="show the rust mask window")
    args = ap.parse_args()

    out = args.out or os.path.join("runs", datetime.now().strftime("%Y%m%d_%H%M%S"))
    os.makedirs(out, exist_ok=True)
    zone = tuple(float(v) for v in args.zone.split(","))
    reg = Register(out, min_hits=args.min_hits)

    telem = None
    if args.telem_port:
        try:
            telem = Telemetry(args.telem_port)
            telem.start()
            print(f"[telemetry] listening on UDP {args.telem_port}")
        except OSError as e:
            print(f"[telemetry] disabled: {e}")
    sim = None
    if args.sim_gps:
        la, lo = (float(v) for v in args.sim_gps.split(","))
        sim = SimGPS(la, lo)
        print("[gps] simulated track starting at", la, lo)

    yolo = YoloDetector(args.model) if args.model else None
    is_file = os.path.isfile(args.source)
    cam = LatestFrame(args.source, realtime=not is_file)
    cam.start()

    writer = None
    seq, n, t_last, fps, last_save = 0, 0, time.time(), 0.0, time.time()
    paused = False
    print(f"[run] output -> {out}   keys: q quit, s save, p pause")
    try:
        while True:
            if not paused:
                frame, seq, ok = cam.read(seq)
                if not ok and frame is None:
                    break
                if not ok:
                    break
                if args.width and frame.shape[1] != args.width:
                    s = args.width / frame.shape[1]
                    frame = cv2.resize(frame, (args.width, int(frame.shape[0] * s)))

                t = telem.get() if telem else {}
                gps = t if (t.get("lat") is not None and time.time() - t.get("_rx", 0) < 3) else None
                if gps is None and sim:
                    gps = sim.get()

                dets, mask = detect_corrosion(frame)
                vd, _ = detect_vegetation(frame, zone)
                dets += vd
                dets += detect_hotspot(t, frame.shape)
                if yolo:
                    dets += yolo(frame)
                for d in dets:
                    reg.add(d, gps, frame)

                view = draw(frame.copy(), dets, zone, gps, t, reg, fps)
                n += 1
                now = time.time()
                if now - t_last >= 1.0:
                    fps = n / (now - t_last)
                    n, t_last = 0, now
                if args.record:
                    if writer is None:
                        writer = cv2.VideoWriter(os.path.join(out, "annotated.mp4"),
                                                 cv2.VideoWriter_fourcc(*"mp4v"), 15,
                                                 (view.shape[1], view.shape[0]))
                    writer.write(view)
                if now - last_save > 10:
                    reg.save()
                    last_save = now
                if args.max_frames and seq >= args.max_frames:
                    break

            if not args.no_gui:
                cv2.imshow("GridDrishti ground station", view)
                if args.debug_mask:
                    cv2.imshow("rust mask", mask)
                k = cv2.waitKey(1) & 0xFF
                if k == ord("q"):
                    break
                if k == ord("s"):
                    print(f"[save] {reg.save()} confirmed defects")
                if k == ord("p"):
                    paused = not paused
    except KeyboardInterrupt:
        pass
    finally:
        cam.ok = False
        if writer:
            writer.release()
        cnt = reg.save()
        if not args.no_gui:
            cv2.destroyAllWindows()
        print(f"[done] {cnt} confirmed defects -> {os.path.join(out, 'defects.csv')} and defect_map.html")


if __name__ == "__main__":
    main()

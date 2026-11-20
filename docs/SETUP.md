# GridDrishti – drone power-line inspection prototype

```
 Drone                                        Laptop (ground station)
 ┌──────────────────────────┐   Wi-Fi video   ┌───────────────────────────────┐
 │ ESP32-CAM ──────────────────────────────────▶ ground_station.py             │
 │ ESP32 + NEO-6M GPS        │   UDP 4210      │  rust / vegetation / thermal  │
 │       + MLX90640 thermal ─────────────────────▶ (+ optional YOLO)          │
 │ KK2.1.5  (flies today)    │                 │  severity P1–P4, GPS tag      │
 │ ESP32+MPU6050 FC (bench)  │                 │  → defects.csv, defect_map.html│
 └──────────────────────────┘                 └───────────────────────────────┘
```

| Folder | What it is |
|---|---|
| `ground_station/` | Python app for the laptop: detection, ranking, map |
| `esp32_cam_stream/` | ESP32-CAM sketch that streams video over Wi-Fi |
| `esp32_telemetry/` | Second ESP32: GPS + MLX90640 thermal, sent to the laptop over UDP |
| `esp32_flight_controller/` | Custom flight controller, ESP32 + MPU6050 (bench prototype) |

---

## 1. Laptop: run it in 2 minutes (no hardware needed)

```bash
cd ground_station
pip install -r requirements.txt
python make_test_video.py test.mp4                 # synthetic pole + rust + tree
python fake_telemetry.py                            # terminal 2: fake GPS + a hot joint
python ground_station.py --source test.mp4 --record
```
Output lands in `runs/<time>/`: `defects.csv` (ranked work list), `defect_map.html`
(open in a browser), `snapshots/`, and `annotated.mp4` (good for your demo video).

With a webcam: `python ground_station.py --source 0 --sim-gps 17.4401,78.3489`.
Hold a rusty bolt and a plant in front of it.

Keys: **q** quit and save · **s** save now · **p** pause.

## 2. The real drone link

1. Turn on a phone hotspot. Put its name and password into **both** ESP32 sketches.
2. Flash `esp32_cam_stream` (Board: *AI Thinker ESP32-CAM*). The Serial Monitor prints
   `Stream: http://192.168.x.x:81/stream`.
3. Flash `esp32_telemetry` on a second ESP32. Install the libraries *TinyGPSPlus* and
   *Adafruit MLX90640* first. It broadcasts on UDP 4210, so you don't need the laptop's IP.
4. Connect the laptop to the same hotspot and run:
   `python ground_station.py --source http://192.168.x.x:81/stream --record`
5. On Windows, click **Allow** when the firewall asks about Python. Otherwise UDP telemetry is blocked.

Power both ESP32 boards from a 5 V UBEC, not from the LiPo directly.
Mount the thermal sensor next to the camera, pointing the same way.
Wi-Fi range is about 50–100 m, which is fine for the demo. For longer range, use an analog
5.8 GHz FPV camera, a USB receiver, and `--source 1`.

**No GPS module yet?** Add `--sim-gps lat,lon` and the map still works with a simulated track.
Say that it is simulated if anyone asks.

## 3. Demo Day test rig (do NOT fly near live lines)

Build a mock pole: a PVC pipe or bamboo with a cross-arm, plus:
- **Corrosion:** rusty bolts, brackets or an old clamp. Real rust shows up best.
- **Vegetation:** a potted plant or branches pushed into the "line" zone.
- **Hotspot:** a cup of hot water or a warm soldering iron (switched off) taped to a joint.
  Use `python fake_telemetry.py 4210 25` if the MLX90640 hasn't arrived.

Fly past it slowly at 3–5 m with `--record` running. That `annotated.mp4` and the
`defect_map.html` screenshot are your demo video and your deck screenshot.

## 4. How severity works

| Defect | Score | Example priority |
|---|---|---|
| Hotspot ΔT >15 °C | 80–100 | P1 dispatch now |
| Hotspot ΔT 4–15 °C | 50–72 | P2 repair in days |
| Corrosion | 30–80, by rusted area | P3–P1 |
| Vegetation in clearance zone | 20–80, by zone coverage | P4–P1 |
| YOLO "broken/damaged" insulator | ~90 | P1 |

P1 ≥ 75, P2 ≥ 50, P3 ≥ 25, otherwise P4. A defect must be seen 3 times (`--min-hits`)
before it is confirmed. Repeated sightings within 6 m are merged into one defect.
The functions to tune are `score_*()` in `ground_station.py`.

Tuning the rust detector: run with `--debug-mask` to see what counts as rust. The HSV and LAB
limits are at the top of `detect_corrosion()`. Clearance zone: `--zone x0,y0,x1,y1`.

## 5. Adding real AI (YOLO)

1. Download **InsPLAD** (github.com/andreluizbvs/InsPLAD) or an insulator-defect or
   corrosion dataset from Roboflow Universe, in YOLO format.
2. `pip install ultralytics`, then
   `yolo detect train data=data.yaml model=yolov8n.pt epochs=50 imgsz=640`
   (use Google Colab's free GPU).
3. Run `python ground_station.py --source ... --model runs/detect/train/weights/best.pt`.
   Class names containing *broken, damaged, flashover, corrosion, rust, nest, vegetation*
   become defects (see `YOLO_CLASS_MAP`).

## 6. ESP32 flight controller: bench checks (PROPS OFF)

Wiring and pins are in the header of `esp32_flight_controller.ino`.

1. **ESC calibration (once):** set `ESC_CALIBRATE 1`, upload, plug in the LiPo when told, wait
   for the beeps, then set it back to 0 and upload again.
2. Open the Serial Monitor at 115200 and keep the drone still while the gyro calibrates.
3. **Tilt right**: R must go positive. **Nose up**: P must go positive. If not, flip `ROLL_SIGN`
   or `PITCH_SIGN`.
4. Arm with throttle down and SwA up. All four motors should idle.
5. Raise the throttle a bit, then **tilt the frame right by hand**: the **right** motors (M1, M2)
   should speed up to push it back. Nose down: the front motors (M1, M4) speed up.
   Yaw stick right: M1 and M3 (CCW) speed up.
6. Switch off the transmitter: motors must stop within 0.1 s (failsafe).
7. Only after all of that: props on, drone tied to a single-axis rig (a rod through the frame),
   then tune P, then D, then I.

Fly all public demos on the **KK2.1.5** until the ESP32 controller passes every step above.

## 7. Rules (India)

Your F450 is in the **micro category (250 g – 2 kg)**. Non-commercial micro drones don't need a
remote pilot certificate, but commercial work for a DISCOM will (and the drone must be
registered on Digital Sky). Fly only in green zones, below 120 m, in line of sight, and keep
well away from live conductors unless the utility has given permission.

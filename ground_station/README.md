# Ground station

Laptop app that turns live drone video into a ranked list of grid defects.

```
video (ESP32-CAM / webcam / file) ─┐
GPS + thermal (UDP 4210) ──────────┼─▶ detect ─▶ score P1–P4 ─▶ GPS tag ─▶ defects.csv
                                   │                                    ─▶ defect_map.html
                                   └────────────────────────────────────▶ annotated.mp4
```

| File | What it does |
|---|---|
| `ground_station.py` | Main app: rust, vegetation and thermal detection, optional YOLO, severity scoring, map and CSV |
| `make_test_video.py` | Makes a synthetic fly-past video (pole, rusty bracket, tree) for testing |
| `fake_telemetry.py` | Pretends to be the drone: sends GPS and a thermal hotspot over UDP |
| `requirements.txt` | `opencv-python`, `numpy` (`ultralytics` only if you use `--model`) |

## Run

```bash
pip install -r requirements.txt
python make_test_video.py test.mp4
python fake_telemetry.py &                         # optional: fake GPS + hotspot
python ground_station.py --source test.mp4 --record
```

| Source | `--source` |
|---|---|
| Laptop webcam | `0` |
| ESP32-CAM | `http://<ip>:81/stream` |
| Phone (IP Webcam app) | `http://<ip>:8080/video` |
| Recorded flight | `flight.mp4` |

Useful flags: `--sim-gps 17.44,78.35` (no GPS module), `--model best.pt` (YOLO), `--debug-mask` (see the rust mask), `--zone x0,y0,x1,y1` (clearance zone), `--no-gui` (headless).

Keys in the window: **q** quit and save · **s** save now · **p** pause.

Each run writes to `runs/<timestamp>/`: `defects.csv`, `defect_map.html`, `snapshots/`, and `annotated.mp4` with `--record`.

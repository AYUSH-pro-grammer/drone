# Telemetry node: GPS + thermal

A second ESP32 that sends GPS position and a thermal hotspot summary to the ground station about 4 times a second (UDP broadcast, port 4210).

| Part | ESP32 pins |
|---|---|
| NEO-6M GPS | TX → GPIO16, RX → GPIO17, VCC, GND |
| MLX90640 thermal camera | SDA → GPIO21, SCL → GPIO22, 3.3 V, GND |

**Libraries:** TinyGPSPlus (Mikal Hart), Adafruit MLX90640.

**Packet:**
```json
{"lat":17.44,"lon":78.34,"alt":520.1,"sats":8,"tmax":49.2,"tmed":31.0,"hx":9,"hy":4,"tw":16,"th":12}
```
`tmax − tmed` is the temperature rise (ΔT) the ground station uses to rank hotspots. `hx`, `hy` give the hottest cell on a 16×12 grid.

Keep this separate from the flight controller ESP32. Wi-Fi and sensor work should never share a chip with flight control.

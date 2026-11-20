# ESP32-CAM video stream

Streams live MJPEG video from the drone to the ground station over Wi-Fi.

- **Board:** AI Thinker ESP32-CAM (Arduino IDE, esp32 core by Espressif)
- **Set** `WIFI_SSID` and `WIFI_PASS` (a phone hotspot works well in the field)
- **Stream:** `http://<ip>:81/stream`. The IP is printed on the Serial Monitor at 115200.
- **Power:** 5 V from a UBEC, not straight from the LiPo
- **Image upside down?** Set `set_vflip(s, 1)` in `setup()`

Use it with: `python ground_station.py --source http://<ip>:81/stream`

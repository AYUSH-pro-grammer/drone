/*
  GridDrishti – telemetry node: GPS (NEO-6M) + thermal (MLX90640) -> UDP to laptop
  -------------------------------------------------------------------------------
  Use a SEPARATE ESP32 dev board from the flight controller (never mix flight
  control with Wi-Fi/sensor work on one chip).

  Wiring
    NEO-6M GPS : TX -> GPIO16 (RX2), RX -> GPIO17 (TX2), VCC 5V/3.3V, GND
    MLX90640   : SDA -> GPIO21, SCL -> GPIO22, VCC 3.3V, GND
  Libraries (Library Manager): "TinyGPSPlus" by Mikal Hart,
                               "Adafruit MLX90640"
  Packet (broadcast on UDP 4210, ~4 per second):
    {"lat":17.44,"lon":78.34,"alt":520.1,"sats":8,"tmax":49.2,"tmed":31.0,
     "hx":9,"hy":4,"tw":16,"th":12}
  hx,hy = hottest cell on a 16x12 grid (thermal image downsampled 2x).
*/
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <TinyGPSPlus.h>
#include <Adafruit_MLX90640.h>
#include <algorithm>

const char* WIFI_SSID = "YOUR_HOTSPOT";
const char* WIFI_PASS = "YOUR_PASSWORD";
const uint16_t UDP_PORT = 4210;
const bool MIRROR_X = true;   // MLX90640 image is mirrored vs. a forward camera; flip if hotspot box is on the wrong side

TinyGPSPlus gps;
HardwareSerial GPSSerial(2);
Adafruit_MLX90640 mlx;
WiFiUDP udp;
float frame[32 * 24];
float sorted_[32 * 24];
bool haveMLX = false;

void setup() {
  Serial.begin(115200);
  GPSSerial.begin(9600, SERIAL_8N1, 16, 17);

  Wire.begin(21, 22);
  Wire.setClock(400000);
  haveMLX = mlx.begin(MLX90640_I2CADDR_DEFAULT, &Wire);
  if (haveMLX) {
    mlx.setMode(MLX90640_CHESS);
    mlx.setResolution(MLX90640_ADC_18BIT);
    mlx.setRefreshRate(MLX90640_8_HZ);   // ~4 full frames/s in chess mode
    Serial.println("MLX90640 OK");
  } else {
    Serial.println("MLX90640 not found - sending GPS only");
  }

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) { delay(300); Serial.print("."); }
  Serial.printf("\nIP %s, broadcasting to UDP %u\n", WiFi.localIP().toString().c_str(), UDP_PORT);
  udp.begin(UDP_PORT);
}

void loop() {
  while (GPSSerial.available()) gps.encode(GPSSerial.read());

  static uint32_t last = 0;
  if (millis() - last < 250) return;
  last = millis();

  char buf[256];
  int n = snprintf(buf, sizeof(buf), "{\"sats\":%u", gps.satellites.isValid() ? gps.satellites.value() : 0);
  if (gps.location.isValid() && gps.location.age() < 3000) {
    n += snprintf(buf + n, sizeof(buf) - n, ",\"lat\":%.7f,\"lon\":%.7f,\"alt\":%.1f",
                  gps.location.lat(), gps.location.lng(), gps.altitude.meters());
  }

  if (haveMLX && mlx.getFrame(frame) == 0) {
    // hottest 2x2 block on a 16x12 grid (averaging cuts single-pixel noise)
    float best = -1000; int bx = 0, by = 0;
    for (int y = 0; y < 12; y++)
      for (int x = 0; x < 16; x++) {
        int i = (2 * y) * 32 + 2 * x;
        float v = (frame[i] + frame[i + 1] + frame[i + 32] + frame[i + 33]) / 4.0f;
        if (v > best) { best = v; bx = x; by = y; }
      }
    memcpy(sorted_, frame, sizeof(frame));
    std::nth_element(sorted_, sorted_ + 384, sorted_ + 768);
    float med = sorted_[384];   // median = "similar components / background"
    if (MIRROR_X) bx = 15 - bx;
    n += snprintf(buf + n, sizeof(buf) - n,
                  ",\"tmax\":%.1f,\"tmed\":%.1f,\"hx\":%d,\"hy\":%d,\"tw\":16,\"th\":12",
                  best, med, bx, by);
  }
  snprintf(buf + n, sizeof(buf) - n, "}");

  udp.beginPacket(IPAddress(255, 255, 255, 255), UDP_PORT);
  udp.print(buf);
  udp.endPacket();
  Serial.println(buf);
}

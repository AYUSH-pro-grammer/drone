/*
  GridDrishti – ESP32-CAM (AI-Thinker) MJPEG video stream
  -------------------------------------------------------
  Board:  "AI Thinker ESP32-CAM"   (Arduino IDE, esp32 core by Espressif)
  Stream: http://<printed-IP>:81/stream   -> give this to ground_station.py --source
  Also:   http://<printed-IP>/            -> simple viewer page

  Wi-Fi: put the laptop and the drone on the same network. Easiest in the field:
  turn on a phone hotspot and connect both the ESP32-CAM and the laptop to it.

  Power the ESP32-CAM from a clean 5 V BEC/UBEC (not straight from the LiPo).
*/
#include "esp_camera.h"
#include "esp_http_server.h"
#include <WiFi.h>

const char* WIFI_SSID = "YOUR_HOTSPOT";
const char* WIFI_PASS = "YOUR_PASSWORD";

// AI-Thinker pin map
#define PWDN_GPIO_NUM 32
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 0
#define SIOD_GPIO_NUM 26
#define SIOC_GPIO_NUM 27
#define Y9_GPIO_NUM 35
#define Y8_GPIO_NUM 34
#define Y7_GPIO_NUM 39
#define Y6_GPIO_NUM 36
#define Y5_GPIO_NUM 21
#define Y4_GPIO_NUM 19
#define Y3_GPIO_NUM 18
#define Y2_GPIO_NUM 5
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM 23
#define PCLK_GPIO_NUM 22

#define PART_BOUNDARY "123456789000000000000987654321"
static const char* STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char* STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

static esp_err_t stream_handler(httpd_req_t* req) {
  char part[64];
  esp_err_t res = httpd_resp_set_type(req, STREAM_CONTENT_TYPE);
  if (res != ESP_OK) return res;
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  while (true) {
    camera_fb_t* fb = esp_camera_fb_get();
    if (!fb) { res = ESP_FAIL; break; }
    size_t hlen = snprintf(part, sizeof(part), STREAM_PART, fb->len);
    res = httpd_resp_send_chunk(req, STREAM_BOUNDARY, strlen(STREAM_BOUNDARY));
    if (res == ESP_OK) res = httpd_resp_send_chunk(req, part, hlen);
    if (res == ESP_OK) res = httpd_resp_send_chunk(req, (const char*)fb->buf, fb->len);
    esp_camera_fb_return(fb);
    if (res != ESP_OK) break;   // client disconnected
  }
  return res;
}

static esp_err_t index_handler(httpd_req_t* req) {
  static const char page[] =
    "<html><body style='margin:0;background:#0D1726'>"
    "<img id=s style='width:100%'><script>"
    "s.src=location.protocol+'//'+location.hostname+':81/stream'</script></body></html>";
  httpd_resp_set_type(req, "text/html");
  return httpd_resp_send(req, page, strlen(page));
}

void startServers() {
  httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
  httpd_handle_t web = NULL, stream = NULL;
  httpd_uri_t idx = {.uri = "/", .method = HTTP_GET, .handler = index_handler, .user_ctx = NULL};
  if (httpd_start(&web, &cfg) == ESP_OK) httpd_register_uri_handler(web, &idx);
  cfg.server_port = 81;
  cfg.ctrl_port += 1;
  httpd_uri_t st = {.uri = "/stream", .method = HTTP_GET, .handler = stream_handler, .user_ctx = NULL};
  if (httpd_start(&stream, &cfg) == ESP_OK) httpd_register_uri_handler(stream, &st);
}

void setup() {
  Serial.begin(115200);
  camera_config_t c = {};
  c.ledc_channel = LEDC_CHANNEL_0;
  c.ledc_timer = LEDC_TIMER_0;
  c.pin_d0 = Y2_GPIO_NUM; c.pin_d1 = Y3_GPIO_NUM; c.pin_d2 = Y4_GPIO_NUM; c.pin_d3 = Y5_GPIO_NUM;
  c.pin_d4 = Y6_GPIO_NUM; c.pin_d5 = Y7_GPIO_NUM; c.pin_d6 = Y8_GPIO_NUM; c.pin_d7 = Y9_GPIO_NUM;
  c.pin_xclk = XCLK_GPIO_NUM; c.pin_pclk = PCLK_GPIO_NUM; c.pin_vsync = VSYNC_GPIO_NUM;
  c.pin_href = HREF_GPIO_NUM; c.pin_sccb_sda = SIOD_GPIO_NUM; c.pin_sccb_scl = SIOC_GPIO_NUM;
  c.pin_pwdn = PWDN_GPIO_NUM; c.pin_reset = RESET_GPIO_NUM;
  c.xclk_freq_hz = 20000000;
  c.pixel_format = PIXFORMAT_JPEG;
  c.frame_size = FRAMESIZE_VGA;      // 640x480: good balance of detail vs. Wi-Fi range
  c.jpeg_quality = 12;               // 10-15; lower = better image, more bandwidth
  c.fb_count = psramFound() ? 2 : 1;
  c.grab_mode = CAMERA_GRAB_LATEST;  // always send the newest frame (low latency)
  c.fb_location = psramFound() ? CAMERA_FB_IN_PSRAM : CAMERA_FB_IN_DRAM;

  if (esp_camera_init(&c) != ESP_OK) {
    Serial.println("Camera init failed - check the board selection and ribbon cable");
    delay(3000);
    ESP.restart();
  }
  sensor_t* s = esp_camera_sensor_get();
  s->set_vflip(s, 0);    // set to 1 if the image is upside-down on your mount
  s->set_hmirror(s, 0);

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);  // Wi-Fi power save adds big video lag
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("Connecting");
  while (WiFi.status() != WL_CONNECTED) { delay(300); Serial.print("."); }
  Serial.printf("\nStream: http://%s:81/stream\n", WiFi.localIP().toString().c_str());
  startServers();
}

void loop() { delay(1000); }

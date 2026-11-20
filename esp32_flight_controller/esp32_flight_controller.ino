/*
  GridDrishti – custom flight controller: ESP32 + MPU6050 (X-quad, self-levelling)
  ===============================================================================
  !!! BENCH TEST WITH PROPELLERS REMOVED !!!
  This is a prototype. Fly the demo video on the KK2.1.5. Only put props on
  after every check in the README passes, with the drone tied down, outdoors,
  nobody near it.

  Board: any ESP32 dev board (esp32 core 2.x or 3.x)

  Wiring
    MPU6050   : SDA -> GPIO21, SCL -> GPIO22, VCC 3.3V, GND. Mount flat, arrow (Y) to the FRONT.
    Receiver  : CH1 roll -> GPIO36 (VP), CH2 pitch -> GPIO39 (VN), CH3 throttle -> GPIO34,
                CH4 yaw -> GPIO35, CH5 (SwA, arm switch) -> GPIO32.  Receiver powered 5V,
                its signal is 3.3V-safe with most FlySky receivers (check yours).
    ESC signal: M1 front-right -> GPIO25, M2 rear-right -> GPIO26,
                M3 rear-left  -> GPIO27, M4 front-left -> GPIO33.  Common GND with ESCs!
    Props     : M1 & M3 spin counter-clockwise, M2 & M4 clockwise (seen from above).

  Arming: throttle fully down + CH5 switch up (>1700us). Disarm: switch down.
  Failsafe: no receiver signal for 100 ms -> motors stop.

  Serial monitor 115200: prints angles, RC inputs and motor outputs at 10 Hz.
  Use it to check directions (see README "bench checks").
*/
#include <Wire.h>

// ---------------- user settings ----------------
#define ESC_CALIBRATE 0     // 1 = one-time ESC range calibration (PROPS OFF, see README), then set back to 0
const int MOTOR_PIN[4] = {25, 26, 27, 33};
const int RC_PIN[5]    = {36, 39, 34, 35, 32};   // roll, pitch, throttle, yaw, arm
const int MOTOR_IDLE = 1100;     // spin when armed at zero throttle
const int MOTOR_MAX  = 1800;     // headroom for control; raise to 2000 later
const float MAX_ANGLE = 25.0f;   // deg, full stick tilt
const float MAX_YAW_RATE = 150.0f; // deg/s, full yaw stick

// Axis sign fixes: tilt right -> R must go positive, nose up -> P must go positive
// in the serial print. If not (board mounted differently), flip the sign here.
const float ROLL_SIGN = 1, PITCH_SIGN = 1, YAW_SIGN = 1;

// PID gains (start here; tune on a single-axis test rig)
float kp_r = 1.3f, ki_r = 0.04f, kd_r = 18.0f, il_r = 400;   // roll & pitch (rate loop)
float kp_y = 4.0f, ki_y = 0.02f, kd_y = 0.0f,  il_y = 400;   // yaw (rate loop)
float kp_level = 4.0f;                                       // angle -> rate (deg/s per deg)

// ---------------- receiver (interrupt pulse capture) ----------------
volatile uint32_t rcRise[5];
volatile uint16_t rcVal[5] = {1500, 1500, 1000, 1500, 1000};
volatile uint32_t rcLast = 0;

void IRAM_ATTR rcIsr(void* arg) {
  int i = (int)(intptr_t)arg;
  uint32_t now = micros();
  if (digitalRead(RC_PIN[i])) rcRise[i] = now;
  else {
    uint32_t w = now - rcRise[i];
    if (w > 900 && w < 2100) { rcVal[i] = w; rcLast = now; }
  }
}

// ---------------- ESC output (LEDC, 400 Hz, 14-bit) ----------------
const int PWM_FREQ = 400, PWM_RES = 14;
const float US_TO_DUTY = (float)((1 << PWM_RES) - 1) / (1000000.0f / PWM_FREQ);

void escBegin() {
  for (int i = 0; i < 4; i++) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcAttach(MOTOR_PIN[i], PWM_FREQ, PWM_RES);
#else
    ledcSetup(i, PWM_FREQ, PWM_RES);
    ledcAttachPin(MOTOR_PIN[i], i);
#endif
  }
}
void escWrite(int i, int us) {
  uint32_t duty = (uint32_t)(us * US_TO_DUTY);
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(MOTOR_PIN[i], duty);
#else
  ledcWrite(i, duty);
#endif
}
void escAll(int us) { for (int i = 0; i < 4; i++) escWrite(i, us); }

// ---------------- MPU6050 ----------------
const uint8_t MPU = 0x68;
float gyroOff[3] = {0, 0, 0};
float gx, gy, gz, ax, ay, az;

void mpuWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU); Wire.write(reg); Wire.write(val); Wire.endTransmission();
}
bool mpuRead() {
  Wire.beginTransmission(MPU); Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(MPU, (uint8_t)14) != 14) return false;
  int16_t r[7];
  for (int i = 0; i < 7; i++) r[i] = (Wire.read() << 8) | Wire.read();
  ax = r[0] / 4096.0f; ay = r[1] / 4096.0f; az = r[2] / 4096.0f;     // g  (+-8g)
  gx = r[4] / 65.5f - gyroOff[0];                                     // deg/s (+-500)
  gy = r[5] / 65.5f - gyroOff[1];
  gz = r[6] / 65.5f - gyroOff[2];
  return true;
}
void mpuBegin() {
  Wire.begin(21, 22);
  Wire.setClock(400000);
  mpuWrite(0x6B, 0x00);   // wake
  mpuWrite(0x1B, 0x08);   // gyro +-500 dps
  mpuWrite(0x1C, 0x10);   // accel +-8 g
  mpuWrite(0x1A, 0x03);   // DLPF ~42 Hz
  delay(100);
  Serial.println("Calibrating gyro - keep the drone still...");
  float s[3] = {0, 0, 0}; int n = 0;
  for (int i = 0; i < 2000; i++) {
    if (mpuRead()) { s[0] += gx; s[1] += gy; s[2] += gz; n++; }
    escAll(1000);           // keep ESCs fed with 'motor off' during calibration
    delayMicroseconds(2500);
  }
  for (int k = 0; k < 3; k++) gyroOff[k] = s[k] / n;
  Serial.println("Gyro calibrated.");
}

// ---------------- PID ----------------
struct PID { float kp, ki, kd, ilim, i = 0, prev = 0; };
PID pidR, pidP, pidY;
float pidStep(PID& p, float err) {
  p.i += p.ki * err;
  p.i = constrain(p.i, -p.ilim, p.ilim);
  float d = p.kd * (err - p.prev);
  p.prev = err;
  return constrain(p.kp * err + p.i + d, -400.0f, 400.0f);
}
void pidReset() { pidR.i = pidP.i = pidY.i = 0; pidR.prev = pidP.prev = pidY.prev = 0; }

// ---------------- main ----------------
float roll = 0, pitch = 0;    // estimated angles (deg)
bool armed = false;
int motor[4] = {1000, 1000, 1000, 1000};
const uint32_t LOOP_US = 4000;  // 250 Hz
uint32_t loopT;

void setup() {
  Serial.begin(115200);
  escBegin();
  escAll(1000);
#if ESC_CALIBRATE
  Serial.println("ESC CALIBRATION - PROPS OFF! Plug in the LiPo within 5 s...");
  escAll(2000); delay(8000);    // ESCs beep 'max'
  escAll(1000); delay(4000);    // ESCs beep 'min' -> calibrated
  Serial.println("Done. Set ESC_CALIBRATE to 0 and re-upload.");
  while (true) { escAll(1000); delay(10); }
#endif
  for (int i = 0; i < 5; i++) {
    pinMode(RC_PIN[i], INPUT);
    attachInterruptArg(digitalPinToInterrupt(RC_PIN[i]), rcIsr, (void*)(intptr_t)i, CHANGE);
  }
  mpuBegin();
  // start the angle estimate from the accelerometer
  mpuRead();
  roll  = ROLL_SIGN  * atan2f(-ax, az) * 57.2958f;
  pitch = PITCH_SIGN * atan2f(ay, sqrtf(ax * ax + az * az)) * 57.2958f;
  pidR = {kp_r, ki_r, kd_r, il_r}; pidP = {kp_r, ki_r, kd_r, il_r}; pidY = {kp_y, ki_y, kd_y, il_y};
  loopT = micros();
}

void loop() {
  // ---- fixed 250 Hz loop ----
  while (micros() - loopT < LOOP_US) {}
  float dt = (micros() - loopT) / 1e6f;
  loopT = micros();

  // ---- sensors ----
  if (!mpuRead()) { escAll(1000); armed = false; return; }   // IMU fault -> stop
  // Board frame: Y forward, X right, Z up.  Roll + = right side down,
  // pitch + = nose up, yaw + = nose turning right (clockwise from above).
  float rateR = ROLL_SIGN * gy, rateP = PITCH_SIGN * gx, rateY = -YAW_SIGN * gz;
  float accR = ROLL_SIGN  * atan2f(-ax, az) * 57.2958f;
  float accP = PITCH_SIGN * atan2f(ay, sqrtf(ax * ax + az * az)) * 57.2958f;
  // complementary filter: trust gyro short-term, accelerometer long-term
  roll  = 0.996f * (roll  + rateR * dt) + 0.004f * accR;
  pitch = 0.996f * (pitch + rateP * dt) + 0.004f * accP;

  // ---- receiver ----
  uint16_t rc[5];
  noInterrupts(); for (int i = 0; i < 5; i++) rc[i] = rcVal[i]; uint32_t last = rcLast; interrupts();
  bool signalOk = (micros() - last) < 100000;

  // ---- arming / failsafe ----
  if (!signalOk) armed = false;
  if (!armed && signalOk && rc[4] > 1700 && rc[2] < 1050) { armed = true; pidReset(); }
  if (armed && rc[4] < 1300) armed = false;
  if (fabsf(roll) > 60 || fabsf(pitch) > 60) armed = false;   // flipped/crashed -> stop

  // ---- control ----
  auto stick = [](int us) { int v = us - 1500; return abs(v) < 12 ? 0 : v; };   // deadband
  float angSpR = stick(rc[0]) / 500.0f * MAX_ANGLE;
  float angSpP = stick(rc[1]) / 500.0f * MAX_ANGLE;
  float rateSpR = kp_level * (angSpR - roll);
  float rateSpP = kp_level * (angSpP - pitch);
  float rateSpY = stick(rc[3]) / 500.0f * MAX_YAW_RATE;

  int thr = rc[2];
  if (armed && thr > 1080) {
    thr = min(thr, MOTOR_MAX - 100);   // leave room for corrections
    float oR = pidStep(pidR, rateSpR - rateR);
    float oP = pidStep(pidP, rateSpP - rateP);
    float oY = pidStep(pidY, rateSpY - rateY);
    // X mixer: M1 FR (CCW), M2 RR (CW), M3 RL (CCW), M4 FL (CW)
    // roll right: lower right motors; nose up: raise front motors;
    // yaw right (clockwise): speed up the CCW props (their reaction torque turns the frame CW)
    motor[0] = thr + oP - oR + oY;   // M1 front-right CCW
    motor[1] = thr - oP - oR - oY;   // M2 rear-right  CW
    motor[2] = thr - oP + oR + oY;   // M3 rear-left   CCW
    motor[3] = thr + oP + oR - oY;   // M4 front-left  CW
    for (int i = 0; i < 4; i++) motor[i] = constrain(motor[i], MOTOR_IDLE, MOTOR_MAX);
  } else {
    pidReset();
    for (int i = 0; i < 4; i++) motor[i] = armed ? MOTOR_IDLE : 1000;
  }
  for (int i = 0; i < 4; i++) escWrite(i, motor[i]);

  // ---- debug print at 10 Hz ----
  static uint8_t cnt = 0;
  if (++cnt >= 25) {
    cnt = 0;
    Serial.printf("%s sig:%d | R%6.1f P%6.1f | rc %4u %4u %4u %4u %4u | M %4d %4d %4d %4d | dt %.4f\n",
                  armed ? "ARMED" : "safe ", signalOk, roll, pitch, rc[0], rc[1], rc[2], rc[3], rc[4],
                  motor[0], motor[1], motor[2], motor[3], dt);
  }
}

# ESP32 + MPU6050 flight controller (prototype)

> ⚠️ **Bench test with propellers removed.** The drone flies on the KK2.1.5 until this controller passes every check in [`docs/SETUP.md`](../docs/SETUP.md#6-esp32-flight-controller-bench-checks-props-off).

Self-levelling X-quad controller:
- **Sensor fusion:** a complementary filter combines gyro and accelerometer readings into roll and pitch
- **Control:** double-loop PID. An angle loop sets target rates; a rate loop drives the motors
- **Loop rate:** 250 Hz control loop, 400 Hz PWM signal to the SimonK ESCs
- **Safety:** arm only at zero throttle with the switch up, receiver-loss failsafe (100 ms), motor cut if tilted past 60°

| Signal | ESP32 pin |
|---|---|
| MPU6050 SDA / SCL | GPIO21 / GPIO22 (Y arrow to the front) |
| Receiver CH1 roll, CH2 pitch, CH3 throttle, CH4 yaw, CH5 arm | GPIO36, 39, 34, 35, 32 |
| ESC M1 front-right, M2 rear-right, M3 rear-left, M4 front-left | GPIO25, 26, 27, 33 |

Props: M1 and M3 spin counter-clockwise, M2 and M4 clockwise. Tune the PID gains at the top of the sketch on a single-axis test rig.

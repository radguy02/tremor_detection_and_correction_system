# 🤝 Tremor Detection and Correction System
### ESP32 + MPU-6050 + Servo | Real-Time PD Control with Band-Pass Filtering

> An active tremor-cancellation system that detects hand tremors in the Parkinson's/essential tremor frequency range (3–12 Hz) and drives a servo motor in real-time to counteract the motion — using an ESP32, MPU-6050 IMU, and a PD feedback controller.

---


## 🧠 How It Works

The system runs a real-time control loop at **200 Hz** on the ESP32:

```
MPU-6050 (raw accel) → Band-Pass Filter (3–12 Hz) → PD Controller → Servo Motor
```

1. **Sensor Reading** — The MPU-6050 accelerometer provides raw 16-bit acceleration data over I2C. The X-axis value is converted to g-force units.

2. **Band-Pass Filtering** — A cascaded 1st-order high-pass (3 Hz) and low-pass (12 Hz) filter isolates the tremor frequency band, rejecting slow intentional movement (< 3 Hz) and high-frequency noise (> 12 Hz).

3. **PD Controller** — A Proportional-Derivative controller computes a correction angle:
   ```
   controlDeg = -Kp × filtered − Kd × derivative
   ```
   The negative sign ensures the servo moves *opposite* to the detected tremor.

4. **Servo Output** — The correction is applied to a servo motor centered at 90°, clamped to a ±45° range (45°–135°) for mechanical safety.

---

## ⚙️ Filter Design

| Parameter | Value |
|-----------|-------|
| High-Pass Corner | 3 Hz |
| Low-Pass Corner | 12 Hz |
| Filter Order | 1st-order (cascaded) |
| Sample Rate | 200 Hz |

The coefficients are computed from the RC time constants at startup:

```
alpha_high = RC_high / (RC_high + DT)   // High-pass coefficient
alpha_low  = DT / (RC_low + DT)         // Low-pass coefficient
```

This range targets pathological tremor (Parkinson's disease: ~4–6 Hz, essential tremor: ~4–12 Hz) while passing voluntary movement through unaffected.

---

## 🎛️ PD Controller Tuning

| Gain | Value | Effect |
|------|-------|--------|
| `Kp` | 45.0 | Maps filtered acceleration (g) → servo degrees |
| `Kd` | 6.0 | Damps oscillation, maps g/s → servo degrees |

Tune `Kp` and `Kd` by observing Serial output. Increase `Kp` for stronger correction; increase `Kd` if the servo oscillates.

---

## 🔧 Hardware

| Component | Details |
|-----------|---------|
| Microcontroller | ESP32 (any variant) |
| IMU | MPU-6050 (I2C, address `0x68`) |
| Actuator | Standard PWM servo motor |
| Servo Pin | GPIO 15 |
| I2C | Default SDA/SCL pins |

### Wiring

```
MPU-6050 VCC  →  3.3V
MPU-6050 GND  →  GND
MPU-6050 SDA  →  ESP32 SDA (GPIO 21)
MPU-6050 SCL  →  ESP32 SCL (GPIO 22)

Servo Signal  →  GPIO 15
Servo VCC     →  5V (external recommended)
Servo GND     →  GND
```

> ⚠️ Power the servo from an external 5V supply if the ESP32 USB power is insufficient.

---

## 📦 Dependencies

Install via Arduino Library Manager or PlatformIO:

| Library | Purpose |
|---------|---------|
| `ESP32Servo` | PWM servo control on ESP32 |
| `Wire` | I2C communication (built-in) |


---

## 🚀 Getting Started

1. **Clone the repo**
   ```bash
   git clone https://github.com/your-username/tremor-suppression-device.git
   ```

2. **Open in Arduino IDE or PlatformIO**

3. **Install dependencies** (see above)

4. **Upload to ESP32**

5. **Open Serial Monitor** at `115200 baud` to observe:
   ```
   Filtered_g    ServoAngle
   0.00123       91
   -0.00451      88
   ...
   ```

---

## 📊 Serial Output

The device streams two tab-separated values at 200 Hz:

| Column | Unit | Description |
|--------|------|-------------|
| `filteredValue` | g | Band-pass filtered acceleration |
| `servoAngle` | degrees | Commanded servo position (45–135°) |

You can plot this live using the **Arduino Serial Plotter** or pipe it into Python/MATLAB for analysis.

---

## 📐 Servo Limits

| Parameter | Value |
|-----------|-------|
| Neutral Position | 90° |
| Minimum Angle | 45° |
| Maximum Angle | 135° |
| Range | ±45° |

---

## 🔬 Background

Tremor is an involuntary, rhythmic muscle contraction affecting millions of people with neurological conditions such as Parkinson's disease and essential tremor. This project explores **active mechanical cancellation** as a low-cost, embedded approach to tremor suppression — an alternative to pharmaceutical treatment or deep brain stimulation.

---

## 🛠️ Potential Improvements

- [ ] Replace 1st-order filters with a 2nd-order Butterworth for sharper roll-off
- [ ] Add gyroscope data fusion (complementary/Kalman filter) for better motion estimation
- [ ] Implement adaptive gain scheduling based on detected tremor amplitude
- [ ] Add a multi-axis servo gimbal for 2D/3D tremor cancellation
- [ ] BLE telemetry for wireless monitoring
- [ ] Battery-powered, wearable enclosure

---

## 📄 License

MIT License — feel free to use, modify, and build on this project.

---

## 🙏 Acknowledgements

- MPU-6050 datasheet and InvenSense application notes
- `ESP32Servo` library by Kevin Harrington

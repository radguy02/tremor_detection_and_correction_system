/*
  - Band-pass filter (3-12 Hz) to isolate tremor
  - PD controller drives a servo to counteract tremor
  - Serial outputs: filteredValue (g) and servoAngle (deg)
*/

#include <Wire.h>
#include <ESP32Servo.h>

#define MPU_ADDR 0x68
#define SERVO_PIN 15   
#define SAMPLE_HZ 200.0
#define DT (1.0 / SAMPLE_HZ)

Servo myServo;

// --- Filter cutoffs ---
const float F_LOW = 3.0;   // high-pass corner (Hz)
const float F_HIGH = 12.0; // low-pass corner (Hz)

// --- Filter state ---
float hp_y = 0.0, hp_x = 0.0;
float lp_y = 0.0;

// --- Precompute coefficients for simple 1st-order HP then LP ---
float RC_high;
float RC_low;
float alpha_high;
float alpha_low;

// --- PD controller gains ---
float Kp = 45.0;    // proportional gain (maps g -> degrees roughly)
float Kd = 6.0;     // derivative gain (maps g/s -> degrees)
float prev_filtered = 0.0;

// Servo limits & neutral
const int SERVO_NEUTRAL = 90;
const int SERVO_MIN = 45;
const int SERVO_MAX = 135;

unsigned long lastMicros = 0;
unsigned long microsPerSample;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);  
  Wire.write(0x00);
  Wire.endTransmission(true);

  delay(50);

  RC_high = 1.0 / (2.0 * PI * F_LOW);
  RC_low  = 1.0 / (2.0 * PI * F_HIGH);
  alpha_high = RC_high / (RC_high + DT);
  alpha_low = DT / (RC_low + DT);

  myServo.attach(SERVO_PIN);
  myServo.write(SERVO_NEUTRAL);

  microsPerSample = (unsigned long)(1e6 / SAMPLE_HZ);
  lastMicros = micros();
  Serial.println("Tremor demo started (MPU6050 via I2C, PD control).");
  Serial.println("Filtered_g\tServoAngle");
}

void loop() {
  unsigned long now = micros();
  if (now - lastMicros < microsPerSample) return;
  lastMicros += microsPerSample;

  // Read accelerometer and gyro
  int16_t ax, ay, az, gx, gy, gz;
  readMPU6050All(ax, ay, az, gx, gy, gz);

  // Convert acceleration raw to g
  float ax_g = (float)ax / 16384.0;
  float ay_g = (float)ay / 16384.0;
  float az_g = (float)az / 16384.0;

  float rawSignal = ax_g;

  // HP then LP
  float hp_out = alpha_high * (hp_y + rawSignal - hp_x);
  hp_x = rawSignal;
  hp_y = hp_out;

  lp_y = lp_y + alpha_low * (hp_out - lp_y);
  float filtered = lp_y;

  float derivative = (filtered - prev_filtered) / DT;


  // We want to move servo opposite to sign of tremor: control = -Kp*filtered - Kd*derivative
  float controlDeg = -Kp * filtered - Kd * derivative;

  int servoAngle = SERVO_NEUTRAL + (int)round(controlDeg);
  servoAngle = constrain(servoAngle, SERVO_MIN, SERVO_MAX);
  myServo.write(servoAngle);

  Serial.print(filtered, 5);
  Serial.print("\t");
  Serial.println(servoAngle);

  prev_filtered = filtered;
}

void readMPU6050All(int16_t &ax, int16_t &ay, int16_t &az, int16_t &gx, int16_t &gy, int16_t &gz) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, (uint8_t)14, (uint8_t)true);

  if (Wire.available() >= 14) {
    ax = (Wire.read() << 8) | Wire.read();
    ay = (Wire.read() << 8) | Wire.read();
    az = (Wire.read() << 8) | Wire.read();
    int16_t temp = (Wire.read() << 8) | Wire.read();
    gx = (Wire.read() << 8) | Wire.read();
    gy = (Wire.read() << 8) | Wire.read();
    gz = (Wire.read() << 8) | Wire.read();
  } else {
    // if read failed, return zeros so to avoid big jumps
    ax = ay = az = gx = gy = gz = 0;
  }
}

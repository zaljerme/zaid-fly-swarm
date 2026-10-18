#include "imu.h"
#include "config.h"

static constexpr float GYRO_SCALE  = 1.0f / 16.4f;    // dps per LSB @ 2000 dps
static constexpr float ACCEL_SCALE = 1.0f / 4096.0f;  // g per LSB  @ 8 g

void Imu::write8(uint8_t reg, uint8_t v) {
  Wire.beginTransmission(ADDR);
  Wire.write(reg);
  Wire.write(v);
  Wire.endTransmission();
}

bool Imu::begin(uint8_t sda, uint8_t scl) {
  Wire.begin(sda, scl, 400000);
  Wire.setTimeOut(2);

  Wire.beginTransmission(ADDR);
  if (Wire.endTransmission() != 0) return ok = false;

  write8(0x6B, 0x80);  delay(100);  // PWR_MGMT_1: reset
  write8(0x6B, 0x01);  delay(10);   // clock = PLL with X gyro
  write8(0x1A, 0x03);               // CONFIG: DLPF 42 Hz -> 1 kHz output
  write8(0x19, 0x00);               // SMPLRT_DIV: 1 kHz
  write8(0x1B, 0x18);               // GYRO_CONFIG: +/-2000 dps
  write8(0x1C, 0x10);               // ACCEL_CONFIG: +/-8 g
  return ok = true;
}

bool Imu::readRaw(int16_t* a, int16_t* g) {
  Wire.beginTransmission(ADDR);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)ADDR, 14) != 14) return false;
  uint8_t b[14];
  for (int i = 0; i < 14; i++) b[i] = Wire.read();
  for (int i = 0; i < 3; i++) {
    a[i] = (int16_t)(b[i * 2] << 8 | b[i * 2 + 1]);
    g[i] = (int16_t)(b[8 + i * 2] << 8 | b[9 + i * 2]);
  }
  return true;
}

void Imu::calibrate(uint16_t samples) {
  double gs[3] = {0}, as[3] = {0};
  int16_t a[3], g[3];
  uint16_t n = 0;
  while (n < samples) {
    if (readRaw(a, g)) {
      for (int i = 0; i < 3; i++) { gs[i] += g[i]; as[i] += a[i]; }
      n++;
    }
    delayMicroseconds(1000);
  }
  for (int i = 0; i < 3; i++) {
    gbias[i] = gs[i] / n;
    abias[i] = as[i] / n;
  }
  abias[2] -= 1.0f / ACCEL_SCALE;  // Z should read +1 g when level
  roll = pitch = 0;
  initialized = false;
}

bool Imu::update(float dt) {
  int16_t a[3], g[3];
  if (!readRaw(a, g)) return false;

  // Sensor axes -> body axes (MPU6050 mounted chip-up, X forward)
  gx =  (g[0] - gbias[0]) * GYRO_SCALE;   // roll rate
  gy =  (g[1] - gbias[1]) * GYRO_SCALE;   // pitch rate
  gz = -(g[2] - gbias[2]) * GYRO_SCALE;   // yaw rate, +CW seen from above
  float ax = (a[0] - abias[0]) * ACCEL_SCALE;
  float ay = (a[1] - abias[1]) * ACCEL_SCALE;
  float az = (a[2] - abias[2]) * ACCEL_SCALE;

  float accRoll  = atan2f(ay, az) * RAD_TO_DEG;
  float accPitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * RAD_TO_DEG;

  if (!initialized) {
    roll = accRoll;
    pitch = accPitch;
    initialized = true;
    return true;
  }

  // Only blend the accelerometer in when it measures ~1 g (not during thrust
  // spikes or impacts), otherwise integrate the gyro alone.
  float gnorm = sqrtf(ax * ax + ay * ay + az * az);
  float alpha = (gnorm > 0.8f && gnorm < 1.2f) ? COMP_ALPHA : 1.0f;
  roll  = alpha * (roll  + gx * dt) + (1.0f - alpha) * accRoll;
  pitch = alpha * (pitch + gy * dt) + (1.0f - alpha) * accPitch;
  return true;
}

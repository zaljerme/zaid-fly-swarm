// =============================================================================
//  Minimal MPU6050 driver + complementary attitude filter
//  Gyro +/-2000 dps, accel +/-8 g, DLPF 42 Hz, 1 kHz sample rate.
// =============================================================================
#pragma once
#include <Arduino.h>
#include <Wire.h>

class Imu {
 public:
  bool begin(uint8_t sda, uint8_t scl);
  void calibrate(uint16_t samples = 1000);   // drone must be level & still
  bool update(float dt);                     // read sensor, run filter

  // Body rates (deg/s) and attitude (deg)
  float gx = 0, gy = 0, gz = 0;
  float roll = 0, pitch = 0;
  bool  ok = false;

 private:
  static constexpr uint8_t ADDR = 0x68;
  void  write8(uint8_t reg, uint8_t v);
  bool  readRaw(int16_t* a, int16_t* g);

  float gbias[3] = {0, 0, 0};
  float abias[3] = {0, 0, 0};
  bool  initialized = false;
};

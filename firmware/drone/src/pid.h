#pragma once
#include <math.h>

// PID with derivative-on-measurement (no kick on setpoint steps),
// low-pass filtered D term and clamped integrator.
struct Pid {
  float kp, ki, kd, iLimit;
  float integ = 0, prevMeas = 0, dState = 0;
  float dAlpha = 1.0f;

  Pid(float p, float i, float d, float lim) : kp(p), ki(i), kd(d), iLimit(lim) {}

  void setDLpf(float cutoffHz, float dt) {
    float rc = 1.0f / (2.0f * (float)M_PI * cutoffHz);
    dAlpha = dt / (rc + dt);
  }

  void reset() { integ = 0; dState = 0; }

  float update(float setpoint, float meas, float dt, bool integrate = true) {
    float err = setpoint - meas;
    if (integrate) {
      integ += ki * err * dt;
      if (integ > iLimit) integ = iLimit;
      if (integ < -iLimit) integ = -iLimit;
    }
    float d = -(meas - prevMeas) / dt;
    prevMeas = meas;
    dState += dAlpha * (d - dState);
    return kp * err + integ + kd * dState;
  }
};

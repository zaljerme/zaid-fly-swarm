// =============================================================================
//  Zaid Fly — drone configuration (pins, tuning, limits)
// =============================================================================
#pragma once
#include <Arduino.h>

// ---- Pin map (Seeed XIAO ESP32-S3 on the Zaid Fly carrier PCB) -------------
//   XIAO pad   GPIO   function
//   D0         1      Motor 1 MOSFET gate (front-right, CCW)
//   D1         2      Motor 2 MOSFET gate (rear-right,  CW)
//   D2         3      Motor 3 MOSFET gate (rear-left,   CCW)
//   D3         4      Motor 4 MOSFET gate (front-left,  CW)
//   D4         5      I2C SDA  -> MPU6050
//   D5         6      I2C SCL  -> MPU6050
//   D6         43     Red status LED
//   D7         44     Green status LED
//   D8         7      Headlights (2x white)
//   D9         8      Taillights (2x red)
//   D10        9      Battery sense (VBAT / 2 divider)
static constexpr uint8_t PIN_MOTOR[4]   = {1, 2, 3, 4};
static constexpr uint8_t PIN_SDA        = 5;
static constexpr uint8_t PIN_SCL        = 6;
static constexpr uint8_t PIN_LED_RED    = 43;
static constexpr uint8_t PIN_LED_GREEN  = 44;
static constexpr uint8_t PIN_HEADLIGHT  = 7;
static constexpr uint8_t PIN_TAILLIGHT  = 8;
static constexpr uint8_t PIN_VBAT       = 9;

// ---- Motors (brushed 720 coreless (7x20 mm) via SI2300 low-side MOSFETs) --------------
static constexpr uint32_t MOTOR_PWM_HZ   = 20000;  // above hearing, low MOSFET loss
static constexpr uint8_t  MOTOR_PWM_BITS = 10;     // 0..1023
static constexpr uint16_t MOTOR_MAX      = (1 << MOTOR_PWM_BITS) - 1;
static constexpr uint16_t MOTOR_IDLE     = 40;     // keeps props spinning when armed

// ---- Control loop -------------------------------------------------------------
static constexpr uint32_t LOOP_HZ        = 1000;
static constexpr float    LOOP_DT        = 1.0f / LOOP_HZ;

// Attitude (outer loop, angle -> rate)
static constexpr float ANGLE_P           = 6.0f;
static constexpr float MAX_ANGLE_DEG     = 30.0f;
static constexpr float MAX_RATE_DPS      = 300.0f;

// Rate (inner loop, rate -> motor mix). Start here, tune per frame.
static constexpr float RATE_P_RP = 0.45f, RATE_I_RP = 0.80f, RATE_D_RP = 0.012f;
static constexpr float RATE_P_Y  = 1.20f, RATE_I_Y  = 0.60f, RATE_D_Y  = 0.0f;
static constexpr float I_LIMIT           = 120.0f;
static constexpr float D_LPF_HZ          = 60.0f;

// Complementary filter: how much we trust the gyro each step
static constexpr float COMP_ALPHA        = 0.996f;

// ---- Safety -------------------------------------------------------------------
static constexpr uint32_t LINK_TIMEOUT_MS   = 150;   // no frame -> failsafe land
static constexpr uint32_t FAILSAFE_LAND_MS  = 2500;  // ramp throttle to 0 over this
static constexpr float    MAX_TILT_KILL_DEG = 70.0f; // crash detection
static constexpr uint16_t VBAT_LOW_MV       = 3300;
static constexpr uint16_t VBAT_CRIT_MV      = 3100;
static constexpr float    VBAT_DIVIDER      = 2.0f;  // R8 = R9 = 10k

/*
 * Inverted Pendulum - State Feedback Control (LQR)
 * Controller: ESP32
 * Loop Rate:  200Hz (5ms)
 */

#include <Arduino.h>
#include "driver/pcnt.h"
#include "esp32-hal-ledc.h"

// PINS
#define ENC_THETA_A  32
#define ENC_THETA_B  33
#define ENC_X_A      26
#define ENC_X_B      25
#define PIN_PWM_A    22
#define PIN_PWM_B    23

// CONFIG
#define PCNT_UNIT_THETA  PCNT_UNIT_0
#define PCNT_UNIT_X      PCNT_UNIT_1
#define PWM_FREQ         30000
#define PWM_RES          8

// ENCODER THETA
const float THETA_CPR       = 4000.0f;           // 1000 PPR, X4 quad
const float DEG_PER_COUNT   = 360.0f / THETA_CPR;

// ENCODER X
const float X_CPR           = 2400.0f;           // 600 PPR * X4 quad
const float PULLEY_R_CM     = 1.6f;
const float CM_PER_REV      = 2.0f * PI * PULLEY_R_CM;
const float CM_PER_COUNT    = CM_PER_REV / X_CPR;

// Control Loop
const float Ts              = 0.005f;            // ms
const float K[4]            = { -38.46398f, -27.55161f, 97.14464f, 8.70459f };
// LQR, Q = diag([300, 1, 1200, 1]); R = 0.1; 
 
// vAriables
int64_t totalThetaCount = 0;
int64_t totalXCount = 0;

unsigned long last_time_us = 0;
unsigned long lastPrintUs = 0;
const unsigned long printPeriodUs = 50000; // for log

float xprev = 0.0f;
float phiprev = 0.0f;


void initPCNT(pcnt_unit_t unit, int pinA, int pinB) {
  pcnt_config_t cfg = {};
  
  // Channel 0
  cfg.pulse_gpio_num = pinA;
  cfg.ctrl_gpio_num  = pinB;
  cfg.channel        = PCNT_CHANNEL_0;
  cfg.unit           = unit;
  cfg.pos_mode       = PCNT_COUNT_DEC;
  cfg.neg_mode       = PCNT_COUNT_INC;
  cfg.lctrl_mode     = PCNT_MODE_REVERSE;
  cfg.hctrl_mode     = PCNT_MODE_KEEP;
  cfg.counter_h_lim  = 30000;
  cfg.counter_l_lim  = -30000;
  pcnt_unit_config(&cfg);

  // Channel 1
  cfg.pulse_gpio_num = pinB;
  cfg.ctrl_gpio_num  = pinA;
  cfg.channel        = PCNT_CHANNEL_1;
  cfg.pos_mode       = PCNT_COUNT_INC;
  cfg.neg_mode       = PCNT_COUNT_DEC;
  pcnt_unit_config(&cfg);

  pcnt_set_filter_value(unit, 1000);
  pcnt_filter_enable(unit);
  
  pcnt_counter_pause(unit);
  pcnt_counter_clear(unit);
  pcnt_counter_resume(unit);
}

float getAngleDeg() {
  int16_t cnt = 0;
  pcnt_get_counter_value(PCNT_UNIT_THETA, &cnt);
  pcnt_counter_clear(PCNT_UNIT_THETA);
  totalThetaCount += cnt;

  float angle = totalThetaCount * DEG_PER_COUNT;
  
  // normalize angle
  while (angle > 180.0f)  angle -= 360.0f;
  while (angle <= -180.0f) angle += 360.0f;
  
  return angle;
}

float getXPositionCm() {
  int16_t cnt = 0;
  pcnt_get_counter_value(PCNT_UNIT_X, &cnt);
  pcnt_counter_clear(PCNT_UNIT_X);
  totalXCount += cnt;
  return totalXCount * CM_PER_COUNT;
}

void setMotor(float u_norm) {
  u_norm = constrain(u_norm, -1.0f, 1.0f);
  int duty = fabs(u_norm) * 255;

  if (u_norm >= 0) {
    ledcWrite(PIN_PWM_A, 0);
    ledcWrite(PIN_PWM_B, duty);
  } else {
    ledcWrite(PIN_PWM_B, 0);
    ledcWrite(PIN_PWM_A, duty);
  }
}


void setup() {
  Serial.begin(115200);

  ledcAttach(PIN_PWM_A, PWM_FREQ, PWM_RES);
  ledcAttach(PIN_PWM_B, PWM_FREQ, PWM_RES);
  ledcWrite(PIN_PWM_A, 0);
  ledcWrite(PIN_PWM_B, 0);

  initPCNT(PCNT_UNIT_THETA, ENC_THETA_A, ENC_THETA_B);
  initPCNT(PCNT_UNIT_X, ENC_X_A, ENC_X_B);

  Serial.println("t_us,x,dx,phi,dphi,u_norm");
}

void loop() {
  unsigned long now_us = micros();
  unsigned long dt_us = now_us - last_time_us;

  if (dt_us < (unsigned long)(Ts * 1e6)) return;
  last_time_us = now_us;
  
  float dt = dt_us * 1e-6f;

  // read sensor
  float x = getXPositionCm() * 0.01f; // m
  float phi = getAngleDeg() * (PI / 180.0f); // rad

  float dx = (x - xprev) / dt;
  float dphi = (phi - phiprev) / dt;

  // prev update
  xprev = x;
  phiprev = phi;

  // u = -Kx
  float state[4] = { x, dx, phi, dphi };
  float u = 0;
  for (int i = 0; i < 4; i++) {
    u -= K[i] * state[i];
  }

  float u_norm = u * 0.4f; // scaling pwm
  setMotor(u_norm);

// log
  if (now_us - lastPrintUs >= printPeriodUs) {
    lastPrintUs = now_us;
    Serial.printf("%lu,%.6f,%.6f,%.6f,%.6f,%.6f\n", 
                  now_us, x, dx, phi, dphi, u_norm);
  }
}
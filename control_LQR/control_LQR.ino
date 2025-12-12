/* ============================================================
   INVERTED PENDULUM - OBSERVER-BASED STATE FEEDBACK CONTROL
   ------------------------------------------------------------
   Measured: x (cart pos), phi (angle)
   Control:  u = -K * state
   Output:   Motor PWM and direction pin
   Logging:  CSV every 0.05 seconds
   ============================================================ */

#include <Arduino.h>
#include "driver/pcnt.h"
#include "esp32-hal-ledc.h"

// =========================================================
// 1. KONFIGURASI ENCODER SUDUT (THETA) - PCNT UNIT 0
// =========================================================
#define ENC_THETA_A_GPIO 32
#define ENC_THETA_B_GPIO 33
#define PIN_PWM_A 22
#define PIN_PWM_B 23
#define PCNT_THETA_UNIT  PCNT_UNIT_0

const int frequency  = 30000;
const int resolution = 8;

// Encoder sudut
const float THETA_PPR       = 1000.0f;
const float THETA_QUAD_MULT = 4.0f;
const float THETA_CPR       = THETA_PPR * THETA_QUAD_MULT;
const float DEG_PER_COUNT   = 360.0f / THETA_CPR;
const float START_ANGLE_DEG = 0.0f;

// Akumulator
int64_t totalThetaCount = 0;

// =========================================================
// 2. KONFIGURASI ENCODER POSISI (X) - PCNT UNIT 1
// =========================================================
#define ENC_X_A_GPIO 26
#define ENC_X_B_GPIO 25
#define PCNT_X_UNIT  PCNT_UNIT_1

const float X_PPR            = 600.0f;
const float X_QUAD_MULT      = 4.0f;
const float X_CPR            = X_PPR * X_QUAD_MULT;
const float PULLEY_RADIUS_CM = 1.6f;
const float CM_PER_REV       = 2.0f * PI * PULLEY_RADIUS_CM;
const float CM_PER_COUNT     = CM_PER_REV / X_CPR;

int64_t totalXCount = 0;

#define PCNT_H_LIM  30000
#define PCNT_L_LIM -30000

// =========================================================
// PRINT PERIOD (0.05 s)
// =========================================================
const unsigned long printPeriodUs = 50000;
unsigned long lastPrintUs = 0;

// =========================================================
// PCNT INIT
// =========================================================
void initPCNTUnit(pcnt_unit_t unit, int pinA, int pinB) {
  pcnt_config_t pcnt_config = {};

  pcnt_config.pulse_gpio_num = pinA;
  pcnt_config.ctrl_gpio_num  = pinB;
  pcnt_config.channel        = PCNT_CHANNEL_0;
  pcnt_config.unit           = unit;
  pcnt_config.pos_mode       = PCNT_COUNT_DEC;
  pcnt_config.neg_mode       = PCNT_COUNT_INC;
  pcnt_config.lctrl_mode     = PCNT_MODE_REVERSE;
  pcnt_config.hctrl_mode     = PCNT_MODE_KEEP;
  pcnt_config.counter_h_lim  = PCNT_H_LIM;
  pcnt_config.counter_l_lim  = PCNT_L_LIM;
  pcnt_unit_config(&pcnt_config);

  pcnt_config.pulse_gpio_num = pinB;
  pcnt_config.ctrl_gpio_num  = pinA;
  pcnt_config.channel        = PCNT_CHANNEL_1;
  pcnt_config.pos_mode       = PCNT_COUNT_INC;
  pcnt_config.neg_mode       = PCNT_COUNT_DEC;
  pcnt_unit_config(&pcnt_config);

  pcnt_set_filter_value(unit, 1000);
  pcnt_filter_enable(unit);

  pcnt_counter_pause(unit);
  pcnt_counter_clear(unit);
  pcnt_counter_resume(unit);
}

// =========================================================
// SENSOR READ
// =========================================================
float getAngleDeg() {
  int16_t cnt = 0;
  pcnt_get_counter_value(PCNT_THETA_UNIT, &cnt);
  pcnt_counter_clear(PCNT_THETA_UNIT);
  totalThetaCount += cnt;

  float angle = START_ANGLE_DEG + totalThetaCount * DEG_PER_COUNT;
  while (angle > 180.0f) angle -= 360.0f;
  while (angle <= -180.0f) angle += 360.0f;
  return angle;
}

float getXPositionCm() {
  int16_t cnt = 0;
  pcnt_get_counter_value(PCNT_X_UNIT, &cnt);
  pcnt_counter_clear(PCNT_X_UNIT);
  totalXCount += cnt;
  return totalXCount * CM_PER_COUNT;
}

float deg2rad(float d) {
  return d * PI / 180.0f;
}

/* ------------------------------------------------------------
   SYSTEM MATRICES
   ------------------------------------------------------------ */
const float A_mat[4][4] = {
  {1.0f, 0.004992f, 1.052e-05f, 1.754e-08f},
  {0.0f, 0.9968f,   0.004207f,  1.052e-05f},
  {0.0f,-2.273e-05f,1.0f,       0.005001f},
  {0.0f,-0.009086f, 0.152f,     1.0f}
};

const float B_mat[4] = {
  7.953e-05f,
  0.0318f,
  0.0002273f,
  0.09086f
};

const float C_mat[2][4] = {
  {1,0,0,0},
  {0,0,1,0}
};

float K[4] = { -38.46398f, -27.55161f, 97.14464f, 8.70459f };

/* ------------------------------------------------------------
   GLOBALS
   ------------------------------------------------------------ */
float Ts = 0.005f;
unsigned long last_time_us = 0;

float xprev = 0.0f;
float phiprev = 0.0f;

/* ------------------------------------------------------------
   MOTOR
   ------------------------------------------------------------ */
void setMotor(float u_norm) {
  u_norm = constrain(u_norm, -1.0f, 1.0f);
  float duty = fabs(u_norm);

  if (u_norm >= 0) {
    ledcWrite(PIN_PWM_A, 0);
    ledcWrite(PIN_PWM_B, duty * 255);
  } else {
    ledcWrite(PIN_PWM_B, 0);
    ledcWrite(PIN_PWM_A, duty * 255);
  }
}

/* ------------------------------------------------------------
   SETUP
   ------------------------------------------------------------ */
void setup() {
  Serial.begin(115200);

  ledcAttach(PIN_PWM_A, frequency, resolution);
  ledcAttach(PIN_PWM_B, frequency, resolution);
  ledcWrite(PIN_PWM_A, 0);
  ledcWrite(PIN_PWM_B, 0);

  initPCNTUnit(PCNT_THETA_UNIT, ENC_THETA_A_GPIO, ENC_THETA_B_GPIO);
  initPCNTUnit(PCNT_X_UNIT, ENC_X_A_GPIO, ENC_X_B_GPIO);

  Serial.println("t_us,x,dx,phi,dphi,u_norm");
}

/* ------------------------------------------------------------
   LOOP
   ------------------------------------------------------------ */
void loop() {
  unsigned long now_us = micros();
  unsigned long dt_us = now_us - last_time_us;
  if (dt_us < (unsigned long)(Ts * 1e6)) return;
  last_time_us = now_us;
  float dt = dt_us * 1e-6f;

  float x = getXPositionCm() * 0.01f;
  float dx = (x - xprev) / dt;
  xprev = x;

  float phi = deg2rad(getAngleDeg());
  float dphi = (phi - phiprev) / dt;
  phiprev = phi;

  float state[4] = { x, dx, phi, dphi };

  float u = 0;
  for (int i = 0; i < 4; i++)
    u -= K[i] * state[i];

  float u_norm = u * 0.4f;
  setMotor(u_norm);

  // ---- CSV PRINT every 0.05 s ----
  if (now_us - lastPrintUs >= printPeriodUs) {
    lastPrintUs = now_us;
    Serial.print(now_us); Serial.print(",");
    Serial.print(x,6);    Serial.print(",");
    Serial.print(dx,6);   Serial.print(",");
    Serial.print(phi,6);  Serial.print(",");
    Serial.print(dphi,6); Serial.print(",");
    Serial.println(u_norm,6);
  }
}

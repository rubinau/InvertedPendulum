/* ============================================================
    INVERTED PENDULUM - ACTIVE DISTURBANCE REJECTION CONTROL (ADRC)
    ------------------------------------------------------------
    Measured: x (cart pos), phi (angle)
    Estimated: phi_hat, phidot_hat, f_hat (total disturbance) via ESO
    Control: u = (u0 - f_hat) / B0_HAT
    Output: Motor PWM and direction pin
    ============================================================ */

#include <Arduino.h>
#include "driver/pcnt.h"
#include "esp32-hal-ledc.h"

// =========================================================
// 1. KONFIGURASI ENCODER SUDUT (THETA) - PCNT UNIT 0
// =========================================================

#define ENC_THETA_A_GPIO 32
#define ENC_THETA_B_GPIO 33
#define PIN_PWM_A 23
#define PIN_PWM_B 22
#define PCNT_THETA_UNIT PCNT_UNIT_0

// PWM CONFIG
const int frequency = 30000;
const int resolution = 8;

// PWM channels
const int pwmChannelA = 0;
const int pwmChannelB = 1;

// Encoder Sudut Parameters
const float THETA_PPR = 1000.0f;
const float THETA_QUAD_MULT = 4.0f;
const float THETA_CPR  = THETA_PPR * THETA_QUAD_MULT;
const float DEG_PER_COUNT = 360.0f / THETA_CPR;
const float START_ANGLE_DEG = 0.0f;

// Akumulator Global Sudut
int64_t totalThetaCount = 0;

// =========================================================
// 2. KONFIGURASI ENCODER POSISI (X) - PCNT UNIT 1
// =========================================================

#define ENC_X_A_GPIO 26
#define ENC_X_B_GPIO 25
#define PCNT_X_UNIT PCNT_UNIT_1

const float X_PPR   = 600.0f;
const float X_QUAD_MULT = 4.0f;
const float X_CPR   = X_PPR * X_QUAD_MULT;

const float PULLEY_RADIUS_CM = 1.6f;
const float CM_PER_REV = 2.0f * PI * PULLEY_RADIUS_CM;
const float CM_PER_COUNT = CM_PER_REV / X_CPR;

// Akumulator Global Posisi
int64_t totalXCount = 0;

#define PCNT_H_LIM  30000
#define PCNT_L_LIM -30000

// =========================================================
// FUNGSI INISIALISASI PCNT
// =========================================================
void initPCNTUnit(pcnt_unit_t unit, int pinA, int pinB)
{
    pcnt_config_t pcnt_config = {};

    // Channel 0
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

    // Channel 1
    pcnt_config.pulse_gpio_num = pinB;
    pcnt_config.ctrl_gpio_num  = pinA;
    pcnt_config.channel        = PCNT_CHANNEL_1;
    pcnt_config.pos_mode       = PCNT_COUNT_INC;
    pcnt_config.neg_mode       = PCNT_COUNT_DEC;
    pcnt_config.lctrl_mode     = PCNT_MODE_REVERSE;
    pcnt_config.hctrl_mode     = PCNT_MODE_KEEP;
    pcnt_unit_config(&pcnt_config);

    // Filter
    pcnt_set_filter_value(unit, 1000);
    pcnt_filter_enable(unit);

    pcnt_counter_pause(unit);
    pcnt_counter_clear(unit);
    pcnt_counter_resume(unit);
}

// =========================================================
// PEMBACAAN ENCODER
// =========================================================
float getAngleDeg()
{
    int16_t pulseRate = 0;
    pcnt_get_counter_value(PCNT_THETA_UNIT, &pulseRate);
    pcnt_counter_clear(PCNT_THETA_UNIT);

    totalThetaCount += pulseRate;

    float angle = START_ANGLE_DEG + ((float)totalThetaCount * DEG_PER_COUNT);

    while (angle > 180.0f) angle -= 360.0f;
    while (angle <= -180.0f) angle += 360.0f;

    return angle;
}

float getXPositionCm()
{
    int16_t pulseRate = 0;
    pcnt_get_counter_value(PCNT_X_UNIT, &pulseRate);
    pcnt_counter_clear(PCNT_X_UNIT);

    totalXCount += pulseRate;

    return (float)totalXCount * CM_PER_COUNT;
}

float deg2rad(float deg)
{
    return deg * 3.14159265358979323846f / 180.0f;
}

/* ------------------------------------------------------------
    ADRC PARAMETERS (TUNING REQUIRED)
    ------------------------------------------------------------ */
const float Ts = 0.005f; 
const float B0_HAT = 18.172f;

// ESO (Observer) Tuning
const float OMEGA_O = 100.0f;
const float L1 = 3.0f * OMEGA_O;
const float L2 = 3.0f * OMEGA_O * OMEGA_O;
const float L3 = OMEGA_O * OMEGA_O * OMEGA_O;

// Controller Tuning
const float OMEGA_C = 15.0f;
const float KP = OMEGA_C * OMEGA_C;
const float KD = 2.0f * OMEGA_C;

// ADRC State Variables
float x_adrc_hat[3] = {0.0f, 0.0f, 0.0f};

const float PHI_REF = 0.0f;
unsigned long last_time_us = 0;
float u_prev = 0.0f;

/* ------------------------------------------------------------
    MOTOR OUTPUT
    ------------------------------------------------------------ */
void setMotor(float u_norm)
{
    if (u_norm >  1.0f) u_norm =  1.0f;
    if (u_norm < -1.0f) u_norm = -1.0f;

    int dir = (u_norm >= 0.0f) ? HIGH : LOW;
    float duty = fabs(u_norm);

    // if (duty <= 0.2f && duty != 0.0f)
    //     duty = 0.2f;
    
    int pwmValue = (int)(duty * 255.0f);

    if (dir == HIGH)
    {
        ledcWrite(pwmChannelA, 0);
        ledcWrite(pwmChannelB, pwmValue);
    }
    else
    {
        ledcWrite(pwmChannelB, 0);
        ledcWrite(pwmChannelA, pwmValue);
    }
}

/* ------------------------------------------------------------
    SETUP
    ------------------------------------------------------------ */
void setup()
{
    Serial.begin(115200);

    // PWM Setup
    ledcSetup(pwmChannelA, frequency, resolution);
    ledcSetup(pwmChannelB, frequency, resolution);

    ledcAttachPin(PIN_PWM_A, pwmChannelA);
    ledcAttachPin(PIN_PWM_B, pwmChannelB);

    ledcWrite(pwmChannelA, 0);
    ledcWrite(pwmChannelB, 0);

    // Encoder init
    initPCNTUnit(PCNT_THETA_UNIT, ENC_THETA_A_GPIO, ENC_THETA_B_GPIO);
    initPCNTUnit(PCNT_X_UNIT, ENC_X_A_GPIO, ENC_X_B_GPIO);

    Serial.println("=============================================");
    Serial.println("SYSTEM READY: ADRC CONTROL MODE");
    Serial.printf("Ts = %.3f ms\n", Ts * 1000.0f);
    Serial.printf("Controller Wc = %.1f, Observer Wo = %.1f\n", OMEGA_C, OMEGA_O);
    Serial.println("=============================================");
}

/* ------------------------------------------------------------
    MAIN CONTROL LOOP
    ------------------------------------------------------------ */
void loop()
{
    unsigned long now_us = micros();

    if (now_us - last_time_us < (unsigned long)(Ts * 1e6)) return;

    float dt = (now_us - last_time_us) / 1e6; 
    last_time_us = now_us;

    float phi_meas = deg2rad(getAngleDeg());

    // ESO
    float e = phi_meas - x_adrc_hat[0];

    float d_phi_hat      = x_adrc_hat[1] + L1 * e;
    float d_phidot_hat   = x_adrc_hat[2] + B0_HAT * u_prev + L2 * e;
    float d_f_hat        = L3 * e;

    x_adrc_hat[0] += d_phi_hat * dt;
    x_adrc_hat[1] += d_phidot_hat * dt;
    x_adrc_hat[2] += d_f_hat * dt;

    // Control Law
    float u0 = KP * (PHI_REF - x_adrc_hat[0]) + KD * (-x_adrc_hat[1]);
    float u  = (u0 - x_adrc_hat[2]) / B0_HAT;

    float u_norm = u * 0.8f;
    u_prev = u;

    setMotor(u_norm);

    // Serial Output
    Serial.print("phi=");
    Serial.print(phi_meas);
    Serial.print("  phi_hat=");
    Serial.print(x_adrc_hat[0]);
    Serial.print("  f_hat=");
    Serial.print(x_adrc_hat[2]);
    Serial.print("  u_norm=");
    Serial.println(u_norm);
}

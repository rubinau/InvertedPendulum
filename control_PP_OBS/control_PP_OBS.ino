/* ============================================================
   INVERTED PENDULUM - OBSERVER-BASED STATE FEEDBACK CONTROL
   ------------------------------------------------------------
   Measured: x (cart pos), phi (angle)
   Estimated: x_hat, xdot_hat, phi_hat, phidot_hat via observer
   Control:  u = -K * (x_hat - x_ref)
   Output:   Motor PWM and direction pin
   ============================================================ */

#include <Arduino.h>
#include "driver/pcnt.h"
#include "esp32-hal-ledc.h"

// =========================================================
// 1. KONFIGURASI ENCODER SUDUT (THETA) - PCNT UNIT 0
// =========================================================
#define ENC_THETA_A_GPIO 21       // Pin A Encoder Sudut
#define ENC_THETA_B_GPIO 22       // Pin B Encoder Sudut
#define PIN_PWM_A 18  // Input A L298N
#define PIN_PWM_B 19  // Input B L298N
#define PCNT_THETA_UNIT  PCNT_UNIT_0 // Menggunakan Unit 0

// Konfigurasi Parameter PWM
const int frequency = 30000;    // Frekuensi 30 kHz (di luar rentang pendengaran manusia)
const int resolution = 8;       // Resolusi 8-bit (0-255)

// Parameter Fisik Encoder Sudut
const float THETA_PPR         = 1000.0f;
const float THETA_QUAD_MULT   = 4.0f;
const float THETA_CPR         = THETA_PPR * THETA_QUAD_MULT;
const float DEG_PER_COUNT     = 360.0f / THETA_CPR;
const float START_ANGLE_DEG   = 0.0f;
const float START_X_CM   = 0.15f;

// Akumulator Global Sudut (64-bit)
int64_t totalThetaCount = 0;

// =========================================================
// 2. KONFIGURASI ENCODER POSISI (X) - PCNT UNIT 1
// =========================================================
#define ENC_X_A_GPIO     26       // Pin A Encoder Posisi (VSPI)
#define ENC_X_B_GPIO     25       // Pin B Encoder Posisi (VSPI)
#define PCNT_X_UNIT      PCNT_UNIT_1 // Menggunakan Unit 1 (BEDA UNIT)

// Parameter Fisik Encoder Posisi & Mekanik
const float X_PPR             = 600.0f;
const float X_QUAD_MULT       = 4.0f;
const float X_CPR             = X_PPR * X_QUAD_MULT;
const float PULLEY_RADIUS_CM  = 1.6f; 
const float CM_PER_REV        = 2.0f * PI * PULLEY_RADIUS_CM;
const float CM_PER_COUNT      = CM_PER_REV / X_CPR;

// Akumulator Global Posisi (64-bit)
int64_t totalXCount = 0;

// Batas Register Hardware (Sama untuk kedua unit)
#define PCNT_H_LIM       30000
#define PCNT_L_LIM      -30000

// =========================================================
// FUNGSI INISIALISASI PCNT GENERIC
// =========================================================
// Fungsi ini bersifat umum untuk mengonfigurasi Unit PCNT mana saja
// dengan parameter pin dan unit yang spesifik.
void initPCNTUnit(pcnt_unit_t unit, int pinA, int pinB) {
    pcnt_config_t pcnt_config = { };

    // --- Konfigurasi Channel 0 (Sinyal Utama: Pin A) ---
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

    // --- Konfigurasi Channel 1 (Sinyal Utama: Pin B) ---
    pcnt_config.pulse_gpio_num = pinB;
    pcnt_config.ctrl_gpio_num  = pinA;
    pcnt_config.channel        = PCNT_CHANNEL_1;
    pcnt_config.unit           = unit; // Unit yang sama untuk Quadrature decoding
    pcnt_config.pos_mode       = PCNT_COUNT_INC; 
    pcnt_config.neg_mode       = PCNT_COUNT_DEC; 
    pcnt_config.lctrl_mode     = PCNT_MODE_REVERSE; 
    pcnt_config.hctrl_mode     = PCNT_MODE_KEEP;    
    pcnt_unit_config(&pcnt_config);

    // --- Filter & Start ---
    // Filter glitch 1000 clock cycles (aprox 12.5us @ 80MHz APB)
    pcnt_set_filter_value(unit, 1000);
    pcnt_filter_enable(unit);

    pcnt_counter_pause(unit);
    pcnt_counter_clear(unit);
    pcnt_counter_resume(unit);
}

// =========================================================
// FUNGSI PEMBACAAN DATA (READ & ACCUMULATE)
// =========================================================

// Pembacaan Sudut Theta (Unit 0)
float getAngleDeg() {
    int16_t pulseRate = 0;
    
    // Baca dan bersihkan register hardware Unit 0
    pcnt_get_counter_value(PCNT_THETA_UNIT, &pulseRate);
    pcnt_counter_clear(PCNT_THETA_UNIT);
    
    // Akumulasi ke variabel software 64-bit
    totalThetaCount += pulseRate;

    // Konversi ke Derajat
    float angle = START_ANGLE_DEG + ((float)totalThetaCount * DEG_PER_COUNT);

    // Normalisasi (-180 s.d 180)
    while (angle > 180.0f)  angle -= 360.0f;
    while (angle <= -180.0f) angle += 360.0f;

    return angle;
}

// Pembacaan Posisi X (Unit 1)
float getXPositionCm() {
    int16_t pulseRate = 0;

    // Baca dan bersihkan register hardware Unit 1
    pcnt_get_counter_value(PCNT_X_UNIT, &pulseRate);
    pcnt_counter_clear(PCNT_X_UNIT);

    // Akumulasi ke variabel software 64-bit
    totalXCount += pulseRate;

    // Konversi ke Centimeter
    float x_pos = (float)totalXCount * CM_PER_COUNT;

    return x_pos;
}

float deg2rad(float deg) {
  return deg * 3.14159265358979323846f / 180.0f;
}

/* ------------------------------------------------------------
   SYSTEM MATRICES (DISCRETE-TIME)
   ------------------------------------------------------------ */

// A matrix
const float A_mat[4][4] = {
  {1.0f,        0.004992f,    1.052e-05f,  1.754e-08f},
  {0.0f,        0.9968f,      0.004207f,   1.052e-05f},
  {0.0f,       -2.273e-05f,   1.0f,        0.005001f},
  {0.0f,       -0.009086f,    0.152f,      1.0f}
};

// B matrix
const float B_mat[4] = {
  {7.953e-05f},
  {0.0318f},
  {0.0002273f},
  {0.09086f}
};

// C matrix
const float C_mat[2][4] = {
  {1.0f, 0.0f, 0.0f, 0.0f},   // x
  {0.0f, 0.0f, 1.0f, 0.0f}    // phi
};

/* ------------------------------------------------------------
   STATE FEEDBACK GAIN K (FROM YOUR MATLAB pole placement)
   ------------------------------------------------------------ */
float K[4] = {
  -12.8173f,
  -8.6306f,
  26.3499f,    
  5.0342f
};

/* ------------------------------------------------------------
   OBSERVER GAIN L (FROM MATLAB place(A',C',obs_poles)')
   ------------------------------------------------------------ */
float L[4][2] = {
  { 0.0752f, -0.0154f },
  { 0.1661f, -0.1093f },
  { -0.0218f, 0.1111f },
  { -0.2340f, 0.6564f }
};

/* ------------------------------------------------------------
   GLOBAL VARIABLES
   ------------------------------------------------------------ */
volatile long enc1_count = 0;
volatile long enc2_count = 0;

float Ts = 0.005f;          // 5ms loop time
unsigned long last_time_us = 0;

float x_hat[4] = {0, 0, 0, 0};
float u_prev = 0.0f;
float xprev;
float phiprev;

/* ------------------------------------------------------------
   MOTOR OUTPUT
   ------------------------------------------------------------ */
void setMotor(float u_norm)
{
    if (u_norm >  1.0f) u_norm =  1.0f;
    if (u_norm < -1.0f) u_norm = -1.0f;

    int dir = (u_norm >= 0.0f) ? HIGH : LOW;
    float duty = fabs(u_norm);
    
    if(dir == HIGH){
      ledcWrite(PIN_PWM_A, 0);
      ledcWrite(PIN_PWM_B, (int)(duty * 255.0f));
    }
    else{
      ledcWrite(PIN_PWM_B, 0);
      ledcWrite(PIN_PWM_A, (int)(duty * 255.0f));

    }
}

/* ------------------------------------------------------------
   SETUP
   ------------------------------------------------------------ */
void setup() {
    Serial.begin(115200);

    // Menghubungkan Kanal ke Pin Fisik
    ledcAttach(PIN_PWM_A, frequency, resolution);
    ledcAttach(PIN_PWM_B, frequency, resolution);

    // Memastikan kondisi awal motor adalah berhenti (Duty Cycle 0)
    ledcWrite(PIN_PWM_A, 0);
    ledcWrite(PIN_PWM_B, 0);

    // Inisialisasi Encoder Sudut pada Unit 0
    initPCNTUnit(PCNT_THETA_UNIT, ENC_THETA_A_GPIO, ENC_THETA_B_GPIO);
    
    // Inisialisasi Encoder Posisi pada Unit 1
    initPCNTUnit(PCNT_X_UNIT, ENC_X_A_GPIO, ENC_X_B_GPIO);

    Serial.println("=============================================");
    Serial.println("SYSTEM READY: Dual PCNT Encoder Initialized");
    Serial.printf("Theta Res : %.6f deg/count (Unit %d)\n", DEG_PER_COUNT, PCNT_THETA_UNIT);
    Serial.printf("Pos X Res : %.6f cm/count  (Unit %d)\n", CM_PER_COUNT, PCNT_X_UNIT);
    Serial.println("=============================================");
}

/* ------------------------------------------------------------
   MAIN CONTROL LOOP
   ------------------------------------------------------------ */
void loop() {
unsigned long now_us = micros();
if (now_us - last_time_us < (unsigned long)(Ts * 1e6)) return;
last_time_us = now_us;

/* --- 1. Read encoders --- */
float x   = getXPositionCm() * 0.01f;
// float dx = (x-xprev)/Ts;
// xprev = x;
float phi = deg2rad(getAngleDeg());
// float dphi = (phi-phiprev)/Ts;
// phiprev = phi;


float y[2] = { x, phi };

/* --- 2. Observer update (DISCRETE) --- */

// y_hat = C*x_hat
float y_hat[2] = {0,0};
y_hat[0] = x_hat[0];  // because C selects x
y_hat[1] = x_hat[2];  // because C selects phi

// innovation e = y - y_hat
float e[2];
e[0] = y[0] - y_hat[0];
e[1] = y[1] - y_hat[1];

// compute control u
float u = 0;
u = -(K[0]*x_hat[0]
      +K[1]*x_hat[1]
      +K[2]*x_hat[2]
      +K[3]*x_hat[3]);

// Observer Update
float x_next[4];

  x_next[0] =  A_mat[0][0]*x_hat[0] + A_mat[0][1]*x_hat[1] +
               A_mat[0][2]*x_hat[2] + A_mat[0][3]*x_hat[3]
               + B_mat[0]*u + (L[0][0]*e[0] + L[0][1]*e[1]);

  x_next[1] =  A_mat[1][0]*x_hat[0] + A_mat[1][1]*x_hat[1] +
               A_mat[1][2]*x_hat[2] + A_mat[1][3]*x_hat[3]
               + B_mat[1]*u + (L[1][0]*e[0] + L[1][1]*e[1]);

  x_next[2] =  A_mat[2][0]*x_hat[0] + A_mat[2][1]*x_hat[1] +
               A_mat[2][2]*x_hat[2] + A_mat[2][3]*x_hat[3]
               + B_mat[2]*u + (L[2][0]*e[0] + L[2][1]*e[1]);

  x_next[3] =  A_mat[3][0]*x_hat[0] + A_mat[3][1]*x_hat[1] +
               A_mat[3][2]*x_hat[2] + A_mat[3][3]*x_hat[3]
               + B_mat[3]*u + (L[3][0]*e[0] + L[3][1]*e[1]);


  /* ---- overwrite ---- */
  for(int i=0;i<4;i++)
    x_hat[i] = x_next[i];

/* scale for motor */
if (x > 0.13f)    u -= 5.0f*(x - 0.13);
if (x < -0.13f)   u -= 5.0f*(x + 0.13f);

float u_norm = u * 0.4f;

// store PREVIOUS CONTROL INPUT = u , NOT pwm!
u_prev = u;

// send to motor
setMotor(u_norm);

/* --- debug --- */
Serial.print("x=");
Serial.print(x);
Serial.print("  phi=");
Serial.print(phi);
Serial.print("  u=");
Serial.println(u_norm);

}

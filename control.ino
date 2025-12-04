/* ============================================================
   INVERTED PENDULUM - OBSERVER-BASED STATE FEEDBACK CONTROL
   ------------------------------------------------------------
   Measured: x (cart pos), phi (angle)
   Estimated: x_hat, xdot_hat, phi_hat, phidot_hat via observer
   Control:  u = -K * (x_hat - x_ref)
   Output:   Motor PWM and direction pin
   ============================================================ */

#include <Arduino.h>

/* ------------------------------------------------------------
   ENCODER PINS (CHANGE TO YOUR SETUP)
   ------------------------------------------------------------ */
const int enc1_A = 2;   // cart encoder
const int enc1_B = 3;

const int enc2_A = 4;   // pendulum encoder
const int enc2_B = 5;

/* ------------------------------------------------------------
   MOTOR DRIVER PINS
   ------------------------------------------------------------ */
const int PWM_PIN = 9;
const int DIR_PIN = 8;

/* ------------------------------------------------------------
   ENCODER & MECHANICAL PARAMETERS
   ------------------------------------------------------------ */
const float ENC1_CPR = 1024.0f;       // cart encoder CPR
const float ENC2_CPR = 1024.0f;       // pendulum encoder CPR

const float WHEEL_RADIUS = 0.02f;     // 2 cm
const float GEAR_RATIO   = 1.0f;      // direct drive

const float ENC2_TO_RAD = 2.0f * PI / ENC2_CPR;

/* ------------------------------------------------------------
   SYSTEM MATRICES (CONTINUOUS-TIME)
   ------------------------------------------------------------ */

// A matrix
const float A_mat[4][4] = {
  { 0.0f,        1.0f,        0.0f,       0.0f        },
  { 0.0f,       -0.6369427f,  0.8426752f, 0.0f        },
  { 0.0f,        0.0f,        0.0f,       1.0f        },
  { 0.0f,       -1.8198362f, 30.4076433f, 0.0f        }
};

// B matrix
const float B_mat[4] = {
  0.0f,
  6.3694268f,
  0.0f,
  18.1983621f
};

// C matrix
const float C_mat[2][4] = {
  { 1.0f, 0.0f, 0.0f, 0.0f },  // x
  { 0.0f, 0.0f, 1.0f, 0.0f }   // phi
};

/* ------------------------------------------------------------
   STATE FEEDBACK GAIN K (FROM YOUR MATLAB pole placement)
   ------------------------------------------------------------ */
float K[4] = {
  -4.70999469f,   // x
  -3.67735311f,   // x_dot
   13.15428331f,  // phi
    2.46066824f   // phi_dot
};

/* ------------------------------------------------------------
   OBSERVER GAIN L (FROM MATLAB place(A',C',obs_poles)')
   ------------------------------------------------------------ */
float L[4][2] = {
  { 43.74203846f,   3.84820935f },
  {452.35889510f,  82.98441431f },
  {  2.30030249f,  43.62106154f },
  { 11.13218361f, 487.18744050f }
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

/* ------------------------------------------------------------
   ENCODER INTERRUPTS
   ------------------------------------------------------------ */
void enc1A_ISR() { enc1_count += (digitalRead(enc1_A) == digitalRead(enc1_B)) ? 1 : -1; }
void enc1B_ISR() { enc1_count += (digitalRead(enc1_A) != digitalRead(enc1_B)) ? 1 : -1; }

void enc2A_ISR() { enc2_count += (digitalRead(enc2_A) == digitalRead(enc2_B)) ? 1 : -1; }
void enc2B_ISR() { enc2_count += (digitalRead(enc2_A) != digitalRead(enc2_B)) ? 1 : -1; }

/* ------------------------------------------------------------
   MOTOR OUTPUT
   ------------------------------------------------------------ */
void setMotor(float u_norm)
{
    if (u_norm >  1.0f) u_norm =  1.0f;
    if (u_norm < -1.0f) u_norm = -1.0f;

    int dir = (u_norm >= 0.0f) ? HIGH : LOW;
    float duty = fabs(u_norm);

    analogWrite(PWM_PIN, (int)(duty * 255.0f));
    digitalWrite(DIR_PIN, dir);
}

/* ------------------------------------------------------------
   SETUP
   ------------------------------------------------------------ */
void setup() {
    Serial.begin(115200);

    pinMode(PWM_PIN, OUTPUT);
    pinMode(DIR_PIN, OUTPUT);

    pinMode(enc1_A, INPUT_PULLUP);
    pinMode(enc1_B, INPUT_PULLUP);
    pinMode(enc2_A, INPUT_PULLUP);
    pinMode(enc2_B, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(enc1_A), enc1A_ISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(enc1_B), enc1B_ISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(enc2_A), enc2A_ISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(enc2_B), enc2B_ISR, CHANGE);

    last_time_us = micros();
}

/* ------------------------------------------------------------
   MAIN CONTROL LOOP
   ------------------------------------------------------------ */
void loop() {
    unsigned long now_us = micros();
    if (now_us - last_time_us < (unsigned long)(Ts * 1e6)) return;
    last_time_us = now_us;

    /* --- 1. Read encoders --- */
    long c1 = enc1_count;
    long c2 = enc2_count;

    float x = (float)c1 / ENC1_CPR * 2.0f * PI * WHEEL_RADIUS / GEAR_RATIO;
    float phi = (float)c2 * ENC2_TO_RAD;

    float y[2] = { x, phi };

    /* --- 2. Observer update --- */

    // y_hat = C * x_hat
    float y_hat[2] = {0,0};
    for (int i = 0; i < 2; i++)
      for (int j = 0; j < 4; j++)
        y_hat[i] += C_mat[i][j] * x_hat[j];

    // innovation
    float e[2] = { y[0] - y_hat[0], y[1] - y_hat[1] };

    // x_hat_dot = A*x_hat + B*u_prev + L*e
    float x_hat_dot[4] = {0,0,0,0};

    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 4; j++)
            x_hat_dot[i] += A_mat[i][j] * x_hat[j];

        x_hat_dot[i] += B_mat[i] * u_prev;
        x_hat_dot[i] += L[i][0] * e[0] + L[i][1] * e[1];
    }

    // integrate
    for (int i = 0; i < 4; i++)
        x_hat[i] += Ts * x_hat_dot[i];

    /* --- 3. Control: u = -K*(x_hat - x_ref) --- */

    float x_ref = 0.0f;
    float phi_ref = 0.0f;

    float err[4] = {
      x_hat[0] - x_ref,
      x_hat[1],
      x_hat[2] - phi_ref,
      x_hat[3]
    };

    float u = 0;
    for (int i = 0; i < 4; i++)
        u -= K[i] * err[i];

    /* scale down to PWM range */
    float u_norm = u * 0.05f;
    u_prev = u;

    setMotor(u_norm);

    /* --- 4. Debugging --- */
    Serial.print("x=");
    Serial.print(x);
    Serial.print("  phi=");
    Serial.print(phi);
    Serial.print("  u_norm=");
    Serial.println(u_norm);
}

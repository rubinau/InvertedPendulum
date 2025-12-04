/* ============================================================
   TEST 3 — CHECK MOTOR PWM + DIR SIGNALS
   Sweeps PWM and toggles direction every few seconds
   ============================================================ */

const int PWM_PIN = 9;
const int DIR_PIN = 8;

void setup() {
    Serial.begin(115200);
    pinMode(PWM_PIN, OUTPUT);
    pinMode(DIR_PIN, OUTPUT);

    Serial.println("PWM + DIR Test Started...");
}

void loop() {

    // Forward direction
    digitalWrite(DIR_PIN, HIGH);
    Serial.println("Direction: FORWARD");

    for (int duty = 0; duty <= 255; duty += 5) {
        analogWrite(PWM_PIN, duty);
        Serial.print("PWM = "); Serial.println(duty);
        delay(20);
    }

    delay(1000);

    // Reverse direction
    digitalWrite(DIR_PIN, LOW);
    Serial.println("Direction: REVERSE");

    for (int duty = 0; duty <= 255; duty += 5) {
        analogWrite(PWM_PIN, duty);
        Serial.print("PWM = "); Serial.println(duty);
        delay(20);
    }

    delay(1000);
}

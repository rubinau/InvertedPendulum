/* ============================================================
   TEST 1 — CHECK ENCODER #1 ONLY
   Reads encoder 1 and prints count + position (in meters)
   ============================================================ */

const int ENCA = 2;
const int ENCB = 3;

volatile long count1 = 0;

const float CPR = 1024.0;
const float WHEEL_R = 0.02;  // adjust for your wheel
const float GEAR = 1.0;

void IRAM_ATTR readA() {
    count1 += (digitalRead(ENCA) == digitalRead(ENCB)) ? 1 : -1;
}

void IRAM_ATTR readB() {
    count1 += (digitalRead(ENCA) != digitalRead(ENCB)) ? 1 : -1;
}

void setup() {
    Serial.begin(115200);

    pinMode(ENCA, INPUT_PULLUP);
    pinMode(ENCB, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(ENCA), readA, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCB), readB, CHANGE);

    Serial.println("Encoder #1 Test Started...");
}

void loop() {
    long c = count1;

    float position = c / CPR * 2.0 * PI * WHEEL_R / GEAR;  // meters

    Serial.print("Count1 = ");
    Serial.print(c);
    Serial.print("   Position = ");
    Serial.print(position);
    Serial.println(" m");

    delay(50);
}

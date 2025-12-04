/* ============================================================
   TEST 2 — CHECK TWO ENCODERS
   Reads both encoder #1 (cart) and encoder #2 (pendulum)
   ============================================================ */

const int EN1A = 2;
const int EN1B = 3;
const int EN2A = 4;
const int EN2B = 5;

volatile long count1 = 0;
volatile long count2 = 0;

void IRAM_ATTR read1A() { count1 += (digitalRead(EN1A) == digitalRead(EN1B)) ? 1 : -1; }
void IRAM_ATTR read1B() { count1 += (digitalRead(EN1A) != digitalRead(EN1B)) ? 1 : -1; }

void IRAM_ATTR read2A() { count2 += (digitalRead(EN2A) == digitalRead(EN2B)) ? 1 : -1; }
void IRAM_ATTR read2B() { count2 += (digitalRead(EN2A) != digitalRead(EN2B)) ? 1 : -1; }

void setup() {
    Serial.begin(115200);

    pinMode(EN1A, INPUT_PULLUP);
    pinMode(EN1B, INPUT_PULLUP);
    pinMode(EN2A, INPUT_PULLUP);
    pinMode(EN2B, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(EN1A), read1A, CHANGE);
    attachInterrupt(digitalPinToInterrupt(EN1B), read1B, CHANGE);
    attachInterrupt(digitalPinToInterrupt(EN2A), read2A, CHANGE);
    attachInterrupt(digitalPinToInterrupt(EN2B), read2B, CHANGE);

    Serial.println("Dual Encoder Test Started...");
}

void loop() {
    long c1 = count1;
    long c2 = count2;

    Serial.print("Enc1: ");
    Serial.print(c1);
    Serial.print("   Enc2: ");
    Serial.println(c2);

    delay(50);
}

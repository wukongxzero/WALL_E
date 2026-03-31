/*
================================================================
  WALL-E MEGA — FINAL
  Encoders wired to UNO not Mega.
  ENA=RIGHT motor, ENB=LEFT motor (from PS2 code)
================================================================
*/

#include <avr/wdt.h>

#define ENA   5
#define IN1   23
#define IN2   22
#define ENB   6
#define IN3   25
#define IN4   24
#define MAX_PWM  200
#define DEADZONE 12
#define TANKSTATUS_PACKET_LENGTH 16
#define LOOP_MS 5

int8_t driveLeft  = 0;
int8_t driveRight = 0;
float  pitchDeg   = 0.0f;

uint8_t btBuf[TANKSTATUS_PACKET_LENGTH];
uint8_t btIdx  = 0;
String  unoLine = "";
unsigned long lastLoop = 0;

int clampi(int x, int lo, int hi) {
    return x < lo ? lo : (x > hi ? hi : x);
}

void setMotor(int en, int a, int b, int signedSpeed) {
    int pwm = clampi(abs(signedSpeed), 0, MAX_PWM);
    if (pwm == 0) {
        digitalWrite(a,LOW); digitalWrite(b,LOW);
        analogWrite(en,0); return;
    }
    digitalWrite(a, signedSpeed>0 ? HIGH : LOW);
    digitalWrite(b, signedSpeed>0 ? LOW  : HIGH);
    analogWrite(en, pwm);
}

void stopMotors() {
    setMotor(ENA,IN1,IN2,0);
    setMotor(ENB,IN3,IN4,0);
}

void parseTankStatus(uint8_t* buf) {
    driveLeft  = (int8_t)buf[0];
    driveRight = (int8_t)buf[1];
}

void sendTankStatus(float eulerX, float eulerY, float eulerZ) {
    uint8_t buf[TANKSTATUS_PACKET_LENGTH];
    memset(buf, 0, TANKSTATUS_PACKET_LENGTH);
    buf[0] = (uint8_t)(int8_t)driveLeft;
    buf[1] = (uint8_t)(int8_t)driveRight;
    memcpy(buf+4,  &eulerX, 4);
    memcpy(buf+8,  &eulerY, 4);
    memcpy(buf+12, &eulerZ, 4);
    Serial1.write(buf, TANKSTATUS_PACKET_LENGTH);
}

void parseUnoLine(String line) {
    line.trim();
    if (line.startsWith("P:"))
        pitchDeg = line.substring(2).toFloat();
}

void setup() {
    if (MCUSR&(1<<WDRF)) MCUSR=0;
    wdt_disable();
    wdt_enable(WDTO_250MS);

    Serial.begin(115200);
    Serial1.begin(9600);
    Serial2.begin(115200);

    pinMode(ENA,OUTPUT); pinMode(IN1,OUTPUT); pinMode(IN2,OUTPUT);
    pinMode(ENB,OUTPUT); pinMode(IN3,OUTPUT); pinMode(IN4,OUTPUT);
    stopMotors();

    Serial.println("WALL-E MEGA ready");
    lastLoop = millis();
}

void loop() {
    wdt_reset();

    while (Serial1.available()) {
        uint8_t c = Serial1.read();
        btBuf[btIdx++] = c;
        if (btIdx >= TANKSTATUS_PACKET_LENGTH) {
            parseTankStatus(btBuf);
            btIdx = 0;
        }
    }

    while (Serial2.available()) {
        char c = (char)Serial2.read();
        if (c=='\n') { parseUnoLine(unoLine); unoLine=""; }
        else unoLine += c;
    }

    unsigned long now = millis();
    if ((now-lastLoop) < LOOP_MS) return;
    lastLoop = now;

    int leftPWM  = (int)driveLeft  * 2;
    int rightPWM = (int)driveRight * 2;
    if (abs(leftPWM)  < DEADZONE) leftPWM  = 0;
    if (abs(rightPWM) < DEADZONE) rightPWM = 0;

    setMotor(ENA, IN1, IN2, rightPWM);
    setMotor(ENB, IN3, IN4, leftPWM);

    static int telCount = 0;
    if (++telCount >= 10) {
        telCount = 0;
        sendTankStatus(pitchDeg, 0.0f, 90.0f);
    }

    static int dbgCount = 0;
    if (++dbgCount >= 100) {
        dbgCount = 0;
        Serial.print("pitch="); Serial.print(pitchDeg,2);
        Serial.print(" L=");    Serial.print((int)driveLeft);
        Serial.print(" R=");    Serial.println((int)driveRight);
    }
}
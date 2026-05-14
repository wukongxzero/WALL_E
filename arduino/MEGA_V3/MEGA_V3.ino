#include <avr/wdt.h>
#include <Wire.h>
#include <math.h>
#include <string.h>
#include "src/TankStatusClass/TankStatusClass.h"

#define MAX_VELOCITY 4095
bool motorsHalted = false;

// ── TCA9548A + AS5600 ─────────────────────────────────────────
#define TCA_ADDR      0x70
#define ENC_ADDR      0x36
#define LEFT_CHANNEL  0
#define RIGHT_CHANNEL 1

bool tcaSelect(uint8_t ch) {
    Wire.beginTransmission(TCA_ADDR);
    Wire.write(1 << ch);
    return Wire.endTransmission() == 0;
}

uint16_t readAngle(uint8_t ch) {
    if (!tcaSelect(ch)) return 0;
    Wire.beginTransmission(ENC_ADDR);
    Wire.write(0x0C);
    if (Wire.endTransmission(false) != 0) return 0;
    if (Wire.requestFrom(ENC_ADDR, (uint8_t)2) != 2) return 0;
    uint16_t high = Wire.read();
    uint16_t low  = Wire.read();
    return ((high << 8) | low) & 0x0FFF;
}

// ── MOTOR PINS ────────────────────────────────────────────────
#define ENA     5
#define IN1     27
#define IN2     26
#define ENB     6
#define IN3     29
#define IN4     28
#define MAX_PWM 250
#define LOOP_MS 5
#define FLIP_THRESHOLD 150

// Ramp config — PWM counts allowed to change per LOOP_MS tick.
// 5 counts / 5 ms = 1000 counts/s → full 0→250 ramp in ~250 ms.
// Lower = smoother/slower, higher = snappier.
#define MAX_PWM_DELTA 5

// ── ODOMETRY ──────────────────────────────────────────────────
#define WHEEL_RADIUS 0.05f
#define CPR          4096.0f

uint16_t leftAnglePrev  = 0;
uint16_t rightAnglePrev = 0;
unsigned long lastEncTime = 0;

TankStatusClass tsLocalIn  = TankStatusClass();
TankStatusClass tsLocalOut = TankStatusClass();

int16_t angleDiff(uint16_t curr, uint16_t prev) {
    int16_t diff = (int16_t)curr - (int16_t)prev;
    if (diff >  2048) diff -= 4096;
    if (diff < -2048) diff += 4096;
    return diff;
}

void updateOdometry() {
    unsigned long now = millis();
    float dt = (now - lastEncTime) / 1000.0f;
    if (dt < 0.01f) return;
    lastEncTime = now;

    uint16_t leftAngle  = readAngle(LEFT_CHANNEL);
    uint16_t rightAngle = readAngle(RIGHT_CHANNEL);

    int16_t leftDiff  = angleDiff(leftAngle,  leftAnglePrev);
    int16_t rightDiff = angleDiff(rightAngle, rightAnglePrev);

    leftAnglePrev  = leftAngle;
    rightAnglePrev = rightAngle;

    float leftDist  = (leftDiff  / CPR) * 2.0f * M_PI * WHEEL_RADIUS;
    float rightDist = (rightDiff / CPR) * 2.0f * M_PI * WHEEL_RADIUS;

    int leftVel  = leftDist  / dt;
    int rightVel = rightDist / dt;

    tsLocalOut.driveLeft  = static_cast<unsigned char>(map(leftVel,   0, MAX_VELOCITY, 0, MAX_PWM));
    tsLocalOut.driveRight = static_cast<unsigned char>(map(rightDiff, 0, MAX_VELOCITY, 0, MAX_PWM));
}

// ── MOTOR DRIVER ──────────────────────────────────────────────
int clampi(int x, int lo, int hi) {
    return x < lo ? lo : (x > hi ? hi : x);
}

void setMotor(int en, int a, int b, int spd) {
    int pwm = clampi(abs(spd), 0, MAX_PWM);
    if (pwm == 0) {
        digitalWrite(a, LOW); digitalWrite(b, LOW);
        analogWrite(en, 0); return;
    }
    digitalWrite(a, spd > 0 ? HIGH : LOW);
    digitalWrite(b, spd > 0 ? LOW  : HIGH);
    analogWrite(en, pwm);
}

// ── STATE ─────────────────────────────────────────────────────
TankStatusClass rxStatus;
TankStatusClass txStatus;

uint8_t  pktBuf[TANKSTATUS_PACKET_LENGTH];
uint8_t  pktIdx = 0;
int lastLeftRaw     = 0;
int lastRightRaw    = 0;

int currentLeftPWM  = 0;   // PWM actually applied (ramped)
int currentRightPWM = 0;
int targetLeftPWM   = 0;   // PWM requested by driveMotors()
int targetRightPWM  = 0;

unsigned long lastLoop  = 0;
unsigned long lastPktMs = 0;
#define PKT_TIMEOUT_MS 500

void stopMotors() {
    targetLeftPWM   = 0;
    targetRightPWM  = 0;
    currentLeftPWM  = 0;
    currentRightPWM = 0;
    setMotor(ENA, IN1, IN2, 0);
    setMotor(ENB, IN3, IN4, 0);
}

// ── RAMP ──────────────────────────────────────────────────────
static inline int rampToward(int current, int target, int maxStep) {
    int diff = target - current;
    if (diff >  maxStep) return current + maxStep;
    if (diff < -maxStep) return current - maxStep;
    return target;
}

void updateMotorRamp() {
    currentLeftPWM  = rampToward(currentLeftPWM,  targetLeftPWM,  MAX_PWM_DELTA);
    currentRightPWM = rampToward(currentRightPWM, targetRightPWM, MAX_PWM_DELTA);
    setMotor(ENA, IN1, IN2, -currentLeftPWM);   // sign matches your wiring
    setMotor(ENB, IN3, IN4,  currentRightPWM);
}

// Called whenever a new TankStatus packet arrives.
// Only sets targets — actual PWM is slewed in updateMotorRamp().
void driveMotors(unsigned char leftIn, unsigned char rightIn) {
    int leftSpd  = (int)leftIn  - 127;
    int rightSpd = (int)rightIn - 127;
    targetLeftPWM  = map(leftSpd,  -127, 127, -MAX_PWM, MAX_PWM);
    targetRightPWM = map(rightSpd, -127, 127, -MAX_PWM, MAX_PWM);
}

// ── SETUP ─────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);

    Wire.begin();
    Wire.setClock(400000);
    Wire.setWireTimeout(3000, true);

    pinMode(ENA,OUTPUT); pinMode(IN1,OUTPUT); pinMode(IN2,OUTPUT);
    pinMode(ENB,OUTPUT); pinMode(IN3,OUTPUT); pinMode(IN4,OUTPUT);
    stopMotors();

    leftAnglePrev  = readAngle(LEFT_CHANNEL);
    rightAnglePrev = readAngle(RIGHT_CHANNEL);

    lastEncTime = millis();
    lastPktMs   = millis();
    lastLoop    = millis();

    /*
    if (MCUSR & (1 << WDRF)) MCUSR = 0;
    wdt_disable();
    wdt_enable(WDTO_1S);
    */

    tsLocalIn.driveLeft   = 127;
    tsLocalIn.driveRight  = 127;
    tsLocalOut.driveLeft  = 127;
    tsLocalOut.driveRight = 127;

    driveMotors(127, 127);   // targets at neutral, ramp idle at 0
}

// ── LOOP ──────────────────────────────────────────────────────
void loop() {
    //wdt_reset();

    while (Serial.available()) {
        uint8_t c = Serial.read();
        pktBuf[pktIdx++] = c;
        if (pktIdx >= TANKSTATUS_PACKET_LENGTH) {
            tsLocalIn.BuildFromBytes(pktBuf);
            pktIdx    = 0;
            lastPktMs = millis();
            driveMotors(tsLocalIn.driveLeft, tsLocalIn.driveRight);
        }
    }
/*
    if ((millis() - lastPktMs) > PKT_TIMEOUT_MS) {
        stopMotors();
        motorsHalted = true;
    } else {
        motorsHalted = false;
    }
*/

    unsigned long now = millis();
    if ((now - lastLoop) < LOOP_MS) return;
    lastLoop = now;

    updateMotorRamp();
    updateOdometry();

    static int odomCount = 0;
    if (++odomCount >= 10) {
        odomCount = 0;
        unsigned char* txBuffer = tsLocalIn.MakeIntoBytes();
        Serial.write(txBuffer, TANKSTATUS_PACKET_LENGTH);
    }
}
#include <avr/wdt.h>
#include <Wire.h>
#include <math.h>
#include <string.h>
#include "src/TankStatusClass/TankStatusClass.h"

bool motorsHalted = false;

// ── TCA9548A + AS5600 ─────────────────────────────────────────
#define TCA_ADDR      0x70
#define ENC_ADDR      0x36
#define LEFT_CHANNEL  0
#define RIGHT_CHANNEL 1

void tcaSelect(uint8_t ch) {
    Wire.beginTransmission(TCA_ADDR);
    Wire.write(1 << ch);
    Wire.endTransmission();
}

uint16_t readAngle(uint8_t ch) {
    tcaSelect(ch);
    Wire.beginTransmission(ENC_ADDR);
    Wire.write(0x0C);
    Wire.endTransmission(false);
    Wire.requestFrom(ENC_ADDR, (uint8_t)2);
    uint16_t high = Wire.read();
    uint16_t low  = Wire.read();
    return ((high << 8) | low) & 0x0FFF;
}

// ── MOTOR PINS ────────────────────────────────────────────────
#define ENA     5
#define IN1     23
#define IN2     22
#define ENB     6
#define IN3     25
#define IN4     24
#define MAX_PWM 200
#define LOOP_MS 5
#define FLIP_THRESHOLD 150

// ── ODOMETRY (Left/Right Velocity Only) ───────────────────────
#define WHEEL_RADIUS 0.05f
#define CPR          4096.0f

uint16_t leftAnglePrev  = 0;
uint16_t rightAnglePrev = 0;
unsigned long lastEncTime = 0;

float leftVel  = 0.0f;
float rightVel = 0.0f;

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

    // Read the current angles from the AS5600 sensors
    uint16_t leftAngle  = readAngle(LEFT_CHANNEL);
    uint16_t rightAngle = readAngle(RIGHT_CHANNEL);

    int16_t leftDiff  = angleDiff(leftAngle,  leftAnglePrev);
    int16_t rightDiff = angleDiff(rightAngle, rightAnglePrev);

    leftAnglePrev  = leftAngle;
    rightAnglePrev = rightAngle;

    float leftDist  = (leftDiff  / CPR) * 2.0f * M_PI * WHEEL_RADIUS;
    float rightDist = (rightDiff / CPR) * 2.0f * M_PI * WHEEL_RADIUS;

    // Calculate isolated Left and Right velocities
    leftVel  = leftDist  / dt;
    rightVel = rightDist / dt;
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

void stopMotors() {
    setMotor(ENA, IN1, IN2, 0);
    setMotor(ENB, IN3, IN4, 0);
}

// ── STATE ─────────────────────────────────────────────────────
TankStatusClass rxStatus; // Received packet instance (for driving)
TankStatusClass txStatus; // Transmit packet instance (for odometry)

uint8_t  pktBuf[TANKSTATUS_PACKET_LENGTH];
uint8_t  pktIdx     = 0;
int lastLeftRaw     = 0;
int lastRightRaw    = 0;
int currentLeftPWM  = 0;
int currentRightPWM = 0;
unsigned long lastLoop  = 0;
unsigned long lastPktMs = 0;
#define PKT_TIMEOUT_MS 500

// ── DRIVE FROM TANKSTATUS ──────────────────────────────────────
void driveMotors() {
    // Only pull driveRight and driveLeft from the receiving TankStatus wrapper
    int throttle = -((int)(*rxStatus.driveRight) - 127);
    int steering = -((int)(*rxStatus.driveLeft)  - 127);

    throttle = clampi(throttle, -124, 124);
    steering = clampi(steering, -124, 124);

    if (abs(throttle - lastLeftRaw)  > FLIP_THRESHOLD) throttle = lastLeftRaw;
    if (abs(steering - lastRightRaw) > FLIP_THRESHOLD) steering = lastRightRaw;

    lastLeftRaw  = throttle;
    lastRightRaw = steering;

    throttle = (throttle * 255) / 124;
    steering = (steering * 255) / 124;

    currentRightPWM = clampi(throttle + steering, -MAX_PWM, MAX_PWM);
    currentLeftPWM  = clampi(throttle - steering, -MAX_PWM, MAX_PWM);

    setMotor(ENA, IN1, IN2, currentRightPWM);
    setMotor(ENB, IN3, IN4, currentLeftPWM);
}

// ── SETUP ─────────────────────────────────────────────────────
void setup() {

    Serial.begin(115200);
    if (MCUSR & (1 << WDRF)) MCUSR = 0;
    wdt_disable();
    wdt_enable(WDTO_1S);


    Wire.begin();
    Wire.setClock(400000);

    pinMode(ENA,OUTPUT); pinMode(IN1,OUTPUT); pinMode(IN2,OUTPUT);
    pinMode(ENB,OUTPUT); pinMode(IN3,OUTPUT); pinMode(IN4,OUTPUT);
    stopMotors();

    // Initialize angles directly from sensor to prevent a massive jump on first loop
    leftAnglePrev  = readAngle(LEFT_CHANNEL);
    rightAnglePrev = readAngle(RIGHT_CHANNEL);
    
    lastEncTime    = millis();
    lastPktMs      = millis();
    lastLoop       = millis();
}

// ── LOOP ──────────────────────────────────────────────────────
void loop() {
    wdt_reset();
 
    // 1. Read incoming TankStatus packets to drive motors
    while (Serial.available()) {
        uint8_t c = Serial.read();
        pktBuf[pktIdx++] = c;
        if (pktIdx >= TANKSTATUS_PACKET_LENGTH) {
            rxStatus.BuildFromBytes(pktBuf);
            pktIdx    = 0;
            lastPktMs = millis();
        }
    }

    if ((millis() - lastPktMs) > PKT_TIMEOUT_MS) {
        stopMotors();
        motorsHalted = true;
    } else {
        motorsHalted = false;
    }

    // 2. Timing loop execution
    unsigned long now = millis();
    if ((now - lastLoop) < LOOP_MS) return;
    lastLoop = now;

    // 3. Update internal math and actuate motors
    updateOdometry();
    if (!motorsHalted) driveMotors();

    // 4. Stream Odometry back using MakeIntoBytes()
    static int odomCount = 0;
    if (++odomCount >= 10) {
        odomCount = 0;
        
        // Map the odometry to the unused floats in txStatus so they get encoded natively
        txStatus.eulerXFloat = leftVel;
        txStatus.eulerYFloat = rightVel;
        // txStatus.eulerZFloat = 0.0; // Unused, remains 0

        // Grab the byte array directly from the class instance
        unsigned char* txBuffer = txStatus.MakeIntoBytes();
        
        // Write the whole struct payload seamlessly
        Serial.write(txBuffer, TANKSTATUS_PACKET_LENGTH);
    }
}

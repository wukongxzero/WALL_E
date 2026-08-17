/*
================================================================
  WALL-E MEGA — V3
  Serial (115200) ↔ Jetson only

  IN:  8-byte TankStatus from Jetson → motors
  OUT: 20-byte odometry packet → Jetson (0xAA 0xBB header)

  TCA9548A + AS5600 encoders for odometry
================================================================
*/

#include <avr/wdt.h>
#include <Wire.h>
#include <math.h>
#include <string.h>

// ── TANKSTATUS (8 bytes, matches Parallax TankStatus.h) ───────
#define TANKSTATUS_PACKET_LENGTH 8


bool motorsHalted = false;


struct TankStatus {
    volatile unsigned char driveLeft;
    volatile unsigned char driveRight;
    volatile short eulerX;
    volatile short eulerY;
    volatile short eulerZ;
    volatile unsigned char changeFlag;
};

void constructTankStatus(struct TankStatus *self) {
    self->driveLeft  = 0;
    self->driveRight = 0;
    self->eulerX     = 0;
    self->eulerY     = 0;
    self->eulerZ     = 90;
    self->changeFlag = 0;
}

void readByteTankStatus(unsigned char *buffer, int byteLength,
                        struct TankStatus *ts) {
    if (byteLength < TANKSTATUS_PACKET_LENGTH || buffer == NULL || ts == NULL)
        return;
    int offset = 0;
    memcpy((void *)&ts->driveLeft,  buffer + offset, 1); offset += 1;
    memcpy((void *)&ts->driveRight, buffer + offset, 1); offset += 1;
    memcpy((void *)&ts->eulerX,     buffer + offset, 2); offset += 2;
    memcpy((void *)&ts->eulerY,     buffer + offset, 2); offset += 2;
    memcpy((void *)&ts->eulerZ,     buffer + offset, 2);
}

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

// ── ODOMETRY ──────────────────────────────────────────────────
#define WHEEL_RADIUS 0.05f
#define TRACK_WIDTH  0.32f
#define CPR          4096.0f

uint16_t leftAnglePrev  = 0;
uint16_t rightAnglePrev = 0;
unsigned long lastEncTime = 0;

float leftVel    = 0.0f;
float rightVel   = 0.0f;
float odom_x     = 0.0f;
float odom_y     = 0.0f;
float odom_theta = 0.0f;

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

    leftVel  = leftDist  / dt;
    rightVel = rightDist / dt;

    float linear  = (rightDist + leftDist) / 2.0f;
    float angular = (rightDist - leftDist) / TRACK_WIDTH;

    odom_theta += angular;
    odom_x     += linear * cosf(odom_theta);
    odom_y     += linear * sinf(odom_theta);
}

void sendOdometry() {
    uint8_t buf[20];
    buf[0] = 0xAA;
    buf[1] = 0xBB;
    memcpy(buf + 2,  &odom_x,     4);
    memcpy(buf + 6,  &odom_y,     4);
    memcpy(buf + 10, &odom_theta, 4);
    memcpy(buf + 14, &leftVel,    4);
    memcpy(buf + 18, &rightVel,   4);
    Serial.write(buf, 20);
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
struct TankStatus rxStatus;

uint8_t  pktBuf[TANKSTATUS_PACKET_LENGTH];
uint8_t  pktIdx   = 0;
int lastLeftRaw   = 0;
int lastRightRaw  = 0;
int currentLeftPWM  = 0;
int currentRightPWM = 0;
unsigned long lastLoop = 0;
unsigned long lastPktMs = 0;
#define PKT_TIMEOUT_MS 500

// ── DRIVE FROM TANKSTATUS ──────────────────────────────────────
void driveMotors() {
    int throttle = -((int)(unsigned char)rxStatus.driveRight - 127);
    int steering = -((int)(unsigned char)rxStatus.driveLeft  - 127);

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
    if (MCUSR & (1 << WDRF)) MCUSR = 0;
    wdt_disable();
    wdt_enable(WDTO_1S);

    Serial.begin(115200);
    Wire.begin();
    Wire.setClock(400000);

    pinMode(ENA,OUTPUT); pinMode(IN1,OUTPUT); pinMode(IN2,OUTPUT);
    pinMode(ENB,OUTPUT); pinMode(IN3,OUTPUT); pinMode(IN4,OUTPUT);
    stopMotors();

    constructTankStatus(&rxStatus);

    leftAnglePrev  = readAngle(LEFT_CHANNEL);
    rightAnglePrev = readAngle(RIGHT_CHANNEL);
    lastEncTime    = millis();
    lastPktMs      = millis();
    lastLoop       = millis();
}

// ── LOOP ──────────────────────────────────────────────────────
void loop() {
    wdt_reset();

    // Read TankStatus from Jetson
    while (Serial.available()) {
        uint8_t c = Serial.read();
        pktBuf[pktIdx++] = c;
        if (pktIdx >= TANKSTATUS_PACKET_LENGTH) {
            readByteTankStatus(pktBuf, TANKSTATUS_PACKET_LENGTH, &rxStatus);
            pktIdx  = 0;
            lastPktMs = millis();
        }
    }

    // Packet timeout → stop
    if ((millis() - lastPktMs) > PKT_TIMEOUT_MS) {
        stopMotors();
        motorsHalted = true;
    } else {
        motorsHalted = false;
    }

    unsigned long now = millis();
    if ((now - lastLoop) < LOOP_MS) return;
    lastLoop = now;

    updateOdometry();
    if(!motorsHalted){
        driveMotors();
    }

    // Odometry at 20Hz
    static int odomCount = 0;
    if (++odomCount >= 10) {
        odomCount = 0;
        sendOdometry();
    }
}
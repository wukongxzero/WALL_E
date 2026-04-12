/*
================================================================
  WALL-E MEGA — FINAL
  Uses TankStatus struct + makeByteTankStatus/readByteTankStatus
  exactly matching TankStatus.h and TankStatus.c
  
  DEBUG COMMANDS (Serial monitor 115200):
    'd' → toggle debug stream
    'p' → print current state snapshot
    's' → force stop motors
================================================================
*/

#include <avr/wdt.h>
#include <string.h>

// ── TANKSTATUS STRUCT (from TankStatus.h) ──
#define TANKSTATUS_PACKET_LENGTH 16
// ── STATE (add these) ──
int lastLeftRaw  = 0;
int lastRightRaw = 0;
#define FLIP_THRESHOLD 150   // if delta exceeds this, reject the update

struct TankStatus {
    volatile unsigned char driveLeft;
    volatile unsigned char driveRight;
    volatile short eulerX;
    volatile short eulerY;
    volatile char  eulerZ;
    volatile int   lockID;
    volatile unsigned char changeFlag;
};

// ── TANKSTATUS FUNCTIONS (from TankStatus.c) ──
void constructTankStatus(struct TankStatus *self) {
    self->driveLeft  = 0;
    self->driveRight = 0;
    self->eulerX     = 0;
    self->eulerY     = 0;
    self->eulerZ     = 90;
    self->changeFlag = 0;
}

void makeByteTankStatus(unsigned char *buffer, int byteLength,
                        struct TankStatus *ts) {
    if (byteLength < TANKSTATUS_PACKET_LENGTH || buffer == NULL || ts == NULL)
        return;
    int offset = 0;
    memcpy(buffer + offset, (const void *)&ts->driveLeft,  sizeof(ts->driveLeft));
    offset += sizeof(ts->driveLeft);
    memcpy(buffer + offset, (const void *)&ts->driveRight, sizeof(ts->driveRight));
    offset += sizeof(ts->driveRight);
    offset += 2;
    memcpy(buffer + offset, (const void *)&ts->eulerX, sizeof(ts->eulerX));
    offset += sizeof(ts->eulerX);
    memcpy(buffer + offset, (const void *)&ts->eulerY, sizeof(ts->eulerY));
    offset += sizeof(ts->eulerY);
    memcpy(buffer + offset, (const void *)&ts->eulerZ, sizeof(ts->eulerZ));
}

void readByteTankStatus(unsigned char *buffer, int byteLength,
                        struct TankStatus *ts) {
    if (byteLength < TANKSTATUS_PACKET_LENGTH || buffer == NULL || ts == NULL)
        return;
    int offset = 0;
    memcpy((void *)&ts->driveLeft,  buffer + offset, sizeof(ts->driveLeft));
    offset += sizeof(ts->driveLeft);
    memcpy((void *)&ts->driveRight, buffer + offset, sizeof(ts->driveRight));
    offset += sizeof(ts->driveRight);
    offset += 2;
    memcpy((void *)&ts->eulerX, buffer + offset, sizeof(ts->eulerX));
    offset += sizeof(ts->eulerX);
    memcpy((void *)&ts->eulerY, buffer + offset, sizeof(ts->eulerY));
    offset += sizeof(ts->eulerY);
    memcpy((void *)&ts->eulerZ, buffer + offset, sizeof(ts->eulerZ));
}

// ── MOTOR PINS ──
#define ENA   5
#define IN1   23
#define IN2   22
#define ENB   6
#define IN3   25
#define IN4   24
#define MAX_PWM  200
#define DEADZONE 12
#define LOOP_MS  5

// ── STATE ──
struct TankStatus rxStatus;
struct TankStatus txStatus;

uint8_t btBuf[TANKSTATUS_PACKET_LENGTH];
uint8_t btIdx    = 0;
uint32_t btBytes = 0;   // total BT bytes received
uint32_t btPkts  = 0;   // total complete packets parsed

float  pitchDeg = 0.0f;
String unoLine  = "";
unsigned long lastLoop = 0;

// current motor PWM values for debug
int currentLeftPWM  = 0;
int currentRightPWM = 0;

// debug flags
bool debugStream  = false;   // 'd' → continuous CSV stream
bool debugPackets = false;   // shows raw packet bytes on each receive

// ── HELPERS ──
int clampi(int x, int lo, int hi) {
    return x < lo ? lo : (x > hi ? hi : x);
}

// ── MOTOR DRIVER ──
void setMotor(int en, int a, int b, int signedSpeed) {
    int pwm = clampi(abs(signedSpeed), 0, MAX_PWM);
    if (pwm == 0) {
        digitalWrite(a, LOW); digitalWrite(b, LOW);
        analogWrite(en, 0); return;
    }
    digitalWrite(a, signedSpeed > 0 ? HIGH : LOW);
    digitalWrite(b, signedSpeed > 0 ? LOW  : HIGH);
    analogWrite(en, pwm);
}

void stopMotors() {
    setMotor(ENA, IN1, IN2, 0);
    setMotor(ENB, IN3, IN4, 0);
    currentLeftPWM = currentRightPWM = 0;
}

// ── PARSE TANKSTATUS FROM BT ──
void parseTankStatus(uint8_t* buf) {
    readByteTankStatus(buf, TANKSTATUS_PACKET_LENGTH, &rxStatus);
    btPkts++;

    // Debug: print raw packet bytes when debugPackets enabled
    if (debugPackets) {
        //Serial.print("PKT["); Serial.print(btPkts); Serial.print("] raw: ");
        for (int i = 0; i < TANKSTATUS_PACKET_LENGTH; i++) {
            if (buf[i] < 0x10) //Serial.print("0");
            Serial.print(buf[i], HEX);
            Serial.print(" ");
        }
//        Serial.println();
//        Serial.print("  → driveL="); Serial.print((int)(unsigned char)rxStatus.driveLeft);
//        Serial.print("  driveR=");   Serial.print((int)(unsigned char)rxStatus.driveRight);
//        Serial.print("  eulerX=");   Serial.print(rxStatus.eulerX);
//        Serial.print("  eulerY=");   Serial.print(rxStatus.eulerY);
//        Serial.print("  eulerZ=");   Serial.println(rxStatus.eulerZ);
    }
}

// ── SEND TANKSTATUS BACK TO PARALLAX ──
//void sendTankStatus(float pitchDegFloat) {
//    txStatus.driveLeft  = rxStatus.driveLeft;
//    txStatus.driveRight = rxStatus.driveRight;
//    txStatus.eulerX     = 0;                      // unused
//    txStatus.eulerY     = (short)pitchDegFloat;   // ← GUI reads this
//    txStatus.eulerZ     = 90;
//    txStatus.changeFlag = 0;
//    txStatus.lockID     = 0;
//
//    uint8_t buf[TANKSTATUS_PACKET_LENGTH];
//    memset(buf, 0, TANKSTATUS_PACKET_LENGTH);
//    makeByteTankStatus(buf, TANKSTATUS_PACKET_LENGTH, &txStatus);
//    Serial1.write(buf, TANKSTATUS_PACKET_LENGTH);
//}
void sendTankStatus(float pitchDegFloat) {
    txStatus.driveLeft  = rxStatus.driveLeft;
    txStatus.driveRight = rxStatus.driveRight;
    txStatus.eulerX     = 0;
    txStatus.eulerY     = (short)pitchDegFloat;   // pitch to GUI
    txStatus.eulerZ     = 90;
    txStatus.changeFlag = 0;
    txStatus.lockID     = 0;

    // ── DEBUG ──
    static int printCount = 0;
    if (++printCount >= 20) {   // print every 20 telemetry sends = ~1Hz
        printCount = 0;
        //Serial.print("TEL eulerY="); Serial.println(txStatus.eulerY);
    }

    uint8_t buf[TANKSTATUS_PACKET_LENGTH];
    memset(buf, 0, TANKSTATUS_PACKET_LENGTH);
    makeByteTankStatus(buf, TANKSTATUS_PACKET_LENGTH, &txStatus);
    Serial1.write(buf, TANKSTATUS_PACKET_LENGTH);
}

// ── DRIVE MOTORS FROM TANKSTATUS ──
//void driveMotors() {
//    int leftRaw  = -((int)(unsigned char)rxStatus.driveLeft  - 127);
//    int rightRaw = -((int)(unsigned char)rxStatus.driveRight - 127);
//
//    leftRaw  = clampi(leftRaw,  -124, 124);
//    rightRaw = clampi(rightRaw, -124, 124);
//
//    // ── FLIP REJECTION ──
//    if (abs(leftRaw - lastLeftRaw) > FLIP_THRESHOLD) {
//        leftRaw = lastLeftRaw;   // ignore this update, hold last good value
//        //Serial.println("DBG: left flip rejected");
//    }
//    if (abs(rightRaw - lastRightRaw) > FLIP_THRESHOLD) {
//        rightRaw = lastRightRaw;
//        //Serial.println("DBG: right flip rejected");
//    }
//
//    lastLeftRaw  = leftRaw;
//    lastRightRaw = rightRaw;
//
//    currentLeftPWM  = leftRaw;
//    currentRightPWM = rightRaw;
//    
//    int throttle = rightRaw*1.4;
//    int turn = leftRaw*1.4;
//    
//    int rightCmd = throttle + turn;
//    int leftCmd = throttle - turn;
//    
//    setMotor(ENA, IN1, IN2, rightCmd);
//    setMotor(ENB, IN3, IN4, leftCmd);
//}
void driveMotors() {
    // ── RAW DECODE (0-255 → -124..+124) ──
    int throttle = -((int)(unsigned char)rxStatus.driveRight - 127);
    int steering = -((int)(unsigned char)rxStatus.driveLeft  - 127);

    throttle = clampi(throttle, -124, 124);
    steering = clampi(steering, -124, 124);

    // ── FLIP REJECTION ──
    if (abs(throttle - lastLeftRaw)  > FLIP_THRESHOLD) throttle = lastLeftRaw;
    if (abs(steering - lastRightRaw) > FLIP_THRESHOLD) steering = lastRightRaw;

    lastLeftRaw  = throttle;
    lastRightRaw = steering;

    // ── SCALE ±124 → ±255 ──
    throttle = (throttle * 255) / 124;
    steering = (steering * 255) / 124;

    // ── ARCADE MIX + CLAMP ──
    currentRightPWM = clampi(throttle + steering, -MAX_PWM, MAX_PWM);
    currentLeftPWM  = clampi(throttle - steering, -MAX_PWM, MAX_PWM);

    setMotor(ENA, IN1, IN2, currentRightPWM);
    setMotor(ENB, IN3, IN4, currentLeftPWM);
}
// ── PARSE PITCH FROM UNO ──
void parseUnoLine(String line) {
    line.trim();
    if (line.startsWith("P:")) {
        pitchDeg = line.substring(2).toFloat();
        if (debugStream) {
           // Serial.print("UNO pitch=");
            //Serial.println(pitchDeg, 2);
        }
    }
}

// ── PRINT STATE SNAPSHOT ──
void printState() {
    //Serial.println("=== STATE SNAPSHOT ===");
    //Serial.print("  pitch=");      Serial.println(pitchDeg, 2);
    //Serial.print("  driveL=");     Serial.print((int)(unsigned char)rxStatus.driveLeft);
    //Serial.print("  driveR=");     Serial.println((int)(unsigned char)rxStatus.driveRight);
    //Serial.print("  leftPWM=");    Serial.print(currentLeftPWM);
    //Serial.print("  rightPWM=");   Serial.println(currentRightPWM);
    //Serial.print("  eulerX=");     Serial.print(rxStatus.eulerX);
    //Serial.print("  eulerY=");     Serial.print(rxStatus.eulerY);
    //Serial.print("  eulerZ=");     Serial.println(rxStatus.eulerZ);
    //Serial.print("  btBytes=");    Serial.print(btBytes);
    //Serial.print("  btPkts=");     Serial.println(btPkts);
    //Serial.print("  debug=");      Serial.print(debugStream ? "ON" : "OFF");
    //Serial.print("  pktDebug=");   Serial.println(debugPackets ? "ON" : "OFF");
    //Serial.println("======================");
}

void setup() {
    if (MCUSR & (1 << WDRF)) MCUSR = 0;
    wdt_disable();
    wdt_enable(WDTO_250MS);

    Serial.begin(115200);
    

    pinMode(ENA,OUTPUT); pinMode(IN1,OUTPUT); pinMode(IN2,OUTPUT);
    pinMode(ENB,OUTPUT); pinMode(IN3,OUTPUT); pinMode(IN4,OUTPUT);
    stopMotors();

    constructTankStatus(&rxStatus);
    constructTankStatus(&txStatus);

    //Serial.println("================================");
    //Serial.println("  WALL-E MEGA READY");
    //Serial.println("  d=debug stream  p=snapshot");
    //Serial.println("  b=packet debug  s=stop");
    //Serial.println("  CSV: ms,driveL,driveR,lPWM,rPWM,pitch,pkts");
    //Serial.println("================================");

    lastLoop = millis();
}

void loop() {
    wdt_reset();
    
    

    // ── BT RECEIVE ──
    while (Serial.available()) {
        uint8_t c = Serial.read();
        btBytes++;
        btBuf[btIdx++] = c;
        if (btIdx >= TANKSTATUS_PACKET_LENGTH) {
            parseTankStatus(btBuf);
            btIdx = 0;
        }
    }

    // ── FIXED RATE LOOP ──
    unsigned long now = millis();
    if ((now - lastLoop) < LOOP_MS) return;
    lastLoop = now;

    // ── DRIVE MOTORS ──
    driveMotors();

    // ── TELEMETRY TO PARALLAX (20Hz) ──
    static int telCount = 0;
    if (++telCount >= 10) {
        telCount = 0;
  
    }

    // ── DEBUG CSV STREAM (5Hz) ──
    static int dbgCount = 0;
    if (++dbgCount >= 40) {
        dbgCount = 0;
        if (debugStream) {
            // CSV: ms, driveL, driveR, leftPWM, rightPWM, pitch, packets
            //Serial.print(millis());                              Serial.print(",");
            //Serial.print((int)(unsigned char)rxStatus.driveLeft);      Serial.print(",");
            //Serial.print((int)(unsigned char)rxStatus.driveRight);     Serial.print(",");
            //Serial.print(currentLeftPWM);                        Serial.print(",");
            //Serial.print(currentRightPWM);                       Serial.print(",");
            //Serial.print(pitchDeg, 2);                           Serial.print(",");
            //Serial.println(btPkts);
        } else {
            // Plain 2Hz status when debug off
            //Serial.print("pitch=");  Serial.print(pitchDeg, 2);
            //Serial.print("  L=");    Serial.print(currentLeftPWM);
            //Serial.print("  R=");    Serial.print(currentRightPWM);
            //Serial.print("  pkts="); Serial.println(btPkts);
        }
    }
}
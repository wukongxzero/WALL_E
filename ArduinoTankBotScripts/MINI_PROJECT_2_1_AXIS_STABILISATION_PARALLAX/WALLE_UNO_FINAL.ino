// TEST 3 — STABILIZER ONLY (no motors, no BT)
// Commands: 0=PP 1=LQR 2=LQG d=stream p=print r=reset

#include <Wire.h>
#include <Servo.h>
#include <avr/wdt.h>
#define TELEM_EVERY 10   // send pitch every 10 loops = 20Hz
int controllerMode = 0;

#define K0_PP    0.7169f
#define K1_PP    0.0327f
#define K0_LQR   2.1050f
#define K1_LQR   0.1475f
#define K0_LQG   2.1050f
#define K1_LQG   0.1475f
#define L0_KAL   0.992877f
#define L1_KAL   10.300535f
#define G_OVER_L 245.25f
#define INV_MLL  2394.64f

// ── EXACT SAME AS YOUR WORKING PID ──
static const uint8_t MPU_ADDR   = 0x68;
static const float   GYRO_SENS  = 131.0f;
static const float   COMP_ALPHA = 0.985f;   // matches your PID
static const uint32_t LOOP_HZ   = 200;
static const uint32_t LOOP_US   = 1000000UL / LOOP_HZ;

static const int SERVO_PIN       = 9;
static const int SERVO_CENTER_US = 1500;
static const int SERVO_RANGE_US  = 300;
static const float CMD_LIMIT     = 25.0f;

Servo platformServo;
uint32_t lastLoopUs = 0;

// IMU state — exact same variable names as your PID
float pitchDeg     = 0.0f;
float gyroPitchDps = 0.0f;
float accPitchDeg  = 0.0f;

// Controller state
float pitchRad    = 0.0f;
float pitchDot    = 0.0f;
float filteredCmd = 0.0f;
float alphaEst    = 0.0f;
float alphaDotEst = 0.0f;

bool debugStream = false;

// ── EXACT SAME MPU FUNCTIONS AS YOUR PID ──
void mpuWrite(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(reg);
    Wire.write(val);
    Wire.endTransmission(true);
}

void mpuReadBytes(uint8_t reg, uint8_t n, uint8_t* buf) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, n, true);
    for (uint8_t i = 0; i < n; i++) buf[i] = Wire.read();
}

void mpuInit() {
    mpuWrite(0x6B, 0x00);  // wake
    delay(50);
    mpuWrite(0x1B, 0x00);  // gyro ±250 dps
    mpuWrite(0x1C, 0x00);  // accel ±2g
    mpuWrite(0x1A, 0x03);  // DLPF ~42Hz — same as your PID
}

float clampf(float x, float lo, float hi) {
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

void applyServo(float u) {
    u = clampf(u, -CMD_LIMIT, CMD_LIMIT);
    int pulse = SERVO_CENTER_US + (int)(u * (SERVO_RANGE_US / CMD_LIMIT));
    platformServo.writeMicroseconds(pulse);
}

//void printMode() {
//    Serial.print("Mode: ");
//    switch(controllerMode) {
//        case 0: Serial.println("POLE PLACEMENT"); break;
//        case 1: Serial.println("LQR");            break;
//        case 2: Serial.println("LQG");            break;
//    }
//}

void setup() {
    // ── EXACT SAME AS YOUR PID SETUP ──
    if (MCUSR & (1 << WDRF)) MCUSR = 0;
    wdt_disable();
    wdt_enable(WDTO_2S);      // 2s gives IMU time to wake

    Serial.begin(115200);
    Wire.begin();
    Wire.setClock(400000);

    mpuInit();

    platformServo.attach(SERVO_PIN);
    platformServo.writeMicroseconds(SERVO_CENTER_US);

    //Serial.println("=== TEST 3 — STABILIZER ===");
    //Serial.println("0=PP 1=LQR 2=LQG d=stream p=print");
    //printMode();

    lastLoopUs = micros();
    delay(200);
}

void loop() {
//    // Handle serial commands
//    if (Serial.available()) {
//        char cmd = Serial.read();
////        if      (cmd=='0') { controllerMode=0; filteredCmd=alphaEst=alphaDotEst=0; printMode(); }
////        else if (cmd=='1') { controllerMode=1; filteredCmd=alphaEst=alphaDotEst=0; printMode(); }
////        else if (cmd=='2') { controllerMode=2; filteredCmd=alphaEst=alphaDotEst=0; printMode(); }
////        else if (cmd=='d') { debugStream=!debugStream; //Serial.println(debugStream?"Stream ON — ms,alpha,cmd":"Stream OFF"); }
////        else if (cmd=='p') { Serial.print("alpha="); //Serial.print(pitchDeg,2); Serial.print("  cmd="); Serial.println(filteredCmd,2); }
//    }

    uint32_t nowUs = micros();
    if ((uint32_t)(nowUs - lastLoopUs) < LOOP_US) return;
    float dt = (nowUs - lastLoopUs) * 1e-6f;
    lastLoopUs = nowUs;
    wdt_reset();

    // ── EXACT SAME IMU READ AS YOUR WORKING PID ──
    int16_t ax, ay, az, gx, gy, gz;
    {
        uint8_t buf[14];
        mpuReadBytes(0x3B, 14, buf);

        ay = (int16_t)((buf[0]  << 8) | buf[1]);
        ax = (int16_t)((buf[2]  << 8) | buf[3]);
        az = (int16_t)((buf[4]  << 8) | buf[5]);
        gx = (int16_t)((buf[8]  << 8) | buf[9]);
        gy = (int16_t)((buf[10] << 8) | buf[11]);
        gz = (int16_t)((buf[12] << 8) | buf[13]);
    }

    // ── EXACT SAME FILTER AS YOUR WORKING PID ──
    float fax = (float)ax;
    float fay = (float)ay;
    float faz = (float)az;

    accPitchDeg  = atan2f(-fax, sqrtf(fay*fay + faz*faz)) * 57.29578f;
    gyroPitchDps = ((float)gy) / GYRO_SENS;

    pitchDeg = COMP_ALPHA * (pitchDeg + gyroPitchDps * dt)
             + (1.0f - COMP_ALPHA) * accPitchDeg;

    // Convert for controller
    pitchRad = pitchDeg * (PI / 180.0f);
    pitchDot = gyroPitchDps * (PI / 180.0f);

    // ── CONTROLLER ──
    float u = 0.0f;
    switch (controllerMode) {
        case 0:
            u = -K0_PP  * pitchRad - K1_PP  * pitchDot;
            break;
        case 1:
            u = -K0_LQR * pitchRad - K1_LQR * pitchDot;
            break;
        case 2: {
            float u_prev   = -K0_LQG * alphaEst - K1_LQG * alphaDotEst;
            float aP       = alphaEst    + dt * alphaDotEst;
            float adP      = alphaDotEst + dt * (G_OVER_L * alphaEst + INV_MLL * u_prev);
            float innov    = accPitchDeg * (PI/180.0f) - aP;
            alphaEst       = aP  + L0_KAL * innov;
            alphaDotEst    = adP + L1_KAL * innov;
            pitchRad = alphaEst;
            pitchDeg = pitchRad * 57.29578f;
            pitchDot = alphaDotEst;
            u = -K0_LQG * alphaEst - K1_LQG * alphaDotEst;
            break;
        }
    }

    // ── SERVO ──
    float cmdDeg = u * (180.0f / PI);
    filteredCmd  = (0.7f * filteredCmd) + (0.3f * cmdDeg);
    applyServo(filteredCmd);

    // ── STREAM ──
    static int sc = 0;
    if (debugStream && ++sc >= 20) {
        sc = 0;
        //Serial.print(millis());      Serial.print(",");
        //Serial.print(pitchDeg, 3);   Serial.print(",");
        //Serial.println(filteredCmd, 3);
    }
    // ── SEND PITCH TO jetson ──
    static int tc = 0;
    if (++tc >= TELEM_EVERY) {
        tc = 0;
        Serial.print("P:");
        Serial.println(pitchDeg, 3);
    }
}
#include <Wire.h>
#include <Servo.h>
#include "src/TankStatusClass/TankStatusClass.h"



TankStatusClass tsLocalOut; // For sending data back to Python/ROS
// ── Configuration ──
static const uint8_t MPU_ADDR = 0x68;
static const float GYRO_SENS = 131.0f;
// Trust the Gyro more (0.995) to stop accelerometer-induced jitter
static const float COMP_ALPHA = 0.995f; 

const int PITCH_PIN = 9;
const int ROLL_PIN  = 10;
const int P_CENTER  = 1400;  
const int R_CENTER  = 1210;
const int SAFE_RANGE = 300; 
const int SAFE_RANGE_PITCH = 400;

const float K0 = 0.6f; //kp
const float K1 = 0.023f;//kd
const float K2 = .00001f;//ki

float pitchIntErr = 0;
float rollIntErr = 0;

const float MAX_INT = 100.0f;

// ── Smoothing Variables ──
float smoothP = P_CENTER;
float smoothR = R_CENTER;
const float LPF_BETA = 0.2f; // Lower = smoother, but slower (range 0.05 to 0.4)

float gyroPitchBias = 0, gyroRollBias = 0;
float pitchDeg = 0, rollDeg = 0, yawDeg = 0;
uint32_t lastUs = 0;

Servo pitchServo, rollServo;

void mpuWrite(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(reg);
    Wire.write(val);
    Wire.endTransmission();
}

void setup() {
    Serial.begin(115200);
    tsLocalOut = TankStatusClass();
    while(!Serial); 
    Wire.begin();
    Wire.setClock(400000);
    
    mpuWrite(0x6B, 0x00); 
    delay(100);
    mpuWrite(0x1B, 0x00); 
    mpuWrite(0x1C, 0x00); 
    mpuWrite(0x1A, 0x03); 
    
//    Serial.println("Calibrating...");
    float sumGx = 0, sumGy = 0;
    for(int i = 0; i < 400; i++) {
        Wire.beginTransmission(MPU_ADDR);
        Wire.write(0x43); 
        Wire.endTransmission(false);
        Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)4, (uint8_t)true);
        sumGx += (float)((int16_t)(Wire.read() << 8 | Wire.read()));
        sumGy += (float)((int16_t)(Wire.read() << 8 | Wire.read()));
        delay(2);
    }
    gyroPitchBias = (sumGx / 400.0f) / GYRO_SENS;
    gyroRollBias  = (sumGy / 400.0f) / GYRO_SENS;

    pitchServo.attach(PITCH_PIN); 
    rollServo.attach(ROLL_PIN);   
    pitchServo.writeMicroseconds(P_CENTER);
    rollServo.writeMicroseconds(R_CENTER);
    
    lastUs = micros();
}

void loop() {
    uint32_t now = micros();
    float dt = (now - lastUs) * 1e-6f;
    if (dt < 0.01f) return; // Run at 100Hz (Better for many servos)
    lastUs = now;

    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B); 
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)14, (uint8_t)true);
    
    int16_t rawAx = Wire.read() << 8 | Wire.read(); 
    int16_t rawAy = Wire.read() << 8 | Wire.read(); 
    int16_t rawAz = Wire.read() << 8 | Wire.read(); 
    Wire.read(); Wire.read(); 
    int16_t rawGx = Wire.read() << 8 | Wire.read();
    int16_t rawGy = Wire.read() << 8 | Wire.read();

    float accP = (atan2f(-(float)rawAx, sqrtf((float)rawAy*rawAy + (float)rawAz*rawAz)) * 57.296f);
    float accR = (atan2f( (float)rawAy, sqrtf((float)rawAx*rawAx + (float)rawAz*rawAz)) * 57.296f);
    float rateP = ((float)rawGx / GYRO_SENS) - gyroPitchBias;
    float rateR = ((float)rawGy / GYRO_SENS) - gyroRollBias;

    pitchIntErr += (pitchDeg * dt);
    rollIntErr  += (rollDeg * dt);
    pitchIntErr = constrain(pitchIntErr, -MAX_INT, MAX_INT);
    rollIntErr  = constrain(rollIntErr, -MAX_INT, MAX_INT);

    pitchDeg = COMP_ALPHA * (pitchDeg + rateP * dt) + (1.0f - COMP_ALPHA) * accP;
    rollDeg  = COMP_ALPHA * (rollDeg  + rateR * dt) + (1.0f - COMP_ALPHA) * accR;

    float pitchRad = pitchDeg * 0.017453f;
    float cosPitch = cos(pitchRad);
    if (cosPitch < 0.2f) {
        cosPitch = 0.2f; 
    }


    float uP = -(K0 * pitchDeg * 0.017453f + K1 * rateP * 0.017453f+ K2 * pitchIntErr * 0.017453f);
    float uR = -(K0 * rollDeg  * 0.017453f + K1 * rateR * 0.017453f+ K2 * rollIntErr  * 0.017453f);

    uR = uR / cosPitch;
    
    
    int pOffset = (int)(uP * 573.0f);
    int rOffset = (int)(uR * 573.0f);

    pOffset = constrain(pOffset, -SAFE_RANGE, SAFE_RANGE);
    rOffset = constrain(rOffset, -SAFE_RANGE, SAFE_RANGE);
    

    // Reduced deadband for smoother feel
    if (abs(pitchDeg) < 0.5) pOffset = 0;  
    if (abs(rollDeg) < 0.5) rOffset = 0;

    int targetP = P_CENTER + pOffset;
    int targetR = R_CENTER + rOffset;

    // ── EMA SMOOTHING ──
    //smoothP = (targetP * LPF_BETA) + (smoothP * (1.0f - LPF_BETA));
    //smoothR = (targetR * LPF_BETA) + (smoothR * (1.0f - LPF_BETA));

    pitchServo.writeMicroseconds((int)targetP);
    rollServo.writeMicroseconds((int)targetR);
/*
    Serial.println("target pitch");
    Serial.println(targetP);
    Serial.println("pitch angle");
    Serial.println(pitchDeg);
    Serial.println("target roll");
    Serial.println(targetR);
    Serial.println("roll angle");
    Serial.println(rollDeg);
    */


    tsLocalOut.eulerYFloat = pitchDeg;
    tsLocalOut.eulerXFloat = rollDeg;
    tsLocalOut.eulerZFloat = yawDeg; 
    static int odomCount = 0;
    if (++odomCount >= 10) {

        static int odomCount = 0;
        odomCount = 0;
        unsigned char* txBuffer = tsLocalOut.MakeIntoBytes();
        Serial.write(txBuffer, TANKSTATUS_PACKET_LENGTH);
    }
}

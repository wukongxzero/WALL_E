// TEST 2 — SERVO + IMU
// Commands: t=stream c=center p=print 1-5=+angles a-e=-angles

#include <Wire.h>
#include <Servo.h>

#define SERVO_PIN       9
#define SERVO_CENTER_US 1500
#define SERVO_RANGE_US  300
#define CMD_LIMIT       25.0f
#define MPU_ADDR        0x68
#define GYRO_SENS       131.0f
#define COMP_ALPHA      0.98f
#define LOOP_US         5000UL   // 200Hz

Servo platformServo;
float pitchDeg = 0.0f, pitchDot = 0.0f;
bool  streaming = false;
uint32_t lastLoopUs = 0;

void mpuWrite(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(MPU_ADDR); Wire.write(reg); Wire.write(val); Wire.endTransmission(true);
}
void mpuReadBytes(uint8_t reg, uint8_t n, uint8_t* buf) {
    Wire.beginTransmission(MPU_ADDR); Wire.write(reg); Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, n, true);
    for (uint8_t i=0; i<n; i++) buf[i]=Wire.read();
}
void mpuInit() {
    mpuWrite(0x6B,0x00); delay(50);
    mpuWrite(0x1B,0x00); mpuWrite(0x1C,0x00); mpuWrite(0x1A,0x05);
}
float clampf(float x, float lo, float hi) { return x<lo?lo:(x>hi?hi:x); }

void applyServo(float deg) {
    deg = clampf(deg, -CMD_LIMIT, CMD_LIMIT);
    platformServo.writeMicroseconds(SERVO_CENTER_US + (int)(deg*(SERVO_RANGE_US/CMD_LIMIT)));
    Serial.print("Servo -> "); Serial.print(deg); Serial.println(" deg");
}

void setup() {
    Serial.begin(115200); Wire.begin(); Wire.setClock(400000); mpuInit();
    platformServo.attach(SERVO_PIN);
    platformServo.writeMicroseconds(SERVO_CENTER_US);
    Serial.println("TEST 2 — SERVO + IMU");
    Serial.println("t=stream c=center p=print 1-5=+10to+25deg a-e=-10to-25deg");
    Wire.beginTransmission(MPU_ADDR);
    Serial.println(Wire.endTransmission()==0 ? "MPU6050: FOUND" : "MPU6050: NOT FOUND");
    lastLoopUs = micros();
}

void loop() {
    uint32_t nowUs = micros();
    if ((uint32_t)(nowUs-lastLoopUs) >= LOOP_US) {
        float dt = (float)(nowUs-lastLoopUs)*1e-6f;
        lastLoopUs = nowUs;
        uint8_t buf[14]; mpuReadBytes(0x3B,14,buf);
        int16_t ay=(int16_t)((buf[0]<<8)|buf[1]);
        int16_t ax=(int16_t)((buf[2]<<8)|buf[3]);
        int16_t az=(int16_t)((buf[4]<<8)|buf[5]);
        int16_t gy=(int16_t)((buf[10]<<8)|buf[11]);
        float accPitch = atan2f(-(float)ax, sqrtf((float)ay*(float)ay+(float)az*(float)az))*57.29578f;
        float gyroDps  = (float)gy/GYRO_SENS;
        pitchDeg = COMP_ALPHA*(pitchDeg+gyroDps*dt)+(1.0f-COMP_ALPHA)*accPitch;
        pitchDot = gyroDps*(PI/180.0f);
        static int sc=0;
        if (streaming && ++sc>=20) {
            sc=0;
            Serial.print("alpha="); Serial.print(pitchDeg,2);
            Serial.print(" deg  rate="); Serial.print(pitchDot,3); Serial.println(" rad/s");
        }
    }
    if (!Serial.available()) return;
    char cmd = Serial.read();
    switch(cmd) {
        case 't': streaming=!streaming; Serial.println(streaming?"Stream ON":"Stream OFF"); break;
        case 'c': applyServo(0);   break;
        case 'p':
            Serial.print("angle="); Serial.print(pitchDeg,2);
            Serial.print(" rate="); Serial.println(pitchDot,3); break;
        case '1': applyServo(10);  break;
        case '2': applyServo(15);  break;
        case '3': applyServo(20);  break;
        case '4': applyServo(25);  break;
        case '5': applyServo(5);   break;
        case 'a': applyServo(-10); break;
        case 'b': applyServo(-15); break;
        case 'd': applyServo(-20); break;
        case 'e': applyServo(-25); break;
    }
}
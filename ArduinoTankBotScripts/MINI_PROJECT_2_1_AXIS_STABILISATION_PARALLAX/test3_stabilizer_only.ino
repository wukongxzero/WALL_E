// TEST 3 — STABILIZER ONLY (no motors, no BT)
// Commands: 0=PP 1=LQR 2=LQG d=stream p=print r=reset h=help

#include <Wire.h>
#include <Servo.h>
#include <avr/wdt.h>

int controllerMode = 0;

#define K0_PP    2.4034f
#define K1_PP    0.1450f
#define K0_LQR   6.6772f
#define K1_LQR   0.4824f
#define K0_LQG   6.6772f
#define K1_LQG   0.4824f
#define L0_KAL   0.992877f
#define L1_KAL   10.030236f
#define G_OVER_L 140.14f
#define INV_MLL  408.16f

#define SERVO_PIN       9
#define SERVO_CENTER_US 1500
#define SERVO_RANGE_US  300
#define CMD_LIMIT       25.0f
#define MPU_ADDR        0x68
#define GYRO_SENS       131.0f
#define COMP_ALPHA      0.98f
#define LOOP_US         5000UL

Servo platformServo;
float pitchDeg=0, pitchRad=0, pitchDot=0, filteredCmd=0;
float alphaEst=0, alphaDotEst=0;
bool  debugStream=false;
uint32_t lastLoopUs=0;

void mpuWrite(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(MPU_ADDR); Wire.write(reg); Wire.write(val); Wire.endTransmission(true);
}
void mpuReadBytes(uint8_t reg, uint8_t n, uint8_t* buf) {
    Wire.beginTransmission(MPU_ADDR); Wire.write(reg); Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR,n,true); for(uint8_t i=0;i<n;i++) buf[i]=Wire.read();
}
void mpuInit() { mpuWrite(0x6B,0x00);delay(50);mpuWrite(0x1B,0x00);mpuWrite(0x1C,0x00);mpuWrite(0x1A,0x05); }
float clampf(float x,float lo,float hi){return x<lo?lo:(x>hi?hi:x);}
void applyServo(float u) {
    u=clampf(u,-CMD_LIMIT,CMD_LIMIT);
    platformServo.writeMicroseconds(SERVO_CENTER_US+(int)(u*(SERVO_RANGE_US/CMD_LIMIT)));
}
void printMode() {
    Serial.print("Mode: ");
    switch(controllerMode){case 0:Serial.println("POLE PLACEMENT");break;case 1:Serial.println("LQR");break;case 2:Serial.println("LQG");}
}

void setup() {
    if(MCUSR&(1<<WDRF))MCUSR=0; wdt_disable(); wdt_enable(WDTO_250MS);
    Serial.begin(115200); Wire.begin(); Wire.setClock(400000); mpuInit();
    platformServo.attach(SERVO_PIN);
    platformServo.writeMicroseconds(SERVO_CENTER_US); delay(500);
    Serial.println("TEST 3 — STABILIZER ONLY");
    Serial.println("0=PP 1=LQR 2=LQG d=stream p=print r=reset");
    printMode();
    lastLoopUs=micros();
}

void loop() {
    uint32_t nowUs=micros();
    if((uint32_t)(nowUs-lastLoopUs)<LOOP_US) {
        if(Serial.available()) {
            char cmd=Serial.read();
            if(cmd=='0'){controllerMode=0;filteredCmd=alphaEst=alphaDotEst=0;printMode();}
            else if(cmd=='1'){controllerMode=1;filteredCmd=alphaEst=alphaDotEst=0;printMode();}
            else if(cmd=='2'){controllerMode=2;filteredCmd=alphaEst=alphaDotEst=0;printMode();}
            else if(cmd=='d'){debugStream=!debugStream;Serial.println(debugStream?"Stream ON":"Stream OFF");
                              if(debugStream)Serial.println("ms,alpha,alphaDot,cmd");}
            else if(cmd=='p'){Serial.print("alpha=");Serial.print(pitchDeg,2);
                              Serial.print(" cmd=");Serial.println(filteredCmd,2);}
            else if(cmd=='r'){Serial.println("Reset");}
        }
        return;
    }
    float dt=(float)(nowUs-lastLoopUs)*1e-6f; lastLoopUs=nowUs; wdt_reset();

    uint8_t buf[14]; mpuReadBytes(0x3B,14,buf);
    int16_t ay=(int16_t)((buf[0]<<8)|buf[1]);
    int16_t ax=(int16_t)((buf[2]<<8)|buf[3]);
    int16_t az=(int16_t)((buf[4]<<8)|buf[5]);
    int16_t gy=(int16_t)((buf[10]<<8)|buf[11]);
    float accPitch=atan2f(-(float)ax,sqrtf((float)ay*(float)ay+(float)az*(float)az))*57.29578f;
    float gyroDps=(float)gy/GYRO_SENS;
    pitchDot=gyroDps*(PI/180.0f);

    float u=0;
    switch(controllerMode) {
        case 0:
            pitchDeg=COMP_ALPHA*(pitchDeg+gyroDps*dt)+(1.0f-COMP_ALPHA)*accPitch;
            pitchRad=pitchDeg*(PI/180.0f);
            u=-K0_PP*pitchRad-K1_PP*pitchDot; break;
        case 1:
            pitchDeg=COMP_ALPHA*(pitchDeg+gyroDps*dt)+(1.0f-COMP_ALPHA)*accPitch;
            pitchRad=pitchDeg*(PI/180.0f);
            u=-K0_LQR*pitchRad-K1_LQR*pitchDot; break;
        case 2: {
            float u_prev=-K0_LQG*alphaEst-K1_LQG*alphaDotEst;
            float aP=alphaEst+dt*alphaDotEst;
            float adP=alphaDotEst+dt*(G_OVER_L*alphaEst+INV_MLL*u_prev);
            float innov=accPitch*(PI/180.0f)-aP;
            alphaEst=aP+L0_KAL*innov; alphaDotEst=adP+L1_KAL*innov;
            pitchRad=alphaEst; pitchDeg=pitchRad*(180.0f/PI); pitchDot=alphaDotEst;
            u=-K0_LQG*alphaEst-K1_LQG*alphaDotEst; break;
        }
    }
    float cmdDeg=u*(180.0f/PI);
    filteredCmd=0.7f*filteredCmd+0.3f*cmdDeg;
    applyServo(filteredCmd);

    static int sc=0;
    if(debugStream&&++sc>=20){sc=0;
        Serial.print(millis());Serial.print(",");
        Serial.print(pitchDeg,3);Serial.print(",");
        Serial.print(pitchDot,3);Serial.print(",");
        Serial.println(filteredCmd,3);}
}
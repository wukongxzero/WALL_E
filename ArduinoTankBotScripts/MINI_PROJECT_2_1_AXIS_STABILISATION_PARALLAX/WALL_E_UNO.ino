// WALL-E UNO — FINAL
// IMU + Controller + Servo → sends pitch to Mega via TX pin
// Change CONTROLLER_MODE: 0=PP 1=LQR 2=LQG

#include <Wire.h>
#include <Servo.h>
#include <avr/wdt.h>

#define CONTROLLER_MODE 0   // ← CHANGE THIS

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
#define TELEM_EVERY     10

Servo platformServo;
float pitchDeg=0,pitchRad=0,pitchDot=0,filteredCmd=0;
float alphaEst=0,alphaDotEst=0;
uint32_t lastLoopUs=0;

void mpuWrite(uint8_t reg,uint8_t val){Wire.beginTransmission(MPU_ADDR);Wire.write(reg);Wire.write(val);Wire.endTransmission(true);}
void mpuReadBytes(uint8_t reg,uint8_t n,uint8_t* buf){Wire.beginTransmission(MPU_ADDR);Wire.write(reg);Wire.endTransmission(false);Wire.requestFrom(MPU_ADDR,n,true);for(uint8_t i=0;i<n;i++)buf[i]=Wire.read();}
void mpuInit(){mpuWrite(0x6B,0x00);delay(50);mpuWrite(0x1B,0x00);mpuWrite(0x1C,0x00);mpuWrite(0x1A,0x05);}
float clampf(float x,float lo,float hi){return x<lo?lo:(x>hi?hi:x);}
void applyServo(float u){u=clampf(u,-CMD_LIMIT,CMD_LIMIT);platformServo.writeMicroseconds(SERVO_CENTER_US+(int)(u*(SERVO_RANGE_US/CMD_LIMIT)));}

void setup() {
    if(MCUSR&(1<<WDRF))MCUSR=0;wdt_disable();wdt_enable(WDTO_250MS);
    Serial.begin(115200);   // TX wire to Mega RX2
    Wire.begin();Wire.setClock(400000);mpuInit();
    platformServo.attach(SERVO_PIN);
    platformServo.writeMicroseconds(SERVO_CENTER_US);delay(500);
    lastLoopUs=micros();
}

void loop() {
    uint32_t nowUs=micros();
    if((uint32_t)(nowUs-lastLoopUs)<LOOP_US)return;
    float dt=(float)(nowUs-lastLoopUs)*1e-6f;lastLoopUs=nowUs;wdt_reset();

    uint8_t buf[14];mpuReadBytes(0x3B,14,buf);
    int16_t ay=(int16_t)((buf[0]<<8)|buf[1]);
    int16_t ax=(int16_t)((buf[2]<<8)|buf[3]);
    int16_t az=(int16_t)((buf[4]<<8)|buf[5]);
    int16_t gy=(int16_t)((buf[10]<<8)|buf[11]);
    float accPitch=atan2f(-(float)ax,sqrtf((float)ay*(float)ay+(float)az*(float)az))*57.29578f;
    float gyroDps=(float)gy/GYRO_SENS;
    pitchDot=gyroDps*(PI/180.0f);

    float u=0;
    #if CONTROLLER_MODE==0
        pitchDeg=COMP_ALPHA*(pitchDeg+gyroDps*dt)+(1-COMP_ALPHA)*accPitch;
        pitchRad=pitchDeg*(PI/180.0f);
        u=-K0_PP*pitchRad-K1_PP*pitchDot;
    #elif CONTROLLER_MODE==1
        pitchDeg=COMP_ALPHA*(pitchDeg+gyroDps*dt)+(1-COMP_ALPHA)*accPitch;
        pitchRad=pitchDeg*(PI/180.0f);
        u=-K0_LQR*pitchRad-K1_LQR*pitchDot;
    #elif CONTROLLER_MODE==2
        float up=-K0_LQG*alphaEst-K1_LQG*alphaDotEst;
        float aP=alphaEst+dt*alphaDotEst;
        float adP=alphaDotEst+dt*(G_OVER_L*alphaEst+INV_MLL*up);
        float inn=accPitch*(PI/180.0f)-aP;
        alphaEst=aP+L0_KAL*inn;alphaDotEst=adP+L1_KAL*inn;
        pitchRad=alphaEst;pitchDeg=pitchRad*57.29578f;pitchDot=alphaDotEst;
        u=-K0_LQG*alphaEst-K1_LQG*alphaDotEst;
    #endif

    filteredCmd=0.7f*filteredCmd+0.3f*u*(180.0f/PI);
    applyServo(filteredCmd);

    static int tc=0;
    if(++tc>=TELEM_EVERY){tc=0;
        Serial.print("P:");Serial.println(pitchDeg,3);}
}
/*
================================================================
  WALL-E UNO V2 — MPU6050 + AS5600 Fused Gimbal Stabilization
  Hardware:
    - MPU6050 on A4/A5
    - TCA9548A (0x70) on A4/A5
    - AS5600 pitch → TCA ch0, roll → TCA ch1
    - Pitch servo → Pin 9, Roll servo → Pin 10
  Serial: "P:<pitch>,R:<roll>\n" @ 20Hz
  Commands: 0=PP 1=LQR 2=LQG r=reset c=recalibrate
================================================================
*/

#include <Wire.h>
#include <Servo.h>
#include <avr/wdt.h>

#define K0_PP   0.7169f
#define K1_PP   0.0327f
#define K0_LQR  2.1050f
#define K1_LQR  0.1475f
#define K0_LQG  2.1050f
#define K1_LQG  0.1475f
#define L0_KAL  0.992877f
#define L1_KAL  10.300535f
#define G_OVER_L 245.25f
#define INV_MLL  2394.64f

#define MPU_ADDR    0x68
#define GYRO_SENS   131.0f
#define LOOP_HZ     200
#define LOOP_US     (1000000UL / LOOP_HZ)
#define TELEM_EVERY 10
#define FUSE_ALPHA  0.98f
#define FUSE_BETA   0.02f

#define TCA_ADDR        0x70
#define ENC_ADDR        0x36
#define PITCH_CHANNEL   0
#define ROLL_CHANNEL    1
#define CPR             4096.0f

#define PITCH_SERVO_PIN  9
#define ROLL_SERVO_PIN   10
#define SERVO_CENTER_US  1500
#define SERVO_RANGE_US   300
#define CMD_LIMIT        25.0f

Servo pitchServo;
Servo rollServo;
int controllerMode = 1;

float pitchDeg=0, rollDeg=0, gyroPitchDps=0, gyroRollDps=0;
float pitchRad=0, pitchDot=0, rollRad=0, rollDot=0;
float filteredPitchCmd=0, filteredRollCmd=0;
float alphaEst=0, alphaDotEst=0, betaEst=0, betaDotEst=0;
float pitchZero=0, rollZero=0;
uint32_t lastLoopUs=0;

void tcaSelect(uint8_t ch) {
    Wire.beginTransmission(TCA_ADDR);
    Wire.write(1<<ch); Wire.endTransmission();
}

float readEncDeg(uint8_t ch) {
    tcaSelect(ch);
    Wire.beginTransmission(ENC_ADDR);
    Wire.write(0x0C); Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)ENC_ADDR,(uint8_t)2);
    uint16_t raw=((Wire.read()<<8)|Wire.read())&0x0FFF;
    return (raw/CPR)*360.0f;
}

float wrapAngle(float d) {
    while(d> 180.0f) d-=360.0f;
    while(d<-180.0f) d+=360.0f;
    return d;
}

void mpuWrite(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(reg); Wire.write(val);
    Wire.endTransmission(true);
}

void mpuReadBytes(uint8_t reg, uint8_t n, uint8_t* buf) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(reg); Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)MPU_ADDR,n,(uint8_t)1);
    for(uint8_t i=0;i<n;i++) buf[i]=Wire.read();
}

void mpuInit() {
    mpuWrite(0x6B,0x00); delay(50);
    mpuWrite(0x1B,0x00); mpuWrite(0x1C,0x00); mpuWrite(0x1A,0x03);
}

float clampf(float x,float lo,float hi){return x<lo?lo:(x>hi?hi:x);}

void applyServo(Servo &s, float u) {
    u=clampf(u,-CMD_LIMIT,CMD_LIMIT);
    s.writeMicroseconds(SERVO_CENTER_US+(int)(u*(SERVO_RANGE_US/CMD_LIMIT)));
}

void resetState() {
    filteredPitchCmd=filteredRollCmd=0;
    alphaEst=alphaDotEst=betaEst=betaDotEst=0;
    pitchDeg=rollDeg=0;
}

void calibrate() {
    pitchZero=readEncDeg(PITCH_CHANNEL);
    rollZero =readEncDeg(ROLL_CHANNEL);
}

void setup() {
    if(MCUSR&(1<<WDRF)) MCUSR=0;
    wdt_disable(); wdt_enable(WDTO_2S);
    Serial.begin(115200);
    Wire.begin(); Wire.setClock(400000);
    mpuInit();
    pitchServo.attach(PITCH_SERVO_PIN);
    rollServo.attach(ROLL_SERVO_PIN);
    pitchServo.writeMicroseconds(SERVO_CENTER_US);
    rollServo.writeMicroseconds(SERVO_CENTER_US);
    delay(500); calibrate();
    lastLoopUs=micros(); delay(200);
}

void loop() {
    if(Serial.available()) {
        char cmd=Serial.read();
        if      (cmd=='0'){controllerMode=0;resetState();}
        else if (cmd=='1'){controllerMode=1;resetState();}
        else if (cmd=='2'){controllerMode=2;resetState();}
        else if (cmd=='r'){resetState();}
        else if (cmd=='c'){calibrate();}
    }

    uint32_t nowUs=micros();
    if((uint32_t)(nowUs-lastLoopUs)<LOOP_US) return;
    float dt=(nowUs-lastLoopUs)*1e-6f;
    lastLoopUs=nowUs;
    wdt_reset();

    uint8_t buf[14];
    mpuReadBytes(0x3B,14,buf);
    int16_t gx=(int16_t)((buf[8]<<8)|buf[9]);
    int16_t gy=(int16_t)((buf[10]<<8)|buf[11]);
    gyroPitchDps=((float)gy)/GYRO_SENS;
    gyroRollDps =((float)gx)/GYRO_SENS;

    float encPitch=wrapAngle(readEncDeg(PITCH_CHANNEL)-pitchZero);
    float encRoll =wrapAngle(readEncDeg(ROLL_CHANNEL) -rollZero);

    pitchDeg=FUSE_ALPHA*(pitchDeg+gyroPitchDps*dt)+FUSE_BETA*encPitch;
    rollDeg =FUSE_ALPHA*(rollDeg +gyroRollDps *dt)+FUSE_BETA*encRoll;

    pitchRad=pitchDeg*(PI/180.0f); pitchDot=gyroPitchDps*(PI/180.0f);
    rollRad =rollDeg *(PI/180.0f); rollDot =gyroRollDps *(PI/180.0f);

    float uPitch=0,uRoll=0;
    switch(controllerMode){
        case 0:
            uPitch=-K0_PP*pitchRad-K1_PP*pitchDot;
            uRoll =-K0_PP*rollRad -K1_PP*rollDot; break;
        case 1:
            uPitch=-K0_LQR*pitchRad-K1_LQR*pitchDot;
            uRoll =-K0_LQR*rollRad -K1_LQR*rollDot; break;
        case 2:{
            float uPp=-(K0_LQG*alphaEst+K1_LQG*alphaDotEst);
            float aP=alphaEst+dt*alphaDotEst;
            float adP=alphaDotEst+dt*(G_OVER_L*alphaEst+INV_MLL*uPp);
            float iP=pitchRad-aP;
            alphaEst=aP+L0_KAL*iP; alphaDotEst=adP+L1_KAL*iP;
            uPitch=-(K0_LQG*alphaEst+K1_LQG*alphaDotEst);

            float uPr=-(K0_LQG*betaEst+K1_LQG*betaDotEst);
            float bP=betaEst+dt*betaDotEst;
            float bdP=betaDotEst+dt*(G_OVER_L*betaEst+INV_MLL*uPr);
            float iR=rollRad-bP;
            betaEst=bP+L0_KAL*iR; betaDotEst=bdP+L1_KAL*iR;
            uRoll=-(K0_LQG*betaEst+K1_LQG*betaDotEst);
            break;
        }
    }

    filteredPitchCmd=0.7f*filteredPitchCmd+0.3f*(uPitch*(180.0f/PI));
    filteredRollCmd =0.7f*filteredRollCmd +0.3f*(uRoll *(180.0f/PI));
    applyServo(pitchServo,filteredPitchCmd);
    applyServo(rollServo, filteredRollCmd);

    static int tc=0;
    if(++tc>=TELEM_EVERY){
        tc=0;
        Serial.print("P:"); Serial.print(pitchDeg,2);
        Serial.print(",R:"); Serial.println(rollDeg,2);
    }
}
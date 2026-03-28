// TEST 4 — FULL SYSTEM (no Parallax, Serial commands)
// DRIVE: w=fwd s=back a=left d=right x=stop
// CTRL:  0=PP 1=LQR 2=LQG
// INFO:  t=telemetry p=state +=faster -=slower

#include <Wire.h>
#include <Servo.h>
#include <avr/wdt.h>

int controllerMode=0;

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

#define SERVO_PIN 9
#define ENA 5
#define IN1 23
#define IN2 22
#define ENB 6
#define IN3 25
#define IN4 24
#define ENC_LEFT  2
#define ENC_RIGHT 3
#define SERVO_CENTER_US 1500
#define SERVO_RANGE_US  300
#define CMD_LIMIT       25.0f
#define MPU_ADDR        0x68
#define GYRO_SENS       131.0f
#define COMP_ALPHA      0.98f
#define LOOP_US         5000UL
#define MAX_PWM         200
#define DEADZONE        12

Servo platformServo;
float pitchDeg=0,pitchRad=0,pitchDot=0,filteredCmd=0;
float alphaEst=0,alphaDotEst=0;
int8_t driveLeft=0,driveRight=0;
int driveSpeed=80;
volatile long encLeft=0,encRight=0;
bool telemetry=false;
uint32_t lastLoopUs=0;

void isrLeft(){encLeft++;}
void isrRight(){encRight++;}

void mpuWrite(uint8_t reg,uint8_t val){Wire.beginTransmission(MPU_ADDR);Wire.write(reg);Wire.write(val);Wire.endTransmission(true);}
void mpuReadBytes(uint8_t reg,uint8_t n,uint8_t* buf){Wire.beginTransmission(MPU_ADDR);Wire.write(reg);Wire.endTransmission(false);Wire.requestFrom(MPU_ADDR,n,true);for(uint8_t i=0;i<n;i++)buf[i]=Wire.read();}
void mpuInit(){mpuWrite(0x6B,0x00);delay(50);mpuWrite(0x1B,0x00);mpuWrite(0x1C,0x00);mpuWrite(0x1A,0x05);}
float clampf(float x,float lo,float hi){return x<lo?lo:(x>hi?hi:x);}
int clampi(int x,int lo,int hi){return x<lo?lo:(x>hi?hi:x);}
void applyServo(float u){u=clampf(u,-CMD_LIMIT,CMD_LIMIT);platformServo.writeMicroseconds(SERVO_CENTER_US+(int)(u*(SERVO_RANGE_US/CMD_LIMIT)));}
void setMotor(int en,int a,int b,int speed){int pwm=clampi(abs(speed),0,MAX_PWM);if(!pwm){digitalWrite(a,LOW);digitalWrite(b,LOW);analogWrite(en,0);return;}digitalWrite(a,speed>0?HIGH:LOW);digitalWrite(b,speed>0?LOW:HIGH);analogWrite(en,pwm);}
void stopMotors(){setMotor(ENA,IN1,IN2,0);setMotor(ENB,IN3,IN4,0);driveLeft=driveRight=0;}

void setup() {
    if(MCUSR&(1<<WDRF))MCUSR=0;wdt_disable();wdt_enable(WDTO_250MS);
    Serial.begin(115200);Wire.begin();Wire.setClock(400000);mpuInit();
    platformServo.attach(SERVO_PIN);platformServo.writeMicroseconds(SERVO_CENTER_US);
    pinMode(ENA,OUTPUT);pinMode(IN1,OUTPUT);pinMode(IN2,OUTPUT);
    pinMode(ENB,OUTPUT);pinMode(IN3,OUTPUT);pinMode(IN4,OUTPUT);
    stopMotors();
    pinMode(ENC_LEFT,INPUT_PULLUP);pinMode(ENC_RIGHT,INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(ENC_LEFT),isrLeft,RISING);
    attachInterrupt(digitalPinToInterrupt(ENC_RIGHT),isrRight,RISING);
    Serial.println("TEST 4 — FULL SYSTEM");
    Serial.println("w/s/a/d/x=drive  0/1/2=controller  t=telemetry  p=state");
    lastLoopUs=micros();delay(200);
}

void loop() {
    if(Serial.available()){
        char cmd=Serial.read();
        switch(cmd){
            case 'w':driveLeft=driveRight=driveSpeed;Serial.println("FWD");break;
            case 's':driveLeft=driveRight=-driveSpeed;Serial.println("BACK");break;
            case 'a':driveLeft=-driveSpeed;driveRight=driveSpeed;Serial.println("LEFT");break;
            case 'd':driveLeft=driveSpeed;driveRight=-driveSpeed;Serial.println("RIGHT");break;
            case 'x':stopMotors();Serial.println("STOP");break;
            case '0':controllerMode=0;filteredCmd=alphaEst=alphaDotEst=0;Serial.println("POLE PLACEMENT");break;
            case '1':controllerMode=1;filteredCmd=alphaEst=alphaDotEst=0;Serial.println("LQR");break;
            case '2':controllerMode=2;filteredCmd=alphaEst=alphaDotEst=0;Serial.println("LQG");break;
            case '+':driveSpeed=clampi(driveSpeed+10,0,MAX_PWM);Serial.print("Speed: ");Serial.println(driveSpeed);break;
            case '-':driveSpeed=clampi(driveSpeed-10,0,MAX_PWM);Serial.print("Speed: ");Serial.println(driveSpeed);break;
            case 'p':Serial.print("alpha=");Serial.print(pitchDeg,2);Serial.print(" cmd=");Serial.print(filteredCmd,2);Serial.print(" L=");Serial.print((int)driveLeft);Serial.print(" R=");Serial.println((int)driveRight);break;
            case 't':telemetry=!telemetry;Serial.println(telemetry?"Telemetry ON — ms,alpha,alphaDot,cmd,encL,encR":"Telemetry OFF");break;
        }
    }
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
    switch(controllerMode){
        case 0:pitchDeg=COMP_ALPHA*(pitchDeg+gyroDps*dt)+(1-COMP_ALPHA)*accPitch;pitchRad=pitchDeg*(PI/180);u=-K0_PP*pitchRad-K1_PP*pitchDot;break;
        case 1:pitchDeg=COMP_ALPHA*(pitchDeg+gyroDps*dt)+(1-COMP_ALPHA)*accPitch;pitchRad=pitchDeg*(PI/180);u=-K0_LQR*pitchRad-K1_LQR*pitchDot;break;
        case 2:{float up=-K0_LQG*alphaEst-K1_LQG*alphaDotEst;float aP=alphaEst+dt*alphaDotEst;float adP=alphaDotEst+dt*(G_OVER_L*alphaEst+INV_MLL*up);float inn=accPitch*(PI/180)-aP;alphaEst=aP+L0_KAL*inn;alphaDotEst=adP+L1_KAL*inn;pitchRad=alphaEst;pitchDeg=pitchRad*57.29578f;pitchDot=alphaDotEst;u=-K0_LQG*alphaEst-K1_LQG*alphaDotEst;break;}
    }
    filteredCmd=0.7f*filteredCmd+0.3f*u*(180.0f/PI);
    applyServo(filteredCmd);

    int lP=(int)driveLeft*2, rP=(int)driveRight*2;
    if(abs(lP)<DEADZONE)lP=0; if(abs(rP)<DEADZONE)rP=0;
    setMotor(ENB,IN3,IN4,lP); setMotor(ENA,IN1,IN2,rP);

    static int tc=0;
    if(telemetry&&++tc>=20){tc=0;
        Serial.print(millis());Serial.print(",");Serial.print(pitchDeg,3);Serial.print(",");
        Serial.print(pitchDot,3);Serial.print(",");Serial.print(filteredCmd,3);Serial.print(",");
        Serial.print(encLeft);Serial.print(",");Serial.println(encRight);}
}
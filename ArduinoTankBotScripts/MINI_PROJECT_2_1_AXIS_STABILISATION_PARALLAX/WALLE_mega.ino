// WALL-E MEGA — FINAL
// BT receive from Parallax → drive motors
// Read pitch from Uno → send telemetry back to Parallax
// HC-05: RX1=19 TX1=18   Uno link: RX2=17

#include <avr/wdt.h>

#define ENA 5
#define IN1 23
#define IN2 22
#define ENB 6
#define IN3 25
#define IN4 24
#define ENC_LEFT  2
#define ENC_RIGHT 3
#define MAX_PWM   200
#define DEADZONE  12
#define TANKSTATUS_PACKET_LENGTH 16
#define LOOP_MS   5

int8_t driveLeft=0,driveRight=0;
float  pitchDeg=0.0f;
volatile long encLeft=0,encRight=0;
uint8_t btBuf[TANKSTATUS_PACKET_LENGTH];
uint8_t btIdx=0;
String  unoLine="";
unsigned long lastLoop=0;

void isrLeft(){encLeft++;}
void isrRight(){encRight++;}
int clampi(int x,int lo,int hi){return x<lo?lo:(x>hi?hi:x);}

void setMotor(int en,int a,int b,int speed){
    int pwm=clampi(abs(speed),0,MAX_PWM);
    if(!pwm){digitalWrite(a,LOW);digitalWrite(b,LOW);analogWrite(en,0);return;}
    digitalWrite(a,speed>0?HIGH:LOW);digitalWrite(b,speed>0?LOW:HIGH);analogWrite(en,pwm);
}
void stopMotors(){setMotor(ENA,IN1,IN2,0);setMotor(ENB,IN3,IN4,0);}

void parseTankStatus(uint8_t* buf){
    driveLeft =(int8_t)buf[0];
    driveRight=(int8_t)buf[1];
}

void sendTankStatus(float eulerX,float eulerY,float eulerZ){
    uint8_t buf[TANKSTATUS_PACKET_LENGTH];
    memset(buf,0,TANKSTATUS_PACKET_LENGTH);
    buf[0]=(uint8_t)(int8_t)driveLeft;
    buf[1]=(uint8_t)(int8_t)driveRight;
    memcpy(buf+4, &eulerX,4);
    memcpy(buf+8, &eulerY,4);
    memcpy(buf+12,&eulerZ,4);
    Serial1.write(buf,TANKSTATUS_PACKET_LENGTH);
}

void parseUnoLine(String line){
    line.trim();
    if(line.startsWith("P:")) pitchDeg=line.substring(2).toFloat();
}

void setup() {
    if(MCUSR&(1<<WDRF))MCUSR=0;wdt_disable();wdt_enable(WDTO_250MS);
    Serial.begin(115200);
    Serial1.begin(9600);    // HC-05
    Serial2.begin(115200);  // Uno
    pinMode(ENA,OUTPUT);pinMode(IN1,OUTPUT);pinMode(IN2,OUTPUT);
    pinMode(ENB,OUTPUT);pinMode(IN3,OUTPUT);pinMode(IN4,OUTPUT);
    stopMotors();
    pinMode(ENC_LEFT,INPUT_PULLUP);pinMode(ENC_RIGHT,INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(ENC_LEFT),isrLeft,RISING);
    attachInterrupt(digitalPinToInterrupt(ENC_RIGHT),isrRight,RISING);
    Serial.println("WALL-E MEGA ready");
    lastLoop=millis();
}

void loop() {
    wdt_reset();

    // Read BT from Parallax
    while(Serial1.available()){
        uint8_t c=Serial1.read();
        btBuf[btIdx++]=c;
        if(btIdx>=TANKSTATUS_PACKET_LENGTH){parseTankStatus(btBuf);btIdx=0;}
    }

    // Read pitch from Uno
    while(Serial2.available()){
        char c=(char)Serial2.read();
        if(c=='\n'){parseUnoLine(unoLine);unoLine="";}
        else unoLine+=c;
    }

    unsigned long now=millis();
    if((now-lastLoop)<LOOP_MS)return;
    lastLoop=now;

    // Drive motors
    int lP=(int)driveLeft*2, rP=(int)driveRight*2;
    if(abs(lP)<DEADZONE)lP=0; if(abs(rP)<DEADZONE)rP=0;
    setMotor(ENB,IN3,IN4,lP);
    setMotor(ENA,IN1,IN2,rP);

    // Send telemetry to Parallax (20Hz)
    static int tc=0;
    if(++tc>=10){tc=0;sendTankStatus(pitchDeg,0.0f,90.0f);}

    // USB debug (5Hz)
    static int dc=0;
    if(++dc>=40){dc=0;
        Serial.print("pitch=");Serial.print(pitchDeg,2);
        Serial.print(" L=");Serial.print((int)driveLeft);
        Serial.print(" R=");Serial.println((int)driveRight);}
}
// TEST 1 — MOTORS ONLY
// Commands: f=fwd b=back l=left r=right s=stop +=faster -=slower

#include <avr/wdt.h>

#define ENA 5
#define IN1 23
#define IN2 22
#define ENB 6
#define IN3 25
#define IN4 24
#define ENC_LEFT  2
#define ENC_RIGHT 3

volatile long encLeft = 0, encRight = 0;
int currentSpeed = 150;

void isrLeft()  { encLeft++;  }
void isrRight() { encRight++; }

int clampi(int x, int lo, int hi) { return x<lo?lo:(x>hi?hi:x); }

void setMotor(int en, int a, int b, int speed) {
    int pwm = clampi(abs(speed), 0, 200);
    if (pwm == 0) { digitalWrite(a,LOW); digitalWrite(b,LOW); analogWrite(en,0); return; }
    digitalWrite(a, speed>0?HIGH:LOW);
    digitalWrite(b, speed>0?LOW:HIGH);
    analogWrite(en, pwm);
}

void stopMotors() {
    setMotor(ENA,IN1,IN2,0);
    setMotor(ENB,IN3,IN4,0);
}

void setup() {
    Serial.begin(115200);
    pinMode(ENA,OUTPUT); pinMode(IN1,OUTPUT); pinMode(IN2,OUTPUT);
    pinMode(ENB,OUTPUT); pinMode(IN3,OUTPUT); pinMode(IN4,OUTPUT);
    stopMotors();
    pinMode(ENC_LEFT,INPUT_PULLUP);
    pinMode(ENC_RIGHT,INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(ENC_LEFT),  isrLeft,  RISING);
    attachInterrupt(digitalPinToInterrupt(ENC_RIGHT), isrRight, RISING);
    Serial.println("TEST 1 — MOTORS");
    Serial.println("f=fwd b=back l=left r=right s=stop +=faster -=slower");
}

void loop() {
    static unsigned long lastPrint = 0;
    if (millis()-lastPrint > 500) {
        lastPrint = millis();
        Serial.print("encL="); Serial.print(encLeft);
        Serial.print(" encR="); Serial.print(encRight);
        Serial.print(" speed="); Serial.println(currentSpeed);
    }
    if (!Serial.available()) return;
    char cmd = Serial.read();
    switch(cmd) {
        case 'f': setMotor(ENA,IN1,IN2, currentSpeed); setMotor(ENB,IN3,IN4, currentSpeed); Serial.println("FORWARD");  break;
        case 'b': setMotor(ENA,IN1,IN2,-currentSpeed); setMotor(ENB,IN3,IN4,-currentSpeed); Serial.println("BACKWARD"); break;
        case 'l': setMotor(ENA,IN1,IN2,-currentSpeed); setMotor(ENB,IN3,IN4, currentSpeed); Serial.println("LEFT");     break;
        case 'r': setMotor(ENA,IN1,IN2, currentSpeed); setMotor(ENB,IN3,IN4,-currentSpeed); Serial.println("RIGHT");    break;
        case 's': stopMotors(); Serial.println("STOP"); break;
        case '+': currentSpeed=clampi(currentSpeed+20,0,200); Serial.print("Speed: "); Serial.println(currentSpeed); break;
        case '-': currentSpeed=clampi(currentSpeed-20,0,200); Serial.print("Speed: "); Serial.println(currentSpeed); break;
    }
}
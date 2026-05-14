#include "src/TankStatusClass/TankStatusClass.h"

// ── STATE ─────────────────────────────────────────────────────
TankStatusClass tsLocalIn;  // For receiving data from Python/ROS
TankStatusClass tsLocalOut; // For sending mock encoder data back

uint8_t  pktBuf[TANKSTATUS_PACKET_LENGTH];
uint8_t  pktIdx     = 0;
unsigned long lastLoop  = 0;
#define LOOP_MS 5

// ── MOCK ENCODER VARIABLES ────────────────────────────────────
int mockLeft = 0;
int mockRight = 255;
int leftStep = 1;
int rightStep = -1;


void updateOdometry() {
    unsigned long now = millis();
    float dt = (now - lastEncTime) / 1000.0f;
    if (dt < 0.01f) return;
    lastEncTime = now;

    uint16_t leftAngle  = readAngle(LEFT_CHANNEL);
    uint16_t rightAngle = readAngle(RIGHT_CHANNEL);

    int16_t leftDiff  = angleDiff(leftAngle,  leftAnglePrev);
    int16_t rightDiff = angleDiff(rightAngle, rightAnglePrev);

    leftAnglePrev  = leftAngle;
    rightAnglePrev = rightAngle;

    float leftDist  = (leftDiff  / CPR) * 2.0f * M_PI * WHEEL_RADIUS;
    float rightDist = (rightDiff / CPR) * 2.0f * M_PI * WHEEL_RADIUS;

    int leftVel  = leftDist  / dt;
    int rightVel = rightDist / dt;



    tsLocalOut.driveLeft = static_cast<unsigned char>(map(leftVel,0,MAX_VELOCITY,0,MAX_PWM));
    tsLocalOut.driveRight = static_cast<unsigned char>(map(rightDiff,0,MAX_VELOCITY,0,MAX_PWM));
}

// ── MOTOR DRIVER ──────────────────────────────────────────────
int clampi(int x, int lo, int hi) {
    return x < lo ? lo : (x > hi ? hi : x);
}
void setMotor(int en, int a, int b, int spd) {
    int pwm = abs(spd);
    if (pwm > 250) pwm = 250; // Safety cap

    if (pwm == 0) {
        digitalWrite(a, LOW); 
        digitalWrite(b, LOW);
        analogWrite(en, 0); 
        return;
    }

    // Direction Logic
    if (spd > 0) {
        digitalWrite(a, HIGH);
        digitalWrite(b, LOW);
    } else {
        digitalWrite(a, LOW);
        digitalWrite(b, HIGH);
    }
    analogWrite(en, pwm);
}

void stopMotors() {
    setMotor(ENA, IN1, IN2, 0);
   setMotor(ENB, IN3, IN4, 0);
}



void setup()
{
  Serial.begin(115200);
  tsLocalIn = TankStatusClass();
  tsLocalOut = TankStatusClass();

}

// ── LOOP ──────────────────────────────────────────────────────
void loop() {
    // 1. Read incoming TankStatus packets (to test receiving)
    while (Serial.available()) {
        uint8_t c = Serial.read();
        pktBuf[pktIdx++] = c;
        if (pktIdx >= TANKSTATUS_PACKET_LENGTH) {
            tsLocalIn.BuildFromBytes(pktBuf);
            pktIdx = 0; 
        }
    }

    // 2. Timing loop execution (5ms)
    unsigned long now = millis();
    if ((now - lastLoop) < LOOP_MS) return;
    lastLoop = now;

    // --- EVERYTHING BELOW HERE HAPPENS EXACTLY ONCE EVERY 5ms ---

    static int odomCount = 0;
    if (++odomCount >= 10) { // Send every 10 loops (50ms / 20Hz)
        odomCount = 0;
        
        // 3. Set values right before you send them
        // WARNING: If these still send as 0, check TankStatusClass.h to make sure
        // you aren't supposed to be doing tsLocalOut.ts.driveLeft = 10;
        tsLocalOut.driveLeft = 0;//tsLocalIn.driveLeft; 
        tsLocalOut.driveRight = 254;//tsLocalIn.driveRight;

        // 4. Stream packet back
        unsigned char* txBuffer = tsLocalOut.MakeIntoBytes();
        Serial.write(txBuffer, TANKSTATUS_PACKET_LENGTH); 
    }
}


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
        tsLocalOut.driveLeft = tsLocalIn.driveLeft; 
        tsLocalOut.driveRight = tsLocalIn.driveRight;

        // 4. Stream packet back
        unsigned char* txBuffer = tsLocalOut.MakeIntoBytes();
        Serial.write(txBuffer, TANKSTATUS_PACKET_LENGTH); 
    }
}


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
            // You could optionally print/use tsLocalIn data here if testing two-way
        }
    }
    //Serial.println(TANKSTATUS_PACKET_LENGTH);

    unsigned long nowBlink = millis();
    // 2. Timing loop execution (5ms)
    unsigned long now = millis();
    if ((now - lastLoop) < LOOP_MS) return;
    lastLoop = now;

    //echo
    tsLocalOut.driveLeft = tsLocalIn.driveLeft;
    tsLocalOut.driveRight = tsLocalIn.driveRight;

   // 5. Stream packet back using MakeIntoBytes()
    static int odomCount = 0;
    if (++odomCount >= 10) { // Send every 10 loops (50ms / 20Hz)
        odomCount = 0;
        
        // Grab the byte array directly from the OUT instance
        unsigned char* txBuffer = tsLocalOut.MakeIntoBytes();
        

        // Write the whole struct payload seamlessly
        Serial.write(txBuffer, TANKSTATUS_PACKET_LENGTH);
    }
}

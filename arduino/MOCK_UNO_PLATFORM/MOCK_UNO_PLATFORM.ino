#include "src/TankStatusClass/TankStatusClass.h"

// ── STATE ─────────────────────────────────────────────────────
TankStatusClass tsLocalIn;  // For receiving data from Python/ROS
TankStatusClass tsLocalOut; // For sending data back to Python/ROS

uint8_t  pktBuf[TANKSTATUS_PACKET_LENGTH];
uint8_t  pktIdx     = 0;
unsigned long lastLoop  = 0;
#define LOOP_MS 5

// ── MOCK IMU VARIABLES ────────────────────────────────────────
float mockEulerX = 0.0;
float mockEulerY = 0.0;
float xIncrement = 0.5; // Change per 50ms
float yIncrement = 0.2;

void setup()
{
  Serial.begin(115200);
  tsLocalIn = TankStatusClass();
  tsLocalOut = TankStatusClass();
}

// ── LOOP ──────────────────────────────────────────────────────
void loop() {
    // 1. Read incoming TankStatus packets
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

    static int streamCount = 0;
    if (++streamCount >= 10) { // Send every 10 loops (50ms / 20Hz)
        streamCount = 0;
        
        // 3. Update mock values to simulate "changing" sensor data
        mockEulerX += xIncrement;
        mockEulerY += yIncrement;

        // Reset if they go out of typical Euler bounds (-180 to 180)
        if (mockEulerX > 180.0) mockEulerX = -180.0;
        if (mockEulerY > 90.0)  mockEulerY = -90.0;

        // 4. Assign to the output object
        // Adjust these member names if your Class uses different labels (e.g., ts.roll / ts.pitch)
        tsLocalOut.eulerXFloat = mockEulerX;
        tsLocalOut.eulerYFloat = mockEulerY;
        tsLocalOut.eulerYFloat = mockEulerY;

        // 5. Stream packet back
        unsigned char* txBuffer = tsLocalOut.MakeIntoBytes();
        Serial.write(txBuffer, TANKSTATUS_PACKET_LENGTH); 
    }
}

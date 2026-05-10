# WALL-E Tank Project API Specification

This document defines the JSON data formats used for communication between the GUI and the WebSocket Gateway.

---

## 1. Tank Control & Telemetry
**Service**: Tank Motor Control & Sensor Data  
**Default Port**: 8881

### Outgoing Commands (GUI -> Server)
Sent when a direction button is pressed. All values are integers.
```json
{
  "driveLeft": 255,   // 0 (Full Back) to 255 (Full Fwd), 127 is Stop
  "driveRight": 255,  // 0 (Full Back) to 255 (Full Fwd), 127 is Stop
  "changeFlag": 1     // 1 to indicate a new command
}
```

### Incoming Telemetry (Server -> GUI)
Streamed status updates from the tank. Values are floats or bytes.
```json
{
  "driveLeft": 127,
  "driveRight": 127,
  "eulerX": 0.0,      // Orientation Pitch
  "eulerY": 0.0,      // Orientation Yaw
  "eulerZ": 0.0       // Orientation Roll
}
```

---

## 2. Chat & AI Service
**Service**: AI Interaction / LLM Gateway  
**Default Port**: 8000 (Endpoint: `/ws/chat`)

### Outgoing Messages (GUI -> Server)
Simple text-wrapped message object.
```json
{
  "message": "Move forward and say hello"
}
```

### Incoming Responses (Server -> GUI)
The server streams the AI response in chunks to allow for real-time typing effects.

**Content Chunk**:
```json
{
  "type": "chunk",
  "content": "Hello! "
}
```

**Completion Signal**:
```json
{
  "type": "done"
}
```

**Error Signal**:
```json
{
  "type": "error",
  "content": "Detailed error message here"
}
```

# WALL-E Treadbot — Control System README
## Platform Stabilizing Stair Climbing Bot

---

## Hardware Overview

```
Arduino UNO
  ├── MPU6050 IMU     (SDA=A4, SCL=A5)
  └── DSServo 25kg    (pin 9)

Arduino MEGA
  ├── L298N left motor  (ENA=5, IN1=23, IN2=22)
  ├── L298N right motor (ENB=6, IN3=25, IN4=24)
  ├── Encoder left      (pin 2, interrupt)
  ├── Encoder right     (pin 3, interrupt)
  ├── HC-05 Bluetooth   (RX1=19, TX1=18)
  └── Link to Uno       (RX2=17)

Parallax Controller
  └── HC-06 Bluetooth → sends/receives TankStatus 16-byte packets
```

---

## Wiring Diagram

```
┌──────────────────────────────────────────┐
│             ARDUINO UNO                  │
│  A4 (SDA) ──────────────→ MPU6050 SDA   │
│  A5 (SCL) ──────────────→ MPU6050 SCL   │
│  3.3V     ──────────────→ MPU6050 VCC   │
│  GND      ──────────────→ MPU6050 GND   │
│  Pin 9    ──────────────→ Servo Signal   │
│  5V       ──────────────→ Servo VCC      │
│  GND      ──────────────→ Servo GND      │
│  Pin 1 (TX)─────────────→ Mega Pin 17   │
│  GND      ──────────────→ Mega GND      │
└──────────────────────────────────────────┘

┌──────────────────────────────────────────┐
│             ARDUINO MEGA                 │
│  Pin 19 (RX1) ←──────── HC-05 TX        │
│  Pin 18 (TX1) ──→[DIV]─→ HC-05 RX       │
│  Pin 17 (RX2) ←──────── Uno Pin 1 (TX)  │
│  GND          ──────────→ Uno GND        │
│  Pin 5  (ENA) ──────────→ L298N ENA      │
│  Pin 23 (IN1) ──────────→ L298N IN1      │
│  Pin 22 (IN2) ──────────→ L298N IN2      │
│  Pin 6  (ENB) ──────────→ L298N ENB      │
│  Pin 25 (IN3) ──────────→ L298N IN3      │
│  Pin 24 (IN4) ──────────→ L298N IN4      │
│  Pin 2        ←──────── Encoder Left     │
│  Pin 3        ←──────── Encoder Right    │
└──────────────────────────────────────────┘

┌──────────────────────────────────────────┐
│                HC-05                     │
│  VCC ──────────────────→ Mega 5V         │
│  GND ──────────────────→ Mega GND        │
│  TX  ──────────────────→ Mega RX1 (19)   │
│  RX  ──→ 1kΩ ──┬──────→ Mega TX1 (18)   │
│                2kΩ                       │
│                 │                        │
│                GND  ← voltage divider!   │
│  (Mega=5V logic, HC-05 RX wants 3.3V)   │
└──────────────────────────────────────────┘
```

---

## The Control System

### Plant Model (from Lagrangian mechanics)

Your platform is modeled as an inverted pendulum on a moving cart:

```
Equation of motion:
  ml²α̈ = τ + mglα - ml²φ̈

Linearized state space:
  ẋ = Ax + Bu

  A = [  0      1   ]     B = [    0      ]
      [ g/l     0   ]         [ 1/(ml²)   ]

States:  x = [α, α̇]   (platform angle, angular rate)
Input:   u = τ         (servo torque)
```

Open loop eigenvalues = ±√(g/l) = ±11.84 rad/s
→ positive eigenvalue confirms instability → controller needed

### Your Parameters (UPDATE THESE after measuring)

```
m = 0.50 kg   ← platform plate + payload mass (weigh with scale)
l = 0.07 m    ← servo shaft to combined CoM   (measure with ruler)
```

To update gains after measuring:
1. Edit lines 8-9 in validate_controllers.py
2. Run: python3 validate_controllers.py
3. Copy new K values into UNO sketch #defines

---

## Controllers

### Approach 1 — Pole Placement
Manually places closed loop poles 2-3x faster than instability rate.
```
Poles: [-23.68, -35.51]
K0 = 2.4034   (angle gain)
K1 = 0.1450   (rate damping)
Settle time: 0.138s    Peak torque: 0.42 N·m ✓
```
Start here. Most conservative. Easiest to tune.

### Approach 2 — LQR
Optimal gains from minimizing cost function J = ∫(200α² + α̇² + 5τ²)dt
```
K0 = 6.6772   (angle gain)
K1 = 0.4824   (rate damping)
Settle time: 0.169s    Peak torque: 1.17 N·m ✓
```
Use this for rough terrain. More aggressive correction.

### Approach 3 — LQG (Kalman + LQR)
Same K gains as LQR but Kalman filter estimates states from noisy IMU.
```
K0 = 6.6772,  K1 = 0.4824  (same as LQR)
L0 = 0.9929   (Kalman angle correction)
L1 = 10.030   (Kalman rate correction)
```
Use this if IMU is noisy/jittery on real hardware.

### Switching Controllers (UNO sketch)
Change one line at top of WALL_E_UNO.ino:
```cpp
#define CONTROLLER_MODE 0   // Pole Placement ← start here
#define CONTROLLER_MODE 1   // LQR
#define CONTROLLER_MODE 2   // LQG
```

---

## Data Flow

```
Parallax joystick
      ↓ Bluetooth (16-byte TankStatus packet)
    HC-05
      ↓ Serial1 (9600 baud)
  MEGA: parse driveLeft, driveRight
      ↓
  MEGA: drive L298N motors

  UNO: read MPU6050
      ↓ complementary filter
  UNO: α, α̇ (platform angle + rate)
      ↓ LQR/PP/LQG controller
  UNO: command DSServo (platform levels out)
      ↓ Serial (115200 baud) "P:12.34\n"
  MEGA: receives pitchDeg from Uno
      ↓ packs into TankStatus eulerX
  MEGA → HC-05 → BT → Parallax TFT displays pitch
```

### TankStatus Packet Format (16 bytes)
```
Byte 0      driveLeft   int8   (-128 to +127)
Byte 1      driveRight  int8   (-128 to +127)
Bytes 2-3   padding     2 bytes
Bytes 4-7   eulerX      float  (platform pitch α)
Bytes 8-11  eulerY      float  (roll, always 0.0)
Bytes 12-15 eulerZ      float  (yaw, always 90.0)
```

---

## File List

### Python (run on laptop)
```
validate_controllers.py    Design all 3 controllers, plot step response
payload_sensitivity.py     Check gains work across different payload weights
```

### Arduino UNO (IMU + Stabilizer)
```
test2_servo_imu.ino        Hardware check: IMU reads angle, servo moves
test3_stabilizer_only.ino  Controller only: tilt platform, servo resists
                           Switch PP/LQR/LQG live via Serial monitor
WALL_E_UNO.ino             FINAL: full stabilizer, sends pitch to Mega
```

### Arduino MEGA (Motors + BT)
```
test1_motors.ino           Hardware check: motors spin, encoders count
test4_full_system.ino      Full system via Serial (no Parallax needed)
                           Drive with w/a/s/d, switch controllers 0/1/2
WALL_E_MEGA.ino            FINAL: BT receive, motors, telemetry to Parallax
```

---

## Testing Order — Follow This Exactly

### Step 0 — Run Python simulation first
```bash
cd your_project_folder
source ~/venv_controls/bin/activate    # activate virtual env
python3 validate_controllers.py        # generates controller_validation.png
python3 payload_sensitivity.py         # generates payload_sensitivity.png
deactivate
```
Confirm all 9 plots are green (servo-feasible).

---

### Step 1 — Test UNO hardware (test2_servo_imu.ino)
```
Flash: test2_servo_imu.ino → Arduino UNO
Open: Serial monitor at 115200 baud

Type 't' → streaming on
Tilt platform forward  → angle should go POSITIVE
Tilt platform backward → angle should go NEGATIVE
Flat on table          → angle should be ~0°

Type '1' → servo moves to +10°
Type 'a' → servo moves to -10°
Type 'c' → servo centers

Pass criteria:
  □ Angle reads correctly when tilted
  □ Servo physically moves both directions
  □ Flat = 0° (within ±2°)
```

---

### Step 2 — Test MEGA motors (test1_motors.ino)
```
Flash: test1_motors.ino → Arduino MEGA
Open: Serial monitor at 115200 baud

Type 'f' → both wheels forward
Type 'b' → both wheels backward
Type 'l' → left turn
Type 'r' → right turn
Type 's' → stop
Type '+' → increase speed
Type '-' → decrease speed

Pass criteria:
  □ Both motors spin correct direction
  □ Encoders count up as wheels turn
  □ No one motor fighting the other
  □ Speed ramps work
```

---

### Step 3 — Test controller alone on UNO (test3_stabilizer_only.ino)
```
Flash: test3_stabilizer_only.ino → Arduino UNO
Open: Serial monitor at 115200 baud

Type 'd' → debug stream on
Tilt platform 10° → servo should push back to level
Type '0' → Pole Placement mode
Type '1' → LQR mode (more aggressive)
Type '2' → LQG mode
Type 'p' → print current state

Pass criteria:
  □ Platform tilts → servo actively resists
  □ Platform returns to ~0° within 0.2s
  □ No oscillation (if oscillating → increase K1)
  □ All 3 modes switch cleanly
```

---

### Step 4 — Full system without Parallax (test4_full_system.ino)
```
Flash: test4_full_system.ino → Arduino MEGA
Connect: UNO TX (pin 1) → Mega RX2 (pin 17)
Connect: GND → GND
Open: Serial monitor on MEGA at 115200 baud

Type 't' → telemetry stream on (CSV format)
Type 'w' → forward
Type 'a' → left
Type 'd' → right
Type 's' → backward
Type 'x' → stop
Type '0'/'1'/'2' → switch controllers
Type 'p' → print full state

Pass criteria:
  □ Motors respond to wasd
  □ Servo stabilizes while driving
  □ Pitch value updates from Uno
  □ Telemetry CSV shows reasonable numbers
  □ Controller switch works live
```

---

### Step 5 — Flash final code
```
Flash: WALL_E_UNO.ino  → Arduino UNO
Flash: WALL_E_MEGA.ino → Arduino MEGA

Wire HC-05 to Mega (see wiring diagram above)

Power on → Parallax connects via BT
Joystick → bot drives
Platform tilts → servo corrects
TFT display shows live pitch angle
```

---

## Tuning Guide (on real hardware)

### If platform oscillates:
```
Symptom: servo hunts back and forth constantly
Fix: increase K1 (rate damping)
     lower COMP_ALPHA (trust accel more: 0.95 instead of 0.98)
     increase low-pass filter weight (0.8 instead of 0.7)
```

### If platform too slow to correct:
```
Symptom: platform drifts before servo catches it
Fix: increase K0 (angle gain)
     decrease R in Python script, rerun, get new gains
```

### If servo buzzes/jitters:
```
Symptom: servo making noise even when platform is level
Fix: add deadband in applyServo():
     if (abs(u) < 0.5) u = 0;   // ignore tiny commands
     switch to LQG (Kalman filters IMU noise)
```

### If 3° offset (your previous milestone bug):
```
Add trim variable in UNO sketch:
     #define ANGLE_OFFSET_DEG  3.0f
     float err = pitchDeg - ANGLE_OFFSET_DEG;
Adjust sign until platform sits level at rest.
```

### Tuning order:
```
1. Fix offset (ANGLE_OFFSET_DEG)
2. Tune K1 first (eliminate oscillation)
3. Tune K0 second (improve response speed)
4. Adjust low-pass filter weight last (smooth out noise)
```

---

## Common Errors

| Error | Cause | Fix |
|---|---|---|
| MPU6050 not found | Wrong I2C wiring | Check SDA=A4, SCL=A5, VCC=3.3V |
| Servo doesn't move | Wrong pin or no power | Check pin 9, servo needs 5-6V |
| Angle flipped | IMU mounted upside down | Flip sign: `accPitch = -accPitch` |
| Motors wrong direction | IN1/IN2 swapped | Swap IN1↔IN2 in code or physically |
| Encoders not counting | No INPUT_PULLUP | Already set in code, check wire |
| HC-05 not connecting | Wrong baud or no divider | Default HC-05 = 9600 baud, add voltage divider on RX |
| Uno pitch not reaching Mega | Missing GND wire | GND must be shared between boards |

---

## Notes for Parallax Integration

Your teammate's Parallax code sends/receives the same 16-byte TankStatus struct.
The Mega handles all BT parsing — your teammate does not need to change anything.

The Mega sends back:
```
eulerX = pitchDeg  (platform pitch from UNO IMU)
eulerY = 0.0       (roll, 1-axis bot)
eulerZ = 90.0      (yaw default)
```

These map directly to the telemetry fields the Parallax TFT displays.
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

```
Equation of motion:
  ml²α̈ = τ + mglα - ml²φ̈

Linearized state space:
  A = [  0      1   ]     B = [    0      ]
      [ g/l     0   ]         [ 1/(ml²)   ]

States:  x = [α, α̇]   (platform angle, angular rate)
Input:   u = τ         (servo torque)
```

Open loop eigenvalues = ±√(g/l) = ±15.7 rad/s
→ positive eigenvalue confirms instability → controller needed

### Measured Parameters

```
m = 0.261 kg  ← platform + payload (weigh with scale)
l = 0.04  m   ← servo shaft to combined CoM (measure with ruler)
```

To update gains after re-measuring:
1. Edit m and l in validate_controllers.py
2. Run: python3 validate_controllers.py
3. Copy new K values into UNO sketch #defines

---

## Controllers

### Pole Placement
```
K0 = 0.7169   K1 = 0.0327
Poles: [-31.4, -47.1]
Conservative. Start here.
```

### LQR
```
K0 = 2.1050   K1 = 0.1475
Q = diag[200,1]   R = 50
More aggressive. Use for rough terrain.
```

### LQG (Kalman + LQR)
```
K0 = 2.1050   K1 = 0.1475   (same as LQR)
L0 = 0.9929   L1 = 10.300
Use when tread vibration causes IMU noise.
```

### Switch controllers live in test3 (Serial monitor):
```
Type '0' → Pole Placement
Type '1' → LQR
Type '2' → LQG
```

### Switch in final UNO sketch (reflash required):
```cpp
#define CONTROLLER_MODE 0   // Pole Placement
#define CONTROLLER_MODE 1   // LQR
#define CONTROLLER_MODE 2   // LQG
```

---

## Data Flow

```
Parallax joystick  OR  laptop bt_controller.py
      ↓ Bluetooth (16-byte TankStatus packet)
    HC-05 on Mega
      ↓ Serial1 (9600 baud)
  MEGA: parse driveLeft, driveRight → drive motors

  UNO: MPU6050 → complementary filter → α, α̇
      ↓ PP / LQR / LQG controller
  UNO: command DSServo → platform levels out
      ↓ Serial "P:12.34\n" to Mega
  MEGA: pitch → TankStatus eulerX → BT → Parallax TFT
```

### TankStatus Packet (16 bytes)
```
Byte 0      driveLeft   int8
Byte 1      driveRight  int8
Bytes 2-3   padding
Bytes 4-7   eulerX      float  ← platform pitch
Bytes 8-11  eulerY      float  ← always 0.0
Bytes 12-15 eulerZ      float  ← always 90.0
```

---

## File List

### Python
```
validate_controllers.py        Design controllers, plot step response
payload_sensitivity.py         Check gains across payload weights
bt_controller.py               Drive bot from laptop over BT (no Parallax)
```

### Arduino UNO
```
test2_servo_imu.ino            Hardware check: IMU + servo
test3_stabilizer_only.ino      Controller test: tilt → servo resists
                               Switch PP/LQR/LQG live, no motors needed
WALL_E_UNO.ino                 FINAL: stabilizer + sends pitch to Mega
```

### Arduino MEGA
```
test1_motors.ino               Hardware check: motors + encoders
mega_motor_test_sequence.ino   5s countdown → Fwd 3min → Left 2s → Right 2s → Stop
test4_full_system.ino          Full system via Serial (no Parallax, no BT)
WALL_E_MEGA.ino                FINAL: BT receive + motors + telemetry
```

---

## Testing Order

### Step 0 — Python simulation
```bash
source ~/venv_controls/bin/activate
python3 validate_controllers.py
python3 payload_sensitivity.py
```
All 9 plots should be green.

---

### Step 1 — UNO hardware (test2_servo_imu.ino)
```
t → stream on → tilt platform → angle changes?
1/a → servo moves +/- direction?
Pass: angle correct, servo moves both ways
```

---

### Step 2 — MEGA motors (test1_motors.ino)
```
f/b/l/r/s → motors respond?
Encoders count when wheels spin?
Pass: both motors correct direction, encoders count
```

---

### Step 3 — Stabilizer only (test3_stabilizer_only.ino)
```
Flash UNO. No motors, no BT needed.
d → stream on → tilt platform → servo fights back?
0/1/2 → switch controllers live
Pass: platform returns to 0° within 0.2s, no oscillation
Note: Pole Placement confirmed working.
```

---

### Step 4 — Motor sequence without wires (mega_motor_test_sequence.ino)
```
Flash MEGA. Power from battery. Remove USB after flash.
5s countdown → Forward 3min → Left 2s → Right 2s → Stop
Type 'stop' to abort, 'test' to repeat
Pass: straight line forward, clean left/right turns
```

---

### Step 5 — Full system via Serial (test4_full_system.ino)
```
Flash MEGA. Wire Uno TX → Mega RX2.
w/a/s/d/x → drive motors
0/1/2 → switch controllers
t → telemetry CSV stream
Pass: motors + servo working together
```

---

### Step 6 — Laptop BT terminal (bt_controller.py)
```
Use this to control bot wirelessly without Parallax.

One-time Ubuntu setup:
  sudo apt install bluez
  bluetoothctl
    power on
    scan on
    pair XX:XX:XX:XX:XX:XX      ← HC-05 MAC address
    trust XX:XX:XX:XX:XX:XX
    quit
  sudo rfcomm bind 0 XX:XX:XX:XX:XX:XX
  sudo usermod -a -G dialout $USER   ← log out/in after this

Run:
  source ~/venv_controls/bin/activate
  pip install pyserial
  python3 bt_controller.py

Keys:
  w=forward  s=backward  a=left  d=right
  x=stop     +=faster    -=slower
  t=run 3-minute test sequence
  q=quit

Sends same 16-byte TankStatus as Parallax.
Mega code unchanged — works with both.
```

---

### Step 7 — Final code
```
Flash WALL_E_UNO.ino  → UNO
Flash WALL_E_MEGA.ino → MEGA
Wire everything. Power from battery.
Parallax connects via BT → full system running.
```

---

## Tuning Guide

| Symptom | Fix |
|---|---|
| Oscillates | Increase K1 or increase R in Python |
| Too slow to correct | Increase K0 or decrease R in Python |
| Servo buzzes at rest | Add deadband: if(abs(u)<0.5) u=0 |
| 3° offset at rest | Add: #define ANGLE_OFFSET_DEG 3.0f |
| LQG gives NaN | Check l value — must be 0.03-0.06m |
| LQR still slow after retuning | Try Q=diag[500,1] R=10 |

Tuning order: fix offset → tune K1 → tune K0 → adjust filter weight

---

## Common Errors

| Error | Cause | Fix |
|---|---|---|
| MPU6050 not found | Wrong I2C wiring | SDA=A4, SCL=A5, VCC=3.3V |
| Servo doesn't move | Wrong pin or no power | Pin 9, servo needs 5-6V |
| Angle flipped | IMU upside down | accPitch = -accPitch |
| Motors wrong direction | IN pins swapped | Swap IN1↔IN2 |
| HC-05 not connecting | Wrong baud | Default 9600, add voltage divider |
| Uno pitch not reaching Mega | Missing GND wire | Share GND between boards |
| BT not pairing | Wrong PIN | Default PIN: 1234 |
| rfcomm0 not found | BT not bound | sudo rfcomm bind 0 MAC |

---

## Parallax Integration Notes

Mega sends back to Parallax TFT:
```
eulerX = pitchDeg   (platform pitch from UNO)
eulerY = 0.0        (roll — 1-axis bot)
eulerZ = 90.0       (yaw default)
```

bt_controller.py sends the same 16-byte format as Parallax.
Mega code works identically with either source.
Parallax teammate does not need to change anything.
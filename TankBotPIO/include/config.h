#pragma once

#define PS2_DAT 38
#define PS2_CMD 39
#define PS2_SEL 40
#define PS2_CLK 41

#define ENA 5
#define IN1 23
#define IN2 22

#define ENB 6
#define IN3 25
#define IN4 24

// ===================== Servo =====================
#define SERVO_PIN 9
#define SERVO_CENTER_US 1500
#define SERVO_RANGE_US 300 // +/- around center

// ===================== MPU6050 =====================
#define MPU_ADDR 0x68
#define GYRO_SENS 131.0f // LSB/(deg/s) for +/-250 dps

// ===================== Stabilizer tuning =====================
#define TARGET_PITCH_DEG 0.0f
#define KP 3.0f
#define KD 0.08f
#define KI 0.01f
#define CMD_LIMIT 25.0f
#define COMP_ALPHA 0.98f

// IMU loop
#define IMU_HZ 200
#define IMU_US 1000000UL / IMU_HZ

// PS2 loop (separate, to avoid lag)

// #define PS2_MS 10 // 100 Hz
#define PS2_MS 50 // 100 Hz

// Motor control tuning
#define DEADZONE 12
#define RAMP_STEP 30 // faster response (was 12)
#define DBG_MS 120
// TimerInturupt
#define USE_TIMER_1 false
#define USE_TIMER_2 false
#define USE_TIMER_3 false
#define USE_TIMER_4 false
#define USE_TIMER_5 true

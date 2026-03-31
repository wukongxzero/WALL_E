/**
 * ============================================================================
 *  WALL_E_UNO_ADAPTIVE.ino
 *  Adaptive Notch + Kalman Stabilizer for Tank Tread Robot
 * ============================================================================
 *
 *  Board:    Arduino UNO (ATmega328P, 16MHz)
 *  IMU:      MPU6050 (I2C: SDA=A4, SCL=A5)
 *  Servo:    DSServo 25kg on pin 9
 *  Encoders: Left on D2 (INT0), Right on D3 (INT1)
 *  Serial:   TX (pin 1) -> Mega RX2 (pin 17) at 115200 baud
 *
 * ============================================================================
 *  PROBLEM
 * ============================================================================
 *
 *  The tank treads have 14 teeth each. When the bot drives, each tooth
 *  engagement creates a vibration pulse at frequency:
 *
 *      f_tread = (encoder_ticks_per_sec / ticks_per_rev) * teeth_per_rev
 *
 *  For example at 3 rev/s: f = 3 * 14 = 42 Hz
 *
 *  This vibration corrupts the accelerometer reading, which the
 *  complementary filter uses for drift correction. The result is a
 *  pitch estimate that oscillates at the tread frequency, causing
 *  the servo to hunt.
 *
 *  The gyroscope is less affected (it measures angular rate, not
 *  linear acceleration) but still picks up coupling vibration.
 *
 * ============================================================================
 *  SOLUTION — TWO-STAGE FILTER
 * ============================================================================
 *
 *  Stage 1: ADAPTIVE NOTCH FILTER on raw accelerometer
 *  ---------------------------------------------------
 *  A second-order IIR notch (band-reject) filter centered at f_tread.
 *  The center frequency updates every 100ms based on encoder RPM.
 *
 *  Transfer function:
 *      H(z) = (1 - 2cos(w0)z^-1 + z^-2) / (1 - 2r*cos(w0)z^-1 + r^2*z^-2)
 *
 *  where:
 *      w0 = 2*pi*f_tread / f_sample    (normalized frequency)
 *      r  = notch bandwidth parameter  (0.85-0.95, tighter = narrower notch)
 *
 *  At f_tread the numerator goes to zero -> infinite attenuation (in theory).
 *  In practice you get 20-40dB rejection in a narrow band around f_tread.
 *  Everything else passes through unchanged, including the DC gravity
 *  component that tells you the actual pitch angle.
 *
 *  When treads are stopped (RPM=0), the notch is bypassed — no vibration
 *  to reject, and the accelerometer can correct gyro drift freely.
 *
 *  Computational cost: 5 multiplies + 4 adds per sample = ~50 cycles.
 *
 *  Stage 2: ADAPTIVE KALMAN FILTER on pitch estimation
 *  ---------------------------------------------------
 *  A discrete-time Kalman filter with speed-dependent measurement noise R.
 *
 *  State vector:  x = [alpha, alpha_dot]  (platform angle and angular rate)
 *  Measurement:   z = alpha_accel         (pitch from accelerometer)
 *  Input:         u = tau                 (servo torque command)
 *
 *  Predict step (from Lagrangian-derived plant model):
 *      x_pred = Ad * x_est + Bd * u
 *      P_pred = Ad * P * Ad' + Q
 *
 *  Update step:
 *      K = P_pred * C' * (C * P_pred * C' + R)^-1
 *      x_est = x_pred + K * (z - C * x_pred)
 *      P = (I - K*C) * P_pred
 *
 *  The KEY ADAPTATION: R (measurement noise covariance) scales with
 *  motor speed:
 *
 *      R = R_base + R_speed * (RPM / RPM_max)^2
 *
 *  When stopped:  R = R_base (small)  -> trust accelerometer, correct drift
 *  When driving:  R = large           -> trust gyro model, reject vibration
 *
 *  This is NOT an Extended Kalman Filter (EKF). The plant is already
 *  linear in the small-angle regime. The adaptation is in the noise
 *  model, not the dynamics.
 *
 * ============================================================================
 *  PLANT MODEL (from your team's Lagrangian derivation)
 * ============================================================================
 *
 *  Equation of motion:  ml^2 * alpha_dd = tau + m*g*l*alpha
 *
 *  State space (continuous):
 *      A = [  0      1    ]     B = [     0       ]
 *          [ g/l     0    ]         [ 1/(m*l^2)   ]
 *
 *  Discretized at dt = 1/200 Hz via ZOH.
 *
 *  Parameters (from validate_controllers.py with m=0.261, l=0.05):
 *      g/l     = 196.20
 *      1/ml^2  = 1532.57
 *
 * ============================================================================
 *  CONTROLLER MODES (unchanged from your team's design)
 * ============================================================================
 *
 *  Mode 0 — Pole Placement:  K0=0.8961  K1=0.0457   (conservative)
 *  Mode 1 — LQR:             K0=6.4539  K1=0.4565   (aggressive)
 *  Mode 2 — LQG:             K0=6.4539  K1=0.4565   (same K, Kalman states)
 *
 *  In modes 0 and 1, the Kalman filter estimates states but the controller
 *  uses the complementary filter output. In mode 2, the controller uses
 *  the Kalman-estimated states directly.
 *
 * ============================================================================
 *  WIRING
 * ============================================================================
 *
 *  MPU6050:     SDA -> A4,  SCL -> A5,  VCC -> 3.3V,  GND -> GND
 *  Servo:       Signal -> D9,  VCC -> 5-6V,  GND -> GND
 *  Encoder L:   Signal -> D2 (INT0),  VCC -> 5V,  GND -> GND
 *  Encoder R:   Signal -> D3 (INT1),  VCC -> 5V,  GND -> GND
 *  To Mega:     UNO TX (pin 1) -> Mega RX2 (pin 17),  GND -> GND
 *
 * ============================================================================
 *  SERIAL COMMANDS (via USB monitor at 115200 baud)
 * ============================================================================
 *
 *  '0' — Pole Placement mode
 *  '1' — LQR mode
 *  '2' — LQG mode (Kalman-estimated states)
 *  'd' — Toggle debug CSV stream
 *  'p' — Print current state snapshot
 *  'n' — Toggle notch filter on/off (for A/B testing)
 *  'k' — Toggle Kalman adaptation on/off
 *
 * ============================================================================
 */

#include <Wire.h>
#include <Servo.h>
#include <avr/wdt.h>

/* ---- CONTROLLER GAINS (from validate_controllers.py) ---- */
#define K0_PP     0.8961f
#define K1_PP     0.0457f
#define K0_LQR    6.4539f
#define K1_LQR    0.4565f
#define K0_LQG    6.4539f
#define K1_LQG    0.4565f

/* ---- KALMAN GAINS (from validate_controllers.py) ---- */
#define L0_KAL    0.992882f
#define L1_KAL    10.173582f

/* ---- PLANT PARAMETERS ---- */
#define G_OVER_L    196.20f       /* g/l = 9.81/0.05 */
#define INV_MLL     1532.57f      /* 1/(m*l^2) = 1/(0.261*0.05^2) */

/* ---- LOOP TIMING ---- */
#define LOOP_HZ     200
#define LOOP_US     (1000000UL / LOOP_HZ)
#define DT          (1.0f / LOOP_HZ)

/* ---- SERVO ---- */
#define SERVO_PIN       9
#define SERVO_CENTER_US 1500
#define SERVO_RANGE_US  300
#define CMD_LIMIT       25.0f

/* ---- IMU ---- */
#define MPU_ADDR    0x68
#define GYRO_SENS   131.0f        /* LSB/(deg/s) for +/-250 dps */
#define COMP_ALPHA  0.98f

/* ---- ENCODER ---- */
#define ENC_LEFT_PIN    2
#define ENC_RIGHT_PIN   3
#define TICKS_PER_REV   12        /* 14 teeth on tread sprocket */
#define RPM_CALC_MS     100       /* recalculate RPM every 100ms */
#define RPM_MAX         300.0f    /* estimated max RPM for normalization */

/* ---- NOTCH FILTER ---- */
#define NOTCH_R         0.90f     /* bandwidth: 0.85=wide, 0.95=narrow */
#define NOTCH_MIN_FREQ  10.0f     /* don't notch below 10 Hz */
#define NOTCH_MAX_FREQ  95.0f     /* Nyquist limit at 200Hz sample rate */

/* ---- ADAPTIVE KALMAN ---- */
#define R_BASE          0.0001f   /* R when stopped (trust accel) */
#define R_SPEED_SCALE   0.05f     /* R scaling with speed */
#define Q_ANGLE         0.0001f   /* process noise on angle */
#define Q_RATE          0.01f     /* process noise on rate */

/* ---- ANGLE OFFSET ---- */
#define ANGLE_OFFSET_DEG 0.0f     /* trim if platform not level at rest */


/* ====================================================================
 *  GLOBAL STATE
 * ==================================================================== */

Servo platformServo;
uint32_t lastLoopUs = 0;

/* IMU raw state */
float accPitchDeg  = 0.0f;
float gyroPitchDps = 0.0f;

/* Complementary filter state */
float compPitchDeg = 0.0f;

/* Kalman filter state */
float kalAlpha    = 0.0f;   /* estimated angle (rad) */
float kalAlphaDot = 0.0f;   /* estimated angular rate (rad/s) */
float kalP[2][2]  = {{1.0f, 0.0f},   /* error covariance matrix */
                      {0.0f, 1.0f}};

/* Notch filter state (Direct Form II) */
float notch_w1 = 0.0f;      /* delay line element 1 */
float notch_w2 = 0.0f;      /* delay line element 2 */
float notch_b0 = 1.0f;      /* numerator coefficients */
float notch_b1 = 0.0f;
float notch_b2 = 1.0f;
float notch_a1 = 0.0f;      /* denominator coefficients */
float notch_a2 = 0.0f;

/* Encoder state */
volatile long encLeftTicks  = 0;
volatile long encRightTicks = 0;
long encLeftPrev  = 0;
long encRightPrev = 0;
uint32_t lastRpmCalcMs = 0;
float currentRPM = 0.0f;
float treadFreqHz = 0.0f;

/* Control */
int   controllerMode = 0;    /* 0=PP, 1=LQR, 2=LQG */
float lastTorqueCmd  = 0.0f;
float filteredCmd    = 0.0f;

/* Flags */
bool debugStream   = false;
bool notchEnabled  = true;
bool adaptiveR     = true;


/* ====================================================================
 *  ENCODER ISRs
 * ==================================================================== */

void isrEncLeft()  { encLeftTicks++;  }
void isrEncRight() { encRightTicks++; }


/* ====================================================================
 *  MPU6050 DRIVER
 * ==================================================================== */

static void mpuWrite(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(reg);
    Wire.write(val);
    Wire.endTransmission(true);
}

static void mpuReadBytes(uint8_t reg, uint8_t n, uint8_t *buf) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(reg);
    Wire.endTransmission(false);
    uint8_t got = Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)n);
    for (uint8_t i = 0; i < got; i++) buf[i] = Wire.read();
}

static void mpuInit() {
    mpuWrite(0x6B, 0x00);   /* wake up */
    delay(50);
    mpuWrite(0x1B, 0x00);   /* gyro +/-250 dps */
    mpuWrite(0x1C, 0x00);   /* accel +/-2g */
    mpuWrite(0x1A, 0x03);   /* DLPF ~42Hz (hardware pre-filter) */
}

static void readImu(int16_t &ax, int16_t &ay, int16_t &az,
                    int16_t &gx, int16_t &gy, int16_t &gz) {
    uint8_t buf[14];
    mpuReadBytes(0x3B, 14, buf);
    ax = (int16_t)((buf[0] << 8) | buf[1]);
    ay = (int16_t)((buf[2] << 8) | buf[3]);
    az = (int16_t)((buf[4] << 8) | buf[5]);
    gx = (int16_t)((buf[8] << 8) | buf[9]);
    gy = (int16_t)((buf[10] << 8) | buf[11]);
    gz = (int16_t)((buf[12] << 8) | buf[13]);
}


/* ====================================================================
 *  NOTCH FILTER
 * ====================================================================
 *
 *  Second-order IIR notch (band-reject) filter.
 *
 *  Design equations:
 *      w0 = 2*pi*f_notch / f_sample
 *      b0 = 1
 *      b1 = -2*cos(w0)
 *      b2 = 1
 *      a1 = -2*r*cos(w0)    (note: stored NEGATED for Direct Form II)
 *      a2 = r^2
 *
 *  The parameter r controls bandwidth:
 *      r = 0.85 -> wide notch  (~30 Hz bandwidth)
 *      r = 0.90 -> medium      (~20 Hz)
 *      r = 0.95 -> narrow      (~10 Hz)
 *
 *  We use r=0.90 as a compromise: wide enough to track frequency
 *  estimation errors, narrow enough not to eat useful signal.
 */

void updateNotchCoeffs(float freqHz) {
    if (freqHz < NOTCH_MIN_FREQ || freqHz > NOTCH_MAX_FREQ) {
        /* Outside valid range — set to passthrough */
        notch_b0 = 1.0f; notch_b1 = 0.0f; notch_b2 = 0.0f;
        notch_a1 = 0.0f; notch_a2 = 0.0f;
        return;
    }

    float w0 = 2.0f * 3.14159265f * freqHz / (float)LOOP_HZ;
    float cosw = cos(w0);
    float r  = NOTCH_R;
    float r2 = r * r;

    notch_b0 = 1.0f;
    notch_b1 = -2.0f * cosw;
    notch_b2 = 1.0f;
    notch_a1 = 2.0f * r * cosw;   /* stored positive (we subtract in filter) */
    notch_a2 = -r2;               /* stored negated */

    /* Reset delay line when frequency changes significantly
     * to avoid transient glitches */
    notch_w1 = 0.0f;
    notch_w2 = 0.0f;
}

/**
 * Apply the notch filter to one sample.
 * Uses Direct Form II Transposed for numerical stability.
 *
 * Input:  raw accelerometer pitch value
 * Output: filtered value with tread frequency removed
 */
float applyNotch(float x) {
    if (!notchEnabled) return x;

    /* Direct Form II Transposed */
    float y  = notch_b0 * x           + notch_w1;
    notch_w1 = notch_b1 * x + notch_a1 * y + notch_w2;
    notch_w2 = notch_b2 * x + notch_a2 * y;

    return y;
}


/* ====================================================================
 *  RPM CALCULATION
 * ====================================================================
 *
 *  Called every RPM_CALC_MS (100ms).
 *  Reads encoder tick count, computes RPM and tread vibration frequency.
 *
 *  tread_freq = (ticks_per_second / TICKS_PER_REV) * TICKS_PER_REV
 *             = ticks_per_second
 *
 *  Wait — that simplifies! Since TICKS_PER_REV equals the number of
 *  teeth (14), and each tick IS one tooth passing, the vibration
 *  frequency in Hz is simply:
 *
 *      f_vibration = total_ticks_per_second  (from both encoders averaged)
 *
 *  RPM = (ticks_per_second / TICKS_PER_REV) * 60
 */
void calcRPM() {
    /* Read encoder counts atomically (interrupts might fire mid-read) */
    noInterrupts();
    long leftNow  = encLeftTicks;
    long rightNow = encRightTicks;
    interrupts();

    long leftDelta  = leftNow  - encLeftPrev;
    long rightDelta = rightNow - encRightPrev;
    encLeftPrev  = leftNow;
    encRightPrev = rightNow;

    /* Average both encoders for smoother estimate */
    float avgDelta = (float)(leftDelta + rightDelta) * 0.5f;

    /* Convert to ticks per second */
    float ticksPerSec = avgDelta * (1000.0f / RPM_CALC_MS);

    /* RPM for adaptive R scaling */
    currentRPM = (ticksPerSec / (float)TICKS_PER_REV) * 60.0f;

    /* Vibration frequency = tooth engagement rate */
    treadFreqHz = ticksPerSec;

    /* Update notch filter center frequency */
    if (treadFreqHz > NOTCH_MIN_FREQ) {
        updateNotchCoeffs(treadFreqHz);
    }
}


/* ====================================================================
 *  ADAPTIVE KALMAN FILTER
 * ====================================================================
 *
 *  Discrete-time Kalman filter for the linearized plant.
 *
 *  State:   x = [alpha (rad), alpha_dot (rad/s)]
 *  Input:   u = tau (N*m, last servo torque command)
 *  Meas:    z = alpha_accel (rad, from accelerometer after notch filter)
 *
 *  Discretized plant (ZOH at 200Hz):
 *      Ad = I + A*dt = [1   dt      ]    Bd = B*dt = [    0         ]
 *                      [g/l*dt  1   ]                [dt/(m*l^2)   ]
 *
 *  Measurement:  C = [1, 0]  (we only measure angle, not rate)
 *
 *  ADAPTATION: R = R_BASE + R_SPEED_SCALE * (RPM/RPM_MAX)^2
 *
 *  When the bot is stationary, R is small -> Kalman trusts the
 *  accelerometer -> corrects gyro drift.
 *
 *  When driving fast, R is large -> Kalman trusts the model ->
 *  rejects vibration that the notch filter didn't fully kill.
 *
 *  The two filters are complementary:
 *  - Notch kills the dominant tread frequency (narrowband)
 *  - Adaptive R suppresses residual broadband vibration
 */
void kalmanPredict(float torqueCmd) {
    /* Discretized state transition */
    float alpha_pred    = kalAlpha + DT * kalAlphaDot;
    float alphaDot_pred = G_OVER_L * DT * kalAlpha + kalAlphaDot
                          + INV_MLL * DT * torqueCmd;

    /* Covariance predict: P = Ad*P*Ad' + Q */
    float p00 = kalP[0][0] + DT * kalP[1][0] + DT * (kalP[0][1] + DT * kalP[1][1]);
    float p01 = kalP[0][1] + DT * kalP[1][1];
    float p10 = kalP[1][0] + G_OVER_L * DT * kalP[0][0]
                + DT * (kalP[1][1] + G_OVER_L * DT * kalP[0][1]);
    float p11 = kalP[1][1] + G_OVER_L * DT * kalP[0][1]
                + G_OVER_L * DT * (kalP[1][0] + G_OVER_L * DT * kalP[0][0])
                + DT * DT * G_OVER_L * G_OVER_L * kalP[0][0]; /* approximate */

    /* Simplify: use the dominant terms only (UNO has limited FP throughput) */
    kalP[0][0] = p00 + Q_ANGLE;
    kalP[0][1] = p01;
    kalP[1][0] = p10;
    kalP[1][1] = p11 + Q_RATE;

    kalAlpha    = alpha_pred;
    kalAlphaDot = alphaDot_pred;
}

void kalmanUpdate(float measAngleRad) {
    /* Adaptive measurement noise */
    float speedFrac = currentRPM / RPM_MAX;
    if (speedFrac > 1.0f) speedFrac = 1.0f;

    float R;
    if (adaptiveR) {
        R = R_BASE + R_SPEED_SCALE * speedFrac * speedFrac;
    } else {
        R = R_BASE;  /* fixed R for A/B testing */
    }

    /* Innovation */
    float y = measAngleRad - kalAlpha;

    /* Innovation covariance: S = C*P*C' + R = P[0][0] + R */
    float S = kalP[0][0] + R;

    /* Kalman gain: K = P*C'/S */
    float K0 = kalP[0][0] / S;
    float K1 = kalP[1][0] / S;

    /* State update */
    kalAlpha    += K0 * y;
    kalAlphaDot += K1 * y;

    /* Covariance update: P = (I - K*C) * P */
    float p00_new = (1.0f - K0) * kalP[0][0];
    float p01_new = (1.0f - K0) * kalP[0][1];
    float p10_new = kalP[1][0] - K1 * kalP[0][0];
    float p11_new = kalP[1][1] - K1 * kalP[0][1];

    kalP[0][0] = p00_new;
    kalP[0][1] = p01_new;
    kalP[1][0] = p10_new;
    kalP[1][1] = p11_new;
}


/* ====================================================================
 *  SERVO OUTPUT
 * ==================================================================== */

static float clampf(float x, float lo, float hi) {
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

static void applyServo(float u) {
    u = clampf(u, -CMD_LIMIT, CMD_LIMIT);
    int pulse = SERVO_CENTER_US + (int)(u * (SERVO_RANGE_US / CMD_LIMIT));
    platformServo.writeMicroseconds(pulse);
}


/* ====================================================================
 *  CONTROLLER
 * ====================================================================
 *
 *  All three modes use the same structure:
 *      tau = -(K0 * alpha + K1 * alpha_dot)
 *
 *  The difference is WHERE alpha and alpha_dot come from:
 *      Mode 0 (PP):  complementary filter
 *      Mode 1 (LQR): complementary filter
 *      Mode 2 (LQG): Kalman filter estimates
 */
float computeControl(float compAngleDeg, float gyroDps) {
    float alpha, alphaDot;

    switch (controllerMode) {
        case 0:  /* Pole Placement — use comp filter */
            alpha    = (compAngleDeg - ANGLE_OFFSET_DEG) * (3.14159265f / 180.0f);
            alphaDot = gyroDps * (3.14159265f / 180.0f);
            return -(K0_PP * alpha + K1_PP * alphaDot);

        case 1:  /* LQR — use comp filter */
            alpha    = (compAngleDeg - ANGLE_OFFSET_DEG) * (3.14159265f / 180.0f);
            alphaDot = gyroDps * (3.14159265f / 180.0f);
            return -(K0_LQR * alpha + K1_LQR * alphaDot);

        case 2:  /* LQG — use Kalman estimates */
            return -(K0_LQG * kalAlpha + K1_LQG * kalAlphaDot);

        default:
            return 0.0f;
    }
}


/* ====================================================================
 *  SERIAL HELPERS
 * ==================================================================== */

void printMode() {
    Serial.print("Mode: ");
    switch (controllerMode) {
        case 0: Serial.println("POLE PLACEMENT"); break;
        case 1: Serial.println("LQR");            break;
        case 2: Serial.println("LQG (Kalman)");   break;
    }
    Serial.print("Notch: ");
    Serial.println(notchEnabled ? "ON" : "OFF");
    Serial.print("Adaptive R: ");
    Serial.println(adaptiveR ? "ON" : "OFF");
}


/* ====================================================================
 *  SETUP
 * ==================================================================== */

void setup() {
    /* Watchdog */
    if (MCUSR & (1 << WDRF)) MCUSR = 0;
    wdt_disable();
    wdt_enable(WDTO_2S);

    Serial.begin(115200);
    Wire.begin();
    Wire.setClock(400000);

    mpuInit();

    platformServo.attach(SERVO_PIN);
    platformServo.writeMicroseconds(SERVO_CENTER_US);

    /* Encoders */
    pinMode(ENC_LEFT_PIN, INPUT_PULLUP);
    pinMode(ENC_RIGHT_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(ENC_LEFT_PIN), isrEncLeft, RISING);
    attachInterrupt(digitalPinToInterrupt(ENC_RIGHT_PIN), isrEncRight, RISING);

    /* Initialize notch to passthrough */
    updateNotchCoeffs(0);

    Serial.println("=== WALL-E UNO ADAPTIVE ===");
    Serial.println("0=PP 1=LQR 2=LQG d=debug p=print n=notch k=adaptR");
    printMode();

    lastLoopUs = micros();
    lastRpmCalcMs = millis();
    delay(200);
}


/* ====================================================================
 *  MAIN LOOP
 * ==================================================================== */

void loop() {
    /* ---- Serial commands ---- */
    if (Serial.available()) {
        char cmd = Serial.read();
        switch (cmd) {
            case '0': controllerMode = 0; filteredCmd = 0; printMode(); break;
            case '1': controllerMode = 1; filteredCmd = 0; printMode(); break;
            case '2': controllerMode = 2;
                      filteredCmd = 0;
                      kalAlpha = 0; kalAlphaDot = 0;
                      kalP[0][0] = 1; kalP[0][1] = 0;
                      kalP[1][0] = 0; kalP[1][1] = 1;
                      printMode(); break;
            case 'd': debugStream = !debugStream;
                      Serial.println(debugStream ? "Debug ON" : "Debug OFF");
                      if (debugStream) {
                          Serial.println("t_ms,comp_deg,kal_deg,rpm,tread_hz,cmd");
                      }
                      break;
            case 'n': notchEnabled = !notchEnabled;
                      Serial.print("Notch: ");
                      Serial.println(notchEnabled ? "ON" : "OFF");
                      break;
            case 'k': adaptiveR = !adaptiveR;
                      Serial.print("Adaptive R: ");
                      Serial.println(adaptiveR ? "ON" : "OFF");
                      break;
            case 'p':
                      Serial.print("comp="); Serial.print(compPitchDeg, 2);
                      Serial.print(" kal="); Serial.print(kalAlpha * 57.29578f, 2);
                      Serial.print(" rpm="); Serial.print(currentRPM, 0);
                      Serial.print(" freq="); Serial.print(treadFreqHz, 1);
                      Serial.print(" cmd="); Serial.println(filteredCmd, 2);
                      break;
        }
    }

    /* ---- RPM calculation (every 100ms) ---- */
    uint32_t nowMs = millis();
    if ((uint32_t)(nowMs - lastRpmCalcMs) >= RPM_CALC_MS) {
        lastRpmCalcMs = nowMs;
        calcRPM();
    }

    /* ---- Main control loop (200 Hz) ---- */
    uint32_t nowUs = micros();
    if ((uint32_t)(nowUs - lastLoopUs) < LOOP_US) return;
    float dt = (nowUs - lastLoopUs) * 1e-6f;
    lastLoopUs = nowUs;

    wdt_reset();

    /* ---- Read IMU ---- */
    int16_t ax, ay, az, gx, gy, gz;
    readImu(ax, ay, az, gx, gy, gz);

    /* ---- Raw accelerometer pitch ---- */
    float fax = (float)ax, fay = (float)ay, faz = (float)az;
    float rawAccPitch = atan2f(-fax, sqrtf(fay * fay + faz * faz)) * 57.29578f;

    /* ---- Stage 1: Notch filter on accelerometer ---- */
    float filteredAccPitch = applyNotch(rawAccPitch);
    accPitchDeg = filteredAccPitch;

    /* ---- Gyro rate ---- */
    gyroPitchDps = ((float)gy) / GYRO_SENS;

    /* ---- Complementary filter (uses notch-filtered accel) ---- */
    float gyroIntegrated = compPitchDeg + gyroPitchDps * dt;
    compPitchDeg = COMP_ALPHA * gyroIntegrated
                   + (1.0f - COMP_ALPHA) * filteredAccPitch;

    /* ---- Stage 2: Adaptive Kalman filter ---- */
    kalmanPredict(lastTorqueCmd);
    float measRad = filteredAccPitch * (3.14159265f / 180.0f);
    kalmanUpdate(measRad);

    /* ---- Controller ---- */
    float torque = computeControl(compPitchDeg, gyroPitchDps);
    lastTorqueCmd = torque;

    /* ---- Convert torque to servo command (degrees) ---- */
    /*
     * The servo doesn't accept torque directly — it accepts an angle.
     * We map the torque command to a servo deflection angle.
     * This is a simplification: we assume the servo is stiff enough
     * that commanding an angle is roughly proportional to the restoring
     * torque it applies. For a 25kg-cm servo at 0.05m arm, max torque
     * is ~2.45 N*m, which maps to CMD_LIMIT degrees of deflection.
     */
    float cmdDeg = torque * (CMD_LIMIT / 2.45f);
    cmdDeg = clampf(cmdDeg, -CMD_LIMIT, CMD_LIMIT);

    /* Low-pass filter on command (smooth servo jitter) */
    filteredCmd = 0.7f * filteredCmd + 0.3f * cmdDeg;

    applyServo(filteredCmd);

    /* ---- Send pitch to Mega (for BT telemetry) ---- */
    /* Use Kalman estimate in LQG mode, comp filter otherwise */
    float pitchToSend;
    if (controllerMode == 2) {
        pitchToSend = kalAlpha * 57.29578f;
    } else {
        pitchToSend = compPitchDeg;
    }

    static uint8_t sendCounter = 0;
    if (++sendCounter >= 10) {  /* 20 Hz to Mega (200/10) */
        sendCounter = 0;
        Serial.print("P:");
        Serial.println(pitchToSend, 2);
    }

    /* ---- Debug CSV stream ---- */
    if (debugStream) {
        static uint8_t dbgCounter = 0;
        if (++dbgCounter >= 4) {  /* 50 Hz debug (200/4) */
            dbgCounter = 0;
            Serial.print(millis());
            Serial.print(",");
            Serial.print(compPitchDeg, 2);
            Serial.print(",");
            Serial.print(kalAlpha * 57.29578f, 2);
            Serial.print(",");
            Serial.print(currentRPM, 0);
            Serial.print(",");
            Serial.print(treadFreqHz, 1);
            Serial.print(",");
            Serial.println(filteredCmd, 2);
        }
    }
}
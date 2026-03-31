import numpy as np
from scipy.linalg import solve_continuous_are
from scipy.signal import lsim, StateSpace, place_poles, cont2discrete
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec

# ══════════════════════════════════════════════════════
#  YOUR BOT PARAMETERS
# ══════════════════════════════════════════════════════
m   = 0.261    # kg  — platform + payload (MEASURE THIS)

l   = 0.04    # m   — servo shaft to CoM (MEASURE THIS)
g   = 9.81
dt  = 1.0/200 # 200Hz (your existing loop rate)
SERVO_MAX_DEG = 25.0  # CMD_LIMIT from your code

# ══════════════════════════════════════════════════════
#  PLANT — derived from Lagrangian
#  α̈ = (g/l)α + (1/ml²)τ
#  State: x = [α (rad), α̇ (rad/s)]
#  Input: u = τ (servo torque, N·m)
# ══════════════════════════════════════════════════════
A = np.array([[0,    1  ],
              [g/l,  0  ]])
B = np.array([[0          ],
              [1/(m*l**2) ]])

print("=" * 60)
print("  WALL-E TREADBOT — CONTROLLER VALIDATION")
print("=" * 60)
print(f"\nPlant (from Lagrangian):")
print(f"  α̈ = (g/l)·α + (1/ml²)·τ")
print(f"  g/l   = {g/l:.2f}  →  instability rate")
print(f"  1/ml² = {1/(m*l**2):.2f}  →  servo authority")
print(f"\nOpen loop eigenvalues: ±{np.sqrt(g/l):.3f} rad/s")
print(f"  +{np.sqrt(g/l):.3f} = unstable pole (platform wants to fall)")
print(f"  -{np.sqrt(g/l):.3f} = stable pole")

# ══════════════════════════════════════════════════════
#  APPROACH 1 — POLE PLACEMENT
#  You manually choose WHERE the closed loop poles go
#  Rule: place poles 2-4x further left than open loop
#  Open loop unstable pole = +11.84
#  → place at -23.7 and -35.5 (2x and 3x further)
# ══════════════════════════════════════════════════════
ol_rate = np.sqrt(g/l)
desired_poles = np.array([-2*ol_rate, -3*ol_rate])
pp = place_poles(A, B, desired_poles)
K_pp = pp.gain_matrix
A_cl_pp = A - B @ K_pp

print(f"\n── POLE PLACEMENT ──")
print(f"  Strategy: place poles at {desired_poles.round(2)}")
print(f"  (2x and 3x faster than open loop instability rate {ol_rate:.2f})")
print(f"  K0 = {K_pp[0][0]:.4f}  (how hard to push per degree of tilt)")
print(f"  K1 = {K_pp[0][1]:.4f}  (how hard to damp per rad/s of rate)")
print(f"  Closed loop poles: {np.linalg.eigvals(A_cl_pp).real.round(3)}")

# ══════════════════════════════════════════════════════
#  APPROACH 2 — LQR
#  Optimal: minimizes cost J = ∫(α²·q1 + α̇²·q2 + τ²·r)dt
#  Q diagonal: [how much you hate angle error, how much you hate fast rates]
#  R: how much you hate servo effort
#  Constraint: peak torque must stay within DSServo 25kg = 2.45 N·m
# ══════════════════════════════════════════════════════
Q = np.diag([200, 1])
R = np.array([[50.0]])
P = solve_continuous_are(A, B, Q, R)
K_lqr = np.linalg.inv(R) @ B.T @ P
A_cl_lqr = A - B @ K_lqr

print(f"\n── LQR ──")
print(f"  Strategy: minimize ∫(200α² + α̇² + 5τ²)dt")
print(f"  Q = diag[200, 1] → heavily penalize angle error")
print(f"  R = 5.0          → moderate servo effort constraint")
print(f"  K0 = {K_lqr[0][0]:.4f}")
print(f"  K1 = {K_lqr[0][1]:.4f}")
print(f"  Closed loop poles: {np.linalg.eigvals(A_cl_lqr).real.round(3)}")

# ══════════════════════════════════════════════════════
#  APPROACH 3 — LQG (LQR + KALMAN FILTER)
#  Same LQR gains BUT states α and α̇ are estimated
#  from noisy IMU rather than measured directly
#  Kalman gain L: how much to trust measurement vs model
#  L0 ≈ 1.0 → almost fully trust IMU angle measurement
#  L1 ≈ 10  → strongly correct rate estimate from angle measurement
# ══════════════════════════════════════════════════════
Ad, Bd, Cd, Dd, _ = cont2discrete(
    (A, B, np.array([[1,0]]), np.zeros((1,1))), dt, method='zoh')
Qn = np.diag([1e-4, 1e-2])
Rn = np.array([[np.radians(0.05)**2]])
from scipy.linalg import solve_discrete_are
Pk = solve_discrete_are(Ad.T, Cd.T, Qn, Rn)
L  = Pk @ Cd.T @ np.linalg.inv(Cd @ Pk @ Cd.T + Rn)

print(f"\n── LQG (Kalman + LQR) ──")
print(f"  Same K0={K_lqr[0][0]:.4f}, K1={K_lqr[0][1]:.4f} as LQR")
print(f"  Kalman L0 = {L[0][0]:.4f}  (trust IMU angle almost fully)")
print(f"  Kalman L1 = {L[1][0]:.4f}  (strongly correct rate from angle)")
print(f"  Process noise Qn = diag[1e-4, 1e-2]")
print(f"  Meas noise    Rn = {Rn[0][0]:.2e} rad² (MPU6050 accel noise)")

# ══════════════════════════════════════════════════════
#  SIMULATE 3 SCENARIOS FOR EACH CONTROLLER
# ══════════════════════════════════════════════════════
t = np.linspace(0, 2, 2000)
x0_tilt   = [np.radians(10), 0]    # 10° initial tilt (terrain bump)
x0_rate   = [0, np.radians(30)]    # sudden angular kick (tread vibration)
x0_small  = [np.radians(2), 0]     # small 2° disturbance

controllers = {
    'Pole Placement': (A_cl_pp,  K_pp[0],  'royalblue'),
    'LQR':           (A_cl_lqr, K_lqr[0], 'forestgreen'),
  'LQG':           (A_cl_lqr, K_lqr[0], 'crimson'),
}

scenarios = {
    '10° tilt (terrain bump)': x0_tilt,
    '30°/s kick (tread vibration)': x0_rate,
    '2° small disturbance': x0_small,
}

SERVO_MAX_TORQUE = 2.45  # N·m

fig = plt.figure(figsize=(18, 12))
fig.suptitle('WALL-E Treadbot — Controller Validation\n'
             'Plant from Lagrangian: α̈ = (g/l)α + (1/ml²)τ',
             fontsize=14, fontweight='bold')

gs = gridspec.GridSpec(3, 3, figure=fig, hspace=0.45, wspace=0.35)

print(f"\n── SIMULATION RESULTS ──")
print(f"{'Controller':<16} {'Scenario':<30} {'Settle(s)':<12} {'Peak τ(N·m)':<14} {'Feasible'}")
print("-"*80)

for col, (ctrl_name, (A_cl, K, color)) in enumerate(controllers.items()):
    for row, (scen_name, x0) in enumerate(scenarios.items()):
        ax = fig.add_subplot(gs[row, col])

        sys_cl = StateSpace(A_cl, np.zeros((2,1)), np.eye(2), np.zeros((2,1)))
        _, y, _ = lsim(sys_cl, U=np.zeros((len(t),1)), T=t, X0=x0)

        alpha_deg = np.degrees(y[:,0])
        tau       = -(K[0]*y[:,0] + K[1]*y[:,1])
        peak_tau  = np.max(np.abs(tau))
        feasible  = peak_tau <= SERVO_MAX_TORQUE

        settled_idx = np.where(np.abs(alpha_deg) < 1.0)[0]
        t_settle = t[settled_idx[0]] if len(settled_idx) > 0 else 999

        print(f"{ctrl_name:<16} {scen_name:<30} {t_settle:<12.3f} "
              f"{peak_tau:<14.3f} {'✓' if feasible else '✗ OVER LIMIT'}")

        bg = '#e8f5e9' if feasible else '#ffebee'
        ax.set_facecolor(bg)
        ax.plot(t, alpha_deg, color=color, linewidth=2)
        ax.axhline(0,  color='k',    linestyle='--', alpha=0.4, linewidth=1)
        ax.axhline(1,  color='gray', linestyle=':',  alpha=0.6, linewidth=1)
        ax.axhline(-1, color='gray', linestyle=':',  alpha=0.6, linewidth=1)
        if t_settle < 999:
            ax.axvline(t_settle, color='orange', linestyle='--',
                      alpha=0.8, linewidth=1.5,
                      label=f'settle={t_settle:.2f}s')
        ax.set_ylim(-5, max(12, np.degrees(x0[0])+2))
        ax.set_xlabel('t (s)', fontsize=8)
        ax.set_ylabel('α (deg)', fontsize=8)
        ax.tick_params(labelsize=7)
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=7)

        if row == 0:
            ax.set_title(f'{ctrl_name}\nK0={K[0]:.2f}  K1={K[1]:.2f}',
                        fontsize=9, fontweight='bold', color=color)
        if col == 0:
            ax.set_ylabel(f'{scen_name}\nα (deg)', fontsize=8)

plt.savefig('controller_validation.png',
            dpi=150, bbox_inches='tight')

# ══════════════════════════════════════════════════════
#  ARDUINO FLASH VALUES SUMMARY
# ══════════════════════════════════════════════════════
print(f"\n{'='*60}")
print(f"  ARDUINO #define VALUES")
print(f"{'='*60}")
print(f"\n// Pole Placement")
print(f"#define K0_PP    {K_pp[0][0]:.4f}f")
print(f"#define K1_PP    {K_pp[0][1]:.4f}f")
print(f"\n// LQR")
print(f"#define K0_LQR   {K_lqr[0][0]:.4f}f")
print(f"#define K1_LQR   {K_lqr[0][1]:.4f}f")
print(f"\n// LQG")
print(f"#define K0_LQG   {K_lqr[0][0]:.4f}f")
print(f"#define K1_LQG   {K_lqr[0][1]:.4f}f")
print(f"#define L0_KAL   {L[0][0]:.6f}f")
print(f"#define L1_KAL   {L[1][0]:.6f}f")
print(f"\n// Plant (for Kalman predict step)")
print(f"#define G_OVER_L    {g/l:.2f}f")
print(f"#define INV_MLL     {1/(m*l**2):.2f}f")
print("\nPlot saved!")

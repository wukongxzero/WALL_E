import numpy as np
from scipy.linalg import solve_continuous_are
from scipy.signal import lsim, StateSpace, place_poles
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

g = 9.81
m_nom = 0.50
l_nom = 0.07

A_nom = np.array([[0,1],[g/l_nom,0]])
B_nom = np.array([[0],[1/(m_nom*l_nom**2)]])

Q = np.diag([200, 1])
R = np.array([[5.0]])
P = solve_continuous_are(A_nom, B_nom, Q, R)
K_lqr = np.linalg.inv(R) @ B_nom.T @ P

ol_rate = np.sqrt(g/l_nom)
pp = place_poles(A_nom, B_nom, np.array([-2*ol_rate, -3*ol_rate]))
K_pp = pp.gain_matrix

payloads = [
    {'label': 'Empty\n(no payload)',    'mp': 0.00, 'hp': 0.00},
    {'label': 'Light\n(250g)',          'mp': 0.25, 'hp': 0.03},
    {'label': 'Nominal\n(500g)',        'mp': 0.50, 'hp': 0.04},
    {'label': 'Heavy\n(1kg)',           'mp': 1.00, 'hp': 0.05},
    {'label': 'Very Heavy\n(2kg)',      'mp': 2.00, 'hp': 0.06},
]

mc = 0.20
hc = 0.03
t  = np.linspace(0, 3, 3000)
x0 = [np.radians(10), 0]
SERVO_MAX = 2.45

fig, axes = plt.subplots(2, len(payloads), figsize=(18, 8))
fig.suptitle('Payload Sensitivity — Nominal Gains on Different Payloads', fontsize=13)

print(f"{'Payload':<20} {'m':>6} {'l':>7} {'g/l':>8} {'LQR':>10} {'PP':>10}")
print("-"*65)

for i, p in enumerate(payloads):
    m_total = mc + p['mp']
    l_eff   = (mc*hc + p['mp']*(hc+p['hp'])) / m_total if p['mp'] > 0 else hc
    A_act   = np.array([[0,1],[g/l_eff,0]])
    B_act   = np.array([[0],[1/(m_total*l_eff**2)]])

    for j, (ctrl, K) in enumerate([('LQR', K_lqr[0]), ('PP', K_pp[0])]):
        K_mat  = K.reshape(1,2)
        A_cl   = A_act - B_act @ K_mat
        stable = all(e.real < 0 for e in np.linalg.eigvals(A_cl))
        sys_cl = StateSpace(A_cl, np.zeros((2,1)), np.eye(2), np.zeros((2,1)))
        _, y, _ = lsim(sys_cl, U=np.zeros((len(t),1)), T=t, X0=x0)
        alpha_deg = np.degrees(y[:,0])
        settled   = np.where(np.abs(alpha_deg) < 1.0)[0]
        t_settle  = t[settled[0]] if len(settled) > 0 else 999
        color = ('forestgreen' if j==0 else 'royalblue') if stable else 'crimson'
        bg    = ('#e8f5e9' if j==0 else '#e8f4ff') if stable else '#ffebee'
        ax = axes[j][i]
        ax.set_facecolor(bg)
        ax.plot(t, alpha_deg, color=color, linewidth=2)
        ax.axhline(0, color='k', linestyle='--', alpha=0.4)
        ax.axhline(1, color='gray', linestyle=':', alpha=0.5)
        ax.axhline(-1,color='gray', linestyle=':', alpha=0.5)
        ax.set_ylim(-20, 15)
        ax.set_xlabel('t (s)', fontsize=8)
        ax.tick_params(labelsize=7)
        ax.grid(True, alpha=0.3)
        status = f"settle={t_settle:.2f}s" if stable else "UNSTABLE"
        ax.text(0.05,0.05,status,transform=ax.transAxes,fontsize=8,color=color,fontweight='bold')
        if i == 0:
            ax.set_ylabel(f'{"LQR" if j==0 else "PP"}\nalpha (deg)', fontsize=8)
        if j == 0:
            ax.set_title(p['label'], fontsize=9, fontweight='bold')

    print(f"{p['label'].replace(chr(10),' '):<20} {m_total:>6.2f} {l_eff:>7.4f} {g/l_eff:>8.2f}")

plt.tight_layout()
plt.savefig('payload_sensitivity.png', dpi=150, bbox_inches='tight')
print("\nSaved: payload_sensitivity.png")
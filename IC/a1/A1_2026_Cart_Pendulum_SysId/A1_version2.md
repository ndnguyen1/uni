# A1 — Cart-Pendulum System Identification: Notes

## Assignment Structure

**Q1: System Identification (60 marks)**

| Part | Task |
|---|---|
| a | Derive continuous-time SS matrices Ac, Bc, Cc (small-angle + slow angular speed approx) |
| b | Discretise using forward-Euler → Ad, Bd, Cd |
| c | Rearrange into regression form `b_l = A_l · θ` |
| d | Lab work: apply pos/neg pulse inputs, record xc and α |
| e | Kalman smoother (forward KF pass + RTS backward pass) → estimate full state |
| f | Offline Least-Squares → identify θ̂ |
| g | Recursive Least-Squares → compare convergence with f) |
| h ⭐ | Challenge: real-time RLS on control board |

**Q2: LQR Control Design (40 marks)**

| Part | Task |
|---|---|
| a | Prove controllability, design LQR, simulate with full state feedback |
| b | Prove observability, design Kalman filter, simulate LQR + Kalman (output feedback only) |
| c ⭐ | Challenge: implement LQR + Kalman on hardware |
| d ⭐ | Challenge: implement RLS + LQR + Kalman on hardware |

**Workflow:** pulse experiment → Kalman smoother → Least-Squares → LQR + Kalman design → simulate → (deploy)

---

## System Model

**State vector:** `x = [xc; α; vc; ω]`
- `xc` — cart position (m)
- `α` — pendulum angle from upright (rad)
- `vc` — cart velocity (m/s)
- `ω` — pendulum angular velocity (rad/s)

**Input:** `u = Vm` (motor voltage, V)
**Output:** `y = [xc; α]` (both measured by encoders)

### Continuous-time model (small-angle linearisation)

```
Ac = [ 0,          0,           1,              0          ]
     [ 0,          0,           0,              1          ]
     [ 0,        gm/M,        -bc/M,         bp/(Ml)       ]
     [ 0,  -(M+m)g/(Ml),    bc/(Ml),  -(M+m)bp/(Mml²)     ]

Bc = [ 0; 0; Kf/M; -Kf/(Ml) ]

Cc = [ 1 0 0 0 ]
     [ 0 1 0 0 ]
```

### Forward-Euler discretisation

```
Ad = I4 + Ac·ts
Bd = Bc·ts
Cd = Cc
```

Note: only rows 3 and 4 (vc and ω equations) contain unknown parameters — rows 1 and 2 are pure kinematics.

### Unknown parameter vector (8 lumped params)

```
θ = [ gm/M;  bc/M;  bp/(Ml);  Kf/M;  (M+m)g/(Ml);  bc/(Ml);  (M+m)bp/(Mml²);  Kf/(Ml) ]
```

### Regression form

```
vc(k+1) - vc(k)  = ts · [ α(k), -vc(k),  ω(k),  Vm(k),  0,       0,      0,       0    ] · θ
ω(k+1)  - ω(k)   = ts · [ 0,     0,       0,     0,    -α(k),  vc(k), -ω(k),  -Vm(k) ] · θ
```

Stacking N-1 time steps → overdetermined system `b_l = A_l · θ`
`A_l ∈ R^(2(N-1) × 8)`,  `b_l ∈ R^(2(N-1))`

---

## What is Offline LS?

Offline LS means collecting **all data first**, then solving in one batch:

```
θ̂ = argmin ||A_l · θ - b_l||²
   = (A_l'A_l)⁻¹ A_l' b_l
```

In MATLAB: `A_l \ b_l`

**Offline vs Recursive:**

| | Offline LS | Recursive LS |
|---|---|---|
| When | After all data collected | Updates at each new time step |
| How | One big matrix solve | Sequential RLS formula |
| Use case | Post-processing in MATLAB | Real-time on hardware |
| Final result | Identical (mathematically equivalent) | Converges to same θ̂ as offline LS |

RLS is just an efficient incremental way to compute the same result — Figure 104 shows RLS (blue) converging toward the offline LS value (red dashed).

---

## Q1e — Kalman Smoother

Uses a **kinematic (constant-velocity) model** as a stand-in since physical parameters are unknown at this stage:

```matlab
Ad_kin = [1, 0, ts, 0;
          0, 1, 0,  ts;
          0, 0, 1,  0;
          0, 0, 0,  1];
```

**Two-pass algorithm:**
1. **Forward KF pass** — standard Kalman filter, left to right through data
2. **Backward RTS smoother pass** — uses forward estimates to smooth backwards

```
Smoother gain:  G_s = P · Ad' / (Ad · P · Ad' + Qf)
Backward pass:  x_smooth(k) = x_fwd(k) + G_s · (x_smooth(k+1) - Ad · x_fwd(k))
```

**Noise tuning:** `Qf = I·1`, `Rf = I·0.001` — Rf << Qf means trust the encoders (they're clean).

---

## Q1f/g — Why We Got Noisy LS Results

### Root cause: `corr(vc_s, omega_s) = 1.0000`

The Kalman smoother with kinematic model + low Rf essentially just differentiates the measurements:

```
vc_s    ≈ diff(xc) / ts
omega_s ≈ diff(alpha_rad) / ts
```

So `corr(vc_s, omega_s) = corr(diff(xc), diff(alpha_rad))`.

In the pulse experiment, a single coupled mode is excited — the cart moves and the pendulum angle follows **proportionally at every time step** (`xc ∝ alpha_rad`). So their derivatives are also proportional → `corr = 1.000`.

### Why this destroys the LS

When `ω = c·vc` and `α = d·vc`, the omega regression row becomes a scalar multiple of the vc regression row. The two equations that should independently identify 8 parameters are now saying the same thing — just scaled/negated.

Result:
- `rank(A_l) = 6/8` (two parameters unidentifiable)
- `cond(A_l) = 2.52e+17` (extreme ill-conditioning)
- Identified values are nonsensical: `theta(1) = -theta(5)`, `theta(4) = -theta(8)`, etc.

### How to fix it experimentally

| Method | Why it helps |
|---|---|
| PRBS or chirp input | Broadband → excites multiple modes, breaks proportional coupling |
| Initial angle offset | Start with pendulum tilted → cart and pendulum move with different trajectories |
| Pos + neg pulses with delay | Allows pendulum to swing freely between pulses, decorrelating states |

The experiment needs **persistent excitation** across all 8 parameter directions, not just one coupled mode.

### Ridge regression (regularisation)

When `A_l` is ill-conditioned, plain LS is unstable. Ridge regression adds a small penalty on `||θ||`:

```
θ̂_ridge = (A_l'A_l + λI)⁻¹ A_l'b_l
```

- Stabilises the solution at the cost of introducing bias
- Tune `λ` so that `θ(1), θ(4), θ(5), θ(8) > 0` (physically required signs)
- With this dataset, even large `λ` couldn't fix the sign issues due to rank deficiency

---

## Q2 — LQR + Kalman Design

### Issue: identified model was degenerate for LQR

Because the identification failed (rank 6/8), the reconstructed `Ac_id` had:
- Rows 3 and 4 identical (vc and ω equations indistinguishable)
- All open-loop eigenvalues on the unit circle (no unstable pendulum mode)
- `rank(ctrb) = 2/4` → not controllable

`lqr()` and `dlqr()` both require the unstable modes to be controllable → both failed.

**Fix:** Fall back to nominal physical parameters (Quanser-class system) when identified model fails validity check.

### LQR design

Designed on continuous-time model (`lqr`) — more numerically robust than `dlqr` on forward-Euler `Ad` when model is near-degenerate.

```matlab
Q_lqr = diag([10, 200, 1, 5]);   % heavily penalise angle deviation
R_lqr = 1;
K_lqr = lqr(Ac_lqr, Bc_lqr, Q_lqr, R_lqr);
```

### Q2b results (nominal model)

- xc tracks 0 → 0.2m reference cleanly (~2s settling)
- α stays within ±1° throughout (pendulum barely tilts)
- Kalman estimate (green) overlays true state (blue) almost perfectly
- Control input u is small (~0.6V max) — looks flat on ±5V scale but is nonzero

### Observer pole note

Observer poles at 0.031 and 0.037 — very aggressive (near dead-beat). Works cleanly in simulation but would amplify noise on real hardware. For hardware deployment, tune toward:

```matlab
Qk = diag([1e-2, 1e-2, 1, 1]);   % slower, more robust observer
Rk = diag([1e-4, 1e-4]);
```
This pushes observer poles toward 0.7–0.9 range.

---

## Key Results Summary

| Item | Value | Note |
|---|---|---|
| Sampling rate | 200 Hz (ts = 0.005s) | From Teensy 4.1 |
| A_l condition number | 2.52e+17 | Extremely ill-conditioned |
| A_l rank | 6/8 | Two parameters unidentifiable |
| corr(vc_s, omega_s) | 1.0000 | Root cause of rank deficiency |
| LQR gain K | [3.16, -16.94, 3.47, -1.26] | With nominal model |
| Closed-loop pole magnitudes | 0.78, 0.98, 0.997, 0.997 | All stable |
| Observer pole magnitudes | 0.031, 0.037, 0.974, 0.974 | Aggressive but stable |

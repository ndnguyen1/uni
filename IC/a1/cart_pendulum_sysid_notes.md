# Cart-Pendulum System Identification Notes
## 48580 Intelligent Control Studio — Assignment 1

---

## Overview

System: cart-pendulum (pendulum pointing downward)  
Unknowns: M, m, l, bc, bp, Kf  
Known: g = 9.81  
Inputs: vm (motor voltage)  
Outputs: xc (cart position), alpha (pendulum angle)  
State vector: x = [xc, alpha, vc, omega]'

---

## Part A — Continuous-Time State-Space (Analytical)

Apply small-angle approximations: cos(α)→1, sin(α)→α, α̇²→0

Linearised EOMs:
```
Kf*vm = (M+m)*xc_ddot + bc*vc + ml*alpha_ddot
0     = ml^2*alpha_ddot + bp*omega + ml*xc_ddot + gml*alpha
```

Solve simultaneously (det = Mml²):

```
xc_ddot    =  (gm/M)*alpha   - (bc/M)*vc   + (bp/(M*l))*omega   + (Kf/M)*vm
alpha_ddot = -(M+m)*g/(M*l)*alpha + (bc/(M*l))*vc - (M+m)*bp/(M*m*l^2)*omega - (Kf/(M*l))*vm
```

```matlab
Ac = [0,              0,          1,                       0             ];
     [0,              0,          0,                       1             ];
     [0,           gm/M,       -bc/M,                bp/(M*l)           ];
     [0, -(M+m)*g/(M*l), bc/(M*l),      -(M+m)*bp/(M*m*l^2)            ];

Bc = [0; 0; Kf/M; -Kf/(M*l)];

Cc = [1 0 0 0;
      0 1 0 0];
```

---

## Part B — Discretisation (Forward-Euler)

Substitute x_dot ≈ (x(k+1) - x(k)) / ts into ẋ = Ac*x + Bc*u:

```
x(k+1) = (I + Ac*ts)*x(k) + Bc*ts*u(k)
```

```matlab
Ad = eye(4) + Ac*ts;
Bd = Bc * ts;
Cd = Cc;  % output equation has no derivatives, unchanged
```

**Why Cd = Cc?** The output equation y = Cx has no derivatives, so discretisation doesn't affect it.

---

## Part C — Rearranging into LS Form b_l = A_l * theta

Write out all 4 rows of x(k+1) = Ad*x(k) + Bd*u(k):

```
xc(k+1)    = xc(k) + ts*vc(k)                                                              [row 1 - no unknowns]
alpha(k+1) = alpha(k) + ts*omega(k)                                                        [row 2 - no unknowns]
vc(k+1)    = (gm/M)*ts*alpha(k) + (1 - bc*ts/M)*vc(k) + (bp*ts/(M*l))*omega(k) + (Kf*ts/M)*vm(k)     [row 3]
omega(k+1) = -(M+m)*g/(M*l)*ts*alpha(k) + bc/(M*l)*ts*vc(k) + (1-(M+m)*bp/(M*m*l^2)*ts)*omega(k) - Kf/(M*l)*ts*vm(k)  [row 4]
```

Rows 1 and 2 are pure kinematics — no unknowns, skip for LS.

**Rearrange rows 3 and 4** — subtract current state from both sides to isolate unknown parameter terms:

Row 3:
```
vc(k+1) - vc(k) = (gm/M)*ts*alpha(k) - (bc/M)*ts*vc(k) + (bp/(M*l))*ts*omega(k) + (Kf/M)*ts*vm(k)
```

Row 4:
```
omega(k+1) - omega(k) = -(M+m)*g/(M*l)*ts*alpha(k) + bc/(M*l)*ts*vc(k) - (M+m)*bp/(M*m*l^2)*ts*omega(k) - Kf/(M*l)*ts*vm(k)
```

**Parameter vector (8 lumped parameters):**
```
theta = [gm/M; bc/M; bp/(M*l); Kf/M; (M+m)*g/(M*l); bc/(M*l); (M+m)*bp/(M*m*l^2); Kf/(M*l)]
```
- theta(1-4): from row 3 (vc equation)
- theta(5-8): from row 4 (omega equation)

**For each time step k, A_l has 2 rows and 8 columns:**
```
A_l(k) = ts * [ alpha(k), -vc(k), omega(k),  vm(k),     0,        0,          0,        0    ]
              [    0,        0,       0,        0,    -alpha(k),  vc(k),  -omega(k),  -vm(k)  ]

b_l(k) = [ vc(k+1)    - vc(k)    ]
          [ omega(k+1) - omega(k) ]
```

Full system: A_l is (2*(N-1) x 8), b_l is (2*(N-1) x 1)

Solve: `theta_hat = A_l \ b_l`

---

## Part D — Lab Data

Data loaded from `Teensy41_test_data_01.mat`:
- Vm: motor voltage input
- xc: cart position
- alpha: pendulum angle (radians)
- ts = 0.0025s (2.5ms sampling period)

Positive and negative pulse inputs applied to excite the system.

---

## Part E — Kalman Smoother

### Why not just differentiate xc and alpha?
`diff(xc)/ts` gives vc but amplifies measurement noise. Kalman smoother gives cleaner velocity estimates.

### Chicken-and-egg problem
- Need Ad numerically to run Kalman smoother
- Need LS parameters to build Ad numerically
- Need Kalman smoother to get clean states for LS

**Solution: use a kinematic (constant velocity) model for the smoother** — no unknown parameters needed.

### Kinematic model
```matlab
Ad_kin = [1, 0, ts, 0;
          0, 1, 0, ts;
          0, 0, 1, 0;
          0, 0, 0, 1];

Bd_kin = zeros(4, 1);  % no input in kinematic model
Cd = [1 0 0 0;
      0 1 0 0];
```

### Observability
System is observable because xc and alpha are measured directly, and vc/omega are just their time derivatives — recoverable from the kinematic relationship. Check: `rank(obsv(Ad_kin, Cd)) == 4`

### Qf and Rf tuning
- **Qf large** — kinematic model ignores real dynamics, so don't trust it much
- **Rf small** — encoders are clean sensors, trust measurements

```matlab
Qf = eye(4) * 1;
Rf = eye(2) * 0.001;
```

### Kalman gain
```matlab
[M_kal, P, Z, E] = dlqe(Ad_kin, eye(4), Cd, Qf, Rf);
```
Arguments: A, G (noise input matrix = eye(4)), C, Qf, Rf

### Forward filter pass
```matlab
N = length(time);
x_hat = zeros(4, 1);
x_hat_stored = zeros(4, N);

for k = 1:N
    x_pred_hat = Ad_kin * x_hat + Bd_kin * Vm(k);
    x_hat = x_pred_hat + M_kal * ([xc(k); alpha(k)] - Cd * x_pred_hat);
    x_hat_stored(:,k) = x_hat;
end
```

### Backward smoother pass
Smoother gain:
```matlab
G_s = P * Ad_kin' * inv(Ad_kin * P * Ad_kin' + Qf);
```

Backward pass:
```matlab
x_hat_smooth = zeros(4, N);
x_hat_smooth(:, N) = x_hat_stored(:, N);

for k = N-1:-1:1
    x_hat_smooth(:,k) = x_hat_stored(:,k) + G_s * (x_hat_smooth(:,k+1) - Ad_kin * x_hat_stored(:,k));
end
```

**Smoother vs filter:** Forward filter uses data up to k only. Smoother uses all data (past + future), giving more accurate and physically consistent velocity estimates. Smoother vc/omega show slightly smaller peaks and smoother transitions.

### Extract smoothed states
```matlab
xc_s    = x_hat_smooth(1,:)';
alpha_s = x_hat_smooth(2,:)';
vc_s    = x_hat_smooth(3,:)';
omega_s = x_hat_smooth(4,:)';
```

---

## Part F — Offline Least Squares (TODO)

Build A_l and b_l from smoothed states, then solve:

```matlab
% Preallocate
A_l = zeros(2*(N-1), 8);
b_l = zeros(2*(N-1), 1);

for k = 1:N-1
    row = 2*k - 1;
    A_l(row,   :) = ts * [alpha_s(k), -vc_s(k), omega_s(k),  Vm(k),       0,          0,           0,        0   ];
    A_l(row+1, :) = ts * [     0,          0,        0,         0,    -alpha_s(k), vc_s(k),  -omega_s(k), -Vm(k)  ];
    b_l(row)   = vc_s(k+1)    - vc_s(k);
    b_l(row+1) = omega_s(k+1) - omega_s(k);
end

theta_hat = A_l \ b_l;
```

---

## Part G — Recursive Least Squares (TODO)

---

## Key Concepts

| Concept | Meaning |
|---|---|
| Qf large vs Rf | Trust measurements more than model |
| Qf small vs Rf | Trust model more than measurements |
| Forward filter | Uses data up to k to estimate x(k) |
| Backward smoother | Uses all data to refine estimates at each k |
| Kinematic model | Assumes constant velocity — valid at fast sampling rates |
| G in dlqe | Noise input matrix — which states process noise enters |
| theta | 8 lumped parameter combinations to be identified |
| b_l = A_l*theta | Linear regression form for least squares |

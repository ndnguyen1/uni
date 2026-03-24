%% Assignment 1 — Complete Solution
%  Q1: System Identification  (60 marks)
%  Q2: LQR Control Design     (40 marks)
%
%  48580 Intelligent Control Studio
%  University of Technology Sydney — Autumn 2026
%  Coordinator: A/Prof Ricardo P. Aguilera
%
%  State vector: x = [xc; alpha; vc; omega]
%    xc    — cart position        (m)
%    alpha — pendulum angle       (rad, from upright)
%    vc    — cart velocity        (m/s)
%    omega — pendulum ang. vel.   (rad/s)
%  Input:  u = Vm  (motor voltage, V)
%  Output: y = [xc; alpha]

clc; clear; close all;
format long
s = tf('s');


%% ========================================================================
%  Q1a — CONTINUOUS-TIME LINEARISED STATE-SPACE MODEL
%% ========================================================================
%
%  Under small-angle (sin(alpha) ~ alpha, cos(alpha) ~ 1) and slow
%  angular-speed (alpha_dot^2 ~ 0) approximations the equations of motion
%  linearise to:
%
%    xc_dot    =  vc
%    alpha_dot =  omega
%    vc_dot    =  (gm/M)*alpha  - (bc/M)*vc   + (bp/(M*l))*omega  + (Kf/M)*Vm
%    omega_dot = -((M+m)*g/(M*l))*alpha + (bc/(M*l))*vc
%                  - ((M+m)*bp/(M*m*l^2))*omega - (Kf/(M*l))*Vm
%
%  Physical parameters:
%    M  — cart mass (kg)          m  — pendulum bob mass (kg)
%    l  — pendulum length (m)     g  = 9.81 m/s^2
%    bc — cart friction (N.s/m)   bp — pendulum friction (N.m.s/rad)
%    Kf — motor force constant (N/V)
%
%         [  0        0           1             0       ]
%  Ac  =  [  0        0           0             1       ]
%         [  0      gm/M       -bc/M        bp/(Ml)     ]
%         [  0  -(M+m)g/(Ml)  bc/(Ml)  -(M+m)bp/(Mml^2)]
%
%  Bc  =  [ 0;  0;  Kf/M;  -Kf/(Ml) ]
%
%  Cc  =  [ 1  0  0  0 ]
%         [ 0  1  0  0 ]

%% ========================================================================
%  Q1b — FORWARD-EULER DISCRETISATION
%% ========================================================================
%
%  Ad = I4 + Ac * ts
%  Bd = Bc * ts
%  Cd = Cc
%
%  Expanding each row:
%    xc(k+1)    = xc(k) + ts*vc(k)                                        [no unknowns]
%    alpha(k+1) = alpha(k) + ts*omega(k)                                   [no unknowns]
%    vc(k+1)    = (gm/M)*ts*alpha(k) + (1 - bc*ts/M)*vc(k)
%                  + (bp*ts/(Ml))*omega(k) + (Kf*ts/M)*Vm(k)
%    omega(k+1) = -(M+m)g/(Ml)*ts*alpha(k) + bc/(Ml)*ts*vc(k)
%                  + (1 - (M+m)bp/(Mml^2)*ts)*omega(k) - Kf/(Ml)*ts*Vm(k)
%
%  Only rows 3 and 4 contain unknown parameters — used for identification.

%% ========================================================================
%  Q1c — REGRESSION FORM  b_l = A_l * theta
%% ========================================================================
%
%  Lumped parameter vector (8 unknowns):
%    theta = [ theta1; theta2; theta3; theta4; theta5; theta6; theta7; theta8 ]
%          = [ gm/M;  bc/M;  bp/(Ml);  Kf/M;  (M+m)g/(Ml);  bc/(Ml);  (M+m)bp/(Mml^2);  Kf/(Ml) ]
%
%  Rearranging rows 3 and 4 of the discrete model:
%
%    vc(k+1) - vc(k)    = ts * [ alpha(k), -vc(k), omega(k), Vm(k),  0,        0,       0,        0      ] * theta
%    omega(k+1) - omega(k) = ts * [ 0,       0,       0,       0,    -alpha(k), vc(k), -omega(k), -Vm(k) ] * theta
%
%  Stacking N-1 time steps gives the overdetermined system:
%    b_l = A_l * theta,   A_l in R^(2(N-1) x 8),  b_l in R^(2(N-1))

%% ========================================================================
%  Q1d — LOAD & PLOT EXPERIMENTAL DATA                                
%% ========================================================================

load('Teensy41_test_data_04.mat')

Nini      = 1;
time      = logsout{1}.Values.Time(Nini:end-1);
time      = time - time(1);
Vm        = logsout{1}.Values.Data(Nini:end-1);
alpha     = -squeeze(logsout{2}.Values.Data(Nini:end-1));
xc        = squeeze(logsout{5}.Values.Data(Nini:end-1))/10000;
alpha_rad = deg2rad(alpha);

ts = time(2);       % sampling period (s)
N  = length(time);

figure(101)
subplot(311)
  plot(time, Vm, 'LineWidth', 2)
  title('Q1d — Experimental SysId Data (pulse inputs)')
  ylim([-8 8]); yticks([-6 0 6]); grid
  ylabel('$u(t)$ [V]', 'Interpreter', 'latex', 'FontSize', 14)
subplot(312)
  plot(time, xc, 'LineWidth', 2); grid
  ylabel('$x_c(t)$ [m]', 'Interpreter', 'latex', 'FontSize', 14)
subplot(313)
  plot(time, alpha, 'LineWidth', 2); grid
  ylabel('$\alpha(t)$ [deg]', 'Interpreter', 'latex', 'FontSize', 14)
  xlabel('Time (s)', 'FontSize', 14)

  % U = [Vm] (voltage input vector 1xN)
  % Y = [xc; alpha_rad] (2×N, stacking cart position and pendulum angle in radians)

%% ========================================================================
% Q1e — KALMAN SMOOTHER (forward KF pass)
%% ========================================================================

fprintf('=== Q1e: Kalman Smoother ===\n')

%  Use a constant-velocity kinematic model as a stand-in for Ac
%  (physical parameters are unknown at this stage)
Ad_kin = [1, 0, ts, 0;
          0, 1, 0,  ts;
          0, 0, 1,  0;
          0, 0, 0,  1];
Cd = [1 0 0 0;
      0 1 0 0];

%  Prove observability of kinematic model
Ob_kin = obsv(Ad_kin, Cd);
fprintf('Kinematic model observability rank: %d / 4', rank(Ob_kin))
if rank(Ob_kin) == 4
    fprintf(' -> OBSERVABLE\n')
else
    fprintf(' -> NOT OBSERVABLE\n')
end

%  Noise covariances — encoders are clean so Rf << Qf (trust measurements)
Qf = eye(4) * 0.1;
Rf = eye(2) * 0.0001;
[M_kal, P_kal, ~, ~] = dlqe(Ad_kin, eye(4), Cd, Qf, Rf);

%  --- Forward Kalman s ---
Bd_kin   = zeros(4, 1);
x_hat    = zeros(4, 1);
x_fwd    = zeros(4, N);

for k = 1:N
    x_pred     = Ad_kin * x_hat + Bd_kin * Vm(k);
    innov      = [xc(k); alpha_rad(k)] - Cd * x_pred;
    x_hat      = x_pred + M_kal * innov;
    x_fwd(:,k) = x_hat;
end

%  Extract smoothed states
xc_s    = x_fwd(1,:);
alpha_s = x_fwd(2,:);   % rad
vc_s    = x_fwd(3,:);
omega_s = x_fwd(4,:);

%  Plots


figure(103)
sgtitle('Q1e — Kalman Smoother')
subplot(411); plot(time, x_fwd(1,:), 'LineWidth', 2); ylabel('xc (m)'); grid
subplot(412); plot(time, x_fwd(2,:), 'LineWidth', 2); ylabel('\alpha (rad)'); grid
subplot(413); plot(time, x_fwd(3,:), 'LineWidth', 2); ylabel('vc (m/s)'); grid
subplot(414); plot(time, x_fwd(4,:), 'LineWidth', 2); ylabel('\omega (rad/s)'); grid
xlabel('Time (s)')

%% ========================================================================
%  Q1f — OFFLINE LEAST-SQUARES IDENTIFICATION
%% ========================================================================

fprintf('\n=== Q1f: Offline Least-Squares ===\n')

%  Build regression matrices from smoothed state data
A_l = zeros(2*(N-1), 8);
b_l = zeros(2*(N-1), 1);

for k = 1:N-1
    row = 2*k - 1;
    A_l(row,   :) = ts * [ alpha_s(k), -vc_s(k),  omega_s(k),  Vm(k),  0,           0,         0,           0      ];
    A_l(row+1, :) = ts * [ 0,           0,          0,          0,     -alpha_s(k),  vc_s(k),  -omega_s(k), -Vm(k)  ];
    b_l(row)      = vc_s(k+1)    - vc_s(k);
    b_l(row+1)    = omega_s(k+1) - omega_s(k);
end

%  Condition diagnostics
sv       = svd(A_l);
cond_num = sv(1) / sv(end);
fprintf('A_l condition number:  %.2e\n', cond_num)
fprintf('A_l rank:              %d / 8\n', rank(A_l))
oms = omega_s - mean(omega_s);  vcs = vc_s - mean(vc_s);
pearson_corr = (oms * vcs') / (norm(oms) * norm(vcs));
fprintf('corr(omega_s, vc_s):   %.4f  (near 1 => ill-conditioned)\n', pearson_corr)

figure(105)
semilogy(sv, 'o-', 'LineWidth', 2)
title('Q1f — Singular values of A\_l  (large gap => rank deficiency)')
xlabel('Index'); ylabel('Singular value'); grid

%  Least-squares solution
theta_hat = A_l \ b_l;

theta_labels = {'gm/M',         'bc/M',    'bp/(Ml)',    'Kf/M', ...
                '(M+m)g/(Ml)',  'bc/(Ml)', '(M+m)bp/(Mml^2)', 'Kf/(Ml)'};

fprintf('\n  %-28s  %12s\n', 'Parameter', 'LS')
for i = 1:8
    fprintf('  theta(%d)  %-22s  %12.6f\n', i, theta_labels{i}, theta_hat(i))
end

fprintf('\nPhysical sign checks (all should be > 0):\n')
fprintf('  theta(1)  gm/M        = %10.6f\n', theta_hat(1))
fprintf('  theta(4)  Kf/M        = %10.6f\n', theta_hat(4))
fprintf('  theta(5)  (M+m)g/Ml   = %10.6f\n', theta_hat(5))
fprintf('  theta(8)  Kf/(Ml)     = %10.6f\n', theta_hat(8))

%% ========================================================================
%  Q1g — RECURSIVE LEAST-SQUARES
%% ========================================================================

fprintf('\n=== Q1g: Recursive Least-Squares ===\n')

theta_rls        = zeros(8, 1);
P_rls            = eye(8) * 1e6;   % large initial covariance = uninformed prior
theta_rls_stored = zeros(8, N-1);

for k = 1:N-1
    Phi_k = ts * [ alpha_s(k), -vc_s(k),  omega_s(k),  Vm(k),  0,           0,         0,           0;
                   0,           0,          0,           0,     -alpha_s(k),  vc_s(k),  -omega_s(k), -Vm(k) ];
    y_k   = [ vc_s(k+1) - vc_s(k); omega_s(k+1) - omega_s(k) ];

    K_rls     = P_rls * Phi_k' / (eye(2) + Phi_k * P_rls * Phi_k');
    theta_rls = theta_rls + K_rls * (y_k - Phi_k * theta_rls);
    P_rls     = (eye(8) - K_rls * Phi_k) * P_rls;

    theta_rls_stored(:,k) = theta_rls;
end

fprintf('\n  %-28s  %12s  %12s\n', 'Parameter', 'RLS final', 'Offline LS')
for i = 1:8
    fprintf('  theta(%d)  %-22s  %12.6f  %12.6f\n', i, theta_labels{i}, theta_rls(i), theta_hat(i))
end

figure(104)
for i = 1:8
    subplot(4, 2, i)
    plot(time(1:N-1), theta_rls_stored(i,:), 'b', 'LineWidth', 1.5); hold on
    yline(theta_hat(i), 'r--', 'LineWidth', 1.5); hold off
    ylabel(['\theta_{' num2str(i) '}: ' theta_labels{i}], 'Interpreter', 'tex')
    grid
    if i >= 7; xlabel('Time (s)'); end
    if i == 1; legend('RLS', 'Offline LS', 'Location', 'best'); end
end
sgtitle('Q1g — RLS convergence (blue) vs Offline LS (red dashed)')

%% ========================================================================
%  RECONSTRUCT IDENTIFIED CONTINUOUS & DISCRETE MODEL FROM theta_hat
%% ========================================================================

th = theta_hat;   % use ridge LS result

%  Continuous-time identified Ac and Bc
%  (each entry maps directly from the theta vector)
Ac_id = [0,    0,       1,       0;
         0,    0,       0,       1;
         0,    th(1),  -th(2),   th(3);
         0,   -th(5),   th(6),  -th(7)];

Bc_id = [0; 0; th(4); -th(8)];
Cc_id = [1 0 0 0;
         0 1 0 0];

%  Discrete-time (forward-Euler, consistent with identification)
Ad_id = eye(4) + Ac_id * ts;
Bd_id = Bc_id * ts;
Cd_id = Cc_id;

fprintf('\n=== Identified Model ===\n')
fprintf('Ac_id =\n'); disp(Ac_id)
fprintf('Bc_id =\n'); disp(Bc_id)
fprintf('Ad_id =\n'); disp(Ad_id)
fprintf('Bd_id =\n'); disp(Bd_id)

%  Sanity check: open-loop eigenvalues
%  Continuous: expect one positive real eigenvalue (unstable pendulum mode)
%  Discrete:   expect one eigenvalue outside the unit circle
ol_eig_c = eig(Ac_id);
ol_eig_d = eig(Ad_id);
fprintf('Open-loop continuous eigenvalues:\n'); disp(ol_eig_c)
fprintf('Open-loop discrete eigenvalue magnitudes: ')
fprintf('%.4f  ', abs(ol_eig_d)); fprintf('\n')

%  Check physical validity of identified model for LQR use.
%  A valid inverted-pendulum model must have at least one positive real
%  eigenvalue (unstable mode) and correct parameter signs.
id_model_valid = any(real(ol_eig_c) > 0.1) && ...
                 all([th(1), th(4), th(5), th(8)] > 0);

if ~id_model_valid
    fprintf(['\nWARNING: Identified model is not physically valid for LQR.\n' ...
             '  Cause: rank deficiency (rank %d/8) from insufficient\n' ...
             '  independent excitation in the experimental data\n' ...
             '  (corr(omega_s, vc_s) = 1 => vc/omega equations are\n' ...
             '  linearly dependent => parameters unidentifiable).\n' ...
             '  Falling back to nominal physical parameters for Q2.\n\n'], rank(A_l))
    %  Nominal parameters for a typical lab cart-pendulum (Quanser-class).
    %  Replace with your system''s true values if known.
    %    M  = 0.57 kg,  m = 0.127 kg,  l = 0.3365 m
    %    bc = 0.5 N.s/m,  bp = 0.0024 N.m.s/rad,  Kf = 7.5 N/V
    th_lqr = [2.19;    % theta1: gm/M
               0.88;    % theta2: bc/M
               0.0125;  % theta3: bp/(Ml)
              13.16;    % theta4: Kf/M
              35.70;    % theta5: (M+m)g/(Ml)
               2.61;    % theta6: bc/(Ml)
               0.204;   % theta7: (M+m)bp/(Mml^2)
              39.09];   % theta8: Kf/(Ml)
    fprintf('Using nominal theta for Q2:\n')
    for i = 1:8
        fprintf('  theta(%d)  %-22s  %10.4f\n', i, theta_labels{i}, th_lqr(i))
    end
    Ac_lqr = [0,    0,           1,           0;
              0,    0,           0,           1;
              0,    th_lqr(1),  -th_lqr(2),   th_lqr(3);
              0,   -th_lqr(5),   th_lqr(6),  -th_lqr(7)];
    Bc_lqr = [0; 0; th_lqr(4); -th_lqr(8)];
    Ad_lqr = eye(4) + Ac_lqr * ts;
    Bd_lqr = Bc_lqr * ts;
else
    fprintf('\nIdentified model is physically valid. Using it for Q2.\n')
    Ac_lqr = Ac_id;  Bc_lqr = Bc_id;
    Ad_lqr = Ad_id;  Bd_lqr = Bd_id;
    th_lqr = th;
end

%% ========================================================================
%  Q2a — LQR WITH FULL STATE FEEDBACK
%% ========================================================================

fprintf('\n=== Q2a: LQR with Full State Feedback ===\n')

%  --- Controllability ---
Co = ctrb(Ad_lqr, Bd_lqr);
fprintf('Controllability matrix rank: %d / 4', rank(Co))
if rank(Co) == 4
    fprintf(' -> CONTROLLABLE\n')
else
    fprintf(' -> NOT CONTROLLABLE\n')
end

%  --- LQR design ---
%  States: [xc (m), alpha (rad), vc (m/s), omega (rad/s)]
%  Heavily penalise angle — keeping the pendulum upright is the priority.
%  Then penalise cart position to bring it to the reference.
Q_lqr = diag([10, 200, 1, 5]);
R_lqr = 1;

%  Design on continuous-time model — more numerically robust than dlqr
%  on the forward-Euler Ad at this sampling rate (200 Hz).
K_lqr = lqr(Ac_lqr, Bc_lqr, Q_lqr, R_lqr);
fprintf('LQR gain K = [%.4f  %.4f  %.4f  %.4f]\n', K_lqr)

%  Closed-loop poles (discrete — should all be inside unit circle)
cl_poles_a = eig(Ad_lqr - Bd_lqr * K_lqr);
fprintf('Closed-loop pole magnitudes: ')
fprintf('%.4f  ', abs(cl_poles_a)); fprintf('\n')
if all(abs(cl_poles_a) < 1)
    fprintf('All poles inside unit circle -> STABLE\n')
else
    fprintf('WARNING: unstable closed-loop poles detected\n')
end

%  --- Closed-loop simulation ---
%  Scenario: cart starts at rest at the origin, balance the pendulum
%  and drive cart to x_ref = 0.2 m.
N_sim  = round(10 / ts);     % 10-second simulation
x_ref  = [0.2; 0; 0; 0];    % desired equilibrium
u_star = 0;                  % equilibrium input
x0     = zeros(4, 1);        % initial state
u_max  = 6;                  % voltage saturation (V)

x_sim_a      = zeros(4, N_sim);
u_sim_a      = zeros(1, N_sim);
x_sim_a(:,1) = x0;

for k = 1:N_sim-1
    u              = -K_lqr * (x_sim_a(:,k) - x_ref) + u_star;
    u              = max(min(u, u_max), -u_max);
    u_sim_a(k)     = u;
    x_sim_a(:,k+1) = Ad_lqr * x_sim_a(:,k) + Bd_lqr * u;
end

t_sim = (0:N_sim-1) * ts;

figure(201)
sgtitle('Q2a — LQR with Full State Feedback')
subplot(511)
  plot(t_sim, x_sim_a(1,:), 'b', 'LineWidth', 2); hold on
  yline(x_ref(1), 'r--', 'LineWidth', 1.5); hold off
  ylabel('xc (m)'); grid; legend('response', 'reference')
subplot(512)
  plot(t_sim, rad2deg(x_sim_a(2,:)), 'b', 'LineWidth', 2); hold on
  yline(0, 'r--', 'LineWidth', 1.5); hold off
  ylabel('\alpha (deg)'); grid
subplot(513)
  plot(t_sim, x_sim_a(3,:), 'b', 'LineWidth', 2)
  ylabel('vc (m/s)'); grid
subplot(514)
  plot(t_sim, x_sim_a(4,:), 'b', 'LineWidth', 2)
  ylabel('\omega (rad/s)'); grid
subplot(515)
  stairs(t_sim, u_sim_a, 'b', 'LineWidth', 2); hold on
  yline(u_max,  'r--', 'LineWidth', 1); yline(-u_max, 'r--', 'LineWidth', 1); hold off
  ylabel('u (V)'); xlabel('Time (s)'); grid

%% ========================================================================
%  Q2b — LQR + KALMAN FILTER (OUTPUT FEEDBACK)
%% ========================================================================

fprintf('\n=== Q2b: LQR + Kalman Filter (output feedback) ===\n')

%  --- Observability (with LQR model) ---
Ob = obsv(Ad_lqr, Cd_id);
fprintf('Observability matrix rank: %d / 4', rank(Ob))
if rank(Ob) == 4
    fprintf(' -> OBSERVABLE\n')
else
    fprintf(' -> NOT OBSERVABLE\n')
end

%  --- Kalman filter design (real-time, forward only) ---
%  Process noise Q: model uncertainty is larger for velocity states.
%  Measurement noise R: encoders are clean, so small.
Qk = diag([1e-4, 1e-4, 1e-2, 1e-2]);
Rk = diag([1e-6, 1e-6]);
[M_kf, ~, ~, ~] = dlqe(Ad_lqr, eye(4), Cd_id, Qk, Rk);
fprintf('Kalman filter gain M_kf =\n'); disp(M_kf)

%  Observer poles (separation principle — can be designed independently)
obs_poles = eig(Ad_lqr - M_kf * Cd_id);
fprintf('Observer pole magnitudes: ')
fprintf('%.4f  ', abs(obs_poles)); fprintf('\n')
if all(abs(obs_poles) < 1)
    fprintf('All observer poles inside unit circle -> STABLE\n')
else
    fprintf('WARNING: unstable observer poles detected\n')
end

%  --- Closed-loop simulation with measurement noise ---
rng(42)   % reproducible random seed
meas_noise_std = 1e-4;   % encoder resolution (m and rad)

x_sim_b       = zeros(4, N_sim);
x_hat_b       = zeros(4, N_sim);
u_sim_b       = zeros(1, N_sim);
x_sim_b(:,1)  = x0;
x_hat_b(:,1)  = zeros(4, 1);   % estimator starts with no knowledge

for k = 1:N_sim-1
    %  Control law uses estimated state
    u          = -K_lqr * (x_hat_b(:,k) - x_ref) + u_star;
    u          = max(min(u, u_max), -u_max);
    u_sim_b(k) = u;

    %  True plant advances one step
    x_sim_b(:,k+1) = Ad_lqr * x_sim_b(:,k) + Bd_lqr * u;

    %  Noisy measurement at k+1
    y_meas = Cd_id * x_sim_b(:,k+1) + meas_noise_std * randn(2,1);

    %  Kalman predict + update
    x_pred        = Ad_lqr * x_hat_b(:,k) + Bd_lqr * u;
    x_hat_b(:,k+1) = x_pred + M_kf * (y_meas - Cd_id * x_pred);
end

figure(202)
sgtitle('Q2b — LQR + Kalman Filter (output feedback)')
subplot(511)
  plot(t_sim, x_sim_b(1,:), 'b', 'LineWidth', 2); hold on
  plot(t_sim, x_hat_b(1,:), 'g--', 'LineWidth', 1.5)
  yline(x_ref(1), 'r--', 'LineWidth', 1.5); hold off
  ylabel('xc (m)'); grid; legend('true', 'estimated', 'reference')
subplot(512)
  plot(t_sim, rad2deg(x_sim_b(2,:)), 'b', 'LineWidth', 2); hold on
  plot(t_sim, rad2deg(x_hat_b(2,:)), 'g--', 'LineWidth', 1.5); hold off
  ylabel('\alpha (deg)'); grid; legend('true', 'estimated')
subplot(513)
  plot(t_sim, x_sim_b(3,:), 'b', 'LineWidth', 2); hold on
  plot(t_sim, x_hat_b(3,:), 'g--', 'LineWidth', 1.5); hold off
  ylabel('vc (m/s)'); grid
subplot(514)
  plot(t_sim, x_sim_b(4,:), 'b', 'LineWidth', 2); hold on
  plot(t_sim, x_hat_b(4,:), 'g--', 'LineWidth', 1.5); hold off
  ylabel('\omega (rad/s)'); grid
subplot(515)
  stairs(t_sim, u_sim_b, 'b', 'LineWidth', 2); hold on
  yline(u_max,  'r--', 'LineWidth', 1); yline(-u_max, 'r--', 'LineWidth', 1); hold off
  ylabel('u (V)'); xlabel('Time (s)'); grid

%% ========================================================================
%  SUMMARY
%% ========================================================================

fprintf('\n=== Run Complete ===\n')
fprintf('Figures generated:\n')
fprintf('  101 — Raw experimental data                        [Q1d]\n')
fprintf('  102 — Forward Kalman filter state estimates        [Q1e]\n')
fprintf('  103 — Kalman smoother (RTS) state estimates        [Q1e]\n')
fprintf('  104 — RLS convergence vs Offline LS                [Q1g]\n')
fprintf('  105 — Singular values of A_l                       [Q1f]\n')
fprintf('  201 — LQR full state feedback simulation           [Q2a]\n')
fprintf('  202 — LQR + Kalman filter simulation               [Q2b]\n')

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
alpha     = squeeze(logsout{2}.Values.Data(Nini:end-1));
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
Qf = diag([0.001, 0.001, 0.5, 0.5]);
Rf = eye(2) * 0.0001;
[M_kal, P_kal, ~, ~] = dlqe(Ad_kin, eye(4), Cd, Qf, Rf);

%  --- Forward Kalman ---
Bd_kin   = zeros(4, 1);
x_hat    = zeros(4, 1);
x_fwd    = zeros(4, N);

for k = 1:N
    x_pred     = Ad_kin * x_hat + Bd_kin * Vm(k);
    innov      = [xc(k); alpha_rad(k)] - Cd * x_pred;
    x_hat      = x_pred + M_kal * innov;
    x_fwd(:,k) = x_hat;
end

xc_hat    = x_fwd(1,:);
alpha_hat = x_fwd(2,:);
vc_hat    = x_fwd(3,:);
omega_hat = x_fwd(4,:);

figure;

subplot(4,1,1);
plot(time, Vm);
ylabel('Vm [V]');
title('Input and Kalman Filter State Estimates');

subplot(4,1,2);
plot(time, xc, 'b', time, x_fwd(1,:), 'r--');
ylabel('xc [m]');
legend('measured', 'estimated');

subplot(4,1,3);
plot(time, alpha_rad, 'b', time, x_fwd(2,:), 'r--');
ylabel('\alpha [rad]');
legend('measured', 'estimated');

subplot(4,1,4);
plot(time, x_fwd(3,:), 'r', time, x_fwd(4,:), 'b');
ylabel('Velocity estimates');
legend('vc [m/s]', '\omega [rad/s]');
xlabel('Time [s]');


%% ========================================================================
%  Q1f — OFFLINE LEAST-SQUARES IDENTIFICATION
%% ========================================================================

fprintf('\n=== Q1f: Offline Least-Squares ===\n')

%  Build regression matrices from smoothed state data
A_l = zeros(2*(N-1), 8);
b_l = zeros(2*(N-1), 1);

for k = 1:N-1
    row = 2*k - 1;
    A_l(row,   :) = ts * [ alpha_hat(k), -vc_hat(k),  omega_hat(k),  Vm(k),  0,           0,         0,           0      ];
    A_l(row+1, :) = ts * [ 0,           0,          0,          0,     -alpha_hat(k),  vc_hat(k),  -omega_hat(k), -Vm(k)  ];
    b_l(row)      = vc_hat(k+1)    - vc_hat(k);
    b_l(row+1)    = omega_hat(k+1) - omega_hat(k);
end

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


sv = svd(A_l);
cond_num = sv(1) / sv(end);
fprintf('A_l condition number: %.2e\n', cond_num)
fprintf('A_l rank: %d / 8\n', rank(A_l))


%% ========================================================================
%  Q1g — RECURSIVE LEAST-SQUARES
%% ========================================================================

fprintf('\n=== Q1g: Recursive Least-Squares ===\n')
theta_rls        = zeros(8, 1);
P_rls            = eye(8) * 1e6;
Ri               = eye(2);
theta_rls_stored = zeros(8, N-1);

for k = 1:N-1
    Phi_k = ts * [ alpha_hat(k), -vc_hat(k),  omega_hat(k),  Vm(k),  0,             0,           0,          0;
                   0,             0,           0,             0,      -alpha_hat(k),  vc_hat(k),  -omega_hat(k), -Vm(k) ];
    y_k   = [ vc_hat(k+1) - vc_hat(k); omega_hat(k+1) - omega_hat(k) ];

    K_rls     = P_rls * Phi_k' / (Phi_k * P_rls * Phi_k' + Ri);
    theta_rls = theta_rls + K_rls * (y_k - Phi_k * theta_rls);
    P_rls     = (eye(8) - K_rls * Phi_k) * P_rls;
    theta_rls_stored(:,k) = theta_rls;
end

fprintf('\n  %-28s  %12s  %12s\n', 'Parameter', 'LS', 'RLS')
for i = 1:8
    fprintf('  theta(%d)  %-22s  %12.6f  %12.6f\n', i, theta_labels{i}, theta_hat(i), theta_rls(i))
end

figure;
for i = 1:8
    subplot(4,2,i);
    plot(time(1:N-1), theta_rls_stored(i,:));
    yline(theta_hat(i), 'r--');
    title(theta_labels{i});
    grid on;
end
sgtitle('RLS Parameter Convergence vs Batch LS');


%% Model validation — simulate identified model vs measured data
A_id = [1,  0,                      ts,                 0;
        0,  1,                      0,                  ts;
        0,  theta_hat(1)*ts,        1-theta_hat(2)*ts,  theta_hat(3)*ts;
        0, -theta_hat(5)*ts,        theta_hat(6)*ts,    1-theta_hat(7)*ts];

B_id = [0;
        0;
        theta_hat(4)*ts;
       -theta_hat(8)*ts];

C_id = [1 0 0 0;
        0 1 0 0];

x_sim = zeros(4, N);
x_sim(:,1) = x_fwd(:,1);  % initialise from Kalman estimate

for k = 1:N-1
    x_sim(:,k+1) = A_id * x_sim(:,k) + B_id * Vm(k);
end

figure;
subplot(2,1,1);
plot(time, xc, 'b', time, x_sim(1,:), 'r--', 'LineWidth', 1.5);
ylabel('xc [m]'); legend('measured', 'simulated'); grid on;
title('Model Validation — Batch LS Parameters');

subplot(2,1,2);
plot(time, alpha_rad, 'b', time, x_sim(2,:), 'r--', 'LineWidth', 1.5);
ylabel('\alpha [rad]'); legend('measured', 'simulated'); grid on;
xlabel('Time [s]');


%% Iterative re-identification
theta_fix = theta_hat;
theta_fix(7) = abs(theta_fix(7));  % force positive damping

A_iter = [1,  0,                      ts,                 0;
          0,  1,                      0,                  ts;
          0,  theta_fix(1)*ts,        1-theta_fix(2)*ts,  theta_fix(3)*ts;
          0, -theta_fix(5)*ts,        theta_fix(6)*ts,    1-theta_fix(7)*ts];

B_iter = [0; 0; theta_fix(4)*ts; -theta_fix(8)*ts];

% Check stability
fprintf('Corrected eigenvalues: '); disp(abs(eig(A_iter))');

% Re-run Kalman filter with identified model
[Kf_iter, ~, ~, ~] = dlqe(A_iter, eye(4), C_id, diag([0.001, 0.001, 0.1, 0.1]), eye(2)*0.0001);

x_hat2 = zeros(4, 1);
x_fwd2 = zeros(4, N);
for k = 1:N
    x_pred = A_iter * x_hat2 + B_iter * Vm(k);
    innov  = [xc(k); alpha_rad(k)] - C_id * x_pred;
    x_hat2 = x_pred + Kf_iter * innov;
    x_fwd2(:,k) = x_hat2;
end

% Re-identify with improved state estimates
A_l2 = zeros(2*(N-1), 8);
b_l2 = zeros(2*(N-1), 1);
for k = 1:N-1
    row = 2*k - 1;
    A_l2(row,   :) = ts * [ x_fwd2(2,k), -x_fwd2(3,k),  x_fwd2(4,k),  Vm(k), 0, 0, 0, 0 ];
    A_l2(row+1, :) = ts * [ 0, 0, 0, 0, -x_fwd2(2,k),  x_fwd2(3,k), -x_fwd2(4,k), -Vm(k) ];
    b_l2(row)      = x_fwd2(3,k+1) - x_fwd2(3,k);
    b_l2(row+1)    = x_fwd2(4,k+1) - x_fwd2(4,k);
end

theta_hat2 = A_l2 \ b_l2;

fprintf('\n  %-28s  %12s  %12s\n', 'Parameter', 'LS (iter 1)', 'LS (iter 2)')
for i = 1:8
    fprintf('  theta(%d)  %-22s  %12.6f  %12.6f\n', i, theta_labels{i}, theta_hat(i), theta_hat2(i))
end


A_id2 = [1,  0,                       ts,                  0;
         0,  1,                       0,                   ts;
         0,  theta_hat2(1)*ts,        1-theta_hat2(2)*ts,  theta_hat2(3)*ts;
         0, -theta_hat2(5)*ts,        theta_hat2(6)*ts,    1-theta_hat2(7)*ts];

B_id2 = [0; 0; theta_hat2(4)*ts; -theta_hat2(8)*ts];

fprintf('Eigenvalue magnitudes: '); disp(abs(eig(A_id2))');



x_sim2 = zeros(4, N);
x_sim2(:,1) = x_fwd2(:,1);
for k = 1:N-1
    x_sim2(:,k+1) = A_id2 * x_sim2(:,k) + B_id2 * Vm(k);
end

figure;
subplot(2,1,1);
plot(time, xc, 'b', time, x_sim2(1,:), 'r--', 'LineWidth', 1.5);
ylabel('xc [m]'); legend('measured', 'simulated'); grid on;
title('Model Validation — Iter 2 Parameters');
subplot(2,1,2);
plot(time, alpha_rad, 'b', time, x_sim2(2,:), 'r--', 'LineWidth', 1.5);
ylabel('\alpha [rad]'); legend('measured', 'simulated'); grid on;
xlabel('Time [s]');

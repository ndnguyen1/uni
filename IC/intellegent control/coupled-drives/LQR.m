clc; clear;

%% Step 1: Load data
load('Teensy41_test_data_01.mat')
Nini     = 1;
time     = logsout{1}.Values.Time(Nini:end-1);
time     = time - time(1);
Vm_left  = squeeze(logsout{1}.Values.Data(Nini:end-1));
Vm_right = squeeze(logsout{2}.Values.Data(Nini:end-1));
w_v      = squeeze(logsout{9}.Values.Data(Nini:end-1));
alpha_v  = squeeze(logsout{10}.Values.Data(Nini:end-1));

%% Step 2: System Identification — order-5 discrete-time model
Ts      = time(2) - time(1);
data_id = iddata([w_v, alpha_v], [Vm_left, Vm_right], Ts);
sys_n4  = n4sid(data_id, 5, 'Ts', Ts);
sys_est = ssest(data_id, sys_n4, 'Ts', Ts);
[Ad, Bd, C, D] = ssdata(sys_est);

fprintf('Discrete eigenvalues:\n');      disp(abs(eig(Ad)))
fprintf('Controllability rank: %d\n',    rank(ctrb(Ad, Bd)))
fprintf('Observability rank:   %d\n',    rank(obsv(Ad, C)))

%% Step 3: Model fit plot
figure;
compare(data_id, sys_est);
title('Model Fit');

%% Step 4: LQR Controller with output weighting
C_kf = C;

% Output weight — penalise w and alpha error directly in volt units
Q_y   = diag([1, 10]);          % weight on [w, alpha] outputs
Q_lqr = C_kf' * Q_y * C_kf;     % maps output penalty back to state space (5x5)
R_lqr = diag([1, 1]);

K = dlqr(Ad, Bd, Q_lqr, R_lqr);

fprintf('\n--- Closed-loop eigenvalues (LQR) ---\n');
disp(abs(eig(Ad - Bd*K)))

%% Step 5: Kalman Observer
Q_kf = diag([1e-3, 1e-3, 1e-2, 1e-2, 1e-2]);
R_kf = diag([1, 1]); 
P    = dare(Ad', C_kf', Q_kf, R_kf);
L    = P * C_kf' / (C_kf * P * C_kf' + R_kf);

fprintf('--- Observer eigenvalues ---\n');
disp(abs(eig(Ad - L*C_kf)))

%% Step 6: Reference and Steady-State Solution
w_ref     = 1.0;   % V
alpha_ref = 0.5;   % V
y_ref     = [w_ref; alpha_ref];

n  = size(Ad, 1);
nu = size(Bd, 2);
ny = size(C_kf, 1);

SS_mat = [(Ad - eye(n)), Bd;
           C_kf,         zeros(ny, nu)];
SS_rhs = [zeros(n, 1); y_ref];
xu_ss  = SS_mat \ SS_rhs;
x_ref  = xu_ss(1:n);
u_ss   = xu_ss(n+1:end);

%% Step 7: Closed-Loop Simulation
N_sim = 5000;
t_sim = (0:N_sim-1) * Ts;

x_true = zeros(5, 1);
x_hat  = zeros(5, 1);

x_true_log = zeros(5, N_sim);
x_hat_log  = zeros(5, N_sim);
u_log      = zeros(2, N_sim);
y_log      = zeros(2, N_sim);

for k = 1:N_sim
    y     = C_kf * x_true;
    x_hat = x_hat + L * (y - C_kf * x_hat);
    u     = u_ss - K * (x_hat - x_ref);

    x_true_log(:, k) = x_true;
    x_hat_log(:, k)  = x_hat;
    u_log(:, k)      = u;
    y_log(:, k)      = y;

    x_true = Ad * x_true + Bd * u;
    x_hat  = Ad * x_hat  + Bd * u;
end

%% Step 8: Plot Results
figure;
subplot(3,1,1);
plot(t_sim, y_log(1,:), 'b', t_sim, ones(1,N_sim)*w_ref, 'r--');
ylabel('w (V)'); title('Angular Velocity'); grid on;
legend('Simulated', 'Reference');

subplot(3,1,2);
plot(t_sim, y_log(2,:), 'b', t_sim, ones(1,N_sim)*alpha_ref, 'r--');
ylabel('\alpha (V)'); title('Jockey Angle'); grid on;
legend('Simulated', 'Reference');

subplot(3,1,3);
plot(t_sim, u_log(1,:), 'b', t_sim, u_log(2,:), 'r');
ylabel('Voltage (V)'); title('Control Effort'); grid on;
legend('u1 (left)', 'u2 (right)');
xlabel('Time (s)');
sgtitle('Closed-Loop State Feedback + Kalman Simulation');
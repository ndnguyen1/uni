%% Step 1: Outputting Data
load('Teensy41_test_data_01.mat')
Nini = 1;
time = logsout{1}.Values.Time(Nini:end-1);
time = time - time(1);

Vm_left  = squeeze(logsout{1}.Values.Data(Nini:end-1));  % DAC Output 1
Vm_right = squeeze(logsout{2}.Values.Data(Nini:end-1));  % DAC Output 2
w_v      = squeeze(logsout{9}.Values.Data(Nini:end-1));  % Input 1 (ADC) - angular velocity
alpha_v  = squeeze(logsout{10}.Values.Data(Nini:end-1)); % Input 2 (ADC) - jockey angle

% Convert ADC to voltage
Vm_left  = -0.0192 * Vm_left  + 9.6;
Vm_right = -0.0192 * Vm_right + 9.6;
w_v      = -0.0192 * w_v      + 9.6;
alpha_v  = -0.0192 * alpha_v  + 9.6;

%% Step 2: Fitting the output voltages to degrees/radians
alpha_deg = 0.7020 * alpha_v + (-0.0234);
alpha_rad = deg2rad(alpha_deg);

%% Step 3: Creating the State Space model
Ts = time(2) - time(1);
data_id = iddata([w_v, alpha_rad], [Vm_left, Vm_right], Ts);
n = 4;
sys_init = n4sid(data_id, n);
sys_est  = ssest(data_id, sys_init);

% Extract state-space matrices
[A, B, C, D] = ssdata(sys_est);

%% Step 4: Verifying Controllability/Observability
n = size(A, 1);
Co = ctrb(A, B);
Ob = obsv(A, C);

rank_Co = rank(Co);
rank_Ob = rank(Ob);

if rank_Co == n
    disp("System is controllable")
else
    disp("System is not controllable")
end
if rank_Ob == n
    disp("System is observable")
else
    disp("System is not observable")
end


%% Step 5: Kalman Filter
% Discretising
sys_c = ss(A, B, C, D);
sys_d = c2d(sys_c, Ts, 'zoh');
[Ad, Bd, Cd, Dd] = ssdata(sys_d);

% Use consistent C for Kalman (direct state outputs)
C_kf = [1 0 0 0;
        0 0 1 0];

% Noise covariances
Q_kf = diag([1e-4, 1e-2, 1e-4, 1e-2]);
R_kf = diag([1e-3, 1e-3]);

% Steady-state Kalman gain via discrete Riccati
P = dare(Ad', C_kf', Q_kf, R_kf);
L = (P * C_kf' / (C_kf * P * C_kf' + R_kf))';   % (4x2)


%% Step 6: Implementing the LQR
Q_lqr = diag([1, 0.1, 1, 0.1]);    % relax state weights significantly
R_lqr = diag([100, 100]);           % penalise input heavily
K = dlqr(Ad, Bd, Q_lqr, R_lqr);



%% Step 8: Closed-loop LQR+Kalman Simulation
N_sim = 1000;
t_sim = (0:N_sim-1) * Ts;

% Reference: w = 3V, alpha = 1.5 deg -> rad
w_ref     = 3;
alpha_ref = deg2rad(1.5);
x_ref     = [w_ref; 0; alpha_ref; 0];

% Initialise
x_true = zeros(4, 1);       % plant state
x_hat  = zeros(4, 1);       % kalman estimate
C_kf   = [1 0 0 0;
           0 0 1 0];

% Storage
x_true_log = zeros(4, N_sim);
x_hat_log  = zeros(4, N_sim);
u_log      = zeros(2, N_sim);
y_log      = zeros(2, N_sim);

for k = 1:N_sim
    % Measurement
    y = C_kf * x_true;

    % Kalman update
    x_hat = x_hat + L * (y - C_kf * x_hat);

    % LQR control
    e = x_ref - x_hat;
    u = K * e;

    % Log
    x_true_log(:, k) = x_true;
    x_hat_log(:, k)  = x_hat;
    u_log(:, k)      = u;
    y_log(:, k)      = y;

    % Propagate plant
    x_true = Ad * x_true + Bd * u;

    % Propagate Kalman
    x_hat = Ad * x_hat + Bd * u;
end

figure;

subplot(3,1,1);
plot(t_sim, y_log(1,:), 'b', t_sim, ones(1,N_sim)*w_ref, 'r--');
ylabel('w (V)'); title('Angular Velocity'); grid on;
legend('Simulated','Reference');

subplot(3,1,2);
plot(t_sim, rad2deg(y_log(2,:)), 'b', t_sim, ones(1,N_sim)*1.5, 'r--');
ylabel('\alpha (deg)'); title('Jockey Angle'); grid on;
legend('Simulated','Reference');

subplot(3,1,3);
plot(t_sim, u_log(1,:), 'b', t_sim, u_log(2,:), 'r');
ylabel('Voltage (V)'); title('Control Effort'); grid on;
legend('u1 (left)','u2 (right)');

xlabel('Time (s)');
sgtitle('Closed-Loop LQR+Kalman Simulation');
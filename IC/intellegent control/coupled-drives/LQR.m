
%% Step 1: Outputting Data

load('Teensy41_test_data_01.mat')
Nini = 1;
time = logsout{1}.Values.Time(Nini:end-1);
time = time - time(1);
Vm_left     = logsout{1}.Values.Data(Nini:end-1);  % Output 1: left motor
Vm_right    = logsout{2}.Values.Data(Nini:end-1);  % Output 2: right motor
w_v         = logsout{3}.Values.Data(Nini:end-1);  % Input 1: angular velocity (Pin A2)
alpha_v     = logsout{4}.Values.Data(Nini:end-1);  % Input 2: jockey angle (Pin A3)

% Fitting the output voltages to degrees/radians
alpha_deg = 0.7020 * alpha_v + (-0.0234);
alpha_rad = deg2rad(alpha_deg);

%% Step 2: Creating the State Space model 
Ts = time(2) - time(1);
data_id = iddata([w_v, alpha_rad], [Vm_left, Vm_right], Ts);

n = 4;
sys_init = n4sid(data_id, n); % Subspace method which always converges as an initial guess
sys_est = ssest(data_id, sys_init); % Iterative prediction error method for final model 

% Extract state-space matrices
[A, B, C, D] = ssdata(sys_est);



%% Step 3: Verifying Controllability/Observability
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

if rank_Ob== n
    disp("System is observable")
else 
    disp("System is not observable")
end 



%% Step 4: Estimating alpha_dot and omega_dot
C_kf = C;

% Discretizing 
Ad = expm(A * Ts);
Bd = (Ad - eye(size(A))) / A * B;
Cd = C_kf;

% Process noise covariance (Initial guesses)
Q_kf = diag([1e-4, 1e-2, 1e-4, 1e-2]);

% Measurement noise covariance (Initial guesses)
R_kf = diag([1e-3, 1e-3]);


sys_kf_d = ss(Ad, Bd, Cd, zeros(2, size(Bd,2)), Ts);
[~, L, ~] = kalman(sys_kf_d, Q_kf, R_kf);


%% Step 5: Implementing the LQR
Q_lqr = diag([100, 1, 10, 1]);
R_lqr = diag([1, 1]);
K = dlqr(Ad, Bd, Q_lqr, R_lqr);



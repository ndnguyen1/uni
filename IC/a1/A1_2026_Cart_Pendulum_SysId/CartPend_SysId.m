%     ┌──────┬─────────────────────────────────────────────────────────────────────────────────────────┐                          
%     │ Part │                                          Task                                           │                          
%     ├──────┼─────────────────────────────────────────────────────────────────────────────────────────┤                          
%     │ a)   │ Derive continuous-time state-space matrices Ac, Bc, Cc analytically using small-angle + │                          
%     │      │  slow angular speed approximations                                                      │                          
%     ├──────┼─────────────────────────────────────────────────────────────────────────────────────────┤                          
%     │ b)   │ Discretise using forward-Euler → get A, B, C analytically                               │                          
%     ├──────┼─────────────────────────────────────────────────────────────────────────────────────────┤                          
%     │ c)   │ Identify the unknown parameters θ, rearrange model into b_ℓ = A_ℓ θ form                │                          
%     ├──────┼─────────────────────────────────────────────────────────────────────────────────────────┤                          
%     │ d)   │ Lab work: apply pulse inputs (pos + neg), record xc and α response data                 │                          
%     ├──────┼─────────────────────────────────────────────────────────────────────────────────────────┤                          
%     │ e)   │ Design a steady-state Kalman smoother to estimate full state x̂(k) from output data.     │                          
%     │      │ Prove observability, explain Qf/Rf tuning                                               │                          
%     ├──────┼─────────────────────────────────────────────────────────────────────────────────────────┤                          
%     │ f)   │ Offline Least-Squares in MATLAB using smoothed state data → report identified θ̂         │                          
%     ├──────┼─────────────────────────────────────────────────────────────────────────────────────────┤                          
%     │ g)   │ Offline Recursive Least-Squares in MATLAB → compare θ̂(k) with f)                        │                          
%     ├──────┼─────────────────────────────────────────────────────────────────────────────────────────┤                          
%     │ h)   │ Challenge: Real-time RLS on control board                                               │                          
%     │ ⭐   │                                                                                         │                          
%     └──────┴─────────────────────────────────────────────────────────────────────────────────────────┘                          
% 
%     ---                                                                                                                         
%     Q2. LQR Control Design (40 marks)                                                                                           
% 
%     ┌──────┬─────────────────────────────────────────────────────────────────────────────────────────┐                          
%     │ Part │                                          Task                                           │                          
%     ├──────┼─────────────────────────────────────────────────────────────────────────────────────────┤                          
%     │ a)   │ Prove controllability, design LQR u(k) = -K(x(k) - x*) + u*, simulate in                │                          
%     │      │ MATLAB/Simulink with all states measured                                                │                          
%     ├──────┼─────────────────────────────────────────────────────────────────────────────────────────┤                          
%     │ b)   │ Prove observability, design Kalman filter for real-time state estimation, simulate      │                          
%     │      │ combined LQR + Kalman with only xc and α measured                                       │                          
%     ├──────┼─────────────────────────────────────────────────────────────────────────────────────────┤                          
%     │ c)   │ Challenge: Implement LQR + Kalman on control board                                      │                          
%     │ ⭐   │                                                                                         │                          
%     ├──────┼─────────────────────────────────────────────────────────────────────────────────────────┤                          
%     │ d)   │ Challenge: Implement RLS + LQR + Kalman on control board                                │                          
%     │ ⭐   │                                                                                         │                          
%     └──────┴─────────────────────────────────────────────────────────────────────────────────────────┘                          
% 
%     ---                                                                                                                         
%     Conclusions (10 marks)                                                                                                      
% 
%     Overall writeup on the system ID and optimal control design process.                                                        
% 
%     ---                                                                                                                         
%     Key workflow:                                                                                                               
%     1. Lab experiment (pulse response data)                                                                                     
%     2. Kalman smoother → full state estimates                                                                                   
%     3. Least-Squares → identify system parameters                                                                               
%     4. Use identified model → LQR + Kalman filter design                                                                        
%     5. Simulate in MATLAB, then deploy to hardware (challenges)                                              





%% Assignment 1 - Q1. System Identification
% 48580 Intelligent Control Studio
% System Identification of a Cart-Pendulum System
%         
% University of Technology Sydney, Australia
% Autumn 2026
%
% Coordinator: A/Prof Ricardo P. Aguilera
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clc;
clear;
format long
s = tf('s');

%% reading data
load('Teensy41_test_data_01.mat')
logsout

Nini = 1;
time = logsout{1}.Values.Time(Nini:end-1);
time = time - time(1);
Vm = logsout{1}.Values.Data(Nini:end-1);
alpha(:,1) = logsout{2}.Values.Data(Nini:end-1);
xc = logsout{3}.Values.Data(Nini:end-1);
xc = squeeze(xc);
alpha = squeeze(alpha);
alpha_rad = deg2rad(alpha);

%% sampling time from Teensy 4.1
ts = time(2);
N = length(time);

%% plots saved data
figure(101)
subplot(311)
plot(time,Vm,'LineWidth',2)
title('SysId Data')
ylim([-8,8])
yticks([-6 0 6])
grid
ylabel('$u(t)$~[V]','FontSize',20,'Interpreter','latex')

subplot(312)
plot(time,xc,'LineWidth',2)
grid
ylabel('$x_c(t)$~[m]','FontSize',20,'Interpreter','latex')

subplot(313)
plot(time,alpha,'LineWidth',2)
grid
ylabel('$\alpha (t)$~[deg]','FontSize',20,'Interpreter','latex')
xlabel('Time (s)','FontSize',20,'Interpreter','latex')

%% System Identification
%  Add your Least Square code here


% Part A: Derive continuous-time state-space matrices Ac, Bc, Cc analytically using small-angle + slow angular speed approximations
% Ac = [0, 0, 1, 0;
%      0, 0, 0, 1;
%      0, gm/M, -bc/M, bp/(M*l);     
%      0, -(M+m)*g/(M*l), bc/(M*l), -(M+m)*bp/(M*m*l^2)];
% 
% Bc = [0; 0; Kf/M; -Kf/(M*l)];
% 
% Cc = [1 0 0 0;
%       0 1 0 0];


% Part B: Discretise using forward-Euler → get A, B, C analytically
% Ad = eye(4) + Ac*ts;
% Bd = Bc * ts;
% Cd = Cc;

% Part C: Identify the unknown parameters θ, rearrange model into b_ℓ = A_ℓ θ form

% xc(k+1) = xc(k) + ts*vc(k)
% alpha(k+1) = alpha(k) + ts*omega(k)
% vc(k+1) = (gm/M)*ts*alpha(k) + (1 - bc*ts/M)*vc(k) + (bp*ts/(M*l))*omega(k) + (Kf*ts/M)*vm(k)
% omega(k+1) = -(M+m)*g/(M*l)*ts*alpha(k) + bc/(M*l)*ts*vc(k) + (1-(M+m)*bp/(M*m*l^2)*ts)*omega(k) - Kf/(M*l)*ts*vm(k)
% 
% xc(k+1) and alpha(k+1) are all kinematics and contain no unknowns so we
% dont need to aprox them in the LS calculation

% we then rearrange rows 3 and 4 to be in the form b_ℓ = A_ℓ θ where θ is
% the known and A_ℓ is all the unknowns

% theta = [gm/M; bc/M; bp/(M*l); Kf/M; (M+m)*g/(M*l); bc/(M*l); (M+m)*bp/(M*m*l^2); Kf/(M*l)];


% Part D: Lab work: apply pulse inputs (pos + neg), record xc and α response data                 

% Part E: Design a steady-state Kalman smoother to estimate full state
% x̂(k) from output data. Prove observability, explain Qf/Rf tuning

% because we don't have numerical values, we use a constant velocity model
% to estimate Ad before running the Kalman smoother. 

Ad_kin = [1, 0, ts, 0;
          0, 1, 0, ts;
          0, 0, 1, 0;
          0, 0, 0, 1];
Cd = [1 0 0 0;
      0 1 0 0];

% because the encoders are relatively clean, we have a high Qf/Rf to trust
% the sensors more
Qf = eye(4) * 1;
Rf = eye(2) * 0.001;

[M_kal, P, Z, E] = dlqe(Ad_kin, eye(4), Cd, Qf, Rf);


% running the forward Kalman filter pass
Bd_kin = zeros(4, 1);
x_hat = zeros(4, 1);
x_hat_stored = zeros(4, N);

for k = 1:N 
    x_pred_hat = Ad_kin * x_hat + Bd_kin * Vm(k);
    x_hat = x_pred_hat + M_kal * ([xc(k); alpha_rad(k)] - Cd * x_pred_hat);
    x_hat_stored(:,k) = x_hat;
end

% plotting the KF pass 
figure(102)
subplot(411)
plot(time, x_hat_stored(1,:), 'LineWidth', 2)
ylabel('xc estimate')
grid

subplot(412)
plot(time, x_hat_stored(2,:), 'LineWidth', 2)
ylabel('alpha_rad estimate')
grid

subplot(413)
plot(time, x_hat_stored(3,:), 'LineWidth', 2)
ylabel('vc estimate')
grid

subplot(414)
plot(time, x_hat_stored(4,:), 'LineWidth', 2)
ylabel('omega estimate')
grid
xlabel('Time (s)')

% running the backward Kalman smoother
G_s = P * Ad_kin' * inv(Ad_kin * P * Ad_kin' + Qf);
x_hat_smooth = zeros(4, N);
x_hat_smooth(:, N) = x_hat_stored(:, N);  % initialise at last step

for k = N-1:-1:1
    x_hat_smooth(:,k) = x_hat_stored(:,k) + G_s * (x_hat_smooth(:,k+1) - Ad_kin * x_hat_stored(:,k));
end

% plotting the double smoothed data
figure(103)
subplot(411)
plot(time, x_hat_smooth(1,:), 'LineWidth', 2)
ylabel('xc smoothed')
grid

subplot(412)
plot(time, x_hat_smooth(2,:), 'LineWidth', 2)
ylabel('alpha_rad smoothed')
grid

subplot(413)
plot(time, x_hat_smooth(3,:), 'LineWidth', 2)
ylabel('vc smoothed')
grid

subplot(414)
plot(time, x_hat_smooth(4,:), 'LineWidth', 2)
ylabel('omega smoothed')
grid
xlabel('Time (s)')

% separating out all the smoothed data
xc_s = x_hat_smooth(1, :);
alpha_rad_s = x_hat_smooth(2, :);
vc_s = x_hat_smooth(3, :);
omega_s = x_hat_smooth(4, :);

% calculating theta at each step
A_l = zeros(2*(N-1), 8);
b_l = zeros(2*(N-1), 1);

for k = 1:N-1
    row = 2*k - 1;
    A_l(row,   :) = ts * [alpha_rad_s(k), -vc_s(k), omega_s(k),  Vm(k),       0,          0,           0,        0   ];
    A_l(row+1, :) = ts * [     0,          0,        0,         0,    -alpha_rad_s(k), vc_s(k),  -omega_s(k), -Vm(k)  ];
    b_l(row)   = vc_s(k+1)    - vc_s(k);
    b_l(row+1) = omega_s(k+1) - omega_s(k);
end

% Condition number before solving
sv = svd(A_l);
fprintf('A_l condition number: %.2e\n', sv(1)/sv(end));
fprintf('A_l rank:             %d / 8\n', rank(A_l));
fprintf('omega_s vs vc_s corr: %.4f  (near 1 => ill-conditioned)\n', ...
    corr(omega_s', vc_s'));

% Singular value plot — gaps show unidentifiable directions
figure(105)
semilogy(sv, 'o-', 'LineWidth', 2)
title('Singular values of A\_l (large gap = rank deficiency)')
xlabel('Index'); ylabel('Singular value'); grid

% Offline LS (plain backslash — may be ill-conditioned)
theta_hat_ls = A_l \ b_l;

% Ridge regression — stabilises the ill-conditioned system.
% Rule of thumb: lambda ~ (noise std)^2 * N.  Tune by checking physical signs.
% Start at 1e-3 and decrease until theta(1),theta(4),theta(5),theta(8) are all
% positive.  If cond(A_l) >> 1e6 you may need lambda up to 1e0.
lambda = 1e-3;
theta_hat = (A_l'*A_l + lambda*eye(8)) \ (A_l'*b_l);

fprintf('\nPhysical checks (all four should be > 0):\n');
fprintf('  theta(1) gm/M      = %8.4f  (plain LS: %8.4f)\n', theta_hat(1),  theta_hat_ls(1));
fprintf('  theta(4) Kf/M      = %8.4f  (plain LS: %8.4f)\n', theta_hat(4),  theta_hat_ls(4));
fprintf('  theta(5) (M+m)g/Ml = %8.4f  (plain LS: %8.4f)\n', theta_hat(5),  theta_hat_ls(5));
fprintf('  theta(8) Kf/(Ml)   = %8.4f  (plain LS: %8.4f)\n', theta_hat(8),  theta_hat_ls(8));

disp('Part F - Offline LS (ridge) identified parameters:')
disp(theta_hat)

% Part G: Offline Recursive Least Squares
% At each time step k we get 2 new measurement rows (vc and omega equations).
% Standard RLS update with a 2-row observation at each step.

theta_rls = zeros(8, 1);
P_rls = eye(8) * 1e6;   % large initial covariance = uninformed prior
theta_rls_stored = zeros(8, N-1);

for k = 1:N-1
    Phi_k = ts * [alpha_rad_s(k), -vc_s(k),  omega_s(k),  Vm(k),           0,          0,           0,        0;
                       0,              0,          0,         0,   -alpha_rad_s(k), vc_s(k), -omega_s(k), -Vm(k)];
    y_k = [vc_s(k+1) - vc_s(k); omega_s(k+1) - omega_s(k)];

    K_rls = P_rls * Phi_k' / (eye(2) + Phi_k * P_rls * Phi_k');
    theta_rls = theta_rls + K_rls * (y_k - Phi_k * theta_rls);
    P_rls = (eye(8) - K_rls * Phi_k) * P_rls;

    theta_rls_stored(:, k) = theta_rls;
end

disp('Part G - RLS final identified parameters:')
disp(theta_rls)

% plot RLS convergence vs offline LS result
theta_labels = {'\theta_1: gm/M', '\theta_2: bc/M', '\theta_3: bp/(Ml)', '\theta_4: Kf/M', ...
                '\theta_5: (M+m)g/(Ml)', '\theta_6: bc/(Ml)', '\theta_7: (M+m)bp/(Mml^2)', '\theta_8: Kf/(Ml)'};

figure(104)
for i = 1:8
    subplot(4, 2, i)
    plot(time(1:N-1), theta_rls_stored(i,:), 'LineWidth', 1.5)
    hold on
    yline(theta_hat(i), 'r--', 'LineWidth', 1.5)
    hold off
    ylabel(theta_labels{i}, 'Interpreter', 'tex')
    grid
    if i >= 7
        xlabel('Time (s)')
    end
end
sgtitle('Part G: RLS convergence (blue) vs Offline LS (red dashed)')


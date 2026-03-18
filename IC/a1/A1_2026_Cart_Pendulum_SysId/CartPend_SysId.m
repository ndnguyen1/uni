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

for k = 1:N t 
    x_pred_hat = Ad_kin * x_hat + Bd_kin * Vm(k);
    x_hat = x_pred_hat + M_kal * ([xc(k); alpha(k)] - Cd * x_pred_hat);
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
ylabel('alpha estimate')
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
ylabel('alpha smoothed')
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
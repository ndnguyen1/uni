%%  Least Square Model Identification of a DC Motor
%   Second Order State-Space Model Identification
%   
%   48580 - Intelligent Control Studio
%   UTS, Australia
%   A/Prof Ricardo P. Aguilera
%
%   March, 2025
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clc;
clear;
%% Pending
% RLS
% RLS + KF
% experiments
format short

%% Settings
Tsim = 20;
Vpeak = 3;
fs = 200;
ts = 1/fs;

% noises gain
Ng_theta = 0.05;
Ng_w =  0.1;
Ng_i = 0.1;

%% Simulation
disp('Simulating DC Motor...')
DC_Motor_full_order_model;

theta_deg = theta*180/pi;

%% Plots 
disp('Plotting SysId data...')

f = figure(11);
f.Position = [10 70 600 700];
plot_font = 18;
subplot(411)
plot(time,u,'LineWidth',1.5)
grid
ylabel('$v_m(t)~(v)$','Interpreter','latex','FontSize',plot_font)
subplot(412)
plot(time,theta_deg)
ylabel('$\theta(t)~(^o)$','Interpreter','latex','FontSize',plot_font)
grid
subplot(413)
plot(time,w)
ylabel('$\omega(t)~(rad/s)$','Interpreter','latex','FontSize',plot_font)
grid
subplot(414)
plot(time,i_m)
ylabel('$\i(t)~(A)$','Interpreter','latex','FontSize',plot_font)
grid
xlabel('t (s)','Interpreter','latex','FontSize',plot_font)

%% 1. Reduced-Order Discrete-time Model
% Continuous-time Model
disp(' ')
disp('1. Reduced-Order (RO) Model...')
Kv = K_m/r;
b_eq =b_T+K_m^2/r;

Ac = [0         1;
      0     -b_eq/J_T  ];

Bc = [0;
      Kv/J_T];

Cc = eye(2);

ct_RO_sys_true = ss(Ac,Bc,Cc,0);

% parameter to be identified
a22 = Ac(2,2);
b2 = Bc(2);

%% Reduced-Order Discrete-time Model
dt_RO_sys_RO = c2d(ct_RO_sys_true,ts);
A_RO = dt_RO_sys_RO.A;
B_RO = dt_RO_sys_RO.B;

theta_1 = 1+ts*a22;
theta_2 = ts*b2;


%% 1.1 Verification 
A_verif = A_RO;
B_verif = B_RO;

disp('1.1 Verification RO model...')
sim("sim_DC_Motor_ROM_verification.slx");

MSE_ROM_theta = immse(theta,theta_verif);
MSE_ROM_w = immse(w,w_verif);

%% 1.2 Plot verification
disp('1.2 Plotting...')
theta_verif_deg = theta_verif*180/pi;
data = [u,theta_deg,w];
data_verif = [u_verif,theta_verif_deg,w_verif];

f = figure(12);
f.Position = [10 70 600 700];
plot_ROM_comparison(data,data_verif,ts)
sgt = sgtitle('Reduced-Order Model Response');
sgt.FontSize = 20;

%% 2. Matlab's SysId
disp(' ')
disp('2. Matlab''s SysId')
sys_id_data = iddata([theta,w],u,ts);
dt_sys_est = ssest(sys_id_data,2,'Ts',ts,'Form','canonical');
A_matlab= dt_sys_est.A
B_matlab= dt_sys_est.B
C_matlab= dt_sys_est.C

%% 2.1 Verification
A_verif = A_matlab;
B_verif = B_matlab;
disp('2.1 Matlab SysId Verification...')
sim("sim_DC_Motor_ROM_verification.slx");
MSE_matlab_theta = immse(theta,theta_verif);
MSE_matlab_w = immse(w,w_verif);

%% 2.2 Plots 
disp('2.2 Plotting...')
theta_verif_deg = theta_verif*180/pi;
data_verif = [u_verif,theta_verif_deg,w_verif];

f = figure(13);
f.Position = [610 70 600 700];
plot_ROM_comparison(data,data_verif,ts)
sgt = sgtitle('Matlab SysId Model Response');
sgt.FontSize = 20;

%% 3 Ricardo's SysId
disp(' ')
disp('3. Ricardo''s SysId...')
x = [theta'; 
     w'];
N=length(time); %length of the recorded data

A_sid = [x(2,1) u(1)];
b_sid = x(2,2);
for k = 2:N-1
    Ai = [x(2,k) u(k)];
    A_sid = [A_sid; Ai];
    
    bi = x(2,k+1);
    b_sid= [b_sid; bi]; 
end
b_sid;
A_sid;

theta_sid = inv(A_sid'*A_sid)*A_sid'*b_sid;

theta_1_hat = theta_sid(1);
theta_2_hat = theta_sid(2);

A_hat = [1 ts;
        0 theta_1_hat]
B_hat = [0;
         theta_2_hat]

%% 3.1 Verification 
A_verif = A_hat;
B_verif = B_hat;
disp('3.1 SysId Verification...')
sim("sim_DC_Motor_ROM_verification.slx");

MSE_custom_LS_theta = immse(theta,theta_verif);
MSE_custom_LS_w = immse(w,w_verif);

%% 3.2 Plots 
disp('3.2 Plotting...')
theta_verif_deg = theta_verif*180/pi;
data = [u,theta_deg,w];
data_verif = [u_verif,theta_verif_deg,w_verif];

f = figure(14);
f.Position = [1210 70 600 700];
plot_ROM_comparison(data,data_verif,ts)
sgt = sgtitle('LS-Based SysId Model Response');
sgt.FontSize = 20;

%% 4. Final comparison
disp(' ')
disp('*******************************************')
disp('4. Comparison')
disp(' ')
disp(['RO Model        - MSE for theta: ',num2str(MSE_ROM_theta)])
disp(['Matlab SysId    - MSE for theta: ',num2str(MSE_matlab_theta)])
disp(['Custom LS SysId - MSE for theta: ',num2str(MSE_custom_LS_theta)])
disp(' ')
disp(['RO Model        - MSE for w: ',num2str(MSE_ROM_w)])
disp(['Matlab SysId    - MSE for w: ',num2str(MSE_matlab_w)])
disp(['Custom LS SysId - MSE for w: ',num2str(MSE_custom_LS_w)])
disp(' ')

disp('Done!!!')
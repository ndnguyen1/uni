%%  Recursive Least Square Model Identification of a DC Motor
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

%% Settings
Tsim = 20;
Vpeak = 3;
fs = 200;
ts = 1/fs;

P = 1*eye(2);
R = 0.001;

% noises gain
Ng_theta = 0.05;
Ng_w =  0.01;
Ng_i = 0.1;

%% Simulation
disp('Simulating DC Motor...')
DC_Motor_full_order_model;

theta_deg = theta*180/pi;

%% Plots 
disp('Plotting measured data...')

f = figure(11);
f.Position = [100 100 800 700];
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


%% Assuming a reduced-order model
%% Continuous-time Model
disp('Continuous-time Model')
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

%% Discrete-time Model
dt_RO_sys_true = c2d(ct_RO_sys_true,ts);
A_true = dt_RO_sys_true.A;
B_true = dt_RO_sys_true.B;

theta_1 = 1+ts*a22;
theta_2 = ts*b2;

%% RLS SysId
disp(' ')
disp('Recursive Least Square SysId...')
x = [theta'; 
     w'];
N=length(time); %length of the recorded data


x_prev = [0 0]';
u_prev = 0;
theta_hat_prev = [0 0 ]';
for k = 1:1:N 
    Ai = [ x_prev(2) u_prev];
    bi = x(2,k);

    K = P*Ai'*inv(Ai*P*Ai' + R);
    theta_hat(:,k) = theta_hat_prev+ K*(bi - Ai*theta_hat_prev);
    P = (eye(2) - K*Ai)*P;

    x_prev = x(:,k);
    u_prev = u(k);
    theta_hat_prev = theta_hat(:,k) ;
end

theta_1_hat = theta_hat(1,end);
theta_2_hat = theta_hat(2,end);

a22_hat = fs*(theta_1_hat-1);
b2_hat = fs*theta_2_hat;

A_hat = [1 ts;
        0 theta_1_hat]
B_hat = [0;
         theta_2_hat]

%% Verification
A_verif = A_hat;
B_verif = B_hat;
disp('SysId Verification...')
sim("sim_DC_Motor_ROM_verification.slx");

MSE_custom_LS_theta = immse(theta,theta_verif);
MSE_custom_LS_w = immse(w,w_verif);

%% Plots 
disp('Plotting...')
theta_verif_deg = theta_verif*180/pi;

f = figure(15);
f.Position = [610 70 600 700];
plot_font = 18;
subplot(311)
plot(time,u_verif,'LineWidth',1.5)
grid
title('RLS-Based SysId','FontSize',plot_font)
ylabel('$v_m(t)~(v)$','Interpreter','latex','FontSize',plot_font)
subplot(312)
plot(time,theta_deg,'b',time,theta_verif_deg,'r')
ylabel('$\theta(t)~(^o)$','Interpreter','latex','FontSize',plot_font)
grid
subplot(313)
plot(time,w,'b',time,w_verif,'r')
ylabel('$\omega(t)~(rad/s)$','Interpreter','latex','FontSize',plot_font)
grid
xlabel('t (s)','Interpreter','latex','FontSize',plot_font)

N_th = length(theta_hat);
figure(16)
subplot(211)
plot(time(1:N_th),theta_hat(1,:),[0 Tsim ],[theta_1 theta_1],'--k')
subplot(212)
plot(time(1:N_th),theta_hat(2,:),[0 Tsim ],[theta_2 theta_2],'--k')




disp(' ')
disp('*******************************************')
disp('Results')
disp(' ')
disp(['Custom RLS SysId - MSE for theta: ',num2str(MSE_custom_LS_theta)])
disp(['Custom RLS SysId - MSE for w: ',num2str(MSE_custom_LS_w)])
disp(' ')

disp('Done!!!')




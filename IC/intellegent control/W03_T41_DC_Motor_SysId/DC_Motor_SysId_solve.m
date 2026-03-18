%% W03 - System Identification Model for a DC-Motor
%         
% 48580 Intelligent Control Studio
% University of Technology Sydney, Australia
% Spring 2026
%
% Coordinator: A/Prof Ricardo P. Aguilera
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clc;
clear;
format long
s = tf('s');

%% reading data
load('Teensy41_test_data_03.mat')
Nini = 1;
time = logsout{1}.Values.Time(Nini:end-1);
time = time - time(1);
u = logsout{1}.Values.Data(Nini:end-1);
alpha = logsout{2}.Values.Data(Nini:end-1);
w = logsout{4}.Values.Data(Nini:end-1);

alpha_deg = alpha*180/pi;

%% sampling time from Teensy 4.1
ts = time(2);

%% plots saved data
figure(101)
subplot(311)
plot(time,u)
grid
subplot(312)
plot(time,alpha)
grid
subplot(313)
plot(time,w)
grid

%% System Identification Guw(s)
% add your code  here


%% 3 Ricardo's SysId
disp(' ')
disp('3. Ricardo''s SysId...')
x = [alpha'; 
     w'];
N=length(time); %length of the recorded data

A_sid = [x(2,1) u(1)];
b_sid = x(2,2);
for k = 2:N-1
    %Ai = [x(2,k) u(k)];
    Ai = [w(k) u(k)];
    A_sid = [A_sid; Ai];
    
    %bi = x(2,k+1);
    bi = w(k+1);
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

Tsim = time(end)
u_test = timeseries(u,time);

sim('sim_DC_Motor_ROM_verification.slx')

disp('3.2 Plotting...')
alpha_verif_deg = alpha_verif*180/pi;


figure(67)
subplot(311)
plot(time,u)
grid
subplot(312)
plot(time,alpha_deg,time,alpha_verif_deg)
grid
subplot(313)
plot(time,w,time,w_verif)
grid
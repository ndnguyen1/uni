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
load('Teensy41_test_data_01.mat')
Nini = 1;
time = logsout{1}.Values.Time(Nini:end-1);
time = time - time(1);
Vm = logsout{1}.Values.Data(Nini:end-1);
alpha = logsout{4}.Values.Data(Nini:end-1);
w = logsout{5}.Values.Data(Nini:end-1);

%% sampling time from Teensy 4.1
ts = time(2);

%% plots saved data
figure(101)
subplot(311)
plot(time,Vm)
grid
subplot(312)
plot(time,alpha)
grid
subplot(313)
plot(time,w)
grid

%% System Identification Guw(s)
% add your code  here


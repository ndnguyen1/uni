%% 41277 Control Design
%  Teensy 4.1 initialization
%  This script will run before compiling the Simulink Model. 

%% Controller Sampling Time
ts = 1/400;     % 400Hz sampling frequency

%% First Order Low Pass Filter Design for motor angular speed
s = tf('s');
fc_lp = 100;         %cutoff frequency in Hz
wc = 2*pi*fc_lp/2;  %cutoff frequency in rad/s

% Continuous time transfer function
Hs = wc / (s + wc);

% Discrete time transfer function
Hz = c2d(Hs, ts, 'zoh');
Hz_num = Hz.numerator{1};   %extract numerator polynomial
Hz_den = Hz.denominator{1}; %extract denominator polynomial



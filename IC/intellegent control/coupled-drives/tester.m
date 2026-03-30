clc; clear;

load('Teensy41_test_data_01.mat')
Nini = 1;
time     = logsout{1}.Values.Time(Nini:end-1);
time     = time - time(1);

% Raw values are already in volts - no conversion needed
Vm_left  = squeeze(logsout{1}.Values.Data(Nini:end-1));
Vm_right = squeeze(logsout{2}.Values.Data(Nini:end-1));
w_v      = squeeze(logsout{10}.Values.Data(Nini:end-1));
alpha_v  = squeeze(logsout{9}.Values.Data(Nini:end-1));

fprintf('Vm_left:  [%.3f, %.3f] V\n', min(Vm_left),  max(Vm_left));
fprintf('Vm_right: [%.3f, %.3f] V\n', min(Vm_right), max(Vm_right));
fprintf('w_v:      [%.3f, %.3f] V\n', min(w_v),      max(w_v));
fprintf('alpha_v:  [%.3f, %.3f] V\n', min(alpha_v),  max(alpha_v));

% Plot to verify
figure;
subplot(2,1,1);
plot(time, Vm_left, 'b', time, Vm_right, 'r');
ylabel('V'); title('Inputs (DAC)');
legend('Vm\_left','Vm\_right'); grid on;

subplot(2,1,2);
plot(time, w_v, 'b', time, alpha_v, 'r');
ylabel('V'); title('Outputs (ADC)');
legend('w\_v','alpha\_v'); grid on;
xlabel('Time (s)');
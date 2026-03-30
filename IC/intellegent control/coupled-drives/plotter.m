clc; clear;
load('Teensy41_test_data_04.mat')
sig = @(s) deal(s.Values.Time, squeeze(s.Values.Data));
[t1,  v1] = sig(logsout{1});  % Vout1
[t2,  v2] = sig(logsout{2});  % Vout2
[ti1, i1] = sig(logsout{9});  % alpha
[ti2, i2] = sig(logsout{10}); % omega

figure;
subplot(2,1,1);
plot(t1,v1, t2,v2);
legend('Vout1','Vout2');
ylabel('Voltage (V)'); title('DAC Outputs'); grid on;

subplot(2,1,2);
plot(ti1,i1, ti2,i2);
legend('alpha','omega');
ylabel('Voltage (V)'); title('Inputs'); grid on;
xlabel('Time (s)');
sgtitle('Teensy41 Test Data 04');
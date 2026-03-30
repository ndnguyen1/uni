
sig = @(s) deal(s.Values.Time, squeeze(s.Values.Data));

[t1,  v1] = sig(logsout{1});  % Vout1
[t2,  v2] = sig(logsout{2});  % Vout2
[ti1, i1] = sig(logsout{9});  % Input1
[ti2, i2] = sig(logsout{10}); % Input2

figure;

subplot(2,1,1);
plot(t1,v1, t2,v2);
legend('Vout1','Vout2');
ylabel('Voltage (V)'); title('DAC Outputs'); grid on;

subplot(2,1,2);
plot(ti1,i1, ti2,i2);
legend('Input1','Input2');
ylabel('Voltage (V)'); title('Inputs'); grid on;

xlabel('Time (s)');
sgtitle('Teensy41 Test Data 01');



%% Step 7: Plant Verification (Open Loop Replay)
N = length(time);
x = zeros(4, N);        % state trajectory
y_hat = zeros(2, N);    % estimated output

u = [Vm_left, Vm_right];

for k = 1:N-1
    x(:, k+1) = Ad * x(:, k) + Bd * u(k, :)';
    y_hat(:, k) = Cd * x(:, k);
end
y_hat(:, N) = Cd * x(:, N);

% Measured outputs
w_measured     = w_v;
alpha_measured = alpha_rad;

figure;
subplot(2,1,1);
plot(time, w_measured, 'b', time, y_hat(1,:), 'r--');
legend('Measured','Model');
ylabel('Angular velocity (V)'); title('w\_v'); grid on;

subplot(2,1,2);
plot(time, alpha_measured, 'b', time, y_hat(2,:), 'r--');
legend('Measured','Model');
ylabel('Angle (rad)'); title('\alpha'); grid on;

xlabel('Time (s)');
sgtitle('Open Loop Plant Verification');
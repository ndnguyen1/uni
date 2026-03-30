%% Step 1: Load and trim
load('Teensy41_test_data_01.mat')

% Find where inputs first become nonzero and trim from there
Nini = find(abs(logsout{1}.Values.Data) > 0.5, 1, 'first');
time     = logsout{1}.Values.Time(Nini:end-1);
time     = time - time(1);
Vm_left  = squeeze(logsout{1}.Values.Data(Nini:end-1));
Vm_right = squeeze(logsout{2}.Values.Data(Nini:end-1));
w_v      = squeeze(logsout{10}.Values.Data(Nini:end-1));
alpha_v  = squeeze(logsout{9}.Values.Data(Nini:end-1));

%% Step 2: Convert ADC/DAC counts to volts
Vm_left  = -0.0192 * Vm_left  + 9.6;
Vm_right = -0.0192 * Vm_right + 9.6;
w_v      = -0.0192 * w_v      + 9.6;
alpha_v  = -0.0192 * alpha_v  + 9.6;

%% Step 3: Convert alpha to radians
alpha_deg = 0.7020 * alpha_v + (-0.0234);
alpha_rad = deg2rad(alpha_deg);

%% Step 4: Remove mean (iddata works better on zero-mean data)
Vm_left  = Vm_left  - mean(Vm_left);
Vm_right = Vm_right - mean(Vm_right);
w_v      = w_v      - mean(w_v);
alpha_rad = alpha_rad - mean(alpha_rad);

%% Step 5: Build iddata and identify
Ts = time(2) - time(1);
data_id = iddata([w_v, alpha_rad], [Vm_left, Vm_right], Ts);

% Try orders 2 and 4
sys2 = n4sid(data_id, 2, 'Ts', Ts);
sys4 = n4sid(data_id, 4, 'Ts', Ts);

disp('--- Order 2 eigenvalues ---')
disp(abs(eig(sys2.A)))

disp('--- Order 4 eigenvalues ---')
disp(abs(eig(sys4.A)))

%% Step 6: Compare fit
figure;
compare(data_id, sys2, sys4);
legend('Measured', 'Order 2', 'Order 4');
title('Model Fit Comparison');
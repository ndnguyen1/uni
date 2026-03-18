%% System parameters
r = 1;
l = 0.1;
J_T = 0.001;
b_T = 0.06;
K_m = 0.1;

%% Full order continuous-time state-space model
Ac  = [ 0 1 0
        0 -b_T/J_T K_m/J_T;
       0 -K_m/l -r/l];

Bc = [0 0 1/l]';

sim('sim_DC_Motor_full_order_model.slx')
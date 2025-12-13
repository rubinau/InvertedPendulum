%% 1. System Parameters
Ts = 0.005;      % Sampling time (5ms)
M  = 0.139;      % Mass of Cart (kg)
m  = 0.0315;     % Mass of Pendulum (kg)
b  = 0.1;        % Friction coefficient
L  = 0.3;        % Full length of pendulum (m)
g  = 9.8;        % Gravity

% Derived Inertia and Length
I = 1/3 * m * L^2; 
l = L/2;           % Center of mass distance
q = (M+m)*(I+m*l^2)-(m*l)^2;
p = I*(M+m)+M*m*l^2; % Denominator helper

%% 2. Continuous State Space Matrices (Linearized)
% State vector: [x, x_dot, phi, phi_dot]
A = [0      1              0            0;
     0 -(I+m*l^2)*b/p  (m^2*g*l^2)/p    0;
     0      0              0            1;
     0 -(m*l*b)/p      m*g*l*(M+m)/p    0];

B = [     0;
     (I+m*l^2)/p;
          0;
        m*l/p];

C = [1 0 0 0;
     0 0 1 0];

D = [0;
     0];

%% 3. Discretize System (for Digital Control)
sys_ss = ss(A,B,C,D);
sysd   = c2d(sys_ss, Ts, 'zoh');

%% 4. LQR Control Design
% Q Matrix: Tuning weights
% Q = diag([x, x_dot, phi, phi_dot])
Q = diag([10, 50, 200, 25]);   

% R Matrix: Control effort penalty
R = 1;                         

% Calculate Gain K
[K_lqr, S, e] = dlqr(sysd.A, sysd.B, Q, R);

%% 5. Output

% Print K Matrix formatted for C++
fprintf('float K[4] = { %.5ff, %.5ff, %.5ff, %.5ff };\n', ...
    K_lqr(1), K_lqr(2), K_lqr(3), K_lqr(4));

fprintf('Ts = %.3f\n', Ts);
%% 1. System Parameters
clear; clc;

Ts = 0.005;  % Sample time (200 Hz)

% Physical Constants (SI Units)
M = 0.139;   % Mass of Cart (kg)
m = 0.0315;  % Mass of Pendulum (kg)
b = 0.1;     % Coefficient of Friction (N/m/s)
L = 0.3;     % Length to tip (m)
l = L/2;     % Length to Center of Mass (m)
g = 9.81;

% Inertia Calculation (Center of Mass)
I = (1/12) * m * L^2; 

%% 2. State Space Construction
p = I*(M+m) + M*m*l^2; 

A = [0      1              0           0;
     0 -(I+m*l^2)*b/p  (m^2*g*l^2)/p   0;
     0      0              0           1;
     0 -(m*l*b)/p       m*g*l*(M+m)/p  0];

B = [     0;
     (I+m*l^2)/p;
          0;
        m*l/p];

C = [1 0 0 0;  % Measuring Position (x)
     0 0 1 0]; % Measuring Angle (phi)

D = [0; 0];

sys_ss = ss(A,B,C,D);
sysd = c2d(sys_ss, Ts, 'zoh');
Ad = sysd.A;
Bd = sysd.B;

%% 3. LQR Controller Design
% Q = diag([x, x_dot, phi, phi_dot])
Q = diag([300, 1, 1200, 1]); 
R = 0.1; 
[K, ~] = dlqr(Ad, Bd, Q, R);

%% 4. Observer Design
desired_poles_c = [-30 -32 -34 -36]; 
desired_poles_d = exp(desired_poles_c * Ts); 
L_obs = place(Ad', C', desired_poles_d)';

%% 5. GENERATE C-CODE OUTPUT (FORMATTED FOR YOU)
fprintf('\n\n/* --- COPY BELOW THIS LINE --- */\n\n');

% --- K Matrix ---
fprintf('float K[4] = { ');
for i = 1:4
    fprintf('%.5ff', K(i)); % added 'f' suffix for float literal
    if i < 4, fprintf(', '); end
end
fprintf(' };\n\n');

% --- L Matrix (4x2) ---
fprintf('float L[4][2] = {\n');
for i = 1:4
    fprintf('  { %.5ff, %.5ff }', L_obs(i,1), L_obs(i,2));
    if i < 4, fprintf(',\n'); else fprintf('\n'); end
end
fprintf('};\n\n');

% --- A Matrix (4x4) ---
fprintf('float A_mat[4][4] = {\n');
for i = 1:4
    fprintf('  { ');
    for j = 1:4
        fprintf('%.5ff', Ad(i,j));
        if j < 4, fprintf(', '); end
    end
    fprintf(' }');
    if i < 4, fprintf(',\n'); else fprintf('\n'); end
end
fprintf('};\n\n');

% --- B Matrix (4) ---
fprintf('float B_mat[4] = { ');
for i = 1:4
    fprintf('%.5ff', Bd(i));
    if i < 4, fprintf(', '); end
end
fprintf(' };\n');
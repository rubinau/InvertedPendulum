Ts = 0.005;
M = 0.139;
m = 0.0315;
b = 0.1;
L = 0.3;
I = 1/3 * m * L^2;
%I = 0.000945;
g = 9.8;
l = L/2;
q = (M+m)*(I+m*l^2)-(m*l)^2;
s = tf('s');

P_cart = (((I+m*l^2)/q)*s^2 - (m*g*l/q))/(s^4 + (b*(I + m*l^2))*s^3/q - ((M + m)*m*g*l)*s^2/q - b*m*g*l*s/q);

P_pend = (m*l*s/q)/(s^3 + (b*(I + m*l^2))*s^2/q - ((M + m)*m*g*l)*s/q - b*m*g*l/q);

sys_tf = [P_cart ; P_pend];

p = I*(M+m)+M*m*l^2; %denominator for the A and B matrices

A = [0      1              0           0;
     0 -(I+m*l^2)*b/p  (m^2*g*l^2)/p   0;
     0      0              0           1;
     0 -(m*l*b)/p       m*g*l*(M+m)/p  0];
B = [     0;
     (I+m*l^2)/p;
          0;
        m*l/p];
C = [1 0 0 0;
     0 0 1 0];
D = [0;
     0];

states = {'x' 'x_dot' 'phi' 'phi_dot'};


inputs = {'u'};
outputs = {'x'; 'phi'};

sys_ss = ss(A,B,C,D,'statename',states,'inputname',inputs,'outputname',outputs)
sysd = c2d(sys_ss, Ts, 'zoh')

%Search Gain matrix
Q = diag([10, 50, 200, 25]);   % penalize x heavily to reduce cart motion
R = 1;                       % penalize control effort
K_lqr = dlqr(sysd.A, sysd.B, Q, R)   % returns K such that u = -K*x

%Observer 3x
desired_p = [-1 -2 -4 -6];
po = 3*desired_p;                 % 3x lebih cepat
zo = exp(po*Ts);

L = place(sysd.A', sysd.C', zo)'  % transpose trick

Acl = [sysd.A - sysd.B*K_lqr        sysd.B*K_lqr ;
       zeros(size(sysd.A))      sysd.A - L*sysd.C];

Bcl = [sysd.B ;
       zeros(size(sysd.B))];

Ccl = [sysd.C    zeros(size(sysd.C))];
Dcl = sysd.D;

sys_cl = ss(Acl, Bcl, Ccl, Dcl, Ts);


% --- initial condition (5 deg tip) ---
phi0 = deg2rad(5);
XINIT = [0; 0; phi0; 0;    % plant states
         0; 0; 0; 0];      % observer states

% quick sanity check
nstates = size(sys_cl.A,1);
if length(XINIT) ~= nstates
    error('Length of XINIT (%d) must equal number of states (%d).', length(XINIT), nstates);
end

% simulate initial condition response and capture outputs
[y,t] = initial(sys_cl, XINIT);   % y: outputs, t: time vector

% plot (x in meters, phi in degrees)
figure;
subplot(2,1,1);
plot(t, y(:,1));
ylabel('x (units)');
grid on;

subplot(2,1,2);
plot(t, rad2deg(y(:,2)));        % convert radians->degrees for plotting
ylabel('\phi (deg)');
xlabel('Time (s)');
grid on;
sgtitle('Initial Condition Test (5 deg tip)');

% compute peaks and print
max_x   = max(abs(y(:,1)));
max_phi = max(abs(y(:,2)));

fprintf('max |x| = %.4g (units)\n', max_x);
fprintf('max |phi| = %.4g (radians)\n', max_phi);
fprintf('max |phi| = %.2f degrees\n', rad2deg(max_phi));

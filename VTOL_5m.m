% Define constants
m = 0.5;    % mass of the quadcopter (kg)
g = 9.81;   % acceleration due to gravity (m/s^2)
r1 = 0.25;
r2 = 0.25;
lx = 0.0196;   % distance from the center of mass to each motor (m)
ly = 0.0196;
lz = 0.0264;
c = 0.1;


% State-space representation
A = [0 0 0 1 0 0 0 0 0 0 0 0;
     0 0 0 0 1 0 0 0 0 0 0 0;
     0 0 0 0 0 1 0 0 0 0 0 0;
     0 0 0 0 0 0 0 -g 0 0 0 0;
     0 0 0 0 0 0 g 0 0 0 0 0;
     0 0 0 0 0 0 0 0 0 0 0 0;
     0 0 0 0 0 0 0 0 0 1 0 0;
     0 0 0 0 0 0 0 0 0 0 1 0;
     0 0 0 0 0 0 0 0 0 0 0 1;
     0 0 0 0 0 0 0 0 0 0 0 0;
     0 0 0 0 0 0 0 0 0 0 0 0;
     0 0 0 0 0 0 0 0 0 0 0 0;];

B = [0 0 0 0 0 ;
     0 0 0 0 0 ;
     0 0 0 0 0 ;
     0 0 0 0 1/m;
     0 0 0 0 0 ;
     1/m 1/m 1/m 1/m 0 ;
     0 0 0 0 0 ;
     0 0 0 0 0 ;
     0 0 0 0 0 ;
     r1/lx r1/lx -r1/lx -r1/lx 0 ;
     r1/ly -r1/ly -r1/ly r1/ly 0 ;
     r2/lz -r2/lz r2/lz -r2/lz 0 ;];
         

C = [1 0 0 0 0 0 0 0 0 0 0 0;
     0 0 1 0 0 0 0 0 0 0 0 0;
     0 0 0 0 1 0 0 0 0 0 0 0;
     0 0 0 0 0 0 1 0 0 0 0 0;
     0 0 0 0 0 0 0 0 1 0 0 0;];
D = zeros(5,5);


% Create state-space system
sys = ss(A, B, C, D);

% LQR design
Q = diag([1, 1, 1, 0, 0, 0, 1, 1, 1, 0, 0, 0]);   % state weights
R = diag([1, 1, 1, 1, 1]);
K = lqr(sys, Q, R);

% Simulation
tspan = 0:0.01:5;        % simulation time span
x0 = [0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0];       % initial state
xv1 = [0; 15; 12; 0; 0; 0; 0; 0; 0; 0; 0; 0];       % Desired state
xv2 = [10; 15; 15; 0; 0; 0; 0; 0; 0; 0; 0; 0];       % Desired state
xv3 = [15; 12; 15; 0; 0; 0; 0; 0; 0; 0; 0; 0];       % Desired state
xv4 = [15; 3; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0];       % Desired state
u1 = zeros(5, length(tspan));
x1 = zeros(12, length(tspan));
u2 = zeros(5, length(tspan));
x2 = zeros(12, length(tspan));
u3 = zeros(5, length(tspan));
x3 = zeros(12, length(tspan));
u4 = zeros(5, length(tspan));
x4 = zeros(12, length(tspan));

% Apply LQR control
for k = 1:length(tspan)-1
    u1(:, k) = -K * x0 + K * xv1;
    xdot = A * x0 + B * u1(:, k);
    x0 = x0 + 0.01 * xdot; % 0.01 is the time step
    x1(:, k+1) = x0;
end
for k = 1:length(tspan)-1
    u2(:, k) = -K * xv1 + K * xv2;
    xdot = A * xv1 + B * u2(:, k);
    xv1 = xv1 + 0.01 * xdot; % 0.01 is the time step
    x2(:, k+1) = xv1;
end
for k = 1:length(tspan)-1
    u3(:, k) = -K * xv2 + K * xv3;
    xdot = A * xv2 + B * u3(:, k);
    xv2 = xv2 + 0.01 * xdot; % 0.01 is the time step
    x3(:, k+1) = xv2;
end
for k = 1:length(tspan)-1
    u4(:, k) = -K * xv3 + K * xv4;
    xdot = A * xv3 + B * u4(:, k);
    xv3 = xv3 + 0.01 * xdot; % 0.01 is the time step
    x4(:, k+1) = xv3;
end
tspan = .01:0.01:20;        % simulation time span
length(tspan)
length(x1)

x1(:,1) = [];
x2(:,1) = [];
x3(:,1) = [];
x4(:,1) = [];
u1(:,1) = [];
u2(:,1) = [];
u3(:,1) = [];
u4(:,1) = [];
length(x1)


% Plot results
figure;
subplot(2, 1, 1);
plot(tspan, [x1,x2,x3,x4], 'LineWidth',2);

title('Quadcopter State Variables');
legend('x', 'y', 'z', 'Xvel', 'Yvel', 'Zvel', 'roll', 'pitch', 'Yaw', 'Rrate', 'Prate', 'Yrate');
xlabel('Time (s)');
ylabel('Angle (rad)');

subplot(2, 1, 2);
plot(tspan, [u1,u2,u3,u4]);

title('Control Input');
legend('F1', 'F2', 'F3', 'F4', 'F5');
xlabel('Time (s)');
ylabel('Force (N)');
writematrix([[x1,x2,x3,x4]; [u1,u2,u3,u4]; tspan]', 'combined_matrix_VTOL5m.csv');

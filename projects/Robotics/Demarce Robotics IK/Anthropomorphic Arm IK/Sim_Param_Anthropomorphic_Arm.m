
% Floor Parameters
floor_w = 1.5;
floor_l = 1.5;
floor_h = 0.01;

%Joint Parameters
joint_r = 0.03;
joint_h = 0.05;

%Link 1 Prameters
link1_r = 0.02;
link1_l = 0.70;

side_l = 0.06;

%Link 2 Prameters
link2_r = 0.02;
link2_l = 0.50;


%Link 3 Prameters
link3_r = 0.02;
link3_l = 0.50;

% Number of points
N = 201;

% Time vector (column)
t = linspace(0, 5, N)';  % 5 seconds simulation (adjust as needed)

%% Define start and end points for the slanted line
start_point = [0.9, -0.5, 1];       % Starting point (x1, y1, z1)
end_point = [0.2, 0.4,0]; % Ending point (x2, y2, z2)

% Interpolate between start and end points
x = linspace(start_point(1), end_point(1), N)';  % x from 0 → 0.5
y = linspace(start_point(2), end_point(2), N)';  % y from 0 → 0.3
z = linspace(start_point(3), end_point(3), N)';  % z from 0 → 0.4

% --- Combine into workspace matrices for From Workspace blocks ---
X_block_data = [t, x];   % 201 x 2
Y_block_data = [t, y];   % 201 x 2
Z_block_data = [t, z];   % 201 x 2

%% --- Spline points for Mechanical Explorer for Straight line ---
% Spline expects N x 3 matrix: [x, y, z]
SplinePoints = [x, y, z];    % 201 x 3


%% ============================================================
%     q₁ Sweep Analysis & Spline Path
%  ------------------------------------------------------------
%  Demonstrates how varying the base joint (q₁) affects the
%  end-effector (x, y) position while q₂ = q₃ = 45° are fixed.
% ============================================================


%% === Fixed Joint Angles ===
q2 = deg2rad(45);     % 45 degrees
q3 = deg2rad(45);     % 45 degrees

%% === Sweep q₁ ===
N = 100;                              % Number of points
q1 = linspace(0, pi*2, N)';           % Sweep q₁ from 0 → 90°

% Preallocate position arrays
x = zeros(N,1);
y = zeros(N,1);
z = zeros(N,1);  % planar, z = 0

%% === Compute End-Effector Position via FK ===
for i = 1:N
    theta1 = q1(i);
    theta2 = q2;
    theta3 = q3;

     % Call the fk_anthrop_arm function to get the joint positions
    [~, ~, joint_positions] = fk_anthrop_arm([theta1, theta2, theta3]);
    
    % Extract the last joint position (End-Effector position)
    x(i) = joint_positions(1);  % End-effector x position (last joint)
    y(i) = joint_positions(2);  % End-effector y position
    z(i) = joint_positions(3);  % End-effector z position
end


%% === Create Time Vector for Simulink Input ===
T = 10;                            % total time (s)
t = linspace(0, T, N)';            % simulation time vector
q1_input = [t, q1];                % time-angle matrix

% Constant joint inputs for Simscape
q2_input = [t, ones(N,1)*q2];
q3_input = [t, ones(N,1)*q3];


%% === Create Spline Points for Mechanical Explorer ===
splinePoints2 = [x, y, z];  % N×3 matrix

%% === Plot End-Effector Position vs q1 ===
figure;

% Plot the X position vs q1
subplot(3,1,1);  % Create 3 subplots
plot(rad2deg(q1), x, 'r-', 'LineWidth', 1.5);  % Plot X position vs q1
xlabel('q₁ (deg)');
ylabel('X Position (m)');
title('End-Effector X Position vs q₁');
grid on;

% Plot the Y position vs q1
subplot(3,1,2);
plot(rad2deg(q1), y, 'b-', 'LineWidth', 1.5);  % Plot Y position vs q1
xlabel('q₁ (deg)');
ylabel('Y Position (m)');
title('End-Effector Y Position vs q₁');
grid on;

% Plot the Z position vs q1
subplot(3,1,3);
plot(rad2deg(q1), z, 'g-', 'LineWidth', 1.5);  % Plot Z position vs q1
xlabel('q₁ (deg)');
ylabel('Z Position (m)');
title('End-Effector Z Position vs q₁');
grid on;

%% === 3D Plot: End-Effector Position (x, y, z) vs q₁ ===
figure;
plot3(x, y, z, 'LineWidth', 2);  % Plot 3D end-effector trajectory
xlabel('X Position (m)');
ylabel('Y Position (m)');
zlabel('Z Position (m)');
title('3D End-Effector Position vs q₁');
grid on;
view(45, 25);  % Nice 3D viewing angle
colormap turbo;  % Optional color map
set(gca, 'FontSize', 12);  % Adjust font size

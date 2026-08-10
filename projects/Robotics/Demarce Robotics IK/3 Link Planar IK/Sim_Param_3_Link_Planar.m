% Floor Parameters
floor_w = 1.5;
floor_l = 1.5;
floor_h = 0.01;

%Joint Parameters
joint_r = 0.02;
joint_h = 0.01;

%Link 1 Prameters
link1_w = 0.04;
link1_l = 0.40;
link1_h = 0.01;

%Link 2 Prameters
link2_w = 0.04;
link2_l = 0.20;
link2_h = 0.01;

%Link 3 Prameters
link3_w = 0.04;
link3_l = 0.10;
link3_h = 0.01;

% Number of points
N = 201;

% Time vector (column)
t = linspace(0, 2, N)';  % 2 seconds simulation

%% Straight line along x-axis in World Coordinate Frame

N = 201;                 % Number of points
t = linspace(0, 5, N)';  % Time vector (0 → 5 seconds)
x = linspace(0, 0.7, N)'; % x from 0 → 0.7 m
y = linspace(0.5, 0, N)'; % y from 0.5 → 0 (downward slope)
z = zeros(N,1);           % z = 0 (planar)

% --- Combine into workspace matrices for From Workspace blocks ---
X_block_data = [t, x];   % 201 x 2
Y_block_data = [t, y];   % 201 x 2
Z_block_data = [t, z];   % 201 x 2


%% --- Spline points for Mechanical Explorer for Straight line ---
% Spline expects N x 3 matrix: [x, y, z]
SplinePoints = [x, y, z];    % 201 x 3

phi = pi;

config = 1;

%% ============================================================
%  3-Link Planar Manipulator: q₁ Sweep Analysis & Spline Path
%  ------------------------------------------------------------
%  Demonstrates how varying the base joint (q₁) affects the
%  end-effector (x, y) position while q₂ = q₃ = 45° are fixed.
%  Includes 3D plot of (x, y, q₁).
% ============================================================


%% === Robot Parameters ===
L1 = 0.4;    % Link 1 length (m)
L2 = 0.2;    % Link 2 length (m)
L3 = 0.1;    % Link 3 length (m)

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

    % Cumulative joint angles
    phi1 = theta1;
    phi2 = theta1 + theta2;
    phi3 = theta1 + theta2 + theta3;

    % Forward kinematics (planar)
    x(i) = L1*cos(phi1) + L2*cos(phi2) + L3*cos(phi3);
    y(i) = L1*sin(phi1) + L2*sin(phi2) + L3*sin(phi3);
    z(i) = 0;
end

%% === Create Time Vector for Simulink Input ===
T = 10;                            % total time (s)
t = linspace(0, T, N)';            % simulation time vector
q1_input = [t, q1];                % time-angle matrix

% Constant joint inputs for Simscape
q2_input = [t, ones(N,1)*q2];
q3_input = [t, ones(N,1)*q3];

%% === Export to Workspace for Simscape ===
assignin('base', 'q1_input', q1_input);
assignin('base', 'q2_input', q2_input);
assignin('base', 'q3_input', q3_input);

%% === Create Spline Points for Mechanical Explorer ===
splinePoints2 = [x, y, z];  % N×3 matrix
assignin('base', 'splinePoints', splinePoints);

%% === Plot End-Effector Trajectory (XY Plane) ===
figure;
plot(x, y, 'b-', 'LineWidth', 2); hold on;
plot(x(1), y(1), 'go', 'MarkerFaceColor','g'); % start
plot(x(end), y(end), 'ro', 'MarkerFaceColor','r'); % end
xlabel('X (m)');
ylabel('Y (m)');
title('End-Effector Trajectory for q₂ = q₃ = 45° (Varying q₁)');
legend('Trajectory','Start','End');
axis equal; grid on;

%% === Plot q₁ vs End-Effector Coordinates ===
figure;
subplot(2,1,1);
plot(rad2deg(q1), x, 'r', 'LineWidth', 1.5);
xlabel('q₁ (deg)');
ylabel('X Position (m)');
title('X Position vs q₁ (q₂=q₃=45°)');
grid on;

subplot(2,1,2);
plot(rad2deg(q1), y, 'b', 'LineWidth', 1.5);
xlabel('q₁ (deg)');
ylabel('Y Position (m)');
title('Y Position vs q₁ (q₂=q₃=45°)');
grid on;

%% === 3D Visualization: q₁ vs X vs Y ===
figure;
plot3(x, y, rad2deg(q1), 'LineWidth', 2);
xlabel('X Position (m)');
ylabel('Y Position (m)');
zlabel('q₁ (deg)');
title('3D Relationship: End-Effector Position vs q₁ (q₂=q₃=45°)');
grid on;
view(45,25); % nice 3D viewing angle
set(gca,'FontSize',10);
colormap turbo;

%% === Display Final Info ===
fprintf('\n✅ Data exported to base workspace:\n');
fprintf(' - q1_input, q2_input, q3_input (for From Workspace blocks)\n');
fprintf(' - splinePoints (for Mechanical Explorer Spline block)\n\n');
fprintf('3D visualization and spline path generated successfully.\n');

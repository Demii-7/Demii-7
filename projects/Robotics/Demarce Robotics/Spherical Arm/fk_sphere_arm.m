function [P_end] = fk_sphere_arm(q)
% fk_sphere_arm Computes forward kinematics for a spherical arm robot
% using the general DH matrix A
% Inputs:
%   q - [theta1, theta2, d3] joint angles (radians)

% Outputs:
%   P_end - 3x1 end-effector position

    % DH parameters
    a1 = 0; a2 = 0; a3 = 0;
    alpha1 = -pi/2; alpha2 = pi/2; alpha3 = 0;
    d1 = 5; d2 =2.5; d3 = q(3);
    theta3 = 0;

    % --- Denavit-Hartenberg matrices (planar case: alpha=0, d=0) ---
    A1 = dh(a1, alpha1, d1, q(1));
    A2 = dh(a2, alpha2, d2, q(2));
    A3 = dh(a3, alpha3, d3, theta3);
   
    % Compute cumulative transformations
    T03 = A1*A2*A3; % Base to end-effector
   
    % Extract position
    P3 = T03(1:3,4);
   
    % Outputs
    P_end = P3;
end

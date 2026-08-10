function [P1, P2, P3] = fk_anthrop_arm(q)
% fk_anthrop_arm Computes forward kinematics for an anthropomorphic arm
% using the general DH matrix A
% Inputs:
%   q - [theta1, theta2, d3] joint angles (radians)

% Outputs:
%   P_end - 3x1 end-effector position

    % DH parameters
    a1 = 0.7; a2 = 0.5; a3 = 0.5;
    alpha1 = pi/2; alpha2 = 0; alpha3 = 0;
    d1 = 0; d2 = 0; d3 = 0;

    % --- Denavit-Hartenberg matrices (planar case: alpha=0, d=0) ---
    A1 = dh(0, alpha1, a1, q(1));
    A2 = dh(a2, alpha2, d2, q(2));
    A3 = dh(a3, alpha3, d3, q(3));
   
     % Compute cumulative transformations
    T01 = A1;
    T02 = A1 * A2;
    T03 = A1*A2*A3; % Base to end-effector
   
    % Extract position
    P1 = T01(1:3,4);
    P2 = T02(1:3,4);
    P3 = T03(1:3,4); 
end

function [q1, q2, q3] = ik_3link(x, y, z, phi, config)
% IK_3LINK Computes inverse kinematics for a 3-link planar robot
%
% Inputs:
%   (x, y, z, phi) - desired end-effector pose (z ignored)
%   config - elbow configuration (0 = elbow-up, 1 = elbow-down)
%
% Outputs:
%   [q1, q2, q3] - joint angles (radians)

    % Link lengths (m)
    a1 = 0.4; a2 = 0.2; a3 = 0.1;

    % Wrist position
    pw = [x - a3*cos(phi); y - a3*sin(phi)];

    % Distance to wrist
    r = norm(pw);
    r_max = a1 + a2;
    r_min = abs(a1 - a2);

    % --- Reachability correction ---
    if r > r_max
        warning('Target beyond max reach. Projecting to reachable boundary.');
        pw = pw * (r_max / r);
        r = r_max;
    elseif r < r_min
        warning('Target within inner singular region. Projecting to inner boundary.');
        pw = pw * (r_min / r);
        r = r_min;
    end

    % --- Compute theta2 ---
    c2 = (r^2 - a1^2 - a2^2) / (2*a1*a2);
    c2 = max(min(c2, 1), -1);  % clamp to [-1, 1]

    if config == 0
        s2 = -sqrt(1 - c2^2);  % elbow-up
    elseif config == 1
        s2 =  sqrt(1 - c2^2);  % elbow-down
    else
        error('Config must be 0 (elbow-up) or 1 (elbow-down)');
    end
    theta2 = atan2(s2, c2);

    % --- Compute theta1 ---
    s1 = ((a1 + a2*c2)*pw(2) - a2*s2*pw(1)) / r;
    c1 = ((a1 + a2*c2)*pw(1) + a2*s2*pw(2)) / r;
    theta1 = atan2(s1, c1);

    % --- Compute theta3 ---
    theta3 = phi - theta1 - theta2;

    % --- Output joint angles ---
    q1 = theta1;
    q2 = theta2;
    q3 = theta3;
end

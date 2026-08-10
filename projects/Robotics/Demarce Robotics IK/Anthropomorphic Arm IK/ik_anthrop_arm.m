function [q1, q2, q3] = ik_anthrop_arm(x, y, z, config)
% IK_ANTHROP_ARM Inverse kinematics for a 3R anthropomorphic arm
% Inputs:
%   x, y, z  - desired end-effector position (meters)
%   config   - elbow config: 0 = elbow-up, 1 = elbow-down
% Outputs:
%   q1, q2, q3 - joint angles (radians)

    %% --- Link lengths ---
    a1 = 0.7;   % shoulder height (vertical offset)
    a2 = 0.5;   % upper arm
    a3 = 0.5;   % forearm

    %% --- Planar distance from shoulder ---
    rp = sqrt(x^2 + y^2);       % XY-plane distance
    dz = z - a1;                % vertical distance from shoulder

    %% --- Distance from shoulder to wrist ---
    d = sqrt(rp^2 + dz^2);

    %% --- Reachable distances for 2R planar arm ---
    d_max = a2 + a3;
    d_min = abs(a2 - a3);

    if d > d_max
        warning('Target beyond max reach. Clamping to boundary.');
        scale = d_max / d;
        rp = rp * scale;
        dz = dz * scale;
        d = d_max;
    elseif d < d_min
        warning('Target inside inner singular region. Clamping to boundary.');
        scale = d_min / d;
        rp = rp * scale;
        dz = dz * scale;
        d = d_min;
    end

    %% --- Law of cosines for elbow ---
    c3 = (d^2 - a2^2 - a3^2) / (2*a2*a3);
    c3 = max(min(c3,1),-1);     % numerical safety

    %% --- Elbow angle (choose configuration) ---
    if config == 0
        s3 = -sqrt(max(0, 1 - c3^2));  % elbow-up
    elseif config == 1
        s3 =  sqrt(max(0, 1 - c3^2));  % elbow-down
    else
        error('Config must be 0 (elbow-up) or 1 (elbow-down)');
    end
    theta3 = atan2(s3, c3);

    %% --- Shoulder pitch ---
    gamma = atan2(dz, rp);           % angle from shoulder to wrist
    beta  = atan2(a3*s3, a2 + a3*c3);% contribution from elbow geometry
    theta2 = gamma - beta;

    %% --- Base rotation ---
    theta1 = atan2(y, x);

    %% --- Return joint angles ---
    q1 = theta1;
    q2 = theta2;
    q3 = theta3;
end


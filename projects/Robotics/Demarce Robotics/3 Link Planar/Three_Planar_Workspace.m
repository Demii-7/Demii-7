
% Joint ranges (rad)
theta1_vals = linspace(-pi/3, pi/3, 30);
theta2_vals = linspace(-2*pi/3, 2*pi/3, 30);
theta3_vals = linspace(-pi/2, pi/2, 30);

% Preallocate
workspace = [];

for t1 = theta1_vals
    for t2 = theta2_vals
        for t3 = theta3_vals
            P = fk_3link([t1 t2 t3]);
            workspace = [workspace; P'];
        end
    end
end

figure;
plot3(workspace(:,1), workspace(:,2), workspace(:,3), '.');
grid on; axis equal;
xlabel('X'); ylabel('Y'); zlabel('Z');
title('3-Link Planar Manipulator Workspace');
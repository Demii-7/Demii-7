
% Joint ranges (rad)
theta1_vals = linspace(0, 2*pi, 30);
theta2_vals = linspace(-3*pi/2, 3*pi/2, 30);
d3 = linspace(2, 8, 30);

% Preallocate
workspace = [];

for t1 = theta1_vals
    for t2 = theta2_vals
        for t3 = d3
            P = fk_sphere_arm([t1 t2 t3]);
            workspace = [workspace; P'];
        end
    end
end

figure;
plot3(workspace(:,1), workspace(:,2), workspace(:,3), '.');
grid on; axis equal;
xlabel('X'); ylabel('Y'); zlabel('Z');
title('Spherical Arm Manipulator Workspace');
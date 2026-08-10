function A = dh(a, alpha, d, theta)
% === Homogeneous transformation matrix (DH standard) ===
% Accepts DH Parameters and outputs tranformation matrix between joints in and i+1

    A = [cos(theta) -sin(theta)*cos(alpha)  sin(theta)*sin(alpha)  a*cos(theta);
         sin(theta)  cos(theta)*cos(alpha) -cos(theta)*sin(alpha)  a*sin(theta);
         0           sin(alpha)              cos(alpha)             d;
         0           0                       0                      1];
end
function NikraveshG = computeNikraveshG(quaternion)
% computeNikraveshG(quaternion) returns a 3x4 matrix from quaternion
%   This is the Euler parameters transformation matrix.  
%   Based on equation 6.38 from Nikravesh book "Computer-Aided Analysis of
%   Machanical Systems." (1988)

% e0 = qw component, e1 = qx component, e2 = qy component, e3 = qz component
e0 = quaternion(1); e1 = quaternion(2); e2 = quaternion(3); e3 = quaternion(4);
NikraveshG = [ -e1 e0 -e3 e2; 
               -e2 e3 e0 -e1;
               -e3 -e2 e1 e0];
end


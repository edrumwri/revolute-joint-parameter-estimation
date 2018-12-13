function NikraveshG = computeNikraveshG(quat)
% computeNikraveshG(quat) returns a 3x4 matrix from quaternion vector quat
%   quat is a 4x1 quaternion in format qw qx qy qz
%
%   This is the Euler parameters transformation matrix.  
%   Based on equation 6.38 from Nikravesh book "Computer-Aided Analysis of
%   Machanical Systems." (1988)

% e0 = qw component, e1 = qx component, e2 = qy component, e3 = qz component
e0 = quat(1); e1 = quat(2); e2 = quat(3); e3 = quat(4);
NikraveshG = [ -e1 e0 -e3 e2; 
               -e2 e3 e0 -e1;
               -e3 -e2 e1 e0];
end


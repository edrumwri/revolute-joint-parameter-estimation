function NikraveshG = computeNikraveshG(quaternion)
% computeNikraveshG(quaternion) returns a 3x4 matrix from quaternion
%   This is the Euler parameters transformation matrix.  
%   Based on equation 6.38 from Nikravesh book "Computer-Aided Analysis of
%   Machanical Systems." (1988)

% e0 = w component, e1 = x component, e2 = y component, e3 = z component
e0 = quaternion(1); e1= quaternion(2); e2 = quaternion(3); e3 = quaternion(4);
NikraveshG = [ -e1 e0 -e3 e2; 
                -e2 e3 e0 -e1;
                -e3 -e2 e1 e0];
end


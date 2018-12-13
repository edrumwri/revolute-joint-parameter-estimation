function NikraveshG = computeNikraveshG(quat)
% computeNikraveshG(quat) returns a 3x4 matrix from quaternion vector quat
%   quat is a 4x1 quaternion in format qw qx qy qz
%
%   This is the Euler parameters transformation matrix.  
%   Based on equation 6.38 from Nikravesh book "Computer-Aided Analysis of
%   Mechanical Systems." (1988)

qw = quat(1); qx = quat(2); qy = quat(3); qz = quat(4);
NikraveshG = [ -qx  qw -qz  qy; 
               -qy  qz  qw -qx;
               -qz -qy  qx  qw];
end


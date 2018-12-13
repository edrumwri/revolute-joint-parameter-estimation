function N = computeN(quat)
% computeN(quat) returns matrix N used in calculation of the jacobian.
%   quat is a 4 elements vector quaternion in format qw qx qy qz
%
%   Returns 7x6 matrix [  eye(3)        zeros(3); 
%                       zeros(4, 3)   0.5 * NikraveshG']

    NG = 0.5 * computeNikraveshG(quat);
    N = [eye(3) zeros(3); zeros(4, 3) NG'];
end


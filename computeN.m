function NSpherical = computeN(quat)
% computeN(quat) returns matrix N used in calculation of the jacobian.
%
%   Returns 7x6 matrix [eye(3)     zeros(3); 
%                       zeros(4,3) NikraveshG']

    NG = 1/2 * computeNikraveshG(quat);
    NSpherical = [eye(3) zeros(3); zeros(4,3) NG'];
end


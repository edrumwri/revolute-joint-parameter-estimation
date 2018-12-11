function Jspherical = computeAnalyticalJacobianSpherical(u, quat)
% computeAnalyticalJacobianSpherical(u, quat) computes the jacobian for 
% a sperical joint using analytical derivation.
%   Returns a 3x6 matrix Jspherical = [eye(3) -skew(Ru)];
%   u is a unit vector defined in the body frame (constant).
    
    wRb = quat2R(quat);
    skew_Ru = getSkewSymmetricMatrix(wRb*u);
    Jspherical = [eye(3) -skew_Ru];
end


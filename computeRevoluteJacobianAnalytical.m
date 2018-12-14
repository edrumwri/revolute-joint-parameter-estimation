function JRev = computeRevoluteJacobianAnalytical(ui, vi, vj, quat)
% computeRevoluteJacobianAnalytica analytical computation of the jacobian
% for the revolute joint.
%   JRev = [eye(3)      -skew(wRi * ui); % Spherical Jacobian
%           zeros(1,3)   v1i' * wRi' * skew(vj);
%           zeros(1,3)   v2i' * wRi' * skew(vj)]
%
%   where wRi = qt2rot(quat) and vectors vi1 and vi2 form the orthogonal 
%   triad vi, v1i and v2i

 JSpherical = computeSphericalJacobianAnalytical(ui, quat);
 
 wRi = qt2rot(quat);
 [v1i, v2i] = computeBasisFromAxis(vi);
 skew_vj = getSkewSymmetricMatrix(vj);
 
 JRev = [JSpherical;
         zeros(1,3) v1i' * wRi' * skew_vj;
         zeros(1,3) v2i' * wRi' * skew_vj];
end


function JRev = computeRevoluteJacobian(ui, vi, vj, quat)
% computeRevoluteJacobian analytical computation of the jacobian
% for the revolute joint.
%   The revolute constraint is f = vjw' * vxw, where vxw = wRi * vxi
%   \dot{f} = vjw' *  (\dot{wRi} * vxi)  
%           = vjw' * w x (wRi * vxi)
%           = vjw' * -(wRi * vxi) x w
%           = vjw' * -skew(wRi * vxi) * w
%   The revolute constraint derivation for vxi stands for either v1i or v2i.

%   JRev = [eye(3)     -skew(wRi * ui); ...        % Spherical Jacobian
%           zeros(1,3) vjw' * -skew(wRi * v1i); ...
%           zeros(1,3) vjw' * -skew(wRi * v2i)]
%
%   where wRi = qt2rot(quat) and vectors vi1 and vi2 form the orthogonal 
%   triad vi, v1i and v2i
%
%   Since there are no linear velocity components in the \dot{f} the first 3 terms are 0
%   and the rest are the angular velocity terms vjw' * -skew(wRi * vxi)
%   for the revolute constraints in the resulting jacobian. 

JSpherical = computeSphericalJacobian(ui, quat);
 
 wRi = qt2rot(quat);
 [v1i, v2i] = computeBasisFromAxis(vi);
 skew_v1w = getSkewSymmetricMatrix(wRi * v1i );
 skew_v2w = getSkewSymmetricMatrix(wRi * v2i);
 
 JRev = [JSpherical; ...
         zeros(1,3) vj' * -skew_v1w;...
         zeros(1,3) vj' * -skew_v2w];
end


function JRevDotV = computeRevoluteJacobianDotVAnalytical(ui, vi, vj, quat, quatDot, velocity)
% computeRevoluteJacobianDotVAnalytical analytical computation of the
%   \dot{jacobian} * v for the revolute joint.
%   The jacobian is = [eye(3)      -skew(wRi * ui); 
%                      zeros(1,3)   v1i' * wRi' * skew(vj);
%                      zeros(1,3)   v2i' * wRi' * skew(vj)]
%
%   where wRi = qt2rot(quat) and vectors vi1 and vi2 form the orthogonal 
%   triad vi, v1i and v2i.

  JSphericalDotV = computeSphericalJacobianDotV(ui, quat, quatDot, velocity);
  
  [v1i, v2i] = computeBasisFromAxis(vi);   
  
  JRevConstDot1 = computeRevConstrDot(v1i, vj, quat, quatDot);
  JRevConstDot2 = computeRevConstrDot(v2i, vj, quat, quatDot);
    
  JRevDotConstr = [JRevConstDot1; JRevConstDot2];
  JRevDotConstrV = JRevDotConstr * velocity; 
  
  JRevDotV = [JSphericalDotV; JRevDotConstrV];
  
end

function JRevConstrDot = computeRevConstrDot(vxi, vj, quat, quatDot)
% Uses the analytical formulation to compute the dot constraint.

  qw = quat(1); qx = quat(2); qy = quat(3); qz = quat(4); 
  qwDot = quatDot(1); qxDot = quatDot(2); qyDot = quatDot(3); qzDot = quatDot(4);
  JRevConstrDot = [0, 0, 0, ...
 -2*vj(3)*((qxDot*qy + qx*qyDot + qwDot*qz + qw*qzDot)*vxi(1) + 2*(qw*qwDot + qy*qyDot)*vxi(2) + (-(qwDot*qx) - qw*qxDot + qyDot*qz + qy*qzDot)*vxi(3)) + 2*vj(2)*((-(qwDot*qy) - qw*qyDot + qxDot*qz + qx*qzDot)*vxi(1) + (qwDot*qx + qw*qxDot + qyDot*qz + qy*qzDot)*vxi(2) + 2*(qw*qwDot + qz*qzDot)*vxi(3)), ...
  2*vj(3)*(2*(qw*qwDot + qx*qxDot)*vxi(1) + (qxDot*qy + qx*qyDot - qwDot*qz - qw*qzDot)*vxi(2) + (qwDot*qy + qw*qyDot + qxDot*qz + qx*qzDot)*vxi(3)) - 2*vj(1)*((-(qwDot*qy) - qw*qyDot + qxDot*qz + qx*qzDot)*vxi(1) + (qwDot*qx + qw*qxDot + qyDot*qz + qy*qzDot)*vxi(2) + 2*(qw*qwDot + qz*qzDot)*vxi(3)), ...
 -2*vj(2)*(2*(qw*qwDot + qx*qxDot)*vxi(1) + (qxDot*qy + qx*qyDot - qwDot*qz - qw*qzDot)*vxi(2) + (qwDot*qy + qw*qyDot + qxDot*qz + qx*qzDot)*vxi(3)) + 2*vj(1)*((qxDot*qy + qx*qyDot + qwDot*qz + qw*qzDot)*vxi(1) + 2*(qw*qwDot + qy*qyDot)*vxi(2) + (-(qwDot*qx) - qw*qxDot + qyDot*qz + qy*qzDot)*vxi(3))];

end

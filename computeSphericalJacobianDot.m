function JSphericalDot = computeSphericalJacobianDot(ui, quat, quatDot)
% computeSphericalJacobianDot analytical computation of the dot{jacobian}
% for the spherical joint.
%   The jacobian is = [eye(3) -skew(R * ui)], where R = qt2rot(quat)

 qw = quat(1); qx = quat(2); qy = quat(3); qz = quat(4); 
 qwDot = quatDot(1); qxDot = quatDot(2); qyDot = quatDot(3); qzDot = quatDot(4); 

JSphericalDot = [0, 0, 0, 0, 2*((-(qwDot*qy) - qw*qyDot + qxDot*qz + qx*qzDot)*ui(1) + (qwDot*qx + qw*qxDot + qyDot*qz + qy*qzDot)*ui(2) + 2*(qw*qwDot + qz*qzDot)*ui(3)), -2*(qxDot*qy + qx*qyDot + qwDot*qz + qw*qzDot)*ui(1) - 4*(qw*qwDot + qy*qyDot)*ui(2) + 2*(qwDot*qx + qw*qxDot - qyDot*qz - qy*qzDot)*ui(3); 
                 0, 0, 0, 2*(qwDot*qy + qw*qyDot - qxDot*qz - qx*qzDot)*ui(1) - 2*(qwDot*qx + qw*qxDot + qyDot*qz + qy*qzDot)*ui(2) - 4*(qw*qwDot + qz*qzDot)*ui(3), 0, 2*(2*(qw*qwDot + qx*qxDot)*ui(1) + (qxDot*qy + qx*qyDot - qwDot*qz - qw*qzDot)*ui(2) + (qwDot*qy + qw*qyDot + qxDot*qz + qx*qzDot)*ui(3)); 
                 0, 0, 0, 2*((qxDot*qy + qx*qyDot + qwDot*qz + qw*qzDot)*ui(1) + 2*(qw*qwDot + qy*qyDot)*ui(2) + (-(qwDot*qx) - qw*qxDot + qyDot*qz + qy*qzDot)*ui(3)), -4*(qw*qwDot + qx*qxDot)*ui(1) + 2*(-(qxDot*qy) - qx*qyDot + qwDot*qz + qw*qzDot)*ui(2) - 2*(qwDot*qy + qw*qyDot + qxDot*qz + qx*qzDot)*ui(3), 0];
end


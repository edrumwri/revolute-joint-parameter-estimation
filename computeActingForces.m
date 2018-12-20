function [F, M] = computeActingForces(mass, J, quat, w)
% computeActingForces given the mass, inertia tensor J, orientation q,
%   and angular velocity w returns the forces acting on a
%   revolute joint: mass matrix M and forces F.
 
 g = 9.8;
 R = qt2rot(quat);
 RJRt = R * J * R';
 
 F = [0; -mass * g; 0;  0; 0; 0] - [0; 0; 0; cross(w, RJRt * w)];
 M = [eye(3) * mass zeros(3); zeros(3)  RJRt];
end

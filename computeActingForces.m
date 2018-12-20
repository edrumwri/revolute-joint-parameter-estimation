function [F, M] = computeActingForces(mass, Ji, quat, wi)
% computeActingForces given the mass, inertia tensor Ji, orientation q,
%   and angular velocity wi returns the forces acting on a
%   revolute joint: mass matrix and generalized inertia matrix M and forces F.
 
 g = 9.8;
 wRi = qt2rot(quat);
 RJRt = wRi * Ji * wRi';
 
 F = [0; -mass * g; 0;  0; 0; 0] - [0; 0; 0; cross(wi, RJRt * wi)];
 M = [eye(3) * mass zeros(3); zeros(3)  RJRt];
end

function [F, M] = computeActingForcesAndInertia(mass, Ji, quat, angularVel)
% computeActingForcesAndInertia given the mass, inertia tensor Ji, orientation quat,
%   and angular velocity angularVel returns the forces acting on a
%   revolute joint: mass matrix and generalized inertia matrix M and forces F.
%   Inertia tensor Ji is the i frame (not fixed to the world)
 
 g = 9.8;
 wRi = qt2rot(quat);
 RJRt = wRi * Ji * wRi';
 
 F = [0; -mass * g; 0;  0; 0; 0] - [0; 0; 0; cross(angularVel, RJRt * angularVel)];
 M = [eye(3) * mass zeros(3); zeros(3)  RJRt];
end

function [F, M] = computeActingForcesAndGeneralizedInertia(mass, Jb, quat, angularVel)
% computeActingForcesAndGeneralizedInertia returns generalized inertia matrix M 
%   and acting forces F for body b.
%
%   mass is the mass of body b.
%   Jb is the inertia tensor of the body b expressed in the body b frame. 
%   quat is the orientation of the body b relative to the global frame.
%   angularVel is the angular velocity of the body expressed in the global frame.
 
 g = 9.8;
 wRb = qt2rot(quat);
 RJRt = wRb * Jb * wRb';
 
 F = [0; -mass * g; 0;  0; 0; 0] - [0; 0; 0; cross(angularVel, RJRt * angularVel)];
 M = [eye(3) * mass zeros(3); zeros(3)  RJRt];
end

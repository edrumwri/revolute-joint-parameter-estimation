function revoluteConstraints = computeRevoluteJointConstraints(ui, vi, vj,  pose)
% computeRevoluteJointConstraints(ui, uj, quat) computes the revolute joint 
% constraints as spherical joint constraints with two aditional constraints.
%   ui is a 3x1 local position vector (on body i) defining the joint
%   location wrt the grasping point.
%   vi is a 3x1 vector for the revolute joint in the body i frame.
%   vj is a 3x1 vector for the revolute joint in the body j frame (fixed to the world).
%   y is a7x1 vector representing the position and orientation of the 
%   grasping point in the global frame.

    x = pose(1:3); % position components
    quat = pose(4:7); % quaternion components
    wRi = quat2R(quat);
    wRj = eye(3); % the 2 body (j) is attached to the world
    
    % Compute global vectors.
    viw = wRi * vi; 
    vjw = wRj * vj; 
    
    % Formulate the parallelism condition of the two vectors wvi and wvj as
    % two independent dot product equations.
    [v1iw, v2iw] = compute2OrtogonalVect(viw);
    revolute_parallel_vect = [v1iw' * vjw; v2iw' * vjw];
    
    sphericalConstraints = x + wRi * ui;
    
    revoluteConstraints = [sphericalConstraints; revolute_parallel_vect];
end

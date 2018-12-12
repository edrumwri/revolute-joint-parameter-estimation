function revoluteConstraints = computeRevoluteJointConstraints(ui, vi, vj, y)
% computeRevoluteJointConstraints(ui, uj, quat) computes the revolute joint 
% constraints as spherical joint constraints with two aditional constraints.
%   ui is a 3x1 constant vector in the frame of link i.
%   vi is a 3x1 vector for the revolute joint in the body i frame.
%   vj is a 3x1 vector for the revolute joint in the body j frame (fixed to the world).
%   y is the position and orientation 7x1 vector.

    x = y(1:3); % position components
    quat = y(4:7); % quaternion components
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

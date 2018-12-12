function sphericalConstraints = computeSphericalJointConstraints(ui, pose)
% computeSphericalJointConstraints(ui, pose) computes the spherical joint
% constraints f = x + R * u.
%   ui is a 3x1 local position vector (on body i) defining the joint
%   location wrt the grasping point.
%   pose is a7x1 vector representing the position and orientation of the 
%   grasping point in the global frame.

    x = pose(1:3); % position components
    quat = pose(4:7); % quaternion components
    wRi = quat2R(quat);
    sphericalConstraints = x + wRi * ui;
    
end


function sphericalConstraints = computeSphericalJointConstraints(ui, pose)
% computeSphericalJointConstraints(ui, pose) computes the spherical joint
% constraints defined as x + R * u = 0.
%   ui is a 3x1 vector representing local position vector on body i
%   pose is a 7x1 vector representing the:
%       position: values 1:3 in pose in the format ex ey ez and
%       orientation: values 4:7 in pose in a quaternion format qw qx qy qz

    x = pose(1:3); % position components
    quat = pose(4:7); % quaternion components
    wRi = quat2R(quat);
    sphericalConstraints = x + wRi * ui;
    
end


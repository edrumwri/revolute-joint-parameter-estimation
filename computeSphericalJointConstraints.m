function sphericalConstraints = computeSphericalJointConstraints(ui, y)
% computeSphericalJointConstraints(u, y) computes the spherical joint
% constraints.
%   ui is a constant vector in the frame of link i.
    x = y(1:3); % position components
    quat = y(4:7); % quaternion components
    wRi = quat2R(quat);
    sphericalConstraints = x + wRi * ui;
    
end


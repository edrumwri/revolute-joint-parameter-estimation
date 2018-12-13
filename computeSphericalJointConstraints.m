function [fSpherical, fDotSpherical] = computeSphericalJointConstraints(ui, pose, velocity)
% computeSphericalJointConstraints(ui, pose, velocity) computes:
%   fSpherical: the spherical joint constraints defined as x + R * u = 0.
%   fDotSpherical: as the first derivative of the f using the analytical derivation.
%
%   ui is a 3x1 vector from the center-of-mass of body i to the revolute 
%   pose is a 7x1 vector representing the:
%       position: values 1:3 in pose in the format ex ey ez and
%       orientation: values 4:7 in pose in a quaternion format qw qx qy qz
%   velocity is a 6x1 OPTIONAL vector containing:
%       velocity: values 1:3 in the format vx vy vz
%       angular velocity: values 4:6 in the format wx wy wz
%       If velocity is not passed the fDotSpherical is zeros(3,1);

    x = pose(1:3); % position components
    quat = pose(4:7); % quaternion components
    wRi = quat2R(quat);
    fSpherical = x + wRi * ui;
    
    if exist('velocity','var')
     % the velocity is passed as an argument.
     vel = velocity(1:3);
     angularVelTensor = getSkewSymmetricMatrix(velocity(4:6));
    
     % f = x + R * u, df/dt = dx/xt + dR/dt * u 
     % dx/dt = vel, dR/dt * u = angularVelTensor * R * u  
     % therfore df/dt = vel + R * u * angularVelTensor 
     fDotSpherical = vel + angularVelTensor * wRi * ui;
    else
        fDotSpherical = zeros(3,1);
    end
end

